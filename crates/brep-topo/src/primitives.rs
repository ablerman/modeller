//! Primitive BRep solid builders.
//!
//! Constructs topologically valid BRep solids directly (bypassing incremental
//! Euler operators), then sews adjacent faces to set up twin half-edge pointers.

use std::f64::consts::TAU;
use std::sync::Arc;

use brep_core::{EntityId, KernelError, Point3, Vec3};
use brep_geom::curve::{CircleCurve, LineCurve};
use brep_geom::surface::{ConicalSurface, CylindricalSurface, Plane, SphericalSurface};

use crate::binding::{CurveBinding, SurfaceBinding};
use crate::entity::{
    Edge, Face, HalfEdge, Loop, LoopKind, Orientation, SolidId, Vertex,
    EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, VertexId,
};
use crate::euler::make_solid_shell;
use crate::store::ShapeStore;

// ── Low-level helpers ─────────────────────────────────────────────────────────

/// Create a face with a closed polygon loop from `vertices` (in CCW order
/// when viewed from outside), leaving twin pointers unset (sentinel).
///
/// Returns (face_id, [half-edge ids in loop order]).
fn make_face_ring(
    store: &mut ShapeStore,
    shell_id: ShellId,
    vertices: &[VertexId],
    surface: SurfaceBinding,
    orientation: Orientation,
) -> Result<(FaceId, Vec<HalfEdgeId>), KernelError> {
    let n = vertices.len();
    assert!(n >= 3, "polygon must have ≥ 3 vertices");

    let sentinel_he: HalfEdgeId = EntityId::from_raw(0, 0);
    let sentinel_e:  EdgeId      = EntityId::from_raw(0, 0);
    let sentinel_l:  LoopId      = EntityId::from_raw(0, 0);

    // 1. Insert face with placeholder loop.
    let face_id = store.insert_face(Face {
        outer_loop: sentinel_l,
        inner_loops: smallvec::SmallVec::new(),
        surface,
        orientation,
        shell: shell_id,
        tolerance: store.tolerance.linear,
    });

    // 2. Insert loop.
    let loop_id = store.insert_loop(Loop {
        first_half_edge: sentinel_he,
        loop_kind: LoopKind::Outer,
        face: face_id,
    });
    store.face_mut(face_id)?.outer_loop = loop_id;

    // 3. Insert half-edges (next/prev filled in after all are created).
    let mut hes: Vec<HalfEdgeId> = Vec::with_capacity(n);
    for i in 0..n {
        let he_id = store.insert_half_edge(HalfEdge {
            origin: vertices[i],
            twin: sentinel_he,
            next: sentinel_he,
            prev: sentinel_he,
            loop_id,
            edge: sentinel_e,
            pcurve: None,
        });
        hes.push(he_id);
    }

    // 4. Wire next/prev.
    for i in 0..n {
        let next_i = (i + 1) % n;
        let prev_i = (i + n - 1) % n;
        store.half_edge_mut(hes[i])?.next = hes[next_i];
        store.half_edge_mut(hes[i])?.prev = hes[prev_i];
    }
    store.loop_mut(loop_id)?.first_half_edge = hes[0];

    // 5. Register face in shell. (Edges are created later in sew_half_edges.)
    store.shell_mut(shell_id)?.faces.push(face_id);

    Ok((face_id, hes))
}

/// Sewing pass: match half-edges by endpoint pair (vi→vj ↔ vj→vi).
///
/// For each half-edge (origin=A, heading towards B = origin of next), find
/// the half-edge in any other loop with (origin=B, heading towards A) and
/// set them as twins.
fn sew_half_edges(store: &mut ShapeStore) -> Result<(), KernelError> {
    use std::collections::HashMap;

    // Collect (origin_vertex_id, next_origin_vertex_id) for every he.
    let he_data: Vec<(HalfEdgeId, VertexId, HalfEdgeId)> = store
        .half_edges
        .iter()
        .map(|(k, he)| {
            let id: HalfEdgeId = crate::store::to_id_pub(k);
            let next_origin = he.next;
            (id, he.origin, next_origin)
        })
        .collect();

    // First pass: build map (origin, next_he) → he
    // We use vertex IDs as keys. We need the destination vertex id.
    // The destination of he is the origin of he.next.
    let mut by_endpoints: HashMap<(u64, u64), HalfEdgeId> = HashMap::new();
    for (he_id, origin_vid, next_he_id) in &he_data {
        if let Ok(next_he) = store.half_edge(*next_he_id) {
            let dest_vid = next_he.origin;
            let key = (
                (origin_vid.index() as u64) << 32 | (origin_vid.generation() as u64),
                (dest_vid.index() as u64) << 32 | (dest_vid.generation() as u64),
            );
            by_endpoints.insert(key, *he_id);
        }
    }

    // Second pass: for each unmatched he, find twin, create one shared Edge.
    let he_ids: Vec<HalfEdgeId> = store
        .half_edges
        .iter()
        .map(|(k, _)| crate::store::to_id_pub(k))
        .collect();

    let sentinel: HalfEdgeId = EntityId::from_raw(0, 0);
    for he_id in &he_ids {
        let he = store.half_edge(*he_id)?;
        if he.twin != sentinel {
            continue; // already paired
        }
        let origin_vid = he.origin;
        let next_he_id = he.next;
        let dest_vid = store.half_edge(next_he_id)?.origin;

        // Look for the reversed edge (dest→origin).
        let twin_key = (
            (dest_vid.index() as u64) << 32 | (dest_vid.generation() as u64),
            (origin_vid.index() as u64) << 32 | (origin_vid.generation() as u64),
        );
        if let Some(&twin_id) = by_endpoints.get(&twin_key) {
            if twin_id != *he_id {
                // Create one Edge for the pair; both half-edges point to it.
                let edge_id = store.insert_edge(Edge {
                    half_edges: [*he_id, twin_id],
                    curve: None,
                    tolerance: store.tolerance.linear,
                    is_degenerate: false,
                });
                store.half_edge_mut(*he_id)?.twin  = twin_id;
                store.half_edge_mut(twin_id)?.twin = *he_id;
                store.half_edge_mut(*he_id)?.edge  = edge_id;
                store.half_edge_mut(twin_id)?.edge = edge_id;
            }
        }
    }

    Ok(())
}

/// Attach a line curve to an edge.
fn attach_line_curve(
    store: &mut ShapeStore,
    edge_id: EdgeId,
    from: Point3,
    to: Point3,
) -> Result<(), KernelError> {
    let dir = to - from;
    if dir.norm() < 1e-15 {
        return Ok(());
    }
    let curve = Arc::new(LineCurve::new(from, dir));
    store.edge_mut(edge_id)?.curve = Some(CurveBinding::new(curve, 0.0, 1.0, true));
    Ok(())
}

// ── Box ───────────────────────────────────────────────────────────────────────

/// Build an axis-aligned box spanning `[0,dx] × [0,dy] × [0,dz]`.
pub fn make_box(store: &mut ShapeStore, dx: f64, dy: f64, dz: f64)
    -> Result<SolidId, KernelError>
{
    if dx <= 0.0 || dy <= 0.0 || dz <= 0.0 {
        return Err(KernelError::DegenerateGeometry(format!(
            "make_box: dimensions must be positive, got ({dx},{dy},{dz})"
        )));
    }
    let crate::euler::MakeSolidShellResult { solid_id, shell_id } = make_solid_shell(store);

    // 8 corner vertices.
    let vs = [
        store.insert_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(dx,  0.0, 0.0), store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(dx,  dy,  0.0), store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(0.0, dy,  0.0), store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(0.0, 0.0, dz),  store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(dx,  0.0, dz),  store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(dx,  dy,  dz),  store.tolerance.linear)),
        store.insert_vertex(Vertex::new(Point3::new(0.0, dy,  dz),  store.tolerance.linear)),
    ];

    // Helper: planar face from 4 vertex indices (CCW from outside).
    let add_face = |store: &mut ShapeStore,
                        vi: [usize; 4],
                        origin: Point3,
                        u: Vec3,
                        v: Vec3,
                        same: bool|
        -> Result<Vec<HalfEdgeId>, KernelError>
    {
        let surf = SurfaceBinding::new(Arc::new(Plane::new(origin, u, v)), same);
        let verts = [vs[vi[0]], vs[vi[1]], vs[vi[2]], vs[vi[3]]];
        let (_, hes) = make_face_ring(store, shell_id, &verts,
            surf, if same { Orientation::Same } else { Orientation::Reversed })?;
        Ok(hes)
    };

    // Six faces (vertex indices chosen so CCW from outside).
    add_face(store, [0, 3, 2, 1], Point3::origin(),          Vec3::y()*dy, Vec3::x()*dx, false)?; // -Z bottom
    add_face(store, [4, 5, 6, 7], Point3::new(0.,0.,dz),     Vec3::x()*dx, Vec3::y()*dy, true )?; // +Z top
    add_face(store, [0, 1, 5, 4], Point3::origin(),          Vec3::x()*dx, Vec3::z()*dz, false)?; // -Y front
    add_face(store, [3, 7, 6, 2], Point3::new(0.,dy,0.),     Vec3::z()*dz, Vec3::x()*dx, true )?; // +Y back
    add_face(store, [0, 4, 7, 3], Point3::origin(),          Vec3::z()*dz, Vec3::y()*dy, false)?; // -X left
    add_face(store, [1, 2, 6, 5], Point3::new(dx,0.,0.),     Vec3::y()*dy, Vec3::z()*dz, true )?; // +X right

    sew_half_edges(store)?;

    // Attach line curves to all edges.
    let edge_ids: Vec<EdgeId> = store.edges.iter()
        .map(|(k, _)| crate::store::to_id_pub(k))
        .collect();
    for eid in edge_ids {
        let [he0, _] = store.edge(eid)?.half_edges;
        let from = store.vertex(store.half_edge(he0)?.origin)?.position;
        let next_he = store.half_edge(he0)?.next;
        let to   = store.vertex(store.half_edge(next_he)?.origin)?.position;
        attach_line_curve(store, eid, from, to)?;
    }

    store.shell_mut(shell_id)?.is_closed = true;
    Ok(solid_id)
}

// ── Helpers for curved-surface primitives ────────────────────────────────────

/// Insert a face + outer loop in `shell_id`, returning both ids.
/// The loop's `first_half_edge` is left as the sentinel; the caller populates it.
fn insert_face_with_loop(
    store: &mut ShapeStore,
    shell_id: ShellId,
    surface: SurfaceBinding,
    orientation: Orientation,
) -> Result<(FaceId, LoopId), KernelError> {
    let sentinel_l: LoopId     = EntityId::from_raw(0, 0);
    let sentinel_he: HalfEdgeId = EntityId::from_raw(0, 0);

    let face_id = store.insert_face(Face {
        outer_loop: sentinel_l,
        inner_loops: smallvec::SmallVec::new(),
        surface,
        orientation,
        shell: shell_id,
        tolerance: store.tolerance.linear,
    });
    let loop_id = store.insert_loop(Loop {
        first_half_edge: sentinel_he,
        loop_kind: LoopKind::Outer,
        face: face_id,
    });
    store.face_mut(face_id)?.outer_loop = loop_id;
    store.shell_mut(shell_id)?.faces.push(face_id);
    Ok((face_id, loop_id))
}

/// Insert a half-edge with all sentinels, then return its id.
fn insert_he(store: &mut ShapeStore, origin: VertexId, loop_id: LoopId) -> HalfEdgeId {
    let s: HalfEdgeId = EntityId::from_raw(0, 0);
    let se: EdgeId    = EntityId::from_raw(0, 0);
    store.insert_half_edge(HalfEdge {
        origin,
        twin: s,
        next: s,
        prev: s,
        loop_id,
        edge: se,
        pcurve: None,
    })
}

/// Wire a cyclic sequence of half-edges: hes[i].next = hes[i+1], etc.
fn wire_loop(store: &mut ShapeStore, hes: &[HalfEdgeId]) -> Result<(), KernelError> {
    let n = hes.len();
    for i in 0..n {
        let next = hes[(i + 1) % n];
        let prev = hes[(i + n - 1) % n];
        store.half_edge_mut(hes[i])?.next = next;
        store.half_edge_mut(hes[i])?.prev = prev;
    }
    Ok(())
}

/// Create an `Edge` backed by `he_a` and `he_b`, wire their twin/edge fields.
fn make_edge_pair(
    store: &mut ShapeStore,
    he_a: HalfEdgeId,
    he_b: HalfEdgeId,
) -> Result<EdgeId, KernelError> {
    store.half_edge_mut(he_a)?.twin = he_b;
    store.half_edge_mut(he_b)?.twin = he_a;
    let eid = store.insert_edge(Edge {
        half_edges: [he_a, he_b],
        curve: None,
        tolerance: store.tolerance.linear,
        is_degenerate: false,
    });
    store.half_edge_mut(he_a)?.edge = eid;
    store.half_edge_mut(he_b)?.edge = eid;
    Ok(eid)
}

// ── Cylinder ──────────────────────────────────────────────────────────────────

/// Right circular cylinder centred on the Z-axis with bottom at z=0.
///
/// Topology (3 faces, 3 edges, 2 vertices, 6 half-edges):
///
/// ```text
/// v_b ── E_seam ── v_t
///  └── E_bot (circle) ──┘    (v_b→v_b)
///  └── E_top (circle) ──┘    (v_t→v_t, via lateral face)
///
/// Lateral loop: he_bot_fwd → he_seam_fwd → he_top_rev → he_seam_rev → ...
/// Bot loop:     he_bot_rev (self-loop at v_b)
/// Top loop:     he_top_fwd (self-loop at v_t)
/// ```
pub fn make_cylinder(store: &mut ShapeStore, radius: f64, height: f64)
    -> Result<SolidId, KernelError>
{
    if radius <= 0.0 || height <= 0.0 {
        return Err(KernelError::DegenerateGeometry(
            "make_cylinder: radius and height must be positive".into(),
        ));
    }
    let crate::euler::MakeSolidShellResult { solid_id, shell_id } = make_solid_shell(store);

    let v_b = store.insert_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0),    store.tolerance.linear));
    let v_t = store.insert_vertex(Vertex::new(Point3::new(radius, 0.0, height), store.tolerance.linear));

    // Three faces.
    let lat_surf = SurfaceBinding::new(
        Arc::new(CylindricalSurface::new(Point3::origin(), Vec3::z(), radius)), true);
    let (_, lat_loop) = insert_face_with_loop(store, shell_id, lat_surf, Orientation::Same)?;

    let bot_surf = SurfaceBinding::new(Arc::new(Plane::new(Point3::origin(), Vec3::x(), Vec3::y())), false);
    let (_, bot_loop) = insert_face_with_loop(store, shell_id, bot_surf, Orientation::Reversed)?;

    let top_surf = SurfaceBinding::new(Arc::new(Plane::new(Point3::new(0.,0.,height), Vec3::x(), Vec3::y())), true);
    let (_, top_loop) = insert_face_with_loop(store, shell_id, top_surf, Orientation::Same)?;

    // Six half-edges.
    let he_bot_fwd  = insert_he(store, v_b, lat_loop); // E_bot fwd side  (in lateral loop)
    let he_seam_fwd = insert_he(store, v_b, lat_loop); // E_seam fwd      (in lateral loop)
    let he_top_rev  = insert_he(store, v_t, lat_loop); // E_top rev side  (in lateral loop)
    let he_seam_rev = insert_he(store, v_t, lat_loop); // E_seam rev      (in lateral loop)
    let he_bot_rev  = insert_he(store, v_b, bot_loop); // E_bot rev side  (in bot loop)
    let he_top_fwd  = insert_he(store, v_t, top_loop); // E_top fwd side  (in top loop)

    // Wire the lateral loop: bot_fwd → seam_fwd → top_rev → seam_rev → bot_fwd
    wire_loop(store, &[he_bot_fwd, he_seam_fwd, he_top_rev, he_seam_rev])?;
    store.loop_mut(lat_loop)?.first_half_edge = he_bot_fwd;

    // Bot / top loops are single-he self-loops.
    wire_loop(store, &[he_bot_rev])?;
    store.loop_mut(bot_loop)?.first_half_edge = he_bot_rev;

    wire_loop(store, &[he_top_fwd])?;
    store.loop_mut(top_loop)?.first_half_edge = he_top_fwd;

    // Three edges.
    let e_bot  = make_edge_pair(store, he_bot_fwd,  he_bot_rev)?;
    let e_seam = make_edge_pair(store, he_seam_fwd, he_seam_rev)?;
    let e_top  = make_edge_pair(store, he_top_fwd,  he_top_rev)?;

    // Geometry.
    attach_line_curve(store, e_seam, store.vertex(v_b)?.position, store.vertex(v_t)?.position)?;
    store.edge_mut(e_bot)?.curve = Some(CurveBinding::new(
        Arc::new(CircleCurve::new(Point3::origin(), radius, Vec3::x(), Vec3::y())),
        0.0, TAU, true,
    ));
    store.edge_mut(e_top)?.curve = Some(CurveBinding::new(
        Arc::new(CircleCurve::new(Point3::new(0.,0.,height), radius, Vec3::x(), Vec3::y())),
        0.0, TAU, true,
    ));

    store.shell_mut(shell_id)?.is_closed = true;
    Ok(solid_id)
}

// ── Sphere ────────────────────────────────────────────────────────────────────

/// Sphere centred at the origin.
///
/// Topology (1 face, 1 edge, 2 vertices, 2 half-edges):
/// A single spherical face whose boundary is just the seam edge from south to
/// north pole.  Both half-edges of the seam share the same loop.
pub fn make_sphere(store: &mut ShapeStore, radius: f64) -> Result<SolidId, KernelError> {
    if radius <= 0.0 {
        return Err(KernelError::DegenerateGeometry(
            "make_sphere: radius must be positive".into(),
        ));
    }
    let crate::euler::MakeSolidShellResult { solid_id, shell_id } = make_solid_shell(store);

    let v_s = store.insert_vertex(Vertex::new(Point3::new(0., 0., -radius), store.tolerance.linear));
    let v_n = store.insert_vertex(Vertex::new(Point3::new(0., 0.,  radius), store.tolerance.linear));

    let surf = SurfaceBinding::new(Arc::new(SphericalSurface::new(Point3::origin(), radius)), true);
    let (_, sph_loop) = insert_face_with_loop(store, shell_id, surf, Orientation::Same)?;

    // Two half-edges forming a 2-cycle in the single loop.
    let he_fwd = insert_he(store, v_s, sph_loop);
    let he_rev = insert_he(store, v_n, sph_loop);

    wire_loop(store, &[he_fwd, he_rev])?;
    store.loop_mut(sph_loop)?.first_half_edge = he_fwd;

    let e_seam = make_edge_pair(store, he_fwd, he_rev)?;
    attach_line_curve(store, e_seam, store.vertex(v_s)?.position, store.vertex(v_n)?.position)?;

    store.shell_mut(shell_id)?.is_closed = true;
    Ok(solid_id)
}

// ── Cone ─────────────────────────────────────────────────────────────────────

/// Right circular cone with apex at z=`height`, base radius `radius` at z=0.
///
/// Topology (2 faces, 2 edges, 2 vertices, 5 half-edges):
///
/// ```text
/// Lateral loop: he_seam_fwd → he_base_fwd → he_seam_rev → ...
///               (apex→v_b) → (v_b circle) → (v_b→apex)
/// Base loop:    he_base_rev (self-loop at v_b)
/// ```
pub fn make_cone(store: &mut ShapeStore, radius: f64, height: f64) -> Result<SolidId, KernelError> {
    if radius <= 0.0 || height <= 0.0 {
        return Err(KernelError::DegenerateGeometry(
            "make_cone: radius and height must be positive".into(),
        ));
    }
    let crate::euler::MakeSolidShellResult { solid_id, shell_id } = make_solid_shell(store);

    let v_apex = store.insert_vertex(Vertex::new(Point3::new(0., 0., height), store.tolerance.linear));
    let v_base = store.insert_vertex(Vertex::new(Point3::new(radius, 0., 0.), store.tolerance.linear));

    // Apex is at (0,0,height); the axis points DOWN toward the base (-Z).
    let half_angle = (radius / height).atan();
    let lat_surf = SurfaceBinding::new(
        Arc::new(ConicalSurface::new(Point3::new(0., 0., height), -Vec3::z(), half_angle, radius)),
        true,
    );
    let (_, lat_loop) = insert_face_with_loop(store, shell_id, lat_surf, Orientation::Same)?;

    let base_surf = SurfaceBinding::new(Arc::new(Plane::new(Point3::origin(), Vec3::x(), Vec3::y())), false);
    let (_, base_loop) = insert_face_with_loop(store, shell_id, base_surf, Orientation::Reversed)?;

    // Five half-edges.
    let he_seam_fwd = insert_he(store, v_apex, lat_loop); // apex→v_base (lateral)
    let he_base_fwd = insert_he(store, v_base, lat_loop); // circle (lateral)
    let he_seam_rev = insert_he(store, v_base, lat_loop); // v_base→apex (lateral)
    let he_base_rev = insert_he(store, v_base, base_loop); // circle reversed (base cap)

    // Wire lateral loop: seam_fwd → base_fwd → seam_rev → seam_fwd
    wire_loop(store, &[he_seam_fwd, he_base_fwd, he_seam_rev])?;
    store.loop_mut(lat_loop)?.first_half_edge = he_seam_fwd;

    wire_loop(store, &[he_base_rev])?;
    store.loop_mut(base_loop)?.first_half_edge = he_base_rev;

    let e_seam = make_edge_pair(store, he_seam_fwd, he_seam_rev)?;
    let e_base = make_edge_pair(store, he_base_fwd, he_base_rev)?;

    attach_line_curve(store, e_seam, store.vertex(v_apex)?.position, store.vertex(v_base)?.position)?;
    store.edge_mut(e_base)?.curve = Some(CurveBinding::new(
        Arc::new(CircleCurve::new(Point3::origin(), radius, Vec3::x(), Vec3::y())),
        0.0, TAU, true,
    ));

    store.shell_mut(shell_id)?.is_closed = true;
    Ok(solid_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::validate::ShapeValidator;

    fn valid(store: &ShapeStore) -> bool {
        let r = ShapeValidator::new(store).validate();
        if !r.is_valid() {
            for issue in &r.issues {
                eprintln!("  validation: {:?}", issue);
            }
        }
        r.is_valid()
    }

    #[test]
    fn box_has_correct_entity_counts() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        assert_eq!(store.vertex_count(), 8);
        assert_eq!(store.face_count(), 6);
        assert_eq!(store.solid_count(), 1);
        assert_eq!(store.edge_count(), 12, "box has 12 edges");
    }

    #[test]
    fn box_validator_passes() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 2.0, 3.0, 4.0).unwrap();
        assert!(valid(&store));
    }

    #[test]
    fn box_shell_is_closed() {
        let mut store = ShapeStore::new();
        let solid_id = make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let shell_id = store.solid(solid_id).unwrap().outer_shell;
        assert!(store.shell(shell_id).unwrap().is_closed);
    }

    #[test]
    fn box_zero_dimension_is_error() {
        let mut store = ShapeStore::new();
        assert!(make_box(&mut store, 0.0, 1.0, 1.0).is_err());
        assert!(make_box(&mut store, 1.0, -1.0, 1.0).is_err());
    }

    #[test]
    fn box_vertices_at_correct_positions() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 2.0, 3.0, 4.0).unwrap();
        for (_, v) in store.vertices.iter() {
            assert!(v.position.x == 0.0 || (v.position.x - 2.0).abs() < 1e-12);
            assert!(v.position.y == 0.0 || (v.position.y - 3.0).abs() < 1e-12);
            assert!(v.position.z == 0.0 || (v.position.z - 4.0).abs() < 1e-12);
        }
    }

    #[test]
    fn box_all_twins_set() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let sentinel: HalfEdgeId = EntityId::from_raw(0, 0);
        for (_, he) in store.half_edges.iter() {
            assert_ne!(he.twin, sentinel, "every half-edge must have a twin");
        }
    }

    #[test]
    fn cylinder_has_correct_entity_counts() {
        let mut store = ShapeStore::new();
        make_cylinder(&mut store, 1.0, 2.0).unwrap();
        assert_eq!(store.face_count(), 3);
        assert_eq!(store.solid_count(), 1);
    }

    #[test]
    fn cylinder_validator_passes() {
        let mut store = ShapeStore::new();
        make_cylinder(&mut store, 1.0, 5.0).unwrap();
        assert!(valid(&store));
    }

    #[test]
    fn cylinder_negative_radius_is_error() {
        let mut store = ShapeStore::new();
        assert!(make_cylinder(&mut store, -1.0, 2.0).is_err());
        assert!(make_cylinder(&mut store, 1.0, 0.0).is_err());
    }

    #[test]
    fn sphere_has_correct_entity_counts() {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        assert_eq!(store.face_count(), 1);
        assert_eq!(store.vertex_count(), 2);
    }

    #[test]
    fn sphere_validator_passes() {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 3.0).unwrap();
        assert!(valid(&store));
    }

    #[test]
    fn sphere_zero_radius_is_error() {
        let mut store = ShapeStore::new();
        assert!(make_sphere(&mut store, 0.0).is_err());
    }

    #[test]
    fn cone_has_correct_entity_counts() {
        let mut store = ShapeStore::new();
        make_cone(&mut store, 1.0, 2.0).unwrap();
        assert_eq!(store.face_count(), 2);
        assert_eq!(store.vertex_count(), 2);
    }

    #[test]
    fn cone_validator_passes() {
        let mut store = ShapeStore::new();
        make_cone(&mut store, 1.0, 2.0).unwrap();
        assert!(valid(&store));
    }

    #[test]
    fn cone_zero_height_is_error() {
        let mut store = ShapeStore::new();
        assert!(make_cone(&mut store, 1.0, 0.0).is_err());
    }
}
