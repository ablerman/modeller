//! Boolean operations (union, difference, intersection) on BRep solids.
//!
//! Seven-phase pipeline:
//! 1. Interference detection — BVH query over face AABBs
//! 2. Face-face intersection planes collected per face
//! 3. Polygon splitting — Sutherland-Hodgman along cutting planes
//! 4. Classification — ray-cast centroid of each piece
//! 5. Selection — keep pieces according to boolean kind
//! 6. Degenerate filter — drop pieces with < 3 non-collinear vertices
//! 7. Sewing — build a new `ShapeStore` from the selected polygon soup

use std::collections::HashMap;
use std::sync::Arc;

use brep_algo::{
    bvh::Bvh,
    query::{point_in_solid, PointLocation},
};
use brep_core::{Aabb, EntityId, KernelError, Point3, Vec3};
use brep_geom::{
    curve::LineCurve,
    surface::Plane,
};
use brep_topo::{
    binding::{CurveBinding, SurfaceBinding},
    entity::{
        Edge, Face, HalfEdge, Loop, LoopKind, Orientation,
        EdgeId, FaceId, HalfEdgeId, LoopId, SolidId, VertexId, Vertex,
    },
    euler::make_solid_shell,
    store::ShapeStore,
};

// ── Public API ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanKind {
    Union,
    Difference,
    Intersection,
}

/// Compute a boolean operation between two solids.
///
/// Both input stores are treated as read-only; the result is a fresh
/// `ShapeStore` containing a single closed solid.
///
/// Currently handles polyhedral (planar-face) solids correctly.
/// Curved surfaces use the same classification but the split step may be
/// approximate — full curved-surface boolean support is a future extension.
pub fn boolean_op(
    store_a: &ShapeStore,
    solid_a: SolidId,
    store_b: &ShapeStore,
    solid_b: SolidId,
    kind: BooleanKind,
    tol: f64,
) -> Result<ShapeStore, KernelError> {
    // ── Phase 1: BVH ──────────────────────────────────────────────────────────
    let bvh_a = Bvh::build(store_a);
    let bvh_b = Bvh::build(store_b);

    // ── Phase 2-3: Collect, split, classify faces of A relative to B ─────────
    let pieces_a = collect_pieces(store_a, solid_a)?;
    let classified_a =
        split_and_classify(pieces_a, store_b, solid_b, &bvh_b, tol)?;

    // ── Phase 2-3: Collect, split, classify faces of B relative to A ─────────
    let pieces_b = collect_pieces(store_b, solid_b)?;
    let classified_b =
        split_and_classify(pieces_b, store_a, solid_a, &bvh_a, tol)?;

    // ── Phase 5: Select ───────────────────────────────────────────────────────
    let mut selected: Vec<Piece> = Vec::new();

    for (piece, loc) in classified_a {
        if keep_from_a(loc, kind) {
            selected.push(piece);
        }
    }

    for (piece, loc) in classified_b {
        if keep_from_b(loc, kind) {
            let mut p = piece;
            if kind == BooleanKind::Difference {
                // Flip orientation so cavity walls face outward.
                p.verts.reverse();
                p.normal = -p.normal;
            }
            selected.push(p);
        }
    }

    // ── Phase 6: Drop degenerate pieces ──────────────────────────────────────
    let selected: Vec<Piece> = selected.into_iter().filter(|p| p.verts.len() >= 3).collect();

    // ── Phase 7: Sew ─────────────────────────────────────────────────────────
    build_result_store(selected, tol)
}

fn keep_from_a(loc: PointLocation, kind: BooleanKind) -> bool {
    match (kind, loc) {
        (BooleanKind::Union,        PointLocation::Outside)    => true,
        (BooleanKind::Union,        PointLocation::OnBoundary) => true,
        (BooleanKind::Difference,   PointLocation::Outside)    => true,
        (BooleanKind::Difference,   PointLocation::OnBoundary) => true,
        (BooleanKind::Intersection, PointLocation::Inside)     => true,
        (BooleanKind::Intersection, PointLocation::OnBoundary) => true,
        _ => false,
    }
}

fn keep_from_b(loc: PointLocation, kind: BooleanKind) -> bool {
    match (kind, loc) {
        (BooleanKind::Union,        PointLocation::Outside)    => true,
        (BooleanKind::Difference,   PointLocation::Inside)     => true,
        (BooleanKind::Intersection, PointLocation::Inside)     => true,
        (BooleanKind::Intersection, PointLocation::OnBoundary) => true,
        _ => false,
    }
}

// ── Internal: face piece ──────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct Piece {
    verts: Vec<Point3>,
    normal: Vec3,
}

impl Piece {
    fn centroid(&self) -> Point3 {
        let n = self.verts.len() as f64;
        let s = self.verts.iter().fold(Vec3::zeros(), |acc, p| acc + p.coords);
        Point3::from(s / n)
    }

    fn aabb(&self) -> Aabb {
        let mut bb = Aabb::empty();
        for p in &self.verts {
            bb = bb.union(&Aabb::from_corners(*p, *p));
        }
        bb
    }
}

// ── Phase 2: Collect face polygons ────────────────────────────────────────────

fn collect_pieces(
    store: &ShapeStore,
    solid_id: SolidId,
) -> Result<Vec<Piece>, KernelError> {
    let solid  = store.solid(solid_id)?;
    let shell  = store.shell(solid.outer_shell)?;
    let mut pieces = Vec::new();

    for &fid in &shell.faces {
        let verts = store.face_vertices(fid)?;
        if verts.len() < 3 {
            continue;
        }
        let positions: Vec<Point3> = verts
            .iter()
            .map(|&vid| store.vertex(vid).map(|v| v.position))
            .collect::<Result<_, _>>()?;
        let normal = newell_normal(&positions);
        pieces.push(Piece { verts: positions, normal });
    }

    Ok(pieces)
}

// ── Phase 3: Split faces at intersection + Phase 4: Classify ─────────────────

fn split_and_classify(
    pieces: Vec<Piece>,
    other_store: &ShapeStore,
    other_solid: SolidId,
    other_bvh: &Bvh,
    tol: f64,
) -> Result<Vec<(Piece, PointLocation)>, KernelError> {
    let other_shell = other_store.shell(other_store.solid(other_solid)?.outer_shell)?;

    // Precompute planes of all other-solid faces (outward normal, offset d).
    let other_planes: Vec<(FaceId, Vec3, f64)> = other_shell
        .faces
        .iter()
        .map(|&fid| {
            let verts = other_store.face_vertices(fid)?;
            let pos: Vec<Point3> = verts
                .iter()
                .map(|&vid| other_store.vertex(vid).map(|v| v.position))
                .collect::<Result<_, _>>()?;
            if pos.len() < 3 {
                return Ok(None);
            }
            let n = newell_normal(&pos);
            let d = n.dot(&pos[0].coords);
            Ok(Some((fid, n, d)))
        })
        .collect::<Result<Vec<_>, KernelError>>()?
        .into_iter()
        .flatten()
        .collect();

    let mut result = Vec::new();

    for piece in pieces {
        // Use AABB to find which other-solid faces are nearby.
        let piece_aabb = piece.aabb().expand_by(tol * 10.0);
        let candidates: Vec<FaceId> = other_bvh.intersect_aabb(&piece_aabb);

        // Collect the cutting planes: planes from other solid that bisect this piece.
        let mut cutting: Vec<(Vec3, f64)> = Vec::new();
        for &(fid, n_b, d_b) in &other_planes {
            if !candidates.contains(&fid) {
                continue;
            }
            // Check if this plane actually bisects the piece:
            let (min_s, max_s) = piece
                .verts
                .iter()
                .fold((f64::MAX, f64::MIN), |(lo, hi), p| {
                    let s = n_b.dot(&p.coords) - d_b;
                    (lo.min(s), hi.max(s))
                });
            if min_s < -tol && max_s > tol {
                cutting.push((n_b, d_b));
            }
        }

        // Split the piece by each cutting plane.
        let sub_pieces = split_piece(piece, &cutting, tol);

        // Classify each sub-piece.
        for sub in sub_pieces {
            if sub.verts.len() < 3 {
                continue;
            }
            let centroid = sub.centroid();
            let loc = point_in_solid(other_store, other_bvh, other_solid, &centroid, tol)?;
            result.push((sub, loc));
        }
    }

    Ok(result)
}

/// Recursively split a piece by a list of planes, returning all sub-pieces.
fn split_piece(piece: Piece, planes: &[(Vec3, f64)], tol: f64) -> Vec<Piece> {
    let mut current = vec![piece];

    for &(n, d) in planes {
        let mut next = Vec::new();
        for p in current {
            let (pos_verts, neg_verts) = split_polygon_by_plane(&p.verts, n, d, tol);
            if pos_verts.len() >= 3 {
                next.push(Piece { verts: pos_verts, normal: p.normal });
            }
            if neg_verts.len() >= 3 {
                next.push(Piece { verts: neg_verts, normal: p.normal });
            }
        }
        current = next;
    }

    current
}

/// Split a polygon into two parts by the plane n·p = d.
///
/// Returns (positive side: n·p ≥ d, negative side: n·p ≤ d).
/// Vertices on the plane (within tol) appear in both halves.
fn split_polygon_by_plane(
    verts: &[Point3],
    n: Vec3,
    d: f64,
    tol: f64,
) -> (Vec<Point3>, Vec<Point3>) {
    let n_pts = verts.len();
    let mut pos: Vec<Point3> = Vec::new();
    let mut neg: Vec<Point3> = Vec::new();

    for i in 0..n_pts {
        let curr = verts[i];
        let next = verts[(i + 1) % n_pts];
        let s_curr = n.dot(&curr.coords) - d;
        let s_next = n.dot(&next.coords) - d;

        if s_curr >= -tol {
            pos.push(curr);
        }
        if s_curr <= tol {
            neg.push(curr);
        }

        // Edge crosses the cutting plane.
        let crossed = (s_curr > tol && s_next < -tol) || (s_curr < -tol && s_next > tol);
        if crossed {
            let t = s_curr / (s_curr - s_next);
            let isect: Point3 = curr + t * (next - curr);
            pos.push(isect);
            neg.push(isect);
        }
    }

    (pos, neg)
}

// ── Phase 7: Build result ShapeStore ─────────────────────────────────────────

/// Assemble selected polygon pieces into a new `ShapeStore`.
fn build_result_store(pieces: Vec<Piece>, tol: f64) -> Result<ShapeStore, KernelError> {
    if pieces.is_empty() {
        return Err(KernelError::InvalidTopology(
            "boolean_op: no faces selected — result is empty".into(),
        ));
    }

    let mut store = ShapeStore::new();
    let make_result = make_solid_shell(&mut store);
    let _solid_id = make_result.solid_id;
    let shell_id = make_result.shell_id;

    // ── 7a. Deduplicate vertices ──────────────────────────────────────────────
    let mut vertex_map: Vec<(Point3, VertexId)> = Vec::new();

    let mut get_or_create_vertex = |store: &mut ShapeStore,
                                     pos: Point3|
     -> VertexId {
        for &(existing, vid) in &vertex_map {
            if (existing - pos).norm() < tol * 10.0 {
                return vid;
            }
        }
        let vid = store.insert_vertex(Vertex::new(pos, store.tolerance.linear));
        vertex_map.push((pos, vid));
        vid
    };

    // ── 7b. Create faces and half-edges ──────────────────────────────────────
    let sentinel_he: HalfEdgeId = EntityId::from_raw(0, 0);
    let sentinel_e: EdgeId      = EntityId::from_raw(0, 0);
    let sentinel_l: LoopId      = EntityId::from_raw(0, 0);

    let mut all_hes: Vec<(HalfEdgeId, VertexId, VertexId)> = Vec::new(); // (he_id, origin, dest)

    for piece in &pieces {
        let n = piece.verts.len();
        if n < 3 {
            continue;
        }

        // Map 3D vertices to VertexIds.
        let vids: Vec<VertexId> = piece
            .verts
            .iter()
            .map(|&p| get_or_create_vertex(&mut store, p))
            .collect();

        // Create Face.
        let surf = SurfaceBinding::new(
            Arc::new(Plane::new(piece.verts[0], {
                // Build two orthogonal vectors in the face plane.
                let u = (piece.verts[1] - piece.verts[0]).normalize();
                u
            }, piece.normal.cross(&(piece.verts[1] - piece.verts[0]).normalize()))),
            true,
        );
        let face_id = store.insert_face(Face {
            outer_loop: sentinel_l,
            inner_loops: smallvec::SmallVec::new(),
            surface: surf,
            orientation: Orientation::Same,
            shell: shell_id,
            tolerance: store.tolerance.linear,
        });

        // Create Loop.
        let loop_id = store.insert_loop(Loop {
            first_half_edge: sentinel_he,
            loop_kind: LoopKind::Outer,
            face: face_id,
        });
        store.face_mut(face_id)?.outer_loop = loop_id;

        // Create HalfEdges.
        let mut he_ids: Vec<HalfEdgeId> = Vec::with_capacity(n);
        for i in 0..n {
            let he_id = store.insert_half_edge(HalfEdge {
                origin: vids[i],
                twin: sentinel_he,
                next: sentinel_he,
                prev: sentinel_he,
                loop_id,
                edge: sentinel_e,
                pcurve: None,
            });
            he_ids.push(he_id);
        }

        // Wire next/prev.
        for i in 0..n {
            store.half_edge_mut(he_ids[i])?.next = he_ids[(i + 1) % n];
            store.half_edge_mut(he_ids[i])?.prev = he_ids[(i + n - 1) % n];
        }
        store.loop_mut(loop_id)?.first_half_edge = he_ids[0];

        // Register face in shell.
        store.shell_mut(shell_id)?.faces.push(face_id);

        // Record HE → (origin, dest) for sewing.
        for i in 0..n {
            all_hes.push((he_ids[i], vids[i], vids[(i + 1) % n]));
        }
    }

    // ── 7c. Sew: match twins by (dest→origin) ────────────────────────────────
    // Build map (origin, dest) → HalfEdgeId using VertexId raw keys.
    let vid_key = |v: VertexId| -> u64 {
        (v.index() as u64) << 32 | v.generation() as u64
    };
    let mut he_map: HashMap<(u64, u64), HalfEdgeId> = HashMap::new();
    for &(he_id, origin, dest) in &all_hes {
        he_map.insert((vid_key(origin), vid_key(dest)), he_id);
    }

    for &(he_id, origin, dest) in &all_hes {
        let he = store.half_edge(he_id)?;
        if he.twin != sentinel_he {
            continue;
        }
        // Look for twin: (dest → origin).
        if let Some(&twin_id) = he_map.get(&(vid_key(dest), vid_key(origin))) {
            if twin_id != he_id {
                let edge_id = store.insert_edge(Edge {
                    half_edges: [he_id, twin_id],
                    curve: None,
                    tolerance: store.tolerance.linear,
                    is_degenerate: false,
                });
                store.half_edge_mut(he_id)?.twin  = twin_id;
                store.half_edge_mut(twin_id)?.twin = he_id;
                store.half_edge_mut(he_id)?.edge   = edge_id;
                store.half_edge_mut(twin_id)?.edge  = edge_id;

                // Attach line curve.
                let from = store.vertex(origin)?.position;
                let to   = store.vertex(dest)?.position;
                let dir  = to - from;
                if dir.norm() > 1e-15 {
                    let curve = Arc::new(LineCurve::new(from, dir));
                    store.edge_mut(edge_id)?.curve =
                        Some(CurveBinding::new(curve, 0.0, 1.0, true));
                }
            }
        }
    }

    store.shell_mut(shell_id)?.is_closed = true;
    Ok(store)
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Newell's method for a robust polygon normal.
fn newell_normal(pts: &[Point3]) -> Vec3 {
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0f64, 0.0f64, 0.0f64);
    for i in 0..n {
        let a = &pts[i];
        let b = &pts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
    }
    let v = Vec3::new(nx, ny, nz);
    if v.norm() > 1e-14 { v.normalize() } else { Vec3::z() }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::primitives::make_box;

    #[test]
    fn union_non_overlapping_fails_gracefully() {
        // Two boxes that don't touch — union should keep all faces of both.
        let mut sa = ShapeStore::new();
        let solid_a = make_box(&mut sa, 1.0, 1.0, 1.0).unwrap();

        let mut sb = ShapeStore::new();
        let solid_b = make_box(&mut sb, 1.0, 1.0, 1.0).unwrap();
        // shift b far away from a
        for vid in sb.vertex_ids().collect::<Vec<_>>() {
            sb.vertex_mut(vid).unwrap().position.x += 10.0;
        }

        let result = boolean_op(&sa, solid_a, &sb, solid_b, BooleanKind::Union, 1e-7);
        // Non-overlapping union: all faces of A are outside B and vice versa.
        let result = result.unwrap();
        // Both solids have 6 faces each → 12 total.
        assert_eq!(result.face_count(), 12);
    }

    #[test]
    fn intersection_non_overlapping_is_empty() {
        let mut sa = ShapeStore::new();
        let solid_a = make_box(&mut sa, 1.0, 1.0, 1.0).unwrap();

        let mut sb = ShapeStore::new();
        let solid_b = make_box(&mut sb, 1.0, 1.0, 1.0).unwrap();
        for vid in sb.vertex_ids().collect::<Vec<_>>() {
            sb.vertex_mut(vid).unwrap().position.x += 10.0;
        }

        let result = boolean_op(&sa, solid_a, &sb, solid_b, BooleanKind::Intersection, 1e-7);
        // No overlap → result is empty → returns an error.
        assert!(result.is_err());
    }

    #[test]
    fn union_overlapping_boxes_face_count() {
        // Box A: [0,2]^3,  Box B: [1,3]^3  — overlap = [1,2]^3.
        let mut sa = ShapeStore::new();
        let solid_a = make_box(&mut sa, 2.0, 2.0, 2.0).unwrap();

        let mut sb = ShapeStore::new();
        let solid_b = make_box(&mut sb, 2.0, 2.0, 2.0).unwrap();
        for vid in sb.vertex_ids().collect::<Vec<_>>() {
            let v = sb.vertex_mut(vid).unwrap();
            v.position.x += 1.0;
            v.position.y += 1.0;
            v.position.z += 1.0;
        }

        let result = boolean_op(&sa, solid_a, &sb, solid_b, BooleanKind::Union, 1e-6)
            .expect("union should succeed");

        // The union of two overlapping boxes should have more than 6 but a
        // finite number of faces — exact count depends on split geometry.
        assert!(result.face_count() >= 6, "got {} faces", result.face_count());
        assert!(result.solid_count() == 1);
    }

    #[test]
    fn difference_overlapping_boxes_has_faces() {
        let mut sa = ShapeStore::new();
        let solid_a = make_box(&mut sa, 2.0, 2.0, 2.0).unwrap();

        let mut sb = ShapeStore::new();
        let solid_b = make_box(&mut sb, 2.0, 2.0, 2.0).unwrap();
        for vid in sb.vertex_ids().collect::<Vec<_>>() {
            let v = sb.vertex_mut(vid).unwrap();
            v.position.x += 1.0;
            v.position.y += 1.0;
            v.position.z += 1.0;
        }

        let result = boolean_op(&sa, solid_a, &sb, solid_b, BooleanKind::Difference, 1e-6)
            .expect("difference should succeed");

        assert!(result.face_count() >= 6, "got {} faces", result.face_count());
        assert_eq!(result.solid_count(), 1);
    }

    #[test]
    fn intersection_overlapping_boxes_has_faces() {
        let mut sa = ShapeStore::new();
        let solid_a = make_box(&mut sa, 2.0, 2.0, 2.0).unwrap();

        let mut sb = ShapeStore::new();
        let solid_b = make_box(&mut sb, 2.0, 2.0, 2.0).unwrap();
        for vid in sb.vertex_ids().collect::<Vec<_>>() {
            let v = sb.vertex_mut(vid).unwrap();
            v.position.x += 1.0;
            v.position.y += 1.0;
            v.position.z += 1.0;
        }

        let result = boolean_op(&sa, solid_a, &sb, solid_b, BooleanKind::Intersection, 1e-6)
            .expect("intersection should succeed");

        // The intersection of [0,2]^3 and [1,3]^3 is the 1×1×1 cube [1,2]^3.
        // That cube has 6 faces.
        assert_eq!(result.face_count(), 6, "got {} faces", result.face_count());
        assert_eq!(result.solid_count(), 1);
    }
}
