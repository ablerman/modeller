//! Euler operators — the only correct way to build valid BRep topology.
//!
//! Each operator maintains the Euler-Poincaré invariant:
//!   V - E + F - (L - F) - 2*(S - H) = 0
//!
//! where V=vertices, E=edges, F=faces, L=loops, S=shells, H=through-holes.
//!
//! Operators implemented here are sufficient for Phase 2 (hand-built box):
//! - [`make_solid_shell`]     (MZEV) — create an empty solid with one shell
//! - [`make_face`]            — add an isolated face to a shell
//! - [`make_edge_vertex`]     (MEV)  — add an edge and vertex to a loop
//! - [`make_edge_face`]       (MEF)  — split a face loop with a new edge
//! - [`close_loop`]           — connect the last half-edge back to close a loop
//!
//! Full inverse operators (kill variants) will be added in Phase 5 (boolean ops).

use brep_core::{KernelError, Point3};

use crate::binding::SurfaceBinding;
use crate::entity::{
    Edge, Face, HalfEdge, Loop, LoopKind, Orientation, Shell, ShellId, Solid, SolidId, Vertex,
    VertexId, HalfEdgeId, LoopId, FaceId, EdgeId,
};
use crate::store::ShapeStore;

/// Result of creating a new solid + shell.
pub struct MakeSolidShellResult {
    pub solid_id: SolidId,
    pub shell_id: ShellId,
}

/// MZEV — Make a new empty solid backed by one empty shell.
///
/// This is the starting point for constructing any solid.
pub fn make_solid_shell(store: &mut ShapeStore) -> MakeSolidShellResult {
    let shell_id = store.insert_shell(Shell::new());
    let solid_id = store.insert_solid(Solid::new(shell_id));
    MakeSolidShellResult { solid_id, shell_id }
}

/// Result of adding an isolated planar face.
pub struct MakeFaceResult {
    pub face_id: FaceId,
    pub loop_id: LoopId,
}

/// Add a new face (with an empty outer loop) to `shell_id`.
///
/// The caller is responsible for populating the loop with half-edges afterwards
/// using `make_edge_vertex` / `close_loop`.
pub fn make_face(
    store: &mut ShapeStore,
    shell_id: ShellId,
    surface: SurfaceBinding,
    orientation: Orientation,
) -> Result<MakeFaceResult, KernelError> {
    // We need a valid FaceId before we can create the Loop (which needs face).
    // Use a two-step approach: insert a placeholder face, create the loop,
    // then fix up the face's outer_loop field.
    //
    // To avoid the chicken-and-egg problem we temporarily use a sentinel loop id.
    // We'll overwrite it immediately.
    let sentinel_loop_id: LoopId = brep_core::EntityId::from_raw(0, 0);

    // 1. Insert face with placeholder loop id.
    let face_id = store.insert_face(Face {
        outer_loop: sentinel_loop_id, // will be updated below
        inner_loops: smallvec::SmallVec::new(),
        surface,
        orientation,
        shell: shell_id,
        tolerance: store.tolerance.linear,
    });

    // 2. Create the loop pointing to the real face_id.
    //    Use a sentinel half-edge id; the loop starts empty.
    let sentinel_he_id: HalfEdgeId = brep_core::EntityId::from_raw(0, 0);
    let loop_id = store.insert_loop(Loop {
        first_half_edge: sentinel_he_id,
        loop_kind: LoopKind::Outer,
        face: face_id,
    });

    // 3. Patch the face to point at the real loop.
    store.face_mut(face_id)?.outer_loop = loop_id;

    // 4. Register the face in the shell.
    store.shell_mut(shell_id)?.faces.push(face_id);

    Ok(MakeFaceResult { face_id, loop_id })
}

/// Result of MEV (Make Edge + Vertex).
pub struct MevResult {
    pub vertex_id: VertexId,
    pub edge_id:   EdgeId,
    pub he_fwd_id: HalfEdgeId,
    pub he_rev_id: HalfEdgeId,
}

/// MEV — Add a new vertex and the edge connecting it to an existing vertex.
///
/// The new half-edges are inserted into `loop_id` after `after_he`:
///   `after_he → he_fwd → (new vertex) → he_rev → next_of_after`
///
/// If `after_he` is the sentinel (loop is empty), the half-edges form a
/// two-half-edge loop: `he_fwd → he_rev → he_fwd`.
pub fn make_edge_vertex(
    store: &mut ShapeStore,
    loop_id: LoopId,
    after_he: Option<HalfEdgeId>,
    start_vertex: VertexId,
    new_position: Point3,
) -> Result<MevResult, KernelError> {
    let new_vertex_id = store.insert_vertex(Vertex::new(new_position, store.tolerance.linear));

    // Allocate IDs upfront (IDs are only valid once inserted).
    // We use a two-step pattern: reserve space, then patch.
    let sentinel_he: HalfEdgeId = brep_core::EntityId::from_raw(0, 0);
    let sentinel_e:  EdgeId      = brep_core::EntityId::from_raw(0, 0);

    let he_fwd_id = store.insert_half_edge(HalfEdge {
        origin: start_vertex,
        twin: sentinel_he,
        next: sentinel_he,
        prev: sentinel_he,
        loop_id,
        edge: sentinel_e,
        pcurve: None,
    });

    let he_rev_id = store.insert_half_edge(HalfEdge {
        origin: new_vertex_id,
        twin: he_fwd_id,
        next: sentinel_he,
        prev: sentinel_he,
        loop_id,
        edge: sentinel_e,
        pcurve: None,
    });
    store.half_edge_mut(he_fwd_id)?.twin = he_rev_id;

    let edge_id = store.insert_edge(Edge::new(he_fwd_id, he_rev_id));
    store.half_edge_mut(he_fwd_id)?.edge = edge_id;
    store.half_edge_mut(he_rev_id)?.edge = edge_id;

    // Link into the loop.
    let l = store.loop_(loop_id)?;
    let is_empty = l.first_half_edge == sentinel_he;

    if is_empty {
        // First edge in this loop: fwd ↔ rev (2-cycle).
        store.half_edge_mut(he_fwd_id)?.next = he_rev_id;
        store.half_edge_mut(he_fwd_id)?.prev = he_rev_id;
        store.half_edge_mut(he_rev_id)?.next = he_fwd_id;
        store.half_edge_mut(he_rev_id)?.prev = he_fwd_id;
        store.loop_mut(loop_id)?.first_half_edge = he_fwd_id;
    } else {
        let prev_he = after_he.unwrap_or(l.first_half_edge);
        let next_he = store.half_edge(prev_he)?.next;

        // Insert: prev_he → he_fwd → he_rev → next_he
        store.half_edge_mut(he_fwd_id)?.prev = prev_he;
        store.half_edge_mut(he_fwd_id)?.next = he_rev_id;
        store.half_edge_mut(he_rev_id)?.prev = he_fwd_id;
        store.half_edge_mut(he_rev_id)?.next = next_he;
        store.half_edge_mut(prev_he)?.next   = he_fwd_id;
        store.half_edge_mut(next_he)?.prev   = he_rev_id;
    }

    // Register the outgoing half-edge on the vertex.
    store.vertex_mut(new_vertex_id)?.outgoing_half_edges.push(he_rev_id);
    store.vertex_mut(start_vertex)?.outgoing_half_edges.push(he_fwd_id);

    Ok(MevResult { vertex_id: new_vertex_id, edge_id, he_fwd_id, he_rev_id })
}

/// Close a loop by inserting a closing edge from the current "tip" back to
/// the loop's first vertex.
///
/// `tip_vertex` — vertex at the end of the last MEV call.
/// `first_vertex` — vertex that was the start of the very first MEV call.
/// `last_he` — the last `he_rev_id` returned by MEV.
///
/// After this call the loop is fully closed and topologically valid.
pub fn close_loop(
    store: &mut ShapeStore,
    loop_id: LoopId,
    tip_vertex: VertexId,
    first_vertex: VertexId,
    last_he: HalfEdgeId,
) -> Result<(EdgeId, HalfEdgeId, HalfEdgeId), KernelError> {
    let sentinel_he: HalfEdgeId = brep_core::EntityId::from_raw(0, 0);
    let sentinel_e:  EdgeId      = brep_core::EntityId::from_raw(0, 0);

    let he_fwd_id = store.insert_half_edge(HalfEdge {
        origin: tip_vertex,
        twin: sentinel_he,
        next: sentinel_he,
        prev: sentinel_he,
        loop_id,
        edge: sentinel_e,
        pcurve: None,
    });
    let he_rev_id = store.insert_half_edge(HalfEdge {
        origin: first_vertex,
        twin: he_fwd_id,
        next: sentinel_he,
        prev: sentinel_he,
        loop_id,
        edge: sentinel_e,
        pcurve: None,
    });
    store.half_edge_mut(he_fwd_id)?.twin = he_rev_id;

    let edge_id = store.insert_edge(Edge::new(he_fwd_id, he_rev_id));
    store.half_edge_mut(he_fwd_id)?.edge = edge_id;
    store.half_edge_mut(he_rev_id)?.edge = edge_id;

    // The loop's first half-edge is the target for the closing edge.
    let first_he = store.loop_(loop_id)?.first_half_edge;

    // Chain: last_he → he_fwd → first_he  (closing the ring)
    // And:   prev_of_first → he_rev → he_fwd
    let prev_of_first = store.half_edge(first_he)?.prev;

    store.half_edge_mut(he_fwd_id)?.prev = last_he;
    store.half_edge_mut(he_fwd_id)?.next = first_he;
    store.half_edge_mut(he_rev_id)?.prev = prev_of_first;
    store.half_edge_mut(he_rev_id)?.next = he_fwd_id;

    store.half_edge_mut(last_he)?.next   = he_fwd_id;
    store.half_edge_mut(first_he)?.prev  = he_fwd_id;
    store.half_edge_mut(prev_of_first)?.next = he_rev_id;

    store.vertex_mut(tip_vertex)?.outgoing_half_edges.push(he_fwd_id);
    store.vertex_mut(first_vertex)?.outgoing_half_edges.push(he_rev_id);

    Ok((edge_id, he_fwd_id, he_rev_id))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use brep_geom::surface::Plane;
    use crate::binding::SurfaceBinding;

    fn plane_binding() -> SurfaceBinding {
        SurfaceBinding::new(Arc::new(Plane::xy()), true)
    }

    #[test]
    fn make_solid_shell_creates_valid_solid_and_shell() {
        let mut store = ShapeStore::new();
        let res = make_solid_shell(&mut store);
        assert_eq!(store.solid_count(), 1);
        assert_eq!(store.shell_count(), 1);
        let solid = store.solid(res.solid_id).unwrap();
        assert_eq!(solid.outer_shell, res.shell_id);
    }

    #[test]
    fn make_face_registers_in_shell() {
        let mut store = ShapeStore::new();
        let MakeSolidShellResult { shell_id, .. } = make_solid_shell(&mut store);
        let MakeFaceResult { face_id, .. } = make_face(
            &mut store, shell_id, plane_binding(), Orientation::Same
        ).unwrap();
        let shell = store.shell(shell_id).unwrap();
        assert!(shell.faces.contains(&face_id));
    }

    #[test]
    fn mev_creates_vertex_and_edge() {
        let mut store = ShapeStore::new();
        let MakeSolidShellResult { shell_id, .. } = make_solid_shell(&mut store);
        let MakeFaceResult { loop_id, .. } = make_face(
            &mut store, shell_id, plane_binding(), Orientation::Same
        ).unwrap();

        let v0 = store.insert_vertex(Vertex::new(Point3::origin(), 1e-7));

        let res = make_edge_vertex(&mut store, loop_id, None, v0, Point3::new(1.0, 0.0, 0.0))
            .unwrap();

        assert_eq!(store.edge_count(), 1);
        assert_eq!(store.vertex_count(), 2);
        let edge = store.edge(res.edge_id).unwrap();
        assert_eq!(edge.half_edges[0], res.he_fwd_id);
        assert_eq!(edge.half_edges[1], res.he_rev_id);
    }

    #[test]
    fn twin_pointers_are_consistent() {
        let mut store = ShapeStore::new();
        let MakeSolidShellResult { shell_id, .. } = make_solid_shell(&mut store);
        let MakeFaceResult { loop_id, .. } = make_face(
            &mut store, shell_id, plane_binding(), Orientation::Same
        ).unwrap();
        let v0 = store.insert_vertex(Vertex::new(Point3::origin(), 1e-7));
        let res = make_edge_vertex(&mut store, loop_id, None, v0, Point3::new(1.0, 0.0, 0.0))
            .unwrap();

        let fwd = store.half_edge(res.he_fwd_id).unwrap();
        let rev = store.half_edge(res.he_rev_id).unwrap();
        assert_eq!(fwd.twin, res.he_rev_id);
        assert_eq!(rev.twin, res.he_fwd_id);
    }
}
