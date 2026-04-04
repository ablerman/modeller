//! [`ShapeStore`] — the central arena owning all topological entities.

use std::collections::HashMap;

use slotmap::{SlotMap, new_key_type};

use brep_core::{Aabb, KernelError, ToleranceContext};

use crate::entity::{
    Compound, CompoundId, Edge, EdgeId, Face, FaceId,
    HalfEdge, HalfEdgeId, Loop, LoopId, ShapeRef, Shell, ShellId,
    Solid, SolidId, Vertex, VertexId,
};

// `slotmap` requires its key types to be declared with the `new_key_type!` macro.
// We bridge between our `EntityId<T>` (from brep-core) and SlotMap keys by
// creating thin SlotMap keys and converting at the store boundary.
new_key_type! {
    pub struct VKey;
    pub struct HeKey;
    pub struct EKey;
    pub struct LKey;
    pub struct FKey;
    pub struct ShKey;
    pub struct SoKey;
    pub struct CoKey;
}

/// Converts a `SlotMap` key to an `EntityId`.
/// Uses `as_ffi()` which encodes `(version << 32) | idx` as a u64.
pub(crate) fn to_id<T>(key: impl slotmap::Key) -> brep_core::EntityId<T> {
    let ffi = key.data().as_ffi();
    let idx = (ffi & 0xFFFF_FFFF) as u32;
    let version = (ffi >> 32) as u32;
    brep_core::EntityId::from_raw(idx, version)
}

/// Public re-export for use by the `validate` module.
pub fn to_id_pub<T>(key: impl slotmap::Key) -> brep_core::EntityId<T> {
    to_id(key)
}

/// Converts an `EntityId` back to a `SlotMap` key using `from_ffi`.
pub(crate) fn from_id<K: slotmap::Key>(id: brep_core::EntityId<impl Sized>) -> K {
    let ffi = ((id.generation() as u64) << 32) | (id.index() as u64);
    K::from(slotmap::KeyData::from_ffi(ffi))
}

/// Public re-export for use by the `validate` module.
pub fn from_id_pub<K: slotmap::Key>(id: brep_core::EntityId<impl Sized>) -> K {
    from_id(id)
}

/// The top-level container for all topological and binding data of a shape.
///
/// All entities live here; handles (`EntityId<T>`) are valid only within the
/// `ShapeStore` they were allocated from.  There is no global arena.
pub struct ShapeStore {
    pub(crate) vertices:   SlotMap<VKey, Vertex>,
    pub(crate) half_edges: SlotMap<HeKey, HalfEdge>,
    pub(crate) edges:      SlotMap<EKey, Edge>,
    pub(crate) loops:      SlotMap<LKey, Loop>,
    pub(crate) faces:      SlotMap<FKey, Face>,
    pub(crate) shells:     SlotMap<ShKey, Shell>,
    pub(crate) solids:     SlotMap<SoKey, Solid>,
    pub(crate) compounds:  SlotMap<CoKey, Compound>,

    pub tolerance: ToleranceContext,

    /// Cached bounding boxes, invalidated when the shape is modified.
    aabb_cache: HashMap<ShapeRef, Aabb>,
}

impl ShapeStore {
    /// Create an empty store with default tolerances.
    pub fn new() -> Self {
        Self::with_tolerance(ToleranceContext::default())
    }

    pub fn with_tolerance(tolerance: ToleranceContext) -> Self {
        Self {
            vertices:   SlotMap::with_key(),
            half_edges: SlotMap::with_key(),
            edges:      SlotMap::with_key(),
            loops:      SlotMap::with_key(),
            faces:      SlotMap::with_key(),
            shells:     SlotMap::with_key(),
            solids:     SlotMap::with_key(),
            compounds:  SlotMap::with_key(),
            tolerance,
            aabb_cache: HashMap::new(),
        }
    }

    // ── ID iterators (for external crates) ───────────────────────────────────

    pub fn face_ids(&self) -> impl Iterator<Item = FaceId> + '_ {
        self.faces.iter().map(|(k, _)| to_id(k))
    }
    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.edges.iter().map(|(k, _)| to_id(k))
    }
    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.vertices.iter().map(|(k, _)| to_id(k))
    }

    // ── Counts ────────────────────────────────────────────────────────────────

    pub fn vertex_count(&self)    -> usize { self.vertices.len() }
    pub fn half_edge_count(&self) -> usize { self.half_edges.len() }
    pub fn edge_count(&self)      -> usize { self.edges.len() }
    pub fn loop_count(&self)      -> usize { self.loops.len() }
    pub fn face_count(&self)      -> usize { self.faces.len() }
    pub fn shell_count(&self)     -> usize { self.shells.len() }
    pub fn solid_count(&self)     -> usize { self.solids.len() }

    // ── Allocation (insert) ───────────────────────────────────────────────────

    pub fn insert_vertex(&mut self, v: Vertex) -> VertexId {
        to_id(self.vertices.insert(v))
    }
    pub fn insert_half_edge(&mut self, he: HalfEdge) -> HalfEdgeId {
        to_id(self.half_edges.insert(he))
    }
    pub fn insert_edge(&mut self, e: Edge) -> EdgeId {
        to_id(self.edges.insert(e))
    }
    pub fn insert_loop(&mut self, l: Loop) -> LoopId {
        to_id(self.loops.insert(l))
    }
    pub fn insert_face(&mut self, f: Face) -> FaceId {
        to_id(self.faces.insert(f))
    }
    pub fn insert_shell(&mut self, s: Shell) -> ShellId {
        to_id(self.shells.insert(s))
    }
    pub fn insert_solid(&mut self, s: Solid) -> SolidId {
        to_id(self.solids.insert(s))
    }
    pub fn insert_compound(&mut self, c: Compound) -> CompoundId {
        to_id(self.compounds.insert(c))
    }

    // ── Immutable access ──────────────────────────────────────────────────────

    pub fn vertex(&self, id: VertexId) -> Result<&Vertex, KernelError> {
        self.vertices.get(from_id::<VKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn half_edge(&self, id: HalfEdgeId) -> Result<&HalfEdge, KernelError> {
        self.half_edges.get(from_id::<HeKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn edge(&self, id: EdgeId) -> Result<&Edge, KernelError> {
        self.edges.get(from_id::<EKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn loop_(&self, id: LoopId) -> Result<&Loop, KernelError> {
        self.loops.get(from_id::<LKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn face(&self, id: FaceId) -> Result<&Face, KernelError> {
        self.faces.get(from_id::<FKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn shell(&self, id: ShellId) -> Result<&Shell, KernelError> {
        self.shells.get(from_id::<ShKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn solid(&self, id: SolidId) -> Result<&Solid, KernelError> {
        self.solids.get(from_id::<SoKey>(id)).ok_or(KernelError::InvalidEntityId)
    }

    // ── Mutable access ────────────────────────────────────────────────────────

    pub fn vertex_mut(&mut self, id: VertexId) -> Result<&mut Vertex, KernelError> {
        self.vertices.get_mut(from_id::<VKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn half_edge_mut(&mut self, id: HalfEdgeId) -> Result<&mut HalfEdge, KernelError> {
        self.half_edges.get_mut(from_id::<HeKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn edge_mut(&mut self, id: EdgeId) -> Result<&mut Edge, KernelError> {
        self.edges.get_mut(from_id::<EKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn loop_mut(&mut self, id: LoopId) -> Result<&mut Loop, KernelError> {
        self.loops.get_mut(from_id::<LKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn face_mut(&mut self, id: FaceId) -> Result<&mut Face, KernelError> {
        self.faces.get_mut(from_id::<FKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn shell_mut(&mut self, id: ShellId) -> Result<&mut Shell, KernelError> {
        self.shells.get_mut(from_id::<ShKey>(id)).ok_or(KernelError::InvalidEntityId)
    }
    pub fn solid_mut(&mut self, id: SolidId) -> Result<&mut Solid, KernelError> {
        self.solids.get_mut(from_id::<SoKey>(id)).ok_or(KernelError::InvalidEntityId)
    }

    // ── Traversal ─────────────────────────────────────────────────────────────

    /// Iterate over all half-edges in the given loop.
    pub fn loop_half_edges(&self, loop_id: LoopId) -> Result<LoopHalfEdgeIter<'_>, KernelError> {
        let l = self.loop_(loop_id)?;
        Ok(LoopHalfEdgeIter {
            store: self,
            start: l.first_half_edge,
            current: Some(l.first_half_edge),
        })
    }

    /// Collect all vertices around a face's outer loop (in order).
    pub fn face_vertices(&self, face_id: FaceId) -> Result<Vec<VertexId>, KernelError> {
        let face = self.face(face_id)?;
        let mut result = Vec::new();
        for item in self.loop_half_edges(face.outer_loop)? {
            let he_id = item?;
            result.push(self.half_edge(he_id)?.origin);
        }
        Ok(result)
    }

    /// Invalidate the bounding-box cache (call after any modification).
    pub fn invalidate_cache(&mut self) {
        self.aabb_cache.clear();
    }
}

impl Default for ShapeStore {
    fn default() -> Self {
        Self::new()
    }
}

// ── Loop half-edge iterator ───────────────────────────────────────────────────

/// Iterates over the half-edges of a single loop.
pub struct LoopHalfEdgeIter<'a> {
    store: &'a ShapeStore,
    start: HalfEdgeId,
    current: Option<HalfEdgeId>,
}

impl<'a> Iterator for LoopHalfEdgeIter<'a> {
    type Item = Result<HalfEdgeId, KernelError>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.current?;
        match self.store.half_edge(current) {
            Err(e) => {
                self.current = None;
                Some(Err(e))
            }
            Ok(he) => {
                let next = he.next;
                // Stop when we've gone all the way around.
                if next == self.start {
                    self.current = None;
                } else {
                    self.current = Some(next);
                }
                Some(Ok(current))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brep_core::Point3;

    fn make_vertex(store: &mut ShapeStore, x: f64) -> VertexId {
        store.insert_vertex(Vertex::new(Point3::new(x, 0.0, 0.0), 1e-7))
    }

    #[test]
    fn empty_store_has_zero_counts() {
        let s = ShapeStore::new();
        assert_eq!(s.vertex_count(), 0);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 0);
    }

    #[test]
    fn insert_and_retrieve_vertex() {
        let mut s = ShapeStore::new();
        let id = make_vertex(&mut s, 1.0);
        let v = s.vertex(id).unwrap();
        assert_eq!(v.position.x, 1.0);
    }

    #[test]
    fn stale_id_returns_error() {
        let mut s = ShapeStore::new();
        let id = make_vertex(&mut s, 0.0);
        // Remove by creating a fresh store (IDs are local to a store).
        let s2 = ShapeStore::new();
        assert!(matches!(s2.vertex(id), Err(KernelError::InvalidEntityId)));
    }

    #[test]
    fn vertex_count_grows() {
        let mut s = ShapeStore::new();
        assert_eq!(s.vertex_count(), 0);
        make_vertex(&mut s, 0.0);
        assert_eq!(s.vertex_count(), 1);
        make_vertex(&mut s, 1.0);
        assert_eq!(s.vertex_count(), 2);
    }

    #[test]
    fn vertex_mut_updates_position() {
        let mut s = ShapeStore::new();
        let id = make_vertex(&mut s, 0.0);
        s.vertex_mut(id).unwrap().position.x = 99.0;
        assert_eq!(s.vertex(id).unwrap().position.x, 99.0);
    }

    #[test]
    fn default_tolerance_is_set() {
        let s = ShapeStore::new();
        assert!(s.tolerance.linear > 0.0);
    }

    #[test]
    fn with_tolerance_sets_tolerance() {
        let tol = ToleranceContext::coarse();
        let s = ShapeStore::with_tolerance(tol);
        assert_eq!(s.tolerance.linear, tol.linear);
    }
}
