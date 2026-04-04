//! Generational entity handle used as an arena key.
//!
//! [`EntityId<T>`] is a `Copy` newtype over a `(u32, u32)` pair
//! (slot index + generation counter).  The phantom type parameter
//! `T` makes handles for different entity types incompatible at
//! compile time while keeping them zero-cost at runtime.
//!
//! # Why not raw indices?
//! A raw `usize` cannot detect use-after-remove.  Generational
//! indices detect stale handles without unsafe code.
//!
//! # Interoperability with `slotmap`
//! `brep-topo` stores entities in `slotmap::SlotMap`.  `slotmap`
//! requires its key type to implement [`slotmap::Key`], so the
//! `brep-topo` crate provides a thin wrapper.  `EntityId` itself
//! intentionally does not depend on `slotmap` so that `brep-core`
//! stays lightweight.

use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

use serde::{Deserialize, Serialize};

/// A typed, generational handle into an arena.
///
/// # Type parameter
/// `T` is the *phantom* entity type.  It is not stored; it only
/// prevents mixing handles of different types.
#[derive(Serialize, Deserialize)]
pub struct EntityId<T> {
    index: u32,
    generation: u32,
    #[serde(skip)]
    _phantom: PhantomData<fn() -> T>,
}

impl<T> EntityId<T> {
    /// Construct directly from raw parts.  Prefer arena allocation.
    #[inline]
    pub fn from_raw(index: u32, generation: u32) -> Self {
        Self { index, generation, _phantom: PhantomData }
    }

    /// Raw slot index (for use by arena implementations).
    #[inline]
    pub fn index(self) -> u32 {
        self.index
    }

    /// Generation counter (for use by arena implementations).
    #[inline]
    pub fn generation(self) -> u32 {
        self.generation
    }
}

// Manual impls so that the PhantomData doesn't add unwanted bounds.
impl<T> Copy for EntityId<T> {}
impl<T> Clone for EntityId<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> PartialEq for EntityId<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index && self.generation == other.generation
    }
}
impl<T> Eq for EntityId<T> {}
impl<T> Hash for EntityId<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
        self.generation.hash(state);
    }
}
impl<T> fmt::Debug for EntityId<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "EntityId({}:{})", self.index, self.generation)
    }
}
impl<T> fmt::Display for EntityId<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.index, self.generation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakeVertex;
    struct FakeEdge;

    type VertexId = EntityId<FakeVertex>;
    type EdgeId = EntityId<FakeEdge>;

    #[test]
    fn copy_semantics() {
        let a: VertexId = EntityId::from_raw(1, 0);
        let b = a; // copy
        assert_eq!(a, b);
    }

    #[test]
    fn equality_checks_both_fields() {
        let a: VertexId = EntityId::from_raw(1, 0);
        let b: VertexId = EntityId::from_raw(1, 1); // same index, different gen
        let c: VertexId = EntityId::from_raw(2, 0); // different index
        assert_ne!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn type_safety_compile_check() {
        // These two lines would not compile if uncommented:
        //   let _: VertexId = EdgeId::from_raw(0, 0);
        let _v: VertexId = EntityId::from_raw(0, 0);
        let _e: EdgeId = EntityId::from_raw(0, 0);
        // Different types even with identical bit patterns — no comparison possible.
    }

    #[test]
    fn hash_consistency() {
        use std::collections::HashMap;
        let mut map: HashMap<VertexId, &str> = HashMap::new();
        let id = EntityId::from_raw(42, 3);
        map.insert(id, "hello");
        assert_eq!(map[&id], "hello");
    }

    #[test]
    fn debug_format() {
        let id: VertexId = EntityId::from_raw(7, 2);
        let s = format!("{:?}", id);
        assert!(s.contains("7") && s.contains("2"));
    }

    #[test]
    fn serde_roundtrip() {
        let id: VertexId = EntityId::from_raw(5, 3);
        let json = serde_json::to_string(&id).unwrap();
        let id2: VertexId = serde_json::from_str(&json).unwrap();
        assert_eq!(id, id2);
    }
}
