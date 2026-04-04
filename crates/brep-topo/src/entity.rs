//! Topological entity structs: Vertex, HalfEdge, Edge, Loop, Face, Shell, Solid, Compound.
//!
//! All entities are stored by value inside [`ShapeStore`] arenas.
//! Cross-references use typed [`EntityId`] handles — never raw pointers or `Rc`.

use smallvec::SmallVec;

use brep_core::{EntityId, Point3};

use crate::binding::{CurveBinding, Pcurve, SurfaceBinding};

// ── Type aliases ──────────────────────────────────────────────────────────────

/// A phantom type tag used for [`EntityId`] typing.
pub struct VertexTag;
pub struct HalfEdgeTag;
pub struct EdgeTag;
pub struct LoopTag;
pub struct FaceTag;
pub struct ShellTag;
pub struct SolidTag;
pub struct CompoundTag;

pub type VertexId    = EntityId<VertexTag>;
pub type HalfEdgeId  = EntityId<HalfEdgeTag>;
pub type EdgeId      = EntityId<EdgeTag>;
pub type LoopId      = EntityId<LoopTag>;
pub type FaceId      = EntityId<FaceTag>;
pub type ShellId     = EntityId<ShellTag>;
pub type SolidId     = EntityId<SolidTag>;
pub type CompoundId  = EntityId<CompoundTag>;

// ── Orientation ───────────────────────────────────────────────────────────────

/// Whether a topological entity's geometry agrees with the underlying
/// geometric object's natural orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    /// Topology and geometry agree in direction / normal sense.
    Same,
    /// Topology is reversed relative to geometry.
    Reversed,
}

impl Orientation {
    pub fn flip(self) -> Self {
        match self {
            Orientation::Same => Orientation::Reversed,
            Orientation::Reversed => Orientation::Same,
        }
    }

    pub fn is_same(self) -> bool {
        self == Orientation::Same
    }
}

// ── Loop kind ─────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopKind {
    /// The outer (outer-boundary) loop of a face.
    Outer,
    /// An inner (hole) loop of a face.
    Inner,
}

// ── Entities ──────────────────────────────────────────────────────────────────

/// A topological vertex.  Geometry: a single 3-D point.
#[derive(Clone, Debug)]
pub struct Vertex {
    /// 3-D position.
    pub position: Point3,
    /// Per-vertex tolerance.  May be larger than the global tolerance
    /// when the vertex was produced by an imprecise operation.
    pub tolerance: f64,
    /// Half-edges that originate at this vertex.
    /// `SmallVec<[_; 4]>` avoids a heap allocation in the typical case
    /// of a manifold vertex with ≤ 4 incident edges.
    pub outgoing_half_edges: SmallVec<[HalfEdgeId; 4]>,
}

impl Vertex {
    pub fn new(position: Point3, tolerance: f64) -> Self {
        Self { position, tolerance, outgoing_half_edges: SmallVec::new() }
    }
}

/// One directed half of an undirected edge.
///
/// A full edge is represented by two opposing `HalfEdge`s connected via `twin`.
/// Each `HalfEdge` belongs to exactly one face [`Loop`].
#[derive(Clone, Debug)]
pub struct HalfEdge {
    /// The vertex this half-edge starts from.
    pub origin: VertexId,
    /// The opposing half-edge (same edge, opposite direction).
    pub twin: HalfEdgeId,
    /// The next half-edge around the same loop (counter-clockwise on the face).
    pub next: HalfEdgeId,
    /// The previous half-edge around the same loop.
    pub prev: HalfEdgeId,
    /// The loop (face boundary) this half-edge belongs to.
    pub loop_id: LoopId,
    /// The full undirected edge.
    pub edge: EdgeId,
    /// Parameter-space curve on the adjacent face's surface.
    /// Required for all non-degenerate edges on curved faces.
    pub pcurve: Option<Pcurve>,
}

/// An undirected topological edge.  Stores the 3-D geometry binding.
#[derive(Clone, Debug)]
pub struct Edge {
    /// The two half-edges: [forward, reverse].
    pub half_edges: [HalfEdgeId; 2],
    /// The 3-D curve geometry, if the edge is not degenerate.
    pub curve: Option<CurveBinding>,
    /// Per-edge tolerance.
    pub tolerance: f64,
    /// True for zero-length edges (e.g. cone apex, sphere poles).
    pub is_degenerate: bool,
}

impl Edge {
    pub fn new(forward: HalfEdgeId, reverse: HalfEdgeId) -> Self {
        Self {
            half_edges: [forward, reverse],
            curve: None,
            tolerance: 0.0,
            is_degenerate: false,
        }
    }

    /// Return the half-edge that is NOT `he`.
    pub fn other_half_edge(&self, he: HalfEdgeId) -> Option<HalfEdgeId> {
        if self.half_edges[0] == he {
            Some(self.half_edges[1])
        } else if self.half_edges[1] == he {
            Some(self.half_edges[0])
        } else {
            None
        }
    }
}

/// An ordered, closed ring of half-edges bounding part of a face.
///
/// A face has exactly one outer loop and zero or more inner loops (holes).
#[derive(Clone, Debug)]
pub struct Loop {
    /// First half-edge in the loop (traverse via `HalfEdge::next`).
    pub first_half_edge: HalfEdgeId,
    /// Whether this is the outer boundary or an inner hole.
    pub loop_kind: LoopKind,
    /// The face this loop belongs to.
    pub face: FaceId,
}

/// A topological face.  Geometry: a surface + orientation.
#[derive(Clone, Debug)]
pub struct Face {
    /// The outer (boundary) loop.
    pub outer_loop: LoopId,
    /// Inner loops (holes).  Usually empty; `SmallVec` avoids allocation.
    pub inner_loops: SmallVec<[LoopId; 2]>,
    /// Surface geometry and orientation.
    pub surface: SurfaceBinding,
    /// Whether the face normal agrees with the surface normal.
    pub orientation: Orientation,
    /// The shell this face belongs to.
    pub shell: ShellId,
    /// Per-face tolerance.
    pub tolerance: f64,
}

/// A connected set of faces, open or closed.
#[derive(Clone, Debug)]
pub struct Shell {
    pub faces: Vec<FaceId>,
    /// True when the shell forms a closed (watertight) boundary.
    pub is_closed: bool,
}

impl Shell {
    pub fn new() -> Self {
        Self { faces: Vec::new(), is_closed: false }
    }
}

impl Default for Shell {
    fn default() -> Self {
        Self::new()
    }
}

/// A bounded region of space, defined by one outer shell and zero or more
/// inner shells (voids / cavities).
#[derive(Clone, Debug)]
pub struct Solid {
    pub outer_shell: ShellId,
    /// Inner shells represent through-holes or internal voids.
    pub inner_shells: Vec<ShellId>,
}

impl Solid {
    pub fn new(outer_shell: ShellId) -> Self {
        Self { outer_shell, inner_shells: Vec::new() }
    }
}

/// A heterogeneous collection of shapes.
#[derive(Clone, Debug)]
pub struct Compound {
    pub children: Vec<ShapeRef>,
}

impl Compound {
    pub fn new() -> Self {
        Self { children: Vec::new() }
    }
}

impl Default for Compound {
    fn default() -> Self {
        Self::new()
    }
}

/// A tagged union for referencing any topological entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShapeRef {
    Vertex(VertexId),
    Edge(EdgeId),
    Face(FaceId),
    Shell(ShellId),
    Solid(SolidId),
    Compound(CompoundId),
}
