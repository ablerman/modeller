//! Profile types and ID newtypes.

// ── Typed IDs ──────────────────────────────────────────────────────────────────

/// Opaque identifier for a profile within a [`Sketch`](crate::Sketch).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash,
         serde::Serialize, serde::Deserialize)]
pub struct ProfileId(pub usize);

/// Opaque identifier for a point within a specific profile.
/// Indices are local to the profile; they correspond directly to the
/// position in the profile's internal point array.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash,
         serde::Serialize, serde::Deserialize)]
pub struct PointId(pub usize);

/// Opaque identifier for a constraint within a [`Sketch`](crate::Sketch).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash,
         serde::Serialize, serde::Deserialize)]
pub struct ConstraintId(pub usize);

/// Identifier for a shared (globally merged) point.
///
/// Multiple profiles can reference the same `GlobalPointId`, making
/// their corresponding vertices structurally coincident — no explicit
/// `Coincident` constraint is needed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash,
         serde::Serialize, serde::Deserialize)]
pub struct GlobalPointId(pub usize);

// ── Profile shape ──────────────────────────────────────────────────────────────

/// How a profile's point list is interpreted geometrically.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ProfileShape {
    /// Straight-segment polygon.
    ///
    /// - `points[i]` is vertex `i`.
    /// - Segment `i` connects `points[i]` to `points[(i+1) % n]`.
    /// - `closed == true`: the last segment wraps back to `points[0]`.
    /// - `closed == false`: there are `n-1` segments; avoid constraining
    ///   the "virtual" wrap-around segment `n-1`.
    Polyline { closed: bool },

    /// Full circle.
    ///
    /// - `points[0]` = center.
    /// - `points[1]` = a point on the circumference (defines the radius).
    Circle,

    /// Circular arc.
    ///
    /// - `points[0]` = start endpoint.
    /// - `points[1]` = end endpoint.
    /// - `points[2]` = center (always on the perpendicular bisector of
    ///   the `start→end` chord; maintained by
    ///   [`Sketch::reproject_arc_center`](crate::Sketch::reproject_arc_center)).
    Arc,
}

impl Default for ProfileShape {
    fn default() -> Self { ProfileShape::Polyline { closed: false } }
}

// ── Profile ────────────────────────────────────────────────────────────────────

/// A single geometric sub-profile within a sketch.
///
/// Points are stored as `[u, v]` coordinates in sketch-plane space.
/// Constraints that apply to this profile are stored on the parent
/// [`Sketch`](crate::Sketch) and reference this profile by [`ProfileId`].
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Profile {
    /// UV coordinates of the profile's control points.
    /// The geometric meaning of each index is determined by `shape`.
    pub(crate) points: Vec<[f64; 2]>,

    /// For each point in `points`, the optional global-point identity.
    /// Two profiles sharing the same `GlobalPointId` at a position are
    /// structurally coincident.
    pub(crate) global_ids: Vec<Option<GlobalPointId>>,

    /// How the points are interpreted geometrically.
    pub shape: ProfileShape,
}

impl Profile {
    pub(crate) fn new(shape: ProfileShape) -> Self {
        Self { points: Vec::new(), global_ids: Vec::new(), shape }
    }

    /// Number of control points.
    pub fn point_count(&self) -> usize { self.points.len() }

    /// UV coordinates of point `pt`, or `None` if out of range.
    pub fn point(&self, pt: PointId) -> Option<(f64, f64)> {
        self.points.get(pt.0).map(|&[u, v]| (u, v))
    }

    /// The global-point identity for local point `pt`, if any.
    pub fn global_id(&self, pt: PointId) -> Option<GlobalPointId> {
        self.global_ids.get(pt.0).copied().flatten()
    }

    /// Iterator over all local point IDs in insertion order.
    pub fn point_ids(&self) -> impl Iterator<Item = PointId> + '_ {
        (0..self.points.len()).map(PointId)
    }

    /// Number of valid segments for this profile.
    pub fn segment_count(&self) -> usize {
        let n = self.points.len();
        match &self.shape {
            ProfileShape::Polyline { closed: true }  => n,
            ProfileShape::Polyline { closed: false } => n.saturating_sub(1),
            ProfileShape::Arc    => 1,
            ProfileShape::Circle => 0,
        }
    }

    /// For segment `seg`, return the indices of its start and end points
    /// within this profile's point array.  Returns `None` if `seg` is out
    /// of range.
    pub fn segment_point_indices(&self, seg: usize) -> Option<(usize, usize)> {
        let n = self.points.len();
        if n == 0 { return None; }
        match &self.shape {
            ProfileShape::Polyline { closed: true } => {
                if seg >= n { return None; }
                Some((seg, (seg + 1) % n))
            }
            ProfileShape::Polyline { closed: false } => {
                if seg + 1 >= n { return None; }
                Some((seg, seg + 1))
            }
            ProfileShape::Arc => {
                if seg == 0 && n >= 2 { Some((0, 1)) } else { None }
            }
            ProfileShape::Circle => None,
        }
    }
}
