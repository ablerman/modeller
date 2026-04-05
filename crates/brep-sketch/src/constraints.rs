//! Geometric constraint types for 2D sketches.

/// A geometric constraint between sketch elements.
///
/// Points are referenced by index into the vertex array.
/// Segments are referenced by index: segment `i` connects `points[i]`
/// to `points[(i + 1) % n]`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SketchConstraint {
    /// Two segments must be parallel (cross product of directions = 0).
    Parallel { seg_a: usize, seg_b: usize },

    /// Two segments must be perpendicular (dot product of directions = 0).
    Perpendicular { seg_a: usize, seg_b: usize },

    /// The angle between two segments must equal `degrees`.
    /// The angle is measured as the absolute angle between the direction
    /// vectors (0°–180°).
    Angle { seg_a: usize, seg_b: usize, degrees: f64 },

    /// A segment must be horizontal (its direction vector has zero vertical component).
    Horizontal { seg: usize },

    /// A segment must be vertical (its direction vector has zero horizontal component).
    Vertical { seg: usize },

    /// Two segments must have equal length.
    EqualLength { seg_a: usize, seg_b: usize },

    /// Two points must be coincident (same position).
    Coincident { pt_a: usize, pt_b: usize },

    /// A point must lie on the infinite line through a segment.
    /// Equation: (u[pt]-u[a])*(v[b]-v[a]) - (v[pt]-v[a])*(u[b]-u[a]) = 0
    PointOnLine { pt: usize, seg: usize },

    /// A segment must have a specific length.
    FixedLength { seg: usize, value: f64 },

    /// Two (possibly non-adjacent) points must be a specific distance apart.
    PointDistance { pt_a: usize, pt_b: usize, value: f64 },

    /// A point must be at a specific (u, v) position.
    /// Intended for use as a temporary drag-pin constraint, not for persistent storage.
    PointFixed { pt: usize, x: f64, y: f64 },

    /// A point must be coincident with the sketch origin (0, 0).
    PointOnOrigin { pt: usize },
    /// A point must lie on the sketch X-axis (v = 0).
    PointOnXAxis { pt: usize },
    /// A point must lie on the sketch Y-axis (u = 0).
    PointOnYAxis { pt: usize },

    /// Two points must be aligned horizontally in the viewport.
    /// Constraint: `perp_u*(u[b]-u[a]) + perp_v*(v[b]-v[a]) = 0`
    /// where `(perp_u, perp_v)` is the viewport "up" direction projected onto the sketch UV plane.
    HorizontalPair { pt_a: usize, pt_b: usize, perp_u: f64, perp_v: f64 },

    /// Two points must be aligned vertically in the viewport.
    /// Constraint: `perp_u*(u[b]-u[a]) + perp_v*(v[b]-v[a]) = 0`
    /// where `(perp_u, perp_v)` is the viewport "right" direction projected onto the sketch UV plane.
    VerticalPair { pt_a: usize, pt_b: usize, perp_u: f64, perp_v: f64 },
}
