//! Geometric constraint types for 2D sketches.

/// A geometric constraint between sketch elements.
///
/// Points are referenced by index into the vertex array.
/// Segments are referenced by index: segment `i` connects `points[i]`
/// to `points[(i + 1) % n]`.
#[derive(Clone, Debug, PartialEq)]
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
}
