//! Geometric constraint types for 2D sketches.
//!
//! This module exposes two constraint representations:
//!
//! - [`SketchConstraint`] — low-level, flat-index representation consumed
//!   directly by the solver.  Indices refer to positions in a flat `[u,v]`
//!   point array.
//!
//! - [`Constraint`] — high-level, typed representation used by [`crate::Sketch`].
//!   References profiles and points by their opaque [`crate::ProfileId`] /
//!   [`crate::PointId`] IDs.  The sketch translates these to `SketchConstraint`
//!   values internally before calling the solver.

use crate::profile::{PointId, ProfileId};

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

    /// A point must lie on the circle defined by `(center_u, center_v)` and `radius`.
    /// Equation: (u[pt] - cu)² + (v[pt] - cv)² = r²
    /// Used for "point on circle" and "point on arc" constraints.
    PointOnCircle { pt: usize, center_u: f64, center_v: f64, radius: f64 },
}

// ── High-level typed constraint ────────────────────────────────────────────────

/// A geometric constraint that references sketch elements by typed IDs.
///
/// Used with [`crate::Sketch`].  The sketch translates these to [`SketchConstraint`]
/// flat-index values before passing them to the solver.
///
/// # Segment indexing
///
/// For [`crate::profile::ProfileShape::Polyline`] profiles, segment `i` connects
/// `points[i]` to `points[(i+1) % n]`.  For open polylines, avoid referencing
/// the wrap-around segment `n-1`.  For arcs, the only valid segment is `0`
/// (connecting start to end endpoint).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    // ── Single-profile, segment-based ─────────────────────────────────────────

    /// Segment must be horizontal (zero vertical component).
    Horizontal { profile: ProfileId, seg: usize },
    /// Segment must be vertical (zero horizontal component).
    Vertical { profile: ProfileId, seg: usize },
    /// Segment must have a specific length.
    FixedLength { profile: ProfileId, seg: usize, value: f64 },
    /// Two segments must be parallel.
    Parallel { profile: ProfileId, seg_a: usize, seg_b: usize },
    /// Two segments must be perpendicular.
    Perpendicular { profile: ProfileId, seg_a: usize, seg_b: usize },
    /// Two segments must have equal length.
    EqualLength { profile: ProfileId, seg_a: usize, seg_b: usize },
    /// The angle between two segments must equal `degrees`.
    Angle { profile: ProfileId, seg_a: usize, seg_b: usize, degrees: f64 },

    // ── Single-profile, point-based ────────────────────────────────────────────

    /// Point must lie at the sketch origin (0, 0).
    PointOnOrigin { profile: ProfileId, pt: PointId },
    /// Point must lie on the sketch X-axis (v = 0).
    PointOnXAxis { profile: ProfileId, pt: PointId },
    /// Point must lie on the sketch Y-axis (u = 0).
    PointOnYAxis { profile: ProfileId, pt: PointId },
    /// Point must be at a fixed UV position.  Intended as a temporary drag-pin.
    PointFixed { profile: ProfileId, pt: PointId, u: f64, v: f64 },
    /// Two points in the same profile must be coincident.
    Coincident { profile: ProfileId, pt_a: PointId, pt_b: PointId },
    /// Point must lie on the infinite line through a segment.
    PointOnLine { profile: ProfileId, pt: PointId, seg: usize },
    /// Two points must be a specific distance apart.
    PointDistance { profile: ProfileId, pt_a: PointId, pt_b: PointId, value: f64 },
    /// Point must lie on the circle defined by `(center_u, center_v, radius)`.
    PointOnCircle { profile: ProfileId, pt: PointId, center_u: f64, center_v: f64, radius: f64 },

    // ── Two-point alignment (same or different profiles) ──────────────────────

    /// Two points must be horizontally aligned in the viewport.
    /// `(perp_u, perp_v)` is the viewport "up" direction projected onto the UV plane.
    /// Works for points from the same or different profiles.
    HorizontalPair {
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
        perp_u: f64, perp_v: f64,
    },
    /// Two points must be vertically aligned in the viewport.
    /// `(perp_u, perp_v)` is the viewport "right" direction projected onto the UV plane.
    /// Works for points from the same or different profiles.
    VerticalPair {
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
        perp_u: f64, perp_v: f64,
    },

    // ── Cross-profile, segment-based ──────────────────────────────────────────

    /// Two segments in different profiles must be parallel.
    CrossParallel {
        profile_a: ProfileId, seg_a: usize,
        profile_b: ProfileId, seg_b: usize,
    },
    /// Two segments in different profiles must be perpendicular.
    CrossPerpendicular {
        profile_a: ProfileId, seg_a: usize,
        profile_b: ProfileId, seg_b: usize,
    },
    /// Two segments in different profiles must have equal length.
    CrossEqualLength {
        profile_a: ProfileId, seg_a: usize,
        profile_b: ProfileId, seg_b: usize,
    },
    /// The angle between two segments in different profiles must equal `degrees`.
    CrossAngle {
        profile_a: ProfileId, seg_a: usize,
        profile_b: ProfileId, seg_b: usize,
        degrees: f64,
    },

    // ── Cross-profile point coincidence ──────────────────────────────────────

    /// Two points in different profiles must be coincident (same position).
    CrossCoincident {
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
    },

    // ── Symmetry ───────────────────────────────────────────────────────────────

    /// A point is equidistant (perpendicular distance) from two line segments.
    Symmetric {
        profile_seg_a: ProfileId, seg_a: usize,
        profile_seg_b: ProfileId, seg_b: usize,
        profile_pt: ProfileId, pt: PointId,
    },
    /// `dist(P_a, P_c) == dist(P_b, P_c)`.  `P_c` is the constrained point.
    SymmetricPoints {
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
        profile_c: ProfileId, pt_c: PointId,
    },
}

impl Constraint {
    /// Short human-readable label for this constraint type.
    pub fn label(&self) -> &'static str {
        match self {
            Constraint::Horizontal { .. }         => "Horizontal",
            Constraint::Vertical { .. }           => "Vertical",
            Constraint::FixedLength { .. }        => "Fixed length",
            Constraint::Parallel { .. }           => "Parallel",
            Constraint::Perpendicular { .. }      => "Perpendicular",
            Constraint::EqualLength { .. }        => "Equal length",
            Constraint::Angle { .. }              => "Angle",
            Constraint::PointOnOrigin { .. }      => "On origin",
            Constraint::PointOnXAxis { .. }       => "On X-axis",
            Constraint::PointOnYAxis { .. }       => "On Y-axis",
            Constraint::PointFixed { .. }         => "Fixed point",
            Constraint::Coincident { .. }         => "Coincident",
            Constraint::PointOnLine { .. }        => "On line",
            Constraint::PointDistance { .. }      => "Point distance",
            Constraint::PointOnCircle { .. }      => "On circle",
            Constraint::HorizontalPair { .. }     => "Horizontal pair",
            Constraint::VerticalPair { .. }       => "Vertical pair",
            Constraint::CrossParallel { .. }      => "Parallel",
            Constraint::CrossPerpendicular { .. } => "Perpendicular",
            Constraint::CrossEqualLength { .. }   => "Equal length",
            Constraint::CrossAngle { .. }         => "Angle",
            Constraint::CrossCoincident { .. }    => "Coincident",
            Constraint::Symmetric { .. }          => "Symmetric",
            Constraint::SymmetricPoints { .. }    => "Symmetric (points)",
        }
    }

    /// Whether this constraint spans two different profiles.
    pub fn is_cross_profile(&self) -> bool {
        match self {
            Constraint::CrossParallel { .. }
            | Constraint::CrossPerpendicular { .. }
            | Constraint::CrossEqualLength { .. }
            | Constraint::CrossAngle { .. }
            | Constraint::CrossCoincident { .. }
            | Constraint::Symmetric { .. }
            | Constraint::SymmetricPoints { .. } => true,
            Constraint::HorizontalPair { profile_a, profile_b, .. }
            | Constraint::VerticalPair  { profile_a, profile_b, .. } => profile_a != profile_b,
            _ => false,
        }
    }

    /// The single profile this constraint belongs to, if it is a single-profile constraint.
    /// Returns `None` for cross-profile constraints.
    pub fn single_profile(&self) -> Option<ProfileId> {
        match self {
            Constraint::Horizontal { profile, .. }
            | Constraint::Vertical { profile, .. }
            | Constraint::FixedLength { profile, .. }
            | Constraint::Parallel { profile, .. }
            | Constraint::Perpendicular { profile, .. }
            | Constraint::EqualLength { profile, .. }
            | Constraint::Angle { profile, .. }
            | Constraint::PointOnOrigin { profile, .. }
            | Constraint::PointOnXAxis { profile, .. }
            | Constraint::PointOnYAxis { profile, .. }
            | Constraint::PointFixed { profile, .. }
            | Constraint::Coincident { profile, .. }
            | Constraint::PointOnLine { profile, .. }
            | Constraint::PointDistance { profile, .. }
            | Constraint::PointOnCircle { profile, .. } => Some(*profile),
            Constraint::HorizontalPair { profile_a, profile_b, .. }
            | Constraint::VerticalPair  { profile_a, profile_b, .. }
                if profile_a == profile_b => Some(*profile_a),
            _ => None,
        }
    }

    /// All profile IDs referenced by this constraint (deduplicated).
    pub fn referenced_profiles(&self) -> impl Iterator<Item = ProfileId> {
        let mut v: Vec<ProfileId> = Vec::new();
        match self {
            Constraint::Horizontal { profile, .. }
            | Constraint::Vertical { profile, .. }
            | Constraint::FixedLength { profile, .. }
            | Constraint::Parallel { profile, .. }
            | Constraint::Perpendicular { profile, .. }
            | Constraint::EqualLength { profile, .. }
            | Constraint::Angle { profile, .. }
            | Constraint::PointOnOrigin { profile, .. }
            | Constraint::PointOnXAxis { profile, .. }
            | Constraint::PointOnYAxis { profile, .. }
            | Constraint::PointFixed { profile, .. }
            | Constraint::Coincident { profile, .. }
            | Constraint::PointOnLine { profile, .. }
            | Constraint::PointDistance { profile, .. }
            | Constraint::PointOnCircle { profile, .. } => v.push(*profile),

            Constraint::HorizontalPair { profile_a, profile_b, .. }
            | Constraint::VerticalPair  { profile_a, profile_b, .. }
            | Constraint::CrossParallel { profile_a, profile_b, .. }
            | Constraint::CrossPerpendicular { profile_a, profile_b, .. }
            | Constraint::CrossEqualLength { profile_a, profile_b, .. }
            | Constraint::CrossAngle { profile_a, profile_b, .. }
            | Constraint::CrossCoincident { profile_a, profile_b, .. } => {
                v.push(*profile_a);
                if profile_b != profile_a { v.push(*profile_b); }
            }

            Constraint::Symmetric { profile_seg_a, profile_seg_b, profile_pt, .. } => {
                v.push(*profile_seg_a);
                if profile_seg_b != profile_seg_a { v.push(*profile_seg_b); }
                if profile_pt != profile_seg_a && profile_pt != profile_seg_b {
                    v.push(*profile_pt);
                }
            }

            Constraint::SymmetricPoints { profile_a, profile_b, profile_c, .. } => {
                v.push(*profile_a);
                if profile_b != profile_a { v.push(*profile_b); }
                if profile_c != profile_a && profile_c != profile_b { v.push(*profile_c); }
            }
        }
        v.into_iter()
    }
}

/// Translate a single-profile [`Constraint`] for profile `pid` into the
/// equivalent flat-index [`SketchConstraint`].
///
/// Returns `None` if `c` is not a single-profile constraint for `pid`, or if
/// it is a cross-profile constraint.
pub(crate) fn to_sketch_constraint(c: &Constraint, pid: ProfileId) -> Option<SketchConstraint> {
    match c {
        Constraint::Horizontal { profile, seg } if *profile == pid =>
            Some(SketchConstraint::Horizontal { seg: *seg }),

        Constraint::Vertical { profile, seg } if *profile == pid =>
            Some(SketchConstraint::Vertical { seg: *seg }),

        Constraint::FixedLength { profile, seg, value } if *profile == pid =>
            Some(SketchConstraint::FixedLength { seg: *seg, value: *value }),

        Constraint::Parallel { profile, seg_a, seg_b } if *profile == pid =>
            Some(SketchConstraint::Parallel { seg_a: *seg_a, seg_b: *seg_b }),

        Constraint::Perpendicular { profile, seg_a, seg_b } if *profile == pid =>
            Some(SketchConstraint::Perpendicular { seg_a: *seg_a, seg_b: *seg_b }),

        Constraint::EqualLength { profile, seg_a, seg_b } if *profile == pid =>
            Some(SketchConstraint::EqualLength { seg_a: *seg_a, seg_b: *seg_b }),

        Constraint::Angle { profile, seg_a, seg_b, degrees } if *profile == pid =>
            Some(SketchConstraint::Angle { seg_a: *seg_a, seg_b: *seg_b, degrees: *degrees }),

        Constraint::PointOnOrigin { profile, pt } if *profile == pid =>
            Some(SketchConstraint::PointOnOrigin { pt: pt.0 }),

        Constraint::PointOnXAxis { profile, pt } if *profile == pid =>
            Some(SketchConstraint::PointOnXAxis { pt: pt.0 }),

        Constraint::PointOnYAxis { profile, pt } if *profile == pid =>
            Some(SketchConstraint::PointOnYAxis { pt: pt.0 }),

        Constraint::PointFixed { profile, pt, u, v } if *profile == pid =>
            Some(SketchConstraint::PointFixed { pt: pt.0, x: *u, y: *v }),

        Constraint::Coincident { profile, pt_a, pt_b } if *profile == pid =>
            Some(SketchConstraint::Coincident { pt_a: pt_a.0, pt_b: pt_b.0 }),

        Constraint::PointOnLine { profile, pt, seg } if *profile == pid =>
            Some(SketchConstraint::PointOnLine { pt: pt.0, seg: *seg }),

        Constraint::PointDistance { profile, pt_a, pt_b, value } if *profile == pid =>
            Some(SketchConstraint::PointDistance { pt_a: pt_a.0, pt_b: pt_b.0, value: *value }),

        Constraint::PointOnCircle { profile, pt, center_u, center_v, radius } if *profile == pid =>
            Some(SketchConstraint::PointOnCircle {
                pt: pt.0, center_u: *center_u, center_v: *center_v, radius: *radius,
            }),

        // HorizontalPair / VerticalPair within the same profile.
        Constraint::HorizontalPair { profile_a, pt_a, profile_b, pt_b, perp_u, perp_v }
            if *profile_a == pid && *profile_b == pid =>
            Some(SketchConstraint::HorizontalPair {
                pt_a: pt_a.0, pt_b: pt_b.0, perp_u: *perp_u, perp_v: *perp_v,
            }),

        Constraint::VerticalPair { profile_a, pt_a, profile_b, pt_b, perp_u, perp_v }
            if *profile_a == pid && *profile_b == pid =>
            Some(SketchConstraint::VerticalPair {
                pt_a: pt_a.0, pt_b: pt_b.0, perp_u: *perp_u, perp_v: *perp_v,
            }),

        _ => None,
    }
}
