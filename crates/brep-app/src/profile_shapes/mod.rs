//! Profile shape types — one sub-module per shape.
//!
//! Each sub-module is the single source of truth for its shape's conventions:
//! what `points[]` indices mean, how the shape is rendered, how hit-testing works,
//! how control points are dragged, and what constraints it supports.

pub mod arc;
pub mod circle;
pub mod polyline;

pub(crate) use arc::{project_center_to_arc_bisector, tessellate_arc_from_center};
pub(crate) use circle::tessellate_circle;
pub(crate) use polyline::rect_corners;

/// How the points of a [`crate::editor::CommittedProfile`] should be interpreted geometrically.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize, PartialEq)]
pub enum ProfileShape {
    /// Straight-line polygon.  Points are vertices; `closed` determines whether the last
    /// point connects back to the first.
    #[default]
    Polyline,
    /// Full circle.  `points[0]` = center, `points[1]` = a point on the circumference
    /// (defines the radius).
    Circle,
    /// Circular arc.  `points[0]` = start endpoint, `points[1]` = end endpoint,
    /// `points[2]` = center (always on the perpendicular bisector of start→end).
    Arc,
}

impl ProfileShape {
    /// Display label for the profile panel (e.g. "Circle 0", "Arc 1", "Rectangle 2").
    pub fn label(&self, index: usize) -> String {
        match self {
            ProfileShape::Circle   => format!("Circle {index}"),
            ProfileShape::Arc      => format!("Arc {index}"),
            ProfileShape::Polyline => format!("Rectangle {index}"),
        }
    }

    /// Whether this shape supports the PointOnCircle / PointOnCurve constraint.
    pub fn supports_point_on_curve(&self) -> bool {
        matches!(self, ProfileShape::Circle | ProfileShape::Arc)
    }

    /// Whether control point `vi` is individually selectable for constraints.
    /// Arc: all three points (start, end, center) are individually selectable.
    /// Other shapes use whole-profile selection.
    pub fn vertex_selectable(&self, _vi: usize) -> bool {
        matches!(self, ProfileShape::Arc)
    }

    /// Apply a vertex drag: mutate `points` after dragging vertex `vi` to `cursor_pt`.
    pub fn apply_vertex_drag(
        &self,
        points: &mut Vec<brep_core::Point3>,
        vi: usize,
        cursor_pt: brep_core::Point3,
        plane: Option<crate::editor::SketchPlane>,
    ) {
        match self {
            ProfileShape::Circle   => circle::apply_vertex_drag(points, vi, cursor_pt),
            ProfileShape::Arc      => arc::apply_vertex_drag(points, vi, cursor_pt, plane),
            ProfileShape::Polyline => polyline::apply_vertex_drag(points, vi, cursor_pt),
        }
    }

    /// Apply a curve-body drag: mutate `points` in response to dragging the curve itself.
    pub fn apply_curve_drag(
        &self,
        points: &mut Vec<brep_core::Point3>,
        cursor_pt: brep_core::Point3,
        plane: Option<crate::editor::SketchPlane>,
    ) {
        match self {
            ProfileShape::Circle   => circle::apply_curve_drag(points, cursor_pt),
            ProfileShape::Arc      => arc::apply_curve_drag(points, cursor_pt, plane),
            ProfileShape::Polyline => {}
        }
    }

    /// Test whether the cursor (screen pixels) is within `threshold_px` of this shape's curve.
    /// Polylines are not tested here (they use vertex snapping instead).
    pub fn hit_test_curve(
        &self,
        points: &[brep_core::Point3],
        plane: Option<crate::editor::SketchPlane>,
        cursor: (f32, f32),
        threshold_px: f32,
        project: &impl Fn(brep_core::Point3) -> Option<(f32, f32)>,
    ) -> bool {
        match self {
            ProfileShape::Circle   => circle::hit_test_curve(points, cursor, threshold_px, project),
            ProfileShape::Arc      => arc::hit_test_curve(points, plane, cursor, threshold_px, project),
            ProfileShape::Polyline => false,
        }
    }

    /// Build the PointOnCircle constraint for `pt_index` lying on this shape.
    /// Returns `None` if the shape doesn't support this constraint or points are missing.
    pub fn point_on_curve_constraint(
        &self,
        points: &[brep_core::Point3],
        pt_index: usize,
        plane: crate::editor::SketchPlane,
    ) -> Option<brep_sketch::SketchConstraint> {
        match self {
            ProfileShape::Circle   => circle::point_on_curve_constraint(points, pt_index, plane),
            ProfileShape::Arc      => arc::point_on_curve_constraint(points, pt_index, plane),
            ProfileShape::Polyline => None,
        }
    }

    /// Render this committed profile onto the painter.
    pub fn draw(
        &self,
        points: &[brep_core::Point3],
        closed: bool,
        plane: Option<crate::editor::SketchPlane>,
        painter: &egui::Painter,
        proj: &impl Fn(brep_core::Point3) -> Option<egui::Pos2>,
        stroke: egui::Stroke,
    ) {
        match self {
            ProfileShape::Circle   => circle::draw(points, painter, proj, stroke),
            ProfileShape::Arc      => arc::draw(points, plane, painter, proj, stroke),
            ProfileShape::Polyline => polyline::draw(points, closed, painter, proj, stroke),
        }
    }
}
