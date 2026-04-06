//! # Polyline shape
//!
//! Conventions:
//! - `points[0..n]` = vertices in order
//! - `closed` flag on the profile determines whether the last vertex connects back to the first

use brep_core::Point3;
use crate::editor::{plane_to_world, world_to_plane, SketchPlane};

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Render a committed polyline.
pub(crate) fn draw(
    points: &[Point3],
    closed: bool,
    painter: &egui::Painter,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    let cn = points.len();
    let seg_count = if closed { cn } else { cn.saturating_sub(1) };
    for i in 0..seg_count {
        if let (Some(a), Some(b)) = (proj(points[i]), proj(points[(i + 1) % cn])) {
            painter.line_segment([a, b], stroke);
        }
    }
    // Corner dots.
    for &pt in points {
        if let Some(p) = proj(pt) {
            painter.circle_filled(p, 3.0, stroke.color);
        }
    }
}

// ── Drag handling ─────────────────────────────────────────────────────────────

/// Apply a vertex drag: move the single vertex `vi` to `cursor_pt`.
pub(crate) fn apply_vertex_drag(points: &mut Vec<Point3>, vi: usize, cursor_pt: Point3) {
    if let Some(pt) = points.get_mut(vi) { *pt = cursor_pt; }
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Compute the four corners of an axis-aligned (in sketch-plane space) rectangle
/// given two opposite corners in world space.
pub(crate) fn rect_corners(c1: Point3, c2: Point3, plane: SketchPlane) -> [Point3; 4] {
    let (u1, v1) = world_to_plane(c1, plane);
    let (u2, v2) = world_to_plane(c2, plane);
    [
        plane_to_world(u1, v1, plane),
        plane_to_world(u2, v1, plane),
        plane_to_world(u2, v2, plane),
        plane_to_world(u1, v2, plane),
    ]
}
