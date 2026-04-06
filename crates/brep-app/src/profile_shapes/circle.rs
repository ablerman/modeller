//! # Circle shape
//!
//! Conventions:
//! - `points[0]` = center
//! - `points[1]` = a point on the circumference (defines the radius)

use brep_core::Point3;
use crate::editor::{plane_to_world, world_to_plane, SketchPlane};

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Render a committed circle.
pub(crate) fn draw(
    points: &[Point3],
    painter: &egui::Painter,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    // points[0] = center, points[1] = boundary point.
    if let (Some(&center), Some(&boundary)) = (points.get(0), points.get(1)) {
        if let (Some(c_screen), Some(b_screen)) = (proj(center), proj(boundary)) {
            let radius = (b_screen - c_screen).length();
            painter.circle_stroke(c_screen, radius, stroke);
            // Small dots at center and boundary — always dim so they don't look "selected".
            let dot_col = egui::Color32::from_rgba_unmultiplied(180, 180, 180, 120);
            painter.circle_filled(c_screen, 2.5, dot_col);
            painter.circle_filled(b_screen, 2.5, dot_col);
        }
    }
}

// ── Drag handling ─────────────────────────────────────────────────────────────

/// Apply a vertex drag: update `points` after dragging vertex `vi` to `cursor_pt`.
///
/// - `vi == 0` (center): translate the whole circle (center + boundary) by the same delta.
/// - other: move that point freely.
pub(crate) fn apply_vertex_drag(
    points: &mut Vec<Point3>,
    vi: usize,
    cursor_pt: Point3,
) {
    if vi == 0 {
        // Dragging the center moves the whole circle.
        let old_center = points.get(0).copied();
        if let (Some(old_c), Some(boundary)) = (old_center, points.get_mut(1)) {
            let delta = cursor_pt - old_c;
            *boundary += delta;
        }
        if let Some(c) = points.get_mut(0) { *c = cursor_pt; }
    } else if let Some(pt) = points.get_mut(vi) {
        *pt = cursor_pt;
    }
}

/// Apply a curve-body drag: move the boundary point (points[1]) to resize the circle.
pub(crate) fn apply_curve_drag(points: &mut Vec<Point3>, cursor_pt: Point3) {
    if let Some(pt) = points.get_mut(1) { *pt = cursor_pt; }
}

// ── Hit-testing ───────────────────────────────────────────────────────────────

/// Test whether the cursor (screen pixels) is within `threshold_px` of the circle curve.
pub(crate) fn hit_test_curve(
    points: &[Point3],
    cursor: (f32, f32),
    threshold_px: f32,
    project: &impl Fn(Point3) -> Option<(f32, f32)>,
) -> bool {
    if let (Some(&center), Some(&boundary)) = (points.get(0), points.get(1)) {
        if let (Some((cx, cy)), Some((bx, by))) = (project(center), project(boundary)) {
            let radius = ((bx - cx).powi(2) + (by - cy).powi(2)).sqrt();
            let dist_to_center = ((cursor.0 - cx).powi(2) + (cursor.1 - cy).powi(2)).sqrt();
            return (dist_to_center - radius).abs() < threshold_px;
        }
    }
    false
}

// ── Constraint ────────────────────────────────────────────────────────────────

/// Build a `PointOnCircle` constraint for `pt_index` lying on this circle.
/// Circle radius = dist(center, boundary).
pub(crate) fn point_on_curve_constraint(
    points: &[brep_core::Point3],
    pt_index: usize,
    plane: SketchPlane,
) -> Option<brep_sketch::SketchConstraint> {
    // Circle stored as [center, boundary].
    let center   = *points.get(0)?;
    let boundary = *points.get(1)?;
    let (u, v) = plane.uv_axes();
    let to_uv = |p: brep_core::Point3| (p.coords.dot(&u), p.coords.dot(&v));
    let (cu, cv) = to_uv(center);
    let (bu, bv) = to_uv(boundary);
    let radius = ((bu - cu).powi(2) + (bv - cv).powi(2)).sqrt();
    Some(brep_sketch::SketchConstraint::PointOnCircle { pt: pt_index, center_u: cu, center_v: cv, radius })
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Tessellate a full circle (center → radius point) into `n` world-space points.
/// The points form a closed polygon when connected back to the first.
pub(crate) fn tessellate_circle(
    center: Point3, radius_pt: Point3, plane: SketchPlane, n: usize,
) -> Vec<Point3> {
    use std::f64::consts::TAU;
    let c = world_to_plane(center, plane);
    let rp = world_to_plane(radius_pt, plane);
    let r = ((rp.0 - c.0).powi(2) + (rp.1 - c.1).powi(2)).sqrt();
    let theta0 = (rp.1 - c.1).atan2(rp.0 - c.0); // start angle at radius_pt
    (0..n).map(|i| {
        let theta = theta0 + TAU * i as f64 / n as f64;
        plane_to_world(c.0 + r * theta.cos(), c.1 + r * theta.sin(), plane)
    }).collect()
}
