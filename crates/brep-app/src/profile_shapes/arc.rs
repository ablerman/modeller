//! # Arc shape
//!
//! Conventions:
//! - `points[0]` = start endpoint
//! - `points[1]` = end endpoint
//! - `points[2]` = center (always on the perpendicular bisector of start→end)
//!
//! The center invariant is maintained on every drag by reprojecting via
//! [`project_center_to_arc_bisector`].

use brep_core::Point3;
use crate::editor::{plane_to_world, world_to_plane, SketchPlane};

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Render a committed arc using a tessellated polyline.  Falls back to a straight
/// line if `plane` is `None`.
pub(crate) fn draw(
    points: &[Point3],
    plane: Option<SketchPlane>,
    painter: &egui::Painter,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    // points[0] = start, points[1] = end_pt, points[2] = center.
    if let (Some(&start), Some(&end_pt), Some(&center)) =
        (points.get(0), points.get(1), points.get(2))
    {
        if let Some(plane) = plane {
            let pts = tessellate_arc_from_center(start, end_pt, center, plane);
            for w in pts.windows(2) {
                if let (Some(a), Some(b)) = (proj(w[0]), proj(w[1])) {
                    painter.line_segment([a, b], stroke);
                }
            }
        } else {
            // Fallback: straight line.
            if let (Some(a), Some(b)) = (proj(start), proj(end_pt)) {
                painter.line_segment([a, b], stroke);
            }
        }
        // Mark start, end, and center as dots.
        for pt in [start, end_pt, center] {
            if let Some(p) = proj(pt) {
                painter.circle_filled(p, 3.0, stroke.color);
            }
        }
    }
}

// ── Drag handling ─────────────────────────────────────────────────────────────

/// Apply a vertex drag: update `points` after dragging vertex `vi` to `cursor_pt`.
///
/// - `vi == 0` or `vi == 1` (endpoint): move the endpoint, then reproject center onto new bisector.
/// - `vi == 2` (center): translate the whole arc by the same delta.
pub(crate) fn apply_vertex_drag(
    points: &mut Vec<Point3>,
    vi: usize,
    cursor_pt: Point3,
    plane: Option<SketchPlane>,
) {
    match vi {
        0 | 1 => {
            if let Some(pt) = points.get_mut(vi) { *pt = cursor_pt; }
            if let (Some(&s), Some(&e), Some(&c), Some(plane)) =
                (points.get(0), points.get(1), points.get(2), plane)
            {
                let new_c = project_center_to_arc_bisector(s, e, c, plane);
                if let Some(center) = points.get_mut(2) { *center = new_c; }
            }
        }
        2 => {
            // Dragging center translates the whole arc.
            let old_c = points.get(2).copied();
            if let Some(old_c) = old_c {
                let delta = cursor_pt - old_c;
                for pt in points.iter_mut() { *pt += delta; }
            }
        }
        _ => {
            if let Some(pt) = points.get_mut(vi) { *pt = cursor_pt; }
        }
    }
}

/// Apply a curve-body drag: project the cursor onto the bisector before storing
/// as the center, so the arc doesn't flip when the cursor is off-bisector.
pub(crate) fn apply_curve_drag(
    points: &mut Vec<Point3>,
    cursor_pt: Point3,
    plane: Option<SketchPlane>,
) {
    if let (Some(&s), Some(&e), Some(plane)) = (points.get(0), points.get(1), plane) {
        let projected = project_center_to_arc_bisector(s, e, cursor_pt, plane);
        if let Some(pt) = points.get_mut(2) { *pt = projected; }
    }
}

// ── Hit-testing ───────────────────────────────────────────────────────────────

/// Test whether the cursor (screen pixels) is within `threshold_px` of the arc curve.
pub(crate) fn hit_test_curve(
    points: &[Point3],
    plane: Option<SketchPlane>,
    cursor: (f32, f32),
    threshold_px: f32,
    project: &impl Fn(Point3) -> Option<(f32, f32)>,
) -> bool {
    if let (Some(&start), Some(&end_pt), Some(&center), Some(plane)) =
        (points.get(0), points.get(1), points.get(2), plane)
    {
        let pts = tessellate_arc_from_center(start, end_pt, center, plane);
        for pair in pts.windows(2) {
            if let (Some((ax, ay)), Some((bx, by))) = (project(pair[0]), project(pair[1])) {
                let seg_dx = bx - ax;
                let seg_dy = by - ay;
                let seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy;
                let dist = if seg_len2 < 1e-6 {
                    ((cursor.0 - ax).powi(2) + (cursor.1 - ay).powi(2)).sqrt()
                } else {
                    let t = ((cursor.0 - ax) * seg_dx + (cursor.1 - ay) * seg_dy) / seg_len2;
                    let t = t.clamp(0.0, 1.0);
                    let px = ax + t * seg_dx;
                    let py = ay + t * seg_dy;
                    ((cursor.0 - px).powi(2) + (cursor.1 - py).powi(2)).sqrt()
                };
                if dist < threshold_px { return true; }
            }
        }
    }
    false
}

// ── Constraint ────────────────────────────────────────────────────────────────

/// Build a `PointOnCircle` constraint for `pt_index` lying on this arc.
/// Arc radius = dist(center, start).
pub(crate) fn point_on_curve_constraint(
    points: &[brep_core::Point3],
    pt_index: usize,
    plane: SketchPlane,
) -> Option<brep_sketch::SketchConstraint> {
    // Arc stored as [start, end_pt, center].
    let start  = *points.get(0)?;
    let center = *points.get(2)?;
    let (u, v) = plane.uv_axes();
    let to_uv = |p: brep_core::Point3| (p.coords.dot(&u), p.coords.dot(&v));
    let (su, sv) = to_uv(start);
    let (cu, cv) = to_uv(center);
    let radius = ((su - cu).powi(2) + (sv - cv).powi(2)).sqrt();
    Some(brep_sketch::SketchConstraint::PointOnCircle { pt: pt_index, center_u: cu, center_v: cv, radius })
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Circumscribed circle through three 2D points.
/// Returns `None` if the points are collinear.
#[allow(dead_code)]
fn circumscribed_circle_2d(
    a: (f64, f64), b: (f64, f64), c: (f64, f64),
) -> Option<((f64, f64), f64)> {
    let n_ab = (-(b.1 - a.1), b.0 - a.0);
    let n_bc = (-(c.1 - b.1), c.0 - b.0);
    let m_ab = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
    let m_bc = ((b.0 + c.0) * 0.5, (b.1 + c.1) * 0.5);
    let dx = m_bc.0 - m_ab.0;
    let dy = m_bc.1 - m_ab.1;
    let det = -n_ab.0 * n_bc.1 + n_bc.0 * n_ab.1;
    if det.abs() < 1e-10 { return None; }
    let t = (-dx * n_bc.1 + n_bc.0 * dy) / det;
    let cx = m_ab.0 + t * n_ab.0;
    let cy = m_ab.1 + t * n_ab.1;
    let r = ((cx - a.0).powi(2) + (cy - a.1).powi(2)).sqrt();
    Some(((cx, cy), r))
}

/// Tessellate a circular arc through `start`, `through`, `end` into world-space points.
/// Returns `None` if the three points are collinear (degenerate arc).
/// The returned slice includes both `start` and `end`.
#[allow(dead_code)]
pub(crate) fn tessellate_arc(
    start: Point3, through: Point3, end: Point3, plane: SketchPlane,
) -> Option<Vec<Point3>> {
    use std::f64::consts::TAU;
    let a = world_to_plane(start,   plane);
    let b = world_to_plane(through, plane);
    let c = world_to_plane(end,     plane);
    let ((cx, cy), r) = circumscribed_circle_2d(a, b, c)?;

    let theta0 = (a.1 - cy).atan2(a.0 - cx);
    let theta1 = (b.1 - cy).atan2(b.0 - cx);
    let theta2 = (c.1 - cy).atan2(c.0 - cx);

    let norm = |ang: f64| (ang % TAU + TAU) % TAU;
    let r1 = norm(theta1 - theta0);
    let r2 = norm(theta2 - theta0);
    // Span: positive = CCW (r1 < r2), negative = CW.
    let span = if r1 < r2 { r2 } else { r2 - TAU };

    // Scale segment count with arc length; minimum 4 segments.
    let n = (32.0 * span.abs() / TAU).ceil().max(4.0) as usize;

    let pts = (0..=n).map(|i| {
        let theta = theta0 + span * (i as f64 / n as f64);
        plane_to_world(cx + r * theta.cos(), cy + r * theta.sin(), plane)
    }).collect();
    Some(pts)
}

/// Project a raw center click onto the perpendicular bisector of (start, end_pt)
/// so that the resulting center is equidistant from both endpoints.
/// The signed distance from the chord midpoint along the bisector is preserved,
/// so the caller's intent (which side to bow toward) is kept.
pub(crate) fn project_center_to_arc_bisector(
    start: Point3, end_pt: Point3, center: Point3, plane: SketchPlane,
) -> Point3 {
    let s = world_to_plane(start,  plane);
    let e = world_to_plane(end_pt, plane);
    let c = world_to_plane(center, plane);
    let mx = (s.0 + e.0) * 0.5;
    let my = (s.1 + e.1) * 0.5;
    let chord_dx = e.0 - s.0;
    let chord_dy = e.1 - s.1;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt();
    if chord_len < 1e-10 { return center; }
    let px = -chord_dy / chord_len;
    let py =  chord_dx / chord_len;
    let t = (c.0 - mx) * px + (c.1 - my) * py;
    plane_to_world(mx + t * px, my + t * py, plane)
}

/// Tessellate a circular arc defined by start, end, and a center hint.
/// The center is projected onto the perpendicular bisector of (start, end_pt)
/// so the arc passes through both endpoints exactly.
/// The arc sweeps the shorter path (|span| ≤ π).
pub(crate) fn tessellate_arc_from_center(
    start: Point3, end_pt: Point3, center: Point3, plane: SketchPlane,
) -> Vec<Point3> {
    use std::f64::consts::PI;
    let s = world_to_plane(start,  plane);
    let e = world_to_plane(end_pt, plane);

    // Project the center onto the perpendicular bisector so both endpoints
    // are exactly at `radius` from `c`, regardless of where the click landed.
    let mx = (s.0 + e.0) * 0.5;
    let my = (s.1 + e.1) * 0.5;
    let chord_dx = e.0 - s.0;
    let chord_dy = e.1 - s.1;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt();
    let c_raw = world_to_plane(center, plane);
    let c = if chord_len < 1e-10 {
        c_raw
    } else {
        let px = -chord_dy / chord_len;
        let py =  chord_dx / chord_len;
        let t = (c_raw.0 - mx) * px + (c_raw.1 - my) * py;
        (mx + t * px, my + t * py)
    };

    let radius = ((s.0 - c.0).powi(2) + (s.1 - c.1).powi(2)).sqrt();
    if radius < 1e-10 { return vec![start, end_pt]; }

    let theta_s = (s.1 - c.1).atan2(s.0 - c.0);
    let theta_e = (e.1 - c.1).atan2(e.0 - c.0);

    // Sweep the shorter arc: normalize delta to (−π, π].
    let mut span = theta_e - theta_s;
    span = span.rem_euclid(2.0 * PI);
    if span > PI { span -= 2.0 * PI; }

    let n = ((32.0 * span.abs() / (2.0 * PI)).ceil() as usize).max(4);
    // Force exact endpoints to avoid any floating-point drift.
    let mut pts: Vec<Point3> = (0..=n).map(|i| {
        let theta = theta_s + span * (i as f64 / n as f64);
        plane_to_world(c.0 + radius * theta.cos(), c.1 + radius * theta.sin(), plane)
    }).collect();
    if let Some(first) = pts.first_mut() { *first = start; }
    if let Some(last)  = pts.last_mut()  { *last  = end_pt; }
    pts
}
