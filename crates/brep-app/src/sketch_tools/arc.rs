//! # Arc tool
//!
//! The Arc tool places a circular arc in three clicks: start point, end point, then a nominal
//! center that is projected onto the perpendicular bisector of start→end to guarantee a
//! geometrically valid arc.
//!
//! ## Drawing (3-click sequence)
//! 1. **Click 1 — start point**: stores `ToolInProgress::Arc1 { start }`.
//! 2. **Click 2 — end point**: advances to `ToolInProgress::Arc2 { start, end_pt }`.
//! 3. **Click 3 — center**: the clicked point is projected onto the perpendicular bisector of
//!    the start→end chord via `project_center_to_arc_bisector`. The arc is then committed to
//!    `sk.committed_profiles` with `shape: ProfileShape::Arc` and
//!    `points: [start, end_pt, center]`. `tool_in_progress` is cleared.
//!
//! Arcs never appear in `sk.points`; they go directly to `sk.committed_profiles`.
//!
//! ## Preview
//! - **After click 1**: a green dot marks the start point; a straight line is drawn from it to
//!   the cursor (= future end point).
//! - **After click 2**: a tessellated arc is drawn from `start` to `end_pt` with the cursor
//!   treated as the candidate center. A small crosshair marks the projected center position.
//!   The end point is also marked with a green dot.
//!
//! ## Commit
//! On click 3 a `CommittedProfile` is pushed:
//! ```text
//! CommittedProfile { points: [start, end_pt, center], shape: Arc, closed: false, plane: Some(sk.plane) }
//! ```
//! A history snapshot is saved before the commit so the operation is undoable.
//!
//! ## Drag (committed arc)
//! Once committed, each of the three stored points can be dragged with the Pointer tool:
//! - **`points[0]` (start)** or **`points[1]` (end)**: repositioned freely; the center is
//!   re-projected onto the new bisector on every frame to keep the arc valid.
//! - **`points[2]` (center)**: repositioned along the perpendicular bisector of start→end.
//!
//! ## Undo
//! `SketchUndoPoint` steps back through `ToolInProgress`:
//! - `Arc2 { start, .. }` → reverts to `Arc1 { start }` (forgets the end point).
//! - `Arc1 { .. }` → clears `tool_in_progress` (back to idle).
//! - `None` → removes the last committed profile.

use brep_core::Point3;
use crate::editor::{
    CommittedProfile, ProfileShape, SketchState, ToolInProgress,
    project_center_to_arc_bisector, push_to_global, sketch_snapshot, tessellate_arc_from_center,
};

/// Advance the arc state machine by one click at point `p`.
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    match sk.tool_in_progress.take() {
        None => {
            sk.tool_in_progress = Some(ToolInProgress::Arc1 { start: p });
        }
        Some(ToolInProgress::Arc1 { start }) => {
            sk.tool_in_progress = Some(ToolInProgress::Arc2 { start, end_pt: p });
        }
        Some(ToolInProgress::Arc2 { start, end_pt }) => {
            let center = project_center_to_arc_bisector(start, end_pt, p, sk.plane);
            sk.history.push("Draw arc", sketch_snapshot(sk));
            let point_indices = push_to_global(&[start, end_pt, center], &mut sk.global_points);
            sk.committed_profiles.push(CommittedProfile {
                points:        Vec::new(),
                point_indices,
                closed:        false,
                shape:         ProfileShape::Arc,
                plane:         Some(sk.plane),
                constraints:   Vec::new(),
                arc_reversed:  false,
            });
        }
        _ => {}
    }
}

/// Draw a tessellated arc preview from `start` to `end_pt` using `center` as the arc center.
///
/// Also marks `center` with a small crosshair so the user can see where it will be projected.
pub(crate) fn draw_preview(
    painter: &egui::Painter,
    start: Point3, end_pt: Point3, center: Point3,
    plane: crate::editor::SketchPlane,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    let pts = tessellate_arc_from_center(start, end_pt, center, plane, false);
    for w in pts.windows(2) {
        if let (Some(a), Some(b)) = (proj(w[0]), proj(w[1])) {
            painter.line_segment([a, b], stroke);
        }
    }
    if let Some(cp) = proj(center) {
        painter.line_segment(
            [cp - egui::vec2(4.0, 0.0), cp + egui::vec2(4.0, 0.0)],
            egui::Stroke::new(1.0, stroke.color),
        );
        painter.line_segment(
            [cp - egui::vec2(0.0, 4.0), cp + egui::vec2(0.0, 4.0)],
            egui::Stroke::new(1.0, stroke.color),
        );
    }
}
