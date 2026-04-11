//! # Circle tool
//!
//! The Circle tool places a full circle in two clicks: center point then a point on the
//! circumference that implicitly defines the radius.
//!
//! ## Drawing (2-click sequence)
//! 1. **Click 1 — center**: stores `ToolInProgress::CircleCenter { center }`.
//! 2. **Click 2 — radius point**: commits the circle to `sk.committed_profiles` with
//!    `shape: ProfileShape::Circle` and `points: [center, radius_pt]`. `tool_in_progress` is
//!    cleared.
//!
//! Circles never appear in `sk.points`; they go directly to `sk.committed_profiles`.
//!
//! ## Preview
//! After click 1 a tessellated circle (64 segments) is drawn centred at the placed center,
//! passing through the current cursor position, updating every frame to show the live radius.
//!
//! ## Commit
//! On click 2 a `CommittedProfile` is pushed:
//! ```text
//! CommittedProfile { points: [center, radius_pt], shape: Circle, closed: true, plane: Some(sk.plane) }
//! ```
//! A history snapshot is saved before the commit so the operation is undoable.
//!
//! ## Drag (committed circle)
//! Once committed, the two stored points can be dragged with the Pointer tool:
//! - **`points[0]` (center)**: moves the entire circle.
//! - **`points[1]` (boundary / radius point)**: changes the radius while keeping the center
//!   fixed.
//!
//! ## Undo
//! `SketchUndoPoint` steps back through `ToolInProgress`:
//! - `CircleCenter { .. }` → clears `tool_in_progress` (back to idle, no geometry added).
//! - `None` → removes the last committed profile.

use brep_core::Point3;
use crate::editor::{
    CommittedProfile, ProfileShape, SketchState, ToolInProgress,
    push_to_global, sketch_snapshot, tessellate_circle,
};

/// Advance the circle state machine by one click at point `p`.
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    match sk.tool_in_progress.take() {
        None => {
            sk.tool_in_progress = Some(ToolInProgress::CircleCenter { center: p });
        }
        Some(ToolInProgress::CircleCenter { center }) => {
            sk.history.push("Draw circle", sketch_snapshot(sk));
            let point_indices = push_to_global(&[center, p], &mut sk.global_points);
            sk.committed_profiles.push(CommittedProfile {
                points:        Vec::new(),
                point_indices,
                closed:        true,
                shape:         ProfileShape::Circle,
                plane:         Some(sk.plane),
                constraints:   Vec::new(),
            });
        }
        _ => {}
    }
}

/// Draw a 64-segment circle preview centred at `center` passing through `radius_pt`.
pub(crate) fn draw_preview(
    painter: &egui::Painter,
    center: Point3, radius_pt: Point3,
    plane: crate::editor::SketchPlane,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    let pts = tessellate_circle(center, radius_pt, plane, 64);
    let n = pts.len();
    for i in 0..n {
        if let (Some(a), Some(b)) = (proj(pts[i]), proj(pts[(i + 1) % n])) {
            painter.line_segment([a, b], stroke);
        }
    }
}
