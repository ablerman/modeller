//! # Rectangle tool
//!
//! The Rectangle tool places an axis-aligned rectangle in two clicks: one corner then the
//! diagonally opposite corner. The four vertices are computed in the sketch plane's UV
//! coordinate system, guaranteeing the edges are orthogonal within that plane.
//!
//! ## Drawing (2-click sequence)
//! 1. **Click 1 — first corner**: stores `ToolInProgress::RectFirst { corner }`.
//! 2. **Click 2 — opposite corner**: computes the four corners via `rect_corners` and commits
//!    the result to `sk.committed_profiles` as a closed `ProfileShape::Polyline` with
//!    `closed: true`. `tool_in_progress` is cleared.
//!
//! Rectangles never appear in `sk.points`; they go directly to `sk.committed_profiles`.
//!
//! ## Preview
//! After click 1 a four-sided outline is drawn from the placed corner to the current cursor
//! position on every frame, giving real-time feedback on the rectangle's size and position.
//!
//! ## Commit
//! On click 2 a `CommittedProfile` is pushed:
//! ```text
//! CommittedProfile { points: [c0, c1, c2, c3], shape: Polyline, closed: true, plane: None }
//! ```
//! The four corners are in winding order within the sketch plane. A history snapshot is saved
//! before the commit so the operation is undoable.
//!
//! ## Drag (committed rectangle)
//! Once committed, individual corner vertices can be dragged with the Pointer tool to reshape
//! the profile. After dragging a corner there is no constraint enforcement — the profile
//! becomes a free quadrilateral. Future constraint support (right-angle, equal-length) is
//! tracked separately.
//!
//! ## Undo
//! `SketchUndoPoint` steps back through `ToolInProgress`:
//! - `RectFirst { .. }` → clears `tool_in_progress` (back to idle, no geometry added).
//! - `None` → removes the last committed profile.

use brep_core::Point3;
use crate::editor::{
    CommittedProfile, ProfileShape, SketchState, ToolInProgress,
    push_to_global, rect_corners, sketch_snapshot,
};

/// Advance the rectangle state machine by one click at point `p`.
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    match sk.tool_in_progress.take() {
        None => {
            sk.tool_in_progress = Some(ToolInProgress::RectFirst { corner: p });
        }
        Some(ToolInProgress::RectFirst { corner }) => {
            let corners = rect_corners(corner, p, sk.plane);
            sk.history.push("Draw rectangle", sketch_snapshot(sk));
            let point_indices = push_to_global(&corners, &mut sk.global_points);
            sk.committed_profiles.push(CommittedProfile {
                points:        Vec::new(),
                point_indices,
                closed:        true,
                shape:         ProfileShape::Polyline,
                plane:         None,
                constraints:   Vec::new(),
                arc_reversed:  false,
            });
        }
        _ => {}
    }
}

/// Draw a rectangle preview from `corner` to the diagonally opposite `cursor` corner.
pub(crate) fn draw_preview(
    painter: &egui::Painter,
    corner: Point3, cursor: Point3,
    plane: crate::editor::SketchPlane,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    let corners = rect_corners(corner, cursor, plane);
    for i in 0..4 {
        if let (Some(a), Some(b)) = (proj(corners[i]), proj(corners[(i + 1) % 4])) {
            painter.line_segment([a, b], stroke);
        }
    }
}
