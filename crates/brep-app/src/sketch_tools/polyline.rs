//! # Polyline tool
//!
//! Each left-click places one point.  The *first* click stores the start position and enters
//! `ToolInProgress::PolylineChain`; every subsequent click commits a 2-point
//! `CommittedProfile` (one segment) whose start point is shared with the previous segment via
//! the `global_points` store.  Adjacent segments therefore share the exact same `Point3` index,
//! so they stay physically connected without requiring a separate Coincident constraint.
//!
//! ## Closing the loop
//! When ≥ 2 segments have been drawn the user can click back on the chain's first vertex
//! (detected in `main.rs` via `snap_committed`) to dispatch `SketchCloseLoop`, which commits
//! one final segment and sets `tool_in_progress = None`.
//!
//! ## Preview
//! A single line segment is drawn from `PolylineChain::pen_global_idx` to the cursor position
//! on every frame (rendered in `ui.rs`).
//!
//! ## Undo
//! `SketchUndoPoint` (Backspace) handled in `editor.rs`:
//! - `segment_count == 0`: remove the pen point from `global_points` and clear
//!   `tool_in_progress`.
//! - `segment_count > 0`: pop the last committed profile, pop the end global point, decrement
//!   `segment_count`, and reset `pen_global_idx` to the previous segment's start.

use brep_core::Point3;
use crate::editor::{
    CommittedProfile, ProfileShape, SketchState, ToolInProgress,
    apply_constraints, push_to_global, sketch_snapshot,
};

/// Handle a left-click at `p` while the Polyline tool is active.
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    match sk.tool_in_progress.take() {
        None => {
            // First click: push the start point into global_points and enter chain mode.
            let chain_start_global_pt_idx = sk.global_points.len();
            sk.global_points.push(p);
            let pen_global_idx = chain_start_global_pt_idx;
            sk.tool_in_progress = Some(ToolInProgress::PolylineChain {
                chain_start_profile: sk.committed_profiles.len(),
                chain_start_global_pt_idx,
                pen_global_idx,
                segment_count: 0,
            });
            // No history push — first click doesn't create a segment yet.
        }
        Some(ToolInProgress::PolylineChain {
            chain_start_profile,
            chain_start_global_pt_idx,
            pen_global_idx,
            segment_count,
        }) => {
            // Subsequent click: commit a 2-point segment sharing `pen_global_idx`.
            sk.history.push("Draw segment", sketch_snapshot(sk));
            let end_indices = push_to_global(&[p], &mut sk.global_points);
            let end_idx = end_indices[0];
            sk.committed_profiles.push(CommittedProfile {
                points:        Vec::new(),
                point_indices: vec![pen_global_idx, end_idx],
                closed:        false,
                shape:         ProfileShape::Polyline,
                plane:         Some(sk.plane),
                constraints:   Vec::new(),
            });
            sk.tool_in_progress = Some(ToolInProgress::PolylineChain {
                chain_start_profile,
                chain_start_global_pt_idx,
                pen_global_idx: end_idx,
                segment_count: segment_count + 1,
            });
            apply_constraints(sk);
        }
        other => {
            // Restore any other tool state we accidentally took.
            sk.tool_in_progress = other;
        }
    }
}
