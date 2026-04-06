//! # Polyline tool
//!
//! The Polyline tool builds an open or closed polygon by placing vertices one click at a time.
//! It is the only tool that accumulates points directly in `sk.points`; all other multi-step
//! tools (Arc, Rectangle, Circle) bypass `sk.points` and commit directly to
//! `sk.committed_profiles`.
//!
//! ## Drawing
//! Each left-click appends a new vertex to `sk.points` and immediately re-solves geometric
//! constraints via `apply_constraints`. There is no `ToolInProgress` state — the tool is always
//! "ready" for the next click.
//!
//! ## Closing the loop
//! When the active profile has ≥ 3 points and is not yet closed, clicking back on vertex 0
//! triggers `SketchCloseLoop` rather than adding a duplicate point. The first vertex is rendered
//! with a distinct orange hint circle while this is possible, giving the user a visual cue.
//! Alternatively the user can press Enter or click the "Close" toolbar button.
//!
//! ## Preview
//! A single line segment is drawn from the last placed vertex to the current cursor position
//! on every frame, showing where the next segment will go. This is rendered inline in `ui.rs`
//! (inside the `None` arm of the `tool_in_progress` match) rather than via a `draw_preview`
//! function.
//!
//! ## Dragging
//! While using the Polyline tool, dragging an existing vertex (snap highlight on `snap_vertex`)
//! repositions it without adding a new point. On release with < 4 px movement:
//! - Clicking vertex 0 when ≥ 3 points exist → `SketchCloseLoop`.
//! - Clicking any other vertex → `SketchSelectVertex(vi)`.
//!
//! ## Undo
//! `SketchUndoPoint` (Backspace) removes the last placed vertex from `sk.points`. If the
//! profile is closed it first opens it (removes the `closed` flag); further backspaces remove
//! vertices one at a time.

use brep_core::Point3;
use crate::editor::{SketchState, apply_constraints, sketch_snapshot};

/// Append `p` to the active profile and re-solve constraints.
///
/// A snapshot is pushed to history before the point is added so the operation can be undone
/// with `SketchUndoPoint`.
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    sk.history.push(sketch_snapshot(sk));
    sk.points.push(p);
    apply_constraints(sk);
}
