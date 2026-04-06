//! # Pointer tool (Select / Move)
//!
//! The Pointer tool is the default non-drawing mode. It does not place geometry; instead it
//! selects and repositions existing sketch elements.
//!
//! ## Drawing
//! The Pointer tool never generates new geometry. Switching to it via the toolbar immediately
//! cancels any in-progress multi-step operation: `tool_in_progress` is set to `None` and the
//! active point list is cleared (see `UiAction::SketchSetTool` in `editor.rs`).
//!
//! ## Selection
//! All selection logic lives in `main.rs` (click detection) and `editor.rs` (action dispatch).
//! The rules are:
//!
//! - **Left-click a vertex** of the active profile → `SketchSelectVertex(vi)`; the vertex is
//!   highlighted and can be used as a constraint target.
//! - **Left-click a committed profile edge / curve body** → `SketchSelectCommitted(Some(pi))`;
//!   the whole profile is highlighted.
//! - **Left-click any arc control point** (`ProfileShape::Arc`; vi=0 start, vi=1 end, vi=2
//!   center) → `SketchSelectCommittedPoint(Some((pi, vi)))`; selects the point independently.
//!   Combine with an active-profile vertex selection (via `SketchSelectVertex`) to enable the
//!   Coincident constraint button — this pins the active vertex at the arc point's current
//!   position via `PointFixed`. The center point is also drag-reshapeable as before.
//! - **Left-click a reference line** → `SketchSelectRef(r)`.
//! - **Left-click a constraint marker** → `SketchSelectConstraint(ci)`.
//! - **Left-click empty space** → `SketchSelectCommitted(None)`, clearing the committed
//!   selection.
//!
//! ## Dragging
//! Drag detection uses a 4 px movement threshold. If the mouse travels more than 4 px before
//! release the event is treated as a drag rather than a click.
//!
//! - **Drag an active-profile vertex**: repositions `sk.points[vi]` live on every mouse-move
//!   frame. The vertex snaps to other vertices while dragging. Release does nothing extra.
//! - **Drag a committed profile vertex**: repositions `sk.committed_profiles[pi].points[vi]`
//!   live each frame. For arc profiles the center (`vi == 2`) is re-projected onto the
//!   perpendicular bisector of start→end on every frame to keep the arc geometrically valid.
//! - **Drag a committed curve body** (circle or arc, not a control point): moves the profile's
//!   boundary point (`points[1]`) which effectively changes the radius / arc endpoint.
//!
//! ## Preview
//! None. The Pointer tool draws no preview geometry.
