pub mod arc;
pub mod circle;
pub mod pointer;
pub mod polyline;
pub mod rectangle;

use brep_core::Point3;
use crate::editor::{DrawTool, SketchState};

/// Dispatch a left-click point to the active tool's state machine.
///
/// Called from `editor::EditorState::apply` when processing `UiAction::SketchAddPoint`.
/// The Pointer tool is excluded upstream (it never generates SketchAddPoint).
pub(crate) fn add_point(sk: &mut SketchState, p: Point3) {
    match sk.active_tool {
        DrawTool::Pointer   => { /* handled in main.rs / editor.rs */ }
        DrawTool::Polyline  => polyline::add_point(sk, p),
        DrawTool::Arc       => arc::add_point(sk, p),
        DrawTool::Rectangle => rectangle::add_point(sk, p),
        DrawTool::Circle    => circle::add_point(sk, p),
    }
}
