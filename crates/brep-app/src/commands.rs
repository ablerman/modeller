//! Command registry: string-identified commands that map to UiActions.
#![allow(dead_code)] // public scripting API — not all items are called yet
//!
//! Each command has a stable string ID (e.g. `"sketch.tool_arc"`) that can be
//! used for keybinding maps, a command palette, or future scripting.
//!
//! UI-modal actions (`SketchBeginAngleInput`, `SketchBeginLengthInput`, etc.)
//! are intentionally absent from this registry — they modify `AppState` in
//! `main.rs` rather than `EditorState`, and are not meaningful scripting
//! targets.  They are still emitted as `UiAction` variants by `Dynamic`
//! action sources and intercepted in `AppState::dispatch_action`.

use brep_bool::BooleanKind;
use brep_sketch::SketchConstraint;

use crate::editor::{DrawTool, EditorState, LengthTarget, ProfileShape, PrimitiveKind, RefEntity, SceneEntry, SketchPlane, UiAction};
use crate::icons;

// ── CommandId ─────────────────────────────────────────────────────────────────

/// Stable string identifier for a user-invocable command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CommandId(pub &'static str);

/// All known command IDs as constants.
pub mod cmd {
    use super::CommandId;
    pub const NEW:                CommandId = CommandId("file.new");
    pub const OPEN:               CommandId = CommandId("file.open");
    pub const SAVE:               CommandId = CommandId("file.save");
    pub const SAVE_AS:            CommandId = CommandId("file.save_as");
    pub const UNDO:               CommandId = CommandId("edit.undo");
    pub const REDO:               CommandId = CommandId("edit.redo");
    pub const DELETE_SELECTED:    CommandId = CommandId("edit.delete_selected");
    pub const ADD_BOX:            CommandId = CommandId("primitive.add_box");
    pub const ADD_CYLINDER:       CommandId = CommandId("primitive.add_cylinder");
    pub const ADD_SPHERE:         CommandId = CommandId("primitive.add_sphere");
    pub const ADD_CONE:           CommandId = CommandId("primitive.add_cone");
    pub const BOOL_UNION:         CommandId = CommandId("boolean.union");
    pub const BOOL_DIFFERENCE:    CommandId = CommandId("boolean.difference");
    pub const BOOL_INTERSECT:     CommandId = CommandId("boolean.intersection");
    pub const SKETCH_XY:          CommandId = CommandId("sketch.enter_xy");
    pub const SKETCH_XZ:          CommandId = CommandId("sketch.enter_xz");
    pub const SKETCH_YZ:          CommandId = CommandId("sketch.enter_yz");
    pub const SKETCH_FINISH:      CommandId = CommandId("sketch.finish");
    pub const SKETCH_CANCEL:      CommandId = CommandId("sketch.cancel");
    pub const SKETCH_UNDO_POINT:  CommandId = CommandId("sketch.undo_point");
    pub const SKETCH_UNDO:        CommandId = CommandId("sketch.undo");
    pub const SKETCH_REDO:        CommandId = CommandId("sketch.redo");
    pub const TOOL_POINTER:       CommandId = CommandId("sketch.tool_pointer");
    pub const TOOL_POLYLINE:      CommandId = CommandId("sketch.tool_polyline");
    pub const TOOL_ARC:           CommandId = CommandId("sketch.tool_arc");
    pub const TOOL_RECT:          CommandId = CommandId("sketch.tool_rect");
    pub const TOOL_CIRCLE:        CommandId = CommandId("sketch.tool_circle");
    pub const CONSTRAIN_H:        CommandId = CommandId("sketch.constrain_horizontal");
    pub const CONSTRAIN_V:        CommandId = CommandId("sketch.constrain_vertical");
    pub const CONSTRAIN_PARALLEL: CommandId = CommandId("sketch.constrain_parallel");
    pub const CONSTRAIN_PERP:     CommandId = CommandId("sketch.constrain_perpendicular");
    pub const CONSTRAIN_EQUAL_LEN:CommandId = CommandId("sketch.constrain_equal_length");
    pub const CONSTRAIN_ANGLE:    CommandId = CommandId("sketch.constrain_angle");
    pub const CONSTRAIN_LENGTH:   CommandId = CommandId("sketch.constrain_length");
    pub const CONSTRAIN_COINCIDENT:CommandId = CommandId("sketch.constrain_coincident");
}

// ── ActionSource ──────────────────────────────────────────────────────────────

/// How a command produces `UiAction`s when invoked.
pub enum ActionSource {
    /// Emits exactly this action.  Used for simple, stateless commands.
    Fixed(UiAction),
    /// Computes actions from current editor state.  Used for commands whose
    /// payload depends on the active selection (e.g. constraint buttons).
    Dynamic(fn(&EditorState) -> Vec<UiAction>),
}

// ── CommandSpec ───────────────────────────────────────────────────────────────

/// The static description of one command.
pub struct CommandSpec {
    pub id:      CommandId,
    pub label:   &'static str,
    pub tooltip: &'static str,
    /// Icon function for toolbar/menu display.  `None` = text-only.
    pub icon:    Option<fn() -> egui::ImageSource<'static>>,
    pub action:  ActionSource,
}

// ── Dynamic action helpers ────────────────────────────────────────────────────

fn h_constraints(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    let ((h_pu, h_pv), _) = ed.camera.sketch_align_dirs(sk.plane);
    let n = sk.points.len();
    let n_pts = sk.pt_selection.len();
    let ref_sel = sk.ref_selection;
    if !sk.seg_selection.is_empty() {
        sk.seg_selection.iter().map(|&s| UiAction::SketchAddConstraint(SketchConstraint::HorizontalPair {
            pt_a: s, pt_b: (s + 1) % n, perp_u: h_pu, perp_v: h_pv,
        })).collect()
    } else if n_pts == 2 {
        vec![UiAction::SketchAddConstraint(SketchConstraint::HorizontalPair {
            pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1], perp_u: h_pu, perp_v: h_pv,
        })]
    } else if n_pts == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::XAxis)) {
        vec![UiAction::SketchAddConstraint(SketchConstraint::PointOnXAxis { pt: sk.pt_selection[0] })]
    } else {
        vec![]
    }
}

fn v_constraints(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    let (_, (v_pu, v_pv)) = ed.camera.sketch_align_dirs(sk.plane);
    let n = sk.points.len();
    let n_pts = sk.pt_selection.len();
    let ref_sel = sk.ref_selection;
    if !sk.seg_selection.is_empty() {
        sk.seg_selection.iter().map(|&s| UiAction::SketchAddConstraint(SketchConstraint::VerticalPair {
            pt_a: s, pt_b: (s + 1) % n, perp_u: v_pu, perp_v: v_pv,
        })).collect()
    } else if n_pts == 2 {
        vec![UiAction::SketchAddConstraint(SketchConstraint::VerticalPair {
            pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1], perp_u: v_pu, perp_v: v_pv,
        })]
    } else if n_pts == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::YAxis)) {
        vec![UiAction::SketchAddConstraint(SketchConstraint::PointOnYAxis { pt: sk.pt_selection[0] })]
    } else {
        vec![]
    }
}

fn parallel_constraint(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    if sk.seg_selection.len() < 2 { return vec![] }
    vec![UiAction::SketchAddConstraint(SketchConstraint::Parallel {
        seg_a: sk.seg_selection[0],
        seg_b: sk.seg_selection[1],
    })]
}

fn perp_constraint(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    if sk.seg_selection.len() < 2 { return vec![] }
    vec![UiAction::SketchAddConstraint(SketchConstraint::Perpendicular {
        seg_a: sk.seg_selection[0],
        seg_b: sk.seg_selection[1],
    })]
}

fn equal_len_constraint(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    if sk.seg_selection.len() < 2 { return vec![] }
    (1..sk.seg_selection.len()).map(|i| UiAction::SketchAddConstraint(SketchConstraint::EqualLength {
        seg_a: sk.seg_selection[0],
        seg_b: sk.seg_selection[i],
    })).collect()
}

fn angle_input(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    if sk.seg_selection.len() < 2 { return vec![] }
    vec![UiAction::SketchBeginAngleInput {
        seg_a: sk.seg_selection[0],
        seg_b: sk.seg_selection[1],
    }]
}

fn length_input(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    let n_sel = sk.seg_selection.len();
    let n_pts = sk.pt_selection.len();
    let target = if n_sel >= 1 {
        LengthTarget::Segment(sk.seg_selection[0])
    } else if n_pts == 2 {
        LengthTarget::Points(sk.pt_selection[0], sk.pt_selection[1])
    } else {
        return vec![]
    };
    vec![UiAction::SketchBeginLengthInput(target)]
}

fn coincident_constraint(ed: &EditorState) -> Vec<UiAction> {
    let Some(sk) = &ed.sketch else { return vec![] };
    let n_sel = sk.seg_selection.len();
    let n_pts = sk.pt_selection.len();
    let c = if n_pts == 2 && n_sel == 0 {
        SketchConstraint::Coincident { pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1] }
    } else if n_sel == 1 && n_pts == 1 {
        SketchConstraint::PointOnLine { pt: sk.pt_selection[0], seg: sk.seg_selection[0] }
    } else if n_pts == 1 && sk.ref_selection == Some(RefEntity::Origin) {
        SketchConstraint::PointOnOrigin { pt: sk.pt_selection[0] }
    } else if n_pts == 1 && n_sel == 0 {
        // Point on committed arc or circle.
        let constraint = (|| -> Option<SketchConstraint> {
            let ci = sk.committed_selection?;
            let cp = sk.committed_profiles.get(ci)?;
            let (u, v) = sk.plane.uv_axes();
            let to_uv = |p: brep_core::Point3| (p.coords.dot(&u), p.coords.dot(&v));
            match &cp.shape {
                ProfileShape::Circle => {
                    let center   = *cp.points.get(0)?;
                    let boundary = *cp.points.get(1)?;
                    let (cu, cv) = to_uv(center);
                    let (bu, bv) = to_uv(boundary);
                    let radius = ((bu - cu).powi(2) + (bv - cv).powi(2)).sqrt();
                    Some(SketchConstraint::PointOnCircle {
                        pt: sk.pt_selection[0], center_u: cu, center_v: cv, radius,
                    })
                }
                ProfileShape::Arc => {
                    // Arc stored as [start, end_pt, center]; radius = dist(center, start)
                    let start  = *cp.points.get(0)?;
                    let center = *cp.points.get(2)?;
                    let (su, sv) = to_uv(start);
                    let (cu, cv) = to_uv(center);
                    let radius = ((su - cu).powi(2) + (sv - cv).powi(2)).sqrt();
                    Some(SketchConstraint::PointOnCircle {
                        pt: sk.pt_selection[0], center_u: cu, center_v: cv, radius,
                    })
                }
                _ => None,
            }
        })();
        match constraint {
            Some(c) => c,
            None => return vec![],
        }
    } else {
        return vec![]
    };
    vec![UiAction::SketchAddConstraint(c)]
}

// ── Command specs ─────────────────────────────────────────────────────────────

pub static SPEC_NEW: CommandSpec = CommandSpec {
    id: cmd::NEW, label: "New", tooltip: "New file",
    icon: None, action: ActionSource::Fixed(UiAction::New),
};
pub static SPEC_OPEN: CommandSpec = CommandSpec {
    id: cmd::OPEN, label: "Open…", tooltip: "Open file",
    icon: None, action: ActionSource::Fixed(UiAction::Open),
};
pub static SPEC_SAVE: CommandSpec = CommandSpec {
    id: cmd::SAVE, label: "Save", tooltip: "Save file",
    icon: None, action: ActionSource::Fixed(UiAction::Save),
};
pub static SPEC_SAVE_AS: CommandSpec = CommandSpec {
    id: cmd::SAVE_AS, label: "Save As…", tooltip: "Save file as",
    icon: None, action: ActionSource::Fixed(UiAction::SaveAs),
};
pub static SPEC_UNDO: CommandSpec = CommandSpec {
    id: cmd::UNDO, label: "Undo", tooltip: "Undo",
    icon: Some(icons::icon_undo), action: ActionSource::Fixed(UiAction::Undo),
};
pub static SPEC_REDO: CommandSpec = CommandSpec {
    id: cmd::REDO, label: "Redo", tooltip: "Redo",
    icon: Some(icons::icon_redo), action: ActionSource::Fixed(UiAction::Redo),
};
pub static SPEC_DELETE_SELECTED: CommandSpec = CommandSpec {
    id: cmd::DELETE_SELECTED, label: "Delete", tooltip: "Delete selected",
    icon: None, action: ActionSource::Fixed(UiAction::DeleteSelected),
};
pub static SPEC_ADD_BOX: CommandSpec = CommandSpec {
    id: cmd::ADD_BOX, label: "☐  Box", tooltip: "Add box",
    icon: None, action: ActionSource::Fixed(UiAction::AddPrimitive(PrimitiveKind::Box)),
};
pub static SPEC_ADD_CYLINDER: CommandSpec = CommandSpec {
    id: cmd::ADD_CYLINDER, label: "◎  Cylinder", tooltip: "Add cylinder",
    icon: None, action: ActionSource::Fixed(UiAction::AddPrimitive(PrimitiveKind::Cylinder)),
};
pub static SPEC_ADD_SPHERE: CommandSpec = CommandSpec {
    id: cmd::ADD_SPHERE, label: "◉  Sphere", tooltip: "Add sphere",
    icon: None, action: ActionSource::Fixed(UiAction::AddPrimitive(PrimitiveKind::Sphere)),
};
pub static SPEC_ADD_CONE: CommandSpec = CommandSpec {
    id: cmd::ADD_CONE, label: "△  Cone", tooltip: "Add cone",
    icon: None, action: ActionSource::Fixed(UiAction::AddPrimitive(PrimitiveKind::Cone)),
};
pub static SPEC_BOOL_UNION: CommandSpec = CommandSpec {
    id: cmd::BOOL_UNION, label: "⊕  Union", tooltip: "Boolean union",
    icon: None, action: ActionSource::Fixed(UiAction::BooleanOp(BooleanKind::Union)),
};
pub static SPEC_BOOL_DIFFERENCE: CommandSpec = CommandSpec {
    id: cmd::BOOL_DIFFERENCE, label: "⊖  Difference", tooltip: "Boolean difference",
    icon: None, action: ActionSource::Fixed(UiAction::BooleanOp(BooleanKind::Difference)),
};
pub static SPEC_BOOL_INTERSECT: CommandSpec = CommandSpec {
    id: cmd::BOOL_INTERSECT, label: "⊗  Intersection", tooltip: "Boolean intersection",
    icon: None, action: ActionSource::Fixed(UiAction::BooleanOp(BooleanKind::Intersection)),
};
pub static SPEC_SKETCH_XY: CommandSpec = CommandSpec {
    id: cmd::SKETCH_XY, label: "XY Plane", tooltip: "Enter sketch on XY plane",
    icon: None, action: ActionSource::Fixed(UiAction::EnterSketch(SketchPlane::XY)),
};
pub static SPEC_SKETCH_XZ: CommandSpec = CommandSpec {
    id: cmd::SKETCH_XZ, label: "XZ Plane", tooltip: "Enter sketch on XZ plane",
    icon: None, action: ActionSource::Fixed(UiAction::EnterSketch(SketchPlane::XZ)),
};
pub static SPEC_SKETCH_YZ: CommandSpec = CommandSpec {
    id: cmd::SKETCH_YZ, label: "YZ Plane", tooltip: "Enter sketch on YZ plane",
    icon: None, action: ActionSource::Fixed(UiAction::EnterSketch(SketchPlane::YZ)),
};
pub static SPEC_SKETCH_FINISH: CommandSpec = CommandSpec {
    id: cmd::SKETCH_FINISH, label: "Finish", tooltip: "Finish sketch",
    icon: None, action: ActionSource::Fixed(UiAction::SketchFinish),
};
pub static SPEC_SKETCH_CANCEL: CommandSpec = CommandSpec {
    id: cmd::SKETCH_CANCEL, label: "✕ Cancel", tooltip: "Cancel sketch",
    icon: None, action: ActionSource::Fixed(UiAction::ExitSketch),
};
pub static SPEC_SKETCH_UNDO_POINT: CommandSpec = CommandSpec {
    id: cmd::SKETCH_UNDO_POINT, label: "Undo", tooltip: "Remove last drawn point",
    icon: None, action: ActionSource::Fixed(UiAction::SketchUndoPoint),
};
pub static SPEC_SKETCH_UNDO: CommandSpec = CommandSpec {
    id: cmd::SKETCH_UNDO, label: "Undo", tooltip: "Undo",
    icon: Some(icons::icon_undo), action: ActionSource::Fixed(UiAction::Undo),
};
pub static SPEC_SKETCH_REDO: CommandSpec = CommandSpec {
    id: cmd::SKETCH_REDO, label: "Redo", tooltip: "Redo",
    icon: Some(icons::icon_redo), action: ActionSource::Fixed(UiAction::Redo),
};
pub static SPEC_TOOL_POINTER: CommandSpec = CommandSpec {
    id: cmd::TOOL_POINTER, label: "Select", tooltip: "Select/move tool",
    icon: Some(icons::icon_pointer), action: ActionSource::Fixed(UiAction::SketchSetTool(DrawTool::Pointer)),
};
pub static SPEC_TOOL_POLYLINE: CommandSpec = CommandSpec {
    id: cmd::TOOL_POLYLINE, label: "Polyline", tooltip: "Polyline tool",
    icon: Some(icons::icon_polyline), action: ActionSource::Fixed(UiAction::SketchSetTool(DrawTool::Polyline)),
};
pub static SPEC_TOOL_ARC: CommandSpec = CommandSpec {
    id: cmd::TOOL_ARC, label: "Arc", tooltip: "Arc tool",
    icon: Some(icons::icon_arc), action: ActionSource::Fixed(UiAction::SketchSetTool(DrawTool::Arc)),
};
pub static SPEC_TOOL_RECT: CommandSpec = CommandSpec {
    id: cmd::TOOL_RECT, label: "Rectangle", tooltip: "Rectangle tool",
    icon: Some(icons::icon_rect), action: ActionSource::Fixed(UiAction::SketchSetTool(DrawTool::Rectangle)),
};
pub static SPEC_TOOL_CIRCLE: CommandSpec = CommandSpec {
    id: cmd::TOOL_CIRCLE, label: "Circle", tooltip: "Circle tool",
    icon: Some(icons::icon_circle), action: ActionSource::Fixed(UiAction::SketchSetTool(DrawTool::Circle)),
};
pub static SPEC_CONSTRAIN_H: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_H, label: "Horizontal", tooltip: "Horizontal constraint",
    icon: Some(icons::icon_horizontal), action: ActionSource::Dynamic(h_constraints),
};
pub static SPEC_CONSTRAIN_V: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_V, label: "Vertical", tooltip: "Vertical constraint",
    icon: Some(icons::icon_vertical), action: ActionSource::Dynamic(v_constraints),
};
pub static SPEC_CONSTRAIN_PARALLEL: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_PARALLEL, label: "Parallel", tooltip: "Parallel constraint",
    icon: Some(icons::icon_parallel), action: ActionSource::Dynamic(parallel_constraint),
};
pub static SPEC_CONSTRAIN_PERP: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_PERP, label: "Perpendicular", tooltip: "Perpendicular constraint",
    icon: Some(icons::icon_perp), action: ActionSource::Dynamic(perp_constraint),
};
pub static SPEC_CONSTRAIN_EQUAL_LEN: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_EQUAL_LEN, label: "Equal Length", tooltip: "Equal length constraint",
    icon: Some(icons::icon_equal_len), action: ActionSource::Dynamic(equal_len_constraint),
};
pub static SPEC_CONSTRAIN_ANGLE: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_ANGLE, label: "Angle…", tooltip: "Angle constraint",
    icon: Some(icons::icon_angle), action: ActionSource::Dynamic(angle_input),
};
pub static SPEC_CONSTRAIN_LENGTH: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_LENGTH, label: "Length…", tooltip: "Length constraint",
    icon: Some(icons::icon_length), action: ActionSource::Dynamic(length_input),
};
pub static SPEC_CONSTRAIN_COINCIDENT: CommandSpec = CommandSpec {
    id: cmd::CONSTRAIN_COINCIDENT, label: "Coincident", tooltip: "Coincident constraint",
    icon: Some(icons::icon_coincident), action: ActionSource::Dynamic(coincident_constraint),
};

// ── Registry ──────────────────────────────────────────────────────────────────

/// All scriptable commands.  UI-modal actions are excluded by design.
pub static COMMAND_REGISTRY: &[&CommandSpec] = &[
    &SPEC_NEW, &SPEC_OPEN, &SPEC_SAVE, &SPEC_SAVE_AS,
    &SPEC_UNDO, &SPEC_REDO, &SPEC_DELETE_SELECTED,
    &SPEC_ADD_BOX, &SPEC_ADD_CYLINDER, &SPEC_ADD_SPHERE, &SPEC_ADD_CONE,
    &SPEC_BOOL_UNION, &SPEC_BOOL_DIFFERENCE, &SPEC_BOOL_INTERSECT,
    &SPEC_SKETCH_XY, &SPEC_SKETCH_XZ, &SPEC_SKETCH_YZ,
    &SPEC_SKETCH_FINISH, &SPEC_SKETCH_CANCEL, &SPEC_SKETCH_UNDO_POINT,
    &SPEC_SKETCH_UNDO, &SPEC_SKETCH_REDO,
    &SPEC_TOOL_POINTER, &SPEC_TOOL_POLYLINE, &SPEC_TOOL_ARC, &SPEC_TOOL_RECT, &SPEC_TOOL_CIRCLE,
    &SPEC_CONSTRAIN_H, &SPEC_CONSTRAIN_V,
    &SPEC_CONSTRAIN_PARALLEL, &SPEC_CONSTRAIN_PERP, &SPEC_CONSTRAIN_EQUAL_LEN,
    &SPEC_CONSTRAIN_ANGLE, &SPEC_CONSTRAIN_LENGTH, &SPEC_CONSTRAIN_COINCIDENT,
];

/// Look up a command by its string ID.
pub fn lookup_command(id: &str) -> Option<&'static CommandSpec> {
    COMMAND_REGISTRY.iter().copied().find(|s| s.id.0 == id)
}

/// Resolve a command to the `UiAction`s it produces given current editor state.
pub fn resolve(spec: &CommandSpec, editor: &EditorState) -> Vec<UiAction> {
    match &spec.action {
        ActionSource::Fixed(a)   => vec![a.clone()],
        ActionSource::Dynamic(f) => f(editor),
    }
}

// ── Enablement helpers (used by toolbar_defs) ─────────────────────────────────

pub fn h_applicable(ed: &EditorState) -> bool {
    let Some(sk) = &ed.sketch else { return false };
    let ref_sel = sk.ref_selection;
    !sk.seg_selection.is_empty()
        || sk.pt_selection.len() == 2
        || (sk.pt_selection.len() == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::XAxis)))
}

pub fn v_applicable(ed: &EditorState) -> bool {
    let Some(sk) = &ed.sketch else { return false };
    let ref_sel = sk.ref_selection;
    !sk.seg_selection.is_empty()
        || sk.pt_selection.len() == 2
        || (sk.pt_selection.len() == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::YAxis)))
}

pub fn two_segs(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.seg_selection.len() >= 2)
}

pub fn exactly_two_segs(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.seg_selection.len() == 2)
}

pub fn has_length_target(ed: &EditorState) -> bool {
    let Some(sk) = &ed.sketch else { return false };
    sk.seg_selection.len() >= 1 || sk.pt_selection.len() == 2
}

pub fn has_coincident_target(ed: &EditorState) -> bool {
    let Some(sk) = &ed.sketch else { return false };
    let n_sel = sk.seg_selection.len();
    let n_pts = sk.pt_selection.len();
    (n_pts == 2 && n_sel == 0)
        || (n_sel == 1 && n_pts == 1)
        || (n_pts == 1 && sk.ref_selection == Some(RefEntity::Origin))
        || (n_pts == 1 && n_sel == 0 && sk.committed_selection.map_or(false, |ci| {
            sk.committed_profiles.get(ci).map_or(false, |cp| {
                matches!(cp.shape, ProfileShape::Circle | ProfileShape::Arc)
            })
        }))
}

pub fn two_solids_selected(ed: &EditorState) -> bool {
    ed.selection.iter()
        .filter(|&&i| matches!(ed.entries.get(i), Some(SceneEntry::Solid(_))))
        .count() == 2
}

pub fn can_undo(ed: &EditorState) -> bool { ed.history.can_undo() }
pub fn can_redo(ed: &EditorState) -> bool { ed.history.can_redo() }
pub fn can_delete(ed: &EditorState) -> bool { !ed.selection.is_empty() }
pub fn in_sketch(ed: &EditorState) -> bool { ed.sketch.is_some() }
pub fn not_in_sketch(ed: &EditorState) -> bool { ed.sketch.is_none() }

pub fn sketch_can_undo(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.history.can_undo())
}
pub fn sketch_can_redo(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.history.can_redo())
}
pub fn sketch_can_finish(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| {
        sk.points.len() >= 3 || !sk.committed_profiles.is_empty()
    })
}

pub fn tool_is_pointer(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.active_tool == DrawTool::Pointer)
}
pub fn tool_is_polyline(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.active_tool == DrawTool::Polyline)
}
pub fn tool_is_arc(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.active_tool == DrawTool::Arc)
}
pub fn tool_is_rect(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.active_tool == DrawTool::Rectangle)
}
pub fn tool_is_circle(ed: &EditorState) -> bool {
    ed.sketch.as_ref().map_or(false, |sk| sk.active_tool == DrawTool::Circle)
}
