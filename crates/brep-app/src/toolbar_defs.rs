//! Static toolbar layout definitions.
//!
//! Each constant is a slice of `ToolbarItemKind` items that describes one
//! toolbar surface.  The actual rendering is handled by `toolbar::render_toolbar`.

use crate::commands::{self, *};
use crate::icons;
use crate::toolbar::ToolbarItemKind;

// ── Sketch viewport toolbar (floating, icon-only) ─────────────────────────────

pub static SKETCH_VIEWPORT_TOOLBAR: &[ToolbarItemKind] = &[
    // Section 1: Undo / Redo
    ToolbarItemKind::Button {
        spec: &SPEC_SKETCH_UNDO,
        is_active: None,
        is_enabled: Some(commands::sketch_can_undo),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_SKETCH_REDO,
        is_active: None,
        is_enabled: Some(commands::sketch_can_redo),
    },
    ToolbarItemKind::Separator,

    // Section 2: Drawing tools (toggle buttons)
    ToolbarItemKind::Button {
        spec: &SPEC_TOOL_POINTER,
        is_active: Some(commands::tool_is_pointer),
        is_enabled: None,
    },
    ToolbarItemKind::Button {
        spec: &SPEC_TOOL_POLYLINE,
        is_active: Some(commands::tool_is_polyline),
        is_enabled: None,
    },
    ToolbarItemKind::Button {
        spec: &SPEC_TOOL_ARC,
        is_active: Some(commands::tool_is_arc),
        is_enabled: None,
    },
    ToolbarItemKind::Button {
        spec: &SPEC_TOOL_RECT,
        is_active: Some(commands::tool_is_rect),
        is_enabled: None,
    },
    ToolbarItemKind::Button {
        spec: &SPEC_TOOL_CIRCLE,
        is_active: Some(commands::tool_is_circle),
        is_enabled: None,
    },
    ToolbarItemKind::Separator,

    // Section 3: Constraints
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_H,
        is_active: None,
        is_enabled: Some(commands::h_applicable),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_V,
        is_active: None,
        is_enabled: Some(commands::v_applicable),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_PARALLEL,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_PERP,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_EQUAL_LEN,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_ANGLE,
        is_active: None,
        is_enabled: Some(commands::exactly_two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_LENGTH,
        is_active: None,
        is_enabled: Some(commands::has_length_target),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_COINCIDENT,
        is_active: None,
        is_enabled: Some(commands::has_coincident_target),
    },
];

// ── Main toolbar (3D mode, top bar) ───────────────────────────────────────────

static FILE_MENU_ITEMS: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button { spec: &SPEC_NEW,     is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_OPEN,    is_active: None, is_enabled: None },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Button { spec: &SPEC_SAVE,    is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_SAVE_AS, is_active: None, is_enabled: None },
];

static PRIMITIVES_MENU_ITEMS: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button { spec: &SPEC_ADD_BOX,      is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_ADD_CYLINDER, is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_ADD_SPHERE,   is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_ADD_CONE,     is_active: None, is_enabled: None },
];

static SKETCH_MENU_ITEMS: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button { spec: &SPEC_SKETCH_XY, is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_SKETCH_XZ, is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_SKETCH_YZ, is_active: None, is_enabled: None },
];

static BOOLEAN_MENU_ITEMS: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button { spec: &SPEC_BOOL_UNION,       is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_BOOL_DIFFERENCE,  is_active: None, is_enabled: None },
    ToolbarItemKind::Button { spec: &SPEC_BOOL_INTERSECT,   is_active: None, is_enabled: None },
];

pub static MAIN_TOOLBAR: &[ToolbarItemKind] = &[
    ToolbarItemKind::Menu {
        label: "File",
        icon: Some(icons::icon_file),
        is_enabled: None,
        children: FILE_MENU_ITEMS,
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Menu {
        label: "Primitives",
        icon: Some(icons::icon_primitives),
        is_enabled: None,
        children: PRIMITIVES_MENU_ITEMS,
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Menu {
        label: "Sketch",
        icon: Some(icons::icon_sketch),
        is_enabled: None,
        children: SKETCH_MENU_ITEMS,
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Menu {
        label: "Boolean",
        icon: Some(icons::icon_boolean),
        is_enabled: Some(commands::two_solids_selected),
        children: BOOLEAN_MENU_ITEMS,
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Button {
        spec: &SPEC_DELETE_SELECTED,
        is_active: None,
        is_enabled: Some(commands::can_delete),
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Button {
        spec: &SPEC_UNDO,
        is_active: None,
        is_enabled: Some(commands::can_undo),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_REDO,
        is_active: None,
        is_enabled: Some(commands::can_redo),
    },
];

// ── Sketch top-bar editing controls ──────────────────────────────────────────

/// Left side: Undo-point and Finish buttons (shown before the constraint section).
pub static SKETCH_EDITING_BUTTONS: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button {
        spec: &SPEC_SKETCH_UNDO_POINT,
        is_active: None,
        is_enabled: None,
    },
    ToolbarItemKind::Button {
        spec: &SPEC_SKETCH_FINISH,
        is_active: None,
        is_enabled: Some(commands::sketch_can_finish),
    },
];

/// Right side: Cancel button (shown at the end of the sketch top bar).
pub static SKETCH_CANCEL_BUTTON: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button {
        spec: &SPEC_SKETCH_CANCEL,
        is_active: None,
        is_enabled: None,
    },
];

// ── Sketch constrain menu items (reused in top-bar dropdown) ──────────────────

pub static SKETCH_CONSTRAIN_MENU: &[ToolbarItemKind] = &[
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_H,
        is_active: None,
        is_enabled: Some(commands::h_applicable),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_V,
        is_active: None,
        is_enabled: Some(commands::v_applicable),
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_PARALLEL,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_PERP,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_EQUAL_LEN,
        is_active: None,
        is_enabled: Some(commands::two_segs),
    },
    ToolbarItemKind::Separator,
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_ANGLE,
        is_active: None,
        is_enabled: Some(commands::exactly_two_segs),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_LENGTH,
        is_active: None,
        is_enabled: Some(commands::has_length_target),
    },
    ToolbarItemKind::Button {
        spec: &SPEC_CONSTRAIN_COINCIDENT,
        is_active: None,
        is_enabled: Some(commands::has_coincident_target),
    },
];
