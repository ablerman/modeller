//! Generic toolbar rendering machinery.
//!
//! Toolbars are expressed as `&'static [ToolbarItemKind]` slices defined in
//! `toolbar_defs.rs`.  `render_toolbar` iterates over a slice and emits
//! `UiAction`s for any button that was clicked.

use crate::commands::{ActionSource, CommandSpec};
use crate::editor::{EditorState, UiAction};

// ── Icon sizing ───────────────────────────────────────────────────────────────

/// Display size for toolbar icons.
#[derive(Clone, Copy)]
pub enum IconSize {
    /// Top toolbar (16 px).
    Toolbar,
    /// Floating viewport overlay (18 px).
    Viewport,
}

fn sized_icon(src: egui::ImageSource<'static>, size: IconSize) -> egui::Image<'static> {
    let px = match size { IconSize::Toolbar => 16.0, IconSize::Viewport => 18.0 };
    egui::Image::new(src).fit_to_exact_size(egui::vec2(px, px))
}

// ── ToolbarItemKind ───────────────────────────────────────────────────────────

/// One item in a toolbar definition.
pub enum ToolbarItemKind {
    /// A push/toggle button backed by a `CommandSpec`.
    Button {
        spec:       &'static CommandSpec,
        /// If `Some`, the button is rendered as "selected/active" when the
        /// predicate returns `true` (e.g. draw-tool toggle buttons).
        is_active:  Option<fn(&EditorState) -> bool>,
        /// If `Some`, the button is greyed out when the predicate returns
        /// `false`.
        is_enabled: Option<fn(&EditorState) -> bool>,
    },
    /// Horizontal separator between button groups.
    Separator,
    /// A drop-down menu containing child toolbar items.
    Menu {
        label:      &'static str,
        icon:       Option<fn() -> egui::ImageSource<'static>>,
        is_enabled: Option<fn(&EditorState) -> bool>,
        children:   &'static [ToolbarItemKind],
    },
    /// Escape hatch for items that cannot be expressed as commands — the
    /// render function is called directly.
    #[allow(dead_code)]
    Custom {
        render: fn(&mut egui::Ui, &EditorState, &mut Vec<UiAction>),
    },
}

// ── render_toolbar ────────────────────────────────────────────────────────────

/// Render a toolbar definition into `ui`, appending any triggered actions
/// to `out`.
pub fn render_toolbar(
    ui:     &mut egui::Ui,
    items:  &'static [ToolbarItemKind],
    editor: &EditorState,
    out:    &mut Vec<UiAction>,
    size:   IconSize,
) {
    for item in items {
        match item {
            ToolbarItemKind::Separator => {
                ui.separator();
            }

            ToolbarItemKind::Button { spec, is_active, is_enabled } => {
                let enabled = is_enabled.map_or(true, |f| f(editor));
                let active  = is_active.map_or(false, |f| f(editor));
                ui.add_enabled_ui(enabled, |ui| {
                    let clicked = if let Some(icon_fn) = spec.icon {
                        ui.add(
                            egui::ImageButton::new(sized_icon(icon_fn(), size))
                                .selected(active),
                        )
                        .on_hover_text(spec.tooltip)
                        .clicked()
                    } else {
                        ui.button(spec.label)
                            .on_hover_text(spec.tooltip)
                            .clicked()
                    };
                    if clicked {
                        let actions = match &spec.action {
                            ActionSource::Fixed(a)   => vec![a.clone()],
                            ActionSource::Dynamic(f) => f(editor),
                        };
                        out.extend(actions);
                    }
                });
            }

            ToolbarItemKind::Menu { label, icon, is_enabled, children } => {
                let enabled = is_enabled.map_or(true, |f| f(editor));
                ui.add_enabled_ui(enabled, |ui| {
                    let btn = match icon {
                        Some(icon_fn) => egui::Button::image_and_text(
                            sized_icon(icon_fn(), size).tint(ui.visuals().text_color()),
                            *label,
                        ),
                        None => egui::Button::new(*label),
                    };
                    egui::menu::menu_custom_button(ui, btn, |ui| {
                        let before = out.len();
                        render_toolbar(ui, children, editor, out, size);
                        if out.len() > before {
                            ui.close_menu();
                        }
                    });
                });
            }

            ToolbarItemKind::Custom { render } => {
                render(ui, editor, out);
            }
        }
    }
}
