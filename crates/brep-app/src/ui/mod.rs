//! egui panel layout: toolbar, object list, properties.

mod constraint_helpers;
mod modals;
mod overlay;
mod panel;

use brep_core::Point3;
use egui::Context;

use crate::editor::{
    AngleDialogTarget, EditorState, LengthTarget, ObjectHistory,
    PrimitiveKind, RefEntity, SceneEntry, SketchState, UiAction,
};
use crate::toolbar::IconSize;
use crate::toolbar_defs;

use constraint_helpers::constraint_icon;
pub(crate) use constraint_helpers::constraint_text;
use overlay::{draw_sketch_outlines, draw_sketch_overlay, draw_viewport_axes};
use panel::draw_sketch_info_panel;

// ── Shared helpers ────────────────────────────────────────────────────────────

/// Icon image sized for use in constraint list labels.
pub(super) fn list_icon(src: egui::ImageSource<'static>) -> egui::Image<'static> {
    egui::Image::new(src).fit_to_exact_size(egui::vec2(14.0, 14.0))
}

// ── Main entry point ──────────────────────────────────────────────────────────

/// Render all egui panels.  Returns any actions the user triggered.
pub fn build_ui(
    ctx: &Context,
    editor: &EditorState,
    viewport: (u32, u32),
    sketch_cursor: Option<Point3>,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
    snap_ref: Option<RefEntity>,
    snap_constraint: Option<usize>,
    snap_committed: Option<(usize, usize)>,
    snap_committed_curve: Option<usize>,
    snap_committed_seg: Option<(usize, usize)>,
    angle_dialog: Option<AngleDialogTarget>,
    length_dialog: Option<(LengthTarget, f64)>,
    context_menu: Option<((f32, f32), usize)>,
) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    // ── Top toolbar ───────────────────────────────────────────────────────────
    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            if let Some(sk) = &editor.sketch {
                ui.label(
                    egui::RichText::new(format!("● Sketch  ({:?} plane)", sk.plane))
                        .color(egui::Color32::from_rgb(100, 200, 255))
                        .strong(),
                );
                ui.label(format!("{} pts", sk.points.len()));
                ui.separator();

                crate::toolbar::render_toolbar(
                    ui, toolbar_defs::SKETCH_EDITING_BUTTONS, editor, &mut actions, IconSize::Toolbar,
                );

                if !sk.points.is_empty() {
                    ui.separator();

                    ui.menu_button("Constrain ▾", |ui| {
                        let before = actions.len();
                        crate::toolbar::render_toolbar(
                            ui, toolbar_defs::SKETCH_CONSTRAIN_MENU, editor, &mut actions, IconSize::Toolbar,
                        );
                        if actions.len() > before { ui.close_menu(); }
                    });

                    draw_conflict_badge(ui, sk);
                    draw_fully_constrained_badge(ui, sk);

                    if !sk.constraints.is_empty() {
                        ui.separator();
                        let list_label = if sk.constraints_conflict {
                            egui::RichText::new(format!("Constraints ({}) ⚠", sk.constraints.len()))
                                .color(egui::Color32::from_rgb(255, 80, 80))
                        } else {
                            egui::RichText::new(format!("Constraints ({})", sk.constraints.len()))
                        };
                        ui.menu_button(list_label, |ui| {
                            let mut to_remove = None;
                            for (i, c) in sk.constraints.iter().enumerate() {
                                let is_violated = sk.violated_constraints.get(i).copied().unwrap_or(false);
                                ui.horizontal(|ui| {
                                    ui.add(list_icon(constraint_icon(c)));
                                    let text = constraint_text(c);
                                    if is_violated {
                                        ui.label(
                                            egui::RichText::new(text)
                                                .color(egui::Color32::from_rgb(255, 100, 100)),
                                        ).on_hover_text("This constraint contributes to the conflict");
                                    } else {
                                        ui.label(text);
                                    }
                                    if ui.small_button("✕").clicked() {
                                        to_remove = Some(i);
                                        ui.close_menu();
                                    }
                                });
                            }
                            if let Some(i) = to_remove {
                                actions.push(UiAction::SketchRemoveConstraint(i));
                            }
                        });
                    }
                }

                ui.separator();
                crate::toolbar::render_toolbar(
                    ui, toolbar_defs::SKETCH_CANCEL_BUTTON, editor, &mut actions, IconSize::Toolbar,
                );
            } else {
                crate::toolbar::render_toolbar(
                    ui, toolbar_defs::MAIN_TOOLBAR, editor, &mut actions, IconSize::Toolbar,
                );
                ui.separator();
                ui.label("Drag: orbit  |  Shift+drag: pan  |  Scroll: zoom");
            }
        });
    });

    // ── Left panel: operations list ───────────────────────────────────────────
    egui::SidePanel::left("objects").min_width(180.0).show(ctx, |ui| {
        ui.strong("Operations");
        ui.separator();
        for (i, entry) in editor.entries.iter().enumerate() {
            let selected = editor.selection.contains(&i);
            match entry {
                SceneEntry::Sketch(sk) => {
                    let resp = ui.selectable_label(
                        selected,
                        egui::RichText::new(format!("✏  {}  ({:?})", sk.name, sk.plane)).italics(),
                    );
                    if resp.double_clicked() {
                        actions.push(UiAction::OpenSketch(i));
                    } else if resp.clicked() {
                        if ui.input(|s| s.modifiers.shift || s.modifiers.ctrl) {
                            actions.push(UiAction::ToggleSelectObject(i));
                        } else {
                            actions.push(UiAction::SelectObject(i));
                        }
                    }
                }
                SceneEntry::Solid(obj) => {
                    match &obj.history {
                        ObjectHistory::Primitive(kind) => {
                            let icon = match kind {
                                PrimitiveKind::Box      => "☐",
                                PrimitiveKind::Cylinder => "◎",
                                PrimitiveKind::Sphere   => "◉",
                                PrimitiveKind::Cone     => "△",
                            };
                            let resp = ui.selectable_label(selected, format!("{icon}  {}", obj.name));
                            if resp.clicked() {
                                if ui.input(|s| s.modifiers.shift || s.modifiers.ctrl) {
                                    actions.push(UiAction::ToggleSelectObject(i));
                                } else {
                                    actions.push(UiAction::SelectObject(i));
                                }
                            }
                        }
                        ObjectHistory::Sketch { plane, extrude_dist, .. } => {
                            ui.horizontal(|ui| {
                                ui.add_space(4.0);
                                ui.label(egui::RichText::new(format!("✏  Sketch  ({plane:?})")).weak().italics());
                            });
                            let resp = ui.selectable_label(
                                selected,
                                egui::RichText::new(format!("  ↑  Extrude  {extrude_dist:.2} u")),
                            );
                            if resp.clicked() {
                                if ui.input(|s| s.modifiers.shift || s.modifiers.ctrl) {
                                    actions.push(UiAction::ToggleSelectObject(i));
                                } else {
                                    actions.push(UiAction::SelectObject(i));
                                }
                            }
                        }
                        ObjectHistory::Boolean { kind, left, right } => {
                            let icon = match kind {
                                brep_bool::BooleanKind::Union        => "⊕",
                                brep_bool::BooleanKind::Difference   => "⊖",
                                brep_bool::BooleanKind::Intersection => "⊗",
                            };
                            let resp = ui.selectable_label(selected, format!("{icon}  {}", obj.name));
                            if resp.clicked() {
                                if ui.input(|s| s.modifiers.shift || s.modifiers.ctrl) {
                                    actions.push(UiAction::ToggleSelectObject(i));
                                } else {
                                    actions.push(UiAction::SelectObject(i));
                                }
                            }
                            ui.indent(format!("bool_{i}"), |ui| {
                                show_history_op(ui, left);
                                show_history_op(ui, right);
                            });
                        }
                    }
                }
            }
        }
        if editor.entries.is_empty() {
            ui.label(egui::RichText::new("(empty scene)").italics().weak());
        }

        // ── Properties ────────────────────────────────────────────────
        ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
            ui.add_space(4.0);
            let cam = &editor.camera;
            let eye = cam.eye();
            ui.label(format!("dist: {:.2}", cam.distance));
            ui.label(format!("eye  ({:.2}, {:.2}, {:.2})", eye.x, eye.y, eye.z));
            ui.strong("Camera");
            ui.separator();
            if let Some((faces, edges, verts)) = editor.selection_stats() {
                ui.label(format!("Verts: {verts}"));
                ui.label(format!("Edges: {edges}"));
                ui.label(format!("Faces: {faces}"));
            } else {
                ui.label(egui::RichText::new("(nothing selected)").italics().weak());
            }
            ui.strong("Properties");
            ui.separator();
        });
    });

    // ── Viewport axes ─────────────────────────────────────────────────────────
    draw_viewport_axes(ctx, editor);

    // ── Orientation cube gizmo ────────────────────────────────────────────────
    if let Some(snap) = crate::gizmo::draw_orientation_cube(ctx, editor) {
        actions.push(snap);
    }

    // ── Sketch info panel ─────────────────────────────────────────────────────
    let (panel_actions, panel_hi) = draw_sketch_info_panel(
        ctx, editor,
        snap_vertex, snap_segment,
        snap_committed, snap_committed_curve, snap_committed_seg,
    );
    actions.extend(panel_actions);

    // ── Sketch viewport toolbar ───────────────────────────────────────────────
    actions.extend(draw_sketch_viewport_toolbar(ctx, editor));

    // ── Sketch overlay ────────────────────────────────────────────────────────
    if let Some(sk) = &editor.sketch {
        actions.extend(draw_sketch_overlay(
            ctx, sk, viewport, &editor.camera,
            snap_vertex, snap_segment, snap_ref, snap_constraint,
            snap_committed, snap_committed_curve, snap_committed_seg,
            sketch_cursor, context_menu, &panel_hi,
        ));
    }

    // ── Finished sketch outlines ──────────────────────────────────────────────
    if editor.sketch.is_none() {
        draw_sketch_outlines(ctx, editor, viewport);
    }

    // ── Length input modal ────────────────────────────────────────────────────
    if let Some((target, initial_len)) = length_dialog {
        actions.extend(modals::draw_length_modal(ctx, editor, target, initial_len));
    }

    // ── Angle input modal ─────────────────────────────────────────────────────
    if let Some(angle_target) = angle_dialog {
        actions.extend(modals::draw_angle_modal(ctx, angle_target));
    }

    actions
}

// ── Small helpers used within this module ────────────────────────────────────

fn show_history_op(ui: &mut egui::Ui, node: &ObjectHistory) {
    match node {
        ObjectHistory::Primitive(kind) => {
            let icon = match kind {
                PrimitiveKind::Box      => "☐",
                PrimitiveKind::Cylinder => "◎",
                PrimitiveKind::Sphere   => "◉",
                PrimitiveKind::Cone     => "△",
            };
            ui.label(egui::RichText::new(format!("{icon}  {}", node.label())).weak().italics());
        }
        ObjectHistory::Sketch { plane, extrude_dist, .. } => {
            ui.label(egui::RichText::new(format!("✏  Sketch ({plane:?})")).weak().italics());
            ui.label(egui::RichText::new(format!("  ↑  Extrude  {extrude_dist:.2} u")).weak().italics());
        }
        ObjectHistory::Boolean { kind, left, right } => {
            let icon = match kind {
                brep_bool::BooleanKind::Union        => "⊕",
                brep_bool::BooleanKind::Difference   => "⊖",
                brep_bool::BooleanKind::Intersection => "⊗",
            };
            ui.collapsing(format!("{icon}  {}", node.label()), |ui| {
                show_history_op(ui, left);
                show_history_op(ui, right);
            });
        }
    }
}

fn draw_conflict_badge(ui: &mut egui::Ui, sk: &SketchState) {
    if !sk.constraints_conflict { return; }
    let n_conflict = sk.violated_constraints.iter().filter(|&&v| v).count();
    let badge = ui.add(
        egui::Label::new(
            egui::RichText::new("⚠ conflict")
                .color(egui::Color32::from_rgb(255, 80, 80))
                .strong(),
        )
        .sense(egui::Sense::hover()),
    );
    badge.on_hover_ui(|ui| {
        ui.set_max_width(280.0);
        ui.label(egui::RichText::new("Constraint conflict").strong());
        ui.separator();
        if n_conflict > 0 {
            ui.label(format!(
                "{} constraint{} cannot be satisfied together:",
                n_conflict,
                if n_conflict == 1 { "" } else { "s" }
            ));
            for (i, c) in sk.constraints.iter().enumerate() {
                if sk.violated_constraints.get(i).copied().unwrap_or(false) {
                    ui.horizontal(|ui| {
                        ui.add(list_icon(constraint_icon(c)));
                        ui.label(
                            egui::RichText::new(constraint_text(c))
                                .color(egui::Color32::from_rgb(255, 100, 100)),
                        );
                    });
                }
            }
        } else {
            ui.label("The solver could not converge. Try removing a constraint.");
        }
        ui.separator();
        ui.label(
            egui::RichText::new("Remove a highlighted constraint to resolve.")
                .italics()
                .weak(),
        );
    });
}

fn draw_fully_constrained_badge(ui: &mut egui::Ui, sk: &SketchState) {
    if !sk.fully_constrained { return; }
    let badge = ui.add(
        egui::Label::new(
            egui::RichText::new("✓ fully constrained")
                .color(egui::Color32::from_rgb(80, 200, 120))
                .strong(),
        )
        .sense(egui::Sense::hover()),
    );
    badge.on_hover_text("All degrees of freedom are constrained.");
}

fn draw_sketch_viewport_toolbar(ctx: &egui::Context, editor: &EditorState) -> Vec<UiAction> {
    if editor.sketch.is_none() { return vec![]; }
    let mut actions: Vec<UiAction> = Vec::new();

    let avail_top = ctx.available_rect().min.y;

    egui::Area::new(egui::Id::new("sketch_viewport_toolbar"))
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, avail_top + 16.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::default()
                .fill(egui::Color32::from_rgba_unmultiplied(28, 28, 36, 220))
                .rounding(egui::Rounding::same(5.0))
                .inner_margin(egui::Margin::same(5.0))
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(80, 80, 100, 180)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 2.0;
                        crate::toolbar::render_toolbar(
                            ui, toolbar_defs::SKETCH_VIEWPORT_TOOLBAR,
                            editor, &mut actions, IconSize::Viewport,
                        );
                    });
                });
        });

    actions
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use super::constraint_helpers::{constraint_text, snap_to_viewport};
    use brep_sketch::SketchConstraint;

    // ── constraint_text ───────────────────────────────────────────────────────

    #[test]
    fn constraint_text_parallel() {
        let c = SketchConstraint::Parallel { seg_a: 0, seg_b: 2 };
        assert_eq!(constraint_text(&c), "Parallel  0 ∥ 2");
    }

    #[test]
    fn constraint_text_horizontal_seg() {
        let c = SketchConstraint::Horizontal { seg: 1 };
        assert_eq!(constraint_text(&c), "Horizontal  seg 1");
    }

    #[test]
    fn constraint_text_vertical_seg() {
        let c = SketchConstraint::Vertical { seg: 3 };
        assert_eq!(constraint_text(&c), "Vertical  seg 3");
    }

    #[test]
    fn constraint_text_angle() {
        let c = SketchConstraint::Angle { seg_a: 0, seg_b: 1, degrees: 90.0 };
        assert_eq!(constraint_text(&c), "Angle  90°  (0/1)");
    }

    #[test]
    fn constraint_text_fixed_length() {
        let c = SketchConstraint::FixedLength { seg: 2, value: 2.5 };
        assert_eq!(constraint_text(&c), "Length  seg 2 = 2.500");
    }

    #[test]
    fn constraint_text_coincident() {
        let c = SketchConstraint::Coincident { pt_a: 0, pt_b: 3 };
        assert_eq!(constraint_text(&c), "Coincident  pt 0 = 3");
    }

    #[test]
    fn constraint_text_point_on_origin() {
        let c = SketchConstraint::PointOnOrigin { pt: 1 };
        assert_eq!(constraint_text(&c), "On Origin  pt 1");
    }

    // ── snap_to_viewport ──────────────────────────────────────────────────────

    #[test]
    fn snap_centre_of_viewport_is_unchanged() {
        let vp   = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos  = egui::pos2(350.0, 275.0);
        let result = snap_to_viewport(pos, size, vp);
        assert!((result.x - pos.x).abs() < 0.1);
        assert!((result.y - pos.y).abs() < 0.1);
    }

    #[test]
    fn snap_near_left_edge_snaps_to_left() {
        let vp   = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos  = egui::pos2(30.0, 275.0);
        let result = snap_to_viewport(pos, size, vp);
        assert_eq!(result.x, 12.0);
    }

    #[test]
    fn snap_near_top_edge_snaps_to_top() {
        let vp   = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos  = egui::pos2(350.0, 20.0);
        let result = snap_to_viewport(pos, size, vp);
        assert_eq!(result.y, 12.0);
    }

    #[test]
    fn panel_outside_viewport_is_clamped_inside() {
        let vp   = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos  = egui::pos2(-200.0, -200.0);
        let result = snap_to_viewport(pos, size, vp);
        assert!(result.x >= 12.0);
        assert!(result.y >= 12.0);
    }

    // ── egui headless smoke tests ─────────────────────────────────────────────

    #[test]
    fn build_ui_empty_state_produces_no_actions() {
        let ed  = EditorState::new_empty();
        let ctx = egui::Context::default();
        let mut actions = Vec::new();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            actions = build_ui(
                ctx, &ed, (800, 600),
                None, None, None, None, None, None, None, None, None, None, None,
            );
        });
        assert!(actions.is_empty());
    }

    #[test]
    fn build_ui_in_sketch_produces_no_actions_without_input() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(crate::editor::SketchPlane::XY));
        let ctx = egui::Context::default();
        let mut actions = Vec::new();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            actions = build_ui(
                ctx, &ed, (800, 600),
                None, None, None, None, None, None, None, None, None, None, None,
            );
        });
        assert!(actions.is_empty());
    }
}
