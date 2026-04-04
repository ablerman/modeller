//! egui panel layout: toolbar, object list, properties.

use brep_bool::BooleanKind;
use egui::Context;

use crate::editor::{EditorState, ObjectHistory, PrimitiveKind, UiAction};

/// Render all egui panels.  Returns any actions the user triggered.
pub fn build_ui(ctx: &Context, editor: &EditorState) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    // ── Top toolbar ───────────────────────────────────────────────────────────
    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.strong("Add:");
            if ui.button("Box").clicked() {
                actions.push(UiAction::AddPrimitive(PrimitiveKind::Box));
            }
            if ui.button("Cylinder").clicked() {
                actions.push(UiAction::AddPrimitive(PrimitiveKind::Cylinder));
            }
            if ui.button("Sphere").clicked() {
                actions.push(UiAction::AddPrimitive(PrimitiveKind::Sphere));
            }
            if ui.button("Cone").clicked() {
                actions.push(UiAction::AddPrimitive(PrimitiveKind::Cone));
            }

            ui.separator();
            ui.strong("Boolean:");

            let two_sel = editor.selection.len() == 2;
            ui.add_enabled_ui(two_sel, |ui| {
                if ui.button("Union").clicked() {
                    actions.push(UiAction::BooleanOp(BooleanKind::Union));
                }
                if ui.button("Difference").clicked() {
                    actions.push(UiAction::BooleanOp(BooleanKind::Difference));
                }
                if ui.button("Intersection").clicked() {
                    actions.push(UiAction::BooleanOp(BooleanKind::Intersection));
                }
            });

            ui.separator();

            let can_delete = !editor.selection.is_empty();
            ui.add_enabled_ui(can_delete, |ui| {
                if ui.button("Delete").clicked() {
                    actions.push(UiAction::DeleteSelected);
                }
            });

            ui.separator();

            ui.add_enabled_ui(editor.history.can_undo(), |ui| {
                if ui.button("Undo").clicked() {
                    actions.push(UiAction::Undo);
                }
            });
            ui.add_enabled_ui(editor.history.can_redo(), |ui| {
                if ui.button("Redo").clicked() {
                    actions.push(UiAction::Redo);
                }
            });

            ui.separator();
            ui.label("Drag: orbit  |  Shift+drag: pan  |  Scroll: zoom");
        });
    });

    // ── Left panel: object list (operation tree) ──────────────────────────────
    egui::SidePanel::left("objects").min_width(180.0).show(ctx, |ui| {
        ui.strong("Objects");
        ui.separator();
        for (i, obj) in editor.objects.iter().enumerate() {
            let selected = editor.selection.contains(&i);
            // Top-level row: selectable label for the result object.
            let resp = ui.selectable_label(selected, &obj.name);
            if resp.clicked() {
                if ui.input(|s| s.modifiers.shift || s.modifiers.ctrl) {
                    actions.push(UiAction::ToggleSelectObject(i));
                } else {
                    actions.push(UiAction::SelectObject(i));
                }
            }
            // Indented history subtree (only shown for boolean results).
            if matches!(obj.history, ObjectHistory::Boolean { .. }) {
                ui.indent(format!("hist_{i}"), |ui| {
                    show_history(ui, &obj.history);
                });
            }
        }
        if editor.objects.is_empty() {
            ui.label(egui::RichText::new("(empty scene)").italics().weak());
        }
    });

    // ── Right panel: properties ───────────────────────────────────────────────
    egui::SidePanel::right("properties").min_width(160.0).show(ctx, |ui| {
        ui.strong("Properties");
        ui.separator();
        if let Some((faces, edges, verts)) = editor.selection_stats() {
            ui.label(format!("Faces:    {faces}"));
            ui.label(format!("Edges:    {edges}"));
            ui.label(format!("Vertices: {verts}"));
        } else {
            ui.label(egui::RichText::new("(nothing selected)").italics().weak());
        }

        ui.separator();
        ui.strong("Camera");
        let cam = &editor.camera;
        let eye = cam.eye();
        ui.label(format!("eye:  ({:.2}, {:.2}, {:.2})", eye.x, eye.y, eye.z));
        ui.label(format!("dist: {:.2}", cam.distance));
    });

    actions
}

/// Recursively render an operation history node as a collapsible tree.
fn show_history(ui: &mut egui::Ui, node: &ObjectHistory) {
    match node {
        ObjectHistory::Primitive(_) => {
            ui.label(
                egui::RichText::new(node.label())
                    .weak()
                    .italics(),
            );
        }
        ObjectHistory::Boolean { left, right, .. } => {
            ui.collapsing(node.label(), |ui| {
                show_history(ui, left);
                show_history(ui, right);
            });
        }
    }
}
