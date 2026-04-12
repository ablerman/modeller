//! History/actions list section of the sketch info panel.

use crate::editor::{SketchState, UiAction};
use crate::icons::icon_trash;

pub(super) fn draw_history_list(
    ui: &mut egui::Ui,
    sk: &SketchState,
    actions: &mut Vec<UiAction>,
) {
    if sk.history.undo_stack.is_empty() {
        ui.label(egui::RichText::new("(none)").italics().weak());
        return;
    }

    egui::ScrollArea::vertical()
        .id_salt("sketch_actions_scroll")
        .max_height(f32::INFINITY)
        .min_scrolled_height(60.0)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            let mut delete_idx: Option<usize> = None;
            for (i, entry) in sk.history.undo_stack.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(&entry.label).monospace());
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            ui.add_space(12.0);
                            let trash = egui::ImageButton::new(
                                egui::Image::new(icon_trash())
                                    .fit_to_exact_size(egui::vec2(14.0, 14.0))
                                    .tint(egui::Color32::from_rgb(210, 60, 60)),
                            );
                            if ui.add(trash).on_hover_text("Undo to before this action").clicked() {
                                delete_idx = Some(i);
                            }
                        },
                    );
                });
            }
            if let Some(i) = delete_idx {
                actions.push(UiAction::SketchDeleteHistoryEntry(i));
            }
        });
}
