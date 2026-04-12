//! Constraints list section of the sketch info panel.

use crate::editor::{SketchState, UiAction};
use crate::icons::icon_trash;
use crate::ui::constraint_helpers::{
    constraint_element_refs, constraint_icon, constraint_text,
    cross_constraint_icon, cross_constraint_label,
};

// list_icon lives in the ui module; child modules can reach parent items.
use super::super::list_icon;

pub(super) fn draw_constraints_list(
    ui: &mut egui::Ui,
    sk: &SketchState,
    actions: &mut Vec<UiAction>,
    hi: &mut super::PanelHighlights,
) {
    let has_any_constraints = !sk.constraints.is_empty()
        || sk.committed_profiles.iter().any(|cp| !cp.constraints.is_empty())
        || !sk.cross_constraints.is_empty();
    if !has_any_constraints {
        ui.label(egui::RichText::new("(none)").italics().weak());
        return;
    }

    egui::ScrollArea::vertical()
        .id_salt("sketch_constraints_scroll")
        .max_height(f32::INFINITY)
        .min_scrolled_height(60.0)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());

            // Active profile constraints.
            let mut remove_idx: Option<usize> = None;
            for (i, c) in sk.constraints.iter().enumerate() {
                let is_violated =
                    sk.violated_constraints.get(i).copied().unwrap_or(false);
                let is_selected = sk.constraint_selection.contains(&i);
                let is_hi = !is_selected && hi.hi_constraints.contains(&i);
                let base_text = constraint_text(c);
                let label_text = if is_violated {
                    egui::RichText::new(&base_text).color(egui::Color32::RED)
                } else if is_hi {
                    egui::RichText::new(&base_text).color(egui::Color32::from_rgb(255, 160, 60))
                } else {
                    egui::RichText::new(&base_text)
                };
                ui.horizontal(|ui| {
                    let row_h = ui.spacing().interact_size.y;
                    let scrollbar_pad = 12.0;
                    let trash_icon_w = 14.0;
                    let trash_total = scrollbar_pad + trash_icon_w
                        + ui.spacing().item_spacing.x * 2.0;
                    let row_w = (ui.available_width() - trash_total).max(0.0);
                    let (row_rect, resp) = ui.allocate_exact_size(
                        egui::vec2(row_w, row_h),
                        egui::Sense::click(),
                    );
                    if is_selected {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            ui.visuals().selection.bg_fill,
                        );
                    } else if is_hi {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            egui::Color32::from_rgba_unmultiplied(255, 160, 60, 30),
                        );
                    } else if resp.hovered() {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            ui.visuals().widgets.hovered.weak_bg_fill,
                        );
                    }
                    let mut child = ui.new_child(
                        egui::UiBuilder::new()
                            .max_rect(row_rect)
                            .layout(egui::Layout::left_to_right(egui::Align::Center)),
                    );
                    child.add(list_icon(constraint_icon(c)));
                    let text_color = if is_selected {
                        ui.visuals().selection.stroke.color
                    } else if is_violated {
                        egui::Color32::RED
                    } else if is_hi {
                        egui::Color32::from_rgb(255, 160, 60)
                    } else {
                        ui.visuals().text_color()
                    };
                    child.add(
                        egui::Label::new(label_text.color(text_color))
                            .selectable(false),
                    );
                    if resp.clicked() {
                        actions.push(UiAction::SketchSelectConstraint(i));
                    }
                    // Hovering a constraint highlights its referenced elements.
                    if resp.hovered() {
                        let (pts, segs) = constraint_element_refs(c);
                        hi.hi_pts.extend(pts);
                        hi.hi_segs.extend(segs);
                    }
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            ui.add_space(scrollbar_pad);
                            let trash = egui::ImageButton::new(
                                egui::Image::new(icon_trash())
                                    .fit_to_exact_size(egui::vec2(14.0, 14.0))
                                    .tint(egui::Color32::from_rgb(210, 60, 60)),
                            );
                            if ui.add(trash).on_hover_text("Remove constraint").clicked() {
                                remove_idx = Some(i);
                            }
                        },
                    );
                });
            }
            if let Some(i) = remove_idx {
                actions.push(UiAction::SketchRemoveConstraint(i));
            }

            // Per-profile constraints (arcs, circles, segments).
            let mut remove_committed: Option<(usize, usize)> = None;
            for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                if cp.constraints.is_empty() { continue; }
                let profile_label = cp.shape.label(pi);
                for (ci, c) in cp.constraints.iter().enumerate() {
                    let is_selected = sk.committed_constraint_selection.contains(&(pi, ci));
                    let is_hi = !is_selected && hi.hi_committed_constraints.contains(&(pi, ci));
                    ui.horizontal(|ui| {
                        let row_h = ui.spacing().interact_size.y;
                        let scrollbar_pad = 12.0;
                        let trash_icon_w = 14.0;
                        let trash_total = scrollbar_pad + trash_icon_w
                            + ui.spacing().item_spacing.x * 2.0;
                        let row_w = (ui.available_width() - trash_total).max(0.0);
                        let (row_rect, resp) = ui.allocate_exact_size(
                            egui::vec2(row_w, row_h),
                            egui::Sense::click(),
                        );
                        if is_selected {
                            ui.painter().rect_filled(
                                row_rect,
                                ui.visuals().widgets.hovered.rounding,
                                ui.visuals().selection.bg_fill,
                            );
                        } else if is_hi {
                            ui.painter().rect_filled(
                                row_rect,
                                ui.visuals().widgets.hovered.rounding,
                                egui::Color32::from_rgba_unmultiplied(255, 160, 60, 30),
                            );
                        } else if resp.hovered() {
                            ui.painter().rect_filled(
                                row_rect,
                                ui.visuals().widgets.hovered.rounding,
                                ui.visuals().widgets.hovered.weak_bg_fill,
                            );
                        }
                        let mut child = ui.new_child(
                            egui::UiBuilder::new()
                                .max_rect(row_rect)
                                .layout(egui::Layout::left_to_right(egui::Align::Center)),
                        );
                        child.add(list_icon(constraint_icon(c)));
                        let text_color = if is_selected {
                            ui.visuals().selection.stroke.color
                        } else if is_hi {
                            egui::Color32::from_rgb(255, 160, 60)
                        } else {
                            ui.visuals().text_color()
                        };
                        child.add(
                            egui::Label::new(
                                egui::RichText::new(
                                    format!("{} — {}", constraint_text(c), profile_label)
                                ).color(text_color),
                            )
                            .selectable(false),
                        );
                        if resp.clicked() {
                            actions.push(UiAction::SketchSelectCommittedConstraint(pi, ci));
                        }
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                ui.add_space(scrollbar_pad);
                                let trash = egui::ImageButton::new(
                                    egui::Image::new(icon_trash())
                                        .fit_to_exact_size(egui::vec2(14.0, 14.0))
                                        .tint(egui::Color32::from_rgb(210, 60, 60)),
                                );
                                if ui.add(trash).on_hover_text("Remove constraint").clicked() {
                                    remove_committed = Some((pi, ci));
                                }
                            },
                        );
                    });
                }
            }
            if let Some((pi, ci)) = remove_committed {
                actions.push(UiAction::SketchRemoveCommittedConstraint(pi, ci));
            }

            // Cross-profile constraints.
            let mut remove_cross_idx: Option<usize> = None;
            for (i, cc) in sk.cross_constraints.iter().enumerate() {
                let is_selected = sk.cross_constraint_selection.contains(&i);
                let is_hi = !is_selected && hi.hi_cross_constraints.contains(&i);
                ui.horizontal(|ui| {
                    let row_h = ui.spacing().interact_size.y;
                    let scrollbar_pad = 12.0;
                    let trash_icon_w = 14.0;
                    let trash_total = scrollbar_pad + trash_icon_w
                        + ui.spacing().item_spacing.x * 2.0;
                    let row_w = (ui.available_width() - trash_total).max(0.0);
                    let (row_rect, resp) = ui.allocate_exact_size(
                        egui::vec2(row_w, row_h),
                        egui::Sense::click(),
                    );
                    if is_selected {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            ui.visuals().selection.bg_fill,
                        );
                    } else if is_hi {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            egui::Color32::from_rgba_unmultiplied(255, 160, 60, 30),
                        );
                    } else if resp.hovered() {
                        ui.painter().rect_filled(
                            row_rect,
                            ui.visuals().widgets.hovered.rounding,
                            ui.visuals().widgets.hovered.weak_bg_fill,
                        );
                    }
                    let mut child = ui.new_child(
                        egui::UiBuilder::new()
                            .max_rect(row_rect)
                            .layout(egui::Layout::left_to_right(egui::Align::Center)),
                    );
                    child.add(list_icon(cross_constraint_icon(cc)));
                    let text_color = if is_selected {
                        ui.visuals().selection.stroke.color
                    } else if is_hi {
                        egui::Color32::from_rgb(255, 160, 60)
                    } else {
                        ui.visuals().text_color()
                    };
                    child.add(
                        egui::Label::new(
                            egui::RichText::new(
                                cross_constraint_label(cc, &sk.committed_profiles)
                            ).color(text_color),
                        )
                        .selectable(false),
                    );
                    if resp.clicked() {
                        actions.push(UiAction::SketchSelectCrossConstraint(i));
                    }
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            ui.add_space(scrollbar_pad);
                            let trash = egui::ImageButton::new(
                                egui::Image::new(icon_trash())
                                    .fit_to_exact_size(egui::vec2(14.0, 14.0))
                                    .tint(egui::Color32::from_rgb(210, 60, 60)),
                            );
                            if ui.add(trash).on_hover_text("Remove constraint").clicked() {
                                remove_cross_idx = Some(i);
                            }
                        },
                    );
                });
            }
            if let Some(i) = remove_cross_idx {
                actions.push(UiAction::SketchRemoveCrossConstraint(i));
            }
        });
}
