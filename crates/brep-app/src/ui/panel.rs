//! Floating sketch info panel (elements, constraints, actions).

use crate::editor::{EditorState, UiAction};
use crate::icons::icon_trash;

use super::constraint_helpers::{
    constraint_element_refs, constraint_icon, constraint_text,
    cross_constraint_icon, cross_constraint_text, snap_to_viewport,
};
use super::list_icon;

// ── Panel highlights ──────────────────────────────────────────────────────────

/// Elements the panel wants the viewport to highlight this frame.
/// Populated by hovering items in the elements list or constraints list.
#[derive(Default)]
pub(super) struct PanelHighlights {
    /// Active-profile point indices to highlight in the viewport.
    pub hi_pts: std::collections::HashSet<usize>,
    /// Active-profile segment indices to highlight in the viewport.
    pub hi_segs: std::collections::HashSet<usize>,
    /// Committed-profile indices to highlight in the viewport.
    pub hi_committed: std::collections::HashSet<usize>,
    /// Active-profile constraint indices to highlight in the constraints list.
    pub hi_constraints: std::collections::HashSet<usize>,
}

// ── Panel ─────────────────────────────────────────────────────────────────────

/// Floating sketch configuration panel.  Returns actions and viewport highlights.
pub(super) fn draw_sketch_info_panel(
    ctx: &egui::Context,
    editor: &EditorState,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
    snap_committed: Option<(usize, usize)>,
    snap_committed_curve: Option<usize>,
    snap_committed_seg: Option<(usize, usize)>,
) -> (Vec<UiAction>, PanelHighlights) {
    let mut actions: Vec<UiAction> = Vec::new();
    let mut hi = PanelHighlights::default();
    let Some(sk) = &editor.sketch else { return (actions, hi) };

    // Seed constraint highlights from whatever is snapped in the viewport.
    if let Some(si) = snap_segment {
        for (ci, c) in sk.constraints.iter().enumerate() {
            if constraint_element_refs(c).1.contains(&si) {
                hi.hi_constraints.insert(ci);
            }
        }
    }
    if let Some(vi) = snap_vertex {
        for (ci, c) in sk.constraints.iter().enumerate() {
            if constraint_element_refs(c).0.contains(&vi) {
                hi.hi_constraints.insert(ci);
            }
        }
    }

    let panel_id = egui::Id::new("sketch_info_panel");
    let viewport = ctx.available_rect();
    let default_pos = egui::pos2(viewport.left() + 12.0, viewport.top() + 12.0);

    let is_dragging = ctx.input(|i| i.pointer.any_down());
    let win_size = egui::AreaState::load(ctx, panel_id)
        .and_then(|s| s.size)
        .unwrap_or(egui::vec2(240.0, 420.0));
    let stored_pos: egui::Pos2 = ctx.data(|d| d.get_temp(panel_id).unwrap_or(default_pos));
    let display_pos = if is_dragging {
        stored_pos
    } else {
        snap_to_viewport(stored_pos, win_size, viewport)
    };

    egui::Window::new("Sketch")
        .id(panel_id)
        .title_bar(false)
        .resizable(true)
        .movable(true)
        .constrain_to(viewport)
        .min_width(200.0)
        .min_height(180.0)
        .default_size(egui::vec2(260.0, 440.0))
        .current_pos(display_pos)
        .frame(egui::Frame::popup(ctx.style().as_ref()))
        .show(ctx, |ui| {
            // ── Finish Sketch button ──────────────────────────────────
            let can_finish = sk.points.len() >= 3
                || sk.committed_profiles.iter().any(|cp| {
                    cp.shape != crate::editor::ProfileShape::Polyline
                        || cp.point_indices.len() >= 2
                });
            ui.add_enabled_ui(can_finish, |ui| {
                if ui.button("✓  Finish Sketch").clicked() {
                    actions.push(UiAction::SketchFinish);
                }
            });

            ui.separator();

            // ── Editable name ─────────────────────────────────────────
            ui.horizontal(|ui| {
                ui.strong("Name");
                let mut name_buf = sk.name.clone();
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut name_buf)
                        .desired_width(ui.available_width()),
                );
                if resp.changed() {
                    actions.push(UiAction::SketchRename(name_buf));
                }
            });

            // ── Plane (read-only) ─────────────────────────────────────
            ui.horizontal(|ui| {
                ui.strong("Plane");
                ui.label(format!("{:?}", sk.plane));
            });

            ui.separator();

            // ── Elements ──────────────────────────────────────────────
            ui.strong("Elements");
            let n = sk.points.len();
            if n == 0 && sk.committed_profiles.is_empty() {
                ui.label(egui::RichText::new("(none)").italics().weak());
            } else {
                // Detect selection changes so we can scroll the new item into view.
                let prev_sel_id = panel_id.with("prev_seg_sel");
                let prev_sel: Vec<usize> = ctx.data(|d| d.get_temp(prev_sel_id).unwrap_or_default());
                let cur_seg_sel: Vec<usize> = sk.sel_segs().collect();
                let sel_changed = prev_sel != cur_seg_sel;
                ctx.data_mut(|d| d.insert_temp(prev_sel_id, cur_seg_sel.clone()));

                // Elements highlighted by currently selected constraints.
                let mut constraint_hi_pts:  std::collections::HashSet<usize> = Default::default();
                let mut constraint_hi_segs: std::collections::HashSet<usize> = Default::default();
                for &ci in &sk.constraint_selection {
                    if let Some(c) = sk.constraints.get(ci) {
                        let (pts, segs) = constraint_element_refs(c);
                        constraint_hi_pts.extend(pts);
                        constraint_hi_segs.extend(segs);
                    }
                }
                let highlight_color = egui::Color32::from_rgb(255, 160, 60);

                egui::ScrollArea::vertical()
                    .id_salt("sketch_elements_scroll")
                    .max_height(f32::INFINITY)
                    .min_scrolled_height(60.0)
                    .show(ui, |ui| {
                        ui.set_min_width(ui.available_width());

                        for i in 0..n {
                            let p = &sk.points[i];
                            let selected = sk.sel_pts().any(|s| s == i);
                            let is_hi = !selected && (snap_vertex == Some(i) || constraint_hi_pts.contains(&i));
                            let text = egui::RichText::new(
                                format!("Point {}  ({:.2}, {:.2}, {:.2})", i, p.x, p.y, p.z)
                            );
                            let text = if is_hi { text.color(highlight_color) } else { text };
                            let resp = ui.selectable_label(selected, text);
                            if resp.clicked() {
                                actions.push(UiAction::SketchPanelSelectVertex(i));
                            }
                            // Hovering a point highlights its constraints and marks it for viewport glow.
                            if resp.hovered() && !selected {
                                hi.hi_pts.insert(i);
                                for (ci, c) in sk.constraints.iter().enumerate() {
                                    if constraint_element_refs(c).0.contains(&i) {
                                        hi.hi_constraints.insert(ci);
                                    }
                                }
                            }
                        }

                        let n_segs = if sk.closed { n } else { n.saturating_sub(1) };
                        for i in 0..n_segs {
                            let j = (i + 1) % n;
                            let selected = sk.sel_segs().any(|s| s == i);
                            let is_hi = !selected && (snap_segment == Some(i) || constraint_hi_segs.contains(&i));
                            let text = egui::RichText::new(format!("Segment {}  ({} → {})", i, i, j));
                            let text = if is_hi { text.color(highlight_color) } else { text };
                            let resp = ui.selectable_label(selected, text);
                            if resp.clicked() {
                                actions.push(UiAction::SketchPanelSelectSegment(i));
                            }
                            if selected && sel_changed && !cur_seg_sel.contains(&i) {
                                resp.scroll_to_me(Some(egui::Align::Center));
                            }
                            // Hovering a segment highlights its constraints and marks it for viewport glow.
                            if resp.hovered() && !selected {
                                hi.hi_segs.insert(i);
                                for (ci, c) in sk.constraints.iter().enumerate() {
                                    if constraint_element_refs(c).1.contains(&i) {
                                        hi.hi_constraints.insert(ci);
                                    }
                                }
                            }
                        }

                        // Committed profiles (circles, arcs, segments, etc.).
                        for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                            let label = cp.shape.label(pi);
                            let is_sel = sk.sel_committed_profile() == Some(pi);
                            let is_hi = !is_sel && (
                                snap_committed_curve == Some(pi)
                                || snap_committed.map(|(p, _)| p) == Some(pi)
                                || snap_committed_seg.map(|(p, _)| p) == Some(pi)
                            );
                            let text = egui::RichText::new(label);
                            let text = if is_hi { text.color(highlight_color) } else { text };
                            let resp = ui.selectable_label(is_sel, text);
                            if resp.clicked() {
                                let new_sel = if is_sel { None } else { Some(pi) };
                                actions.push(UiAction::SketchSelectCommitted(new_sel));
                            }
                            // Hovering a committed profile marks it for viewport glow.
                            if resp.hovered() && !is_sel {
                                hi.hi_committed.insert(pi);
                            }
                        }
                    });
            }

            ui.separator();

            // ── Constraints ───────────────────────────────────────────
            ui.strong("Constraints");
            let has_any_constraints = !sk.constraints.is_empty()
                || sk.committed_profiles.iter().any(|cp| !cp.constraints.is_empty())
                || !sk.cross_constraints.is_empty();
            if !has_any_constraints {
                ui.label(egui::RichText::new("(none)").italics().weak());
            } else {
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
                                ui.horizontal(|ui| {
                                    ui.add(list_icon(constraint_icon(c)));
                                    ui.label(format!("{} — {}", constraint_text(c), profile_label));
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.add_space(12.0);
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
                                } else {
                                    ui.visuals().text_color()
                                };
                                child.add(
                                    egui::Label::new(
                                        egui::RichText::new(cross_constraint_text(cc))
                                            .color(text_color),
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

            ui.separator();

            // ── Actions ───────────────────────────────────────────────
            ui.strong("Actions");
            if sk.history.undo_stack.is_empty() {
                ui.label(egui::RichText::new("(none)").italics().weak());
            } else {
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
        });

    if let Some(state) = egui::AreaState::load(ctx, panel_id) {
        ctx.data_mut(|d| d.insert_temp(panel_id, state.left_top_pos()));
    }

    (actions, hi)
}
