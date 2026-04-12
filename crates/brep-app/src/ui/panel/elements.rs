//! Elements list section of the sketch info panel.

use crate::editor::{SketchState, UiAction};
use crate::ui::constraint_helpers::{constraint_element_refs, cross_constraint_element_refs};

pub(super) fn draw_elements_list(
    ui: &mut egui::Ui,
    ctx: &egui::Context,
    panel_id: egui::Id,
    sk: &SketchState,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
    snap_committed: Option<(usize, usize)>,
    snap_committed_curve: Option<usize>,
    snap_committed_seg: Option<(usize, usize)>,
    actions: &mut Vec<UiAction>,
    hi: &mut super::PanelHighlights,
) {
    let n = sk.points.len();
    if n == 0 && sk.committed_profiles.is_empty() {
        ui.label(egui::RichText::new("(none)").italics().weak());
        return;
    }

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
                    let pt_a = i;
                    let pt_b = (i + 1) % n;
                    for (ci, c) in sk.constraints.iter().enumerate() {
                        let (pts, segs) = constraint_element_refs(c);
                        if segs.contains(&i) {
                            hi.hi_constraints.insert(ci);
                        }
                        // HorizontalPair / VerticalPair reference endpoints, not segment index.
                        if pts.len() == 2 && pts.contains(&pt_a) && pts.contains(&pt_b) {
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
                // Hovering a committed profile marks it for viewport glow and
                // highlights all constraints referencing it.
                if resp.hovered() && !is_sel {
                    hi.hi_committed.insert(pi);
                    for ci in 0..cp.constraints.len() {
                        hi.hi_committed_constraints.insert((pi, ci));
                    }
                    for (i, cc) in sk.cross_constraints.iter().enumerate() {
                        let (verts, segs) = cross_constraint_element_refs(cc);
                        if verts.iter().any(|(p, _)| *p == pi) || segs.iter().any(|(p, _)| *p == pi) {
                            hi.hi_cross_constraints.insert(i);
                        }
                    }
                }
            }
        });
}
