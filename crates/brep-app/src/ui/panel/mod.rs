//! Floating sketch info panel (elements, constraints, actions).

mod constraints;
mod elements;
mod history;

use crate::editor::{EditorState, UiAction};

use super::constraint_helpers::{constraint_element_refs, cross_constraint_element_refs, snap_to_viewport};

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
    /// Per-profile constraint (profile_idx, constraint_idx) pairs to highlight.
    pub hi_committed_constraints: std::collections::HashSet<(usize, usize)>,
    /// Cross-profile constraint indices to highlight.
    pub hi_cross_constraints: std::collections::HashSet<usize>,
}

// ── Panel ─────────────────────────────────────────────────────────────────────

/// Floating sketch configuration panel.  Returns actions and viewport highlights.
pub(super) fn draw_sketch_info_panel(
    ctx: &egui::Context,
    editor: &EditorState,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
    snap_constraint: Option<usize>,
    snap_committed: Option<(usize, usize)>,
    snap_committed_curve: Option<usize>,
    snap_committed_seg: Option<(usize, usize)>,
) -> (Vec<UiAction>, PanelHighlights) {
    let mut actions: Vec<UiAction> = Vec::new();
    let mut hi = PanelHighlights::default();
    let Some(sk) = &editor.sketch else { return (actions, hi) };

    // Seed constraint highlights from whatever is snapped in the viewport.
    if let Some(sc) = snap_constraint {
        hi.hi_constraints.insert(sc);
    }
    if let Some(si) = snap_segment {
        let n = sk.points.len();
        let pt_a = si;
        let pt_b = (si + 1) % n;
        for (ci, c) in sk.constraints.iter().enumerate() {
            let (pts, segs) = constraint_element_refs(c);
            // Segment-based constraint referencing this segment directly.
            if segs.contains(&si) {
                hi.hi_constraints.insert(ci);
            }
            // Point-pair constraints (HorizontalPair / VerticalPair) whose two
            // referenced points are the endpoints of this segment.
            if pts.len() == 2 && pts.contains(&pt_a) && pts.contains(&pt_b) {
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

    // Seed per-profile and cross-profile highlights from committed-profile snap.
    if let Some((pi, si)) = snap_committed_seg {
        if let Some(cp) = sk.committed_profiles.get(pi) {
            for (ci, c) in cp.constraints.iter().enumerate() {
                let (pts, segs) = constraint_element_refs(c);
                if segs.contains(&si) {
                    hi.hi_committed_constraints.insert((pi, ci));
                }
                // HorizontalPair / VerticalPair on a committed segment: pt_a=si, pt_b=si+1.
                if pts.len() == 2 && pts.contains(&si) && pts.contains(&(si + 1)) {
                    hi.hi_committed_constraints.insert((pi, ci));
                }
            }
        }
        for (i, cc) in sk.cross_constraints.iter().enumerate() {
            if cross_constraint_element_refs(cc).1.contains(&(pi, si)) {
                hi.hi_cross_constraints.insert(i);
            }
        }
    }
    if let Some((pi, vi)) = snap_committed {
        if let Some(cp) = sk.committed_profiles.get(pi) {
            for (ci, c) in cp.constraints.iter().enumerate() {
                if constraint_element_refs(c).0.contains(&vi) {
                    hi.hi_committed_constraints.insert((pi, ci));
                }
            }
        }
        for (i, cc) in sk.cross_constraints.iter().enumerate() {
            if cross_constraint_element_refs(cc).0.contains(&(pi, vi)) {
                hi.hi_cross_constraints.insert(i);
            }
        }
    }
    if let Some(pi) = snap_committed_curve {
        // Hovering the curve itself — highlight all constraints for this profile.
        if let Some(cp) = sk.committed_profiles.get(pi) {
            for ci in 0..cp.constraints.len() {
                hi.hi_committed_constraints.insert((pi, ci));
            }
        }
        for (i, cc) in sk.cross_constraints.iter().enumerate() {
            let (verts, segs) = cross_constraint_element_refs(cc);
            if verts.iter().any(|(p, _)| *p == pi) || segs.iter().any(|(p, _)| *p == pi) {
                hi.hi_cross_constraints.insert(i);
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
            elements::draw_elements_list(
                ui, ctx, panel_id, sk,
                snap_vertex, snap_segment,
                snap_committed, snap_committed_curve, snap_committed_seg,
                &mut actions, &mut hi,
            );

            ui.separator();

            // ── Constraints ───────────────────────────────────────────
            ui.strong("Constraints");
            constraints::draw_constraints_list(ui, sk, &mut actions, &mut hi);

            ui.separator();

            // ── Actions ───────────────────────────────────────────────
            ui.strong("Actions");
            history::draw_history_list(ui, sk, &mut actions);
        });

    if let Some(state) = egui::AreaState::load(ctx, panel_id) {
        ctx.data_mut(|d| d.insert_temp(panel_id, state.left_top_pos()));
    }

    (actions, hi)
}
