//! Viewport sketch overlay and finished-sketch outline rendering.

use brep_core::Point3;

use crate::editor::{EditorState, RefEntity, SceneEntry, SketchState, ToolInProgress, UiAction, ViewportCamera};

use super::constraint_helpers::draw_constraint_marker;

// ── Committed profile helper ──────────────────────────────────────────────────

fn draw_committed_profile(
    painter: &egui::Painter,
    pts: &[Point3],
    cp: &crate::editor::CommittedProfile,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    cp.shape.draw(pts, cp.closed, cp.plane, painter, proj, stroke, cp.arc_reversed);
}

// ── Active sketch overlay ─────────────────────────────────────────────────────

/// Draw all sketch geometry (active profile, committed profiles, constraint markers, preview)
/// as an egui layer over the 3-D viewport.  Returns any actions triggered by context menus.
pub(super) fn draw_sketch_overlay(
    ctx: &egui::Context,
    sk: &SketchState,
    viewport: (u32, u32),
    cam: &ViewportCamera,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
    snap_ref: Option<RefEntity>,
    snap_constraint: Option<usize>,
    snap_committed: Option<(usize, usize)>,
    snap_committed_curve: Option<usize>,
    snap_committed_seg: Option<(usize, usize)>,
    sketch_cursor: Option<Point3>,
    context_menu: Option<((f32, f32), usize)>,
) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    let (w, h) = viewport;
    let painter = ctx.layer_painter(
        egui::LayerId::new(egui::Order::Background, "sketch_overlay".into()),
    );

    // project_to_screen returns physical pixel coords; egui painter uses logical points.
    let ppp = ctx.pixels_per_point();
    let proj = |p: Point3| -> Option<egui::Pos2> {
        cam.project_to_screen(p, w, h).map(|(x, y)| egui::pos2(x / ppp, y / ppp))
    };

    let edge_stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 200, 255));
    let preview_stroke = egui::Stroke::new(
        1.5,
        egui::Color32::from_rgba_unmultiplied(200, 230, 255, 140),
    );

    // Draw origin and axis lines.
    {
        let origin_world = sk.plane.origin();
        let (u_axis, v_axis) = sk.plane.uv_axes();
        let far = 500.0_f64;

        let x_selected = sk.sel_ref() == Some(RefEntity::XAxis);
        let y_selected = sk.sel_ref() == Some(RefEntity::YAxis);
        let o_selected = sk.sel_ref() == Some(RefEntity::Origin);

        let x_col = if x_selected {
            egui::Color32::from_rgb(255, 100, 100)
        } else if snap_ref == Some(RefEntity::XAxis) {
            egui::Color32::from_rgb(255, 140, 140)
        } else {
            egui::Color32::from_rgba_unmultiplied(180, 60, 60, 100)
        };
        let y_col = if y_selected {
            egui::Color32::from_rgb(80, 210, 80)
        } else if snap_ref == Some(RefEntity::YAxis) {
            egui::Color32::from_rgb(120, 230, 120)
        } else {
            egui::Color32::from_rgba_unmultiplied(50, 150, 50, 100)
        };

        if let (Some(a), Some(b)) = (
            proj(origin_world - u_axis * far),
            proj(origin_world + u_axis * far),
        ) {
            painter.line_segment([a, b], egui::Stroke::new(if x_selected { 1.5 } else { 1.0 }, x_col));
        }
        if let (Some(a), Some(b)) = (
            proj(origin_world - v_axis * far),
            proj(origin_world + v_axis * far),
        ) {
            painter.line_segment([a, b], egui::Stroke::new(if y_selected { 1.5 } else { 1.0 }, y_col));
        }
        if let Some(op) = proj(origin_world) {
            let o_col = if o_selected {
                egui::Color32::from_rgb(255, 200, 60)
            } else if snap_ref == Some(RefEntity::Origin) {
                egui::Color32::WHITE
            } else {
                egui::Color32::from_rgba_unmultiplied(200, 200, 200, 140)
            };
            let sz   = if o_selected || snap_ref == Some(RefEntity::Origin) { 6.0 } else { 4.5 };
            let w_px = if o_selected { 2.0 } else { 1.5 };
            painter.line_segment(
                [op - egui::vec2(sz, 0.0), op + egui::vec2(sz, 0.0)],
                egui::Stroke::new(w_px, o_col),
            );
            painter.line_segment(
                [op - egui::vec2(0.0, sz), op + egui::vec2(0.0, sz)],
                egui::Stroke::new(w_px, o_col),
            );
        }
    }

    // Draw committed profiles first.
    let committed_sel = sk.sel_committed_profile();
    for (pi, cp) in sk.committed_profiles.iter().enumerate() {
        let is_sel         = committed_sel == Some(pi);
        let is_curve_hov   = snap_committed_curve == Some(pi);
        let cp_stroke = if is_sel {
            egui::Stroke::new(edge_stroke.width + 1.0, egui::Color32::from_rgb(255, 160, 60))
        } else if is_curve_hov {
            egui::Stroke::new(edge_stroke.width + 1.0, egui::Color32::from_rgba_unmultiplied(100, 200, 255, 210))
        } else {
            edge_stroke
        };
        let pts = crate::editor::resolved_points(cp, &sk.global_points);
        draw_committed_profile(&painter, &pts, cp, &proj, cp_stroke);
    }

    // Snap highlight on committed profile vertices.
    if let Some((snap_pi, snap_vi)) = snap_committed {
        if let Some(cp) = sk.committed_profiles.get(snap_pi) {
            if let Some(&gi) = cp.point_indices.get(snap_vi) {
                if let Some(p) = proj(sk.global_points[gi]) {
                    painter.circle_stroke(p, 7.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
                    painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
                }
            }
        }
    }

    // Selected committed points — orange dots.
    for (sel_pi, sel_vi) in sk.sel_committed_pts() {
        if let Some(cp) = sk.committed_profiles.get(sel_pi) {
            if let Some(&gi) = cp.point_indices.get(sel_vi) {
                if let Some(p) = proj(sk.global_points[gi]) {
                    painter.circle_filled(p, 6.0, egui::Color32::from_rgb(255, 160, 60));
                }
            }
        }
    }

    // Hovered committed polyline segment — blue highlight.
    if let Some((hov_pi, hov_si)) = snap_committed_seg {
        if let Some(cp) = sk.committed_profiles.get(hov_pi) {
            let cn = cp.point_indices.len();
            if hov_si < cn {
                let a_pt = sk.global_points[cp.point_indices[hov_si]];
                let b_pt = sk.global_points[cp.point_indices[(hov_si + 1) % cn]];
                if let (Some(a), Some(b)) = (proj(a_pt), proj(b_pt)) {
                    painter.line_segment(
                        [a, b],
                        egui::Stroke::new(edge_stroke.width + 1.5, egui::Color32::from_rgba_unmultiplied(100, 200, 255, 210)),
                    );
                }
            }
        }
    }

    // Selected committed polyline segments — orange highlight.
    for (sel_pi, sel_si) in sk.sel_committed_segs() {
        if let Some(cp) = sk.committed_profiles.get(sel_pi) {
            let cn = cp.point_indices.len();
            if sel_si < cn {
                let a_pt = sk.global_points[cp.point_indices[sel_si]];
                let b_pt = sk.global_points[cp.point_indices[(sel_si + 1) % cn]];
                if let (Some(a), Some(b)) = (proj(a_pt), proj(b_pt)) {
                    painter.line_segment(
                        [a, b],
                        egui::Stroke::new(edge_stroke.width + 1.5, egui::Color32::from_rgb(255, 160, 60)),
                    );
                }
            }
        }
    }

    // Draw active profile edges.
    let n = sk.points.len();
    let seg_count = if sk.closed { n } else { n.saturating_sub(1) };
    for i in 0..seg_count {
        if let (Some(a), Some(b)) = (proj(sk.points[i]), proj(sk.points[(i + 1) % n])) {
            painter.line_segment([a, b], edge_stroke);
        }
    }

    // Hovered segment highlight.
    if let Some(si) = snap_segment {
        if si < seg_count {
            if let (Some(a), Some(b)) = (proj(sk.points[si]), proj(sk.points[(si + 1) % n])) {
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(3.5, egui::Color32::from_rgba_unmultiplied(100, 200, 255, 210)),
                );
            }
        }
    }

    // Selected segment highlight (orange).
    let sel_stroke = egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 160, 60));
    for si in sk.sel_segs() {
        if si < seg_count {
            if let (Some(a), Some(b)) = (proj(sk.points[si]), proj(sk.points[(si + 1) % n])) {
                painter.line_segment([a, b], sel_stroke);
            }
        }
    }

    // Constraint markers.
    for (i, c) in sk.constraints.iter().enumerate() {
        let selected = sk.constraint_selection.contains(&i);
        let hovered  = snap_constraint == Some(i);
        draw_constraint_marker(&painter, c, &sk.points, n, &proj, selected, hovered);
    }

    // Preview — shows what will be committed on the next click.
    if !sk.closed {
        let cursor_snapped = snap_vertex
            .and_then(|i| sk.points.get(i).copied())
            .or(sketch_cursor);

        match &sk.tool_in_progress {
            None => {
                if let (Some(last), Some(cur)) = (sk.points.last(), cursor_snapped) {
                    if let (Some(a), Some(b)) = (proj(*last), proj(cur)) {
                        painter.line_segment([a, b], preview_stroke);
                    }
                }
            }
            Some(ToolInProgress::PolylineChain { pen_global_idx, .. }) => {
                if let Some(cur) = cursor_snapped {
                    let pen = sk.global_points[*pen_global_idx];
                    if let (Some(a), Some(b)) = (proj(pen), proj(cur)) {
                        painter.line_segment([a, b], preview_stroke);
                    }
                }
            }
            Some(ToolInProgress::Arc1 { start }) => {
                if let Some(sp) = proj(*start) {
                    painter.circle_stroke(
                        sp, 5.0,
                        egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(100, 255, 180, 200)),
                    );
                }
                if let Some(cur) = cursor_snapped {
                    if let (Some(a), Some(b)) = (proj(*start), proj(cur)) {
                        painter.line_segment([a, b], preview_stroke);
                    }
                }
            }
            Some(ToolInProgress::Arc2 { start, end_pt }) => {
                if let Some(cur) = cursor_snapped {
                    crate::sketch_tools::arc::draw_preview(
                        &painter, *start, *end_pt, cur, sk.plane, &proj, preview_stroke,
                    );
                }
                if let Some(tp) = proj(*end_pt) {
                    painter.circle_stroke(
                        tp, 5.0,
                        egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(100, 255, 180, 200)),
                    );
                }
            }
            Some(ToolInProgress::RectFirst { corner }) => {
                if let Some(cur) = cursor_snapped {
                    crate::sketch_tools::rectangle::draw_preview(
                        &painter, *corner, cur, sk.plane, &proj, preview_stroke,
                    );
                }
            }
            Some(ToolInProgress::CircleCenter { center }) => {
                if let Some(cur) = cursor_snapped {
                    crate::sketch_tools::circle::draw_preview(
                        &painter, *center, cur, sk.plane, &proj, preview_stroke,
                    );
                }
            }
        }
    }

    // Vertex dots — highlight snapped/selected vertices.
    let can_close = !sk.closed && n >= 3 && sk.tool_in_progress.is_none();
    for (i, &pt) in sk.points.iter().enumerate() {
        if let Some(p) = proj(pt) {
            let is_snap     = snap_vertex == Some(i);
            let is_selected = sk.sel_pts().any(|s| s == i);
            let is_close_hint = can_close && i == 0;
            if is_selected {
                painter.circle_filled(p, 6.0, egui::Color32::from_rgb(255, 160, 60));
            } else if is_snap && is_close_hint {
                painter.circle_stroke(p, 8.0, egui::Stroke::new(2.5, egui::Color32::from_rgb(80, 220, 80)));
                painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
            } else if is_snap {
                painter.circle_stroke(p, 7.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
                painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
            } else if is_close_hint {
                painter.circle_stroke(p, 6.0, egui::Stroke::new(1.5, egui::Color32::from_rgb(80, 220, 80)));
                painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
            } else {
                painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
            }
        }
    }

    // Cursor dot — only shown while actively drawing (not in pointer mode).
    if snap_vertex.is_none() && sk.active_tool != crate::editor::DrawTool::Pointer {
        if let Some(c) = sketch_cursor {
            if let Some(p) = proj(c) {
                painter.circle_filled(p, 3.0, egui::Color32::WHITE);
            }
        }
    }

    // Arc context menu (right-click → "Reverse arc direction").
    if let Some(((px, py), pi)) = context_menu {
        let is_arc = sk.committed_profiles.get(pi)
            .map_or(false, |cp| matches!(cp.shape, crate::profile_shapes::ProfileShape::Arc));
        if is_arc {
            let pos = egui::Pos2::new(px / ppp, py / ppp);
            egui::Area::new(egui::Id::new("arc_context_menu"))
                .fixed_pos(pos)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| {
                        if ui.button("Reverse arc direction").clicked() {
                            actions.push(UiAction::SketchReverseArc(pi));
                        }
                    });
                });
        }
    }

    actions
}

// ── Finished sketch outlines ──────────────────────────────────────────────────

/// Draw faint outlines of finished (non-editing) sketches when not in sketch mode.
pub(super) fn draw_sketch_outlines(
    ctx: &egui::Context,
    editor: &EditorState,
    viewport: (u32, u32),
) {
    let (w, h) = viewport;
    let cam = &editor.camera;
    let ppp = ctx.pixels_per_point();
    let proj = |p: Point3| -> Option<egui::Pos2> {
        cam.project_to_screen(p, w, h).map(|(x, y)| egui::pos2(x / ppp, y / ppp))
    };
    let painter = ctx.layer_painter(
        egui::LayerId::new(egui::Order::Background, "sketch_outlines".into()),
    );
    for (i, entry) in editor.entries.iter().enumerate() {
        if let SceneEntry::Sketch(sk) = entry {
            let is_selected = editor.selection.contains(&i);
            let stroke = if is_selected {
                egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 200, 255))
            } else {
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(100, 160, 200, 80))
            };
            let n = sk.points.len();
            for j in 0..n {
                if let (Some(a), Some(b)) = (proj(sk.points[j]), proj(sk.points[(j + 1) % n])) {
                    painter.line_segment([a, b], stroke);
                }
            }
            for cp in &sk.extra_profiles {
                draw_committed_profile(&painter, &cp.points, cp, &proj, stroke);
            }
            if is_selected {
                for &pt in &sk.points {
                    if let Some(p) = proj(pt) {
                        painter.circle_filled(p, 3.5, egui::Color32::from_rgb(100, 200, 255));
                    }
                }
            }
        }
    }
}

// ── Viewport axis lines ───────────────────────────────────────────────────────

/// Draw X/Y/Z axis lines from the world origin into the central viewport area.
pub(super) fn draw_viewport_axes(ctx: &egui::Context, editor: &EditorState) {
    let Some(viewport) = ctx.input(|i| i.screen_rect().is_finite().then(|| i.screen_rect())) else { return };

    let cam = &editor.camera;
    let eye = cam.eye();
    use nalgebra::Matrix4;
    let f = (cam.target - eye).normalize();
    let r = f.cross(&brep_core::Vec3::z()).normalize();
    let u = r.cross(&f);
    let view = Matrix4::<f64>::new(
         r.x,  r.y,  r.z, -r.dot(&eye.coords),
         u.x,  u.y,  u.z, -u.dot(&eye.coords),
        -f.x, -f.y, -f.z,  f.dot(&eye.coords),
         0.0,  0.0,  0.0,  1.0,
    );
    let w = viewport.width() as f64;
    let h = viewport.height() as f64;
    let aspect = w / h;
    let fov = cam.fov_y_deg.to_radians();
    let fc = 1.0 / (fov * 0.5).tan();
    let (near, far) = (0.05, 1000.0_f64);
    let proj_mat = Matrix4::<f64>::new(
        fc / aspect, 0.0,  0.0,                    0.0,
        0.0,         fc,   0.0,                    0.0,
        0.0,         0.0,  far / (near - far),      far * near / (near - far),
        0.0,         0.0, -1.0,                     0.0,
    );
    let vp = proj_mat * view;

    let project = |pt: brep_core::Point3| -> Option<egui::Pos2> {
        let clip = vp * nalgebra::Vector4::new(pt.x, pt.y, pt.z, 1.0);
        if clip.w <= 0.0 { return None; }
        let ndcx = clip.x / clip.w;
        let ndcy = clip.y / clip.w;
        Some(egui::pos2(
            (( ndcx + 1.0) * 0.5 * w) as f32 + viewport.left(),
            ((-ndcy + 1.0) * 0.5 * h) as f32 + viewport.top(),
        ))
    };

    let origin = brep_core::Point3::origin();
    let axes: &[(brep_core::Point3, egui::Color32, &str)] = &[
        (origin + brep_core::Vec3::x(), egui::Color32::from_rgb(220, 70,  70),  "X"),
        (origin + brep_core::Vec3::y(), egui::Color32::from_rgb(80,  200, 80),  "Y"),
        (origin + brep_core::Vec3::z(), egui::Color32::from_rgb(80,  130, 230), "Z"),
    ];

    let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("vp_axes")));
    let Some(o2) = project(origin) else { return };

    for &(end_pt, color, label) in axes {
        let Some(e2) = project(end_pt) else { continue };
        painter.line_segment([o2, e2], egui::Stroke::new(1.5, color));
        painter.text(
            e2,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::proportional(11.0),
            color,
        );
    }
}
