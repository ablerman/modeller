//! egui panel layout: toolbar, object list, properties.

use brep_core::Point3;
use brep_sketch::SketchConstraint;
use egui::Context;

use crate::editor::{EditorState, LengthTarget, ObjectHistory, PrimitiveKind, RefEntity, SceneEntry, SketchState, ToolInProgress, UiAction};
use crate::icons::{icon_angle, icon_coincident, icon_equal_len, icon_horizontal, icon_length, icon_parallel, icon_perp, icon_trash, icon_vertical};
use crate::toolbar::IconSize;
use crate::toolbar_defs;

/// Icon image sized for use in constraint list labels.
fn list_icon(src: egui::ImageSource<'static>) -> egui::Image<'static> {
    egui::Image::new(src).fit_to_exact_size(egui::vec2(14.0, 14.0))
}

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
    angle_dialog: Option<(usize, usize)>,
    length_dialog: Option<(LengthTarget, f64)>,
) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    // ── Top toolbar ───────────────────────────────────────────────────────────
    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            if let Some(sk) = &editor.sketch {
                // ── Sketch toolbar ────────────────────────────────────────────
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

                // ── Constraint dropdown ───────────────────────────────────
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

                    // Active constraints list.
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
                // ── Normal toolbar (data-driven) ──────────────────────────────
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
                            // Legacy: sketch+extrude solids loaded from old saves.
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

        // ── Properties (merged into left panel) ───────────────────────────
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

    // ── Viewport axes (drawn into the 3-D area via egui painter) ─────────────
    draw_viewport_axes(ctx, editor);

    // ── Orientation cube gizmo (top-right, draggable) ─────────────────────────
    if let Some(snap) = crate::gizmo::draw_orientation_cube(ctx, editor) {
        actions.push(snap);
    }

    // ── Sketch info panel (top-left of viewport, sketch mode only) ────────────
    actions.extend(draw_sketch_info_panel(ctx, editor));

    // ── Sketch viewport toolbar (floating, top-center of canvas) ─────────────
    actions.extend(draw_sketch_viewport_toolbar(ctx, editor));

    // ── Sketch overlay ────────────────────────────────────────────────────────
    if let Some(sk) = &editor.sketch {
        let (w, h) = viewport;
        let cam = &editor.camera;
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

            let x_selected = sk.ref_selection == Some(RefEntity::XAxis);
            let y_selected = sk.ref_selection == Some(RefEntity::YAxis);
            let o_selected = sk.ref_selection == Some(RefEntity::Origin);

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

            // X-axis line
            if let (Some(a), Some(b)) = (
                proj(origin_world - u_axis * far),
                proj(origin_world + u_axis * far),
            ) {
                painter.line_segment([a, b], egui::Stroke::new(if x_selected { 1.5 } else { 1.0 }, x_col));
            }
            // Y-axis line
            if let (Some(a), Some(b)) = (
                proj(origin_world - v_axis * far),
                proj(origin_world + v_axis * far),
            ) {
                painter.line_segment([a, b], egui::Stroke::new(if y_selected { 1.5 } else { 1.0 }, y_col));
            }
            // Origin cross marker
            if let Some(op) = proj(origin_world) {
                let o_col = if o_selected {
                    egui::Color32::from_rgb(255, 200, 60)
                } else if snap_ref == Some(RefEntity::Origin) {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_rgba_unmultiplied(200, 200, 200, 140)
                };
                let sz = if o_selected || snap_ref == Some(RefEntity::Origin) { 6.0 } else { 4.5 };
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

        // Draw committed profiles (circles, arcs, rectangles, etc.) first.
        let committed_sel = sk.committed_selection;
        for (pi, cp) in sk.committed_profiles.iter().enumerate() {
            let is_sel = committed_sel == Some(pi);
            let is_curve_hovered = snap_committed_curve == Some(pi);
            let cp_stroke = if is_sel {
                egui::Stroke::new(edge_stroke.width + 1.0, egui::Color32::from_rgb(255, 160, 60))
            } else if is_curve_hovered {
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
        // Selected committed points — highlighted as orange dots.
        for &(sel_pi, sel_vi) in &sk.committed_pt_selection {
            if let Some(cp) = sk.committed_profiles.get(sel_pi) {
                if let Some(&gi) = cp.point_indices.get(sel_vi) {
                    if let Some(p) = proj(sk.global_points[gi]) {
                        painter.circle_filled(p, 6.0, egui::Color32::from_rgb(255, 160, 60));
                    }
                }
            }
        }
        // Hovered committed polyline segment — blue highlight over the base rendering.
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
        for &(sel_pi, sel_si) in &sk.committed_seg_selection {
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
        for &si in &sk.seg_selection {
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
                    // Active polyline: line from last active-profile point to cursor.
                    if let (Some(last), Some(cur)) = (sk.points.last(), cursor_snapped) {
                        if let (Some(a), Some(b)) = (proj(*last), proj(cur)) {
                            painter.line_segment([a, b], preview_stroke);
                        }
                    }
                }
                Some(ToolInProgress::PolylineChain { pen_global_idx, .. }) => {
                    // Per-segment polyline chain: line from pen point to cursor.
                    if let Some(cur) = cursor_snapped {
                        let pen = sk.global_points[*pen_global_idx];
                        if let (Some(a), Some(b)) = (proj(pen), proj(cur)) {
                            painter.line_segment([a, b], preview_stroke);
                        }
                    }
                }
                Some(ToolInProgress::Arc1 { start }) => {
                    // Arc start placed: mark it and draw a line to the cursor (= future through-point).
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
                    // start + end placed; next click = arc center.
                    // Preview: arc from start to end_pt with cursor as center.
                    if let Some(cur) = cursor_snapped {
                        crate::sketch_tools::arc::draw_preview(&painter, *start, *end_pt, cur, sk.plane, &proj, preview_stroke);
                    }
                    // Mark the end point.
                    if let Some(tp) = proj(*end_pt) {
                        painter.circle_stroke(
                            tp, 5.0,
                            egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(100, 255, 180, 200)),
                        );
                    }
                }
                Some(ToolInProgress::RectFirst { corner }) => {
                    // First corner placed: draw rectangle preview to cursor.
                    if let Some(cur) = cursor_snapped {
                        crate::sketch_tools::rectangle::draw_preview(&painter, *corner, cur, sk.plane, &proj, preview_stroke);
                    }
                }
                Some(ToolInProgress::CircleCenter { center }) => {
                    // Center placed: draw circle preview to cursor.
                    if let Some(cur) = cursor_snapped {
                        crate::sketch_tools::circle::draw_preview(&painter, *center, cur, sk.plane, &proj, preview_stroke);
                    }
                }
            }
        }
        // Vertex dots — highlight snapped/selected vertices.
        // Suppress the close-loop hint while a multi-step tool is in progress.
        let can_close = !sk.closed && n >= 3 && sk.tool_in_progress.is_none();
        for (i, &pt) in sk.points.iter().enumerate() {
            if let Some(p) = proj(pt) {
                let is_snap = snap_vertex == Some(i);
                let is_selected = sk.pt_selection.contains(&i);
                // Green ring on vertex 0 hints that clicking it will close the loop.
                let is_close_hint = can_close && i == 0;
                if is_selected {
                    painter.circle_filled(p, 6.0, egui::Color32::from_rgb(255, 160, 60));
                } else if is_snap && is_close_hint {
                    // Green ring when hovering the close-loop vertex.
                    painter.circle_stroke(p, 8.0, egui::Stroke::new(2.5, egui::Color32::from_rgb(80, 220, 80)));
                    painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
                } else if is_snap {
                    painter.circle_stroke(p, 7.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
                    painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
                } else if is_close_hint {
                    // Subtle green tint when not hovering but loop can be closed.
                    painter.circle_stroke(p, 6.0, egui::Stroke::new(1.5, egui::Color32::from_rgb(80, 220, 80)));
                    painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
                } else {
                    painter.circle_filled(p, 4.0, egui::Color32::YELLOW);
                }
            }
        }
        // Cursor dot — hidden while snapping.
        if snap_vertex.is_none() {
            if let Some(c) = sketch_cursor {
                if let Some(p) = proj(c) {
                    painter.circle_filled(p, 3.0, egui::Color32::WHITE);
                }
            }
        }
    }

    // ── Finished sketch outlines (selected entries highlighted, others dimmed) ──
    if editor.sketch.is_none() {
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
                // Draw primary profile.
                let n = sk.points.len();
                for j in 0..n {
                    if let (Some(a), Some(b)) = (proj(sk.points[j]), proj(sk.points[(j + 1) % n])) {
                        painter.line_segment([a, b], stroke);
                    }
                }
                // Draw extra profiles (circles, arcs, rectangles, etc.).
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

    // ── Length input modal ────────────────────────────────────────────────────
    if let Some((target, initial_len)) = length_dialog {
        let len_id = egui::Id::new("sketch_length_value");
        let len_init_id = egui::Id::new("sketch_length_initialized");
        let description = match target {
            LengthTarget::Segment(seg) => format!("Length of segment {seg}:"),
            LengthTarget::Points(a, b) => format!("Distance between vertex {a} and vertex {b}:"),
        };

        // Seed the input with the actual current length the first time this dialog opens.
        let already_initialized: bool = ctx.data(|d| d.get_temp(len_init_id).unwrap_or(false));
        if !already_initialized {
            ctx.data_mut(|d| {
                d.insert_temp(len_id, initial_len);
                d.insert_temp(len_init_id, true);
            });
        }

        let modal = egui::Modal::new(egui::Id::new("length_input_modal")).show(ctx, |ui| {
            ui.set_min_width(240.0);
            ui.heading("Set Length");
            ui.add_space(4.0);
            ui.label(&description);
            ui.add_space(6.0);

            let mut val: f64 = ctx.data_mut(|d| *d.get_temp_mut_or(len_id, initial_len));
            let resp = ui.add(
                egui::DragValue::new(&mut val)
                    .speed(0.01)
                    .range(1e-3..=1e6)
                    .min_decimals(3),
            );
            if resp.changed() {
                ctx.data_mut(|d| d.insert_temp(len_id, val));
            }

            ui.add_space(10.0);
            ui.horizontal(|ui| {
                let apply = ui.button("Apply");
                let cancel = ui.button("Cancel");

                let confirm = apply.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter));
                let dismiss = cancel.clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape));

                if confirm {
                    let value = ctx.data(|d| d.get_temp(len_id).unwrap_or(initial_len));
                    ctx.data_mut(|d| d.remove::<bool>(len_init_id));
                    match target {
                        LengthTarget::Segment(_) => {
                            // Apply to all currently selected segments (multi-select support).
                            let segs: Vec<usize> = editor.sketch.as_ref()
                                .map(|sk| sk.seg_selection.clone())
                                .unwrap_or_default();
                            let segs = if segs.is_empty() {
                                // Fallback: use the target segment directly.
                                match target { LengthTarget::Segment(s) => vec![s], _ => vec![] }
                            } else { segs };
                            for seg in segs {
                                actions.push(UiAction::SketchAddConstraint(
                                    SketchConstraint::FixedLength { seg, value }
                                ));
                            }
                        }
                        LengthTarget::Points(pt_a, pt_b) => {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::PointDistance { pt_a, pt_b, value }
                            ));
                        }
                    }
                } else if dismiss {
                    ctx.data_mut(|d| d.remove::<bool>(len_init_id));
                    actions.push(UiAction::SketchCancelLengthInput);
                }
            });
        });

        if modal.should_close() {
            ctx.data_mut(|d| d.remove::<bool>(len_init_id));
            actions.push(UiAction::SketchCancelLengthInput);
        }
    }

    // ── Angle input modal ─────────────────────────────────────────────────────
    if let Some((seg_a, seg_b)) = angle_dialog {
        let angle_id = egui::Id::new("sketch_angle_deg");

        let modal = egui::Modal::new(egui::Id::new("angle_input_modal")).show(ctx, |ui| {
            ui.set_min_width(220.0);
            ui.heading("Set Angle");
            ui.add_space(4.0);
            ui.label(format!("Angle between segment {seg_a} and segment {seg_b}:"));
            ui.add_space(6.0);

            let mut deg: f64 = ctx.data_mut(|d| *d.get_temp_mut_or(angle_id, 90.0_f64));
            let resp = ui.add(
                egui::DragValue::new(&mut deg)
                    .speed(1.0)
                    .range(1.0..=179.0)
                    .suffix(" °"),
            );
            if resp.changed() {
                ctx.data_mut(|d| *d.get_temp_mut_or::<f64>(angle_id, 90.0) = deg);
            }

            ui.add_space(10.0);
            ui.horizontal(|ui| {
                let apply = ui.button("Apply");
                let cancel = ui.button("Cancel");

                if apply.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    let degrees = ctx.data_mut(|d| *d.get_temp_mut_or::<f64>(angle_id, 90.0));
                    actions.push(UiAction::SketchAddConstraint(SketchConstraint::Angle {
                        seg_a, seg_b, degrees,
                    }));
                }
                if cancel.clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    actions.push(UiAction::SketchCancelAngleInput);
                }
            });
        });

        if modal.should_close() {
            actions.push(UiAction::SketchCancelAngleInput);
        }
    }

    actions
}

// ── Viewport axis lines ───────────────────────────────────────────────────────

/// Draw X/Y/Z axis lines from the world origin into the central viewport area.
fn draw_viewport_axes(ctx: &egui::Context, editor: &EditorState) {
    let Some(viewport) = ctx.input(|i| i.screen_rect().is_finite().then(|| i.screen_rect())) else { return };

    // Project a world point to egui screen coords (pixels, top-left origin).
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
    let proj = Matrix4::<f64>::new(
        fc / aspect, 0.0,  0.0,                     0.0,
        0.0,         fc,   0.0,                     0.0,
        0.0,         0.0,  far / (near - far),       far * near / (near - far),
        0.0,         0.0, -1.0,                      0.0,
    );
    let vp = proj * view;

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

/// Icon image source for a constraint (used in the constraint list).
fn constraint_icon(c: &SketchConstraint) -> egui::ImageSource<'static> {
    match c {
        SketchConstraint::Parallel { .. }     => icon_parallel(),
        SketchConstraint::Perpendicular { .. } => icon_perp(),
        SketchConstraint::Angle { .. }        => icon_angle(),
        SketchConstraint::Horizontal { .. }   => icon_horizontal(),
        SketchConstraint::Vertical { .. }     => icon_vertical(),
        SketchConstraint::EqualLength { .. }  => icon_equal_len(),
        SketchConstraint::Coincident { .. }   => icon_coincident(),
        SketchConstraint::FixedLength { .. }  => icon_length(),
        SketchConstraint::PointDistance { .. } => icon_length(),
        SketchConstraint::PointFixed { .. }   => icon_coincident(),
        SketchConstraint::PointOnOrigin { .. } => icon_coincident(),
        SketchConstraint::PointOnXAxis { .. }  => icon_horizontal(),
        SketchConstraint::PointOnYAxis { .. }  => icon_vertical(),
        SketchConstraint::HorizontalPair { .. } => icon_horizontal(),
        SketchConstraint::VerticalPair { .. }   => icon_vertical(),
        SketchConstraint::PointOnLine { .. }    => icon_coincident(),
        SketchConstraint::PointOnCircle { .. }  => icon_coincident(),
    }
}

/// Short text description of a constraint (used alongside its icon in the constraint list).
pub(crate) fn constraint_text(c: &SketchConstraint) -> String {
    match c {
        SketchConstraint::Parallel { seg_a, seg_b } =>
            format!("Parallel  {seg_a} ∥ {seg_b}"),
        SketchConstraint::Perpendicular { seg_a, seg_b } =>
            format!("Perpendicular  {seg_a} ⊥ {seg_b}"),
        SketchConstraint::Angle { seg_a, seg_b, degrees } =>
            format!("Angle  {degrees:.0}°  ({seg_a}/{seg_b})"),
        SketchConstraint::Horizontal { seg } =>
            format!("Horizontal  seg {seg}"),
        SketchConstraint::Vertical { seg } =>
            format!("Vertical  seg {seg}"),
        SketchConstraint::EqualLength { seg_a, seg_b } =>
            format!("Equal Length  {seg_a} = {seg_b}"),
        SketchConstraint::Coincident { pt_a, pt_b } =>
            format!("Coincident  pt {pt_a} = {pt_b}"),
        SketchConstraint::FixedLength { seg, value } =>
            format!("Length  seg {seg} = {value:.3}"),
        SketchConstraint::PointDistance { pt_a, pt_b, value } =>
            format!("Distance  pt {pt_a}–{pt_b} = {value:.3}"),
        SketchConstraint::PointFixed { .. }      => String::new(),
        SketchConstraint::PointOnOrigin { pt }    => format!("On Origin  pt {pt}"),
        SketchConstraint::PointOnXAxis { pt }     => format!("On X-axis  pt {pt}"),
        SketchConstraint::PointOnYAxis { pt }     => format!("On Y-axis  pt {pt}"),
        SketchConstraint::HorizontalPair { pt_a, pt_b, .. } => format!("Horizontal  pt {pt_a}–{pt_b}"),
        SketchConstraint::VerticalPair   { pt_a, pt_b, .. } => format!("Vertical  pt {pt_a}–{pt_b}"),
        SketchConstraint::PointOnLine { pt, seg }           => format!("Coincident  pt {pt} on seg {seg}"),
        SketchConstraint::PointOnCircle { pt, .. }          => format!("Coincident  pt {pt} on curve"),
    }
}

/// Draw a small symbol near the segment midpoint to indicate a constraint.
fn draw_constraint_marker(
    painter: &egui::Painter,
    c: &SketchConstraint,
    points: &[brep_core::Point3],
    n: usize,
    proj: &impl Fn(brep_core::Point3) -> Option<egui::Pos2>,
    selected: bool,
    hovered: bool,
) {
    let midpoint = |seg: usize| -> Option<egui::Pos2> {
        let a = proj(points[seg])?;
        let b = proj(points[(seg + 1) % n])?;
        Some(egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5))
    };
    let color = if selected {
        egui::Color32::from_rgb(255, 160, 60)   // orange — matches element selection
    } else if hovered {
        egui::Color32::from_rgb(220, 210, 100)  // bright yellow-gold on hover
    } else {
        egui::Color32::from_rgb(160, 155, 55)   // subdued default
    };
    let font = egui::FontId::proportional(10.0);
    let draw = |seg: usize, sym: &str| {
        if let Some(m) = midpoint(seg) {
            painter.text(m + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, sym, font.clone(), color);
        }
    };
    match c {
        SketchConstraint::Parallel { seg_a, seg_b } => {
            draw(*seg_a, "∥"); draw(*seg_b, "∥");
        }
        SketchConstraint::Perpendicular { seg_a, seg_b } => {
            draw(*seg_a, "⊥"); draw(*seg_b, "⊥");
        }
        SketchConstraint::Angle { seg_a, degrees, .. } => {
            draw(*seg_a, &format!("∠{:.0}°", degrees));
        }
        SketchConstraint::Horizontal { seg } => { draw(*seg, "↔"); }
        SketchConstraint::Vertical { seg } => { draw(*seg, "↕"); }
        SketchConstraint::EqualLength { seg_a, seg_b } => {
            draw(*seg_a, "≡"); draw(*seg_b, "≡");
        }
        SketchConstraint::Coincident { pt_a, .. } => {
            if let Some(p) = proj(points[*pt_a]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
            }
        }
        SketchConstraint::FixedLength { seg, value } => {
            draw(*seg, &format!("⟺{value:.2}"));
        }
        SketchConstraint::PointDistance { pt_a, pt_b, value } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(
                    mid + egui::vec2(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    &format!("⟺{value:.2}"),
                    font.clone(),
                    color,
                );
            }
        }
        SketchConstraint::PointFixed { .. } => {} // internal drag pin — no overlay marker
        SketchConstraint::PointOnOrigin { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "⊙", font.clone(), color);
            }
        }
        SketchConstraint::PointOnXAxis { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "∈X", font.clone(), color);
            }
        }
        SketchConstraint::PointOnYAxis { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "∈Y", font.clone(), color);
            }
        }
        SketchConstraint::HorizontalPair { pt_a, pt_b, .. } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(mid + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "↔", font.clone(), color);
            }
        }
        SketchConstraint::VerticalPair { pt_a, pt_b, .. } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(mid + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "↕", font.clone(), color);
            }
        }
        SketchConstraint::PointOnLine { pt, seg } => {
            // Draw a small circle at the point to indicate it's constrained to the line.
            if let Some(p) = proj(points[*pt]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
            }
            if let (Some(a), Some(b)) = (proj(points[*seg]), proj(points[(*seg + 1) % n])) {
                let m = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(m + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "◯", font.clone(), color);
            }
        }
        SketchConstraint::PointOnCircle { pt, .. } => {
            if let Some(p) = proj(points[*pt]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "◎", font.clone(), color);
            }
        }
    }
}

/// Snap a window rect's top-left position to the nearest viewport edge if within SNAP_DIST.
pub(crate) fn snap_to_viewport(pos: egui::Pos2, size: egui::Vec2, viewport: egui::Rect) -> egui::Pos2 {
    const GAP: f32 = 12.0;
    const SNAP: f32 = 40.0;
    let mut p = pos;
    // Clamp inside viewport first.
    p.x = p.x.clamp(viewport.left() + GAP, (viewport.right() - GAP - size.x).max(viewport.left() + GAP));
    p.y = p.y.clamp(viewport.top() + GAP, (viewport.bottom() - GAP - size.y).max(viewport.top() + GAP));
    // Then snap to edges.
    if (p.x - (viewport.left()  + GAP)).abs() < SNAP { p.x = viewport.left()  + GAP; }
    else if (p.x + size.x - (viewport.right()  - GAP)).abs() < SNAP { p.x = viewport.right()  - GAP - size.x; }
    if (p.y - (viewport.top()   + GAP)).abs() < SNAP { p.y = viewport.top()   + GAP; }
    else if (p.y + size.y - (viewport.bottom() - GAP)).abs() < SNAP { p.y = viewport.bottom() - GAP - size.y; }
    p
}

/// Floating sketch configuration panel — resizable, draggable, constrained to the viewport.
fn draw_sketch_info_panel(ctx: &egui::Context, editor: &EditorState) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();
    let Some(sk) = &editor.sketch else { return actions };

    let panel_id = egui::Id::new("sketch_info_panel");
    let viewport = ctx.available_rect();
    let default_pos = egui::pos2(viewport.left() + 12.0, viewport.top() + 12.0);

    // Snap to viewport edge once the pointer is released.
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
            if n == 0 {
                ui.label(egui::RichText::new("(none)").italics().weak());
            } else {
                // Detect selection changes so we can scroll the new item into view.
                let prev_sel_id = panel_id.with("prev_seg_sel");
                let prev_sel: Vec<usize> = ctx.data(|d| d.get_temp(prev_sel_id).unwrap_or_default());
                let sel_changed = prev_sel != sk.seg_selection;
                ctx.data_mut(|d| d.insert_temp(prev_sel_id, sk.seg_selection.clone()));

                // Elements list takes all remaining space above the constraints section.
                // Use a fixed-ratio split: ~40% of window height for elements.
                egui::ScrollArea::vertical()
                    .id_salt("sketch_elements_scroll")
                    .max_height(f32::INFINITY)
                    .min_scrolled_height(60.0)
                    .show(ui, |ui| {
                        ui.set_min_width(ui.available_width());
                        for i in 0..n {
                            let p = &sk.points[i];
                            let selected = sk.pt_selection.contains(&i);
                            let resp = ui.selectable_label(
                                selected,
                                format!("Point {}  ({:.2}, {:.2}, {:.2})", i, p.x, p.y, p.z),
                            );
                            if resp.clicked() {
                                actions.push(UiAction::SketchPanelSelectVertex(i));
                            }
                        }
                        let n_segs = if sk.closed { n } else { n.saturating_sub(1) };
                        for i in 0..n_segs {
                            let j = (i + 1) % n;
                            let selected = sk.seg_selection.contains(&i);
                            let resp = ui.selectable_label(
                                selected,
                                format!("Segment {}  ({} → {})", i, i, j),
                            );
                            if resp.clicked() {
                                actions.push(UiAction::SketchPanelSelectSegment(i));
                            }
                            // Scroll to the first newly-selected segment.
                            if selected && sel_changed && !prev_sel.contains(&i) {
                                resp.scroll_to_me(Some(egui::Align::Center));
                            }
                        }
                        // Committed profiles (circles, arcs, rectangles).
                        for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                            let label = cp.shape.label(pi);
                            let is_sel = sk.committed_selection == Some(pi);
                            let resp = ui.selectable_label(is_sel, label);
                            if resp.clicked() {
                                let new_sel = if is_sel { None } else { Some(pi) };
                                actions.push(UiAction::SketchSelectCommitted(new_sel));
                            }
                        }
                    });
            }

            ui.separator();

            // ── Constraints ───────────────────────────────────────────
            ui.strong("Constraints");
            if sk.constraints.is_empty() {
                ui.label(egui::RichText::new("(none)").italics().weak());
            } else {
                egui::ScrollArea::vertical()
                    .id_salt("sketch_constraints_scroll")
                    .max_height(f32::INFINITY)
                    .min_scrolled_height(60.0)
                    .show(ui, |ui| {
                        ui.set_min_width(ui.available_width());
                        let mut remove_idx: Option<usize> = None;
                        for (i, c) in sk.constraints.iter().enumerate() {
                            let is_violated =
                                sk.violated_constraints.get(i).copied().unwrap_or(false);
                            let is_selected = sk.constraint_selection.contains(&i);
                            ui.horizontal(|ui| {
                                ui.add(list_icon(constraint_icon(c)));
                                let label_text = if is_violated {
                                    egui::RichText::new(constraint_text(c))
                                        .color(egui::Color32::RED)
                                } else {
                                    egui::RichText::new(constraint_text(c))
                                };
                                let resp = ui.selectable_label(is_selected, label_text);
                                if resp.clicked() {
                                    actions.push(UiAction::SketchSelectConstraint(i));
                                }
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        ui.add_space(12.0); // keep clear of the scrollbar
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

    // Persist the window position after each frame so snap logic has fresh data.
    if let Some(state) = egui::AreaState::load(ctx, panel_id) {
        ctx.data_mut(|d| d.insert_temp(panel_id, state.left_top_pos()));
    }

    actions
}

/// Recursively render a history node as an operation row (used inside boolean subtrees).
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

// ── Committed profile rendering ───────────────────────────────────────────────

/// Render a committed profile (circle, arc, or polyline) using its stored shape.
fn draw_committed_profile(
    painter: &egui::Painter,
    pts: &[Point3],
    cp: &crate::editor::CommittedProfile,
    proj: &impl Fn(Point3) -> Option<egui::Pos2>,
    stroke: egui::Stroke,
) {
    cp.shape.draw(pts, cp.closed, cp.plane, painter, proj, stroke);
}

// ── Constraint conflict badge ─────────────────────────────────────────────────

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

// ── Sketch viewport toolbar ───────────────────────────────────────────────────

/// Floating toolbar at the top-centre of the sketch canvas.
/// Returns any actions the user triggered.
fn draw_sketch_viewport_toolbar(ctx: &egui::Context, editor: &EditorState) -> Vec<UiAction> {
    if editor.sketch.is_none() { return vec![]; }
    let mut actions: Vec<UiAction> = Vec::new();

    // Anchor to the top-centre of the viewport canvas (below the top panel).
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

#[cfg(test)]
mod tests {
    use super::*;

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
        let vp = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos = egui::pos2(350.0, 275.0); // well away from all edges
        let result = snap_to_viewport(pos, size, vp);
        assert!((result.x - pos.x).abs() < 0.1, "x should be unchanged");
        assert!((result.y - pos.y).abs() < 0.1, "y should be unchanged");
    }

    #[test]
    fn snap_near_left_edge_snaps_to_left() {
        let vp = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos = egui::pos2(30.0, 275.0); // within 40px snap zone of left+12
        let result = snap_to_viewport(pos, size, vp);
        assert_eq!(result.x, 12.0, "should snap to left GAP");
    }

    #[test]
    fn snap_near_top_edge_snaps_to_top() {
        let vp = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos = egui::pos2(350.0, 20.0); // within 40px snap zone of top+12
        let result = snap_to_viewport(pos, size, vp);
        assert_eq!(result.y, 12.0, "should snap to top GAP");
    }

    #[test]
    fn panel_outside_viewport_is_clamped_inside() {
        let vp = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
        let size = egui::vec2(100.0, 50.0);
        let pos = egui::pos2(-200.0, -200.0); // far outside
        let result = snap_to_viewport(pos, size, vp);
        assert!(result.x >= 12.0, "x must be inside viewport");
        assert!(result.y >= 12.0, "y must be inside viewport");
    }

    // ── egui headless smoke tests ─────────────────────────────────────────────

    #[test]
    fn build_ui_empty_state_produces_no_actions() {
        let ed = EditorState::new_empty();
        let ctx = egui::Context::default();
        let mut actions = Vec::new();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            actions = build_ui(
                ctx, &ed, (800, 600),
                None, None, None, None, None, None, None, None, None, None,
            );
        });
        assert!(actions.is_empty(), "no input should produce no actions");
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
                None, None, None, None, None, None, None, None, None, None,
            );
        });
        assert!(actions.is_empty(), "no input in sketch mode should produce no actions");
    }
}
