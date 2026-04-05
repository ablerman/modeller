//! egui panel layout: toolbar, object list, properties.

use brep_bool::BooleanKind;
use brep_core::Point3;
use brep_sketch::SketchConstraint;
use egui::Context;

use crate::editor::{EditorState, LengthTarget, ObjectHistory, PrimitiveKind, RefEntity, SceneEntry, SketchPlane, UiAction};

// ── Toolbar menu icons (image files) ─────────────────────────────────────────
fn icon_file()       -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/file.svg") }
fn icon_primitives() -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/primitives.svg") }
fn icon_sketch()     -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/sketch.svg") }
fn icon_boolean()    -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/boolean.svg") }

// ── Constraint icons (image files) ───────────────────────────────────────────
fn icon_parallel()   -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/parallel.svg") }
fn icon_perp()       -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/perpendicular.svg") }
fn icon_angle()      -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/angle.svg") }
fn icon_horizontal() -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/horizontal.svg") }
fn icon_vertical()   -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/vertical.svg") }
fn icon_equal_len()  -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/equal_length.svg") }
fn icon_length()     -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/length.svg") }
fn icon_coincident() -> egui::ImageSource<'static> { egui::include_image!("../assets/icons/coincident.svg") }

/// Icon image sized for use in toolbar buttons.
fn toolbar_icon(src: egui::ImageSource<'static>) -> egui::Image<'static> {
    egui::Image::new(src).fit_to_exact_size(egui::vec2(16.0, 16.0))
}

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

                if ui.button("Undo").clicked() {
                    actions.push(UiAction::SketchUndoPoint);
                }
                ui.add_enabled_ui(sk.points.len() >= 3, |ui| {
                    if ui.button("Finish").clicked() {
                        actions.push(UiAction::SketchFinish);
                    }
                });

                // ── Constraint dropdown ───────────────────────────────────
                if !sk.points.is_empty() {
                    ui.separator();

                    let n_sel = sk.seg_selection.len();
                    let n_pts = sk.pt_selection.len();
                    let length_target: Option<LengthTarget> = if n_sel == 1 {
                        Some(LengthTarget::Segment(sk.seg_selection[0]))
                    } else if n_pts == 2 {
                        Some(LengthTarget::Points(sk.pt_selection[0], sk.pt_selection[1]))
                    } else {
                        None
                    };

                    // Precompute viewport-aligned perp directions and point pair for H/V constraints.
                    let ((h_pu, h_pv), (v_pu, v_pv)) = editor.camera.sketch_align_dirs(sk.plane);
                    let ref_sel = sk.ref_selection;
                    let pt1 = sk.pt_selection.first().copied();

                    // Build the constraint each button would apply, or None if not applicable.
                    // H/V: pair of points (via segment or explicit), or 1 pt + origin/axis ref.
                    let h_constraint: Option<SketchConstraint> = if n_sel == 1 {
                        let s = sk.seg_selection[0];
                        Some(SketchConstraint::HorizontalPair { pt_a: s, pt_b: (s + 1) % sk.points.len(), perp_u: h_pu, perp_v: h_pv })
                    } else if n_pts == 2 {
                        Some(SketchConstraint::HorizontalPair { pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1], perp_u: h_pu, perp_v: h_pv })
                    } else if n_pts == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::XAxis)) {
                        Some(SketchConstraint::PointOnXAxis { pt: pt1.unwrap() })
                    } else {
                        None
                    };
                    let v_constraint: Option<SketchConstraint> = if n_sel == 1 {
                        let s = sk.seg_selection[0];
                        Some(SketchConstraint::VerticalPair { pt_a: s, pt_b: (s + 1) % sk.points.len(), perp_u: v_pu, perp_v: v_pv })
                    } else if n_pts == 2 {
                        Some(SketchConstraint::VerticalPair { pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1], perp_u: v_pu, perp_v: v_pv })
                    } else if n_pts == 1 && matches!(ref_sel, Some(RefEntity::Origin) | Some(RefEntity::YAxis)) {
                        Some(SketchConstraint::PointOnYAxis { pt: pt1.unwrap() })
                    } else {
                        None
                    };
                    let coincident_constraint: Option<SketchConstraint> = if n_pts == 2 && n_sel == 0 {
                        Some(SketchConstraint::Coincident { pt_a: sk.pt_selection[0], pt_b: sk.pt_selection[1] })
                    } else if n_sel == 1 && n_pts == 1 {
                        Some(SketchConstraint::PointOnLine { pt: sk.pt_selection[0], seg: sk.seg_selection[0] })
                    } else if n_pts == 1 && ref_sel == Some(RefEntity::Origin) {
                        Some(SketchConstraint::PointOnOrigin { pt: pt1.unwrap() })
                    } else {
                        None
                    };

                    ui.menu_button("Constrain ▾", |ui| {
                        ui.add_enabled_ui(h_constraint.is_some(), |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_horizontal()), "Horizontal")).clicked() {
                                if let Some(c) = h_constraint.clone() {
                                    actions.push(UiAction::SketchAddConstraint(c));
                                }
                                ui.close_menu();
                            }
                        });
                        ui.add_enabled_ui(v_constraint.is_some(), |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_vertical()), "Vertical")).clicked() {
                                if let Some(c) = v_constraint.clone() {
                                    actions.push(UiAction::SketchAddConstraint(c));
                                }
                                ui.close_menu();
                            }
                        });
                        ui.separator();
                        ui.add_enabled_ui(n_sel == 2, |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_parallel()), "Parallel")).clicked() {
                                actions.push(UiAction::SketchAddConstraint(
                                    SketchConstraint::Parallel {
                                        seg_a: sk.seg_selection[0],
                                        seg_b: sk.seg_selection[1],
                                    }));
                                ui.close_menu();
                            }
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_perp()), "Perpendicular")).clicked() {
                                actions.push(UiAction::SketchAddConstraint(
                                    SketchConstraint::Perpendicular {
                                        seg_a: sk.seg_selection[0],
                                        seg_b: sk.seg_selection[1],
                                    }));
                                ui.close_menu();
                            }
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_equal_len()), "Equal Length")).clicked() {
                                actions.push(UiAction::SketchAddConstraint(
                                    SketchConstraint::EqualLength {
                                        seg_a: sk.seg_selection[0],
                                        seg_b: sk.seg_selection[1],
                                    }));
                                ui.close_menu();
                            }
                        });
                        ui.separator();
                        ui.add_enabled_ui(n_sel == 2, |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_angle()), "Angle…")).clicked() {
                                actions.push(UiAction::SketchBeginAngleInput {
                                    seg_a: sk.seg_selection[0],
                                    seg_b: sk.seg_selection[1],
                                });
                                ui.close_menu();
                            }
                        });
                        ui.add_enabled_ui(length_target.is_some(), |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_length()), "Length…")).clicked() {
                                if let Some(t) = length_target {
                                    actions.push(UiAction::SketchBeginLengthInput(t));
                                }
                                ui.close_menu();
                            }
                        });
                        ui.add_enabled_ui(coincident_constraint.is_some(), |ui| {
                            if ui.add(egui::Button::image_and_text(toolbar_icon(icon_coincident()), "Coincident")).clicked() {
                                if let Some(c) = coincident_constraint.clone() {
                                    actions.push(UiAction::SketchAddConstraint(c));
                                }
                                ui.close_menu();
                            }
                        });
                    });

                    if sk.constraints_conflict {
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
                if ui.button("✕ Cancel").clicked() {
                    actions.push(UiAction::ExitSketch);
                }
            } else {
                // ── Normal toolbar ────────────────────────────────────────────
                egui::menu::menu_custom_button(ui,
                    egui::Button::image_and_text(toolbar_icon(icon_file()), "File")
                        .image_tint_follows_text_color(true),
                    |ui| {
                    if ui.button("New").clicked() {
                        actions.push(UiAction::New);
                        ui.close_menu();
                    }
                    if ui.button("Open…").clicked() {
                        actions.push(UiAction::Open);
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Save").clicked() {
                        actions.push(UiAction::Save);
                        ui.close_menu();
                    }
                    if ui.button("Save As…").clicked() {
                        actions.push(UiAction::SaveAs);
                        ui.close_menu();
                    }
                });
                ui.separator();
                egui::menu::menu_custom_button(ui,
                    egui::Button::image_and_text(toolbar_icon(icon_primitives()), "Primitives")
                        .image_tint_follows_text_color(true),
                    |ui| {
                    if ui.button("☐  Box").clicked() {
                        actions.push(UiAction::AddPrimitive(PrimitiveKind::Box));
                        ui.close_menu();
                    }
                    if ui.button("◎  Cylinder").clicked() {
                        actions.push(UiAction::AddPrimitive(PrimitiveKind::Cylinder));
                        ui.close_menu();
                    }
                    if ui.button("◉  Sphere").clicked() {
                        actions.push(UiAction::AddPrimitive(PrimitiveKind::Sphere));
                        ui.close_menu();
                    }
                    if ui.button("△  Cone").clicked() {
                        actions.push(UiAction::AddPrimitive(PrimitiveKind::Cone));
                        ui.close_menu();
                    }
                });

                ui.separator();
                egui::menu::menu_custom_button(ui,
                    egui::Button::image_and_text(toolbar_icon(icon_sketch()), "Sketch")
                        .image_tint_follows_text_color(true),
                    |ui| {
                    for (label, plane) in [
                        ("XY Plane", SketchPlane::XY),
                        ("XZ Plane", SketchPlane::XZ),
                        ("YZ Plane", SketchPlane::YZ),
                    ] {
                        if ui.button(label).clicked() {
                            actions.push(UiAction::EnterSketch(plane));
                            ui.close_menu();
                        }
                    }
                });

                ui.separator();

                let two_sel = editor.selection.iter()
                    .filter(|&&i| matches!(editor.entries.get(i), Some(SceneEntry::Solid(_))))
                    .count() == 2;
                ui.add_enabled_ui(two_sel, |ui| {
                    egui::menu::menu_custom_button(ui,
                        egui::Button::image_and_text(toolbar_icon(icon_boolean()), "Boolean")
                            .image_tint_follows_text_color(true),
                        |ui| {
                        if ui.button("⊕  Union").clicked() {
                            actions.push(UiAction::BooleanOp(BooleanKind::Union));
                            ui.close_menu();
                        }
                        if ui.button("⊖  Difference").clicked() {
                            actions.push(UiAction::BooleanOp(BooleanKind::Difference));
                            ui.close_menu();
                        }
                        if ui.button("⊗  Intersection").clicked() {
                            actions.push(UiAction::BooleanOp(BooleanKind::Intersection));
                            ui.close_menu();
                        }
                    });
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
    if let Some(snap) = draw_orientation_cube(ctx, editor) {
        actions.push(snap);
    }

    // ── Sketch info panel (top-left of viewport, sketch mode only) ────────────
    actions.extend(draw_sketch_info_panel(ctx, editor));

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

        // Draw placed edges.
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
        // Preview line from last placed point to cursor (only when loop is open).
        if !sk.closed {
            if let Some(last) = sk.points.last() {
                let cursor_target = snap_vertex
                    .and_then(|i| sk.points.get(i).copied())
                    .or(sketch_cursor);
                if let Some(cursor_world) = cursor_target {
                    if let (Some(a), Some(b)) = (proj(*last), proj(cursor_world)) {
                        painter.line_segment([a, b], preview_stroke);
                    }
                }
            }
        }
        // Vertex dots — highlight snapped/selected vertices.
        let can_close = !sk.closed && n >= 3;
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
                let n = sk.points.len();
                for j in 0..n {
                    if let (Some(a), Some(b)) = (proj(sk.points[j]), proj(sk.points[(j + 1) % n])) {
                        painter.line_segment([a, b], stroke);
                    }
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
                    let c = match target {
                        LengthTarget::Segment(seg) =>
                            SketchConstraint::FixedLength { seg, value },
                        LengthTarget::Points(pt_a, pt_b) =>
                            SketchConstraint::PointDistance { pt_a, pt_b, value },
                    };
                    actions.push(UiAction::SketchAddConstraint(c));
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

// ── Orientation cube gizmo ────────────────────────────────────────────────────

/// Draw the orientation cube in the bottom-right corner.
/// Returns a `SnapCamera` or `OrbitCamera` action if the user interacted.
fn draw_orientation_cube(ctx: &egui::Context, editor: &EditorState) -> Option<UiAction> {
    const SIZE: f32 = 90.0;
    const HALF: f32 = 28.0;
    /// Fraction of each edge to chamfer (0.3 = 30% from each end).
    const T: f32 = 0.3;

    use std::f64::consts::{PI, FRAC_PI_2};
    // Elevation for isometric corner views: arcsin(1/√3).
    const ISO_EL: f64 = 0.6154797086703873;

    let screen = ctx.input(|i| i.screen_rect());
    let fixed_pos = egui::pos2(screen.right() - SIZE - 12.0, screen.bottom() - SIZE - 12.0);
    let mut snap = None;

    egui::Area::new(egui::Id::new("orient_cube"))
        .fixed_pos(fixed_pos)
        .order(egui::Order::Foreground)
        .movable(false)
        .show(ctx, |ui| {
            let (rect, resp) = ui.allocate_exact_size(
                egui::vec2(SIZE, SIZE),
                egui::Sense::click_and_drag(),
            );

            if resp.dragged() {
                let d = resp.drag_delta();
                snap = Some(UiAction::OrbitCamera { dx: d.x, dy: d.y });
            }

            let centre = rect.center();
            let painter = ui.painter_at(rect.expand(4.0));

            painter.circle_filled(
                centre, SIZE * 0.5,
                egui::Color32::from_rgba_unmultiplied(30, 30, 40, 180),
            );

            // Camera rotation vectors (no translation).
            let az = editor.camera.azimuth as f32;
            let el = editor.camera.elevation as f32;
            let (saz, caz) = (az.sin(), az.cos());
            let (sel, cel) = (el.sin(), el.cos());
            let r   = [-saz,       caz,       0.0_f32]; // screen right in world
            let up  = [-caz * sel, -saz * sel, cel];    // screen up in world
            let fwd = [-cel * caz, -cel * saz, -sel];   // into screen

            let dot3 = |a: [f32;3], b: [f32;3]| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
            let lerp3 = |a: [f32;3], b: [f32;3], t: f32| -> [f32;3] {
                [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t]
            };
            let proj = |v: [f32;3]| -> egui::Pos2 {
                centre + egui::vec2(dot3(r, v), -dot3(up, v)) * HALF
            };

            // 8 cube corners at ±1 on each axis.
            let c: [[f32;3];8] = [
                [-1.,-1.,-1.], [ 1.,-1.,-1.],
                [ 1., 1.,-1.], [-1., 1.,-1.],
                [-1.,-1., 1.], [ 1.,-1., 1.],
                [ 1., 1., 1.], [-1., 1., 1.],
            ];

            // Each face: [corner indices CCW], face normal, label, snap (az,el), fill rgb.
            let faces: [([usize;4], [f32;3], &str, (f64,f64), [u8;3]); 6] = [
                ([1,2,6,5], [ 1., 0., 0.], "Right",  (0.,         0.),    [200,60,60]),
                ([0,4,7,3], [-1., 0., 0.], "Left",   (PI,         0.),    [160,40,40]),
                ([2,3,7,6], [ 0., 1., 0.], "Back",   (FRAC_PI_2,  0.),    [60,180,60]),
                ([0,1,5,4], [ 0.,-1., 0.], "Front",  (-FRAC_PI_2, 0.),    [40,130,40]),
                ([4,5,6,7], [ 0., 0., 1.], "Top",    (0., FRAC_PI_2-0.05),[60,100,220]),
                ([0,3,2,1], [ 0., 0.,-1.], "Bottom", (0.,-(FRAC_PI_2-0.05)),[40,70,160]),
            ];

            // Each corner: [neighbor indices (flip one coord each)], snap (az,el).
            let corner_neighbors: [[usize;3];8] = [
                [1,3,4], [0,2,5], [3,1,6], [2,0,7],
                [5,7,0], [4,6,1], [7,5,2], [6,4,3],
            ];
            let corner_snaps: [(f64,f64);8] = [
                (-3.*PI/4., -ISO_EL), (-PI/4., -ISO_EL),
                ( PI/4.,    -ISO_EL), ( 3.*PI/4., -ISO_EL),
                (-3.*PI/4.,  ISO_EL), (-PI/4.,  ISO_EL),
                ( PI/4.,     ISO_EL), ( 3.*PI/4., ISO_EL),
            ];

            // Chamfered face polygon: for face [c0,c1,c2,c3] the 8-gon is built
            // by replacing each corner with 2 points at fraction T along each edge.
            let chamfer_poly = |fi: usize| -> Vec<egui::Pos2> {
                let ci = faces[fi].0;
                let mut pts = Vec::with_capacity(8);
                for i in 0..4 {
                    let a = c[ci[i]];
                    let b = c[ci[(i+1)%4]];
                    pts.push(proj(lerp3(a, b, T)));
                    pts.push(proj(lerp3(a, b, 1.-T)));
                }
                pts
            };

            // Corner triangle: vertices are lerp(corner, each_neighbor, T).
            let corner_tri = |ci: usize| -> Vec<egui::Pos2> {
                let ns = corner_neighbors[ci];
                vec![
                    proj(lerp3(c[ci], c[ns[0]], T)),
                    proj(lerp3(c[ci], c[ns[1]], T)),
                    proj(lerp3(c[ci], c[ns[2]], T)),
                ]
            };

            // Build draw list sorted back-to-front.
            // Tag: false = face(fi), true = corner(ci).
            let mut items: Vec<(f32, bool, usize)> = Vec::with_capacity(14);
            for fi in 0..6 {
                items.push((dot3(faces[fi].1, fwd), false, fi));
            }
            for ci in 0..8 {
                // Normalize corner position to unit sphere for comparable depth.
                let d = dot3(c[ci], fwd) / 1.7321;
                items.push((d, true, ci));
            }
            items.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

            let click_pos = if resp.clicked() { resp.interact_pointer_pos() } else { None };

            for (_, is_corner, idx) in &items {
                if !is_corner {
                    let fi = *idx;
                    let (_, normal, label, (snap_az, snap_el), rgb) = faces[fi];
                    let facing = dot3(normal, fwd) < 0.0;
                    let poly = chamfer_poly(fi);
                    let alpha = if facing { 235u8 } else { 80 };
                    let fill = egui::Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], alpha);
                    let stroke = egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(200,200,200,150));
                    painter.add(egui::Shape::convex_polygon(poly.clone(), fill, stroke));

                    if facing {
                        let fc = poly.iter().fold(egui::Vec2::ZERO, |a,p| a+p.to_vec2()) / poly.len() as f32;
                        painter.text(fc.to_pos2(), egui::Align2::CENTER_CENTER, label,
                            egui::FontId::proportional(9.0), egui::Color32::WHITE);
                        if let Some(click) = click_pos {
                            if point_in_polygon2d(click, &poly) {
                                snap = Some(UiAction::SnapCamera { azimuth: snap_az, elevation: snap_el });
                            }
                        }
                    }
                } else {
                    let ci = *idx;
                    let facing = dot3(c[ci], fwd) < 0.0;
                    let tri = corner_tri(ci);
                    let alpha = if facing { 210u8 } else { 55 };
                    let fill = egui::Color32::from_rgba_unmultiplied(200, 200, 215, alpha);
                    let stroke = egui::Stroke::new(0.8, egui::Color32::from_rgba_unmultiplied(200,200,200,120));
                    painter.add(egui::Shape::convex_polygon(tri.clone(), fill, stroke));

                    if facing {
                        if let Some(click) = click_pos {
                            if point_in_polygon2d(click, &tri) {
                                let (az, el) = corner_snaps[ci];
                                snap = Some(UiAction::SnapCamera { azimuth: az, elevation: el });
                            }
                        }
                    }
                }
            }
        });

    snap
}

/// Test whether `p` is inside a convex polygon (screen space).
fn point_in_polygon2d(p: egui::Pos2, poly: &[egui::Pos2]) -> bool {
    let n = poly.len();
    let mut sign = 0i32;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        let s = if cross > 0.0 { 1 } else if cross < 0.0 { -1 } else { 0 };
        if s != 0 {
            if sign == 0 { sign = s; } else if sign != s { return false; }
        }
    }
    true
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
    }
}

/// Short text description of a constraint (used alongside its icon in the constraint list).
fn constraint_text(c: &SketchConstraint) -> String {
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
        SketchConstraint::PointOnLine { pt, seg }           => format!("On Line  pt {pt} on seg {seg}"),
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
    }
}

/// Floating sketch configuration panel — top-left of the viewport.
fn draw_sketch_info_panel(ctx: &egui::Context, editor: &EditorState) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();
    let Some(sk) = &editor.sketch else { return actions };

    // Position just inside the viewport area (below toolbar, right of left panel).
    let avail = ctx.available_rect();
    let pos = egui::pos2(avail.left() + 12.0, avail.top() + 12.0);

    egui::Area::new(egui::Id::new("sketch_info_panel"))
        .fixed_pos(pos)
        .order(egui::Order::Foreground)
        .movable(false)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_min_width(220.0);

                // ── Finish Sketch button ──────────────────────────────────
                ui.add_enabled_ui(sk.points.len() >= 3, |ui| {
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
                        egui::TextEdit::singleline(&mut name_buf).desired_width(140.0),
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
                    egui::ScrollArea::vertical()
                        .id_salt("sketch_elements_scroll")
                        .max_height(120.0)
                        .show(ui, |ui| {
                            for i in 0..n {
                                let p = &sk.points[i];
                                let selected = sk.pt_selection.contains(&i);
                                let resp = ui.selectable_label(
                                    selected,
                                    format!("Point {}  ({:.2}, {:.2}, {:.2})", i, p.x, p.y, p.z),
                                );
                                if resp.clicked() {
                                    actions.push(UiAction::SketchSelectVertex(i));
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
                                    actions.push(UiAction::SketchSelectSegment(i));
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
                        .max_height(120.0)
                        .show(ui, |ui| {
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
                                            if ui.small_button("✕").clicked() {
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
            });
        });

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
