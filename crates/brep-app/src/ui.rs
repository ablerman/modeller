//! egui panel layout: toolbar, object list, properties.

use brep_bool::BooleanKind;
use brep_core::Point3;
use brep_sketch::SketchConstraint;
use egui::Context;

use crate::editor::{EditorState, ObjectHistory, PrimitiveKind, SketchPlane, UiAction};

/// Render all egui panels.  Returns any actions the user triggered.
pub fn build_ui(
    ctx: &Context,
    editor: &EditorState,
    viewport: (u32, u32),
    sketch_cursor: Option<Point3>,
    snap_vertex: Option<usize>,
    snap_segment: Option<usize>,
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

                if ui.button("Undo pt").clicked() {
                    actions.push(UiAction::SketchUndoPoint);
                }
                ui.add_enabled_ui(sk.points.len() >= 3 && !sk.closed, |ui| {
                    if ui.button("Close Loop").clicked() {
                        actions.push(UiAction::SketchClose);
                    }
                });

                if sk.closed {
                    ui.separator();
                    let mut d = sk.extrude_dist;
                    if ui.add(
                        egui::DragValue::new(&mut d)
                            .speed(0.05)
                            .range(0.001..=1000.0)
                            .suffix(" u"),
                    ).changed() {
                        actions.push(UiAction::SketchSetDistance(d));
                    }
                    if ui.button("Extrude").clicked() {
                        actions.push(UiAction::SketchExtrude(sk.extrude_dist));
                    }
                }

                // ── Constraint buttons ────────────────────────────────────
                if !sk.points.is_empty() {
                    ui.separator();
                    ui.strong("Constrain:");

                    let n_sel = sk.seg_selection.len();

                    // 1-segment constraints.
                    ui.add_enabled_ui(n_sel == 1, |ui| {
                        if ui.button("Horizontal").clicked() {
                            let seg = sk.seg_selection[0];
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::Horizontal { seg }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                        if ui.button("Vertical").clicked() {
                            let seg = sk.seg_selection[0];
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::Vertical { seg }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                    });

                    // 2-segment constraints.
                    ui.add_enabled_ui(n_sel == 2, |ui| {
                        if ui.button("Parallel").clicked() {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::Parallel {
                                    seg_a: sk.seg_selection[0],
                                    seg_b: sk.seg_selection[1],
                                }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                        if ui.button("Perp.").clicked() {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::Perpendicular {
                                    seg_a: sk.seg_selection[0],
                                    seg_b: sk.seg_selection[1],
                                }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                        if ui.button("= Len").clicked() {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::EqualLength {
                                    seg_a: sk.seg_selection[0],
                                    seg_b: sk.seg_selection[1],
                                }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                    });

                    // Angle: 2 segments + degree input.
                    if n_sel == 2 {
                        let angle_id = egui::Id::new("sketch_angle_deg");
                        let mut deg: f64 = ctx.data_mut(|d| {
                            *d.get_temp_mut_or(angle_id, 90.0_f64)
                        });
                        ui.add(
                            egui::DragValue::new(&mut deg)
                                .speed(1.0)
                                .range(1.0..=179.0)
                                .suffix("°"),
                        );
                        ctx.data_mut(|d| {
                            *d.get_temp_mut_or::<f64>(angle_id, 90.0) = deg;
                        });
                        if ui.button("Angle").clicked() {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::Angle {
                                    seg_a: sk.seg_selection[0],
                                    seg_b: sk.seg_selection[1],
                                    degrees: deg,
                                }));
                            actions.push(UiAction::SketchClearSegSelection);
                        }
                    }

                    if sk.constraints_conflict {
                        ui.colored_label(egui::Color32::RED, "⚠ conflict");
                    }

                    // Active constraints list.
                    if !sk.constraints.is_empty() {
                        ui.separator();
                        ui.menu_button(
                            format!("Constraints ({})", sk.constraints.len()),
                            |ui| {
                                let mut to_remove = None;
                                for (i, c) in sk.constraints.iter().enumerate() {
                                    ui.horizontal(|ui| {
                                        ui.label(constraint_label(c));
                                        if ui.small_button("✕").clicked() {
                                            to_remove = Some(i);
                                            ui.close_menu();
                                        }
                                    });
                                }
                                if let Some(i) = to_remove {
                                    actions.push(UiAction::SketchRemoveConstraint(i));
                                }
                            },
                        );
                    }
                }

                ui.separator();
                if ui.button("✕ Cancel").clicked() {
                    actions.push(UiAction::ExitSketch);
                }
            } else {
                // ── Normal toolbar ────────────────────────────────────────────
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
                ui.menu_button("Sketch ▾", |ui| {
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
            }
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

    // ── Viewport axes (drawn into the 3-D area via egui painter) ─────────────
    draw_viewport_axes(ctx, editor);

    // ── Orientation cube gizmo (top-right, draggable) ─────────────────────────
    if let Some(snap) = draw_orientation_cube(ctx, editor) {
        actions.push(snap);
    }

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
        for c in &sk.constraints {
            draw_constraint_marker(&painter, c, &sk.points, n, &proj);
        }
        // Preview line from last placed point to cursor (snaps to vertex if applicable).
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
        // Vertex dots — highlight snapped vertex.
        for (i, &pt) in sk.points.iter().enumerate() {
            if let Some(p) = proj(pt) {
                let is_snap = snap_vertex == Some(i);
                let is_close = i == 0 && is_snap && sk.points.len() >= 3;
                if is_close {
                    // Green ring = "click here to close the loop"
                    painter.circle_stroke(p, 8.0, egui::Stroke::new(2.5, egui::Color32::from_rgb(80, 230, 80)));
                    painter.circle_filled(p, 5.0, egui::Color32::YELLOW);
                } else if is_snap {
                    // White ring = hovering another vertex
                    painter.circle_stroke(p, 7.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
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

/// Short display label for a constraint.
fn constraint_label(c: &SketchConstraint) -> String {
    match c {
        SketchConstraint::Parallel { seg_a, seg_b } =>
            format!("Parallel  seg {seg_a} ∥ {seg_b}"),
        SketchConstraint::Perpendicular { seg_a, seg_b } =>
            format!("Perp.  seg {seg_a} ⊥ {seg_b}"),
        SketchConstraint::Angle { seg_a, seg_b, degrees } =>
            format!("Angle  {degrees:.0}°  seg {seg_a}/{seg_b}"),
        SketchConstraint::Horizontal { seg } =>
            format!("Horizontal  seg {seg}"),
        SketchConstraint::Vertical { seg } =>
            format!("Vertical  seg {seg}"),
        SketchConstraint::EqualLength { seg_a, seg_b } =>
            format!("Equal len  seg {seg_a} = {seg_b}"),
        SketchConstraint::Coincident { pt_a, pt_b } =>
            format!("Coincident  pt {pt_a} = {pt_b}"),
    }
}

/// Draw a small symbol near the segment midpoint to indicate a constraint.
fn draw_constraint_marker(
    painter: &egui::Painter,
    c: &SketchConstraint,
    points: &[brep_core::Point3],
    n: usize,
    proj: &impl Fn(brep_core::Point3) -> Option<egui::Pos2>,
) {
    let midpoint = |seg: usize| -> Option<egui::Pos2> {
        let a = proj(points[seg])?;
        let b = proj(points[(seg + 1) % n])?;
        Some(egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5))
    };
    let color = egui::Color32::from_rgb(230, 220, 70);
    let font = egui::FontId::proportional(11.0);
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
            draw(*seg_a, "⊾"); draw(*seg_b, "⊾");
        }
        SketchConstraint::Angle { seg_a, degrees, .. } => {
            draw(*seg_a, &format!("{:.0}°", degrees));
        }
        SketchConstraint::Horizontal { seg } => { draw(*seg, "—"); }
        SketchConstraint::Vertical { seg } => { draw(*seg, "|"); }
        SketchConstraint::EqualLength { seg_a, seg_b } => {
            draw(*seg_a, "="); draw(*seg_b, "=");
        }
        SketchConstraint::Coincident { pt_a, .. } => {
            if let Some(p) = proj(points[*pt_a]) {
                painter.circle_stroke(p, 6.0, egui::Stroke::new(1.5, color));
            }
        }
    }
}

/// Recursively render an operation history node as a collapsible tree.
fn show_history(ui: &mut egui::Ui, node: &ObjectHistory) {
    match node {
        ObjectHistory::Primitive(_) => {
            ui.label(egui::RichText::new(node.label()).weak().italics());
        }
        ObjectHistory::Sketch { plane } => {
            ui.label(
                egui::RichText::new(format!("Sketch ({plane:?})"))
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
