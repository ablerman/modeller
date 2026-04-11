//! Orientation cube gizmo — draws and handles interaction with the view cube.

use crate::editor::{EditorState, UiAction};

/// Draw the orientation cube in the bottom-right corner.
/// Returns a `SnapCamera` or `OrbitCamera` action if the user interacted.
pub fn draw_orientation_cube(ctx: &egui::Context, editor: &EditorState) -> Option<UiAction> {
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

            // Each face: [corner indices CCW], face normal, label, snap (az,el), fill rgb, face right (world).
            let faces: [([usize;4], [f32;3], &str, (f64,f64), [u8;3], [f32;3]); 6] = [
                ([1,2,6,5], [ 1., 0., 0.], "Right",  (0.,         0.),    [200,60,60],  [ 0., 1., 0.]),
                ([0,4,7,3], [-1., 0., 0.], "Left",   (PI,         0.),    [160,40,40],  [ 0.,-1., 0.]),
                ([2,3,7,6], [ 0., 1., 0.], "Back",   (FRAC_PI_2,  0.),    [60,180,60],  [-1., 0., 0.]),
                ([0,1,5,4], [ 0.,-1., 0.], "Front",  (-FRAC_PI_2, 0.),    [40,130,40],  [ 1., 0., 0.]),
                ([4,5,6,7], [ 0., 0., 1.], "Top",    (0., FRAC_PI_2-0.05),[60,100,220], [ 0., 1., 0.]),
                ([0,3,2,1], [ 0., 0.,-1.], "Bottom", (0.,-(FRAC_PI_2-0.05)),[40,70,160],[ 0., 1., 0.]),
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
                    let (_, normal, label, (snap_az, snap_el), rgb, face_right) = faces[fi];
                    let facing = dot3(normal, fwd) < 0.0;
                    let poly = chamfer_poly(fi);
                    let alpha = if facing { 235u8 } else { 80 };
                    let fill = egui::Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], alpha);
                    let stroke = egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(200,200,200,150));
                    painter.add(egui::Shape::convex_polygon(poly.clone(), fill, stroke));

                    if facing {
                        let fc = poly.iter().fold(egui::Vec2::ZERO, |a,p| a+p.to_vec2()) / poly.len() as f32;

                        let fr_screen = egui::vec2(dot3(r, face_right), -dot3(up, face_right));
                        let mut angle = if fr_screen.length() > 0.001 {
                            fr_screen.y.atan2(fr_screen.x)
                        } else {
                            0.0f32
                        };
                        // Keep text upright — angles outside [-π/2, π/2] appear upside-down.
                        if angle > std::f32::consts::FRAC_PI_2 {
                            angle -= std::f32::consts::PI;
                        } else if angle < -std::f32::consts::FRAC_PI_2 {
                            angle += std::f32::consts::PI;
                        }
                        let galley = ctx.fonts(|f| f.layout_no_wrap(
                            label.to_string(),
                            egui::FontId::proportional(9.0),
                            egui::Color32::WHITE,
                        ));
                        let hw = galley.size().x * 0.5;
                        let hh = galley.size().y * 0.5;
                        let (sa, ca) = (angle.sin(), angle.cos());
                        let text_pos = fc.to_pos2() - egui::vec2(hw * ca - hh * sa, hw * sa + hh * ca);
                        let mut ts = egui::epaint::TextShape::new(text_pos, galley, egui::Color32::WHITE);
                        ts.angle = angle;
                        painter.add(egui::Shape::Text(ts));

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
