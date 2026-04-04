use brep_core::Point3;
use brep_render::{Camera, RenderOptions, render_to_png};
use brep_topo::{
    primitives::{make_box, make_cone, make_cylinder, make_sphere},
    store::ShapeStore,
};

fn main() {
    let opts = RenderOptions {
        width: 600,
        height: 600,
        chord_tolerance: 0.003,
        ..Default::default()
    };

    // ── Box ──────────────────────────────────────────────────────────────────
    {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(2.2, 1.8, 1.6), Point3::new(0.5, 0.5, 0.5)),
            &opts,
        )
        .unwrap();
        std::fs::write("renders/box.png", &png).unwrap();
        println!("Wrote renders/box.png  ({} bytes)", png.len());
    }

    // ── Cylinder ─────────────────────────────────────────────────────────────
    {
        let mut store = ShapeStore::new();
        make_cylinder(&mut store, 1.0, 2.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.5, 2.5, 2.2), Point3::new(0.0, 0.0, 1.0)),
            &opts,
        )
        .unwrap();
        std::fs::write("renders/cylinder.png", &png).unwrap();
        println!("Wrote renders/cylinder.png  ({} bytes)", png.len());
    }

    // ── Sphere ───────────────────────────────────────────────────────────────
    {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.2, 2.0, 1.5), Point3::origin()),
            &opts,
        )
        .unwrap();
        std::fs::write("renders/sphere.png", &png).unwrap();
        println!("Wrote renders/sphere.png  ({} bytes)", png.len());
    }

    // ── Cone ─────────────────────────────────────────────────────────────────
    {
        let mut store = ShapeStore::new();
        make_cone(&mut store, 1.0, 2.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.5, 2.5, 2.5), Point3::new(0.0, 0.0, 1.0)),
            &opts,
        )
        .unwrap();
        std::fs::write("renders/cone.png", &png).unwrap();
        println!("Wrote renders/cone.png  ({} bytes)", png.len());
    }
}
