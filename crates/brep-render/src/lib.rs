//! Software rasterizer for BRep models.
//!
//! Converts a [`ShapeStore`] to a PNG image entirely on the CPU — no GPU
//! or window required.  The pipeline is:
//!
//! 1. Tessellate with `brep-mesh` (adaptive chord tolerance).
//! 2. Transform each triangle through a view + perspective matrix.
//! 3. Rasterise with a Z-buffer using barycentric coverage tests.
//! 4. Shade each fragment with a flat-normal two-light Phong model.
//! 5. Encode as PNG via the `png` crate.
//!
//! # Example
//! ```no_run
//! use brep_render::{Camera, Light, RenderOptions, render_to_png};
//! use brep_topo::{primitives::make_box, store::ShapeStore};
//! use brep_core::Point3;
//!
//! let mut store = ShapeStore::new();
//! make_box(&mut store, 2.0, 1.0, 1.0).unwrap();
//!
//! let png = render_to_png(
//!     &store,
//!     &Camera::look_at(Point3::new(3.0, 3.0, 2.5), Point3::new(1.0, 0.5, 0.5)),
//!     &RenderOptions::default(),
//! ).unwrap();
//! std::fs::write("/tmp/box.png", png).unwrap();
//! ```

#[cfg(feature = "window")]
pub mod gpu;

use std::io::Cursor;

use brep_core::{Point3, Vec3};
use brep_mesh::{tessellate, TessellationOptions};
use brep_topo::store::ShapeStore;
use nalgebra::{Matrix4, Vector4};

// ── Public API types ──────────────────────────────────────────────────────────

/// Pinhole camera: position, target, and field-of-view.
#[derive(Clone, Debug)]
pub struct Camera {
    /// World-space eye position.
    pub eye: Point3,
    /// Point the camera looks toward.
    pub target: Point3,
    /// World-space up vector (default: +Z).
    pub up: Vec3,
    /// Vertical field of view in degrees.
    pub fov_y_deg: f64,
}

impl Camera {
    /// Construct a camera at `eye` looking at `target`, +Z up, 45° FOV.
    pub fn look_at(eye: Point3, target: Point3) -> Self {
        Self {
            eye,
            target,
            up: Vec3::z(),
            fov_y_deg: 45.0,
        }
    }
}

/// A single directional light.
#[derive(Clone, Debug)]
pub struct Light {
    /// Unit direction *toward* the light source.
    pub direction: Vec3,
    /// Linear RGB intensity.
    pub color: [f32; 3],
}

/// Rendering settings.
#[derive(Clone, Debug)]
pub struct RenderOptions {
    /// Output image width in pixels.
    pub width: u32,
    /// Output image height in pixels.
    pub height: u32,
    /// Background colour (sRGB u8).
    pub background: [u8; 3],
    /// Base diffuse colour of the model (linear).
    pub face_color: [f32; 3],
    /// Ambient contribution (0–1).
    pub ambient: f32,
    /// Specular intensity (0–1).
    pub specular: f32,
    /// Specular shininess exponent.
    pub shininess: f32,
    /// Chord tolerance passed to the tessellator.
    pub chord_tolerance: f64,
}

impl Default for RenderOptions {
    fn default() -> Self {
        Self {
            width: 600,
            height: 600,
            background: [30, 30, 38],
            face_color: [0.38, 0.62, 0.90],
            ambient: 0.12,
            specular: 0.55,
            shininess: 48.0,
            chord_tolerance: 0.005,
        }
    }
}

/// Two-light preset: a warm key + cool fill.
pub fn default_lights() -> Vec<Light> {
    vec![
        // Key light — warm white from upper-right-front.
        Light {
            direction: Vec3::new(1.2, 1.0, 1.8).normalize(),
            color: [1.0, 0.97, 0.90],
        },
        // Fill light — cool from left.
        Light {
            direction: Vec3::new(-1.0, 0.4, 0.5).normalize(),
            color: [0.35, 0.45, 0.65],
        },
    ]
}

/// Render `store` and return PNG bytes.
pub fn render_to_png(
    store: &ShapeStore,
    camera: &Camera,
    opts: &RenderOptions,
) -> Result<Vec<u8>, RenderError> {
    let rgb = render_rgb(store, camera, &default_lights(), opts)?;
    encode_png(opts.width, opts.height, &rgb)
}

/// Render `store` with custom lights and return PNG bytes.
pub fn render_to_png_lit(
    store: &ShapeStore,
    camera: &Camera,
    lights: &[Light],
    opts: &RenderOptions,
) -> Result<Vec<u8>, RenderError> {
    let rgb = render_rgb(store, camera, lights, opts)?;
    encode_png(opts.width, opts.height, &rgb)
}

/// Error type for rendering.
#[derive(Debug)]
pub enum RenderError {
    Tessellation(brep_core::KernelError),
    Encode(String),
}
impl std::fmt::Display for RenderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Tessellation(e) => write!(f, "tessellation: {e}"),
            Self::Encode(e) => write!(f, "PNG encode: {e}"),
        }
    }
}
impl std::error::Error for RenderError {}

// ── Core rasterizer ───────────────────────────────────────────────────────────

fn render_rgb(
    store: &ShapeStore,
    camera: &Camera,
    lights: &[Light],
    opts: &RenderOptions,
) -> Result<Vec<u8>, RenderError> {
    let w = opts.width as usize;
    let h = opts.height as usize;

    // Tessellate.
    let tess_opts = TessellationOptions {
        chord_tolerance: opts.chord_tolerance,
        min_segments: 16,
    };
    let meshes = tessellate(store, &tess_opts).map_err(RenderError::Tessellation)?;

    // View + projection matrices.
    let view = look_at(&camera.eye, &camera.target, &camera.up);
    let near = 0.05;
    let far  = 1000.0;
    let proj = perspective(camera.fov_y_deg.to_radians(), w as f64 / h as f64, near, far);
    let vp: Matrix4<f64> = proj * view;

    // Pixel + depth buffers.
    let mut pixels: Vec<[u8; 3]> = vec![[opts.background[0], opts.background[1], opts.background[2]]; w * h];
    let mut depth: Vec<f32> = vec![1.0; w * h]; // NDC z, smaller = closer

    let view_dir: Vec3 = (camera.eye - camera.target).normalize();

    for mesh in &meshes {
        for tri in &mesh.triangles {
            rasterize(
                &vp, w, h,
                &tri.positions, tri.normal,
                &view_dir, lights, opts,
                &mut pixels, &mut depth,
            );
        }
    }

    // Flatten to raw RGB bytes.
    let mut rgb = Vec::with_capacity(w * h * 3);
    for p in &pixels {
        rgb.extend_from_slice(p);
    }
    Ok(rgb)
}

/// Rasterise a single triangle into the pixel + depth buffers.
#[allow(clippy::too_many_arguments)]
fn rasterize(
    vp: &Matrix4<f64>,
    w: usize,
    h: usize,
    positions: &[Point3; 3],
    normal: Vec3,
    view_dir: &Vec3,
    lights: &[Light],
    opts: &RenderOptions,
    pixels: &mut Vec<[u8; 3]>,
    depth: &mut Vec<f32>,
) {
    // Backface cull: skip triangles facing away from the camera.
    if normal.dot(view_dir) < 0.0 {
        return;
    }

    // Project all three vertices to clip space, then NDC, then screen.
    let clip: [Vector4<f64>; 3] = [
        vp * Vector4::new(positions[0].x, positions[0].y, positions[0].z, 1.0),
        vp * Vector4::new(positions[1].x, positions[1].y, positions[1].z, 1.0),
        vp * Vector4::new(positions[2].x, positions[2].y, positions[2].z, 1.0),
    ];

    // Discard triangles fully behind the near/far plane.
    if clip.iter().all(|c| c.w <= 0.0) { return; }

    let ndc: [(f64, f64, f64); 3] = [
        (clip[0].x / clip[0].w, clip[0].y / clip[0].w, clip[0].z / clip[0].w),
        (clip[1].x / clip[1].w, clip[1].y / clip[1].w, clip[1].z / clip[1].w),
        (clip[2].x / clip[2].w, clip[2].y / clip[2].w, clip[2].z / clip[2].w),
    ];

    // NDC → screen pixels (y-down).
    let sx: [f64; 3] = [
        (ndc[0].0 + 1.0) * 0.5 * (w as f64),
        (ndc[1].0 + 1.0) * 0.5 * (w as f64),
        (ndc[2].0 + 1.0) * 0.5 * (w as f64),
    ];
    let sy: [f64; 3] = [
        (1.0 - ndc[0].1) * 0.5 * (h as f64),
        (1.0 - ndc[1].1) * 0.5 * (h as f64),
        (1.0 - ndc[2].1) * 0.5 * (h as f64),
    ];
    let sz: [f64; 3] = [ndc[0].2, ndc[1].2, ndc[2].2];

    // Bounding box (integer, clamped).
    let x_min = sx.iter().cloned().fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let x_max = sx.iter().cloned().fold(f64::MIN, f64::max).ceil().min(w as f64) as usize;
    let y_min = sy.iter().cloned().fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let y_max = sy.iter().cloned().fold(f64::MIN, f64::max).ceil().min(h as f64) as usize;

    // Area of the triangle in screen space (signed; used for barycentric denom).
    let area = edge_fn(sx[0], sy[0], sx[1], sy[1], sx[2], sy[2]);
    if area.abs() < 1e-4 { return; }
    let inv_area = 1.0 / area;

    // Compute fragment colour once (flat shading, normal is constant per tri).
    let color = shade(normal, view_dir, lights, opts);

    for py in y_min..y_max {
        for px in x_min..x_max {
            let fx = px as f64 + 0.5;
            let fy = py as f64 + 0.5;

            // Barycentric weights w.r.t. vertex 0, 1, 2.
            let w2 = edge_fn(sx[0], sy[0], sx[1], sy[1], fx, fy) * inv_area;
            let w0 = edge_fn(sx[1], sy[1], sx[2], sy[2], fx, fy) * inv_area;
            let w1 = edge_fn(sx[2], sy[2], sx[0], sy[0], fx, fy) * inv_area;

            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 { continue; }

            // Interpolate depth.
            let fz = (w0 * sz[0] + w1 * sz[1] + w2 * sz[2]) as f32;
            if fz < -1.0 || fz > 1.0 { continue; }

            let idx = py * w + px;
            if fz < depth[idx] {
                depth[idx] = fz;
                pixels[idx] = color;
            }
        }
    }
}

/// 2D edge function (positive when P is to the left of A→B).
#[inline]
fn edge_fn(ax: f64, ay: f64, bx: f64, by: f64, px: f64, py: f64) -> f64 {
    (bx - ax) * (py - ay) - (by - ay) * (px - ax)
}

// ── Shading ───────────────────────────────────────────────────────────────────

fn shade(
    normal: Vec3,
    view_dir: &Vec3,
    lights: &[Light],
    opts: &RenderOptions,
) -> [u8; 3] {
    let n = normal.normalize();
    let v = view_dir.normalize();

    let mut r = opts.ambient * opts.face_color[0];
    let mut g = opts.ambient * opts.face_color[1];
    let mut b = opts.ambient * opts.face_color[2];

    for light in lights {
        let l = light.direction.normalize();
        let diff = n.dot(&l).max(0.0) as f32;

        // Blinn-Phong half-vector.
        let h = (l + v).normalize();
        let spec = n.dot(&h).max(0.0).powf(opts.shininess as f64) as f32 * opts.specular;

        r += diff * light.color[0] * opts.face_color[0] + spec * light.color[0];
        g += diff * light.color[1] * opts.face_color[1] + spec * light.color[1];
        b += diff * light.color[2] * opts.face_color[2] + spec * light.color[2];
    }

    [linear_to_srgb(r), linear_to_srgb(g), linear_to_srgb(b)]
}

/// Linear light value → gamma-corrected sRGB u8.
#[inline]
fn linear_to_srgb(x: f32) -> u8 {
    (x.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0).round() as u8
}

// ── Camera math ───────────────────────────────────────────────────────────────

/// Right-handed look-at view matrix.
fn look_at(eye: &Point3, target: &Point3, up: &Vec3) -> Matrix4<f64> {
    let f = (target - eye).normalize(); // forward
    let s = f.cross(up).normalize();    // right
    let u = s.cross(&f);                // recomputed up

    Matrix4::new(
         s.x,  s.y,  s.z, -s.dot(&eye.coords),
         u.x,  u.y,  u.z, -u.dot(&eye.coords),
        -f.x, -f.y, -f.z,  f.dot(&eye.coords),
         0.0,  0.0,  0.0,  1.0,
    )
}

/// Right-handed perspective projection (NDC z in [-1, +1]).
fn perspective(fov_y_rad: f64, aspect: f64, near: f64, far: f64) -> Matrix4<f64> {
    let f = 1.0 / (fov_y_rad * 0.5).tan();
    Matrix4::new(
        f / aspect, 0.0,  0.0,                            0.0,
        0.0,        f,    0.0,                            0.0,
        0.0,        0.0,  (far + near) / (near - far),    (2.0 * far * near) / (near - far),
        0.0,        0.0, -1.0,                            0.0,
    )
}

// ── PNG encoding ─────────────────────────────────────────────────────────────

fn encode_png(w: u32, h: u32, rgb: &[u8]) -> Result<Vec<u8>, RenderError> {
    let mut buf = Vec::new();
    {
        let mut enc = png::Encoder::new(Cursor::new(&mut buf), w, h);
        enc.set_color(png::ColorType::Rgb);
        enc.set_depth(png::BitDepth::Eight);
        let mut writer = enc
            .write_header()
            .map_err(|e| RenderError::Encode(e.to_string()))?;
        writer
            .write_image_data(rgb)
            .map_err(|e| RenderError::Encode(e.to_string()))?;
    }
    Ok(buf)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::primitives::{make_box, make_cylinder, make_cone, make_sphere};

    fn small_opts() -> RenderOptions {
        RenderOptions { width: 64, height: 64, chord_tolerance: 0.02, ..Default::default() }
    }

    #[test]
    fn render_box_returns_png() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(2.5, 2.5, 2.0), Point3::new(0.5, 0.5, 0.5)),
            &small_opts(),
        ).unwrap();
        // PNG magic bytes: 137 80 78 71
        assert_eq!(&png[..4], &[137, 80, 78, 71]);
    }

    #[test]
    fn render_cylinder_returns_png() {
        let mut store = ShapeStore::new();
        make_cylinder(&mut store, 1.0, 2.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.0, 2.5, 2.0), Point3::new(0.0, 0.0, 1.0)),
            &small_opts(),
        ).unwrap();
        assert_eq!(&png[..4], &[137, 80, 78, 71]);
    }

    #[test]
    fn render_sphere_returns_png() {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.0, 2.0, 1.5), Point3::origin()),
            &small_opts(),
        ).unwrap();
        assert_eq!(&png[..4], &[137, 80, 78, 71]);
    }

    #[test]
    fn render_cone_returns_png() {
        let mut store = ShapeStore::new();
        make_cone(&mut store, 1.0, 2.0).unwrap();
        let png = render_to_png(
            &store,
            &Camera::look_at(Point3::new(3.0, 2.5, 2.0), Point3::new(0.0, 0.0, 1.0)),
            &small_opts(),
        ).unwrap();
        assert_eq!(&png[..4], &[137, 80, 78, 71]);
    }

    #[test]
    fn render_produces_non_background_pixels() {
        // At full resolution the object should cover some pixels.
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        let opts = RenderOptions { width: 200, height: 200, chord_tolerance: 0.02, ..Default::default() };
        let rgb = render_rgb(
            &store,
            &Camera::look_at(Point3::new(3.0, 2.0, 1.5), Point3::origin()),
            &default_lights(),
            &opts,
        ).unwrap();
        let bg = opts.background;
        let non_bg = rgb.chunks(3).filter(|p| p != &[bg[0], bg[1], bg[2]]).count();
        assert!(non_bg > 500, "expected object pixels, got {non_bg}");
    }
}
