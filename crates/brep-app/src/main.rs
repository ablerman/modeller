//! BRep Modeller — interactive solid-modelling application.
//!
//! Architecture:
//!   winit event loop → wgpu 3-D renderer + egui overlay → BRep kernel

mod commands;
mod editor;
mod gizmo;
mod icons;
mod profile_shapes;
mod sketch_tools;
mod toolbar;
mod toolbar_defs;
mod ui;

use std::sync::Arc;

use brep_algo::bvh::Ray;
use brep_config::ConfigStore;
use brep_core::{Point3, Vec3};
use brep_render::gpu::GpuRenderer;
use editor::{EditorState, UiAction};
use egui_wgpu::ScreenDescriptor;
use winit::{
    application::ApplicationHandler,
    dpi::{PhysicalPosition, PhysicalSize},
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

// ── Window geometry ───────────────────────────────────────────────────────────

/// Saved window position and size, in physical pixels.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct WindowGeometry {
    x: i32,
    y: i32,
    width: u32,
    height: u32,
}

impl Default for WindowGeometry {
    fn default() -> Self {
        Self { x: 100, y: 100, width: 1600, height: 1000 }
    }
}

// ── GPU mesh cache ────────────────────────────────────────────────────────────

struct GpuMeshEntry {
    mesh: brep_render::gpu::GpuMesh,
    #[allow(dead_code)]
    selected: bool,
}

// ── Per-window state ──────────────────────────────────────────────────────────

struct AppState {
    window: Arc<Window>,

    // wgpu core
    surface: wgpu::Surface<'static>,
    surface_config: wgpu::SurfaceConfiguration,
    device: wgpu::Device,
    queue: wgpu::Queue,

    // 3-D renderer
    gpu_renderer: GpuRenderer,
    gpu_meshes: Vec<GpuMeshEntry>,
    /// Cached tessellated (positions, normals) per object ID.
    /// Avoids re-tessellating unchanged objects when the scene is dirtied.
    tess_cache: std::collections::HashMap<u64, (Vec<[f32; 3]>, Vec<[f32; 3]>)>,

    // egui
    egui_ctx: egui::Context,
    egui_state: egui_winit::State,
    egui_renderer: egui_wgpu::Renderer,

    // application
    editor: EditorState,
    current_file: Option<std::path::PathBuf>,

    // mouse state for orbit/pan/pick
    mouse_pressed: Option<MouseButton>,
    last_cursor: Option<(f32, f32)>,
    /// Position where mouse button was first pressed (for click-vs-drag detection).
    mouse_press_origin: Option<(f32, f32)>,
    /// Current cursor position projected onto the active sketch plane (if in sketch mode).
    sketch_cursor: Option<Point3>,
    /// Index of the sketch vertex the cursor is currently snapping to (if any).
    snap_vertex: Option<usize>,
    /// Index of the sketch segment the cursor is hovering over (if any).
    snap_segment: Option<usize>,
    /// Reference entity (origin / axis) the cursor is snapping to (if any).
    snap_ref: Option<editor::RefEntity>,
    /// When Some((seg_a, seg_b)), the angle-input modal dialog is open.
    angle_dialog: Option<(usize, usize)>,
    /// When Some((target, initial_len)), the length-input modal dialog is open.
    /// `initial_len` is the measured length/distance at the moment the dialog was opened.
    length_dialog: Option<(editor::LengthTarget, f64)>,
    /// Index of the sketch constraint marker the cursor is hovering over (if any).
    snap_constraint: Option<usize>,
    /// Index of the vertex currently being dragged (set on mouse-press over a snap vertex).
    drag_vertex: Option<usize>,
    /// (profile_idx, vertex_idx) in committed_profiles that the cursor is snapping to.
    snap_committed: Option<(usize, usize)>,
    /// (profile_idx, vertex_idx) currently being dragged within committed_profiles.
    drag_committed: Option<(usize, usize)>,
    /// Index of committed circle/arc profile whose curve the cursor is hovering over.
    snap_committed_curve: Option<usize>,
    /// Index of committed circle/arc profile whose curve is being dragged (resize).
    drag_committed_curve: Option<usize>,
    /// (profile_idx, segment_idx) of committed polyline segment the cursor is hovering over.
    snap_committed_seg: Option<(usize, usize)>,
    /// Time of the most recent click (for double-click detection).
    last_click_time: Option<std::time::Instant>,
    /// Screen position of the most recent click (for double-click detection).
    last_click_pos: Option<(f32, f32)>,
    /// Whether SketchSaveDragHistory has been pushed for the current drag gesture.
    /// Reset to false on mouse-press; set to true on the first actual movement.
    drag_history_saved: bool,
}

impl AppState {
    async fn new(window: Arc<Window>) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("no wgpu adapter found");

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("brep device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: Default::default(),
                },
                None,
            )
            .await
            .expect("failed to create wgpu device");

        let caps = surface.get_capabilities(&adapter);
        let surface_format = caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(caps.formats[0]);

        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::AutoVsync,
            desired_maximum_frame_latency: 2,
            alpha_mode: caps.alpha_modes[0],
            view_formats: vec![],
        };
        surface.configure(&device, &surface_config);

        let gpu_renderer = GpuRenderer::new(
            &device,
            surface_format,
            DEPTH_FORMAT,
            surface_config.width,
            surface_config.height,
        );

        // egui setup
        let egui_ctx = egui::Context::default();
        egui_extras::install_image_loaders(&egui_ctx);
        let egui_state = egui_winit::State::new(
            egui_ctx.clone(),
            egui::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            None,
        );
        // egui-wgpu 0.30 Renderer::new(device, output_format, depth_format, samples, dithering)
        let egui_renderer = egui_wgpu::Renderer::new(&device, surface_format, None, 1, false);

        let editor = EditorState::new();

        Self {
            window,
            surface,
            surface_config,
            device,
            queue,
            gpu_renderer,
            gpu_meshes: Vec::new(),
            tess_cache: std::collections::HashMap::new(),
            egui_ctx,
            egui_state,
            egui_renderer,
            editor,
            current_file: None,
            mouse_pressed: None,
            last_cursor: None,
            mouse_press_origin: None,
            sketch_cursor: None,
            snap_vertex: None,
            snap_segment: None,
            snap_ref: None,
            angle_dialog: None,
            length_dialog: None,
            snap_constraint: None,
            drag_vertex: None,
            snap_committed: None,
            drag_committed: None,
            snap_committed_curve: None,
            drag_committed_curve: None,
            snap_committed_seg: None,
            last_click_time: None,
            last_click_pos: None,
            drag_history_saved: false,
        }
    }

    fn resize(&mut self, new_size: PhysicalSize<u32>) {
        if new_size.width == 0 || new_size.height == 0 {
            return;
        }
        self.surface_config.width = new_size.width;
        self.surface_config.height = new_size.height;
        self.surface.configure(&self.device, &self.surface_config);
        self.gpu_renderer.resize(&self.device, new_size.width, new_size.height);
    }

    /// Re-upload GPU meshes whenever the scene changes.
    ///
    /// Tessellation results are cached per object ID so that adding or selecting
    /// one object does not re-tessellate all others.
    fn sync_meshes(&mut self) {
        if !self.editor.scene_dirty {
            return;
        }
        self.editor.scene_dirty = false;

        // Evict cache entries for IDs no longer in the scene.
        let live_ids: std::collections::HashSet<u64> =
            self.editor.entries.iter().map(|e| e.id()).collect();
        self.tess_cache.retain(|id, _| live_ids.contains(id));

        self.gpu_meshes.clear();
        for (i, entry) in self.editor.entries.iter().enumerate() {
            let obj = match entry { editor::SceneEntry::Solid(s) => s, _ => continue };
            let selected = self.editor.selection.contains(&i);
            let base_color = if selected { [0.90, 0.70, 0.20f32] } else { [0.38, 0.62, 0.90f32] };
            let cached = self.tess_cache.entry(obj.id).or_insert_with(|| {
                brep_render::prepare_mesh_cpu(&obj.store)
            });
            let verts = brep_render::apply_color(&cached.0, &cached.1, base_color);
            let mesh = self.gpu_renderer.upload_vertices(&self.device, &verts);
            self.gpu_meshes.push(GpuMeshEntry { mesh, selected });
        }
    }

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.editor.tick_animation();
        self.sync_meshes();

        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("frame encoder"),
            });

        // 3-D pass — clear and draw solid geometry.
        {
            let bg = wgpu::Color { r: 0.118, g: 0.118, b: 0.149, a: 1.0 };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3d pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(bg),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.gpu_renderer.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });

            let cam = &self.editor.camera;
            let meshes: Vec<&brep_render::gpu::GpuMesh> =
                self.gpu_meshes.iter().map(|e| &e.mesh).collect();
            self.gpu_renderer.render(
                &mut pass,
                &meshes,
                &self.queue,
                cam.eye(),
                cam.target,
                cam.up(),
                cam.fov_y_deg,
            );
        }

        // egui — build UI, tessellate, upload, render.
        let egui_ctx = self.egui_ctx.clone();
        let raw_input = self.egui_state.take_egui_input(&self.window);
        let mut actions: Vec<UiAction> = Vec::new();
        let sketch_cursor = self.sketch_cursor;
        let snap_vertex = self.snap_vertex;
        let snap_segment = self.snap_segment;
        let snap_ref = self.snap_ref;
        let snap_constraint = self.snap_constraint;
        let snap_committed = self.snap_committed;
        let snap_committed_curve = self.snap_committed_curve;
        let snap_committed_seg = self.snap_committed_seg;
        let angle_dialog = self.angle_dialog;
        let length_dialog = self.length_dialog;  // Option<(LengthTarget, f64)>
        let vp = (self.surface_config.width, self.surface_config.height);
        let full_output = egui_ctx.run(raw_input, |ctx| {
            actions = ui::build_ui(ctx, &self.editor, vp, sketch_cursor, snap_vertex, snap_segment, snap_ref, snap_constraint, snap_committed, snap_committed_curve, snap_committed_seg, angle_dialog, length_dialog);
        });

        let tris = egui_ctx.tessellate(full_output.shapes, egui_ctx.pixels_per_point());
        let screen_desc = ScreenDescriptor {
            size_in_pixels: [self.surface_config.width, self.surface_config.height],
            pixels_per_point: egui_ctx.pixels_per_point(),
        };

        for (id, delta) in &full_output.textures_delta.set {
            self.egui_renderer
                .update_texture(&self.device, &self.queue, *id, delta);
        }
        for id in &full_output.textures_delta.free {
            self.egui_renderer.free_texture(id);
        }

        // In egui-wgpu 0.30, update_buffers returns additional CommandBuffers to submit.
        let egui_cmds = self.egui_renderer.update_buffers(
            &self.device,
            &self.queue,
            &mut encoder,
            &tris,
            &screen_desc,
        );

        // egui-wgpu 0.30 render() takes &mut RenderPass<'static>.
        // We use forget_lifetime() to convert RenderPass<'enc> → RenderPass<'static>;
        // this is safe because the pass is dropped before the encoder is finished.
        {
            let mut pass = encoder
                .begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("egui pass"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: None,
                    ..Default::default()
                })
                .forget_lifetime();
            self.egui_renderer.render(&mut pass, &tris, &screen_desc);
        }

        // Submit: egui's extra command buffers first, then the main encoder.
        self.queue
            .submit(egui_cmds.into_iter().chain(std::iter::once(encoder.finish())));
        output.present();

        self.egui_state
            .handle_platform_output(&self.window, full_output.platform_output);

        // Apply UI actions after rendering (avoids borrow conflicts during egui run).
        let mut needs_mesh_update = false;
        for action in actions {
            needs_mesh_update |= self.dispatch_action(action);
        }
        if needs_mesh_update {
            self.sync_meshes();
        }

        Ok(())
    }
}

// ── Command dispatch ──────────────────────────────────────────────────────────

impl AppState {
    /// Dispatch one `UiAction`, intercepting app-level actions before passing
    /// to `EditorState::apply`.  Returns `true` if GPU meshes need re-upload.
    fn dispatch_action(&mut self, action: UiAction) -> bool {
        // Intercept actions that modify AppState rather than EditorState.
        match &action {
            UiAction::New    => { self.do_new();     return false; }
            UiAction::Open   => { self.do_open();    return false; }
            UiAction::Save   => { self.do_save();    return false; }
            UiAction::SaveAs => { self.do_save_as(); return false; }
            UiAction::SketchBeginAngleInput { seg_a, seg_b } => {
                self.angle_dialog = Some((*seg_a, *seg_b));
                return false;
            }
            UiAction::SketchCancelAngleInput => {
                self.angle_dialog = None;
                return false;
            }
            UiAction::SketchBeginLengthInput(target) => {
                let initial = current_length(*target, &self.editor.sketch);
                self.length_dialog = Some((*target, initial));
                return false;
            }
            UiAction::SketchCancelLengthInput => {
                self.length_dialog = None;
                return false;
            }
            // Confirm (SketchAddConstraint) closes both dialogs.
            UiAction::SketchAddConstraint(_) => {
                self.angle_dialog = None;
                self.length_dialog = None;
            }
            UiAction::ExitSketch => {
                self.angle_dialog = None;
                self.length_dialog = None;
            }
            _ => {}
        }
        self.editor.apply(action)
    }

    /// Invoke a command by its string ID.  Returns `true` if GPU meshes need
    /// re-upload, `false` if the command was unknown or produced no geometry change.
    #[allow(dead_code)]
    pub fn execute_command(&mut self, id: &str) -> bool {
        let Some(spec) = commands::lookup_command(id) else { return false };
        let actions = commands::resolve(spec, &self.editor);
        let mut changed = false;
        for action in actions {
            changed |= self.dispatch_action(action);
        }
        if changed { self.sync_meshes(); }
        changed
    }
}

// ── File I/O ──────────────────────────────────────────────────────────────────

impl AppState {
    fn update_window_title(&self) {
        let title = match &self.current_file {
            Some(path) => {
                let name = path.file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or("untitled");
                format!("BRep Modeller — {name}")
            }
            None => "BRep Modeller".to_string(),
        };
        self.window.set_title(&title);
    }

    fn do_save_to(&mut self, path: std::path::PathBuf) {
        let scene = self.editor.to_saved_scene();
        match serde_json::to_string_pretty(&scene) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&path, json) {
                    eprintln!("save error: {e}");
                } else {
                    self.current_file = Some(path);
                    self.update_window_title();
                }
            }
            Err(e) => eprintln!("serialize error: {e}"),
        }
    }


    fn do_save(&mut self) {
        if let Some(path) = self.current_file.clone() {
            self.do_save_to(path);
        } else {
            self.do_save_as();
        }
    }

    fn do_save_as(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .set_title("Save Scene")
            .add_filter("Modeller scene", &["modl"])
            .save_file()
        {
            let path = if path.extension().is_none() {
                path.with_extension("modl")
            } else {
                path
            };
            self.do_save_to(path);
        }
    }

    fn do_open_file(&mut self, path: std::path::PathBuf) {
        match std::fs::read_to_string(&path) {
            Ok(json) => match serde_json::from_str::<editor::SavedScene>(&json) {
                Ok(scene) => {
                    self.editor.load_saved_scene(scene);
                    self.tess_cache.clear();
                    self.current_file = Some(path);
                    self.update_window_title();
                }
                Err(e) => eprintln!("parse error: {e}"),
            },
            Err(e) => eprintln!("read error: {e}"),
        }
    }

    fn do_open(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .set_title("Open Scene")
            .add_filter("Modeller scene", &["modl"])
            .pick_file()
        {
            self.do_open_file(path);
        }
    }

    fn do_new(&mut self) {
        self.editor.load_saved_scene(editor::SavedScene {
            version: 2,
            entries: Vec::new(),
            camera:  editor::SavedCamera {
                target:    [0.0, 0.0, 0.0],
                distance:  6.0,
                azimuth:   0.8,
                elevation: 0.5,
            },
        });
        self.tess_cache.clear();
        self.current_file = None;
        self.update_window_title();
    }
}

// ── Sketch helpers ────────────────────────────────────────────────────────────

fn ray_plane_intersect(ray: &Ray, plane_origin: &Point3, plane_normal: &Vec3) -> Option<Point3> {
    let denom = ray.direction.dot(plane_normal);
    if denom.abs() < 1e-12 { return None; }
    let t = (plane_origin - ray.origin).dot(plane_normal) / denom;
    if t < 0.0 { return None; }
    Some(ray.at(t))
}

/// Measure the current length/distance for a `LengthTarget` from sketch geometry.
fn current_length(target: editor::LengthTarget, sketch: &Option<editor::SketchState>) -> f64 {
    let Some(sk) = sketch else { return 1.0 };
    let n = sk.points.len();
    if n == 0 { return 1.0; }
    match target {
        editor::LengthTarget::Segment(seg) if seg < n => {
            let a = sk.points[seg];
            let b = sk.points[(seg + 1) % n];
            (b - a).norm()
        }
        editor::LengthTarget::Points(pa, pb) if pa < n && pb < n => {
            (sk.points[pb] - sk.points[pa]).norm()
        }
        _ => 1.0,
    }
}

/// Perpendicular distance from point `(px, py)` to segment `(ax,ay)→(bx,by)`.
fn point_to_segment_dist(px: f32, py: f32, ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    let dx = bx - ax;
    let dy = by - ay;
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-6 {
        return ((px - ax).powi(2) + (py - ay).powi(2)).sqrt();
    }
    let t = ((px - ax) * dx + (py - ay) * dy) / len2;
    let t = t.clamp(0.0, 1.0);
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
}

/// Primary screen-space position (physical pixels) of a constraint's visual marker.
/// Returns `None` for constraints that have no drawn marker (e.g. `PointFixed`).
fn constraint_marker_screen_pos(
    c: &brep_sketch::SketchConstraint,
    points: &[Point3],
    n: usize,
    cam: &editor::ViewportCamera,
    w: u32,
    h: u32,
) -> Option<(f32, f32)> {
    use brep_sketch::SketchConstraint as SC;
    let midpoint_pt = |seg: usize| -> Option<Point3> {
        let a = points.get(seg)?;
        let b = points.get((seg + 1) % n)?;
        Some(Point3::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5))
    };
    let midpoint_pts = |pa: usize, pb: usize| -> Option<Point3> {
        let a = points.get(pa)?;
        let b = points.get(pb)?;
        Some(Point3::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5))
    };
    let world_pos: Option<Point3> = match c {
        SC::Parallel { seg_a, .. }        => midpoint_pt(*seg_a),
        SC::Perpendicular { seg_a, .. }   => midpoint_pt(*seg_a),
        SC::Angle { seg_a, .. }           => midpoint_pt(*seg_a),
        SC::Horizontal { seg }            => midpoint_pt(*seg),
        SC::Vertical { seg }              => midpoint_pt(*seg),
        SC::EqualLength { seg_a, .. }     => midpoint_pt(*seg_a),
        SC::Coincident { pt_a, .. }       => points.get(*pt_a).copied(),
        SC::FixedLength { seg, .. }       => midpoint_pt(*seg),
        SC::PointDistance { pt_a, pt_b, .. } => midpoint_pts(*pt_a, *pt_b),
        SC::PointFixed { .. }             => None,
        SC::PointOnOrigin { pt }          => points.get(*pt).copied(),
        SC::PointOnXAxis { pt }           => points.get(*pt).copied(),
        SC::PointOnYAxis { pt }           => points.get(*pt).copied(),
        SC::HorizontalPair { pt_a, pt_b, .. } => midpoint_pts(*pt_a, *pt_b),
        SC::VerticalPair { pt_a, pt_b, .. }   => midpoint_pts(*pt_a, *pt_b),
        SC::PointOnLine { pt, .. }        => points.get(*pt).copied(),
        SC::PointOnCircle { pt, .. }      => points.get(*pt).copied(),
    };
    world_pos.and_then(|p| cam.project_to_screen(p, w, h))
}

// ── winit ApplicationHandler ──────────────────────────────────────────────────

struct App {
    state: Option<AppState>,
    config: ConfigStore,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let geo = self.config
            .get::<WindowGeometry>("window.geometry")
            .ok()
            .flatten()
            .unwrap_or_default();
        let attrs = Window::default_attributes()
            .with_title("BRep Modeller")
            .with_inner_size(PhysicalSize::new(geo.width, geo.height))
            .with_position(PhysicalPosition::new(geo.x, geo.y));
        let window = Arc::new(event_loop.create_window(attrs).unwrap());
        let mut state = pollster::block_on(AppState::new(window));

        // Reopen the last file that was open when the app was closed.
        if let Ok(Some(path_str)) = self.config.get::<String>("last_file") {
            let path = std::path::PathBuf::from(path_str);
            if path.exists() {
                state.do_open_file(path);
            }
        }

        self.state = Some(state);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        let state = match self.state.as_mut() {
            Some(s) => s,
            None => return,
        };

        // Let egui consume relevant events first.
        let egui_consumed = state
            .egui_state
            .on_window_event(&state.window, &event)
            .consumed;

        match event {
            WindowEvent::CloseRequested => {
                // Persist the currently open file path so we can reopen it next launch.
                if let Some(path) = &state.current_file {
                    if let Some(s) = path.to_str() {
                        let _ = self.config.set("last_file", &s.to_string());
                    }
                }
                event_loop.exit();
            }

            WindowEvent::KeyboardInput { event: key_event, .. } => {
                if !egui_consumed && key_event.state == ElementState::Pressed {
                    let modifiers =
                        state.egui_ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
                    if modifiers {
                        match key_event.physical_key {
                            PhysicalKey::Code(KeyCode::KeyZ) => {
                                state.dispatch_action(UiAction::Undo);
                            }
                            PhysicalKey::Code(KeyCode::KeyY) => {
                                state.dispatch_action(UiAction::Redo);
                            }
                            PhysicalKey::Code(KeyCode::KeyS) => {
                                state.dispatch_action(UiAction::Save);
                            }
                            PhysicalKey::Code(KeyCode::KeyO) => {
                                state.dispatch_action(UiAction::Open);
                            }
                            PhysicalKey::Code(KeyCode::KeyN) => {
                                state.dispatch_action(UiAction::New);
                            }
                            _ => {}
                        }
                    }
                    if matches!(
                        key_event.physical_key,
                        PhysicalKey::Code(KeyCode::Delete | KeyCode::Backspace)
                    ) {
                        state.dispatch_action(UiAction::DeleteSelected);
                    }
                    if matches!(key_event.physical_key, PhysicalKey::Code(KeyCode::Escape)) {
                        state.dispatch_action(UiAction::SketchAbortActive);
                    }
                    // Enter: commit the active polyline as an open profile.
                    if matches!(key_event.physical_key, PhysicalKey::Code(KeyCode::Enter | KeyCode::NumpadEnter)) {
                        if state.editor.sketch.is_some() {
                            state.dispatch_action(UiAction::SketchCommitPolyline);
                        }
                    }
                }
            }

            WindowEvent::MouseInput { state: btn_state, button, .. } => {
                if !egui_consumed {
                    match btn_state {
                        ElementState::Pressed => {
                            state.mouse_pressed = Some(button);
                            state.mouse_press_origin = state.last_cursor;
                            if button == MouseButton::Left && state.editor.sketch.is_some() {
                                // Committed vertex drag (pointer tool — takes priority).
                                let is_pointer = state.editor.sketch.as_ref()
                                    .map_or(false, |sk| sk.active_tool == editor::DrawTool::Pointer);
                                if is_pointer && state.snap_committed.is_some() {
                                    state.drag_committed = state.snap_committed;
                                    state.drag_history_saved = false;
                                } else if is_pointer && state.snap_committed_curve.is_some() {
                                    state.drag_committed_curve = state.snap_committed_curve;
                                    state.drag_history_saved = false;
                                } else if state.snap_vertex.is_some() {
                                    // Begin active-profile vertex drag.
                                    state.drag_vertex = state.snap_vertex;
                                    state.drag_history_saved = false;
                                }
                            }
                        }
                        ElementState::Released => {
                            // If left button released with little movement → pick.
                            if button == MouseButton::Left {
                                let is_click = match (state.mouse_press_origin, state.last_cursor) {
                                    (Some(origin), Some(cur)) => {
                                        let dx = (cur.0 - origin.0).abs();
                                        let dy = (cur.1 - origin.1).abs();
                                        dx < 4.0 && dy < 4.0
                                    }
                                    _ => false,
                                };

                                // Committed curve drag released.
                                if let Some(pi) = state.drag_committed_curve.take() {
                                    if is_click {
                                        state.editor.apply(UiAction::SketchSelectCommitted(Some(pi)));
                                    }
                                    // Drag already moved the boundary point; nothing else needed.
                                }
                                // Committed vertex drag released.
                                if let Some((pi, vi)) = state.drag_committed.take() {
                                    if is_click {
                                        let is_pointer = state.editor.sketch.as_ref()
                                            .map_or(false, |sk| sk.active_tool == editor::DrawTool::Pointer);
                                        if is_pointer {
                                            let shape = state.editor.sketch.as_ref()
                                                .and_then(|sk| sk.committed_profiles.get(pi))
                                                .map(|cp| cp.shape.clone());
                                            // Arc: all three control points (start, end, center) are
                                            // individually selectable for constraints.
                                            // Everything else → profile selection.
                                            if shape.as_ref().map_or(false, |s| s.vertex_selectable(vi)) {
                                                state.editor.apply(UiAction::SketchSelectCommittedPoint(Some((pi, vi))));
                                            } else {
                                                state.editor.apply(UiAction::SketchSelectCommitted(Some(pi)));
                                            }
                                        } else {
                                            // Drawing tool: click on committed vertex → add point there (snap to it).
                                            let pt = state.editor.sketch.as_ref()
                                                .and_then(|sk| {
                                                    let cp = sk.committed_profiles.get(pi)?;
                                                    let gi = cp.point_indices.get(vi)?;
                                                    sk.global_points.get(*gi).copied()
                                                });
                                            if let Some(p) = pt {
                                                state.editor.apply(UiAction::SketchAddPoint(p));
                                            }
                                        }
                                    }
                                    // Drag already moved the point; nothing else needed.
                                } else if let Some(drag_vi) = state.drag_vertex.take() {
                                    // We were dragging an active-profile vertex. Tiny movement = click.
                                    if is_click {
                                        let tool_busy = state.editor.sketch.as_ref()
                                            .map_or(false, |sk| sk.tool_in_progress.is_some());
                                        if tool_busy {
                                            // Multi-step tool in progress: use vertex position as a tool point.
                                            if let Some(p) = state.editor.sketch.as_ref()
                                                .and_then(|sk| sk.points.get(drag_vi).copied())
                                            {
                                                state.editor.apply(UiAction::SketchAddPoint(p));
                                            }
                                        } else {
                                            let can_close = state.editor.sketch.as_ref()
                                                .map_or(false, |sk| !sk.closed && sk.points.len() >= 3);
                                            if drag_vi == 0 && can_close {
                                                state.editor.apply(UiAction::SketchCloseLoop);
                                            } else {
                                                state.editor.apply(UiAction::SketchSelectVertex(drag_vi));
                                            }
                                        }
                                    }
                                    // Otherwise the drag already updated the positions;
                                    // nothing more to do on release.
                                } else if is_click {
                                    let is_double_click = if let (Some(t), Some(pos)) = (state.last_click_time, state.last_click_pos) {
                                        let cur = state.last_cursor.unwrap_or(pos);
                                        t.elapsed().as_millis() < 400
                                            && (cur.0 - pos.0).abs() < 8.0
                                            && (cur.1 - pos.1).abs() < 8.0
                                    } else {
                                        false
                                    };

                                    let handled_as_double_click = if is_double_click {
                                        let should_commit = state.editor.sketch.as_ref().map_or(false, |sk| {
                                            sk.active_tool == editor::DrawTool::Polyline
                                                && (
                                                    // Legacy active-profile polyline.
                                                    (sk.tool_in_progress.is_none() && !sk.points.is_empty())
                                                    // Per-segment chain with at least one segment.
                                                    || matches!(
                                                        &sk.tool_in_progress,
                                                        Some(editor::ToolInProgress::PolylineChain { segment_count, .. })
                                                            if *segment_count > 0
                                                    )
                                                )
                                        });
                                        if should_commit {
                                            state.editor.apply(UiAction::SketchCommitPolyline);
                                            state.last_click_time = None;
                                            state.last_click_pos = None;
                                            true
                                        } else {
                                            false
                                        }
                                    } else {
                                        false
                                    };

                                    if !handled_as_double_click {
                                        if let Some(_cur) = state.last_cursor {
                                            let w = state.surface_config.width;
                                            let h = state.surface_config.height;
                                            if state.editor.sketch.is_some() {
                                                let tool_busy = state.editor.sketch.as_ref()
                                                    .map_or(false, |sk| sk.tool_in_progress.is_some());
                                                if tool_busy {
                                                    // Multi-step tool: all clicks add a point.
                                                    // Snap to a hovered vertex if available.
                                                    let pt = state.snap_vertex
                                                        .and_then(|vi| state.editor.sketch.as_ref()
                                                            .and_then(|sk| sk.points.get(vi).copied()))
                                                        .or(state.sketch_cursor);
                                                    if let Some(p) = pt {
                                                        state.editor.apply(UiAction::SketchAddPoint(p));
                                                    }
                                                } else if let Some(vi) = state.snap_vertex {
                                                    // Clicking vertex 0 with ≥3 pts closes the polyline loop.
                                                    let can_close = state.editor.sketch.as_ref()
                                                        .map_or(false, |sk| !sk.closed && sk.points.len() >= 3);
                                                    if vi == 0 && can_close {
                                                        state.editor.apply(UiAction::SketchCloseLoop);
                                                    } else {
                                                        state.editor.apply(UiAction::SketchSelectVertex(vi));
                                                    }
                                                } else if let Some((snap_pi, snap_vi)) = state.snap_committed {
                                                    // Clicking the chain's first vertex while drawing closes the loop.
                                                    let can_close_chain = state.editor.sketch.as_ref()
                                                        .map_or(false, |sk| {
                                                            if let Some(editor::ToolInProgress::PolylineChain {
                                                                chain_start_profile,
                                                                segment_count,
                                                                ..
                                                            }) = &sk.tool_in_progress {
                                                                *segment_count >= 2
                                                                    && snap_pi == *chain_start_profile
                                                                    && snap_vi == 0
                                                            } else {
                                                                false
                                                            }
                                                        });
                                                    if can_close_chain {
                                                        state.editor.apply(UiAction::SketchCloseLoop);
                                                    } else {
                                                        let is_pointer = state.editor.sketch.as_ref()
                                                            .map_or(false, |sk| sk.active_tool == editor::DrawTool::Pointer);
                                                        if is_pointer {
                                                            let shape = state.editor.sketch.as_ref()
                                                                .and_then(|sk| sk.committed_profiles.get(snap_pi))
                                                                .map(|cp| cp.shape.clone());
                                                            if shape.as_ref().map_or(false, |s| s.vertex_selectable(snap_vi)) {
                                                                state.editor.apply(UiAction::SketchSelectCommittedPoint(Some((snap_pi, snap_vi))));
                                                            } else {
                                                                state.editor.apply(UiAction::SketchSelectCommitted(Some(snap_pi)));
                                                            }
                                                        } else {
                                                            // Snap to committed vertex as next drawing point.
                                                            let pt = state.editor.sketch.as_ref()
                                                                .and_then(|sk| {
                                                                    let cp = sk.committed_profiles.get(snap_pi)?;
                                                                    let gi = cp.point_indices.get(snap_vi)?;
                                                                    sk.global_points.get(*gi).copied()
                                                                });
                                                            if let Some(p) = pt {
                                                                state.editor.apply(UiAction::SketchAddPoint(p));
                                                            }
                                                        }
                                                    }
                                                } else if let Some(r) = state.snap_ref {
                                                    state.editor.apply(UiAction::SketchSelectRef(r));
                                                } else if let Some(ci) = state.snap_constraint {
                                                    state.editor.apply(UiAction::SketchSelectConstraint(ci));
                                                } else if let Some(seg) = state.snap_segment {
                                                    state.editor.apply(UiAction::SketchSelectSegment(seg));
                                                } else if let Some((pi, si)) = state.snap_committed_seg {
                                                    // Pointer + click on a committed polyline segment.
                                                    let is_pointer = state.editor.sketch.as_ref()
                                                        .map_or(false, |sk| sk.active_tool == editor::DrawTool::Pointer);
                                                    if is_pointer {
                                                        state.editor.apply(UiAction::SketchSelectCommittedSeg(Some((pi, si))));
                                                    }
                                                } else {
                                                    let is_pointer = state.editor.sketch.as_ref()
                                                        .map_or(false, |sk| sk.active_tool == editor::DrawTool::Pointer);
                                                    if is_pointer {
                                                        if let Some(pi) = state.snap_committed_curve {
                                                            // Pointer + click on circle/arc curve → select it.
                                                            state.editor.apply(UiAction::SketchSelectCommitted(Some(pi)));
                                                        } else {
                                                            // Pointer tool: clicking empty space clears committed selection.
                                                            state.editor.apply(UiAction::SketchSelectCommitted(None));
                                                        }
                                                    } else if let Some(p) = state.sketch_cursor {
                                                        state.editor.apply(UiAction::SketchAddPoint(p));
                                                    }
                                                }
                                            } else {
                                                let cur = _cur;
                                                let shift = state.egui_ctx.input(|i| i.modifiers.shift || i.modifiers.ctrl);
                                                if let Some(idx) = state.editor.pick_at_screen(cur.0, cur.1, w, h) {
                                                    if shift {
                                                        state.editor.apply(UiAction::ToggleSelectObject(idx));
                                                    } else {
                                                        state.editor.apply(UiAction::SelectObject(idx));
                                                    }
                                                    state.editor.scene_dirty = true;
                                                } else if !shift {
                                                    state.editor.apply(UiAction::ClearSelection);
                                                    state.editor.scene_dirty = true;
                                                }
                                            }
                                        }
                                        state.last_click_time = Some(std::time::Instant::now());
                                        state.last_click_pos = state.last_cursor;
                                    }
                                }
                            }
                            state.drag_committed = None;
                            state.drag_committed_curve = None;
                            state.mouse_pressed = None;
                            state.last_cursor = None;
                            state.mouse_press_origin = None;
                        }
                    }
                } else {
                    state.drag_committed = None;
                    state.drag_committed_curve = None;
                    state.mouse_pressed = None;
                    state.last_cursor = None;
                    state.mouse_press_origin = None;
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                let cur = (position.x as f32, position.y as f32);

                // Orbit/pan only when not dragging a sketch vertex or committed vertex.
                if state.drag_vertex.is_none() && state.drag_committed.is_none() && state.drag_committed_curve.is_none() {
                    if let (Some(btn), Some(last)) = (state.mouse_pressed, state.last_cursor) {
                        let dx = cur.0 - last.0;
                        let dy = cur.1 - last.1;
                        let shift = state.egui_ctx.input(|i| i.modifiers.shift);
                        match btn {
                            MouseButton::Left if shift => state.editor.camera.pan(dx, dy),
                            MouseButton::Left => state.editor.camera.orbit(dx, dy),
                            MouseButton::Right | MouseButton::Middle => {
                                state.editor.camera.pan(dx, dy)
                            }
                            _ => {}
                        }
                    }
                }
                state.last_cursor = Some(cur);

                // Update sketch cursor: project mouse ray onto the sketch plane.
                if state.editor.sketch.is_some() {
                    let w = state.surface_config.width;
                    let h = state.surface_config.height;
                    let (plane_origin, plane_normal) = {
                        let sk = state.editor.sketch.as_ref().unwrap();
                        (sk.plane.origin(), sk.plane.normal())
                    };
                    let ray = state.editor.camera.unproject_ray(cur.0, cur.1, w, h);
                    state.sketch_cursor = ray_plane_intersect(&ray, &plane_origin, &plane_normal);

                    let has_moved = match state.mouse_press_origin {
                        Some(origin) => {
                            let dx = cur.0 - origin.0;
                            let dy = cur.1 - origin.1;
                            (dx * dx + dy * dy).sqrt() > 2.0
                        }
                        None => false,
                    };

                    // Save drag history on the first actual movement (not on press, to avoid
                    // recording a "Move point" entry for plain clicks / selections).
                    let is_dragging = state.drag_vertex.is_some()
                        || state.drag_committed.is_some()
                        || state.drag_committed_curve.is_some();
                    if is_dragging && has_moved && !state.drag_history_saved {
                        state.editor.apply(UiAction::SketchSaveDragHistory);
                        state.drag_history_saved = true;
                    }

                    // If dragging a vertex, apply position update immediately.
                    if let Some(drag_vi) = state.drag_vertex {
                        if state.mouse_pressed == Some(MouseButton::Left) && has_moved {
                            if let Some(cursor_pt) = state.sketch_cursor {
                                state.editor.drag_sketch_vertex(drag_vi, cursor_pt);
                            }
                        }
                    }

                    // Drag committed profile vertex if one is being held.
                    if let Some((pi, vi)) = state.drag_committed {
                        if state.mouse_pressed == Some(MouseButton::Left) && has_moved {
                            if let Some(cursor_pt) = state.sketch_cursor {
                                if let Some(sk) = &mut state.editor.sketch {
                                    if let Some(cp) = sk.committed_profiles.get(pi).cloned() {
                                        let plane = cp.plane;
                                        let mut pts: Vec<_> = cp.point_indices.iter()
                                            .map(|&gi| sk.global_points[gi])
                                            .collect();
                                        cp.shape.apply_vertex_drag(&mut pts, vi, cursor_pt, plane);
                                        for (&gi, p) in cp.point_indices.iter().zip(pts.iter()) {
                                            sk.global_points[gi] = *p;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Drag committed curve: for circles move boundary point (vi=1) = resize.
                    if let Some(pi) = state.drag_committed_curve {
                        if state.mouse_pressed == Some(MouseButton::Left) && has_moved {
                            if let Some(cursor_pt) = state.sketch_cursor {
                                if let Some(sk) = &mut state.editor.sketch {
                                    if let Some(cp) = sk.committed_profiles.get(pi).cloned() {
                                        let plane = cp.plane;
                                        let mut pts: Vec<_> = cp.point_indices.iter()
                                            .map(|&gi| sk.global_points[gi])
                                            .collect();
                                        cp.shape.apply_curve_drag(&mut pts, cursor_pt, plane);
                                        for (&gi, p) in cp.point_indices.iter().zip(pts.iter()) {
                                            sk.global_points[gi] = *p;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Snap to nearest vertex (works for both open and closed sketches).
                    const SNAP_PX: f32 = 15.0;
                    let mut snapped = None;
                    let sk = state.editor.sketch.as_ref().unwrap();
                    for (i, &pt) in sk.points.iter().enumerate() {
                        if let Some((px, py)) = state.editor.camera.project_to_screen(pt, w, h) {
                            let dx = px - cur.0;
                            let dy = py - cur.1;
                            if (dx * dx + dy * dy).sqrt() < SNAP_PX {
                                snapped = Some(i);
                                break;
                            }
                        }
                    }
                    state.snap_vertex = snapped;

                    // Snap to committed profile vertices (for pointer tool drag / constraint target).
                    let mut snap_comm = None;
                    'outer: for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                        for (vi, &gi) in cp.point_indices.iter().enumerate() {
                            let pt = sk.global_points[gi];
                            if let Some((px, py)) = state.editor.camera.project_to_screen(pt, w, h) {
                                let dx = px - cur.0;
                                let dy = py - cur.1;
                                if (dx * dx + dy * dy).sqrt() < SNAP_PX {
                                    snap_comm = Some((pi, vi));
                                    break 'outer;
                                }
                            }
                        }
                    }
                    state.snap_committed = snap_comm;

                    // Snap to committed circle/arc curves (hover the curve itself, not just control points).
                    const CURVE_SNAP_PX: f32 = 8.0;
                    let mut snap_curve: Option<usize> = None;
                    if snap_comm.is_none() {
                        for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                            let pts: Vec<_> = cp.point_indices.iter().map(|&gi| sk.global_points[gi]).collect();
                            if cp.shape.hit_test_curve(
                                &pts, cp.plane, cur, CURVE_SNAP_PX,
                                &|pt| state.editor.camera.project_to_screen(pt, w, h),
                            ) {
                                snap_curve = Some(pi);
                                break;
                            }
                        }
                    }
                    state.snap_committed_curve = snap_curve;

                    // Snap to committed polyline segments (only when no vertex or curve snap).
                    let mut snap_seg: Option<(usize, usize)> = None;
                    if snap_comm.is_none() && snap_curve.is_none() {
                        for (pi, cp) in sk.committed_profiles.iter().enumerate() {
                            let pts: Vec<_> = cp.point_indices.iter().map(|&gi| sk.global_points[gi]).collect();
                            if let Some(si) = cp.shape.hit_test_segment(
                                &pts, cp.closed, cur, CURVE_SNAP_PX,
                                &|pt| state.editor.camera.project_to_screen(pt, w, h),
                            ) {
                                snap_seg = Some((pi, si));
                                break;
                            }
                        }
                    }
                    state.snap_committed_seg = snap_seg;

                    // Reference entity snap (origin / axes) — only when not snapping to a vertex.
                    if state.snap_vertex.is_none() {
                        const REF_SNAP_PX: f32 = 12.0;
                        let origin_world = sk.plane.origin();
                        let (u_axis, v_axis) = sk.plane.uv_axes();
                        let far = 500.0_f64;

                        // Check origin first.
                        let sr = state.editor.camera.project_to_screen(origin_world, w, h)
                            .and_then(|(ox, oy)| {
                                let dx = ox - cur.0;
                                let dy = oy - cur.1;
                                if (dx * dx + dy * dy).sqrt() < REF_SNAP_PX {
                                    Some(editor::RefEntity::Origin)
                                } else { None }
                            });

                        // Check X-axis.
                        let sr = sr.or_else(|| {
                            let pa = state.editor.camera.project_to_screen(origin_world + u_axis * far, w, h);
                            let pb = state.editor.camera.project_to_screen(origin_world - u_axis * far, w, h);
                            if let (Some((ax, ay)), Some((bx, by))) = (pa, pb) {
                                if point_to_segment_dist(cur.0, cur.1, ax, ay, bx, by) < REF_SNAP_PX {
                                    return Some(editor::RefEntity::XAxis);
                                }
                            }
                            None
                        });

                        // Check Y-axis.
                        let sr = sr.or_else(|| {
                            let pa = state.editor.camera.project_to_screen(origin_world + v_axis * far, w, h);
                            let pb = state.editor.camera.project_to_screen(origin_world - v_axis * far, w, h);
                            if let (Some((ax, ay)), Some((bx, by))) = (pa, pb) {
                                if point_to_segment_dist(cur.0, cur.1, ax, ay, bx, by) < REF_SNAP_PX {
                                    return Some(editor::RefEntity::YAxis);
                                }
                            }
                            None
                        });

                        state.snap_ref = sr;
                    } else {
                        state.snap_ref = None;
                    }

                    // Segment hover — only when not hovering a vertex or reference entity.
                    if state.snap_vertex.is_none() && state.snap_ref.is_none() {
                        const SEG_SNAP_PX: f32 = 10.0;
                        let n = sk.points.len();
                        let seg_count = if sk.closed { n } else { n.saturating_sub(1) };
                        let mut nearest_seg = None;
                        for i in 0..seg_count {
                            let pa = state.editor.camera.project_to_screen(sk.points[i], w, h);
                            let pb = state.editor.camera.project_to_screen(sk.points[(i + 1) % n], w, h);
                            if let (Some((ax, ay)), Some((bx, by))) = (pa, pb) {
                                if point_to_segment_dist(cur.0, cur.1, ax, ay, bx, by) < SEG_SNAP_PX {
                                    nearest_seg = Some(i);
                                    break;
                                }
                            }
                        }
                        state.snap_segment = nearest_seg;
                    } else {
                        state.snap_segment = None;
                    }

                    // Constraint marker hover — only when not snapping to vertex/ref/segment.
                    if state.snap_vertex.is_none() && state.snap_ref.is_none() && state.snap_segment.is_none() {
                        const CONSTRAINT_SNAP_PX: f32 = 14.0;
                        let n = sk.points.len();
                        let mut snapped_c = None;
                        for (i, c) in sk.constraints.iter().enumerate() {
                            if let Some((mx, my)) = constraint_marker_screen_pos(
                                c, &sk.points, n, &state.editor.camera, w, h,
                            ) {
                                let dx = cur.0 - mx;
                                let dy = cur.1 - my;
                                if (dx * dx + dy * dy).sqrt() < CONSTRAINT_SNAP_PX {
                                    snapped_c = Some(i);
                                    break;
                                }
                            }
                        }
                        state.snap_constraint = snapped_c;
                    } else {
                        state.snap_constraint = None;
                    }
                } else {
                    state.sketch_cursor = None;
                    state.snap_vertex = None;
                    state.snap_ref = None;
                    state.snap_segment = None;
                    state.snap_constraint = None;
                }
            }

            WindowEvent::MouseWheel { delta, .. } => {
                if !egui_consumed {
                    let scroll = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(pos) => pos.y as f32 * 0.02,
                    };
                    let cursor_ray = state.last_cursor.map(|(cx, cy)| {
                        let w = state.surface_config.width;
                        let h = state.surface_config.height;
                        state.editor.camera.unproject_ray(cx, cy, w, h)
                    });
                    state.editor.camera.zoom(scroll, cursor_ray.as_ref());
                }
            }

            WindowEvent::Resized(new_size) => {
                state.resize(new_size);
                if let Ok(pos) = state.window.outer_position() {
                    let _ = self.config.set("window.geometry", &WindowGeometry {
                        x: pos.x, y: pos.y,
                        width: new_size.width, height: new_size.height,
                    });
                }
            }

            WindowEvent::Moved(pos) => {
                let size = state.window.inner_size();
                let _ = self.config.set("window.geometry", &WindowGeometry {
                    x: pos.x, y: pos.y,
                    width: size.width, height: size.height,
                });
            }

            WindowEvent::RedrawRequested => {
                match state.render() {
                    Ok(()) => {}
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        let size = state.window.inner_size();
                        state.resize(size);
                    }
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    Err(e) => eprintln!("render error: {e:?}"),
                }
                state.window.request_redraw();
            }

            _ => {}
        }
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() {
    let config = ConfigStore::open().expect("failed to open config database");
    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    let mut app = App { state: None, config };
    event_loop.run_app(&mut app).unwrap();
}
