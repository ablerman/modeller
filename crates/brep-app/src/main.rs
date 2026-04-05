//! BRep Modeller — interactive solid-modelling application.
//!
//! Architecture:
//!   winit event loop → wgpu 3-D renderer + egui overlay → BRep kernel

mod editor;
mod ui;

use std::sync::Arc;

use brep_algo::bvh::Ray;
use brep_core::{Point3, Vec3};
use brep_render::gpu::GpuRenderer;
use editor::{EditorState, UiAction};
use egui_wgpu::ScreenDescriptor;
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

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
    /// When Some((seg_a, seg_b)), the angle-input modal dialog is open.
    angle_dialog: Option<(usize, usize)>,
    /// When Some((target, initial_len)), the length-input modal dialog is open.
    /// `initial_len` is the measured length/distance at the moment the dialog was opened.
    length_dialog: Option<(editor::LengthTarget, f64)>,
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
            mouse_pressed: None,
            last_cursor: None,
            mouse_press_origin: None,
            sketch_cursor: None,
            snap_vertex: None,
            snap_segment: None,
            angle_dialog: None,
            length_dialog: None,
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

        // Evict cache entries that no longer correspond to any live object.
        let live_ids: std::collections::HashSet<u64> =
            self.editor.objects.iter().map(|o| o.id).collect();
        self.tess_cache.retain(|id, _| live_ids.contains(id));

        self.gpu_meshes.clear();
        for (i, obj) in self.editor.objects.iter().enumerate() {
            let selected = self.editor.selection.contains(&i);
            let base_color = if selected {
                [0.90, 0.70, 0.20f32]
            } else {
                [0.38, 0.62, 0.90f32]
            };
            // Use cached tessellation if available; otherwise compute and cache.
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
        let angle_dialog = self.angle_dialog;
        let length_dialog = self.length_dialog;  // Option<(LengthTarget, f64)>
        let vp = (self.surface_config.width, self.surface_config.height);
        let full_output = egui_ctx.run(raw_input, |ctx| {
            actions = ui::build_ui(ctx, &self.editor, vp, sketch_cursor, snap_vertex, snap_segment, angle_dialog, length_dialog);
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
            // Intercept app-level dialog actions before passing to the editor.
            match &action {
                UiAction::SketchBeginAngleInput { seg_a, seg_b } => {
                    self.angle_dialog = Some((*seg_a, *seg_b));
                    continue;
                }
                UiAction::SketchCancelAngleInput => {
                    self.angle_dialog = None;
                    continue;
                }
                UiAction::SketchBeginLengthInput(target) => {
                    let initial = current_length(*target, &self.editor.sketch);
                    self.length_dialog = Some((*target, initial));
                    continue;
                }
                UiAction::SketchCancelLengthInput => {
                    self.length_dialog = None;
                    continue;
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
            let changed = self.editor.apply(action);
            needs_mesh_update |= changed;
        }
        if needs_mesh_update {
            self.sync_meshes();
        }

        Ok(())
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

// ── winit ApplicationHandler ──────────────────────────────────────────────────

struct App {
    state: Option<AppState>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let attrs = Window::default_attributes()
            .with_title("BRep Modeller")
            .with_maximized(true);
        let window = Arc::new(event_loop.create_window(attrs).unwrap());
        let state = pollster::block_on(AppState::new(window));
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
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::KeyboardInput { event: key_event, .. } => {
                if !egui_consumed && key_event.state == ElementState::Pressed {
                    let modifiers =
                        state.egui_ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
                    if modifiers {
                        match key_event.physical_key {
                            PhysicalKey::Code(KeyCode::KeyZ) => {
                                state.editor.apply(UiAction::Undo);
                            }
                            PhysicalKey::Code(KeyCode::KeyY) => {
                                state.editor.apply(UiAction::Redo);
                            }
                            _ => {}
                        }
                    }
                    if matches!(
                        key_event.physical_key,
                        PhysicalKey::Code(KeyCode::Delete | KeyCode::Backspace)
                    ) {
                        state.editor.apply(UiAction::DeleteSelected);
                    }
                }
            }

            WindowEvent::MouseInput { state: btn_state, button, .. } => {
                if !egui_consumed {
                    match btn_state {
                        ElementState::Pressed => {
                            state.mouse_pressed = Some(button);
                            state.mouse_press_origin = state.last_cursor;
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
                                if is_click {
                                    if let Some(_cur) = state.last_cursor {
                                        let w = state.surface_config.width;
                                        let h = state.surface_config.height;
                                        if state.editor.sketch.is_some() {
                                            let pts_len = state.editor.sketch.as_ref()
                                                .map_or(0, |s| s.points.len());
                                            let is_closed = state.editor.sketch.as_ref()
                                                .map_or(false, |s| s.closed);
                                            if state.snap_vertex == Some(0) && pts_len >= 3 && !is_closed {
                                                // Click on first vertex → close loop.
                                                state.editor.apply(UiAction::SketchClose);
                                            } else if let Some(vi) = state.snap_vertex {
                                                // Click on any vertex (when closed, or after close) → select for constraints.
                                                if is_closed {
                                                    state.editor.apply(UiAction::SketchSelectVertex(vi));
                                                }
                                                // When not closed and vi != 0, do nothing (can't interact with interior verts yet).
                                            } else if let Some(seg) = state.snap_segment {
                                                // Click on a segment → select/deselect for constraints.
                                                state.editor.apply(UiAction::SketchSelectSegment(seg));
                                            } else if !is_closed {
                                                // Free click on the workplane → add point.
                                                if let Some(p) = state.sketch_cursor {
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
                                        } // end else (not in sketch mode)
                                    }
                                }
                            }
                            state.mouse_pressed = None;
                            state.last_cursor = None;
                            state.mouse_press_origin = None;
                        }
                    }
                } else {
                    state.mouse_pressed = None;
                    state.last_cursor = None;
                    state.mouse_press_origin = None;
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                let cur = (position.x as f32, position.y as f32);
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
                state.last_cursor = Some(cur);

                // Update sketch cursor: project mouse ray onto the sketch plane.
                if let Some(sk) = &state.editor.sketch {
                    let w = state.surface_config.width;
                    let h = state.surface_config.height;
                    let ray = state.editor.camera.unproject_ray(cur.0, cur.1, w, h);
                    state.sketch_cursor = ray_plane_intersect(&ray, &sk.plane.origin(), &sk.plane.normal());

                    // Detect snap to nearest existing vertex (screen-space threshold).
                    if !sk.closed {
                        const SNAP_PX: f32 = 15.0;
                        let mut snapped = None;
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
                    } else {
                        state.snap_vertex = None;
                    }

                    // Segment hover — only when not hovering a vertex.
                    if state.snap_vertex.is_none() {
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
                } else {
                    state.sketch_cursor = None;
                    state.snap_vertex = None;
                    state.snap_segment = None;
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
    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    let mut app = App { state: None };
    event_loop.run_app(&mut app).unwrap();
}
