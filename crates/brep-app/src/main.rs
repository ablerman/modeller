//! BRep Modeller — interactive solid-modelling application.
//!
//! Architecture:
//!   winit event loop → wgpu 3-D renderer + egui overlay → BRep kernel

mod editor;
mod ui;

use std::sync::Arc;

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
            egui_ctx,
            egui_state,
            egui_renderer,
            editor,
            mouse_pressed: None,
            last_cursor: None,
            mouse_press_origin: None,
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
    fn sync_meshes(&mut self) {
        if !self.editor.scene_dirty {
            return;
        }
        self.editor.scene_dirty = false;
        self.gpu_meshes.clear();
        for (i, obj) in self.editor.objects.iter().enumerate() {
            let selected = self.editor.selection.contains(&i);
            let base_color = if selected {
                [0.90, 0.70, 0.20f32]
            } else {
                [0.38, 0.62, 0.90f32]
            };
            let mesh = self.gpu_renderer.upload_store(&self.device, &obj.store, base_color);
            self.gpu_meshes.push(GpuMeshEntry { mesh, selected });
        }
    }

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
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
        let full_output = egui_ctx.run(raw_input, |ctx| {
            actions = ui::build_ui(ctx, &self.editor);
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
            let changed = self.editor.apply(action);
            needs_mesh_update |= changed;
        }
        if needs_mesh_update {
            self.sync_meshes();
        }

        Ok(())
    }
}

// ── winit ApplicationHandler ──────────────────────────────────────────────────

struct App {
    state: Option<AppState>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let attrs = Window::default_attributes()
            .with_title("BRep Modeller")
            .with_inner_size(PhysicalSize::new(1280u32, 800u32));
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
                                    if let Some(cur) = state.last_cursor {
                                        let w = state.surface_config.width;
                                        let h = state.surface_config.height;
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
            }

            WindowEvent::MouseWheel { delta, .. } => {
                if !egui_consumed {
                    let scroll = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(pos) => pos.y as f32 * 0.02,
                    };
                    state.editor.camera.zoom(scroll);
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
