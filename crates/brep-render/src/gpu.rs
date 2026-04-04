//! GPU renderer using wgpu.
//!
//! Tessellates a [`ShapeStore`] and uploads the result to GPU vertex buffers.
//! Renders with a Blinn-Phong WGSL shader matching the CPU rasterizer's lighting.
//!
//! # Usage
//! ```no_run
//! // Inside a wgpu application:
//! let renderer = GpuRenderer::new(&device, &queue, surface_format, depth_format, width, height);
//! let meshes = renderer.upload_store(&device, &store, [0.38, 0.62, 0.90]);
//! // In each frame:
//! renderer.render(&mut pass, &meshes, eye, target, up, fov_y_deg, width, height);
//! ```

use brep_core::{Point3, Vec3};
use brep_mesh::{tessellate, TessellationOptions};
use brep_topo::store::ShapeStore;
use bytemuck::{Pod, Zeroable};
use nalgebra::Matrix4;
use wgpu::util::DeviceExt;

// ── Vertex layout ─────────────────────────────────────────────────────────────

/// A single vertex on the GPU: position, normal, and a per-object base colour.
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct GpuVertex {
    pub position: [f32; 3],
    pub _pad0: f32,
    pub normal: [f32; 3],
    pub _pad1: f32,
    pub color: [f32; 3],
    pub _pad2: f32,
}

impl GpuVertex {
    const ATTRIBS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![
        0 => Float32x3,  // position  (uses _pad0 internally — wgpu ignores the pad)
        1 => Float32x3,  // normal
        2 => Float32x3,  // color
    ];

    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GpuVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

// ── Camera uniform ────────────────────────────────────────────────────────────

/// Camera data uploaded as a uniform buffer (group 0, binding 0).
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct CameraUniform {
    /// Combined model-view-projection matrix (column-major, f32).
    pub mvp: [[f32; 4]; 4],
    /// Normalised direction from model toward the camera eye.
    pub view_dir: [f32; 3],
    pub _pad: f32,
}

impl CameraUniform {
    pub fn from_camera(
        eye: Point3,
        target: Point3,
        up: Vec3,
        fov_y_deg: f64,
        width: u32,
        height: u32,
    ) -> Self {
        let view = look_at_mat(&eye, &target, &up);
        let proj = perspective_mat(
            fov_y_deg.to_radians(),
            width as f64 / height as f64,
            0.05,
            1000.0,
        );
        let mvp64: Matrix4<f64> = proj * view;
        let mvp32: [[f32; 4]; 4] = matrix4_to_f32(mvp64);
        let dir = (eye - target).normalize();
        CameraUniform {
            mvp: mvp32,
            view_dir: [dir.x as f32, dir.y as f32, dir.z as f32],
            _pad: 0.0,
        }
    }
}

// ── Uploaded mesh ─────────────────────────────────────────────────────────────

/// A tessellated solid uploaded to a wgpu vertex buffer.
pub struct GpuMesh {
    pub vertex_buf: wgpu::Buffer,
    pub vertex_count: u32,
}

// ── GpuRenderer ──────────────────────────────────────────────────────────────

/// Holds the wgpu render pipeline and camera bind group for 3-D solid rendering.
pub struct GpuRenderer {
    pipeline: wgpu::RenderPipeline,
    camera_buf: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    depth_texture: wgpu::Texture,
    pub depth_view: wgpu::TextureView,
    pub surface_format: wgpu::TextureFormat,
    width: u32,
    height: u32,
}

impl GpuRenderer {
    /// Create the render pipeline.  Call once after wgpu device creation.
    pub fn new(
        device: &wgpu::Device,
        surface_format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        width: u32,
        height: u32,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("brep shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        // Camera uniform bind group layout.
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        // Allocate a zeroed camera buffer.
        let camera_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("camera buf"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera bg"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buf.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("brep pipeline layout"),
            bind_group_layouts: &[&bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("brep pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[GpuVertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: depth_format,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let (depth_texture, depth_view) = make_depth_texture(device, depth_format, width, height);

        Self {
            pipeline,
            camera_buf,
            camera_bind_group,
            depth_texture,
            depth_view,
            surface_format,
            width,
            height,
        }
    }

    /// Tessellate `store` and upload all triangles to a GPU vertex buffer.
    ///
    /// `base_color` is the diffuse RGB colour (linear, 0–1) for all faces.
    pub fn upload_store(
        &self,
        device: &wgpu::Device,
        store: &ShapeStore,
        base_color: [f32; 3],
    ) -> GpuMesh {
        let opts = TessellationOptions {
            chord_tolerance: 0.005,
            min_segments: 16,
        };
        let face_meshes = tessellate(store, &opts).unwrap_or_default();

        // ── Collect flat triangle soup ────────────────────────────────────────
        struct RawVert { pos: [f32; 3], flat_n: [f32; 3] }
        let mut raw: Vec<RawVert> = Vec::new();
        for fm in &face_meshes {
            for tri in &fm.triangles {
                let n = tri.normal;
                let nf = [n.x as f32, n.y as f32, n.z as f32];
                for i in 0..3 {
                    let p = tri.positions[i];
                    raw.push(RawVert {
                        pos: [p.x as f32, p.y as f32, p.z as f32],
                        flat_n: nf,
                    });
                }
            }
        }

        // ── Compute smooth per-vertex normals with crease-angle ───────────────
        // cos(40°) ≈ 0.766 — hard edges sharper than 40° stay crisp.
        const CREASE_COS: f32 = 0.766;
        // Grid cell size: coarser than the weld distance so neighbours always
        // land in at most 2^3 = 8 adjacent cells.
        const CELL: f32 = 0.01;

        // Build a spatial hash: cell key → list of vertex indices.
        let cell_key = |pos: &[f32; 3]| -> (i32, i32, i32) {
            (
                (pos[0] / CELL).floor() as i32,
                (pos[1] / CELL).floor() as i32,
                (pos[2] / CELL).floor() as i32,
            )
        };
        let mut grid: std::collections::HashMap<(i32,i32,i32), Vec<usize>> =
            std::collections::HashMap::with_capacity(raw.len());
        for (i, rv) in raw.iter().enumerate() {
            grid.entry(cell_key(&rv.pos)).or_default().push(i);
        }

        let smooth_normals: Vec<[f32; 3]> = raw
            .iter()
            .map(|v| {
                let (cx, cy, cz) = cell_key(&v.pos);
                let mut sum = v.flat_n;
                // Check the 3×3×3 neighbourhood of cells.
                for dx in -1i32..=1 {
                for dy in -1i32..=1 {
                for dz in -1i32..=1 {
                    let Some(bucket) = grid.get(&(cx+dx, cy+dy, cz+dz)) else { continue };
                    for &j in bucket {
                        let other = &raw[j];
                        // Same position?
                        let ex = v.pos[0] - other.pos[0];
                        let ey = v.pos[1] - other.pos[1];
                        let ez = v.pos[2] - other.pos[2];
                        if ex*ex + ey*ey + ez*ez > CELL * CELL * 0.01 { continue; }
                        // Within crease angle?
                        let dot = v.flat_n[0]*other.flat_n[0]
                                + v.flat_n[1]*other.flat_n[1]
                                + v.flat_n[2]*other.flat_n[2];
                        if dot < CREASE_COS { continue; }
                        sum[0] += other.flat_n[0];
                        sum[1] += other.flat_n[1];
                        sum[2] += other.flat_n[2];
                    }
                }}}
                let len = (sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2]).sqrt().max(1e-12);
                [sum[0]/len, sum[1]/len, sum[2]/len]
            })
            .collect();

        let mut verts: Vec<GpuVertex> = Vec::with_capacity(raw.len());
        for (rv, sn) in raw.iter().zip(smooth_normals.iter()) {
            verts.push(GpuVertex {
                position: rv.pos,
                _pad0: 0.0,
                normal: *sn,
                _pad1: 0.0,
                color: base_color,
                _pad2: 0.0,
            });
        }

        let vertex_count = verts.len() as u32;
        // Create a non-empty buffer even if there are no vertices (wgpu requires non-zero size).
        let data: &[u8] = if verts.is_empty() {
            &[0u8; std::mem::size_of::<GpuVertex>()]
        } else {
            bytemuck::cast_slice(&verts)
        };
        let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vbo"),
            contents: data,
            usage: wgpu::BufferUsages::VERTEX,
        });
        GpuMesh { vertex_buf, vertex_count }
    }

    /// Resize the depth texture after a window resize.
    pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) {
        self.width = width;
        self.height = height;
        let depth_format = self.depth_texture.format();
        let (t, v) = make_depth_texture(device, depth_format, width, height);
        self.depth_texture = t;
        self.depth_view = v;
    }

    /// Upload camera data to the uniform buffer, then draw all meshes.
    pub fn render(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        meshes: &[&GpuMesh],
        queue: &wgpu::Queue,
        eye: Point3,
        target: Point3,
        up: Vec3,
        fov_y_deg: f64,
    ) {
        let uniform = CameraUniform::from_camera(
            eye, target, up, fov_y_deg, self.width, self.height,
        );
        queue.write_buffer(&self.camera_buf, 0, bytemuck::bytes_of(&uniform));

        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.camera_bind_group, &[]);
        for mesh in meshes {
            if mesh.vertex_count == 0 {
                continue;
            }
            pass.set_vertex_buffer(0, mesh.vertex_buf.slice(..));
            pass.draw(0..mesh.vertex_count, 0..1);
        }
    }
}

// ── Depth texture helper ──────────────────────────────────────────────────────

fn make_depth_texture(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("depth texture"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

// ── Camera math ───────────────────────────────────────────────────────────────

fn look_at_mat(eye: &Point3, target: &Point3, up: &Vec3) -> Matrix4<f64> {
    let f = (target - eye).normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(&f);
    Matrix4::new(
         s.x,  s.y,  s.z, -s.dot(&eye.coords),
         u.x,  u.y,  u.z, -u.dot(&eye.coords),
        -f.x, -f.y, -f.z,  f.dot(&eye.coords),
         0.0,  0.0,  0.0,  1.0,
    )
}

fn perspective_mat(fov_y_rad: f64, aspect: f64, near: f64, far: f64) -> Matrix4<f64> {
    let f = 1.0 / (fov_y_rad * 0.5).tan();
    // wgpu uses NDC z in [0, 1] (Vulkan convention), not [-1, 1] (OpenGL).
    Matrix4::new(
        f / aspect, 0.0,  0.0,                      0.0,
        0.0,        f,    0.0,                      0.0,
        0.0,        0.0,  far / (near - far),        (far * near) / (near - far),
        0.0,        0.0, -1.0,                       0.0,
    )
}

fn matrix4_to_f32(m: Matrix4<f64>) -> [[f32; 4]; 4] {
    // nalgebra uses column-major storage; WGSL mat4x4 is also column-major.
    let c = m.as_slice();
    [
        [c[0] as f32, c[1] as f32, c[2] as f32, c[3] as f32],
        [c[4] as f32, c[5] as f32, c[6] as f32, c[7] as f32],
        [c[8] as f32, c[9] as f32, c[10] as f32, c[11] as f32],
        [c[12] as f32, c[13] as f32, c[14] as f32, c[15] as f32],
    ]
}

// ── WGSL shader ───────────────────────────────────────────────────────────────

const SHADER: &str = r#"
struct Camera {
    mvp: mat4x4<f32>,
    view_dir: vec3<f32>,
    _pad: f32,
}

@group(0) @binding(0) var<uniform> cam: Camera;

struct Vin {
    @location(0) pos:   vec3<f32>,
    @location(1) norm:  vec3<f32>,
    @location(2) color: vec3<f32>,
}

struct Vout {
    @builtin(position) clip:  vec4<f32>,
    @location(0)       norm:  vec3<f32>,
    @location(1)       color: vec3<f32>,
}

@vertex
fn vs_main(v: Vin) -> Vout {
    var out: Vout;
    out.clip  = cam.mvp * vec4<f32>(v.pos, 1.0);
    out.norm  = v.norm;
    out.color = v.color;
    return out;
}

fn srgb(x: f32) -> f32 { return pow(clamp(x, 0.0, 1.0), 1.0 / 2.2); }

@fragment
fn fs_main(in: Vout) -> @location(0) vec4<f32> {
    let n        = normalize(in.norm);
    let v        = normalize(cam.view_dir);
    let ambient  = 0.12;
    let specular = 0.55;
    let shine    = 48.0;

    var r = ambient * in.color.r;
    var g = ambient * in.color.g;
    var b = ambient * in.color.b;

    // Key light — warm white from upper-right-front.
    let l1  = normalize(vec3<f32>(0.5222, 0.4352, 0.7830));
    let c1  = vec3<f32>(1.0, 0.97, 0.90);
    let d1  = max(dot(n, l1), 0.0);
    let h1  = normalize(l1 + v);
    let s1  = pow(max(dot(n, h1), 0.0), shine) * specular;
    r += d1 * c1.r * in.color.r + s1 * c1.r;
    g += d1 * c1.g * in.color.g + s1 * c1.g;
    b += d1 * c1.b * in.color.b + s1 * c1.b;

    // Fill light — cool from the left.
    let l2  = normalize(vec3<f32>(-0.8165, 0.3266, 0.4082));
    let c2  = vec3<f32>(0.35, 0.45, 0.65);
    let d2  = max(dot(n, l2), 0.0);
    let h2  = normalize(l2 + v);
    let s2  = pow(max(dot(n, h2), 0.0), shine) * specular;
    r += d2 * c2.r * in.color.r + s2 * c2.r;
    g += d2 * c2.g * in.color.g + s2 * c2.g;
    b += d2 * c2.b * in.color.b + s2 * c2.b;

    return vec4<f32>(srgb(r), srgb(g), srgb(b), 1.0);
}
"#;
