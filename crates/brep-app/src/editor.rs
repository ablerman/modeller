//! Editor state: scene objects, selection, camera, undo/redo.

use std::f64::consts::PI;

use brep_algo::bvh::{Bvh, Ray};
use brep_bool::{boolean_op, BooleanKind};
use brep_core::{Point3, Vec3};
use brep_topo::{
    primitives::{make_box, make_cone, make_cylinder, make_sphere},
    store::ShapeStore,
    entity::SolidId,
};

// ── Operation history tree ────────────────────────────────────────────────────

/// Records how an object was produced — forms a tree of operations.
#[derive(Clone, Debug)]
pub enum ObjectHistory {
    Primitive(PrimitiveKind),
    Boolean {
        kind: BooleanKind,
        left:  Box<ObjectHistory>,
        right: Box<ObjectHistory>,
    },
}

impl ObjectHistory {
    pub fn label(&self) -> &'static str {
        match self {
            ObjectHistory::Primitive(PrimitiveKind::Box)      => "Box",
            ObjectHistory::Primitive(PrimitiveKind::Cylinder) => "Cylinder",
            ObjectHistory::Primitive(PrimitiveKind::Sphere)   => "Sphere",
            ObjectHistory::Primitive(PrimitiveKind::Cone)     => "Cone",
            ObjectHistory::Boolean { kind: BooleanKind::Union,        .. } => "Union",
            ObjectHistory::Boolean { kind: BooleanKind::Difference,   .. } => "Difference",
            ObjectHistory::Boolean { kind: BooleanKind::Intersection, .. } => "Intersection",
        }
    }
}

// ── Scene object ──────────────────────────────────────────────────────────────

/// One logical solid in the scene.  Each object owns its own `ShapeStore`.
#[derive(Clone)]
pub struct SceneObject {
    pub store: ShapeStore,
    pub solid_id: SolidId,
    pub name: String,
    pub history: ObjectHistory,
}

// ── Camera ────────────────────────────────────────────────────────────────────

/// Orbit camera represented in spherical coordinates.
pub struct ViewportCamera {
    /// The point the camera orbits around.
    pub target: Point3,
    /// Distance from target to eye.
    pub distance: f64,
    /// Horizontal angle in radians (longitude).
    pub azimuth: f64,
    /// Vertical angle in radians (latitude, clamped away from poles).
    pub elevation: f64,
    /// Vertical field-of-view in degrees.
    pub fov_y_deg: f64,
}

impl Default for ViewportCamera {
    fn default() -> Self {
        Self {
            target: Point3::new(0.0, 0.0, 0.0),
            distance: 6.0,
            azimuth: 0.8,
            elevation: 0.5,
            fov_y_deg: 45.0,
        }
    }
}

impl ViewportCamera {
    /// Compute the eye position from spherical coordinates.
    pub fn eye(&self) -> Point3 {
        let x = self.distance * self.elevation.cos() * self.azimuth.cos();
        let y = self.distance * self.elevation.cos() * self.azimuth.sin();
        let z = self.distance * self.elevation.sin();
        self.target + Vec3::new(x, y, z)
    }

    /// Up vector (always world-Z unless near a pole).
    pub fn up(&self) -> Vec3 {
        Vec3::z()
    }

    /// Orbit by screen-space deltas (pixels).
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.azimuth -= dx as f64 * 0.005;
        self.elevation =
            (self.elevation + dy as f64 * 0.005).clamp(-PI / 2.0 + 0.05, PI / 2.0 - 0.05);
    }

    /// Pan by screen-space deltas (pixels).
    pub fn pan(&mut self, dx: f32, dy: f32) {
        let right = Vec3::new(-self.azimuth.sin(), self.azimuth.cos(), 0.0);
        let scale = self.distance * 0.001;
        self.target -= right * (dx as f64 * scale);
        self.target += Vec3::z() * (dy as f64 * scale);
    }

    /// Zoom by a scroll delta (positive = zoom in).
    pub fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta as f64 * 0.1)).clamp(0.1, 10_000.0);
    }

    /// Unproject a screen pixel into a world-space ray.
    ///
    /// `(screen_x, screen_y)` are pixel coordinates from the top-left.
    pub fn unproject_ray(&self, screen_x: f32, screen_y: f32, w: u32, h: u32) -> Ray {
        // NDC xy in [-1, 1].
        let ndc_x = 2.0 * screen_x as f64 / w as f64 - 1.0;
        let ndc_y = 1.0 - 2.0 * screen_y as f64 / h as f64;

        // Inverse of the view-projection matrix.
        use nalgebra::Matrix4;
        let eye = self.eye();
        let f = (self.target - eye).normalize();
        let right = f.cross(&Vec3::z()).normalize();
        let up_cam = right.cross(&f);

        let view = Matrix4::new(
             right.x,  right.y,  right.z, -right.dot(&eye.coords),
             up_cam.x, up_cam.y, up_cam.z, -up_cam.dot(&eye.coords),
            -f.x, -f.y, -f.z,  f.dot(&eye.coords),
             0.0,  0.0,  0.0,  1.0,
        );
        let aspect = w as f64 / h as f64;
        let fov = self.fov_y_deg.to_radians();
        let fc = 1.0 / (fov * 0.5).tan();
        let proj = Matrix4::new(
            fc / aspect, 0.0, 0.0, 0.0,
            0.0, fc, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, // simplified: not using far/near here, just direction
            0.0, 0.0, -1.0, 0.0,
        );
        let inv_vp = (proj * view).try_inverse().unwrap_or(Matrix4::identity());

        // Unproject point at near plane (w = 1 in clip space for eye at origin).
        let clip = nalgebra::Vector4::new(ndc_x, ndc_y, -1.0, 1.0);
        let world_h = inv_vp * clip;
        let world = Point3::new(world_h.x / world_h.w, world_h.y / world_h.w, world_h.z / world_h.w);
        let direction = (world - eye).normalize();

        Ray::new(eye, direction)
    }
}

// ── Undo/redo ─────────────────────────────────────────────────────────────────

/// A snapshot of the full scene, used for undo/redo.
type SceneSnapshot = Vec<SceneObject>;

/// Maintains an undo/redo stack of whole-scene snapshots.
pub struct History {
    /// Stack of past states; `undo_stack[last]` is the most recent saved state.
    undo_stack: Vec<SceneSnapshot>,
    /// Stack of states saved by undo, available for redo.
    redo_stack: Vec<SceneSnapshot>,
}

impl History {
    pub fn new() -> Self {
        Self {
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
        }
    }

    /// Save a snapshot before a mutating operation.
    pub fn push(&mut self, snapshot: SceneSnapshot) {
        self.undo_stack.push(snapshot);
        self.redo_stack.clear();
    }

    /// Restore the previous state and save the current one for redo.
    pub fn undo(&mut self, current: SceneSnapshot) -> Option<SceneSnapshot> {
        let prev = self.undo_stack.pop()?;
        self.redo_stack.push(current);
        Some(prev)
    }

    /// Re-apply the next state.
    pub fn redo(&mut self, current: SceneSnapshot) -> Option<SceneSnapshot> {
        let next = self.redo_stack.pop()?;
        self.undo_stack.push(current);
        Some(next)
    }

    pub fn can_undo(&self) -> bool { !self.undo_stack.is_empty() }
    pub fn can_redo(&self) -> bool { !self.redo_stack.is_empty() }
}

// ── Primitive kind ────────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq)]
pub enum PrimitiveKind {
    Box,
    Cylinder,
    Sphere,
    Cone,
}

// ── UI actions ────────────────────────────────────────────────────────────────

/// An action the UI requests the editor to perform.
pub enum UiAction {
    AddPrimitive(PrimitiveKind),
    DeleteSelected,
    BooleanOp(BooleanKind),
    SelectObject(usize),
    ToggleSelectObject(usize),
    ClearSelection,
    Undo,
    Redo,
}

// ── EditorState ───────────────────────────────────────────────────────────────

/// All mutable application state outside of rendering.
pub struct EditorState {
    pub objects: Vec<SceneObject>,
    pub selection: std::collections::HashSet<usize>,
    pub camera: ViewportCamera,
    pub history: History,
    /// Tracks how many objects have been created (for unique naming).
    name_counter: u32,
    /// Set to `true` whenever objects change, so meshes can be re-uploaded.
    pub scene_dirty: bool,
}

impl EditorState {
    pub fn new() -> Self {
        let mut s = Self {
            objects: Vec::new(),
            selection: std::collections::HashSet::new(),
            camera: ViewportCamera::default(),
            history: History::new(),
            name_counter: 0,
            scene_dirty: true,
        };
        // Start with a default box so the viewport isn't empty.
        s.add_primitive(PrimitiveKind::Box);
        s.history = History::new(); // clear the initial-add from history
        s
    }

    /// Apply a UI action.  Returns `true` if the scene geometry changed.
    pub fn apply(&mut self, action: UiAction) -> bool {
        match action {
            UiAction::AddPrimitive(kind) => {
                self.save_snapshot();
                self.add_primitive(kind);
                self.scene_dirty = true;
                true
            }
            UiAction::DeleteSelected => {
                if self.selection.is_empty() {
                    return false;
                }
                self.save_snapshot();
                let mut indices: Vec<usize> = self.selection.iter().cloned().collect();
                indices.sort_unstable_by(|a, b| b.cmp(a));
                for i in indices {
                    self.objects.remove(i);
                }
                self.selection.clear();
                self.scene_dirty = true;
                true
            }
            UiAction::BooleanOp(kind) => {
                if self.selection.len() != 2 {
                    return false;
                }
                let mut sel: Vec<usize> = self.selection.iter().cloned().collect();
                sel.sort_unstable();
                let (a_idx, b_idx) = (sel[0], sel[1]);
                self.save_snapshot();
                let result = boolean_op(
                    &self.objects[a_idx].store,
                    self.objects[a_idx].solid_id,
                    &self.objects[b_idx].store,
                    self.objects[b_idx].solid_id,
                    kind,
                    1e-7,
                );
                match result {
                    Ok(new_store) => {
                        let new_solid = new_store
                            .solid_ids()
                            .next()
                            .expect("boolean result has a solid");
                        let name = format!("Bool-{}", self.next_name());
                        // Remove in reverse order, capturing histories first.
                        let (hi, lo) = if a_idx > b_idx { (a_idx, b_idx) } else { (b_idx, a_idx) };
                        let hi_hist = self.objects[hi].history.clone();
                        let lo_hist = self.objects[lo].history.clone();
                        let (left_hist, right_hist) = if a_idx < b_idx {
                            (lo_hist, hi_hist)
                        } else {
                            (hi_hist, lo_hist)
                        };
                        self.objects.remove(hi);
                        self.objects.remove(lo);
                        let history = ObjectHistory::Boolean {
                            kind,
                            left:  Box::new(left_hist),
                            right: Box::new(right_hist),
                        };
                        self.objects.push(SceneObject {
                            store: new_store,
                            solid_id: new_solid,
                            name,
                            history,
                        });
                        self.selection.clear();
                        self.selection.insert(self.objects.len() - 1);
                        self.scene_dirty = true;
                        true
                    }
                    Err(_) => false,
                }
            }
            UiAction::SelectObject(i) => {
                self.selection.clear();
                self.selection.insert(i);
                false
            }
            UiAction::ToggleSelectObject(i) => {
                if self.selection.contains(&i) {
                    self.selection.remove(&i);
                } else {
                    self.selection.insert(i);
                }
                false
            }
            UiAction::ClearSelection => {
                self.selection.clear();
                false
            }
            UiAction::Undo => {
                if let Some(prev) = self.history.undo(self.objects.clone()) {
                    self.objects = prev;
                    self.selection.clear();
                    self.scene_dirty = true;
                    true
                } else {
                    false
                }
            }
            UiAction::Redo => {
                if let Some(next) = self.history.redo(self.objects.clone()) {
                    self.objects = next;
                    self.selection.clear();
                    self.scene_dirty = true;
                    true
                } else {
                    false
                }
            }
        }
    }

    fn save_snapshot(&mut self) {
        self.history.push(self.objects.clone());
    }

    fn next_name(&mut self) -> u32 {
        self.name_counter += 1;
        self.name_counter
    }

    fn add_primitive(&mut self, kind: PrimitiveKind) {
        let mut store = ShapeStore::new();
        let result = match kind {
            PrimitiveKind::Box      => make_box(&mut store, 1.0, 1.0, 1.0),
            PrimitiveKind::Cylinder => make_cylinder(&mut store, 0.5, 1.0),
            PrimitiveKind::Sphere   => make_sphere(&mut store, 0.5),
            PrimitiveKind::Cone     => make_cone(&mut store, 0.5, 1.0),
        };
        if let Ok(solid_id) = result {
            let name = format!("{:?}-{}", kind, self.next_name());
            let history = ObjectHistory::Primitive(kind);
            self.objects.push(SceneObject { store, solid_id, name, history });
        }
    }

    /// Cast a ray from the given screen pixel and return the index of the first
    /// object hit, or `None` if the ray misses everything.
    ///
    /// `screen_x` / `screen_y` are in pixels from the top-left.
    pub fn pick_at_screen(
        &self,
        screen_x: f32,
        screen_y: f32,
        viewport_w: u32,
        viewport_h: u32,
    ) -> Option<usize> {
        let ray = self.camera.unproject_ray(screen_x, screen_y, viewport_w, viewport_h);

        let mut best: Option<(usize, f64)> = None;
        for (i, obj) in self.objects.iter().enumerate() {
            let bvh = Bvh::build(&obj.store);
            let hits = bvh.intersect_ray(&ray);
            if !hits.is_empty() {
                // Estimate distance as distance from eye to the nearest face centroid.
                let eye = self.camera.eye();
                let d = hits
                    .iter()
                    .filter_map(|&fid| {
                        let verts = obj.store.face_vertices(fid).ok()?;
                        let n = verts.len() as f64;
                        if n == 0.0 { return None; }
                        let cx = verts.iter().filter_map(|&v| obj.store.vertex(v).ok()).map(|v| v.position.x).sum::<f64>() / n;
                        let cy = verts.iter().filter_map(|&v| obj.store.vertex(v).ok()).map(|v| v.position.y).sum::<f64>() / n;
                        let cz = verts.iter().filter_map(|&v| obj.store.vertex(v).ok()).map(|v| v.position.z).sum::<f64>() / n;
                        Some((Point3::new(cx, cy, cz) - eye).norm_squared())
                    })
                    .fold(f64::MAX, f64::min);
                if d < best.map_or(f64::MAX, |(_, d)| d) {
                    best = Some((i, d));
                }
            }
        }
        best.map(|(i, _)| i)
    }

    /// Return (face count, edge count, vertex count) for the selected object(s).
    pub fn selection_stats(&self) -> Option<(usize, usize, usize)> {
        let idx = self.selection.iter().next()?;
        let obj = self.objects.get(*idx)?;
        Some((
            obj.store.face_count(),
            obj.store.edge_count(),
            obj.store.vertex_count(),
        ))
    }
}
