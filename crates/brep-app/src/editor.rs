//! Editor state: scene objects, selection, camera, undo/redo.

use std::f64::consts::PI;
use std::time::Instant;

use brep_algo::bvh::{Bvh, Ray};
use brep_bool::{boolean_op, BooleanKind};
use brep_core::{Point3, Vec3};
use brep_sketch::{solve_constraints, SketchConstraint, SolveResult};
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
    Sketch { plane: SketchPlane },
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
            ObjectHistory::Sketch { .. }                      => "Sketch",
            ObjectHistory::Boolean { kind: BooleanKind::Union,        .. } => "Union",
            ObjectHistory::Boolean { kind: BooleanKind::Difference,   .. } => "Difference",
            ObjectHistory::Boolean { kind: BooleanKind::Intersection, .. } => "Intersection",
        }
    }
}

// ── Sketch ────────────────────────────────────────────────────────────────────

/// Which principal plane the sketch lives on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SketchPlane { XY, XZ, YZ }

impl SketchPlane {
    pub fn normal(self) -> Vec3 {
        match self { SketchPlane::XY => Vec3::z(), SketchPlane::XZ => Vec3::y(), SketchPlane::YZ => Vec3::x() }
    }
    pub fn origin(self) -> Point3 { Point3::origin() }

    /// The two in-plane unit axes `(U, V)` as world-space vectors.
    pub fn uv_axes(self) -> (Vec3, Vec3) {
        match self {
            SketchPlane::XY => (Vec3::x(), Vec3::y()),
            SketchPlane::XZ => (Vec3::x(), Vec3::z()),
            SketchPlane::YZ => (Vec3::y(), Vec3::z()),
        }
    }
}

/// What a length constraint applies to — used for the modal dialog.
#[derive(Clone, Copy, Debug)]
pub enum LengthTarget {
    /// Constrain a single segment to a fixed length.
    Segment(usize),
    /// Constrain the distance between two (possibly non-adjacent) vertices.
    Points(usize, usize),
}

/// Live in-progress 2D sketch being drawn by the user.
#[derive(Clone, Debug)]
pub struct SketchState {
    pub plane:                SketchPlane,
    pub points:               Vec<Point3>,  // world-space vertices on the plane (placed in order)
    pub closed:               bool,
    pub extrude_dist:         f64,
    /// Active geometric constraints on this sketch.
    pub constraints:          Vec<SketchConstraint>,
    /// Indices of currently selected segments (max 2).
    pub seg_selection:        Vec<usize>,
    /// Indices of currently selected vertices (max 2, for length/distance constraints).
    pub pt_selection:         Vec<usize>,
    /// Set when the last constraint solve did not converge.
    pub constraints_conflict: bool,
}

// ── Scene object ──────────────────────────────────────────────────────────────

/// One logical solid in the scene.  Each object owns its own `ShapeStore`.
#[derive(Clone)]
pub struct SceneObject {
    pub store: ShapeStore,
    pub solid_id: SolidId,
    pub name: String,
    pub history: ObjectHistory,
    /// Stable identity for GPU mesh caching — never reused.
    pub id: u64,
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
    ///
    /// If `cursor_ray` is provided, the target drifts toward the point on the
    /// ray at the current distance so that the geometry under the cursor stays
    /// roughly fixed.
    pub fn zoom(&mut self, delta: f32, cursor_ray: Option<&brep_algo::bvh::Ray>) {
        let factor = 1.0 - delta as f64 * 0.1;
        let new_dist = (self.distance * factor).clamp(0.1, 10_000.0);
        let dist_delta = self.distance - new_dist; // positive when zooming in

        if let Some(ray) = cursor_ray {
            // Point on the cursor ray at the current eye distance.
            let focus = ray.at(self.distance);
            // Shift target by the fraction of the distance change, in the
            // direction from target toward the cursor focus point.
            let shift = (focus - self.target) * (dist_delta / self.distance);
            self.target += shift;
        }

        self.distance = new_dist;
    }

    /// Unproject a screen pixel into a world-space ray.
    ///
    /// `(screen_x, screen_y)` are pixel coordinates from the top-left.
    pub fn unproject_ray(&self, screen_x: f32, screen_y: f32, w: u32, h: u32) -> Ray {
        // NDC in [-1, 1]: right/up are positive.
        let ndc_x = 2.0 * screen_x as f64 / w as f64 - 1.0;
        let ndc_y = 1.0 - 2.0 * screen_y as f64 / h as f64;

        let eye    = self.eye();
        let f      = (self.target - eye).normalize();   // forward (into scene)
        let right  = f.cross(&Vec3::z()).normalize();   // screen right
        let up_cam = right.cross(&f);                   // screen up

        // Analytical perspective unproject — no matrix inversion needed.
        // Ray at NDC (nx, ny) goes through camera-space point (nx·tan_x, ny·tan_y, -1).
        let aspect  = w as f64 / h as f64;
        let half_tan = (self.fov_y_deg.to_radians() * 0.5).tan(); // tan(fov_y/2)
        let direction = (f + ndc_x * aspect * half_tan * right + ndc_y * half_tan * up_cam)
            .normalize();

        Ray::new(eye, direction)
    }

    /// Project a world-space point to pixel coordinates `(x, y)` from the top-left.
    /// Returns `None` if the point is behind the camera.
    pub fn project_to_screen(&self, pt: Point3, w: u32, h: u32) -> Option<(f32, f32)> {
        use nalgebra::{Matrix4, Vector4};
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
        let fc = 1.0 / (self.fov_y_deg.to_radians() * 0.5).tan();
        let (near, far) = (0.05, 1000.0_f64);
        let proj = Matrix4::new(
            fc / aspect, 0.0,  0.0,                   0.0,
            0.0,         fc,   0.0,                   0.0,
            0.0,         0.0,  far / (near - far),     (far * near) / (near - far),
            0.0,         0.0, -1.0,                    0.0,
        );
        let clip = proj * view * Vector4::new(pt.x, pt.y, pt.z, 1.0);
        if clip.w <= 0.0 { return None; }
        let nx = clip.x / clip.w;
        let ny = clip.y / clip.w;
        Some((
            ((nx + 1.0) * 0.5 * w as f64) as f32,
            ((1.0 - ny) * 0.5 * h as f64) as f32,
        ))
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
    /// Snap the camera to look along `dir` with `up` as the up vector.
    SnapCamera { azimuth: f64, elevation: f64 },
    /// Orbit the camera by screen-space pixel deltas (from the gizmo drag).
    OrbitCamera { dx: f32, dy: f32 },
    // ── Sketch actions ────────────────────────────────────────────────────────
    EnterSketch(SketchPlane),
    ExitSketch,
    SketchAddPoint(Point3),
    SketchUndoPoint,
    SketchClose,
    SketchSetDistance(f64),
    SketchExtrude(f64),
    // ── Sketch constraint actions ─────────────────────────────────────────────
    /// Toggle-select a segment by index (max 2 at a time).
    SketchSelectSegment(usize),
    SketchClearSegSelection,
    SketchAddConstraint(SketchConstraint),
    /// Remove a constraint by its index in `SketchState::constraints`.
    SketchRemoveConstraint(usize),
    /// Open the angle-input dialog for two selected segments.
    SketchBeginAngleInput { seg_a: usize, seg_b: usize },
    /// Dismiss the angle-input dialog without applying.
    SketchCancelAngleInput,
    /// Toggle-select a vertex by index (max 2 at a time, clears seg_selection).
    SketchSelectVertex(usize),
    SketchClearPtSelection,
    /// Open the length-input dialog.
    SketchBeginLengthInput(LengthTarget),
    /// Dismiss the length-input dialog without applying.
    SketchCancelLengthInput,
}

// ── Camera animation ──────────────────────────────────────────────────────────

struct CameraAnimation {
    start_az:  f64,
    start_el:  f64,
    target_az: f64,
    target_el: f64,
    started:   Instant,
    /// Duration in seconds.
    duration:  f64,
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
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
    /// Monotonically increasing counter for assigning stable object IDs.
    next_object_id: u64,
    /// Set to `true` whenever objects change, so meshes can be re-uploaded.
    pub scene_dirty: bool,
    /// In-progress camera snap animation, if any.
    camera_anim: Option<CameraAnimation>,
    /// Active sketch being drawn, if any.
    pub sketch: Option<SketchState>,
}

/// Run the constraint solver on the sketch, updating point positions in place.
fn apply_constraints(sk: &mut SketchState) {
    if sk.constraints.is_empty() || sk.points.len() < 2 { return; }
    let (u, v) = sk.plane.uv_axes();
    // Map Point3 → [f64; 2] in plane coordinates.
    let mut pts2d: Vec<[f64; 2]> = sk.points.iter()
        .map(|p| [p.coords.dot(&u), p.coords.dot(&v)])
        .collect();
    let n = pts2d.len();
    let result = solve_constraints(&mut pts2d, &sk.constraints, n);
    sk.constraints_conflict = result == SolveResult::Conflict;
    if result == SolveResult::Ok {
        for (i, p) in sk.points.iter_mut().enumerate() {
            *p = Point3::origin() + u * pts2d[i][0] + v * pts2d[i][1];
        }
    }
}

impl EditorState {
    pub fn new() -> Self {
        let mut s = Self {
            objects: Vec::new(),
            selection: std::collections::HashSet::new(),
            camera: ViewportCamera::default(),
            history: History::new(),
            name_counter: 0,
            next_object_id: 0,
            scene_dirty: true,
            camera_anim: None,
            sketch: None,
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
                        let id = self.alloc_id();
                        self.objects.push(SceneObject {
                            store: new_store,
                            solid_id: new_solid,
                            name,
                            history,
                            id,
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
            UiAction::SnapCamera { azimuth, elevation } => {
                self.camera_anim = Some(CameraAnimation {
                    start_az:  self.camera.azimuth,
                    start_el:  self.camera.elevation,
                    target_az: azimuth,
                    target_el: elevation,
                    started:   Instant::now(),
                    duration:  0.2,
                });
                false
            }
            UiAction::OrbitCamera { dx, dy } => {
                self.camera.orbit(dx, dy);
                false
            }

            // ── Sketch ────────────────────────────────────────────────────────
            UiAction::EnterSketch(plane) => {
                self.sketch = Some(SketchState {
                    plane,
                    points: Vec::new(),
                    closed: false,
                    extrude_dist: 1.0,
                    constraints: Vec::new(),
                    seg_selection: Vec::new(),
                    pt_selection: Vec::new(),
                    constraints_conflict: false,
                });
                self.selection.clear();
                // Animate the camera to look at the chosen plane.
                let (az, el) = match plane {
                    SketchPlane::XY => (self.camera.azimuth, PI / 2.0 - 0.05),
                    SketchPlane::XZ => (-PI / 2.0, 0.0),
                    SketchPlane::YZ => (0.0, 0.0),
                };
                self.camera_anim = Some(CameraAnimation {
                    start_az:  self.camera.azimuth,
                    start_el:  self.camera.elevation,
                    target_az: az,
                    target_el: el,
                    started:   Instant::now(),
                    duration:  0.3,
                });
                false
            }
            UiAction::ExitSketch => {
                self.sketch = None;
                false
            }
            UiAction::SketchAddPoint(p) => {
                if let Some(sk) = &mut self.sketch {
                    if !sk.closed {
                        sk.points.push(p);
                        apply_constraints(sk);
                    }
                }
                false
            }
            UiAction::SketchUndoPoint => {
                if let Some(sk) = &mut self.sketch {
                    if sk.closed { sk.closed = false; } else { sk.points.pop(); }
                }
                false
            }
            UiAction::SketchClose => {
                if let Some(sk) = &mut self.sketch {
                    if sk.points.len() >= 3 { sk.closed = true; }
                }
                false
            }
            UiAction::SketchSetDistance(d) => {
                if let Some(sk) = &mut self.sketch { sk.extrude_dist = d; }
                false
            }
            UiAction::SketchSelectSegment(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.pt_selection.clear(); // segment selection clears vertex selection
                    if let Some(pos) = sk.seg_selection.iter().position(|&x| x == i) {
                        sk.seg_selection.remove(pos);
                    } else {
                        if sk.seg_selection.len() >= 2 { sk.seg_selection.remove(0); }
                        sk.seg_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchClearSegSelection => {
                if let Some(sk) = &mut self.sketch { sk.seg_selection.clear(); }
                false
            }
            UiAction::SketchSelectVertex(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.seg_selection.clear(); // vertex selection clears segment selection
                    if let Some(pos) = sk.pt_selection.iter().position(|&x| x == i) {
                        sk.pt_selection.remove(pos);
                    } else {
                        if sk.pt_selection.len() >= 2 { sk.pt_selection.remove(0); }
                        sk.pt_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchClearPtSelection => {
                if let Some(sk) = &mut self.sketch { sk.pt_selection.clear(); }
                false
            }
            UiAction::SketchAddConstraint(c) => {
                if let Some(sk) = &mut self.sketch {
                    sk.constraints.push(c);
                    apply_constraints(sk);
                }
                false
            }
            // Handled at the app layer (main.rs); editor is not involved.
            UiAction::SketchBeginAngleInput { .. }
            | UiAction::SketchCancelAngleInput
            | UiAction::SketchBeginLengthInput(_)
            | UiAction::SketchCancelLengthInput => false,

            UiAction::SketchRemoveConstraint(idx) => {
                if let Some(sk) = &mut self.sketch {
                    if idx < sk.constraints.len() {
                        sk.constraints.remove(idx);
                        apply_constraints(sk);
                    }
                }
                false
            }

            UiAction::SketchExtrude(dist) => {
                let Some(sk) = self.sketch.take() else { return false };
                if sk.points.len() < 3 { self.sketch = None; return false; }
                use brep_param::{ExtrudeFeature, Feature};
                use std::sync::Arc;
                let feature = ExtrudeFeature {
                    profile: sk.points,
                    direction: sk.plane.normal(),
                    distance: dist,
                };
                if let Ok(store_arc) = feature.compute(&[]) {
                    let store: brep_topo::store::ShapeStore =
                        Arc::try_unwrap(store_arc).unwrap_or_else(|a| (*a).clone());
                    let solid_id = store.solid_ids().next().expect("extruded solid");
                    let name = format!("Sketch-{}", self.next_name());
                    let id = self.alloc_id();
                    self.save_snapshot();
                    self.objects.push(SceneObject {
                        store,
                        solid_id,
                        name,
                        history: ObjectHistory::Sketch { plane: sk.plane },
                        id,
                    });
                    self.selection.clear();
                    self.selection.insert(self.objects.len() - 1);
                    self.scene_dirty = true;
                }
                true
            }
        }
    }

    /// Advance any in-progress camera animation.  Returns `true` while animating
    /// (caller should keep requesting redraws).
    pub fn tick_animation(&mut self) -> bool {
        let Some(anim) = &self.camera_anim else { return false };
        let elapsed = anim.started.elapsed().as_secs_f64();
        let t = smoothstep((elapsed / anim.duration).min(1.0));
        self.camera.azimuth   = anim.start_az  + (anim.target_az - anim.start_az)  * t;
        self.camera.elevation = anim.start_el  + (anim.target_el - anim.start_el)  * t;
        if elapsed >= anim.duration {
            self.camera_anim = None;
            false
        } else {
            true
        }
    }

    fn save_snapshot(&mut self) {
        self.history.push(self.objects.clone());
    }

    fn next_name(&mut self) -> u32 {
        self.name_counter += 1;
        self.name_counter
    }

    fn alloc_id(&mut self) -> u64 {
        let id = self.next_object_id;
        self.next_object_id += 1;
        id
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
            let id = self.alloc_id();
            self.objects.push(SceneObject { store, solid_id, name, history, id });
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
