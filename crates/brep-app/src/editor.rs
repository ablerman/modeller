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
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum ObjectHistory {
    Primitive(PrimitiveKind),
    Sketch {
        plane:        SketchPlane,
        points:       Vec<Point3>,
        constraints:  Vec<SketchConstraint>,
        extrude_dist: f64,
    },
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
            ObjectHistory::Sketch { .. }              => "Sketch",
            ObjectHistory::Boolean { kind: BooleanKind::Union,        .. } => "Union",
            ObjectHistory::Boolean { kind: BooleanKind::Difference,   .. } => "Difference",
            ObjectHistory::Boolean { kind: BooleanKind::Intersection, .. } => "Intersection",
        }
    }
}

// ── Sketch ────────────────────────────────────────────────────────────────────

/// A fixed reference entity in the sketch that points/segments can be constrained against.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RefEntity {
    /// The sketch origin (0, 0) in plane coordinates.
    Origin,
    /// The sketch X-axis (the U-axis, v = 0).
    XAxis,
    /// The sketch Y-axis (the V-axis, u = 0).
    YAxis,
}

/// Which principal plane the sketch lives on.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
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

/// Active drawing tool in the sketcher.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DrawTool { Pointer, Polyline, Arc, Rectangle, Circle }

/// Intermediate state accumulated by multi-click drawing tools (Arc/Rect/Circle).
/// Stored in `SketchState`; cleared when the tool operation is committed or cancelled.
#[derive(Clone, Debug)]
pub enum ToolInProgress {
    /// Arc tool: first point (start) has been placed.
    Arc1 { start: Point3 },
    /// Arc tool: start and end-point have been placed; waiting for arc center.
    Arc2 { start: Point3, end_pt: Point3 },
    /// Circle tool: center has been placed; waiting for radius point.
    CircleCenter { center: Point3 },
    /// Rectangle tool: first corner has been placed; waiting for opposite corner.
    RectFirst { corner: Point3 },
}

/// What a length constraint applies to — used for the modal dialog.
#[derive(Clone, Copy, Debug)]
pub enum LengthTarget {
    /// Constrain a single segment to a fixed length.
    Segment(usize),
    /// Constrain the distance between two (possibly non-adjacent) vertices.
    Points(usize, usize),
}

// ── Sketch plane geometry helpers ─────────────────────────────────────────────

/// Project a world-space point onto the sketch plane, returning (u, v) coordinates.
pub(crate) fn world_to_plane(p: Point3, plane: SketchPlane) -> (f64, f64) {
    let (u, v) = plane.uv_axes();
    (p.coords.dot(&u), p.coords.dot(&v))
}

/// Lift 2D sketch-plane coordinates back to world space.
pub(crate) fn plane_to_world(u_val: f64, v_val: f64, plane: SketchPlane) -> Point3 {
    let (u, v) = plane.uv_axes();
    Point3::from(u * u_val + v * v_val)
}

/// Circumscribed circle through three 2D points.
/// Returns `None` if the points are collinear.
fn circumscribed_circle_2d(
    a: (f64, f64), b: (f64, f64), c: (f64, f64),
) -> Option<((f64, f64), f64)> {
    let n_ab = (-(b.1 - a.1), b.0 - a.0);
    let n_bc = (-(c.1 - b.1), c.0 - b.0);
    let m_ab = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
    let m_bc = ((b.0 + c.0) * 0.5, (b.1 + c.1) * 0.5);
    let dx = m_bc.0 - m_ab.0;
    let dy = m_bc.1 - m_ab.1;
    let det = -n_ab.0 * n_bc.1 + n_bc.0 * n_ab.1;
    if det.abs() < 1e-10 { return None; }
    let t = (-dx * n_bc.1 + n_bc.0 * dy) / det;
    let cx = m_ab.0 + t * n_ab.0;
    let cy = m_ab.1 + t * n_ab.1;
    let r = ((cx - a.0).powi(2) + (cy - a.1).powi(2)).sqrt();
    Some(((cx, cy), r))
}

/// Tessellate a circular arc through `start`, `through`, `end` into world-space points.
/// Returns `None` if the three points are collinear (degenerate arc).
/// The returned slice includes both `start` and `end`.
pub(crate) fn tessellate_arc(
    start: Point3, through: Point3, end: Point3, plane: SketchPlane,
) -> Option<Vec<Point3>> {
    use std::f64::consts::TAU;
    let a = world_to_plane(start,   plane);
    let b = world_to_plane(through, plane);
    let c = world_to_plane(end,     plane);
    let ((cx, cy), r) = circumscribed_circle_2d(a, b, c)?;

    let theta0 = (a.1 - cy).atan2(a.0 - cx);
    let theta1 = (b.1 - cy).atan2(b.0 - cx);
    let theta2 = (c.1 - cy).atan2(c.0 - cx);

    let norm = |ang: f64| (ang % TAU + TAU) % TAU;
    let r1 = norm(theta1 - theta0);
    let r2 = norm(theta2 - theta0);
    // Span: positive = CCW (r1 < r2), negative = CW.
    let span = if r1 < r2 { r2 } else { r2 - TAU };

    // Scale segment count with arc length; minimum 4 segments.
    let n = (32.0 * span.abs() / TAU).ceil().max(4.0) as usize;

    let pts = (0..=n).map(|i| {
        let theta = theta0 + span * (i as f64 / n as f64);
        plane_to_world(cx + r * theta.cos(), cy + r * theta.sin(), plane)
    }).collect();
    Some(pts)
}

/// Project a raw center click onto the perpendicular bisector of (start, end_pt)
/// so that the resulting center is equidistant from both endpoints.
/// The signed distance from the chord midpoint along the bisector is preserved,
/// so the caller's intent (which side to bow toward) is kept.
pub(crate) fn project_center_to_arc_bisector(
    start: Point3, end_pt: Point3, center: Point3, plane: SketchPlane,
) -> Point3 {
    let s = world_to_plane(start,  plane);
    let e = world_to_plane(end_pt, plane);
    let c = world_to_plane(center, plane);
    let mx = (s.0 + e.0) * 0.5;
    let my = (s.1 + e.1) * 0.5;
    let chord_dx = e.0 - s.0;
    let chord_dy = e.1 - s.1;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt();
    if chord_len < 1e-10 { return center; }
    let px = -chord_dy / chord_len;
    let py =  chord_dx / chord_len;
    let t = (c.0 - mx) * px + (c.1 - my) * py;
    plane_to_world(mx + t * px, my + t * py, plane)
}

/// Tessellate a circular arc defined by start, end, and a center hint.
/// The center is projected onto the perpendicular bisector of (start, end_pt)
/// so the arc passes through both endpoints exactly.
/// The arc sweeps the shorter path (|span| ≤ π).
pub(crate) fn tessellate_arc_from_center(
    start: Point3, end_pt: Point3, center: Point3, plane: SketchPlane,
) -> Vec<Point3> {
    use std::f64::consts::PI;
    let s = world_to_plane(start,  plane);
    let e = world_to_plane(end_pt, plane);

    // Project the center onto the perpendicular bisector so both endpoints
    // are exactly at `radius` from `c`, regardless of where the click landed.
    let mx = (s.0 + e.0) * 0.5;
    let my = (s.1 + e.1) * 0.5;
    let chord_dx = e.0 - s.0;
    let chord_dy = e.1 - s.1;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt();
    let c_raw = world_to_plane(center, plane);
    let c = if chord_len < 1e-10 {
        c_raw
    } else {
        let px = -chord_dy / chord_len;
        let py =  chord_dx / chord_len;
        let t = (c_raw.0 - mx) * px + (c_raw.1 - my) * py;
        (mx + t * px, my + t * py)
    };

    let radius = ((s.0 - c.0).powi(2) + (s.1 - c.1).powi(2)).sqrt();
    if radius < 1e-10 { return vec![start, end_pt]; }

    let theta_s = (s.1 - c.1).atan2(s.0 - c.0);
    let theta_e = (e.1 - c.1).atan2(e.0 - c.0);

    // Sweep the shorter arc: normalize delta to (−π, π].
    let mut span = theta_e - theta_s;
    span = span.rem_euclid(2.0 * PI);
    if span > PI { span -= 2.0 * PI; }

    let n = ((32.0 * span.abs() / (2.0 * PI)).ceil() as usize).max(4);
    // Force exact endpoints to avoid any floating-point drift.
    let mut pts: Vec<Point3> = (0..=n).map(|i| {
        let theta = theta_s + span * (i as f64 / n as f64);
        plane_to_world(c.0 + radius * theta.cos(), c.1 + radius * theta.sin(), plane)
    }).collect();
    if let Some(first) = pts.first_mut() { *first = start; }
    if let Some(last)  = pts.last_mut()  { *last  = end_pt; }
    pts
}

/// Tessellate a full circle (center → radius point) into `n` world-space points.
/// The points form a closed polygon when connected back to the first.
pub(crate) fn tessellate_circle(
    center: Point3, radius_pt: Point3, plane: SketchPlane, n: usize,
) -> Vec<Point3> {
    use std::f64::consts::TAU;
    let c = world_to_plane(center, plane);
    let rp = world_to_plane(radius_pt, plane);
    let r = ((rp.0 - c.0).powi(2) + (rp.1 - c.1).powi(2)).sqrt();
    let theta0 = (rp.1 - c.1).atan2(rp.0 - c.0); // start angle at radius_pt
    (0..n).map(|i| {
        let theta = theta0 + TAU * i as f64 / n as f64;
        plane_to_world(c.0 + r * theta.cos(), c.1 + r * theta.sin(), plane)
    }).collect()
}

/// Compute the four corners of an axis-aligned (in sketch-plane space) rectangle
/// given two opposite corners in world space.
pub(crate) fn rect_corners(c1: Point3, c2: Point3, plane: SketchPlane) -> [Point3; 4] {
    let (u1, v1) = world_to_plane(c1, plane);
    let (u2, v2) = world_to_plane(c2, plane);
    [
        plane_to_world(u1, v1, plane),
        plane_to_world(u2, v1, plane),
        plane_to_world(u2, v2, plane),
        plane_to_world(u1, v2, plane),
    ]
}

// ── Sketch state ──────────────────────────────────────────────────────────────

/// Live in-progress 2D sketch being drawn by the user.
#[derive(Clone, Debug)]
pub struct SketchState {
    /// Display name for this sketch (editable before finishing).
    pub name:                 String,
    pub plane:                SketchPlane,
    pub points:               Vec<Point3>,  // world-space vertices on the plane (placed in order)
    /// Whether the polyline has been closed (last point connected back to first).
    pub closed:               bool,
    /// Active geometric constraints on this sketch.
    pub constraints:          Vec<SketchConstraint>,
    /// Indices of currently selected segments (max 2).
    pub seg_selection:        Vec<usize>,
    /// Indices of currently selected vertices (max 2, for length/distance constraints).
    pub pt_selection:         Vec<usize>,
    /// Selected reference entity (origin / axis), if any.
    pub ref_selection:        Option<RefEntity>,
    /// Set when the last constraint solve did not converge.
    pub constraints_conflict: bool,
    /// Parallel to `constraints`: `true` if removing that constraint would let
    /// the system converge (i.e., it is one of the conflicting constraints).
    /// Empty when there is no conflict.
    pub violated_constraints: Vec<bool>,
    /// Indices of currently selected constraints (for highlighting).
    pub constraint_selection: Vec<usize>,
    /// Independent undo/redo stack for sketch edits.
    pub history: SketchHistory,
    /// Currently active drawing tool.
    pub active_tool: DrawTool,
    /// Partial state for multi-click tools (Arc, Rectangle, Circle).
    /// `None` when idle (Polyline or between tool operations).
    pub tool_in_progress: Option<ToolInProgress>,
    /// Closed profiles already committed in this sketch session (e.g. each finished
    /// rectangle or circle).  The active `points`/`closed`/`constraints` represent
    /// the profile currently being drawn.
    pub committed_profiles: Vec<CommittedProfile>,
    /// Index of the committed profile currently selected (for constraints like PointOnCircle).
    pub committed_selection: Option<usize>,
    /// A specific control point within a committed profile that is selected (profile_idx, vertex_idx).
    /// Used for arc/circle center point selection — separate from curve selection.
    pub committed_pt_selection: Option<(usize, usize)>,
}

/// How the points of a `CommittedProfile` should be interpreted geometrically.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize, PartialEq)]
pub enum ProfileShape {
    /// Straight-line polygon.  Points are vertices; `closed` determines whether the last
    /// point connects back to the first.
    #[default]
    Polyline,
    /// Full circle.  `points[0]` = center, `points[1]` = a point on the circumference
    /// (defines the radius).
    Circle,
    /// Circular arc.  `points[0]` = start, `points[1]` = through-point, `points[2]` = end.
    Arc,
}

/// A single sub-profile within a sketch (e.g. one circle, one rectangle, or one
/// closed polyline).  A `SketchState` can accumulate many of these alongside the active
/// open profile being drawn.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CommittedProfile {
    pub points:      Vec<Point3>,
    pub closed:      bool,
    /// Geometric interpretation of `points`.  Defaults to `Polyline` for backward compat.
    #[serde(default)]
    pub shape:       ProfileShape,
    /// The sketch plane this profile lives on (needed for arc tessellation during rendering).
    #[serde(default)]
    pub plane:       Option<SketchPlane>,
    pub constraints: Vec<SketchConstraint>,
}

/// A sketch stored in the operations list.  Contains one or more profiles.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct RawSketch {
    pub name:        String,
    pub plane:       SketchPlane,
    /// Primary profile (the one that was active when Finish was pressed, or the first
    /// profile for backward compatibility).
    pub points:      Vec<Point3>,
    pub constraints: Vec<SketchConstraint>,
    /// Additional profiles (circles, rectangles, extra polylines).
    #[serde(default)]
    pub extra_profiles: Vec<CommittedProfile>,
    /// Stable identity (for future use, e.g. referencing by extrude ops).
    pub id:          u64,
}

/// One entry in the operations list — either a closed sketch profile or a solid.
#[derive(Clone)]
pub enum SceneEntry {
    Sketch(RawSketch),
    Solid(SceneObject),
}

impl SceneEntry {
    #[allow(dead_code)]
    pub fn name(&self) -> &str {
        match self { Self::Sketch(s) => &s.name, Self::Solid(s) => &s.name }
    }
    pub fn id(&self) -> u64 {
        match self { Self::Sketch(s) => s.id, Self::Solid(s) => s.id }
    }
    pub fn as_solid(&self) -> Option<&SceneObject> {
        match self { Self::Solid(s) => Some(s), _ => None }
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

    /// Returns the viewport-aligned perpendicular directions for horizontal and vertical
    /// constraints on the given sketch plane, as unit (perp_u, perp_v) pairs in UV space.
    ///
    /// - `h_perp`: normal to the horizontal direction (= camera-up projected onto UV plane)
    /// - `v_perp`: normal to the vertical direction (= camera-right projected onto UV plane)
    pub fn sketch_align_dirs(&self, plane: SketchPlane) -> ((f64, f64), (f64, f64)) {
        let eye = self.eye();
        let f = (self.target - eye).normalize();
        let right = f.cross(&Vec3::z()).normalize();
        let up_cam = right.cross(&f);
        let (u_axis, v_axis) = plane.uv_axes();

        let h_pu = up_cam.dot(&u_axis);
        let h_pv = up_cam.dot(&v_axis);
        let h_len = (h_pu * h_pu + h_pv * h_pv).sqrt().max(1e-12);

        let v_pu = right.dot(&u_axis);
        let v_pv = right.dot(&v_axis);
        let v_len = (v_pu * v_pu + v_pv * v_pv).sqrt().max(1e-12);

        ((h_pu / h_len, h_pv / h_len), (v_pu / v_len, v_pv / v_len))
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

/// A snapshot of the full operations list, used for undo/redo.
type SceneSnapshot = Vec<SceneEntry>;

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

// ── Sketch undo/redo ──────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct SketchSnapshot {
    points:             Vec<Point3>,
    constraints:        Vec<SketchConstraint>,
    closed:             bool,
    committed_profiles: Vec<CommittedProfile>,
}

#[derive(Clone, Debug)]
pub struct SketchHistory {
    undo_stack: Vec<SketchSnapshot>,
    redo_stack: Vec<SketchSnapshot>,
}

impl SketchHistory {
    fn new() -> Self { Self { undo_stack: Vec::new(), redo_stack: Vec::new() } }

    fn push(&mut self, snapshot: SketchSnapshot) {
        self.undo_stack.push(snapshot);
        self.redo_stack.clear();
    }

    fn undo(&mut self, current: SketchSnapshot) -> Option<SketchSnapshot> {
        let prev = self.undo_stack.pop()?;
        self.redo_stack.push(current);
        Some(prev)
    }

    fn redo(&mut self, current: SketchSnapshot) -> Option<SketchSnapshot> {
        let next = self.redo_stack.pop()?;
        self.undo_stack.push(current);
        Some(next)
    }

    pub fn can_undo(&self) -> bool { !self.undo_stack.is_empty() }
    pub fn can_redo(&self) -> bool { !self.redo_stack.is_empty() }
}

// ── Primitive kind ────────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PrimitiveKind {
    Box,
    Cylinder,
    Sphere,
    Cone,
}

// ── UI actions ────────────────────────────────────────────────────────────────

/// An action the UI requests the editor to perform.
#[derive(Clone)]
pub enum UiAction {
    // ── File actions ──────────────────────────────────────────────────────────
    New,
    Open,
    Save,
    SaveAs,
    // ── Scene actions ─────────────────────────────────────────────────────────
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
    /// Connect the last point back to the first, closing the polyline. Stays in sketch mode.
    SketchCloseLoop,
    /// Save the sketch to the operations list and exit sketch mode.
    SketchFinish,
    // ── Sketch constraint actions ─────────────────────────────────────────────
    /// Toggle-select a segment by index (max 2 at a time).
    SketchSelectSegment(usize),
    SketchAddConstraint(SketchConstraint),
    /// Remove a constraint by its index in `SketchState::constraints`.
    SketchRemoveConstraint(usize),
    /// Open the angle-input dialog for two selected segments.
    SketchBeginAngleInput { seg_a: usize, seg_b: usize },
    /// Dismiss the angle-input dialog without applying.
    SketchCancelAngleInput,
    /// Toggle-select a vertex by index (max 2 at a time, clears seg_selection).
    SketchSelectVertex(usize),
    /// Select (or toggle off) a reference entity (origin / axis).
    SketchSelectRef(RefEntity),
    /// Open the length-input dialog.
    SketchBeginLengthInput(LengthTarget),
    /// Dismiss the length-input dialog without applying.
    SketchCancelLengthInput,
    /// Re-open a finished sketch from the operations list for editing.
    OpenSketch(usize),
    /// Rename the active sketch.
    SketchRename(String),
    /// Toggle-select a constraint by index.
    SketchSelectConstraint(usize),
    /// Toggle-select a segment from the panel list (no max-2 cap, preserves other segs).
    SketchPanelSelectSegment(usize),
    /// Toggle-select a vertex from the panel list (no max-2 cap, preserves other pts).
    SketchPanelSelectVertex(usize),
    /// Switch the active drawing tool.
    SketchSetTool(DrawTool),
    /// Cancel the current in-progress drawing operation and clear the active profile points.
    /// Committed profiles (circles, rectangles) are kept.
    SketchAbortActive,
    /// Commit the active polyline points as an open committed profile, then reset the active profile.
    /// Requires ≥2 points and no multi-step tool in progress.
    SketchCommitPolyline,
    /// Select (or deselect) a committed profile by index, for use with constraints.
    SketchSelectCommitted(Option<usize>),
    /// Select a specific control point within a committed profile (profile_idx, vertex_idx).
    SketchSelectCommittedPoint(Option<(usize, usize)>),
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
    pub entries: Vec<SceneEntry>,
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
        sk.violated_constraints.clear();
        for (i, p) in sk.points.iter_mut().enumerate() {
            *p = Point3::origin() + u * pts2d[i][0] + v * pts2d[i][1];
        }
    } else {
        // Identify which constraints are involved in the conflict: try removing
        // each one in turn — if removal makes the system solvable, that constraint
        // is a contributor.
        sk.violated_constraints = find_conflicting_constraints(&pts2d, &sk.constraints, n);
    }
}

/// For each constraint, check whether removing it allows the remaining system to
/// converge.  Returns a parallel `Vec<bool>` — `true` means that constraint is
/// involved in the conflict.
fn find_conflicting_constraints(
    pts: &[[f64; 2]],
    constraints: &[SketchConstraint],
    n_pts: usize,
) -> Vec<bool> {
    constraints
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let reduced: Vec<&SketchConstraint> = constraints
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, c)| c)
                .collect();
            if reduced.is_empty() {
                // Removing the only constraint always succeeds.
                return true;
            }
            let reduced_owned: Vec<SketchConstraint> = reduced.into_iter().cloned().collect();
            let mut pts_copy: Vec<[f64; 2]> = pts.to_vec();
            solve_constraints(&mut pts_copy, &reduced_owned, n_pts) == SolveResult::Ok
        })
        .collect()
}

fn sketch_snapshot(sk: &SketchState) -> SketchSnapshot {
    SketchSnapshot {
        points:             sk.points.clone(),
        constraints:        sk.constraints.clone(),
        closed:             sk.closed,
        committed_profiles: sk.committed_profiles.clone(),
    }
}

impl EditorState {
    pub fn new() -> Self {
        let mut s = Self {
            entries: Vec::new(),
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
            // File actions are handled at the app layer (main.rs).
            UiAction::New | UiAction::Open | UiAction::Save | UiAction::SaveAs => false,
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
                    self.entries.remove(i);
                }
                self.selection.clear();
                self.scene_dirty = true;
                true
            }
            UiAction::BooleanOp(kind) => {
                // Only operate on selected solid entries.
                let solid_sel: Vec<usize> = self.selection.iter().cloned()
                    .filter(|&i| matches!(self.entries.get(i), Some(SceneEntry::Solid(_))))
                    .collect();
                if solid_sel.len() != 2 {
                    return false;
                }
                let mut sel = solid_sel;
                sel.sort_unstable();
                let (a_idx, b_idx) = (sel[0], sel[1]);
                // Clone all needed data before any mutable borrows.
                let a_store    = self.entries[a_idx].as_solid().unwrap().store.clone();
                let a_solid_id = self.entries[a_idx].as_solid().unwrap().solid_id;
                let b_store    = self.entries[b_idx].as_solid().unwrap().store.clone();
                let b_solid_id = self.entries[b_idx].as_solid().unwrap().solid_id;
                let (hi, lo)   = if a_idx > b_idx { (a_idx, b_idx) } else { (b_idx, a_idx) };
                let hi_hist    = self.entries[hi].as_solid().unwrap().history.clone();
                let lo_hist    = self.entries[lo].as_solid().unwrap().history.clone();
                self.save_snapshot();
                let result = boolean_op(&a_store, a_solid_id, &b_store, b_solid_id, kind, 1e-7);
                match result {
                    Ok(new_store) => {
                        let new_solid_id = new_store.solid_ids().next()
                            .expect("boolean result has a solid");
                        let name = format!("Bool-{}", self.next_name());
                        let (left_hist, right_hist) = if a_idx < b_idx {
                            (lo_hist, hi_hist)
                        } else {
                            (hi_hist, lo_hist)
                        };
                        self.entries.remove(hi);
                        self.entries.remove(lo);
                        let history = ObjectHistory::Boolean {
                            kind,
                            left:  Box::new(left_hist),
                            right: Box::new(right_hist),
                        };
                        let id = self.alloc_id();
                        self.entries.push(SceneEntry::Solid(SceneObject {
                            store: new_store,
                            solid_id: new_solid_id,
                            name,
                            history,
                            id,
                        }));
                        self.selection.clear();
                        self.selection.insert(self.entries.len() - 1);
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
                if let Some(sk) = &mut self.sketch {
                    if sk.history.can_undo() {
                        let current = sketch_snapshot(sk);
                        if let Some(prev) = sk.history.undo(current) {
                            sk.points              = prev.points;
                            sk.constraints         = prev.constraints;
                            sk.closed              = prev.closed;
                            sk.committed_profiles  = prev.committed_profiles;
                            sk.tool_in_progress    = None;
                            apply_constraints(sk);
                        }
                    }
                    false
                } else if let Some(prev) = self.history.undo(self.entries.clone()) {
                    self.entries = prev;
                    self.selection.clear();
                    self.scene_dirty = true;
                    true
                } else {
                    false
                }
            }
            UiAction::Redo => {
                if let Some(sk) = &mut self.sketch {
                    if sk.history.can_redo() {
                        let current = sketch_snapshot(sk);
                        if let Some(next) = sk.history.redo(current) {
                            sk.points              = next.points;
                            sk.constraints         = next.constraints;
                            sk.closed              = next.closed;
                            sk.committed_profiles  = next.committed_profiles;
                            sk.tool_in_progress    = None;
                            apply_constraints(sk);
                        }
                    }
                    false
                } else if let Some(next) = self.history.redo(self.entries.clone()) {
                    self.entries = next;
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
                    name: format!("Sketch-{}", self.next_name()),
                    plane,
                    points: Vec::new(),
                    closed: false,
                    constraints: Vec::new(),
                    seg_selection: Vec::new(),
                    pt_selection: Vec::new(),
                    ref_selection: None,
                    constraints_conflict: false,
                    violated_constraints: Vec::new(),
                    constraint_selection: Vec::new(),
                    history:            SketchHistory::new(),
                    active_tool:        DrawTool::Polyline,
                    tool_in_progress:   None,
                    committed_profiles: Vec::new(),
                    committed_selection: None,
                    committed_pt_selection: None,
                });
                self.selection.clear();
                // Animate the camera to look at the chosen plane with axes oriented
                // so that the sketch U axis goes right and V axis goes up on screen.
                // XY: U=X right, V=Y up  → look from +Z (az=-π/2, el≈π/2)
                // XZ: U=X right, V=Z up  → look from -Y (az=-π/2, el=0)
                // YZ: U=Y right, V=Z up  → look from +X (az=0,    el=0)
                let (az, el) = match plane {
                    SketchPlane::XY => (-PI / 2.0, PI / 2.0 - 0.05),
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
            UiAction::OpenSketch(i) => {
                let Some(SceneEntry::Sketch(raw)) = self.entries.get(i).cloned() else {
                    return false;
                };
                self.entries.remove(i);
                self.selection.clear();
                let plane = raw.plane;
                // Backfill the plane for any committed profiles that predate the field.
                let committed_profiles: Vec<_> = raw.extra_profiles.into_iter()
                    .map(|mut cp| { if cp.plane.is_none() { cp.plane = Some(plane); } cp })
                    .collect();
                self.sketch = Some(SketchState {
                    name:                 raw.name,
                    plane,
                    points:               raw.points,
                    closed:               true,
                    constraints:          raw.constraints,
                    seg_selection:        Vec::new(),
                    pt_selection:         Vec::new(),
                    ref_selection:        None,
                    constraints_conflict: false,
                    violated_constraints: Vec::new(),
                    constraint_selection: Vec::new(),
                    history:              SketchHistory::new(),
                    active_tool:          DrawTool::Polyline,
                    tool_in_progress:     None,
                    committed_profiles,
                    committed_selection:    None,
                    committed_pt_selection: None,
                });
                // Animate camera to face the sketch plane.
                let (az, el) = match raw.plane {
                    SketchPlane::XY => (-PI / 2.0, PI / 2.0 - 0.05),
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
            UiAction::SketchRename(name) => {
                if let Some(sk) = &mut self.sketch {
                    sk.name = name;
                }
                false
            }
            UiAction::SketchSelectConstraint(i) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some(pos) = sk.constraint_selection.iter().position(|&x| x == i) {
                        sk.constraint_selection.remove(pos);
                    } else {
                        sk.constraint_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchPanelSelectSegment(i) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some(pos) = sk.seg_selection.iter().position(|&x| x == i) {
                        sk.seg_selection.remove(pos);
                    } else {
                        sk.seg_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchPanelSelectVertex(i) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some(pos) = sk.pt_selection.iter().position(|&x| x == i) {
                        sk.pt_selection.remove(pos);
                    } else {
                        sk.pt_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchSetTool(t) => {
                if let Some(sk) = &mut self.sketch {
                    sk.active_tool = t;
                    sk.tool_in_progress = None;
                }
                false
            }
            UiAction::SketchAbortActive => {
                if let Some(sk) = &mut self.sketch {
                    sk.tool_in_progress = None;
                    sk.points.clear();
                    sk.closed = false;
                    sk.seg_selection.clear();
                    sk.pt_selection.clear();
                    sk.committed_selection = None;
                    sk.committed_pt_selection = None;
                }
                false
            }
            UiAction::SketchCommitPolyline => {
                if let Some(sk) = &mut self.sketch {
                    if sk.points.len() >= 2 && sk.tool_in_progress.is_none() {
                        sk.history.push(sketch_snapshot(sk));
                        let pts = std::mem::take(&mut sk.points);
                        let constraints = std::mem::take(&mut sk.constraints);
                        sk.closed = false;
                        sk.seg_selection.clear();
                        sk.pt_selection.clear();
                        sk.committed_profiles.push(CommittedProfile {
                            points:      pts,
                            closed:      false,
                            shape:       ProfileShape::Polyline,
                            plane:       None,
                            constraints,
                        });
                    }
                }
                false
            }
            UiAction::SketchSelectCommitted(idx) => {
                if let Some(sk) = &mut self.sketch {
                    sk.committed_selection = idx;
                    sk.committed_pt_selection = None;
                }
                false
            }
            UiAction::SketchSelectCommittedPoint(idx) => {
                if let Some(sk) = &mut self.sketch {
                    sk.committed_pt_selection = idx;
                    sk.committed_selection = None;
                }
                false
            }
            UiAction::SketchAddPoint(p) => {
                let Some(sk) = &mut self.sketch else { return false };
                if sk.closed { return false; }
                if sk.active_tool == DrawTool::Pointer { return false; }
                match sk.active_tool {
                    DrawTool::Pointer => unreachable!(),
                    DrawTool::Polyline => {
                        sk.history.push(sketch_snapshot(sk));
                        sk.points.push(p);
                        apply_constraints(sk);
                    }
                    DrawTool::Arc => {
                        match sk.tool_in_progress.take() {
                            None => {
                                sk.tool_in_progress = Some(ToolInProgress::Arc1 { start: p });
                            }
                            Some(ToolInProgress::Arc1 { start }) => {
                                sk.tool_in_progress = Some(ToolInProgress::Arc2 { start, end_pt: p });
                            }
                            Some(ToolInProgress::Arc2 { start, end_pt }) => {
                                // p = arc center click; project onto perpendicular bisector before storing.
                                let center = project_center_to_arc_bisector(start, end_pt, p, sk.plane);
                                sk.history.push(sketch_snapshot(sk));
                                sk.committed_profiles.push(CommittedProfile {
                                    points:      vec![start, end_pt, center],
                                    closed:      false,
                                    shape:       ProfileShape::Arc,
                                    plane:       Some(sk.plane),
                                    constraints: Vec::new(),
                                });
                            }
                            _ => {}
                        }
                    }
                    DrawTool::Rectangle => {
                        match sk.tool_in_progress.take() {
                            None => {
                                sk.tool_in_progress = Some(ToolInProgress::RectFirst { corner: p });
                            }
                            Some(ToolInProgress::RectFirst { corner }) => {
                                let corners = rect_corners(corner, p, sk.plane);
                                sk.history.push(sketch_snapshot(sk));
                                sk.committed_profiles.push(CommittedProfile {
                                    points:      corners.to_vec(),
                                    closed:      true,
                                    shape:       ProfileShape::Polyline,
                                    plane:       None,
                                    constraints: Vec::new(),
                                });
                            }
                            _ => {}
                        }
                    }
                    DrawTool::Circle => {
                        match sk.tool_in_progress.take() {
                            None => {
                                sk.tool_in_progress = Some(ToolInProgress::CircleCenter { center: p });
                            }
                            Some(ToolInProgress::CircleCenter { center }) => {
                                sk.history.push(sketch_snapshot(sk));
                                sk.committed_profiles.push(CommittedProfile {
                                    points:      vec![center, p],
                                    closed:      true,
                                    shape:       ProfileShape::Circle,
                                    plane:       Some(sk.plane),
                                    constraints: Vec::new(),
                                });
                            }
                            _ => {}
                        }
                    }
                }
                false
            }
            UiAction::SketchUndoPoint => {
                if let Some(sk) = &mut self.sketch {
                    match sk.tool_in_progress.take() {
                        // Step back through multi-click tool state before touching points.
                        Some(ToolInProgress::Arc2 { start, .. }) => {
                            // Step back: forget end_pt, keep start.
                            sk.tool_in_progress = Some(ToolInProgress::Arc1 { start });
                        }
                        Some(_) => {
                            // Arc1, RectFirst, CircleCenter: cancel back to idle.
                        }
                        None => {
                            // No tool in progress: undo the last committed point or profile.
                            sk.history.push(sketch_snapshot(sk));
                            if sk.closed {
                                sk.closed = false;
                            } else if !sk.points.is_empty() {
                                sk.points.pop();
                            } else {
                                // Active profile is empty — remove the last committed profile.
                                sk.committed_profiles.pop();
                            }
                        }
                    }
                }
                false
            }
            UiAction::SketchCloseLoop => {
                // Only close when no multi-step tool is in progress.
                if let Some(sk) = &mut self.sketch {
                    if sk.points.len() >= 3 && sk.tool_in_progress.is_none() {
                        sk.history.push(sketch_snapshot(sk));
                        sk.closed = true;
                    }
                }
                false
            }
            UiAction::SketchFinish => {
                // Save to entries and exit sketch mode.
                let Some(sk) = self.sketch.take() else { return false };
                let has_active = sk.points.len() >= 3;
                if !has_active && sk.committed_profiles.is_empty() {
                    self.sketch = Some(sk);
                    return false;
                }
                // Build the full list of profiles to save.
                let mut extra_profiles = sk.committed_profiles;
                let (primary_points, primary_constraints) = if has_active {
                    let persistent: Vec<_> = sk.constraints.iter()
                        .filter(|c| !matches!(c, SketchConstraint::PointFixed { .. }))
                        .cloned()
                        .collect();
                    (sk.points, persistent)
                } else {
                    // No active profile — promote first committed as primary.
                    let first = extra_profiles.remove(0);
                    (first.points, first.constraints)
                };
                let id = self.alloc_id();
                let name = sk.name.clone();
                self.save_snapshot();
                self.entries.push(SceneEntry::Sketch(RawSketch {
                    name,
                    plane:          sk.plane,
                    points:         primary_points,
                    constraints:    primary_constraints,
                    extra_profiles,
                    id,
                }));
                // Sketch entries are not "selected" (no solid to operate on).
                false
            }
            UiAction::SketchSelectSegment(i) => {
                if let Some(sk) = &mut self.sketch {
                    // Toggle segment (max 2); points are unaffected.
                    if let Some(pos) = sk.seg_selection.iter().position(|&x| x == i) {
                        sk.seg_selection.remove(pos);
                    } else {
                        if sk.seg_selection.len() >= 2 { sk.seg_selection.remove(0); }
                        sk.seg_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchSelectVertex(i) => {
                if let Some(sk) = &mut self.sketch {
                    // Toggle point (max 2); segments are unaffected.
                    if let Some(pos) = sk.pt_selection.iter().position(|&x| x == i) {
                        sk.pt_selection.remove(pos);
                    } else {
                        if sk.pt_selection.len() >= 2 { sk.pt_selection.remove(0); }
                        sk.pt_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchSelectRef(r) => {
                if let Some(sk) = &mut self.sketch {
                    if sk.ref_selection == Some(r) {
                        sk.ref_selection = None;
                    } else {
                        sk.ref_selection = Some(r);
                    }
                }
                false
            }
            UiAction::SketchAddConstraint(c) => {
                if let Some(sk) = &mut self.sketch {
                    sk.history.push(sketch_snapshot(sk));
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
                        sk.history.push(sketch_snapshot(sk));
                        sk.constraints.remove(idx);
                        if !sk.violated_constraints.is_empty() {
                            sk.violated_constraints.remove(idx);
                        }
                        apply_constraints(sk);
                    }
                }
                false
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
        self.history.push(self.entries.clone());
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
            self.entries.push(SceneEntry::Solid(SceneObject { store, solid_id, name, history, id }));
        }
    }

    /// Move a sketch vertex to `target` while respecting constraints.
    ///
    /// Temporarily injects a `PointFixed` pin constraint for the dragged vertex
    /// so the solver holds it at the cursor position while adjusting other free
    /// vertices to satisfy all other constraints.
    pub fn drag_sketch_vertex(&mut self, idx: usize, target: Point3) {
        let Some(sk) = &mut self.sketch else { return };
        if idx >= sk.points.len() { return; }

        let (u, v) = sk.plane.uv_axes();
        let pin_x = target.coords.dot(&u);
        let pin_y = target.coords.dot(&v);

        if !sk.constraints.is_empty() && sk.points.len() >= 2 {
            // Save positions so we can restore if the pin conflicts with existing constraints.
            let pts_backup = sk.points.clone();
            sk.points[idx] = target;
            let pin_c = SketchConstraint::PointFixed { pt: idx, x: pin_x, y: pin_y };
            sk.constraints.push(pin_c);
            apply_constraints(sk);
            let conflict = sk.constraints_conflict;
            sk.constraints.pop();
            if conflict {
                // The cursor position is incompatible with existing constraints — snap back.
                sk.points = pts_backup;
                apply_constraints(sk);
            }
        } else {
            sk.points[idx] = target;
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
        for (i, entry) in self.entries.iter().enumerate() {
            let obj = match entry { SceneEntry::Solid(s) => s, _ => continue };
            let bvh = Bvh::build(&obj.store);
            let hits = bvh.intersect_ray(&ray);
            if !hits.is_empty() {
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

    /// Return (face count, edge count, vertex count) for the selected solid entry.
    pub fn selection_stats(&self) -> Option<(usize, usize, usize)> {
        let idx = self.selection.iter().next()?;
        let obj = self.entries.get(*idx)?.as_solid()?;
        Some((
            obj.store.face_count(),
            obj.store.edge_count(),
            obj.store.vertex_count(),
        ))
    }

    /// Serialize the current scene to a `SavedScene`.
    pub fn to_saved_scene(&self) -> SavedScene {
        let entries = self.entries.iter().map(|e| match e {
            SceneEntry::Sketch(s) => SavedEntry::Sketch {
                name:        s.name.clone(),
                plane:       s.plane,
                points:      s.points.clone(),
                constraints: s.constraints.clone(),
            },
            SceneEntry::Solid(o) => SavedEntry::Solid {
                name:    o.name.clone(),
                history: o.history.clone(),
            },
        }).collect();
        SavedScene {
            version: 2,
            entries,
            camera: SavedCamera {
                target:    self.camera.target.coords.into(),
                distance:  self.camera.distance,
                azimuth:   self.camera.azimuth,
                elevation: self.camera.elevation,
            },
        }
    }

    /// Rebuild the scene from a `SavedScene`, replacing all current entries.
    pub fn load_saved_scene(&mut self, scene: SavedScene) {
        self.entries.clear();
        self.selection.clear();
        self.history = History::new();
        self.sketch = None;
        self.name_counter = 0;

        for saved in scene.entries {
            match saved {
                SavedEntry::Sketch { name, plane, points, constraints } => {
                    let id = { let v = self.next_object_id; self.next_object_id += 1; v };
                    self.entries.push(SceneEntry::Sketch(RawSketch { name, plane, points, constraints, extra_profiles: Vec::new(), id }));
                }
                SavedEntry::Solid { name, history } => {
                    if let Some(obj) = rebuild_solid(name, &history, &mut self.next_object_id) {
                        self.entries.push(SceneEntry::Solid(obj));
                    }
                }
            }
        }

        self.camera.target    = Point3::new(scene.camera.target[0], scene.camera.target[1], scene.camera.target[2]);
        self.camera.distance  = scene.camera.distance;
        self.camera.azimuth   = scene.camera.azimuth;
        self.camera.elevation = scene.camera.elevation;
        self.scene_dirty = true;
    }
}

// ── Saved-scene data types ───────────────────────────────────────────────────

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SavedScene {
    pub version: u32,
    pub entries: Vec<SavedEntry>,
    pub camera:  SavedCamera,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub enum SavedEntry {
    Sketch {
        name:        String,
        plane:       SketchPlane,
        points:      Vec<Point3>,
        constraints: Vec<SketchConstraint>,
    },
    Solid {
        name:    String,
        history: ObjectHistory,
    },
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SavedCamera {
    pub target:    [f64; 3],
    pub distance:  f64,
    pub azimuth:   f64,
    pub elevation: f64,
}

// ── Scene reconstruction ─────────────────────────────────────────────────────

/// Recursively rebuild a solid `SceneObject` from a history tree.
fn rebuild_solid(name: String, history: &ObjectHistory, next_id: &mut u64) -> Option<SceneObject> {
    let id = { let v = *next_id; *next_id += 1; v };
    match history {
        ObjectHistory::Primitive(kind) => {
            let mut store = ShapeStore::new();
            let result = match kind {
                PrimitiveKind::Box      => brep_topo::primitives::make_box(&mut store, 1.0, 1.0, 1.0),
                PrimitiveKind::Cylinder => brep_topo::primitives::make_cylinder(&mut store, 0.5, 1.0),
                PrimitiveKind::Sphere   => brep_topo::primitives::make_sphere(&mut store, 0.5),
                PrimitiveKind::Cone     => brep_topo::primitives::make_cone(&mut store, 0.5, 1.0),
            };
            let solid_id = result.ok()?;
            Some(SceneObject { store, solid_id, name, history: history.clone(), id })
        }
        ObjectHistory::Sketch { plane, points, constraints: _, extrude_dist } => {
            use brep_param::{ExtrudeFeature, Feature};
            use std::sync::Arc;
            if points.len() < 3 { return None; }
            let feature = ExtrudeFeature {
                profile:   points.clone(),
                direction: plane.normal(),
                distance:  *extrude_dist,
            };
            let store_arc = feature.compute(&[]).ok()?;
            let store: ShapeStore = Arc::try_unwrap(store_arc).unwrap_or_else(|a| (*a).clone());
            let solid_id = store.solid_ids().next()?;
            Some(SceneObject { store, solid_id, name, history: history.clone(), id })
        }
        ObjectHistory::Boolean { kind, left, right } => {
            let mut scratch = u64::MAX / 2;
            let left_obj  = rebuild_solid(String::new(), left,  &mut scratch)?;
            let right_obj = rebuild_solid(String::new(), right, &mut scratch)?;
            let result = brep_bool::boolean_op(
                &left_obj.store,  left_obj.solid_id,
                &right_obj.store, right_obj.solid_id,
                *kind, 1e-7,
            );
            let new_store = result.ok()?;
            let solid_id  = new_store.solid_ids().next()?;
            Some(SceneObject { store: new_store, solid_id, name, history: history.clone(), id })
        }
    }
}
