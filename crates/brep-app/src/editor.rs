//! Editor state: scene objects, selection, camera, undo/redo.

use std::f64::consts::PI;
use std::time::Instant;

use brep_algo::bvh::{Bvh, Ray};
use brep_bool::{boolean_op, BooleanKind};
use brep_core::{Point3, Vec3};
use brep_sketch::{
    apply_constraints as sketch_apply, compute_dof, SketchConstraint,
    Constraint as BConstraint, GlobalPointId as BGlobalPointId,
    PointId as BPointId, ProfileId as BProfileId,
};
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

/// A single item in the ordered sketch selection list.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SketchSelection {
    /// Active-profile segment index.
    Segment(usize),
    /// Active-profile vertex index.
    Point(usize),
    /// Reference entity (origin / axis).
    Ref(RefEntity),
    /// Committed profile index (selected as a whole, for radius/PointOnCurve).
    CommittedProfile(usize),
    /// A control point within a committed profile: (profile_idx, vertex_idx).
    CommittedPoint(usize, usize),
    /// A segment within a committed profile: (profile_idx, segment_idx).
    CommittedSegment(usize, usize),
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

/// Intermediate state accumulated by multi-click drawing tools (Arc/Rect/Circle/Polyline).
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
    /// Polyline tool: one or more points have been placed.  Each completed segment
    /// lives in `committed_profiles`; `pen_global_idx` is the last placed point.
    PolylineChain {
        /// Index of the first CommittedProfile belonging to this chain.
        chain_start_profile: usize,
        /// Index into `SketchState::global_points` at which this chain's points begin.
        chain_start_global_pt_idx: usize,
        /// Index into `global_points` of the current "pen" (last placed point).
        pen_global_idx: usize,
        /// Number of segments committed so far (0 = only the start point was placed).
        segment_count: usize,
        /// Length of `history.undo_stack` when this chain began (for abort clean-up).
        chain_start_history_len: usize,
    },
}

/// What a length constraint applies to — used for the modal dialog.
#[derive(Clone, Copy, Debug)]
pub enum LengthTarget {
    /// Constrain a single active-profile segment to a fixed length.
    Segment(usize),
    /// Constrain the distance between two active-profile vertices.
    Points(usize, usize),
    /// Constrain a committed-profile segment to a fixed length (profile_idx, segment_idx).
    CommittedSegment(usize, usize),
    /// Constrain the radius of a committed circle or arc (profile_idx).
    CommittedRadius(usize),
}

/// A geometric constraint that spans two different committed profiles.
/// Stored on `SketchState.cross_constraints`; solved after per-profile constraints.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum CommittedCrossConstraint {
    /// Segment-based constraints: si_a/si_b are local segment indices.
    Parallel      { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize },
    Perpendicular { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize },
    EqualLength   { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize },
    Angle         { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize, degrees: f64 },
    /// Point-based constraints: vi_a/vi_b are local vertex indices.
    HorizontalPair { pi_a: usize, vi_a: usize, pi_b: usize, vi_b: usize,
                     perp_u: f64, perp_v: f64 },
    VerticalPair   { pi_a: usize, vi_a: usize, pi_b: usize, vi_b: usize,
                     perp_u: f64, perp_v: f64 },
    /// Point equidistant (perpendicular distance) from two segments.
    /// `gi_p` is the global-points index of the reference point.
    Symmetric { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize, gi_p: usize },
    /// Three-point symmetry: dist(P_a, P_c) == dist(P_b, P_c).
    /// P_c is the constrained point; P_a and P_b are the reference points.
    /// vi_* are local vertex indices within their respective committed profiles.
    SymmetricPoints { pi_a: usize, vi_a: usize, pi_b: usize, vi_b: usize, pi_c: usize, vi_c: usize },
}

impl CommittedCrossConstraint {
    pub fn label(&self) -> &'static str {
        match self {
            CommittedCrossConstraint::Parallel      { .. } => "Parallel",
            CommittedCrossConstraint::Perpendicular { .. } => "Perpendicular",
            CommittedCrossConstraint::EqualLength   { .. } => "Equal Length",
            CommittedCrossConstraint::Angle         { .. } => "Angle",
            CommittedCrossConstraint::HorizontalPair { .. } => "Horizontal",
            CommittedCrossConstraint::VerticalPair   { .. } => "Vertical",
            CommittedCrossConstraint::Symmetric      { .. } => "Symmetric",
            CommittedCrossConstraint::SymmetricPoints { .. } => "Symmetric (points)",
        }
    }
}

/// What the angle dialog targets.
#[derive(Clone, Copy, Debug)]
pub enum AngleDialogTarget {
    /// Two segments in the active profile.
    ActiveProfile { seg_a: usize, seg_b: usize },
    /// Two segments in different (or same) committed profiles.
    CrossProfile  { pi_a: usize, si_a: usize, pi_b: usize, si_b: usize },
    /// Two segments in the same committed profile (per-profile Angle constraint).
    CommittedProfile { pi: usize, si_a: usize, si_b: usize },
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

pub(crate) use crate::profile_shapes::{
    project_center_to_arc_bisector, rect_corners,
    tessellate_arc_from_center, tessellate_circle,
};

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
    /// Ordered selection list — preserves insertion order so constraint builders can
    /// inspect which item was selected first / last.
    pub selection:            Vec<SketchSelection>,
    /// Set when the last constraint solve did not converge.
    pub constraints_conflict: bool,
    /// Parallel to `constraints`: `true` if removing that constraint would let
    /// the system converge (i.e., it is one of the conflicting constraints).
    /// Empty when there is no conflict.
    pub violated_constraints: Vec<bool>,
    /// Indices of currently selected constraints (for highlighting).
    pub constraint_selection: Vec<usize>,
    /// Indices of currently selected cross-profile constraints (for highlighting).
    pub cross_constraint_selection: Vec<usize>,
    /// Selected per-profile constraints: (profile_idx, constraint_idx within that profile).
    pub committed_constraint_selection: Vec<(usize, usize)>,
    /// Whether the sketch is fully constrained (DOF = 0) after the last solve.
    pub fully_constrained: bool,
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
    /// Flat store of all 3-D points belonging to committed profiles.
    /// `CommittedProfile::point_indices` holds indices into this vec.
    pub global_points: Vec<Point3>,
    /// Constraints that span two different committed profiles.
    pub cross_constraints: Vec<CommittedCrossConstraint>,
}

impl SketchState {
    // ── Selection helpers ──────────────────────────────────────────────────────

    /// Active-profile selected segment indices in insertion order.
    pub fn sel_segs(&self) -> impl Iterator<Item = usize> + '_ {
        self.selection.iter().filter_map(|s| match s {
            SketchSelection::Segment(i) => Some(*i),
            _ => None,
        })
    }

    /// Active-profile selected point indices in insertion order.
    pub fn sel_pts(&self) -> impl Iterator<Item = usize> + '_ {
        self.selection.iter().filter_map(|s| match s {
            SketchSelection::Point(i) => Some(*i),
            _ => None,
        })
    }

    /// Selected reference entity, if any.
    pub fn sel_ref(&self) -> Option<RefEntity> {
        self.selection.iter().find_map(|s| match s {
            SketchSelection::Ref(r) => Some(*r),
            _ => None,
        })
    }

    /// Selected committed profile index, if any.
    pub fn sel_committed_profile(&self) -> Option<usize> {
        self.selection.iter().find_map(|s| match s {
            SketchSelection::CommittedProfile(pi) => Some(*pi),
            _ => None,
        })
    }

    /// Selected committed-profile points as `(profile_idx, vertex_idx)` in insertion order.
    pub fn sel_committed_pts(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.selection.iter().filter_map(|s| match s {
            SketchSelection::CommittedPoint(pi, vi) => Some((*pi, *vi)),
            _ => None,
        })
    }

    /// Selected committed-profile segments as `(profile_idx, segment_idx)` in insertion order.
    pub fn sel_committed_segs(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.selection.iter().filter_map(|s| match s {
            SketchSelection::CommittedSegment(pi, si) => Some((*pi, *si)),
            _ => None,
        })
    }

    /// Toggle `item` in the selection list.  If already present it is removed; otherwise
    /// the oldest item of the same variant is evicted when `count >= max`, then `item` is appended.
    pub(crate) fn toggle_and_cap(&mut self, item: SketchSelection, max: usize) {
        if let Some(pos) = self.selection.iter().position(|s| s == &item) {
            self.selection.remove(pos);
            return;
        }
        // Evict oldest of same discriminant while over cap.
        fn same_variant(a: &SketchSelection, b: &SketchSelection) -> bool {
            std::mem::discriminant(a) == std::mem::discriminant(b)
        }
        while max < usize::MAX
            && self.selection.iter().filter(|s| same_variant(s, &item)).count() >= max
        {
            let oldest = self.selection.iter().position(|s| same_variant(s, &item)).unwrap();
            self.selection.remove(oldest);
        }
        self.selection.push(item);
    }
}

pub use crate::profile_shapes::ProfileShape;

/// A single sub-profile within a sketch (e.g. one circle, one rectangle, or one
/// closed polyline).  A `SketchState` can accumulate many of these alongside the active
/// open profile being drawn.
///
/// During a live sketch session `point_indices` (into `SketchState::global_points`) is
/// the authoritative source; `points` is only populated when serialising to/from disk.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CommittedProfile {
    /// Populated only when serialising / deserialising.  Empty during an active sketch session.
    #[serde(default)]
    pub points:        Vec<Point3>,
    /// Indices into `SketchState::global_points`.  Not serialised; reconstructed on load.
    #[serde(skip)]
    pub point_indices: Vec<usize>,
    pub closed:        bool,
    /// Geometric interpretation of `points`/`point_indices`.  Defaults to `Polyline`.
    #[serde(default)]
    pub shape:         ProfileShape,
    /// The sketch plane this profile lives on (needed for arc tessellation during rendering).
    #[serde(default)]
    pub plane:         Option<SketchPlane>,
    pub constraints:   Vec<SketchConstraint>,
    /// For `ProfileShape::Arc`: whether to sweep the longer arc path instead of the shorter one.
    /// Defaults to `false`; toggled by `UiAction::SketchReverseArc`.
    #[serde(default)]
    pub arc_reversed:  bool,
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
pub(crate) struct SketchSnapshot {
    points:             Vec<Point3>,
    constraints:        Vec<SketchConstraint>,
    closed:             bool,
    committed_profiles: Vec<CommittedProfile>,
    global_points:      Vec<Point3>,
    cross_constraints:  Vec<CommittedCrossConstraint>,
}

/// One entry in the sketch action log: the label displayed to the user and the
/// snapshot of sketch state captured *before* the action was applied.
#[derive(Clone, Debug)]
pub struct SketchHistoryEntry {
    pub label:    String,
    pub snapshot: SketchSnapshot,
}

#[derive(Clone, Debug)]
pub struct SketchHistory {
    pub undo_stack: Vec<SketchHistoryEntry>,
    pub redo_stack: Vec<SketchHistoryEntry>,
}

impl SketchHistory {
    fn new() -> Self { Self { undo_stack: Vec::new(), redo_stack: Vec::new() } }

    pub(crate) fn push(&mut self, label: impl Into<String>, snapshot: SketchSnapshot) {
        self.undo_stack.push(SketchHistoryEntry { label: label.into(), snapshot });
        self.redo_stack.clear();
    }

    fn undo(&mut self, label: impl Into<String>, current: SketchSnapshot) -> Option<SketchSnapshot> {
        let prev = self.undo_stack.pop()?;
        self.redo_stack.push(SketchHistoryEntry { label: label.into(), snapshot: current });
        Some(prev.snapshot)
    }

    fn redo(&mut self, label: impl Into<String>, current: SketchSnapshot) -> Option<SketchSnapshot> {
        let next = self.redo_stack.pop()?;
        self.undo_stack.push(SketchHistoryEntry { label: label.into(), snapshot: current });
        Some(next.snapshot)
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
    /// Toggle-select a vertex by index (max 2 at a time).
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
    /// Toggle-select a cross-profile constraint by index.
    SketchSelectCrossConstraint(usize),
    /// Toggle-select a per-profile constraint by (profile_idx, constraint_idx).
    SketchSelectCommittedConstraint(usize, usize),
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
    /// Select a specific segment within a committed polyline (profile_idx, segment_idx).
    SketchSelectCommittedSeg(Option<(usize, usize)>),
    /// Add a constraint to a specific committed profile and re-solve it.
    SketchAddCommittedConstraint(usize, SketchConstraint),
    /// Remove a constraint from a committed profile by (profile_idx, constraint_idx).
    SketchRemoveCommittedConstraint(usize, usize),
    /// Delete the action-log entry at the given index, restoring the sketch to
    /// the state captured before that action (discards all subsequent actions).
    SketchDeleteHistoryEntry(usize),
    /// Merge two global points: replace every occurrence of `gi_replace` in
    /// `committed_profiles[*].point_indices` with `gi_keep`, then snap the kept
    /// point to the average of the two positions.  Makes cross-profile endpoints
    /// structurally coincident without a separate constraint.
    SketchMergeGlobalPoints { gi_keep: usize, gi_replace: usize },
    /// Add a cross-profile constraint and re-solve.
    SketchAddCrossConstraint(CommittedCrossConstraint),
    /// Remove the cross-profile constraint at the given index.
    SketchRemoveCrossConstraint(usize),
    /// Open the angle-input dialog targeting committed profiles.
    SketchBeginCommittedAngleInput(AngleDialogTarget),
    /// Remove committed profile at the given index, adjusting cross-constraint indices.
    SketchDeleteCommittedProfile(usize),
    /// Flip the arc direction (short ↔ long path) for the committed arc at the given index.
    SketchReverseArc(usize),
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
pub(crate) fn apply_constraints(sk: &mut SketchState) {
    if sk.constraints.is_empty() || sk.points.is_empty() { return; }
    let (u, v) = sk.plane.uv_axes();
    let mut pts2d: Vec<[f64; 2]> = sk.points.iter()
        .map(|p| [p.coords.dot(&u), p.coords.dot(&v)])
        .collect();
    let n = pts2d.len();
    let result = sketch_apply(&mut pts2d, &sk.constraints, n);
    sk.constraints_conflict = result.conflict;
    sk.violated_constraints = result.violated;
    if !result.conflict {
        sk.fully_constrained = compute_dof(&pts2d, &sk.constraints, n) == 0;
        for (i, p) in sk.points.iter_mut().enumerate() {
            *p = Point3::origin() + u * pts2d[i][0] + v * pts2d[i][1];
        }
    } else {
        sk.fully_constrained = false;
    }
}

/// Resolve a committed profile's point indices to a `Vec<Point3>` for read-only use
/// (rendering, hit-testing, constraint building).
pub(crate) fn resolved_points(cp: &CommittedProfile, global_points: &[Point3]) -> Vec<Point3> {
    cp.point_indices.iter().map(|&gi| global_points[gi]).collect()
}

/// Push `pts` into `global_points` and return their (new) indices.
pub(crate) fn push_to_global(pts: &[Point3], global_points: &mut Vec<Point3>) -> Vec<usize> {
    let start = global_points.len();
    global_points.extend_from_slice(pts);
    (start..global_points.len()).collect()
}

// ── brep_sketch bridge ────────────────────────────────────────────────────────

/// Convert an app `SketchPlane` to a `brep_sketch::Plane`.
fn sketch_plane_to_bplane(sp: SketchPlane) -> brep_sketch::Plane {
    match sp {
        SketchPlane::XY => brep_sketch::Plane::xy(),
        SketchPlane::XZ => brep_sketch::Plane::xz(),
        SketchPlane::YZ => brep_sketch::Plane::yz(),
    }
}

/// Translate a single-profile `SketchConstraint` (which uses flat point/segment indices)
/// into a `brep_sketch::Constraint` bound to `pid`.
fn sketch_constraint_to_bconstraint(c: &SketchConstraint, pid: BProfileId) -> Option<BConstraint> {
    Some(match *c {
        SketchConstraint::Horizontal { seg }
            => BConstraint::Horizontal { profile: pid, seg },
        SketchConstraint::Vertical { seg }
            => BConstraint::Vertical { profile: pid, seg },
        SketchConstraint::FixedLength { seg, value }
            => BConstraint::FixedLength { profile: pid, seg, value },
        SketchConstraint::Parallel { seg_a, seg_b }
            => BConstraint::Parallel { profile: pid, seg_a, seg_b },
        SketchConstraint::Perpendicular { seg_a, seg_b }
            => BConstraint::Perpendicular { profile: pid, seg_a, seg_b },
        SketchConstraint::EqualLength { seg_a, seg_b }
            => BConstraint::EqualLength { profile: pid, seg_a, seg_b },
        SketchConstraint::Angle { seg_a, seg_b, degrees }
            => BConstraint::Angle { profile: pid, seg_a, seg_b, degrees },
        SketchConstraint::PointOnOrigin { pt }
            => BConstraint::PointOnOrigin { profile: pid, pt: BPointId(pt) },
        SketchConstraint::PointOnXAxis { pt }
            => BConstraint::PointOnXAxis { profile: pid, pt: BPointId(pt) },
        SketchConstraint::PointOnYAxis { pt }
            => BConstraint::PointOnYAxis { profile: pid, pt: BPointId(pt) },
        SketchConstraint::PointFixed { pt, x, y }
            => BConstraint::PointFixed { profile: pid, pt: BPointId(pt), u: x, v: y },
        SketchConstraint::Coincident { pt_a, pt_b }
            => BConstraint::Coincident { profile: pid, pt_a: BPointId(pt_a), pt_b: BPointId(pt_b) },
        SketchConstraint::PointOnLine { pt, seg }
            => BConstraint::PointOnLine { profile: pid, pt: BPointId(pt), seg },
        SketchConstraint::PointDistance { pt_a, pt_b, value }
            => BConstraint::PointDistance { profile: pid, pt_a: BPointId(pt_a), pt_b: BPointId(pt_b), value },
        SketchConstraint::PointOnCircle { pt, center_u, center_v, radius }
            => BConstraint::PointOnCircle { profile: pid, pt: BPointId(pt), center_u, center_v, radius },
        SketchConstraint::HorizontalPair { pt_a, pt_b, perp_u, perp_v }
            => BConstraint::HorizontalPair { profile_a: pid, pt_a: BPointId(pt_a), profile_b: pid, pt_b: BPointId(pt_b), perp_u, perp_v },
        SketchConstraint::VerticalPair { pt_a, pt_b, perp_u, perp_v }
            => BConstraint::VerticalPair { profile_a: pid, pt_a: BPointId(pt_a), profile_b: pid, pt_b: BPointId(pt_b), perp_u, perp_v },
    })
}

/// Translate a `CommittedCrossConstraint` into a `brep_sketch::Constraint`.
///
/// `pid_map[i]` is the `BProfileId` assigned to `committed_profiles[i]`.
/// `gid_map[gi]` is the `BGlobalPointId` assigned to `global_points[gi]`.
///
/// Returns `None` if any referenced index is out of range.
fn cross_constraint_to_bconstraint(
    cc: &CommittedCrossConstraint,
    pid_map: &[BProfileId],
    gid_map: &[BGlobalPointId],
    bsketch: &brep_sketch::Sketch,
) -> Option<BConstraint> {
    Some(match *cc {
        CommittedCrossConstraint::Parallel { pi_a, si_a, pi_b, si_b } =>
            BConstraint::CrossParallel {
                profile_a: *pid_map.get(pi_a)?, seg_a: si_a,
                profile_b: *pid_map.get(pi_b)?, seg_b: si_b,
            },
        CommittedCrossConstraint::Perpendicular { pi_a, si_a, pi_b, si_b } =>
            BConstraint::CrossPerpendicular {
                profile_a: *pid_map.get(pi_a)?, seg_a: si_a,
                profile_b: *pid_map.get(pi_b)?, seg_b: si_b,
            },
        CommittedCrossConstraint::EqualLength { pi_a, si_a, pi_b, si_b } =>
            BConstraint::CrossEqualLength {
                profile_a: *pid_map.get(pi_a)?, seg_a: si_a,
                profile_b: *pid_map.get(pi_b)?, seg_b: si_b,
            },
        CommittedCrossConstraint::Angle { pi_a, si_a, pi_b, si_b, degrees } =>
            BConstraint::CrossAngle {
                profile_a: *pid_map.get(pi_a)?, seg_a: si_a,
                profile_b: *pid_map.get(pi_b)?, seg_b: si_b,
                degrees,
            },
        CommittedCrossConstraint::HorizontalPair { pi_a, vi_a, pi_b, vi_b, perp_u, perp_v } =>
            BConstraint::HorizontalPair {
                profile_a: *pid_map.get(pi_a)?, pt_a: BPointId(vi_a),
                profile_b: *pid_map.get(pi_b)?, pt_b: BPointId(vi_b),
                perp_u, perp_v,
            },
        CommittedCrossConstraint::VerticalPair { pi_a, vi_a, pi_b, vi_b, perp_u, perp_v } =>
            BConstraint::VerticalPair {
                profile_a: *pid_map.get(pi_a)?, pt_a: BPointId(vi_a),
                profile_b: *pid_map.get(pi_b)?, pt_b: BPointId(vi_b),
                perp_u, perp_v,
            },
        CommittedCrossConstraint::Symmetric { pi_a, si_a, pi_b, si_b, gi_p } => {
            let pa = *pid_map.get(pi_a)?;
            let pb = *pid_map.get(pi_b)?;
            let gid = *gid_map.get(gi_p)?;
            // Find which profile/point has this global ID as its reference.
            let (profile_pt, pt) = find_profile_for_global_id(bsketch, gid, pid_map)?;
            BConstraint::Symmetric {
                profile_seg_a: pa, seg_a: si_a,
                profile_seg_b: pb, seg_b: si_b,
                profile_pt, pt,
            }
        }
        CommittedCrossConstraint::SymmetricPoints { pi_a, vi_a, pi_b, vi_b, pi_c, vi_c } =>
            BConstraint::SymmetricPoints {
                profile_a: *pid_map.get(pi_a)?, pt_a: BPointId(vi_a),
                profile_b: *pid_map.get(pi_b)?, pt_b: BPointId(vi_b),
                profile_c: *pid_map.get(pi_c)?, pt_c: BPointId(vi_c),
            },
    })
}

/// Search `pid_map` profiles in `bsketch` for one that contains a point
/// backed by `gid`.  Returns `(ProfileId, PointId)` of the first match.
fn find_profile_for_global_id(
    bsketch: &brep_sketch::Sketch,
    gid: BGlobalPointId,
    pid_map: &[BProfileId],
) -> Option<(BProfileId, BPointId)> {
    for &pid in pid_map {
        if let Some(prof) = bsketch.profile(pid) {
            for pt_id in prof.point_ids() {
                if prof.global_id(pt_id) == Some(gid) {
                    return Some((pid, pt_id));
                }
            }
        }
    }
    None
}

/// Intermediate result of building a `brep_sketch::Sketch` from `SketchState`.
struct CommittedSketchBuild {
    sketch:  brep_sketch::Sketch,
    pid_map: Vec<BProfileId>,      // committed_profiles[i] → ProfileId
    gid_map: Vec<BGlobalPointId>,  // global_points[i] → GlobalPointId
}

/// Build a `brep_sketch::Sketch` from the committed profiles and global points.
///
/// Each committed profile is created with its points as shared global points.
/// Per-profile constraints are added as `Constraint` variants.
///
/// The active profile (`sk.points` / `sk.constraints`) is **not** included —
/// it is solved separately by `apply_constraints`.
fn build_committed_sketch(sk: &SketchState) -> CommittedSketchBuild {
    let (u, v) = sk.plane.uv_axes();
    let to_uv = |p: Point3| -> (f64, f64) { (p.coords.dot(&u), p.coords.dot(&v)) };

    let mut bsketch = brep_sketch::Sketch::new(sketch_plane_to_bplane(sk.plane), "");

    // Register every global point; keep mapping gi → GlobalPointId.
    let gid_map: Vec<BGlobalPointId> = sk.global_points.iter()
        .map(|&p| { let (pu, pv) = to_uv(p); bsketch.alloc_global_point((pu, pv)) })
        .collect();

    // Register every committed profile; keep mapping pi → ProfileId.
    let mut pid_map: Vec<BProfileId> = Vec::with_capacity(sk.committed_profiles.len());
    for cp in &sk.committed_profiles {
        let pid = match cp.shape {
            ProfileShape::Polyline => bsketch.add_polyline_profile(cp.closed),
            ProfileShape::Circle   => bsketch.alloc_circle_profile(),
            ProfileShape::Arc      => bsketch.alloc_arc_profile(),
        };
        for &gi in &cp.point_indices {
            if let Some(&gid) = gid_map.get(gi) {
                let _ = bsketch.push_shared_point(pid, gid);
            }
        }
        for c in &cp.constraints {
            if let Some(bc) = sketch_constraint_to_bconstraint(c, pid) {
                let _ = bsketch.add_constraint(bc);
            }
        }
        pid_map.push(pid);
    }

    CommittedSketchBuild { sketch: bsketch, pid_map, gid_map }
}

/// Solve all committed-profile constraints and cross-profile constraints,
/// writing updated positions back to `sk.global_points`.
///
/// This replaces `apply_committed_profile_constraints` +
/// `apply_cross_constraints` with a unified call into `brep_sketch::Sketch`.
pub(crate) fn apply_all_constraints(sk: &mut SketchState) {
    if sk.committed_profiles.is_empty() { return; }

    let mut build = build_committed_sketch(sk);

    // Add cross-profile constraints.
    let cross = sk.cross_constraints.clone();
    for cc in &cross {
        if let Some(bc) = cross_constraint_to_bconstraint(
            cc, &build.pid_map, &build.gid_map, &build.sketch,
        ) {
            let _ = build.sketch.add_constraint(bc);
        }
    }

    let report = build.sketch.solve();

    // Even if not fully converged, write back whatever the solver produced
    // for the global points that did converge.
    let (u, v) = sk.plane.uv_axes();
    let from_uv = |pu: f64, pv: f64| -> Point3 { Point3::origin() + u * pu + v * pv };

    for (gi, &gid) in build.gid_map.iter().enumerate() {
        if let Ok(uv) = build.sketch.global_point_uv(gid) {
            sk.global_points[gi] = from_uv(uv[0], uv[1]);
        }
    }

    // For profiles whose points aren't all global (e.g. future private-point
    // profiles), also write back directly — currently all points are shared,
    // so this is a no-op but kept for safety.
    for (pi, cp) in sk.committed_profiles.iter().enumerate() {
        let pid = build.pid_map[pi];
        if let Some(prof) = build.sketch.profile(pid) {
            for (local_idx, &gi) in cp.point_indices.iter().enumerate() {
                if let Some((pu, pv)) = prof.point(BPointId(local_idx)) {
                    // Only update if this point has no GlobalPointId (private point).
                    if prof.global_id(BPointId(local_idx)).is_none() {
                        if gi < sk.global_points.len() {
                            sk.global_points[gi] = from_uv(pu, pv);
                        }
                    }
                }
            }
        }
    }

    if !report.converged {
        sk.constraints_conflict = true;
        sk.fully_constrained = false;
    } else {
        sk.fully_constrained = report.dof == Some(0);
    }
}

/// Attempt to assemble a simple head-to-tail chain of 2-point Polyline profiles into
/// a single ordered point list (suitable for extrusion).  Returns an empty Vec if the
/// profiles cannot be arranged into an unambiguous linear or closed chain.
pub(crate) fn assemble_polyline_chain(profiles: &[CommittedProfile]) -> Vec<Point3> {
    // Collect only 2-point Polyline segments that have their serialised points populated.
    let segs: Vec<&CommittedProfile> = profiles.iter()
        .filter(|cp| cp.shape == ProfileShape::Polyline && cp.points.len() == 2)
        .collect();
    if segs.len() < 2 { return Vec::new(); }

    // Find a start segment: one whose start point is not the end point of any other segment.
    let start_idx = segs.iter().position(|s| {
        !segs.iter().any(|o| std::ptr::eq(*o, *s) == false
            && (o.points[1] - s.points[0]).norm() < 1e-9)
    });
    let Some(start) = start_idx else { return Vec::new(); };

    let mut ordered: Vec<&CommittedProfile> = vec![segs[start]];
    let mut used = vec![false; segs.len()];
    used[start] = true;
    loop {
        let last_end = ordered.last().unwrap().points[1];
        let next = segs.iter().enumerate().find(|(i, s)| {
            !used[*i] && (s.points[0] - last_end).norm() < 1e-9
        });
        match next {
            Some((i, s)) => { used[i] = true; ordered.push(s); }
            None => break,
        }
    }
    if ordered.len() != segs.len() { return Vec::new(); }
    let mut pts: Vec<Point3> = ordered.iter().map(|s| s.points[0]).collect();
    pts.push(ordered.last().unwrap().points[1]);
    pts
}

pub(crate) fn sketch_snapshot(sk: &SketchState) -> SketchSnapshot {
    SketchSnapshot {
        points:             sk.points.clone(),
        constraints:        sk.constraints.clone(),
        closed:             sk.closed,
        committed_profiles: sk.committed_profiles.clone(),
        global_points:      sk.global_points.clone(),
        cross_constraints:  sk.cross_constraints.clone(),
    }
}

/// Short human-readable label for a constraint type, used in the action log.
pub(crate) fn constraint_action_label(c: &SketchConstraint) -> &'static str {
    match c {
        SketchConstraint::Parallel { .. }       => "Parallel",
        SketchConstraint::Perpendicular { .. }  => "Perpendicular",
        SketchConstraint::Angle { .. }          => "Angle",
        SketchConstraint::Horizontal { .. }     => "Horizontal",
        SketchConstraint::Vertical { .. }       => "Vertical",
        SketchConstraint::EqualLength { .. }    => "Equal length",
        SketchConstraint::Coincident { .. }     => "Coincident",
        SketchConstraint::PointOnLine { .. }    => "Coincident",
        SketchConstraint::FixedLength { .. }    => "Fixed length",
        SketchConstraint::PointDistance { .. }  => "Point distance",
        SketchConstraint::PointFixed { .. }     => "Fixed point",
        SketchConstraint::PointOnOrigin { .. }  => "On origin",
        SketchConstraint::PointOnXAxis { .. }   => "On X-axis",
        SketchConstraint::PointOnYAxis { .. }   => "On Y-axis",
        SketchConstraint::HorizontalPair { .. } => "Horizontal pair",
        SketchConstraint::VerticalPair { .. }   => "Vertical pair",
        SketchConstraint::PointOnCircle { .. }  => "Coincident",
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

    /// Construct a blank editor state with no entries and no history.
    #[cfg(test)]
    pub fn new_empty() -> Self {
        Self {
            entries: Vec::new(),
            selection: std::collections::HashSet::new(),
            camera: ViewportCamera::default(),
            history: History::new(),
            name_counter: 0,
            next_object_id: 0,
            scene_dirty: false,
            camera_anim: None,
            sketch: None,
        }
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
                        let label = sk.history.undo_stack.last().map(|e| e.label.clone()).unwrap_or_default();
                        let current = sketch_snapshot(sk);
                        if let Some(prev) = sk.history.undo(label, current) {
                            sk.points              = prev.points;
                            sk.constraints         = prev.constraints;
                            sk.closed              = prev.closed;
                            sk.committed_profiles  = prev.committed_profiles;
                            sk.global_points       = prev.global_points;
                            sk.cross_constraints   = prev.cross_constraints;
                            sk.tool_in_progress    = None;
                            apply_constraints(sk);
                            apply_all_constraints(sk);
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
                        let label = sk.history.redo_stack.last().map(|e| e.label.clone()).unwrap_or_default();
                        let current = sketch_snapshot(sk);
                        if let Some(next) = sk.history.redo(label, current) {
                            sk.points              = next.points;
                            sk.constraints         = next.constraints;
                            sk.closed              = next.closed;
                            sk.committed_profiles  = next.committed_profiles;
                            sk.global_points       = next.global_points;
                            sk.cross_constraints   = next.cross_constraints;
                            sk.tool_in_progress    = None;
                            apply_constraints(sk);
                            apply_all_constraints(sk);
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
                    selection:   Vec::new(),
                    constraints_conflict: false,
                    violated_constraints: Vec::new(),
                    constraint_selection: Vec::new(),
                    cross_constraint_selection: Vec::new(),
                    committed_constraint_selection: Vec::new(),
                    fully_constrained:  false,
                    history:            SketchHistory::new(),
                    active_tool:        DrawTool::Pointer,
                    tool_in_progress:   None,
                    committed_profiles: Vec::new(),
                    global_points:      Vec::new(),
                    cross_constraints:  Vec::new(),
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
                // Rebuild global_points from serialised profile points and set point_indices.
                let mut global_points: Vec<Point3> = Vec::new();
                let committed_profiles: Vec<_> = raw.extra_profiles.into_iter()
                    .map(|mut cp| {
                        if cp.plane.is_none() { cp.plane = Some(plane); }
                        // Migrate: move owned points into global store.
                        cp.point_indices = push_to_global(&cp.points, &mut global_points);
                        cp.points.clear();
                        cp
                    })
                    .collect();
                self.sketch = Some(SketchState {
                    name:                 raw.name,
                    plane,
                    points:               raw.points,
                    closed:               true,
                    constraints:          raw.constraints,
                    selection:            Vec::new(),
                    constraints_conflict: false,
                    violated_constraints: Vec::new(),
                    constraint_selection: Vec::new(),
                    cross_constraint_selection: Vec::new(),
                    committed_constraint_selection: Vec::new(),
                    fully_constrained:    false,
                    history:              SketchHistory::new(),
                    active_tool:          DrawTool::Pointer,
                    tool_in_progress:     None,
                    committed_profiles,
                    global_points,
                    cross_constraints:      Vec::new(),
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
            UiAction::SketchSelectCrossConstraint(i) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some(pos) = sk.cross_constraint_selection.iter().position(|&x| x == i) {
                        sk.cross_constraint_selection.remove(pos);
                    } else {
                        sk.cross_constraint_selection.push(i);
                    }
                }
                false
            }
            UiAction::SketchSelectCommittedConstraint(pi, ci) => {
                if let Some(sk) = &mut self.sketch {
                    let key = (pi, ci);
                    if let Some(pos) = sk.committed_constraint_selection.iter().position(|&x| x == key) {
                        sk.committed_constraint_selection.remove(pos);
                    } else {
                        sk.committed_constraint_selection.push(key);
                    }
                }
                false
            }
            UiAction::SketchPanelSelectSegment(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.toggle_and_cap(SketchSelection::Segment(i), usize::MAX);
                }
                false
            }
            UiAction::SketchPanelSelectVertex(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.toggle_and_cap(SketchSelection::Point(i), usize::MAX);
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
                    let has_work = sk.tool_in_progress.is_some() || !sk.points.is_empty();
                    if has_work {
                        // If aborting a polyline chain, remove all profiles and global points
                        // added since the chain started.
                        if let Some(ToolInProgress::PolylineChain {
                            chain_start_profile, chain_start_global_pt_idx,
                            chain_start_history_len, ..
                        }) = sk.tool_in_progress {
                            sk.committed_profiles.truncate(chain_start_profile);
                            sk.global_points.truncate(chain_start_global_pt_idx);
                            sk.history.undo_stack.truncate(chain_start_history_len);
                            sk.history.redo_stack.clear();
                        }
                        sk.tool_in_progress = None;
                        sk.points.clear();
                        sk.closed = false;
                        // Clear everything except Ref (matches previous behaviour where
                        // Ref was not cleared in the has-work branch).
                        sk.selection.retain(|s| matches!(s, SketchSelection::Ref(_)));
                    } else {
                        // Nothing in progress — deselect everything and ensure pointer tool.
                        sk.active_tool = DrawTool::Pointer;
                        sk.selection.clear();
                        sk.constraint_selection.clear();
                        sk.cross_constraint_selection.clear();
                        sk.committed_constraint_selection.clear();
                    }
                }
                false
            }
            UiAction::SketchCommitPolyline => {
                if let Some(sk) = &mut self.sketch {
                    // Per-segment chain: just end the chain (segments already committed).
                    if matches!(
                        sk.tool_in_progress,
                        Some(ToolInProgress::PolylineChain { segment_count, .. }) if segment_count > 0
                    ) {
                        sk.tool_in_progress = None;
                    } else if sk.points.len() >= 2 && sk.tool_in_progress.is_none() {
                        // Legacy active-profile polyline.
                        sk.history.push("Commit polyline", sketch_snapshot(sk));
                        let pts = std::mem::take(&mut sk.points);
                        let constraints = std::mem::take(&mut sk.constraints);
                        sk.closed = false;
                        sk.selection.retain(|s| !matches!(s, SketchSelection::Segment(_) | SketchSelection::Point(_)));
                        let point_indices = push_to_global(&pts, &mut sk.global_points);
                        sk.committed_profiles.push(CommittedProfile {
                            points:        Vec::new(),
                            point_indices,
                            closed:        false,
                            shape:         ProfileShape::Polyline,
                            plane:         None,
                            constraints,
                            arc_reversed:  false,
                        });
                    }
                }
                false
            }
            UiAction::SketchSelectCommitted(idx) => {
                if let Some(sk) = &mut self.sketch {
                    sk.selection.retain(|s| !matches!(
                        s, SketchSelection::CommittedProfile(_)
                         | SketchSelection::CommittedPoint(..)
                         | SketchSelection::CommittedSegment(..)
                    ));
                    if let Some(pi) = idx {
                        sk.selection.push(SketchSelection::CommittedProfile(pi));
                    }
                }
                false
            }
            UiAction::SketchSelectCommittedPoint(idx) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some((pi, vi)) = idx {
                        // Toggle: cap at 3 (supports 3-point symmetry), clear profile selection.
                        sk.toggle_and_cap(SketchSelection::CommittedPoint(pi, vi), 3);
                        sk.selection.retain(|s| !matches!(s, SketchSelection::CommittedProfile(_)));
                    } else {
                        sk.selection.retain(|s| !matches!(
                            s, SketchSelection::CommittedPoint(..)
                             | SketchSelection::CommittedSegment(..)
                        ));
                    }
                }
                false
            }
            UiAction::SketchSelectCommittedSeg(idx) => {
                if let Some(sk) = &mut self.sketch {
                    if let Some((pi, si)) = idx {
                        // Toggle: cap at 2, clear profile selection.
                        sk.toggle_and_cap(SketchSelection::CommittedSegment(pi, si), 2);
                        sk.selection.retain(|s| !matches!(s, SketchSelection::CommittedProfile(_)));
                    } else {
                        sk.selection.retain(|s| !matches!(
                            s, SketchSelection::CommittedSegment(..)
                             | SketchSelection::CommittedPoint(..)
                        ));
                    }
                }
                false
            }
            UiAction::SketchAddPoint(p) => {
                let Some(sk) = &mut self.sketch else { return false };
                if sk.closed { return false; }
                if sk.active_tool == DrawTool::Pointer { return false; }
                crate::sketch_tools::add_point(sk, p);
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
                        Some(ToolInProgress::PolylineChain {
                            chain_start_profile,
                            chain_start_global_pt_idx,
                            pen_global_idx,
                            segment_count,
                            chain_start_history_len,
                        }) => {
                            if segment_count == 0 {
                                // Only the start point was placed — remove it.
                                sk.global_points.truncate(chain_start_global_pt_idx);
                                // tool_in_progress already taken (None).
                            } else {
                                // Remove the last segment and its endpoint.
                                let prev_pen = sk.committed_profiles.last()
                                    .and_then(|cp| cp.point_indices.first().copied())
                                    .unwrap_or(pen_global_idx);
                                sk.committed_profiles.pop();
                                // Remove the end point that was added for this segment.
                                sk.global_points.pop();
                                sk.tool_in_progress = Some(ToolInProgress::PolylineChain {
                                    chain_start_profile,
                                    chain_start_global_pt_idx,
                                    pen_global_idx: prev_pen,
                                    segment_count: segment_count - 1,
                                    chain_start_history_len,
                                });
                            }
                        }
                        Some(_) => {
                            // Arc1, RectFirst, CircleCenter: cancel back to idle.
                        }
                        None => {
                            // No tool in progress: undo the last committed point or profile.
                            sk.history.push("Undo point", sketch_snapshot(sk));
                            if sk.closed {
                                sk.closed = false;
                            } else if !sk.points.is_empty() {
                                sk.points.pop();
                            } else {
                                // Active profile is empty — remove the last committed profile.
                                if let Some(cp) = sk.committed_profiles.pop() {
                                    // If the removed profile owned the tail of global_points, shrink it.
                                    if let Some(&last_gi) = cp.point_indices.last() {
                                        // Only trim if these indices are at the tail and not shared.
                                        let shared = sk.committed_profiles.iter()
                                            .any(|o| o.point_indices.iter().any(|&gi| gi == last_gi));
                                        if !shared && last_gi + 1 == sk.global_points.len() {
                                            sk.global_points.pop();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                false
            }
            UiAction::SketchCloseLoop => {
                if let Some(sk) = &mut self.sketch {
                    if let Some(ToolInProgress::PolylineChain {
                        chain_start_profile,
                        chain_start_global_pt_idx,
                        pen_global_idx,
                        segment_count,
                        chain_start_history_len,
                    }) = sk.tool_in_progress.take() {
                        if segment_count >= 2 {
                            sk.history.push("Close loop", sketch_snapshot(sk));
                            let chain_start_pt_idx =
                                sk.committed_profiles[chain_start_profile].point_indices[0];
                            sk.committed_profiles.push(CommittedProfile {
                                points:        Vec::new(),
                                point_indices: vec![pen_global_idx, chain_start_pt_idx],
                                closed:        false,
                                shape:         ProfileShape::Polyline,
                                plane:         Some(sk.plane),
                                constraints:   Vec::new(),
                                arc_reversed:  false,
                            });
                            // tool_in_progress = None (already taken).
                        } else {
                            // Not enough segments to close — put state back.
                            sk.tool_in_progress = Some(ToolInProgress::PolylineChain {
                                chain_start_profile,
                                chain_start_global_pt_idx,
                                pen_global_idx,
                                segment_count,
                                chain_start_history_len,
                            });
                        }
                    } else if sk.points.len() >= 3 && sk.tool_in_progress.is_none() {
                        // Legacy path: close the active profile.
                        sk.history.push("Close loop", sketch_snapshot(sk));
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
                // Resolve global_points back into CommittedProfile.points for serialisation.
                let mut serialisable: Vec<CommittedProfile> = sk.committed_profiles.iter()
                    .map(|cp| {
                        let mut out = cp.clone();
                        out.points = resolved_points(cp, &sk.global_points);
                        out.point_indices.clear();
                        out
                    })
                    .collect();
                let (primary_points, primary_constraints) = if has_active {
                    let persistent: Vec<_> = sk.constraints.iter()
                        .filter(|c| !matches!(c, SketchConstraint::PointFixed { .. }))
                        .cloned()
                        .collect();
                    (sk.points, persistent)
                } else {
                    // No active profile — try to find a contiguous polyline chain first.
                    let chain = assemble_polyline_chain(&serialisable);
                    if chain.len() >= 3 {
                        // Remove all segments that form a simple chain and use assembled points.
                        serialisable.retain(|cp| cp.shape != ProfileShape::Polyline || cp.points.len() != 2);
                        (chain, Vec::new())
                    } else {
                        // Promote first committed as primary.
                        let first = serialisable.remove(0);
                        (first.points, first.constraints)
                    }
                };
                let id = self.alloc_id();
                let name = sk.name.clone();
                self.save_snapshot();
                self.entries.push(SceneEntry::Sketch(RawSketch {
                    name,
                    plane:          sk.plane,
                    points:         primary_points,
                    constraints:    primary_constraints,
                    extra_profiles: serialisable,
                    id,
                }));
                // Sketch entries are not "selected" (no solid to operate on).
                false
            }
            UiAction::SketchSelectSegment(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.toggle_and_cap(SketchSelection::Segment(i), 2);
                }
                false
            }
            UiAction::SketchSelectVertex(i) => {
                if let Some(sk) = &mut self.sketch {
                    sk.toggle_and_cap(SketchSelection::Point(i), 2);
                }
                false
            }
            UiAction::SketchSelectRef(r) => {
                if let Some(sk) = &mut self.sketch {
                    let already = sk.sel_ref() == Some(r);
                    sk.selection.retain(|s| !matches!(s, SketchSelection::Ref(_)));
                    if !already { sk.selection.push(SketchSelection::Ref(r)); }
                }
                false
            }
            UiAction::SketchAddConstraint(c) => {
                if let Some(sk) = &mut self.sketch {
                    let label = constraint_action_label(&c);
                    sk.history.push(label, sketch_snapshot(sk));
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
                        sk.history.push("Remove constraint", sketch_snapshot(sk));
                        sk.constraints.remove(idx);
                        if !sk.violated_constraints.is_empty() {
                            sk.violated_constraints.remove(idx);
                        }
                        apply_constraints(sk);
                    }
                }
                false
            }
            UiAction::SketchAddCommittedConstraint(pi, c) => {
                if let Some(sk) = &mut self.sketch {
                    if pi < sk.committed_profiles.len() {
                        let label = constraint_action_label(&c);
                        sk.history.push(label, sketch_snapshot(sk));
                        sk.committed_profiles[pi].constraints.push(c);
                        apply_all_constraints(sk);
                    }
                }
                false
            }
            UiAction::SketchRemoveCommittedConstraint(pi, ci) => {
                if let Some(sk) = &mut self.sketch {
                    let valid = sk.committed_profiles.get(pi)
                        .map_or(false, |cp| ci < cp.constraints.len());
                    if valid {
                        sk.history.push("Remove constraint", sketch_snapshot(sk));
                        sk.committed_profiles[pi].constraints.remove(ci);
                    }
                }
                false
            }
            UiAction::SketchMergeGlobalPoints { gi_keep, gi_replace } => {
                if let Some(sk) = &mut self.sketch {
                    if gi_keep < sk.global_points.len() && gi_replace < sk.global_points.len()
                        && gi_keep != gi_replace
                    {
                        sk.history.push("Merge points", sketch_snapshot(sk));
                        // Snap gi_keep to the average of the two positions.
                        let p_keep    = sk.global_points[gi_keep];
                        let p_replace = sk.global_points[gi_replace];
                        sk.global_points[gi_keep] = Point3::origin()
                            + (p_keep.coords + p_replace.coords) * 0.5;
                        // Repoint every profile that referenced gi_replace to gi_keep.
                        for cp in &mut sk.committed_profiles {
                            for gi in &mut cp.point_indices {
                                if *gi == gi_replace { *gi = gi_keep; }
                            }
                        }
                        sk.selection.retain(|s| !matches!(s, SketchSelection::CommittedPoint(..)));
                        apply_constraints(sk);
                    }
                }
                true
            }
            UiAction::SketchAddCrossConstraint(cc) => {
                if let Some(sk) = &mut self.sketch {
                    sk.history.push(cc.label(), sketch_snapshot(sk));
                    sk.cross_constraints.push(cc);
                    sk.selection.retain(|s| !matches!(s, SketchSelection::CommittedSegment(..)));
                    apply_all_constraints(sk);
                }
                false
            }
            UiAction::SketchRemoveCrossConstraint(idx) => {
                if let Some(sk) = &mut self.sketch {
                    if idx < sk.cross_constraints.len() {
                        sk.history.push("Remove constraint", sketch_snapshot(sk));
                        sk.cross_constraints.remove(idx);
                    }
                }
                false
            }
            UiAction::SketchBeginCommittedAngleInput(_) => {
                // Handled in main.rs dispatch_action; should not reach here.
                false
            }
            UiAction::SketchDeleteCommittedProfile(pi) => {
                let pi = pi;
                if let Some(sk) = &mut self.sketch {
                    if pi < sk.committed_profiles.len() {
                        sk.history.push("Delete profile", sketch_snapshot(sk));
                        sk.committed_profiles.remove(pi);
                        // Remove cross-constraints referencing pi; shift indices > pi.
                        sk.cross_constraints.retain(|cc| {
                            match cc {
                                CommittedCrossConstraint::SymmetricPoints { pi_a, pi_b, pi_c, .. } =>
                                    *pi_a != pi && *pi_b != pi && *pi_c != pi,
                                CommittedCrossConstraint::Parallel        { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Perpendicular { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::EqualLength   { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Angle         { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::HorizontalPair { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::VerticalPair   { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Symmetric      { pi_a, pi_b, .. } =>
                                    *pi_a != pi && *pi_b != pi,
                            }
                        });
                        for cc in &mut sk.cross_constraints {
                            match cc {
                                CommittedCrossConstraint::SymmetricPoints { pi_a, pi_b, pi_c, .. } => {
                                    if *pi_a > pi { *pi_a -= 1; }
                                    if *pi_b > pi { *pi_b -= 1; }
                                    if *pi_c > pi { *pi_c -= 1; }
                                }
                                CommittedCrossConstraint::Parallel        { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Perpendicular { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::EqualLength   { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Angle         { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::HorizontalPair { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::VerticalPair   { pi_a, pi_b, .. }
                                | CommittedCrossConstraint::Symmetric      { pi_a, pi_b, .. } => {
                                    if *pi_a > pi { *pi_a -= 1; }
                                    if *pi_b > pi { *pi_b -= 1; }
                                }
                            }
                        }
                        sk.selection.retain(|s| !matches!(
                            s, SketchSelection::CommittedProfile(_)
                             | SketchSelection::CommittedPoint(..)
                             | SketchSelection::CommittedSegment(..)
                        ));
                    }
                }
                true
            }
            UiAction::SketchReverseArc(pi) => {
                if let Some(sk) = &mut self.sketch {
                    if pi < sk.committed_profiles.len() {
                        let snap = sketch_snapshot(sk);
                        sk.history.push("Reverse arc", snap);
                        sk.committed_profiles[pi].arc_reversed = !sk.committed_profiles[pi].arc_reversed;
                    }
                }
                true
            }
            UiAction::SketchDeleteHistoryEntry(idx) => {
                if let Some(sk) = &mut self.sketch {
                    if idx < sk.history.undo_stack.len() {
                        let snap = sk.history.undo_stack[idx].snapshot.clone();
                        sk.history.undo_stack.truncate(idx);
                        sk.history.redo_stack.clear();
                        sk.points             = snap.points;
                        sk.constraints        = snap.constraints;
                        sk.closed             = snap.closed;
                        sk.committed_profiles = snap.committed_profiles;
                        sk.global_points      = snap.global_points;
                        sk.cross_constraints  = snap.cross_constraints;
                        sk.tool_in_progress   = None;
                        apply_constraints(sk);
                        apply_all_constraints(sk);
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

        if !sk.constraints.is_empty() && !sk.points.is_empty() {
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

    /// Move a committed profile vertex to `target`, then re-apply that profile's
    /// constraints (and cross-profile constraints) so dragging respects constraints.
    pub fn drag_committed_vertex(&mut self, pi: usize, vi: usize, target: Point3) {
        let Some(sk) = &mut self.sketch else { return };
        let Some(cp) = sk.committed_profiles.get(pi) else { return };
        let plane = cp.plane.unwrap_or(sk.plane);
        let cp_shape  = cp.shape.clone();
        let cp_indices = cp.point_indices.clone();
        let has_constraints = !sk.committed_profiles[pi].constraints.is_empty();

        // Move the dragged point and let the shape logic adjust any dependent points.
        let mut pts: Vec<Point3> = cp_indices.iter().map(|&gi| sk.global_points[gi]).collect();
        cp_shape.apply_vertex_drag(&mut pts, vi, target, Some(plane));
        for (&gi, &p) in cp_indices.iter().zip(pts.iter()) {
            sk.global_points[gi] = p;
        }

        if has_constraints {
            // Inject a PointFixed pin so the solver holds the dragged vertex at the
            // cursor while adjusting other vertices to satisfy remaining constraints.
            let (u, v) = plane.uv_axes();
            let pin_x = target.coords.dot(&u);
            let pin_y = target.coords.dot(&v);
            let pin = SketchConstraint::PointFixed { pt: vi, x: pin_x, y: pin_y };
            sk.committed_profiles[pi].constraints.push(pin);
            apply_all_constraints(sk);
            sk.committed_profiles[pi].constraints.pop();
        } else {
            apply_all_constraints(sk);
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

#[cfg(test)]
mod tests {
    use super::*;

    // ── Undo / Redo ───────────────────────────────────────────────────────────

    #[test]
    fn undo_restores_entry_count() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Box));
        assert_eq!(ed.entries.len(), 1);
        ed.apply(UiAction::Undo);
        assert_eq!(ed.entries.len(), 0);
    }

    #[test]
    fn redo_reapplies_after_undo() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Cylinder));
        ed.apply(UiAction::Undo);
        assert_eq!(ed.entries.len(), 0);
        ed.apply(UiAction::Redo);
        assert_eq!(ed.entries.len(), 1);
    }

    #[test]
    fn undo_with_empty_history_returns_false() {
        let mut ed = EditorState::new_empty();
        assert!(!ed.apply(UiAction::Undo));
    }

    // ── Selection ─────────────────────────────────────────────────────────────

    #[test]
    fn select_object_sets_selection() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Box));
        ed.apply(UiAction::SelectObject(0));
        assert!(ed.selection.contains(&0));
        assert_eq!(ed.selection.len(), 1);
    }

    #[test]
    fn toggle_select_adds_then_removes() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Box));
        ed.apply(UiAction::ToggleSelectObject(0));
        assert!(ed.selection.contains(&0));
        ed.apply(UiAction::ToggleSelectObject(0));
        assert!(!ed.selection.contains(&0));
    }

    #[test]
    fn delete_selected_removes_entries() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Box));
        ed.apply(UiAction::SelectObject(0));
        ed.apply(UiAction::DeleteSelected);
        assert_eq!(ed.entries.len(), 0);
        assert!(ed.selection.is_empty());
    }

    #[test]
    fn delete_with_no_selection_is_noop() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::AddPrimitive(PrimitiveKind::Sphere));
        let changed = ed.apply(UiAction::DeleteSelected);
        assert!(!changed);
        assert_eq!(ed.entries.len(), 1);
    }

    // ── Sketch lifecycle ──────────────────────────────────────────────────────

    #[test]
    fn enter_sketch_sets_plane() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(SketchPlane::XY));
        assert!(ed.sketch.is_some());
        assert_eq!(ed.sketch.as_ref().unwrap().plane, SketchPlane::XY);
    }

    #[test]
    fn exit_sketch_clears_sketch_state() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(SketchPlane::XZ));
        ed.apply(UiAction::ExitSketch);
        assert!(ed.sketch.is_none());
    }

    #[test]
    fn set_tool_changes_active_tool() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(SketchPlane::XY));
        ed.apply(UiAction::SketchSetTool(DrawTool::Polyline));
        assert_eq!(ed.sketch.as_ref().unwrap().active_tool, DrawTool::Polyline);
        ed.apply(UiAction::SketchSetTool(DrawTool::Circle));
        assert_eq!(ed.sketch.as_ref().unwrap().active_tool, DrawTool::Circle);
    }

    #[test]
    fn sketch_rename_updates_name() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(SketchPlane::YZ));
        ed.apply(UiAction::SketchRename("MySketch".to_string()));
        assert_eq!(ed.sketch.as_ref().unwrap().name, "MySketch");
    }

    // ── Sketch undo ───────────────────────────────────────────────────────────

    #[test]
    fn sketch_constraint_is_undoable() {
        let mut ed = EditorState::new_empty();
        ed.apply(UiAction::EnterSketch(SketchPlane::XY));
        // Place two points to create a segment.
        ed.apply(UiAction::SketchAddPoint(Point3::new(0.0, 0.0, 0.0)));
        ed.apply(UiAction::SketchAddPoint(Point3::new(1.0, 0.1, 0.0)));
        // Add a constraint.
        ed.apply(UiAction::SketchAddConstraint(SketchConstraint::Horizontal { seg: 0 }));
        assert_eq!(ed.sketch.as_ref().unwrap().constraints.len(), 1);
        // Undo inside the sketch: constraint should be removed.
        ed.apply(UiAction::Undo);
        assert_eq!(ed.sketch.as_ref().unwrap().constraints.len(), 0);
    }

    #[test]
    fn arc_center_on_origin_holds_during_drag() {
        let mut ed = EditorState::new_empty();

        // Enter sketch and switch to arc tool.
        ed.apply(UiAction::EnterSketch(SketchPlane::XY));
        ed.apply(UiAction::SketchSetTool(DrawTool::Arc));

        // Draw an arc with 3 clicks:
        //   start (-1, 0), end (1, 0), third point (0, 1) which gets projected
        //   onto the perpendicular bisector of the chord → center near (0, ~0.5).
        ed.apply(UiAction::SketchAddPoint(Point3::new(-1.0, 0.0, 0.0)));
        ed.apply(UiAction::SketchAddPoint(Point3::new( 1.0, 0.0, 0.0)));
        ed.apply(UiAction::SketchAddPoint(Point3::new( 0.0, 1.0, 0.0)));

        assert_eq!(
            ed.sketch.as_ref().unwrap().committed_profiles.len(), 1,
            "arc should be committed after 3 clicks"
        );

        // Apply PointOnOrigin to the center (vi=2).
        ed.apply(UiAction::SketchAddCommittedConstraint(
            0,
            SketchConstraint::PointOnOrigin { pt: 2 },
        ));

        // Verify center snapped to origin.
        {
            let sk = ed.sketch.as_ref().unwrap();
            let pts = resolved_points(&sk.committed_profiles[0], &sk.global_points);
            assert!(
                pts[2].coords.norm() < 1e-6,
                "center should be at origin after constraint, got {:?}", pts[2]
            );
        }

        // Drag the start handle (vi=0) to a new position.
        ed.drag_committed_vertex(0, 0, Point3::new(-2.0, 0.0, 0.0));

        // Center must remain at origin after the drag.
        {
            let sk = ed.sketch.as_ref().unwrap();
            let pts = resolved_points(&sk.committed_profiles[0], &sk.global_points);
            assert!(
                pts[2].coords.norm() < 1e-6,
                "center should remain at origin after dragging start handle, got {:?}", pts[2]
            );
        }
    }
}
