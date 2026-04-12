//! The [`Sketch`] type: a collection of profiles and constraints on a plane.
//!
//! All coordinates are in plane UV space.  Use [`crate::Plane::world_to_uv`] and
//! [`crate::Plane::uv_to_world`] to convert to/from world space.

use crate::arc::project_center_to_arc_bisector;
use crate::constraints::{to_sketch_constraint, Constraint, SketchConstraint};
use crate::geometry::Plane;
use crate::profile::{ConstraintId, GlobalPointId, PointId, Profile, ProfileId, ProfileShape};
use crate::solver::{apply_constraints, compute_dof, solve_constraints, SolveResult};

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors returned by sketch mutation operations.
#[derive(Debug, thiserror::Error)]
pub enum SketchError {
    #[error("profile {0:?} does not exist")]
    NoSuchProfile(ProfileId),

    #[error("point {pt:?} does not exist in profile {profile:?}")]
    NoSuchPoint { profile: ProfileId, pt: PointId },

    #[error("constraint {0:?} does not exist")]
    NoSuchConstraint(ConstraintId),

    #[error("global point {0:?} does not exist")]
    NoSuchGlobalPoint(GlobalPointId),

    #[error("profile {0:?} has too few points for this operation")]
    TooFewPoints(ProfileId),

    #[error("profile {0:?} is not an arc")]
    NotArc(ProfileId),

    #[error("profile {0:?} is not a polyline")]
    NotPolyline(ProfileId),
}

pub type SketchResult<T> = Result<T, SketchError>;

// ── Solve report ──────────────────────────────────────────────────────────────

/// Per-constraint solve status.
#[derive(Clone, Debug)]
pub struct ConstraintStatus {
    pub id: ConstraintId,
    /// Whether this constraint was violated (part of a conflicting set).
    pub violated: bool,
}

/// Result of a full sketch solve.
#[derive(Clone, Debug)]
pub struct FullSolveReport {
    /// Whether all constraints were satisfied after solving.
    pub converged: bool,
    /// Per-constraint violation info (only populated when `converged == false`).
    pub constraint_statuses: Vec<ConstraintStatus>,
    /// Remaining degrees of freedom after solving.
    ///
    /// - `Some(0)`  → fully constrained.
    /// - `Some(n)`  where `n > 0` → `n` free motion modes remain.
    /// - `Some(n)`  where `n < 0` → over-constrained (dependent constraints present).
    /// - `None`     → not computed because `converged == false` (DOF is undefined
    ///                at a conflicted or unconverged state).
    ///
    /// Note: cross-profile constraints are not included in this count; the
    /// value may be conservatively high for multi-profile sketches that use them.
    pub dof: Option<i32>,
}

// ── Snapshot ──────────────────────────────────────────────────────────────────

/// A complete deep copy of a [`Sketch`]'s logical state, used for undo/redo.
///
/// The caller owns the undo stack; `Sketch` does not maintain history internally.
///
/// # Example
/// ```ignore
/// let snap = sketch.snapshot();
/// sketch.push_point(pid, (1.0, 2.0))?;
/// sketch.restore(snap); // undo
/// ```
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SketchSnapshot {
    plane:              Plane,
    name:               String,
    profiles:           Vec<(ProfileId, Profile)>,
    constraints:        Vec<(ConstraintId, Constraint)>,
    global_points:      Vec<(GlobalPointId, [f64; 2])>,
    next_profile_id:    usize,
    next_constraint_id: usize,
    next_global_id:     usize,
}

// ── Sketch ────────────────────────────────────────────────────────────────────

/// A 2D parametric sketch: profiles and constraints on a plane.
///
/// All coordinates are in plane UV space.  No world-space types appear here.
/// Use [`Plane::world_to_uv`] / [`Plane::uv_to_world`] on `self.plane` to
/// convert between spaces.
///
/// # Undo / redo
///
/// Call [`snapshot`](Self::snapshot) before any mutating operation to save
/// state, and [`restore`](Self::restore) to roll back.  The caller manages
/// the undo stack.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Sketch {
    /// The plane this sketch lives on.
    pub plane: Plane,
    /// Human-readable name.
    pub name: String,

    profiles:           Vec<(ProfileId, Profile)>,
    constraints:        Vec<(ConstraintId, Constraint)>,
    /// Flat store of globally shared point positions (UV).
    global_points:      Vec<(GlobalPointId, [f64; 2])>,

    next_profile_id:    usize,
    next_constraint_id: usize,
    next_global_id:     usize,
}

impl Sketch {
    // ── Construction ──────────────────────────────────────────────────────────

    /// Create an empty sketch on the given plane.
    pub fn new(plane: Plane, name: impl Into<String>) -> Self {
        Self {
            plane,
            name: name.into(),
            profiles:           Vec::new(),
            constraints:        Vec::new(),
            global_points:      Vec::new(),
            next_profile_id:    0,
            next_constraint_id: 0,
            next_global_id:     0,
        }
    }

    // ── Snapshot / restore ────────────────────────────────────────────────────

    /// Capture a deep copy of the current sketch state for undo/redo.
    pub fn snapshot(&self) -> SketchSnapshot {
        SketchSnapshot {
            plane:              self.plane,
            name:               self.name.clone(),
            profiles:           self.profiles.clone(),
            constraints:        self.constraints.clone(),
            global_points:      self.global_points.clone(),
            next_profile_id:    self.next_profile_id,
            next_constraint_id: self.next_constraint_id,
            next_global_id:     self.next_global_id,
        }
    }

    /// Restore the sketch to a previously taken snapshot.
    pub fn restore(&mut self, snap: SketchSnapshot) {
        self.plane              = snap.plane;
        self.name               = snap.name;
        self.profiles           = snap.profiles;
        self.constraints        = snap.constraints;
        self.global_points      = snap.global_points;
        self.next_profile_id    = snap.next_profile_id;
        self.next_constraint_id = snap.next_constraint_id;
        self.next_global_id     = snap.next_global_id;
    }

    // ── Profile creation ──────────────────────────────────────────────────────

    /// Add an empty polyline profile and return its [`ProfileId`].
    ///
    /// Add points with [`push_point`](Self::push_point) or
    /// [`push_shared_point`](Self::push_shared_point).
    pub fn add_polyline_profile(&mut self, closed: bool) -> ProfileId {
        self.alloc_profile(ProfileShape::Polyline { closed })
    }

    /// Add a circle profile (center + radius point) and return its [`ProfileId`].
    pub fn add_circle_profile(&mut self, center: (f64, f64), radius_pt: (f64, f64)) -> ProfileId {
        let pid = self.alloc_profile(ProfileShape::Circle);
        let idx = self.profile_pos(pid).unwrap();
        let p = &mut self.profiles[idx].1;
        p.points.push([center.0, center.1]);
        p.global_ids.push(None);
        p.points.push([radius_pt.0, radius_pt.1]);
        p.global_ids.push(None);
        pid
    }

    /// Add an arc profile (start, end, center) and return its [`ProfileId`].
    ///
    /// The center is projected onto the perpendicular bisector of the
    /// `start→end` chord so the arc invariant is maintained.
    pub fn add_arc_profile(
        &mut self,
        start:  (f64, f64),
        end_pt: (f64, f64),
        center: (f64, f64),
    ) -> ProfileId {
        let c_proj = project_center_to_arc_bisector(start, end_pt, center);
        let pid = self.alloc_profile(ProfileShape::Arc);
        let idx = self.profile_pos(pid).unwrap();
        let p = &mut self.profiles[idx].1;
        for uv in [start, end_pt, c_proj] {
            p.points.push([uv.0, uv.1]);
            p.global_ids.push(None);
        }
        pid
    }

    /// Add an empty arc profile with no points.
    ///
    /// Push exactly three points in order — start, end, center — using
    /// [`push_point`](Self::push_point) or
    /// [`push_shared_point`](Self::push_shared_point).
    pub fn alloc_arc_profile(&mut self) -> ProfileId {
        self.alloc_profile(ProfileShape::Arc)
    }

    /// Add an empty circle profile with no points.
    ///
    /// Push exactly two points in order — center, radius point — using
    /// [`push_point`](Self::push_point) or
    /// [`push_shared_point`](Self::push_shared_point).
    pub fn alloc_circle_profile(&mut self) -> ProfileId {
        self.alloc_profile(ProfileShape::Circle)
    }

    /// Remove a profile and all constraints that reference it.
    pub fn remove_profile(&mut self, profile: ProfileId) -> SketchResult<()> {
        let pos = self.profile_pos(profile)
            .ok_or(SketchError::NoSuchProfile(profile))?;
        self.profiles.remove(pos);
        self.constraints.retain(|(_, c)| {
            !c.referenced_profiles().any(|p| p == profile)
        });
        Ok(())
    }

    // ── Point management ──────────────────────────────────────────────────────

    /// Append a private point to a profile and return its [`PointId`].
    pub fn push_point(&mut self, profile: ProfileId, uv: (f64, f64)) -> SketchResult<PointId> {
        let idx = self.profile_pos(profile)
            .ok_or(SketchError::NoSuchProfile(profile))?;
        let p = &mut self.profiles[idx].1;
        let id = PointId(p.points.len());
        p.points.push([uv.0, uv.1]);
        p.global_ids.push(None);
        Ok(id)
    }

    /// Append a point backed by a [`GlobalPointId`] to a profile.
    ///
    /// The point's position is taken from the global store, and future moves
    /// via [`move_point`](Self::move_point) will propagate to all profiles that
    /// share the same global ID.
    pub fn push_shared_point(
        &mut self,
        profile:   ProfileId,
        global_id: GlobalPointId,
    ) -> SketchResult<PointId> {
        let uv = self.global_point_uv(global_id)?;
        let idx = self.profile_pos(profile)
            .ok_or(SketchError::NoSuchProfile(profile))?;
        let p = &mut self.profiles[idx].1;
        let id = PointId(p.points.len());
        p.points.push(uv);
        p.global_ids.push(Some(global_id));
        Ok(id)
    }

    /// Move point `pt` of `profile` to UV coordinates `uv`.
    ///
    /// If the point is backed by a [`GlobalPointId`], all other profiles that
    /// share the same global ID are also updated.
    pub fn move_point(
        &mut self,
        profile: ProfileId,
        pt: PointId,
        uv: (f64, f64),
    ) -> SketchResult<()> {
        // Check profile exists and get the optional global id.
        let maybe_gid = {
            let idx = self.profile_pos(profile)
                .ok_or(SketchError::NoSuchProfile(profile))?;
            let p = &self.profiles[idx].1;
            if pt.0 >= p.points.len() {
                return Err(SketchError::NoSuchPoint { profile, pt });
            }
            p.global_ids[pt.0]
        };

        if let Some(gid) = maybe_gid {
            // Update global store and propagate.
            self.set_global_point_uv(gid, [uv.0, uv.1])?;
            for (_, prof) in &mut self.profiles {
                for (i, maybe) in prof.global_ids.iter().enumerate() {
                    if *maybe == Some(gid) {
                        prof.points[i] = [uv.0, uv.1];
                    }
                }
            }
        } else {
            let idx = self.profile_pos(profile).unwrap();
            self.profiles[idx].1.points[pt.0] = [uv.0, uv.1];
        }
        Ok(())
    }

    /// Set the `closed` flag of a polyline profile.
    pub fn set_closed(&mut self, profile: ProfileId, closed: bool) -> SketchResult<()> {
        let idx = self.profile_pos(profile)
            .ok_or(SketchError::NoSuchProfile(profile))?;
        match &mut self.profiles[idx].1.shape {
            ProfileShape::Polyline { closed: c } => { *c = closed; Ok(()) }
            _ => Err(SketchError::NotPolyline(profile)),
        }
    }

    // ── Global point management ───────────────────────────────────────────────

    /// Allocate a new global point at UV position `uv`.
    ///
    /// Use this to create a shared endpoint when building a polyline chain
    /// where adjacent segments share a vertex.  Then call
    /// [`push_shared_point`](Self::push_shared_point) on each profile.
    pub fn alloc_global_point(&mut self, uv: (f64, f64)) -> GlobalPointId {
        let id = GlobalPointId(self.next_global_id);
        self.next_global_id += 1;
        self.global_points.push((id, [uv.0, uv.1]));
        id
    }

    /// Return the UV position of a global point.
    pub fn global_point_uv(&self, id: GlobalPointId) -> SketchResult<[f64; 2]> {
        self.global_points.iter()
            .find(|(gid, _)| *gid == id)
            .map(|(_, uv)| *uv)
            .ok_or(SketchError::NoSuchGlobalPoint(id))
    }

    /// Merge two global points: redirect every profile that references
    /// `replace` to use `keep`, and snap `keep` to the average of the two
    /// positions.
    ///
    /// This makes cross-profile endpoints structurally coincident without
    /// needing an explicit `Coincident` constraint.
    pub fn merge_global_points(
        &mut self,
        keep:    GlobalPointId,
        replace: GlobalPointId,
    ) -> SketchResult<()> {
        if keep == replace { return Ok(()); }
        let uv_keep    = self.global_point_uv(keep)?;
        let uv_replace = self.global_point_uv(replace)?;
        let avg = [
            (uv_keep[0] + uv_replace[0]) * 0.5,
            (uv_keep[1] + uv_replace[1]) * 0.5,
        ];
        self.set_global_point_uv(keep, avg)?;
        for (_, prof) in &mut self.profiles {
            for (i, maybe) in prof.global_ids.iter_mut().enumerate() {
                if *maybe == Some(replace) {
                    *maybe = Some(keep);
                    prof.points[i] = avg;
                } else if *maybe == Some(keep) {
                    prof.points[i] = avg;
                }
            }
        }
        self.global_points.retain(|(gid, _)| *gid != replace);
        Ok(())
    }

    // ── Constraint management ─────────────────────────────────────────────────

    /// Add a constraint and return its [`ConstraintId`].
    ///
    /// Returns an error if any referenced profile does not exist.
    pub fn add_constraint(&mut self, c: Constraint) -> SketchResult<ConstraintId> {
        // Validate all referenced profiles exist.
        let missing: Vec<ProfileId> = c.referenced_profiles()
            .filter(|&pid| self.profile(pid).is_none())
            .collect();
        if let Some(&pid) = missing.first() {
            return Err(SketchError::NoSuchProfile(pid));
        }
        let id = ConstraintId(self.next_constraint_id);
        self.next_constraint_id += 1;
        self.constraints.push((id, c));
        Ok(id)
    }

    /// Remove a constraint by ID.
    pub fn remove_constraint(&mut self, id: ConstraintId) -> SketchResult<()> {
        let pos = self.constraints.iter().position(|(cid, _)| *cid == id)
            .ok_or(SketchError::NoSuchConstraint(id))?;
        self.constraints.remove(pos);
        Ok(())
    }

    /// Iterate over all `(ConstraintId, &Constraint)` pairs.
    pub fn constraints(&self) -> impl Iterator<Item = (ConstraintId, &Constraint)> {
        self.constraints.iter().map(|(id, c)| (*id, c))
    }

    /// Iterate over constraints that belong to a specific profile (single-profile
    /// constraints whose `profile` field equals `pid`, including same-profile pairs).
    pub fn profile_constraints(
        &self,
        pid: ProfileId,
    ) -> impl Iterator<Item = (ConstraintId, &Constraint)> {
        self.constraints.iter()
            .filter(move |(_, c)| c.single_profile() == Some(pid))
            .map(|(id, c)| (*id, c))
    }

    // ── Profile access ────────────────────────────────────────────────────────

    /// Iterate over all `(ProfileId, &Profile)` pairs in insertion order.
    pub fn profiles(&self) -> impl Iterator<Item = (ProfileId, &Profile)> {
        self.profiles.iter().map(|(id, p)| (*id, p))
    }

    /// Get a reference to a profile by ID.
    pub fn profile(&self, id: ProfileId) -> Option<&Profile> {
        self.profiles.iter().find(|(pid, _)| *pid == id).map(|(_, p)| p)
    }

    // ── Drag helper ───────────────────────────────────────────────────────────

    /// Drag point `pt` of `profile` to `uv`, respecting constraints.
    ///
    /// Temporarily injects a `PointFixed` pin constraint, runs the solver,
    /// then removes the pin.  If the pin conflicts with existing constraints,
    /// the sketch is reverted to its pre-drag state.
    pub fn drag_point(
        &mut self,
        profile: ProfileId,
        pt: PointId,
        uv: (f64, f64),
    ) -> SketchResult<FullSolveReport> {
        let backup = self.snapshot();

        // Move the point to the target first (good starting point for solver).
        self.move_point(profile, pt, uv)?;

        // Add a temporary PointFixed pin.
        let pin = Constraint::PointFixed { profile, pt, u: uv.0, v: uv.1 };
        let pin_id = self.add_constraint(pin)?;

        let report = self.solve();

        self.remove_constraint(pin_id)?;

        if !report.converged {
            self.restore(backup);
            return Ok(self.solve());
        }
        Ok(report)
    }

    // ── Arc helper ────────────────────────────────────────────────────────────

    /// Re-project an arc profile's center onto the perpendicular bisector of
    /// its `start→end` chord.
    ///
    /// Call this after moving an arc endpoint by drag so the center stays
    /// geometrically consistent.
    pub fn reproject_arc_center(&mut self, profile: ProfileId) -> SketchResult<()> {
        let idx = self.profile_pos(profile)
            .ok_or(SketchError::NoSuchProfile(profile))?;
        if !matches!(self.profiles[idx].1.shape, ProfileShape::Arc) {
            return Err(SketchError::NotArc(profile));
        }
        if self.profiles[idx].1.points.len() < 3 {
            return Err(SketchError::TooFewPoints(profile));
        }
        let [su, sv] = self.profiles[idx].1.points[0];
        let [eu, ev] = self.profiles[idx].1.points[1];
        let [cu, cv] = self.profiles[idx].1.points[2];
        let (nu, nv) = project_center_to_arc_bisector((su, sv), (eu, ev), (cu, cv));
        self.profiles[idx].1.points[2] = [nu, nv];
        Ok(())
    }

    // ── Solve ─────────────────────────────────────────────────────────────────

    /// Run the constraint solver over the entire sketch.
    ///
    /// Uses a 3-pass strategy:
    /// 1. Solve each profile's local constraints.
    /// 2. Apply cross-profile constraints.
    /// 3. Repeat three times for convergence.
    ///
    /// Point positions are updated in-place.
    pub fn solve(&mut self) -> FullSolveReport {
        let mut all_converged = true;
        let mut statuses: Vec<ConstraintStatus> = Vec::new();

        for _ in 0..3 {
            let profile_ids: Vec<ProfileId> =
                self.profiles.iter().map(|(id, _)| *id).collect();

            for pid in &profile_ids {
                if !self.solve_profile_impl(*pid, &mut statuses) {
                    all_converged = false;
                }
            }

            if !self.solve_cross_impl() {
                all_converged = false;
            }
        }

        let dof = if all_converged { Some(self.compute_sketch_dof()) } else { None };
        FullSolveReport { converged: all_converged, constraint_statuses: statuses, dof }
    }

    /// Run the solver for a single profile's local constraints only.
    ///
    /// Useful when only one profile has been mutated and a full re-solve
    /// would be wasteful.
    pub fn solve_profile_only(&mut self, profile_id: ProfileId) -> SketchResult<FullSolveReport> {
        if self.profile_pos(profile_id).is_none() {
            return Err(SketchError::NoSuchProfile(profile_id));
        }
        let mut statuses = Vec::new();
        let converged = self.solve_profile_impl(profile_id, &mut statuses);
        let dof = if converged {
            let profile = self.profile(profile_id).unwrap();
            let n = profile.points.len();
            let local_scs: Vec<SketchConstraint> = self.constraints.iter()
                .filter_map(|(_, c)| to_sketch_constraint(c, profile_id))
                .collect();
            Some(compute_dof(&profile.points, &local_scs, n))
        } else {
            None
        };
        Ok(FullSolveReport { converged, constraint_statuses: statuses, dof })
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    fn alloc_profile(&mut self, shape: ProfileShape) -> ProfileId {
        let id = ProfileId(self.next_profile_id);
        self.next_profile_id += 1;
        self.profiles.push((id, Profile::new(shape)));
        id
    }

    fn profile_pos(&self, id: ProfileId) -> Option<usize> {
        self.profiles.iter().position(|(pid, _)| *pid == id)
    }

    fn set_global_point_uv(
        &mut self,
        id: GlobalPointId,
        uv: [f64; 2],
    ) -> SketchResult<()> {
        self.global_points.iter_mut()
            .find(|(gid, _)| *gid == id)
            .map(|(_, p)| { *p = uv; })
            .ok_or(SketchError::NoSuchGlobalPoint(id))
    }

    /// After solving a profile, propagate updated point positions to the global
    /// store and to all other profiles that share the same GlobalPointId.
    fn propagate_profile_to_globals(&mut self, profile_idx: usize) {
        let updates: Vec<(GlobalPointId, [f64; 2])> = {
            let p = &self.profiles[profile_idx].1;
            p.global_ids.iter()
                .zip(p.points.iter())
                .filter_map(|(maybe_gid, &pt)| maybe_gid.map(|gid| (gid, pt)))
                .collect()
        };

        for (gid, uv) in updates {
            for (id, stored) in &mut self.global_points {
                if *id == gid { *stored = uv; }
            }
            for (_, prof) in &mut self.profiles {
                for (i, maybe) in prof.global_ids.iter().enumerate() {
                    if *maybe == Some(gid) {
                        prof.points[i] = uv;
                    }
                }
            }
        }
    }

    /// Solve single-profile constraints for `pid`.  Returns `true` on convergence.
    fn solve_profile_impl(
        &mut self,
        pid: ProfileId,
        statuses: &mut Vec<ConstraintStatus>,
    ) -> bool {
        let profile_idx = match self.profile_pos(pid) {
            Some(i) => i,
            None => return true,
        };

        let n = self.profiles[profile_idx].1.points.len();
        if n < 2 { return true; }

        // Translate constraints before taking the mutable profile borrow.
        let local_scs: Vec<SketchConstraint> = self.constraints.iter()
            .filter_map(|(_, c)| to_sketch_constraint(c, pid))
            .collect();

        if local_scs.is_empty() { return true; }

        let result = apply_constraints(&mut self.profiles[profile_idx].1.points, &local_scs, n);

        // Collect violated constraint IDs for the report.
        if result.conflict {
            // Map violated booleans back to ConstraintIds.
            let ids: Vec<ConstraintId> = self.constraints.iter()
                .filter_map(|(cid, c)| to_sketch_constraint(c, pid).map(|_| *cid))
                .collect();
            for (i, &violated) in result.violated.iter().enumerate() {
                if violated {
                    if let Some(cid) = ids.get(i) {
                        statuses.push(ConstraintStatus { id: *cid, violated: true });
                    }
                }
            }
        }

        self.propagate_profile_to_globals(profile_idx);

        !result.conflict
    }

    /// Solve all cross-profile constraints.  Returns `true` if all succeeded.
    fn solve_cross_impl(&mut self) -> bool {
        let cross: Vec<(ConstraintId, Constraint)> = self.constraints.iter()
            .filter(|(_, c)| c.is_cross_profile())
            .cloned()
            .collect();

        let mut all_ok = true;

        for (_, c) in &cross {
            match c {
                // ── 4-point segment-based cross constraints ────────────────────
                Constraint::CrossParallel { profile_a, seg_a, profile_b, seg_b } => {
                    let sc = SketchConstraint::Parallel { seg_a: 0, seg_b: 2 };
                    self.solve_4pt_cross(*profile_a, *seg_a, *profile_b, *seg_b, sc, &mut all_ok);
                }
                Constraint::CrossPerpendicular { profile_a, seg_a, profile_b, seg_b } => {
                    let sc = SketchConstraint::Perpendicular { seg_a: 0, seg_b: 2 };
                    self.solve_4pt_cross(*profile_a, *seg_a, *profile_b, *seg_b, sc, &mut all_ok);
                }
                Constraint::CrossEqualLength { profile_a, seg_a, profile_b, seg_b } => {
                    let sc = SketchConstraint::EqualLength { seg_a: 0, seg_b: 2 };
                    self.solve_4pt_cross(*profile_a, *seg_a, *profile_b, *seg_b, sc, &mut all_ok);
                }
                Constraint::CrossAngle { profile_a, seg_a, profile_b, seg_b, degrees } => {
                    let sc = SketchConstraint::Angle { seg_a: 0, seg_b: 2, degrees: *degrees };
                    self.solve_4pt_cross(*profile_a, *seg_a, *profile_b, *seg_b, sc, &mut all_ok);
                }

                // ── 2-point alignment constraints ──────────────────────────────
                Constraint::HorizontalPair { profile_a, pt_a, profile_b, pt_b, perp_u, perp_v }
                    if profile_a != profile_b =>
                {
                    let sc = SketchConstraint::HorizontalPair {
                        pt_a: 0, pt_b: 1, perp_u: *perp_u, perp_v: *perp_v,
                    };
                    self.solve_2pt_cross(*profile_a, *pt_a, *profile_b, *pt_b, sc, &mut all_ok);
                }
                Constraint::VerticalPair { profile_a, pt_a, profile_b, pt_b, perp_u, perp_v }
                    if profile_a != profile_b =>
                {
                    let sc = SketchConstraint::VerticalPair {
                        pt_a: 0, pt_b: 1, perp_u: *perp_u, perp_v: *perp_v,
                    };
                    self.solve_2pt_cross(*profile_a, *pt_a, *profile_b, *pt_b, sc, &mut all_ok);
                }

                // ── Cross-profile coincident ───────────────────────────────────
                Constraint::CrossCoincident { profile_a, pt_a, profile_b, pt_b } => {
                    let sc = SketchConstraint::Coincident { pt_a: 0, pt_b: 1 };
                    self.solve_2pt_cross(*profile_a, *pt_a, *profile_b, *pt_b, sc, &mut all_ok);
                }

                // ── Symmetric (perpendicular-distance equidistant) ─────────────
                Constraint::Symmetric {
                    profile_seg_a, seg_a,
                    profile_seg_b, seg_b,
                    profile_pt, pt,
                } => {
                    self.solve_symmetric(
                        *profile_seg_a, *seg_a,
                        *profile_seg_b, *seg_b,
                        *profile_pt, *pt,
                        &mut all_ok,
                    );
                }

                // ── SymmetricPoints (equal distances from two reference points) ──
                Constraint::SymmetricPoints {
                    profile_a, pt_a,
                    profile_b, pt_b,
                    profile_c, pt_c,
                } => {
                    self.solve_symmetric_points(
                        *profile_a, *pt_a,
                        *profile_b, *pt_b,
                        *profile_c, *pt_c,
                        &mut all_ok,
                    );
                }

                _ => {} // same-profile HorizontalPair/VerticalPair: handled in solve_profile_impl
            }
        }

        all_ok
    }

    /// Compute the total DOF across all profiles using single-profile constraints only.
    ///
    /// Cross-profile constraints are excluded; the result is a conservative upper
    /// bound for multi-profile sketches that use them.
    fn compute_sketch_dof(&self) -> i32 {
        let mut total = 0i32;
        for &(pid, ref profile) in &self.profiles {
            let n = profile.points.len();
            if n < 2 {
                total += 2 * n as i32;
                continue;
            }
            let local_scs: Vec<SketchConstraint> = self.constraints.iter()
                .filter_map(|(_, c)| to_sketch_constraint(c, pid))
                .collect();
            total += compute_dof(&profile.points, &local_scs, n);
        }
        total
    }

    /// Solve a cross-profile constraint using a 4-point synthetic array.
    /// Segment `seg_a` of `profile_a` maps to indices 0–1; `seg_b` of `profile_b` to 2–3.
    fn solve_4pt_cross(
        &mut self,
        profile_a: ProfileId, seg_a: usize,
        profile_b: ProfileId, seg_b: usize,
        constraint: SketchConstraint,
        all_ok: &mut bool,
    ) {
        let (ai0, ai1) = match self.profile(profile_a)
            .and_then(|p| p.segment_point_indices(seg_a))
        {
            Some(v) => v, None => return,
        };
        let (bi0, bi1) = match self.profile(profile_b)
            .and_then(|p| p.segment_point_indices(seg_b))
        {
            Some(v) => v, None => return,
        };

        let pa = self.profile_pos(profile_a).unwrap();
        let pb = self.profile_pos(profile_b).unwrap();

        // Read points (Copy, so borrows end immediately).
        let a0 = self.profiles[pa].1.points[ai0];
        let a1 = self.profiles[pa].1.points[ai1];
        let b0 = self.profiles[pb].1.points[bi0];
        let b1 = self.profiles[pb].1.points[bi1];

        let mut pts = vec![a0, a1, b0, b1];
        if solve_constraints(&mut pts, &[constraint], 4) == SolveResult::Ok {
            self.profiles[pa].1.points[ai0] = pts[0];
            self.profiles[pa].1.points[ai1] = pts[1];
            self.profiles[pb].1.points[bi0] = pts[2];
            self.profiles[pb].1.points[bi1] = pts[3];
            self.propagate_profile_to_globals(pa);
            self.propagate_profile_to_globals(pb);
        } else {
            *all_ok = false;
        }
    }

    /// Solve a 2-point cross-profile constraint.
    fn solve_2pt_cross(
        &mut self,
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
        constraint: SketchConstraint,
        all_ok: &mut bool,
    ) {
        let pa = match self.profile_pos(profile_a) { Some(i) => i, None => return };
        let pb = match self.profile_pos(profile_b) { Some(i) => i, None => return };

        if pt_a.0 >= self.profiles[pa].1.points.len()
            || pt_b.0 >= self.profiles[pb].1.points.len()
        { return; }

        let uv_a = self.profiles[pa].1.points[pt_a.0];
        let uv_b = self.profiles[pb].1.points[pt_b.0];

        let mut pts = vec![uv_a, uv_b];
        if solve_constraints(&mut pts, &[constraint], 2) == SolveResult::Ok {
            self.profiles[pa].1.points[pt_a.0] = pts[0];
            self.profiles[pb].1.points[pt_b.0] = pts[1];
            self.propagate_profile_to_globals(pa);
            self.propagate_profile_to_globals(pb);
        } else {
            *all_ok = false;
        }
    }

    /// Solve `Symmetric`: point `pt` of `profile_pt` is equidistant (perpendicular
    /// distance) from segment `seg_a` of `profile_seg_a` and segment `seg_b` of
    /// `profile_seg_b`.
    fn solve_symmetric(
        &mut self,
        profile_seg_a: ProfileId, seg_a: usize,
        profile_seg_b: ProfileId, seg_b: usize,
        profile_pt:    ProfileId, pt: PointId,
        _all_ok: &mut bool,
    ) {
        let (ai0, ai1) = match self.profile(profile_seg_a)
            .and_then(|p| p.segment_point_indices(seg_a))
        { Some(v) => v, None => return };
        let (bi0, bi1) = match self.profile(profile_seg_b)
            .and_then(|p| p.segment_point_indices(seg_b))
        { Some(v) => v, None => return };

        let pa = self.profile_pos(profile_seg_a).unwrap();
        let pb = self.profile_pos(profile_seg_b).unwrap();
        let pp = match self.profile_pos(profile_pt) { Some(i) => i, None => return };

        if pt.0 >= self.profiles[pp].1.points.len() { return; }

        let [ax0, ay0] = self.profiles[pa].1.points[ai0];
        let [ax1, ay1] = self.profiles[pa].1.points[ai1];
        let [bx0, by0] = self.profiles[pb].1.points[bi0];
        let [bx1, by1] = self.profiles[pb].1.points[bi1];
        let [mut px, mut py] = self.profiles[pp].1.points[pt.0];

        // Perpendicular unit normals to each segment.
        let (n1x, n1y) = {
            let (dx, dy) = (ax1 - ax0, ay1 - ay0);
            let l = (dx * dx + dy * dy).sqrt();
            if l < 1e-10 { return } else { (-dy / l, dx / l) }
        };
        let (n2x, n2y) = {
            let (dx, dy) = (bx1 - bx0, by1 - by0);
            let l = (dx * dx + dy * dy).sqrt();
            if l < 1e-10 { return } else { (-dy / l, dx / l) }
        };

        // Newton iterate: minimise f = d1² - d2²
        for _ in 0..20 {
            let d1 = n1x * (px - ax0) + n1y * (py - ay0);
            let d2 = n2x * (px - bx0) + n2y * (py - by0);
            let f  = d1 * d1 - d2 * d2;
            if f.abs() < 1e-9 { break; }
            let (gx, gy) = (2.0 * d1 * n1x - 2.0 * d2 * n2x,
                            2.0 * d1 * n1y - 2.0 * d2 * n2y);
            let g2 = gx * gx + gy * gy;
            if g2 < 1e-12 { break; }
            px -= f / g2 * gx;
            py -= f / g2 * gy;
        }

        self.profiles[pp].1.points[pt.0] = [px, py];
        self.propagate_profile_to_globals(pp);
    }

    /// Solve `SymmetricPoints`: `dist(P_a, P_c) == dist(P_b, P_c)`.
    /// `P_c` is the constrained point; `P_a` and `P_b` are the reference points.
    fn solve_symmetric_points(
        &mut self,
        profile_a: ProfileId, pt_a: PointId,
        profile_b: ProfileId, pt_b: PointId,
        profile_c: ProfileId, pt_c: PointId,
        _all_ok: &mut bool,
    ) {
        let pa = match self.profile_pos(profile_a) { Some(i) => i, None => return };
        let pb = match self.profile_pos(profile_b) { Some(i) => i, None => return };
        let pc = match self.profile_pos(profile_c) { Some(i) => i, None => return };

        if pt_a.0 >= self.profiles[pa].1.points.len() { return; }
        if pt_b.0 >= self.profiles[pb].1.points.len() { return; }
        if pt_c.0 >= self.profiles[pc].1.points.len() { return; }

        let [ax, ay] = self.profiles[pa].1.points[pt_a.0];
        let [bx, by] = self.profiles[pb].1.points[pt_b.0];
        let [mut cx, mut cy] = self.profiles[pc].1.points[pt_c.0];

        // Newton iterate: minimise f = dist(A,C)² - dist(B,C)²
        // ∇f = [2(cx-ax) - 2(cx-bx), 2(cy-ay) - 2(cy-by)]
        for _ in 0..20 {
            let dax = cx - ax; let day = cy - ay;
            let dbx = cx - bx; let dby = cy - by;
            let f = dax*dax + day*day - dbx*dbx - dby*dby;
            if f.abs() < 1e-9 { break; }
            let (gx, gy) = (2.0*dax - 2.0*dbx, 2.0*day - 2.0*dby);
            let g2 = gx*gx + gy*gy;
            if g2 < 1e-12 { break; }
            cx -= f / g2 * gx;
            cy -= f / g2 * gy;
        }

        self.profiles[pc].1.points[pt_c.0] = [cx, cy];
        self.propagate_profile_to_globals(pc);
    }
}
