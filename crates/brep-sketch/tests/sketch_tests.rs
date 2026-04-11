//! Integration tests for the [`brep_sketch::Sketch`] high-level API.

use brep_sketch::{
    Constraint, Plane, Sketch, SketchError,
    PointId, ProfileId,
};

// ── Helpers ───────────────────────────────────────────────────────────────────

fn approx_eq(a: f64, b: f64) -> bool { (a - b).abs() < 1e-6 }

fn point(sketch: &Sketch, pid: ProfileId, pt: PointId) -> (f64, f64) {
    sketch.profile(pid).unwrap().point(pt).unwrap()
}

fn seg_len(p0: (f64, f64), p1: (f64, f64)) -> f64 {
    let dx = p1.0 - p0.0;
    let dy = p1.1 - p0.1;
    (dx * dx + dy * dy).sqrt()
}

// ── Plane ─────────────────────────────────────────────────────────────────────

#[test]
fn plane_xy_round_trip() {
    let plane = Plane::xy();
    let (u, v) = plane.world_to_uv([3.0, 4.0, 0.0]);
    let back = plane.uv_to_world(u, v);
    assert!(approx_eq(back[0], 3.0));
    assert!(approx_eq(back[1], 4.0));
    assert!(approx_eq(back[2], 0.0));
}

#[test]
fn plane_arbitrary_origin() {
    let plane = Plane {
        origin: [1.0, 2.0, 0.0],
        u_axis: [1.0, 0.0, 0.0],
        v_axis: [0.0, 1.0, 0.0],
    };
    let (u, v) = plane.world_to_uv([4.0, 6.0, 0.0]);
    assert!(approx_eq(u, 3.0));
    assert!(approx_eq(v, 4.0));
    let back = plane.uv_to_world(u, v);
    assert!(approx_eq(back[0], 4.0));
    assert!(approx_eq(back[1], 6.0));
}

// ── Profile creation ──────────────────────────────────────────────────────────

#[test]
fn add_polyline_profile_and_push_points() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    let p0 = sketch.push_point(pid, (0.0, 0.0)).unwrap();
    let p1 = sketch.push_point(pid, (1.0, 1.0)).unwrap();
    assert_eq!(sketch.profile(pid).unwrap().point_count(), 2);
    assert_eq!(point(&sketch, pid, p0), (0.0, 0.0));
    assert_eq!(point(&sketch, pid, p1), (1.0, 1.0));
}

#[test]
fn add_circle_profile_stores_two_points() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_circle_profile((0.0, 0.0), (1.0, 0.0));
    assert_eq!(sketch.profile(pid).unwrap().point_count(), 2);
    assert_eq!(point(&sketch, pid, PointId(0)), (0.0, 0.0));
    assert_eq!(point(&sketch, pid, PointId(1)), (1.0, 0.0));
}

#[test]
fn add_arc_profile_projects_center_onto_bisector() {
    // Horizontal chord (0,0)→(2,0); perpendicular bisector is the vertical x=1.
    // Center at (0.5, 3.0) should project to (1.0, 3.0).
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_arc_profile((0.0, 0.0), (2.0, 0.0), (0.5, 3.0));
    let center = point(&sketch, pid, PointId(2));
    assert!(approx_eq(center.0, 1.0), "center_u = {}", center.0);
    assert!(approx_eq(center.1, 3.0), "center_v = {}", center.1);
}

// ── Global points / merging ───────────────────────────────────────────────────

#[test]
fn merge_global_points_averages_positions() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let g0 = sketch.alloc_global_point((0.0, 0.0));
    let g1 = sketch.alloc_global_point((2.0, 4.0));
    sketch.merge_global_points(g0, g1).unwrap();
    let uv = sketch.global_point_uv(g0).unwrap();
    assert!(approx_eq(uv[0], 1.0));
    assert!(approx_eq(uv[1], 2.0));
}

#[test]
fn merge_global_points_propagates_to_profiles() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let g0 = sketch.alloc_global_point((0.0, 0.0));
    let g1 = sketch.alloc_global_point((2.0, 0.0));

    let pa = sketch.add_polyline_profile(false);
    sketch.push_shared_point(pa, g0).unwrap();
    sketch.push_point(pa, (1.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    sketch.push_point(pb, (1.0, 0.0)).unwrap();
    sketch.push_shared_point(pb, g1).unwrap();

    sketch.merge_global_points(g0, g1).unwrap();

    // Both profiles' shared points should now be at (1.0, 0.0).
    let pa_pt0 = point(&sketch, pa, PointId(0));
    let pb_pt1 = point(&sketch, pb, PointId(1));
    assert!(approx_eq(pa_pt0.0, 1.0));
    assert!(approx_eq(pa_pt0.1, 0.0));
    assert!(approx_eq(pb_pt1.0, 1.0));
    assert!(approx_eq(pb_pt1.1, 0.0));
}

// ── Constraint solving — single profile ───────────────────────────────────────

#[test]
fn horizontal_constraint_equalises_v() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.5)).unwrap();

    sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 0 }).unwrap();
    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    assert!(approx_eq(p0.1, p1.1), "v coords differ: {} vs {}", p0.1, p1.1);
}

#[test]
fn vertical_constraint_equalises_u() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (0.3, 1.0)).unwrap();

    sketch.add_constraint(Constraint::Vertical { profile: pid, seg: 0 }).unwrap();
    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    assert!(approx_eq(p0.0, p1.0), "u coords differ: {} vs {}", p0.0, p1.0);
}

#[test]
fn fixed_length_constraint() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (0.5, 0.0)).unwrap();

    sketch.add_constraint(Constraint::FixedLength { profile: pid, seg: 0, value: 2.0 }).unwrap();
    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    let l = seg_len(p0, p1);
    assert!(approx_eq(l, 2.0), "length = {}", l);
}

#[test]
fn parallel_two_segments() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    // Closed square-ish shape; seg 0 and seg 2 should be parallel (opposite sides).
    let pid = sketch.add_polyline_profile(true);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.1)).unwrap();
    sketch.push_point(pid, (1.0, 1.0)).unwrap();
    sketch.push_point(pid, (0.0, 1.0)).unwrap();

    sketch.add_constraint(Constraint::Parallel { profile: pid, seg_a: 0, seg_b: 2 }).unwrap();
    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    let p2 = point(&sketch, pid, PointId(2));
    let p3 = point(&sketch, pid, PointId(3));

    // Seg 0: p0→p1, seg 2: p2→p3.  Cross product of directions ≈ 0.
    let d0 = (p1.0 - p0.0, p1.1 - p0.1);
    let d2 = (p3.0 - p2.0, p3.1 - p2.1);
    let cross = d0.0 * d2.1 - d0.1 * d2.0;
    assert!(cross.abs() < 1e-5, "cross product = {}", cross);
}

#[test]
fn point_on_origin_constraint() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.5, 0.7)).unwrap();
    sketch.push_point(pid, (1.0, 1.0)).unwrap();

    sketch.add_constraint(Constraint::PointOnOrigin { profile: pid, pt: PointId(0) }).unwrap();
    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    assert!(approx_eq(p0.0, 0.0), "u = {}", p0.0);
    assert!(approx_eq(p0.1, 0.0), "v = {}", p0.1);
}

#[test]
fn square_profile_constraints() {
    // A closed square: 4 points.  Constrain:
    //   - seg 0 horizontal, seg 2 horizontal
    //   - seg 1 vertical,   seg 3 vertical
    //   - equal length seg 0 and seg 1
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(true);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.1)).unwrap();
    sketch.push_point(pid, (1.1, 1.0)).unwrap();
    sketch.push_point(pid, (0.1, 1.1)).unwrap();

    // Fix one point so the system has a unique solution.
    sketch.add_constraint(Constraint::PointOnOrigin { profile: pid, pt: PointId(0) }).unwrap();
    sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 0 }).unwrap();
    sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 2 }).unwrap();
    sketch.add_constraint(Constraint::Vertical   { profile: pid, seg: 1 }).unwrap();
    sketch.add_constraint(Constraint::Vertical   { profile: pid, seg: 3 }).unwrap();
    sketch.add_constraint(Constraint::EqualLength { profile: pid, seg_a: 0, seg_b: 1 }).unwrap();

    let report = sketch.solve();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    let p2 = point(&sketch, pid, PointId(2));
    let p3 = point(&sketch, pid, PointId(3));

    // p0 pinned at origin.
    assert!(approx_eq(p0.0, 0.0) && approx_eq(p0.1, 0.0));
    // seg 0 (p0→p1) horizontal: same v.
    assert!(approx_eq(p0.1, p1.1), "seg0 not horizontal");
    // seg 1 (p1→p2) vertical: same u.
    assert!(approx_eq(p1.0, p2.0), "seg1 not vertical");
    // seg 2 (p2→p3) horizontal: same v.
    assert!(approx_eq(p2.1, p3.1), "seg2 not horizontal");
    // seg 3 (p3→p0) vertical: same u.
    assert!(approx_eq(p3.0, p0.0), "seg3 not vertical");
    // Equal length seg0 and seg1.
    let l0 = seg_len(p0, p1);
    let l1 = seg_len(p1, p2);
    assert!(approx_eq(l0, l1), "side lengths differ: {} vs {}", l0, l1);
}

// ── Cross-profile constraints ─────────────────────────────────────────────────

#[test]
fn cross_parallel_two_segments() {
    let mut sketch = Sketch::new(Plane::xy(), "test");

    let pa = sketch.add_polyline_profile(false);
    sketch.push_point(pa, (0.0, 0.0)).unwrap();
    sketch.push_point(pa, (1.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    sketch.push_point(pb, (0.0, 1.0)).unwrap();
    sketch.push_point(pb, (1.0, 1.3)).unwrap(); // slightly off

    sketch.add_constraint(Constraint::CrossParallel {
        profile_a: pa, seg_a: 0,
        profile_b: pb, seg_b: 0,
    }).unwrap();

    let report = sketch.solve();
    assert!(report.converged);

    let a0 = point(&sketch, pa, PointId(0));
    let a1 = point(&sketch, pa, PointId(1));
    let b0 = point(&sketch, pb, PointId(0));
    let b1 = point(&sketch, pb, PointId(1));

    let da = (a1.0 - a0.0, a1.1 - a0.1);
    let db = (b1.0 - b0.0, b1.1 - b0.1);
    let cross = da.0 * db.1 - da.1 * db.0;
    assert!(cross.abs() < 1e-5, "cross = {}", cross);
}

#[test]
fn cross_equal_length() {
    let mut sketch = Sketch::new(Plane::xy(), "test");

    let pa = sketch.add_polyline_profile(false);
    sketch.push_point(pa, (0.0, 0.0)).unwrap();
    sketch.push_point(pa, (3.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    sketch.push_point(pb, (0.0, 1.0)).unwrap();
    sketch.push_point(pb, (1.0, 1.0)).unwrap();

    sketch.add_constraint(Constraint::CrossEqualLength {
        profile_a: pa, seg_a: 0,
        profile_b: pb, seg_b: 0,
    }).unwrap();

    let report = sketch.solve();
    assert!(report.converged);

    let la = seg_len(point(&sketch, pa, PointId(0)), point(&sketch, pa, PointId(1)));
    let lb = seg_len(point(&sketch, pb, PointId(0)), point(&sketch, pb, PointId(1)));
    assert!(approx_eq(la, lb), "lengths differ: {} vs {}", la, lb);
}

// ── Symmetric (equidistant from two segments) ─────────────────────────────────

#[test]
fn symmetric_point_equidistant() {
    // Two parallel horizontal segments at v=0 and v=2.
    // A point in between should be equidistant (v=1) after the constraint.
    let mut sketch = Sketch::new(Plane::xy(), "test");

    let pa = sketch.add_polyline_profile(false);
    sketch.push_point(pa, (-1.0, 0.0)).unwrap();
    sketch.push_point(pa, (1.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    sketch.push_point(pb, (-1.0, 2.0)).unwrap();
    sketch.push_point(pb, (1.0, 2.0)).unwrap();

    let pc = sketch.add_polyline_profile(false);
    let pt_c = sketch.push_point(pc, (0.0, 0.7)).unwrap();
    sketch.push_point(pc, (1.0, 0.0)).unwrap(); // dummy second point

    sketch.add_constraint(Constraint::Symmetric {
        profile_seg_a: pa, seg_a: 0,
        profile_seg_b: pb, seg_b: 0,
        profile_pt: pc, pt: pt_c,
    }).unwrap();

    sketch.solve();

    let pt = point(&sketch, pc, pt_c);
    // Perpendicular distance to pa (v=0) is pt.1; to pb (v=2) is 2-pt.1.
    // Equal means pt.1 == 2 - pt.1 → pt.1 = 1.0.
    assert!(approx_eq(pt.1, 1.0), "v = {}", pt.1);
}

// ── SymmetricPoints ───────────────────────────────────────────────────────────

#[test]
fn symmetric_points_equal_distance() {
    // P_a = (0,0), P_b = (4,0).  P_c starts at (2, 0.3) (almost equidistant).
    // After solve, dist(P_a, P_c) == dist(P_b, P_c) → P_c.u should be 2.0.
    let mut sketch = Sketch::new(Plane::xy(), "test");

    let pa = sketch.add_polyline_profile(false);
    let pt_a = sketch.push_point(pa, (0.0, 0.0)).unwrap();
    sketch.push_point(pa, (1.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    let pt_b = sketch.push_point(pb, (4.0, 0.0)).unwrap();
    sketch.push_point(pb, (5.0, 0.0)).unwrap();

    let pc = sketch.add_polyline_profile(false);
    let pt_c = sketch.push_point(pc, (2.3, 1.0)).unwrap();
    sketch.push_point(pc, (3.0, 0.0)).unwrap();

    sketch.add_constraint(Constraint::SymmetricPoints {
        profile_a: pa, pt_a,
        profile_b: pb, pt_b,
        profile_c: pc, pt_c,
    }).unwrap();

    sketch.solve();

    let a = point(&sketch, pa, pt_a);
    let b = point(&sketch, pb, pt_b);
    let c = point(&sketch, pc, pt_c);
    let da = seg_len(a, c);
    let db = seg_len(b, c);
    assert!(approx_eq(da, db), "dist(A,C)={} dist(B,C)={}", da, db);
}

// ── Drag ──────────────────────────────────────────────────────────────────────

#[test]
fn drag_with_no_constraints_moves_point() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    let p1 = sketch.push_point(pid, (1.0, 0.0)).unwrap();

    let report = sketch.drag_point(pid, p1, (5.0, 3.0)).unwrap();
    assert!(report.converged);
    let p = point(&sketch, pid, p1);
    assert!(approx_eq(p.0, 5.0) && approx_eq(p.1, 3.0));
}

#[test]
fn drag_constrained_point_respects_horizontal() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    let p1 = sketch.push_point(pid, (1.0, 0.0)).unwrap();

    sketch.add_constraint(Constraint::PointOnOrigin { profile: pid, pt: PointId(0) }).unwrap();
    sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 0 }).unwrap();
    sketch.solve();

    // Drag p1 horizontally; v should stay 0 due to the Horizontal constraint.
    sketch.drag_point(pid, p1, (3.0, 2.0)).unwrap();
    let p = point(&sketch, pid, p1);
    assert!(approx_eq(p.1, 0.0), "v should be 0, got {}", p.1);
}

#[test]
fn drag_with_conflict_reverts_sketch() {
    // PointOnOrigin on pt0, and we try to drag pt0 far away.
    // The pin + PointOnOrigin conflict, so the sketch should revert.
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    let p0 = sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.0)).unwrap();

    sketch.add_constraint(Constraint::PointOnOrigin { profile: pid, pt: p0 }).unwrap();
    sketch.solve();

    let before = point(&sketch, pid, p0);
    sketch.drag_point(pid, p0, (10.0, 10.0)).unwrap();
    let after = point(&sketch, pid, p0);

    // Should have reverted — still at origin.
    assert!(approx_eq(after.0, before.0) && approx_eq(after.1, before.1),
        "pt moved to {:?}", after);
}

// ── Snapshot / restore ────────────────────────────────────────────────────────

#[test]
fn snapshot_restore_round_trip() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.0)).unwrap();

    let snap = sketch.snapshot();

    // Add a second profile.
    sketch.add_polyline_profile(false);
    assert_eq!(sketch.profiles().count(), 2);

    // Restore — only the original profile should remain.
    sketch.restore(snap);
    assert_eq!(sketch.profiles().count(), 1);
}

#[test]
fn snapshot_is_independent_clone() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    let p0 = sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.0)).unwrap();

    let snap = sketch.snapshot();

    // Move a point AFTER taking the snapshot.
    sketch.move_point(pid, p0, (99.0, 99.0)).unwrap();

    // Restoring the snapshot should revert the move.
    sketch.restore(snap);
    let pt = point(&sketch, pid, p0);
    assert!(approx_eq(pt.0, 0.0) && approx_eq(pt.1, 0.0), "got {:?}", pt);
}

// ── Remove profile cascades constraints ───────────────────────────────────────

#[test]
fn remove_profile_removes_its_constraints() {
    let mut sketch = Sketch::new(Plane::xy(), "test");

    let pa = sketch.add_polyline_profile(false);
    sketch.push_point(pa, (0.0, 0.0)).unwrap();
    sketch.push_point(pa, (1.0, 0.0)).unwrap();

    let pb = sketch.add_polyline_profile(false);
    sketch.push_point(pb, (0.0, 1.0)).unwrap();
    sketch.push_point(pb, (1.0, 1.0)).unwrap();

    sketch.add_constraint(Constraint::Horizontal { profile: pa, seg: 0 }).unwrap();
    sketch.add_constraint(Constraint::CrossParallel {
        profile_a: pa, seg_a: 0,
        profile_b: pb, seg_b: 0,
    }).unwrap();

    assert_eq!(sketch.constraints().count(), 2);

    sketch.remove_profile(pa).unwrap();

    // Both constraints reference pa, so both should be gone.
    assert_eq!(sketch.constraints().count(), 0);
    assert_eq!(sketch.profiles().count(), 1);
}

// ── Error paths ───────────────────────────────────────────────────────────────

#[test]
fn add_constraint_invalid_profile_errors() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let fake = ProfileId(999);
    let result = sketch.add_constraint(Constraint::Horizontal { profile: fake, seg: 0 });
    assert!(matches!(result, Err(SketchError::NoSuchProfile(_))));
}

#[test]
fn move_point_invalid_pt_errors() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    let result = sketch.move_point(pid, PointId(99), (1.0, 1.0));
    assert!(matches!(result, Err(SketchError::NoSuchPoint { .. })));
}

#[test]
fn move_point_invalid_profile_errors() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let result = sketch.move_point(ProfileId(42), PointId(0), (1.0, 1.0));
    assert!(matches!(result, Err(SketchError::NoSuchProfile(_))));
}

// ── Conflicting constraints ───────────────────────────────────────────────────

#[test]
fn conflicting_constraints_reported() {
    // PointOnOrigin + PointFixed at (1,1) on the same point — conflict.
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    let p0 = sketch.push_point(pid, (0.5, 0.5)).unwrap();
    sketch.push_point(pid, (1.0, 0.0)).unwrap();

    sketch.add_constraint(Constraint::PointOnOrigin { profile: pid, pt: p0 }).unwrap();
    sketch.add_constraint(Constraint::PointFixed { profile: pid, pt: p0, u: 5.0, v: 5.0 }).unwrap();

    let report = sketch.solve();
    assert!(!report.converged, "should have detected conflict");
}

// ── Arc geometry ──────────────────────────────────────────────────────────────

#[test]
fn arc_center_reproject_after_endpoint_drag() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    // Horizontal chord (0,0)→(2,0), center at (1,3).
    let pid = sketch.add_arc_profile((0.0, 0.0), (2.0, 0.0), (1.0, 3.0));

    // Move the end endpoint; center should be re-projected.
    sketch.move_point(pid, PointId(1), (4.0, 0.0)).unwrap();
    sketch.reproject_arc_center(pid).unwrap();

    // New chord is (0,0)→(4,0). Bisector is x=2. Center_v stays ≈ 3.
    let center = point(&sketch, pid, PointId(2));
    assert!(approx_eq(center.0, 2.0), "center_u = {}", center.0);
}

// ── solve_profile_only ────────────────────────────────────────────────────────

#[test]
fn solve_profile_only_non_existent_profile_errors() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let result = sketch.solve_profile_only(ProfileId(999));
    assert!(matches!(result, Err(SketchError::NoSuchProfile(_))));
}

#[test]
fn solve_profile_only_solves_single_profile() {
    let mut sketch = Sketch::new(Plane::xy(), "test");
    let pid = sketch.add_polyline_profile(false);
    sketch.push_point(pid, (0.0, 0.0)).unwrap();
    sketch.push_point(pid, (1.0, 0.5)).unwrap();

    sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 0 }).unwrap();

    let report = sketch.solve_profile_only(pid).unwrap();
    assert!(report.converged);

    let p0 = point(&sketch, pid, PointId(0));
    let p1 = point(&sketch, pid, PointId(1));
    assert!(approx_eq(p0.1, p1.1));
}
