use brep_sketch::{solve_constraints, SketchConstraint};
use criterion::{black_box, criterion_group, criterion_main, Criterion};

// ── Drag scenario ─────────────────────────────────────────────────────────────
//
// Simulates dragging a vertex in a sketch constrained with a pin + downstream
// coincident + horizontal chain — the primary beneficiary of propagation.
//
// Layout: pts 0-3 form a rectangle-like polygon.
//   seg0: pt0→pt1  (Horizontal)
//   seg1: pt1→pt2  (Vertical)
//   seg2: pt2→pt3  (Horizontal)
//   seg3: pt3→pt0  (Vertical)
//
// Drag adds a PointFixed pin on pt0, which propagates through the chain.

fn drag_scenario(c: &mut Criterion) {
    let constraints = vec![
        SketchConstraint::PointFixed { pt: 0, x: 0.0, y: 0.0 },
        SketchConstraint::Coincident { pt_a: 0, pt_b: 1 },
        SketchConstraint::Horizontal { seg: 1 }, // pt1 → pt2
        SketchConstraint::Vertical { seg: 2 },   // pt2 → pt3
    ];

    c.bench_function("drag_pin_coincident_horizontal_chain", |b| {
        b.iter(|| {
            let mut pts = black_box(vec![
                [1.0_f64, 1.0],
                [5.0, 3.0],
                [7.0, 3.0],
                [7.0, 8.0],
            ]);
            solve_constraints(&mut pts, &constraints, 4)
        })
    });
}

// ── Pure Newton scenario ──────────────────────────────────────────────────────
//
// Constraints that propagation cannot handle — all fall through to Newton.
// Measures overhead of the propagation pass on a purely Newton workload.
//
// Layout: 4-point polygon.
//   seg0: pt0→pt1  Parallel to seg2
//   seg1: pt1→pt2  Perpendicular to seg0
//   seg2: pt2→pt3
//   seg3: pt3→pt0  EqualLength to seg1

fn pure_newton_scenario(c: &mut Criterion) {
    let constraints = vec![
        SketchConstraint::Parallel { seg_a: 0, seg_b: 2 },
        SketchConstraint::Perpendicular { seg_a: 0, seg_b: 1 },
        SketchConstraint::EqualLength { seg_a: 1, seg_b: 3 },
    ];

    c.bench_function("pure_newton_parallel_perp_equal", |b| {
        b.iter(|| {
            let mut pts = black_box(vec![
                [0.0_f64, 0.0],
                [3.0, 0.2],
                [3.1, 2.0],
                [0.1, 1.9],
            ]);
            solve_constraints(&mut pts, &constraints, 4)
        })
    });
}

// ── Mixed scenario ────────────────────────────────────────────────────────────
//
// Realistic sketch: origin anchor + axis constraints + Newton constraints.
// Propagation handles the anchors; Newton handles the rest.

fn mixed_scenario(c: &mut Criterion) {
    let constraints = vec![
        SketchConstraint::PointOnOrigin { pt: 0 },
        SketchConstraint::PointOnXAxis { pt: 1 },
        SketchConstraint::Parallel { seg_a: 0, seg_b: 2 },
        SketchConstraint::FixedLength { seg: 0, value: 3.0 },
        SketchConstraint::EqualLength { seg_a: 1, seg_b: 3 },
    ];

    c.bench_function("mixed_propagation_and_newton", |b| {
        b.iter(|| {
            let mut pts = black_box(vec![
                [0.1_f64, 0.1],
                [3.0, 0.2],
                [3.1, 2.0],
                [0.1, 1.9],
            ]);
            solve_constraints(&mut pts, &constraints, 4)
        })
    });
}

// ── Medium Newton scenario (12 pts, nv=24) ────────────────────────────────────
//
// 12-point polygon arranged in a near-regular 12-gon (radius 5).
// All constraints are Newton-only (no propagation): opposite segment pairs
// constrained parallel and equal-length, plus a fixed-length anchor.
//
// Constraint count: 13  (6 Parallel + 6 EqualLength + 1 FixedLength)
// Variables: nv = 24

fn medium_newton_scenario(c: &mut Criterion) {
    // Near-regular 12-gon with slight perturbations so Newton has real work to do.
    // pt[k] ≈ [5*cos(k*π/6), 5*sin(k*π/6)] + small noise.
    let pts_base: Vec<[f64; 2]> = vec![
        [ 5.04,  0.10],  // 0
        [ 4.28,  2.55],  // 1
        [ 2.48,  4.40],  // 2
        [-0.08,  5.03],  // 3
        [-2.55,  4.28],  // 4
        [-4.38,  2.52],  // 5
        [-5.03, -0.07],  // 6
        [-4.30, -2.48],  // 7
        [-2.52, -4.36],  // 8
        [ 0.06, -5.02],  // 9
        [ 2.50, -4.30],  // 10
        [ 4.35, -2.50],  // 11
    ];

    // In a regular 12-gon, segment k is antiparallel to segment k+6
    // (cross product of direction vectors = 0 ↔ parallel constraint satisfied).
    let constraints = vec![
        SketchConstraint::Parallel     { seg_a: 0, seg_b: 6  },
        SketchConstraint::Parallel     { seg_a: 1, seg_b: 7  },
        SketchConstraint::Parallel     { seg_a: 2, seg_b: 8  },
        SketchConstraint::Parallel     { seg_a: 3, seg_b: 9  },
        SketchConstraint::Parallel     { seg_a: 4, seg_b: 10 },
        SketchConstraint::Parallel     { seg_a: 5, seg_b: 11 },
        SketchConstraint::EqualLength  { seg_a: 0, seg_b: 6  },
        SketchConstraint::EqualLength  { seg_a: 1, seg_b: 7  },
        SketchConstraint::EqualLength  { seg_a: 2, seg_b: 8  },
        SketchConstraint::EqualLength  { seg_a: 3, seg_b: 9  },
        SketchConstraint::EqualLength  { seg_a: 4, seg_b: 10 },
        SketchConstraint::EqualLength  { seg_a: 5, seg_b: 11 },
        SketchConstraint::FixedLength  { seg: 0, value: 2.588 },
    ];

    c.bench_function("medium_newton_12pt", |b| {
        b.iter(|| {
            let mut pts = black_box(pts_base.clone());
            solve_constraints(&mut pts, &constraints, 12)
        })
    });
}

// ── Large Newton scenario (24 pts, nv=48) ────────────────────────────────────
//
// 24-point polygon arranged in a near-regular 24-gon (radius 5).
// All constraints are Newton-only.
//
// Constraint count: 25  (12 Parallel + 12 EqualLength + 1 FixedLength)
// Variables: nv = 48

fn large_newton_scenario(c: &mut Criterion) {
    use std::f64::consts::PI;

    // Near-regular 24-gon, slight noise on each point.
    let noise = [
        0.05, -0.08, 0.03, -0.06, 0.07, -0.04, 0.09, -0.02,
        0.06, -0.07, 0.04, -0.05, 0.08, -0.03, 0.05, -0.09,
        0.02, -0.06, 0.07, -0.04, 0.03, -0.08, 0.06, -0.05,
    ];
    let pts_base: Vec<[f64; 2]> = (0..24)
        .map(|k| {
            let angle = k as f64 * PI / 12.0;
            [5.0 * angle.cos() + noise[k], 5.0 * angle.sin() + noise[(k + 3) % 24]]
        })
        .collect();

    // Opposite segment pairs (k and k+12) are parallel and equal-length.
    let mut constraints: Vec<SketchConstraint> = (0..12)
        .flat_map(|k| {
            [
                SketchConstraint::Parallel    { seg_a: k,     seg_b: k + 12 },
                SketchConstraint::EqualLength { seg_a: k,     seg_b: k + 12 },
            ]
        })
        .collect();
    constraints.push(SketchConstraint::FixedLength { seg: 0, value: 1.305 });

    c.bench_function("large_newton_24pt", |b| {
        b.iter(|| {
            let mut pts = black_box(pts_base.clone());
            solve_constraints(&mut pts, &constraints, 24)
        })
    });
}

criterion_group!(
    benches,
    drag_scenario,
    pure_newton_scenario,
    mixed_scenario,
    medium_newton_scenario,
    large_newton_scenario,
);
criterion_main!(benches);
