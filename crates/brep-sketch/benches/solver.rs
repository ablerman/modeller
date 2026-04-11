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

criterion_group!(benches, drag_scenario, pure_newton_scenario, mixed_scenario);
criterion_main!(benches);
