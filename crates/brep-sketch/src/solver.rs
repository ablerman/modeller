//! Gauss-Newton constraint solver for 2D sketch geometry.
//!
//! The solver operates on a flat array of 2D point coordinates
//! `[[u, v], ...]` and adjusts them to satisfy a set of geometric
//! constraints using Gauss-Newton iteration with Levenberg-Marquardt
//! damping.

use nalgebra::{DMatrix, DVector};

use crate::constraints::SketchConstraint;

// ── Geometric propagation ─────────────────────────────────────────────────────

/// Tracks how much of a point's position is known from direct constraints.
#[derive(Clone, Copy)]
enum Determined {
    Free,
    U(f64),         // u fixed (e.g. PointOnYAxis)
    V(f64),         // v fixed (e.g. PointOnXAxis)
    Both(f64, f64), // fully determined
}

impl Determined {
    fn known_u(self) -> Option<f64> {
        match self {
            Determined::U(u) | Determined::Both(u, _) => Some(u),
            _ => None,
        }
    }

    fn known_v(self) -> Option<f64> {
        match self {
            Determined::V(v) | Determined::Both(_, v) => Some(v),
            _ => None,
        }
    }

    /// Merge new knowledge into existing knowledge.
    fn merge(self, other: Determined) -> Determined {
        let u = self.known_u().or_else(|| other.known_u());
        let v = self.known_v().or_else(|| other.known_v());
        match (u, v) {
            (Some(u), Some(v)) => Determined::Both(u, v),
            (Some(u), None)    => Determined::U(u),
            (None,    Some(v)) => Determined::V(v),
            (None,    None)    => Determined::Free,
        }
    }
}

/// Apply one pass over the constraint list, resolving any constraint whose
/// result can be computed directly from already-determined points.
/// Returns `true` if at least one constraint was newly applied.
fn propagate_once(
    pts: &mut Vec<[f64; 2]>,
    constraints: &[SketchConstraint],
    n: usize,
    det: &mut Vec<Determined>,
) -> bool {
    let mut progress = false;

    for c in constraints {
        match c {
            SketchConstraint::PointFixed { pt, x, y } => {
                // Skip if already fully determined to a different value — conflict
                // for Newton to detect.  Only apply if free or already matching.
                match det[*pt] {
                    Determined::Both(u, v) if u == *x && v == *y => {} // already done
                    Determined::Both(_, _) => {}                        // conflict, skip
                    _ => {
                        pts[*pt] = [*x, *y];
                        det[*pt] = Determined::Both(*x, *y);
                        progress = true;
                    }
                }
            }
            SketchConstraint::PointOnOrigin { pt } => {
                match det[*pt] {
                    Determined::Both(u, v) if u == 0.0 && v == 0.0 => {}
                    Determined::Both(_, _) => {} // conflict, skip
                    _ => {
                        pts[*pt] = [0.0, 0.0];
                        det[*pt] = Determined::Both(0.0, 0.0);
                        progress = true;
                    }
                }
            }
            SketchConstraint::PointOnXAxis { pt } => {
                match det[*pt].known_v() {
                    Some(v) if v == 0.0 => {} // already satisfied
                    Some(_) => {}              // conflict with other V constraint, skip
                    None => {
                        pts[*pt][1] = 0.0;
                        det[*pt] = det[*pt].merge(Determined::V(0.0));
                        progress = true;
                    }
                }
            }
            SketchConstraint::PointOnYAxis { pt } => {
                match det[*pt].known_u() {
                    Some(u) if u == 0.0 => {}
                    Some(_) => {}
                    None => {
                        pts[*pt][0] = 0.0;
                        det[*pt] = det[*pt].merge(Determined::U(0.0));
                        progress = true;
                    }
                }
            }
            SketchConstraint::Coincident { pt_a, pt_b } => {
                match (det[*pt_a], det[*pt_b]) {
                    (Determined::Both(u, v), other) => {
                        if matches!(other, Determined::Both(bu, bv) if bu == u && bv == v) {
                            // already matching
                        } else if matches!(other, Determined::Both(_, _)) {
                            // conflict, skip
                        } else {
                            pts[*pt_b] = [u, v];
                            det[*pt_b] = Determined::Both(u, v);
                            progress = true;
                        }
                    }
                    (_, Determined::Both(u, v)) => {
                        if matches!(det[*pt_a], Determined::Both(au, av) if au == u && av == v) {
                            // already matching
                        } else if matches!(det[*pt_a], Determined::Both(_, _)) {
                            // conflict, skip
                        } else {
                            pts[*pt_a] = [u, v];
                            det[*pt_a] = Determined::Both(u, v);
                            progress = true;
                        }
                    }
                    _ => {}
                }
            }
            SketchConstraint::Horizontal { seg } => {
                let s = *seg;
                let sn = (s + 1) % n;
                match (det[s], det[sn]) {
                    (Determined::Both(_, v), _) => match det[sn].known_v() {
                        Some(sv) if sv == v => {}
                        Some(_) => {} // conflict
                        None => {
                            pts[sn][1] = v;
                            det[sn] = det[sn].merge(Determined::V(v));
                            progress = true;
                        }
                    },
                    (_, Determined::Both(_, v)) => match det[s].known_v() {
                        Some(sv) if sv == v => {}
                        Some(_) => {}
                        None => {
                            pts[s][1] = v;
                            det[s] = det[s].merge(Determined::V(v));
                            progress = true;
                        }
                    },
                    _ => {}
                }
            }
            SketchConstraint::Vertical { seg } => {
                let s = *seg;
                let sn = (s + 1) % n;
                match (det[s], det[sn]) {
                    (Determined::Both(u, _), _) => match det[sn].known_u() {
                        Some(su) if su == u => {}
                        Some(_) => {}
                        None => {
                            pts[sn][0] = u;
                            det[sn] = det[sn].merge(Determined::U(u));
                            progress = true;
                        }
                    },
                    (_, Determined::Both(u, _)) => match det[s].known_u() {
                        Some(su) if su == u => {}
                        Some(_) => {}
                        None => {
                            pts[s][0] = u;
                            det[s] = det[s].merge(Determined::U(u));
                            progress = true;
                        }
                    },
                    _ => {}
                }
            }
            _ => {}
        }
    }

    progress
}

/// Run propagation to a fixed point, seeding with direct constraints.
fn propagate(pts: &mut Vec<[f64; 2]>, constraints: &[SketchConstraint], n: usize) {
    let mut det = vec![Determined::Free; pts.len()];
    while propagate_once(pts, constraints, n, &mut det) {}
}

/// Result of a constraint solve.
#[derive(Debug, PartialEq)]
pub enum SolveResult {
    /// All constraints satisfied within tolerance.
    Ok,
    /// Solver did not converge — constraints may be conflicting or
    /// over-constrained.  The input `pts` array is left unchanged.
    Conflict,
}

/// Solve the constraints in-place.
///
/// `pts` contains one `[u, v]` entry per sketch vertex.
/// `n_pts` is the total number of vertices (used for modular segment
/// indexing: segment `i` goes from `pts[i]` to `pts[(i+1) % n_pts]`).
///
/// On `SolveResult::Ok` the array is updated.
/// On `SolveResult::Conflict` the array is left unchanged.
pub fn solve_constraints(
    pts: &mut Vec<[f64; 2]>,
    constraints: &[SketchConstraint],
    n_pts: usize,
) -> SolveResult {
    if constraints.is_empty() || pts.len() < 2 {
        return SolveResult::Ok;
    }

    // Save original positions so we can restore on conflict (propagation
    // modifies pts in-place before Newton runs).
    let pts_backup: Vec<[f64; 2]> = pts.clone();

    // Geometric propagation pre-pass: resolve closed-form constraints first.
    propagate(pts, constraints, n_pts);

    // Flatten [u,v] pairs into a single Vec<f64>.
    let mut vars: Vec<f64> = pts.iter().flat_map(|p| [p[0], p[1]]).collect();

    const MAX_ITER: usize = 50;
    const CONVERGENCE_TOL: f64 = 1e-8;
    const LAMBDA: f64 = 1e-4;

    let nv = vars.len();
    let m = count_residuals(constraints);

    // Pre-allocated working buffers — reused across iterations.
    let mut r = vec![0.0_f64; m];
    let mut j_flat = vec![0.0_f64; m * nv]; // row-major J[m × nv]

    for _ in 0..MAX_ITER {
        fill_residuals(&vars, constraints, n_pts, &mut r);
        let norm_sq: f64 = r.iter().map(|x| x * x).sum();
        if norm_sq.sqrt() < CONVERGENCE_TOL {
            break;
        }

        // Build analytical Jacobian.
        j_flat.fill(0.0);
        fill_jacobian(&vars, constraints, n_pts, nv, &mut j_flat);

        // JᵀJ + λI and Jᵀr via nalgebra (optimised BLAS path).
        let j = DMatrix::from_row_slice(m, nv, &j_flat);
        let r_vec = DVector::from_row_slice(&r);
        let jt_r: DVector<f64> = j.tr_mul(&r_vec);
        let mut jt_j: DMatrix<f64> = j.tr_mul(&j);
        for i in 0..nv {
            jt_j[(i, i)] += LAMBDA;
        }

        // JᵀJ + λI is symmetric positive definite; Cholesky is ~2× faster
        // than full inversion and avoids computing the n×n inverse matrix.
        let Some(chol) = nalgebra::linalg::Cholesky::new(jt_j) else { break };
        let delta = chol.solve(&jt_r);

        for i in 0..nv {
            vars[i] -= delta[i];
        }
    }

    // Check final residual.
    fill_residuals(&vars, constraints, n_pts, &mut r);
    let final_norm: f64 = r.iter().map(|x| x * x).sum::<f64>().sqrt();

    if final_norm > 1e-4 {
        *pts = pts_backup;
        return SolveResult::Conflict;
    }

    // Write back solved positions.
    for (i, chunk) in vars.chunks(2).enumerate() {
        pts[i] = [chunk[0], chunk[1]];
    }
    SolveResult::Ok
}

// ── High-level apply ──────────────────────────────────────────────────────────

/// Result of [`apply_constraints`].
pub struct ApplyResult {
    /// `true` if the solver could not satisfy all constraints.
    pub conflict: bool,
    /// Parallel to the constraint slice — `true` for each constraint that
    /// contributed to the conflict.  Empty when `conflict` is `false`.
    pub violated: Vec<bool>,
}

/// Run the constraint solver and, on failure, identify the conflicting constraints.
///
/// On success the `pts` array is updated in place.
/// On conflict `pts` is left unchanged and `violated` identifies culprits.
pub fn apply_constraints(
    pts: &mut Vec<[f64; 2]>,
    constraints: &[SketchConstraint],
    n_pts: usize,
) -> ApplyResult {
    if constraints.is_empty() || pts.len() < 2 {
        return ApplyResult { conflict: false, violated: vec![] };
    }
    match solve_constraints(pts, constraints, n_pts) {
        SolveResult::Ok => ApplyResult { conflict: false, violated: vec![] },
        SolveResult::Conflict => {
            let violated = find_conflicting_constraints(pts, constraints, n_pts);
            ApplyResult { conflict: true, violated }
        }
    }
}

/// For each constraint, check whether removing it allows the remaining system
/// to converge.  Returns a parallel `Vec<bool>` — `true` means that constraint
/// is involved in the conflict.
fn find_conflicting_constraints(
    pts: &[[f64; 2]],
    constraints: &[SketchConstraint],
    n_pts: usize,
) -> Vec<bool> {
    constraints
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let reduced: Vec<SketchConstraint> = constraints
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, c)| c.clone())
                .collect();
            if reduced.is_empty() {
                return true;
            }
            let mut pts_copy: Vec<[f64; 2]> = pts.to_vec();
            solve_constraints(&mut pts_copy, &reduced, n_pts) == SolveResult::Ok
        })
        .collect()
}

/// Number of scalar residuals for the given constraint list.
/// `Coincident` produces 2; every other constraint produces 1.
fn count_residuals(constraints: &[SketchConstraint]) -> usize {
    constraints
        .iter()
        .map(|c| match c {
            SketchConstraint::Coincident { .. }
            | SketchConstraint::PointFixed { .. }
            | SketchConstraint::PointOnOrigin { .. } => 2,
            _ => 1,
        })
        .sum()
}

// (helper used below — returns u, v coords of a variable-index point)
#[inline]
fn pv(vars: &[f64], i: usize) -> (f64, f64) { (vars[2 * i], vars[2 * i + 1]) }

/// Fill the residual vector in-place.
///
/// The constraint order determines which `r[row]` each residual lands in.
/// `fill_jacobian` must iterate constraints in the same order.
fn fill_residuals(
    vars: &[f64],
    constraints: &[SketchConstraint],
    n: usize,
    r: &mut [f64],
) {
    let pt = |i: usize| -> (f64, f64) { (vars[2 * i], vars[2 * i + 1]) };
    let seg_dir = |s: usize| -> (f64, f64) {
        let (ax, ay) = pt(s);
        let (bx, by) = pt((s + 1) % n);
        (bx - ax, by - ay)
    };

    let mut row = 0usize;
    for c in constraints {
        match c {
            SketchConstraint::Parallel { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                r[row] = ax * by - ay * bx;
                row += 1;
            }
            SketchConstraint::Perpendicular { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                r[row] = ax * bx + ay * by;
                row += 1;
            }
            SketchConstraint::Angle { seg_a, seg_b, degrees } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                let la = (ax * ax + ay * ay).sqrt().max(1e-12);
                let lb = (bx * bx + by * by).sqrt().max(1e-12);
                let cos_actual = (ax * bx + ay * by) / (la * lb);
                let cos_target = degrees.to_radians().cos();
                r[row] = cos_actual - cos_target;
                row += 1;
            }
            SketchConstraint::Horizontal { seg } => {
                let (_, dy) = seg_dir(*seg);
                r[row] = dy;
                row += 1;
            }
            SketchConstraint::Vertical { seg } => {
                let (dx, _) = seg_dir(*seg);
                r[row] = dx;
                row += 1;
            }
            SketchConstraint::EqualLength { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                r[row] = ax * ax + ay * ay - bx * bx - by * by;
                row += 1;
            }
            SketchConstraint::Coincident { pt_a, pt_b } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                r[row] = ax - bx;
                r[row + 1] = ay - by;
                row += 2;
            }
            SketchConstraint::PointOnLine { pt: pi, seg } => {
                // r = (u_pt - u_a)*(v_b - v_a) - (v_pt - v_a)*(u_b - u_a)
                let (u_a, v_a) = pt(*seg);
                let (u_b, v_b) = pt((*seg + 1) % n);
                let (u_p, v_p) = pt(*pi);
                r[row] = (u_p - u_a) * (v_b - v_a) - (v_p - v_a) * (u_b - u_a);
                row += 1;
            }
            SketchConstraint::FixedLength { seg, value } => {
                let (dx, dy) = seg_dir(*seg);
                r[row] = dx * dx + dy * dy - value * value;
                row += 1;
            }
            SketchConstraint::PointDistance { pt_a, pt_b, value } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                let dx = bx - ax;
                let dy = by - ay;
                r[row] = dx * dx + dy * dy - value * value;
                row += 1;
            }
            SketchConstraint::PointFixed { pt: pi, x, y } => {
                let (pu, pv) = pt(*pi);
                r[row] = pu - x;
                r[row + 1] = pv - y;
                row += 2;
            }
            SketchConstraint::PointOnOrigin { pt: pi } => {
                let (pu, pv) = pt(*pi);
                r[row] = pu;
                r[row + 1] = pv;
                row += 2;
            }
            SketchConstraint::PointOnXAxis { pt: pi } => {
                let (_, pv) = pt(*pi);
                r[row] = pv;
                row += 1;
            }
            SketchConstraint::PointOnYAxis { pt: pi } => {
                let (pu, _) = pt(*pi);
                r[row] = pu;
                row += 1;
            }
            SketchConstraint::HorizontalPair { pt_a, pt_b, perp_u, perp_v } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                r[row] = perp_u * (bx - ax) + perp_v * (by - ay);
                row += 1;
            }
            SketchConstraint::VerticalPair { pt_a, pt_b, perp_u, perp_v } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                r[row] = perp_u * (bx - ax) + perp_v * (by - ay);
                row += 1;
            }
            SketchConstraint::PointOnCircle { pt: pi, center_u, center_v, radius } => {
                let (pu, pv) = pt(*pi);
                let du = pu - center_u;
                let dv = pv - center_v;
                r[row] = du * du + dv * dv - radius * radius;
                row += 1;
            }
        }
    }
}

/// Fill the Jacobian analytically (row-major J[m × nv], pre-zeroed).
///
/// For each constraint `c` mapped to residual row `row`, we write the
/// partial derivative ∂r_row/∂vars[col] into `j_flat[row*nv + col]`.
/// All partial derivatives are exact closed-form expressions — no
/// finite-difference perturbation is needed.
fn fill_jacobian(
    vars: &[f64],
    constraints: &[SketchConstraint],
    n: usize,
    nv: usize,
    j_flat: &mut [f64],
) {
    let pt = |i: usize| -> (f64, f64) { (vars[2 * i], vars[2 * i + 1]) };
    let seg_dir = |s: usize| -> (f64, f64) {
        let (ax, ay) = pt(s);
        let (bx, by) = pt((s + 1) % n);
        (bx - ax, by - ay)
    };

    let mut row = 0usize;
    for c in constraints {
        match c {
            SketchConstraint::Horizontal { seg } => {
                // r = v[(s+1)%n] - v[s]
                let s = *seg;
                let sn = (s + 1) % n;
                j_flat[row * nv + 2 * s + 1] = -1.0;
                j_flat[row * nv + 2 * sn + 1] = 1.0;
                row += 1;
            }
            SketchConstraint::Vertical { seg } => {
                // r = u[(s+1)%n] - u[s]
                let s = *seg;
                let sn = (s + 1) % n;
                j_flat[row * nv + 2 * s] = -1.0;
                j_flat[row * nv + 2 * sn] = 1.0;
                row += 1;
            }
            SketchConstraint::Parallel { seg_a, seg_b } => {
                // r = dxa*dyb - dya*dxb
                // ∂r/∂u[a]  = -dyb,  ∂r/∂u[an] = +dyb
                // ∂r/∂v[a]  = +dxb,  ∂r/∂v[an] = -dxb
                // ∂r/∂u[b]  = +dya,  ∂r/∂u[bn] = -dya
                // ∂r/∂v[b]  = -dxa,  ∂r/∂v[bn] = +dxa
                let (dxa, dya) = seg_dir(*seg_a);
                let (dxb, dyb) = seg_dir(*seg_b);
                let a = *seg_a;
                let an = (a + 1) % n;
                let b = *seg_b;
                let bn = (b + 1) % n;
                // Use += to handle the degenerate case seg_a == seg_b (shared indices).
                j_flat[row * nv + 2 * a] += -dyb;
                j_flat[row * nv + 2 * an] += dyb;
                j_flat[row * nv + 2 * a + 1] += dxb;
                j_flat[row * nv + 2 * an + 1] += -dxb;
                j_flat[row * nv + 2 * b] += dya;
                j_flat[row * nv + 2 * bn] += -dya;
                j_flat[row * nv + 2 * b + 1] += -dxa;
                j_flat[row * nv + 2 * bn + 1] += dxa;
                row += 1;
            }
            SketchConstraint::Perpendicular { seg_a, seg_b } => {
                // r = dxa*dxb + dya*dyb
                // ∂r/∂u[a]  = -dxb,  ∂r/∂u[an] = +dxb
                // ∂r/∂v[a]  = -dyb,  ∂r/∂v[an] = +dyb
                // ∂r/∂u[b]  = -dxa,  ∂r/∂u[bn] = +dxa
                // ∂r/∂v[b]  = -dya,  ∂r/∂v[bn] = +dya
                let (dxa, dya) = seg_dir(*seg_a);
                let (dxb, dyb) = seg_dir(*seg_b);
                let a = *seg_a;
                let an = (a + 1) % n;
                let b = *seg_b;
                let bn = (b + 1) % n;
                j_flat[row * nv + 2 * a] += -dxb;
                j_flat[row * nv + 2 * an] += dxb;
                j_flat[row * nv + 2 * a + 1] += -dyb;
                j_flat[row * nv + 2 * an + 1] += dyb;
                j_flat[row * nv + 2 * b] += -dxa;
                j_flat[row * nv + 2 * bn] += dxa;
                j_flat[row * nv + 2 * b + 1] += -dya;
                j_flat[row * nv + 2 * bn + 1] += dya;
                row += 1;
            }
            SketchConstraint::Angle { seg_a, seg_b, degrees: _ } => {
                // r = cos(actual) - cos(target);  cos(target) is constant.
                // cos = dot / (la * lb)
                // ∂cos/∂dxa = (dxb*la² - dot*dxa) / (la³*lb)
                // ∂cos/∂dya = (dyb*la² - dot*dya) / (la³*lb)
                // ∂cos/∂dxb = (dxa*lb² - dot*dxb) / (la*lb³)
                // ∂cos/∂dyb = (dya*lb² - dot*dyb) / (la*lb³)
                let (dxa, dya) = seg_dir(*seg_a);
                let (dxb, dyb) = seg_dir(*seg_b);
                let la2 = dxa * dxa + dya * dya;
                let lb2 = dxb * dxb + dyb * dyb;
                let la = la2.sqrt().max(1e-12);
                let lb = lb2.sqrt().max(1e-12);
                let dot = dxa * dxb + dya * dyb;
                let la3lb = la2 * la * lb;
                let lalb3 = la * lb2 * lb;
                let d_dxa = (dxb * la2 - dot * dxa) / la3lb;
                let d_dya = (dyb * la2 - dot * dya) / la3lb;
                let d_dxb = (dxa * lb2 - dot * dxb) / lalb3;
                let d_dyb = (dya * lb2 - dot * dyb) / lalb3;
                let a = *seg_a;
                let an = (a + 1) % n;
                let b = *seg_b;
                let bn = (b + 1) % n;
                // dxa = u[an]-u[a]  →  ∂/∂u[a] = -1,  ∂/∂u[an] = +1
                j_flat[row * nv + 2 * a] += -d_dxa;
                j_flat[row * nv + 2 * an] += d_dxa;
                j_flat[row * nv + 2 * a + 1] += -d_dya;
                j_flat[row * nv + 2 * an + 1] += d_dya;
                j_flat[row * nv + 2 * b] += -d_dxb;
                j_flat[row * nv + 2 * bn] += d_dxb;
                j_flat[row * nv + 2 * b + 1] += -d_dyb;
                j_flat[row * nv + 2 * bn + 1] += d_dyb;
                row += 1;
            }
            SketchConstraint::EqualLength { seg_a, seg_b } => {
                // r = |a|² - |b|² = dxa²+dya² - dxb²-dyb²
                // ∂r/∂u[a]  = -2*dxa,  ∂r/∂u[an] = +2*dxa
                // ∂r/∂v[a]  = -2*dya,  ∂r/∂v[an] = +2*dya
                // ∂r/∂u[b]  = +2*dxb,  ∂r/∂u[bn] = -2*dxb
                // ∂r/∂v[b]  = +2*dyb,  ∂r/∂v[bn] = -2*dyb
                let (dxa, dya) = seg_dir(*seg_a);
                let (dxb, dyb) = seg_dir(*seg_b);
                let a = *seg_a;
                let an = (a + 1) % n;
                let b = *seg_b;
                let bn = (b + 1) % n;
                j_flat[row * nv + 2 * a] += -2.0 * dxa;
                j_flat[row * nv + 2 * an] += 2.0 * dxa;
                j_flat[row * nv + 2 * a + 1] += -2.0 * dya;
                j_flat[row * nv + 2 * an + 1] += 2.0 * dya;
                j_flat[row * nv + 2 * b] += 2.0 * dxb;
                j_flat[row * nv + 2 * bn] += -2.0 * dxb;
                j_flat[row * nv + 2 * b + 1] += 2.0 * dyb;
                j_flat[row * nv + 2 * bn + 1] += -2.0 * dyb;
                row += 1;
            }
            SketchConstraint::Coincident { pt_a, pt_b } => {
                // r0 = u[a] - u[b],  r1 = v[a] - v[b]
                let a = *pt_a;
                let b = *pt_b;
                j_flat[row * nv + 2 * a] = 1.0;
                j_flat[row * nv + 2 * b] = -1.0;
                j_flat[(row + 1) * nv + 2 * a + 1] = 1.0;
                j_flat[(row + 1) * nv + 2 * b + 1] = -1.0;
                row += 2;
            }
            SketchConstraint::PointOnLine { pt: pi, seg } => {
                // r = (u_p - u_a)*(v_b - v_a) - (v_p - v_a)*(u_b - u_a)
                // Let: dx = u_b - u_a, dy = v_b - v_a, px = u_p - u_a, py = v_p - v_a
                // ∂r/∂u_p  =  dy          ∂r/∂v_p  = -dx
                // ∂r/∂u_a  = -dy + py     ∂r/∂v_a  = -px + dx
                // ∂r/∂u_b  = -py          ∂r/∂v_b  =  px
                let s  = *seg;
                let sn = (s + 1) % n;
                let p  = *pi;
                let (u_a, v_a) = pt(s);
                let (u_b, v_b) = pt(sn);
                let (u_p, v_p) = pt(p);
                let dx = u_b - u_a;
                let dy = v_b - v_a;
                let px = u_p - u_a;
                let py = v_p - v_a;
                // Use += to handle degenerate case where p == s or p == sn.
                j_flat[row * nv + 2 * p]      +=  dy;
                j_flat[row * nv + 2 * p + 1]  += -dx;
                j_flat[row * nv + 2 * s]      += -dy + py;
                j_flat[row * nv + 2 * s + 1]  += -px + dx;
                j_flat[row * nv + 2 * sn]     += -py;
                j_flat[row * nv + 2 * sn + 1] +=  px;
                row += 1;
            }
            SketchConstraint::FixedLength { seg, value: _ } => {
                // r = dx²+dy² - L²
                // ∂r/∂u[s]  = -2*dx,  ∂r/∂u[sn] = +2*dx
                // ∂r/∂v[s]  = -2*dy,  ∂r/∂v[sn] = +2*dy
                let s = *seg;
                let sn = (s + 1) % n;
                let (dx, dy) = seg_dir(s);
                j_flat[row * nv + 2 * s] = -2.0 * dx;
                j_flat[row * nv + 2 * sn] = 2.0 * dx;
                j_flat[row * nv + 2 * s + 1] = -2.0 * dy;
                j_flat[row * nv + 2 * sn + 1] = 2.0 * dy;
                row += 1;
            }
            SketchConstraint::PointDistance { pt_a, pt_b, value: _ } => {
                // r = (u[b]-u[a])²+(v[b]-v[a])² - D²
                // ∂r/∂u[a] = -2*dx,  ∂r/∂u[b] = +2*dx
                // ∂r/∂v[a] = -2*dy,  ∂r/∂v[b] = +2*dy
                let a = *pt_a;
                let b = *pt_b;
                let (ax, ay) = pt(a);
                let (bx, by) = pt(b);
                let dx = bx - ax;
                let dy = by - ay;
                j_flat[row * nv + 2 * a] = -2.0 * dx;
                j_flat[row * nv + 2 * b] = 2.0 * dx;
                j_flat[row * nv + 2 * a + 1] = -2.0 * dy;
                j_flat[row * nv + 2 * b + 1] = 2.0 * dy;
                row += 1;
            }
            SketchConstraint::PointFixed { pt: pi, .. } => {
                // r0 = u[pi] - x,  r1 = v[pi] - y   (x, y are constants)
                // ∂r0/∂u[pi] = 1,  ∂r1/∂v[pi] = 1
                let p = *pi;
                j_flat[row * nv + 2 * p] = 1.0;
                j_flat[(row + 1) * nv + 2 * p + 1] = 1.0;
                row += 2;
            }
            SketchConstraint::PointOnOrigin { pt: pi } => {
                // r0 = u[p],  r1 = v[p]
                let p = *pi;
                j_flat[row * nv + 2 * p] = 1.0;
                j_flat[(row + 1) * nv + 2 * p + 1] = 1.0;
                row += 2;
            }
            SketchConstraint::PointOnXAxis { pt: pi } => {
                // r = v[p]   →   ∂r/∂v[p] = 1
                let p = *pi;
                j_flat[row * nv + 2 * p + 1] = 1.0;
                row += 1;
            }
            SketchConstraint::PointOnYAxis { pt: pi } => {
                // r = u[p]   →   ∂r/∂u[p] = 1
                let p = *pi;
                j_flat[row * nv + 2 * p] = 1.0;
                row += 1;
            }
            SketchConstraint::HorizontalPair { pt_a, pt_b, perp_u, perp_v }
            | SketchConstraint::VerticalPair { pt_a, pt_b, perp_u, perp_v } => {
                // r = perp_u*(u[b]-u[a]) + perp_v*(v[b]-v[a])
                // ∂r/∂u[a] = -perp_u,  ∂r/∂v[a] = -perp_v
                // ∂r/∂u[b] = +perp_u,  ∂r/∂v[b] = +perp_v
                let a = *pt_a;
                let b = *pt_b;
                j_flat[row * nv + 2 * a]     = -perp_u;
                j_flat[row * nv + 2 * a + 1] = -perp_v;
                j_flat[row * nv + 2 * b]     = *perp_u;
                j_flat[row * nv + 2 * b + 1] = *perp_v;
                row += 1;
            }
            SketchConstraint::PointOnCircle { pt: pi, center_u, center_v, .. } => {
                // r = (u[p] - cu)² + (v[p] - cv)² - r²
                // ∂r/∂u[p] = 2*(u[p] - cu),  ∂r/∂v[p] = 2*(v[p] - cv)
                let p = *pi;
                let (pu, pv) = pv(vars, p);
                j_flat[row * nv + 2 * p]     = 2.0 * (pu - center_u);
                j_flat[row * nv + 2 * p + 1] = 2.0 * (pv - center_v);
                row += 1;
            }
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64) -> bool { (a - b).abs() < 1e-5 }

    /// Segment direction for a solved point array.
    fn dir(pts: &[[f64; 2]], seg: usize) -> (f64, f64) {
        let n = pts.len();
        let [ax, ay] = pts[seg];
        let [bx, by] = pts[(seg + 1) % n];
        (bx - ax, by - ay)
    }

    #[test]
    fn horizontal_snaps_segment() {
        // Segment 0 is slightly tilted; constraint should make it horizontal.
        let mut pts = vec![[0.0, 0.0], [1.0, 0.3]];
        let cs = vec![SketchConstraint::Horizontal { seg: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let (_, dy) = dir(&pts, 0);
        assert!(dy.abs() < 1e-5, "dy should be ~0, got {dy}");
    }

    #[test]
    fn vertical_snaps_segment() {
        let mut pts = vec![[0.0, 0.0], [0.3, 1.0]];
        let cs = vec![SketchConstraint::Vertical { seg: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let (dx, _) = dir(&pts, 0);
        assert!(dx.abs() < 1e-5, "dx should be ~0, got {dx}");
    }

    #[test]
    fn parallel_two_segments() {
        // 3-point chain: seg 0 is [0,0]→[1,0.1], seg 1 is [1,0.1]→[2,0.5]
        // After constraining parallel, cross product should be ~0.
        let mut pts = vec![[0.0, 0.0], [1.0, 0.1], [2.0, 0.5]];
        let cs = vec![SketchConstraint::Parallel { seg_a: 0, seg_b: 1 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        let (ax, ay) = dir(&pts, 0);
        let (bx, by) = dir(&pts, 1);
        let cross = ax * by - ay * bx;
        assert!(cross.abs() < 1e-5, "cross product should be ~0, got {cross}");
    }

    #[test]
    fn perpendicular_two_segments() {
        let mut pts = vec![[0.0, 0.0], [1.0, 0.1], [0.9, 1.1]];
        let cs = vec![SketchConstraint::Perpendicular { seg_a: 0, seg_b: 1 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        let (ax, ay) = dir(&pts, 0);
        let (bx, by) = dir(&pts, 1);
        let dot = ax * bx + ay * by;
        assert!(dot.abs() < 1e-5, "dot product should be ~0, got {dot}");
    }

    #[test]
    fn equal_length_two_segments() {
        let mut pts = vec![[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]];
        let cs = vec![SketchConstraint::EqualLength { seg_a: 0, seg_b: 1 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        let (ax, ay) = dir(&pts, 0);
        let (bx, by) = dir(&pts, 1);
        let la = (ax * ax + ay * ay).sqrt();
        let lb = (bx * bx + by * by).sqrt();
        assert!(approx_eq(la, lb), "lengths should be equal: {la} vs {lb}");
    }

    #[test]
    fn angle_90_equals_perpendicular() {
        let mut pts = vec![[0.0, 0.0], [1.0, 0.1], [0.9, 1.1]];
        let cs = vec![SketchConstraint::Angle { seg_a: 0, seg_b: 1, degrees: 90.0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        let (ax, ay) = dir(&pts, 0);
        let (bx, by) = dir(&pts, 1);
        let dot = ax * bx + ay * by;
        assert!(dot.abs() < 1e-4, "90° angle ↔ dot ≈ 0, got {dot}");
    }

    #[test]
    fn fixed_length_segment() {
        // Segment 0 starts as length 2.0; constrain to 1.0.
        let mut pts = vec![[0.0, 0.0], [2.0, 0.0]];
        let cs = vec![SketchConstraint::FixedLength { seg: 0, value: 1.0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let (dx, dy) = dir(&pts, 0);
        let len = (dx * dx + dy * dy).sqrt();
        assert!(approx_eq(len, 1.0), "length should be 1.0, got {len}");
    }

    #[test]
    fn point_distance_constraint() {
        // Two independent points; constrain their distance to 2.0.
        let mut pts = vec![[0.0, 0.0], [3.0, 0.0]];
        let cs = vec![SketchConstraint::PointDistance { pt_a: 0, pt_b: 1, value: 2.0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let dx = pts[1][0] - pts[0][0];
        let dy = pts[1][1] - pts[0][1];
        let dist = (dx * dx + dy * dy).sqrt();
        assert!(approx_eq(dist, 2.0), "distance should be 2.0, got {dist}");
    }

    #[test]
    fn no_constraints_is_noop() {
        let mut pts = vec![[0.0, 0.3], [1.5, 0.7]];
        let orig = pts.clone();
        assert_eq!(solve_constraints(&mut pts, &[], 2), SolveResult::Ok);
        assert_eq!(pts, orig);
    }

    // ── Coincident ────────────────────────────────────────────────────────────

    #[test]
    fn coincident_merges_two_points() {
        let mut pts = vec![[0.0, 0.0], [1.0, 1.0]];
        let cs = vec![SketchConstraint::Coincident { pt_a: 0, pt_b: 1 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], pts[1][0]), "u coords should match");
        assert!(approx_eq(pts[0][1], pts[1][1]), "v coords should match");
    }

    // ── PointOnLine ───────────────────────────────────────────────────────────

    #[test]
    fn point_on_line_collinear() {
        // seg 0: [0,0]→[2,0] (horizontal).  pt 2 starts off the line; should land on it.
        let mut pts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 0.5]];
        let cs = vec![SketchConstraint::PointOnLine { pt: 2, seg: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        // Residual: (u_p - u_a)*(v_b - v_a) - (v_p - v_a)*(u_b - u_a) = 0
        let r = (pts[2][0] - pts[0][0]) * (pts[1][1] - pts[0][1])
            - (pts[2][1] - pts[0][1]) * (pts[1][0] - pts[0][0]);
        assert!(r.abs() < 1e-5, "residual should be ~0, got {r}");
    }

    // ── PointFixed ────────────────────────────────────────────────────────────

    #[test]
    fn point_fixed_pins_position() {
        let mut pts = vec![[0.5, 0.5], [2.0, 2.0]];
        let cs = vec![SketchConstraint::PointFixed { pt: 0, x: 1.0, y: 3.0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 1.0), "u should be 1.0, got {}", pts[0][0]);
        assert!(approx_eq(pts[0][1], 3.0), "v should be 3.0, got {}", pts[0][1]);
    }

    // ── PointOnOrigin / PointOnXAxis / PointOnYAxis ───────────────────────────

    #[test]
    fn point_on_origin() {
        let mut pts = vec![[0.4, 0.7], [1.0, 1.0]];
        let cs = vec![SketchConstraint::PointOnOrigin { pt: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 0.0), "u should be 0, got {}", pts[0][0]);
        assert!(approx_eq(pts[0][1], 0.0), "v should be 0, got {}", pts[0][1]);
    }

    #[test]
    fn point_on_x_axis() {
        let mut pts = vec![[2.0, 0.8], [3.0, 1.0]];
        let cs = vec![SketchConstraint::PointOnXAxis { pt: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][1], 0.0), "v should be 0, got {}", pts[0][1]);
    }

    #[test]
    fn point_on_y_axis() {
        let mut pts = vec![[0.8, 2.0], [1.0, 3.0]];
        let cs = vec![SketchConstraint::PointOnYAxis { pt: 0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 0.0), "u should be 0, got {}", pts[0][0]);
    }

    // ── HorizontalPair / VerticalPair ─────────────────────────────────────────

    #[test]
    fn horizontal_pair_aligns_points() {
        // perp = (0, 1): standard Y-up camera, "horizontal" means same v.
        let mut pts = vec![[0.0, 0.0], [1.0, 0.5]];
        let cs = vec![SketchConstraint::HorizontalPair {
            pt_a: 0, pt_b: 1, perp_u: 0.0, perp_v: 1.0,
        }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        // Residual: perp_u*(u[b]-u[a]) + perp_v*(v[b]-v[a]) = v[b] - v[a] = 0
        assert!(approx_eq(pts[0][1], pts[1][1]), "v coords should match");
    }

    #[test]
    fn vertical_pair_aligns_points() {
        // perp = (1, 0): "vertical" means same u.
        let mut pts = vec![[0.0, 0.0], [0.5, 1.0]];
        let cs = vec![SketchConstraint::VerticalPair {
            pt_a: 0, pt_b: 1, perp_u: 1.0, perp_v: 0.0,
        }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], pts[1][0]), "u coords should match");
    }

    // ── PointOnCircle ─────────────────────────────────────────────────────────

    #[test]
    fn point_on_circle_lands_on_circumference() {
        // Circle centered at (2, 2), radius 1. Point starts at (3.5, 2) — outside.
        let mut pts = vec![[3.5, 2.0], [0.0, 0.0]];
        let cs = vec![SketchConstraint::PointOnCircle {
            pt: 0, center_u: 2.0, center_v: 2.0, radius: 1.0,
        }];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let du = pts[0][0] - 2.0;
        let dv = pts[0][1] - 2.0;
        let r = (du * du + dv * dv).sqrt();
        assert!(approx_eq(r, 1.0), "point should be at radius 1.0, got {r}");
    }

    // ── Angle (non-90°) ───────────────────────────────────────────────────────

    #[test]
    fn angle_45_degrees() {
        use std::f64::consts::PI;
        let mut pts = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 2.0]];
        let cs = vec![SketchConstraint::Angle { seg_a: 0, seg_b: 1, degrees: 45.0 }];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        let (ax, ay) = dir(&pts, 0);
        let (bx, by) = dir(&pts, 1);
        let la = (ax * ax + ay * ay).sqrt();
        let lb = (bx * bx + by * by).sqrt();
        let cos_actual = (ax * bx + ay * by) / (la * lb);
        let cos_target = (45.0_f64 * PI / 180.0).cos();
        assert!(
            (cos_actual - cos_target).abs() < 1e-4,
            "cos(angle) should be {cos_target:.4}, got {cos_actual:.4}"
        );
    }

    // ── Combined constraints ──────────────────────────────────────────────────

    #[test]
    fn horizontal_and_fixed_length_together() {
        // Constrain seg 0 to be horizontal AND have length 3.0.
        let mut pts = vec![[0.0, 0.0], [2.0, 0.4]];
        let cs = vec![
            SketchConstraint::Horizontal { seg: 0 },
            SketchConstraint::FixedLength { seg: 0, value: 3.0 },
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        let (dx, dy) = dir(&pts, 0);
        let len = (dx * dx + dy * dy).sqrt();
        assert!(dy.abs() < 1e-5, "should be horizontal, dy={dy}");
        assert!(approx_eq(len, 3.0), "length should be 3.0, got {len}");
    }

    #[test]
    fn coincident_and_fixed_point() {
        // Pin pt 0, then force pt 1 to coincide with it.
        let mut pts = vec![[0.0, 0.0], [5.0, 5.0]];
        let cs = vec![
            SketchConstraint::PointFixed { pt: 0, x: 1.0, y: 2.0 },
            SketchConstraint::Coincident { pt_a: 0, pt_b: 1 },
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 1.0));
        assert!(approx_eq(pts[0][1], 2.0));
        assert!(approx_eq(pts[1][0], 1.0));
        assert!(approx_eq(pts[1][1], 2.0));
    }

    // ── apply_constraints: conflict detection ─────────────────────────────────

    #[test]
    fn conflicting_constraints_detected() {
        // Force pt 0 to origin AND to (5, 5) simultaneously — impossible.
        let mut pts = vec![[1.0, 1.0], [2.0, 2.0]];
        let cs = vec![
            SketchConstraint::PointOnOrigin { pt: 0 },
            SketchConstraint::PointFixed { pt: 0, x: 5.0, y: 5.0 },
        ];
        let result = apply_constraints(&mut pts, &cs, 2);
        assert!(result.conflict, "contradictory pin constraints should conflict");
    }

    #[test]
    fn conflict_leaves_points_unchanged() {
        let orig = vec![[1.0, 1.0], [2.0, 2.0]];
        let mut pts = orig.clone();
        let cs = vec![
            SketchConstraint::PointOnOrigin { pt: 0 },
            SketchConstraint::PointFixed { pt: 0, x: 5.0, y: 5.0 },
        ];
        apply_constraints(&mut pts, &cs, 2);
        assert_eq!(pts, orig, "pts must not be modified on conflict");
    }

    #[test]
    fn violated_vec_identifies_culprits() {
        // Both constraints are culprits — removing either one resolves the conflict.
        let mut pts = vec![[1.0, 1.0], [2.0, 2.0]];
        let cs = vec![
            SketchConstraint::PointOnOrigin { pt: 0 },
            SketchConstraint::PointFixed { pt: 0, x: 5.0, y: 5.0 },
        ];
        let result = apply_constraints(&mut pts, &cs, 2);
        assert_eq!(result.violated.len(), 2);
        assert!(result.violated[0], "PointOnOrigin should be flagged");
        assert!(result.violated[1], "PointFixed should be flagged");
    }

    #[test]
    fn non_conflicting_constraints_have_empty_violated() {
        let mut pts = vec![[0.0, 0.0], [1.0, 0.3]];
        let cs = vec![SketchConstraint::Horizontal { seg: 0 }];
        let result = apply_constraints(&mut pts, &cs, 2);
        assert!(!result.conflict);
        assert!(result.violated.is_empty());
    }

    // ── Propagation pre-pass tests ────────────────────────────────────────────

    #[test]
    fn propagation_only_fixed_coincident() {
        // PointFixed + Coincident: fully resolved by propagation, Newton sees
        // residuals already at zero on the first iteration.
        let mut pts = vec![[0.0, 0.0], [5.0, 5.0]];
        let cs = vec![
            SketchConstraint::PointFixed { pt: 0, x: 3.0, y: 4.0 },
            SketchConstraint::Coincident { pt_a: 0, pt_b: 1 },
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 2), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 3.0), "pt0.u should be 3, got {}", pts[0][0]);
        assert!(approx_eq(pts[0][1], 4.0), "pt0.v should be 4, got {}", pts[0][1]);
        assert!(approx_eq(pts[1][0], 3.0), "pt1.u should be 3, got {}", pts[1][0]);
        assert!(approx_eq(pts[1][1], 4.0), "pt1.v should be 4, got {}", pts[1][1]);
    }

    #[test]
    fn propagation_cascade_fixed_coincident_horizontal() {
        // PointFixed on pt0 → Coincident pt0-pt1 → Horizontal seg1 (pt1→pt2)
        // All three resolved by propagation cascade.
        // pts: 0=(0,0), 1=(5,5), 2=(9,2)
        // After: pt0=(1,2), pt1=(1,2) via Coincident, pt2.v=2 via Horizontal seg1
        let mut pts = vec![[0.0, 0.0], [5.0, 5.0], [9.0, 2.0]];
        let cs = vec![
            SketchConstraint::PointFixed { pt: 0, x: 1.0, y: 2.0 },
            SketchConstraint::Coincident { pt_a: 0, pt_b: 1 },
            SketchConstraint::Horizontal { seg: 1 }, // seg1: pt1→pt2
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 1.0) && approx_eq(pts[0][1], 2.0), "pt0 should be (1,2)");
        assert!(approx_eq(pts[1][0], 1.0) && approx_eq(pts[1][1], 2.0), "pt1 should be (1,2)");
        assert!(approx_eq(pts[2][1], 2.0), "pt2.v should be 2 (horizontal with pt1)");
    }

    #[test]
    fn propagation_mixed_with_newton() {
        // PointFixed + Coincident (propagatable) + Parallel (needs Newton).
        // Propagation handles the first two; Newton handles Parallel.
        // seg0: pt0→pt1, seg1: pt1→pt2
        // After: pt0=(0,0) (fixed), pt1=(0,0) (coincident with pt0),
        //        pt2 must be parallel to seg0 direction.
        let mut pts = vec![[0.0, 0.0], [3.0, 3.0], [5.0, 2.0]];
        let seg0_dir_before = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]);
        let _ = seg0_dir_before; // direction will change after coincident
        let cs = vec![
            SketchConstraint::PointFixed { pt: 0, x: 0.0, y: 0.0 },
            SketchConstraint::Horizontal { seg: 0 }, // seg0 must be horizontal
            SketchConstraint::Parallel { seg_a: 0, seg_b: 1 }, // seg1 parallel to seg0
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        // seg0 horizontal
        let dy0 = (pts[1][1] - pts[0][1]).abs();
        assert!(dy0 < 1e-5, "seg0 should be horizontal, dy={dy0}");
        // seg1 parallel to seg0 (also horizontal)
        let dy1 = (pts[2][1] - pts[1][1]).abs();
        assert!(dy1 < 1e-5, "seg1 should be parallel (horizontal), dy={dy1}");
    }

    #[test]
    fn propagation_origin_and_axes() {
        // PointOnOrigin, PointOnXAxis, PointOnYAxis resolved by propagation.
        let mut pts = vec![[3.0, 4.0], [5.0, 7.0], [2.0, 8.0]];
        let cs = vec![
            SketchConstraint::PointOnOrigin { pt: 0 },
            SketchConstraint::PointOnXAxis { pt: 1 },
            SketchConstraint::PointOnYAxis { pt: 2 },
        ];
        assert_eq!(solve_constraints(&mut pts, &cs, 3), SolveResult::Ok);
        assert!(approx_eq(pts[0][0], 0.0) && approx_eq(pts[0][1], 0.0), "pt0 should be origin");
        assert!(approx_eq(pts[1][1], 0.0), "pt1.v should be 0 (on X axis)");
        assert!(approx_eq(pts[2][0], 0.0), "pt2.u should be 0 (on Y axis)");
    }
}
