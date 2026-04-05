//! Gauss-Newton constraint solver for 2D sketch geometry.
//!
//! The solver operates on a flat array of 2D point coordinates
//! `[[u, v], ...]` and adjusts them to satisfy a set of geometric
//! constraints using Gauss-Newton iteration with Levenberg-Marquardt
//! damping.

use nalgebra::{DMatrix, DVector};

use crate::constraints::SketchConstraint;

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
        // pts was never modified during the solve loop; no restore needed.
        return SolveResult::Conflict;
    }

    // Write back solved positions.
    for (i, chunk) in vars.chunks(2).enumerate() {
        pts[i] = [chunk[0], chunk[1]];
    }
    SolveResult::Ok
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
}
