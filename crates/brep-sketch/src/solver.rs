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
    let original = vars.clone();

    const MAX_ITER: usize = 50;
    const CONVERGENCE_TOL: f64 = 1e-8;
    const LAMBDA: f64 = 1e-4;
    const EPS: f64 = 1e-7;

    for _ in 0..MAX_ITER {
        let r = residuals(&vars, constraints, n_pts);
        let norm_sq: f64 = r.iter().map(|x| x * x).sum();
        if norm_sq.sqrt() < CONVERGENCE_TOL {
            break;
        }

        let m = r.len();
        let nv = vars.len();

        // Build numerical Jacobian J[m x nv].
        let mut j_flat = vec![0.0_f64; m * nv];
        for col in 0..nv {
            vars[col] += EPS;
            let r2 = residuals(&vars, constraints, n_pts);
            vars[col] -= EPS;
            for row in 0..m {
                j_flat[row * nv + col] = (r2[row] - r[row]) / EPS;
            }
        }

        // JᵀJ + λI
        let mut jt_j = vec![0.0_f64; nv * nv];
        for c in 0..nv {
            for d in 0..nv {
                let mut sum = 0.0;
                for i in 0..m {
                    sum += j_flat[i * nv + c] * j_flat[i * nv + d];
                }
                jt_j[c * nv + d] = sum + if c == d { LAMBDA } else { 0.0 };
            }
        }

        // Jᵀr
        let mut jt_r = vec![0.0_f64; nv];
        for c in 0..nv {
            let mut sum = 0.0;
            for i in 0..m {
                sum += j_flat[i * nv + c] * r[i];
            }
            jt_r[c] = sum;
        }

        let mat = DMatrix::from_row_slice(nv, nv, &jt_j);
        let rhs = DVector::from_vec(jt_r);
        let Some(inv) = mat.try_inverse() else { break };
        let delta = inv * rhs;
        for i in 0..nv {
            vars[i] -= delta[i];
        }
    }

    let final_r = residuals(&vars, constraints, n_pts);
    let final_norm: f64 = final_r.iter().map(|x| x * x).sum::<f64>().sqrt();

    if final_norm > 1e-4 {
        // Restore original positions.
        for (i, chunk) in original.chunks(2).enumerate() {
            pts[i] = [chunk[0], chunk[1]];
        }
        return SolveResult::Conflict;
    }

    // Write back solved positions.
    for (i, chunk) in vars.chunks(2).enumerate() {
        pts[i] = [chunk[0], chunk[1]];
    }
    SolveResult::Ok
}

/// Evaluate all constraint residuals.  Returns a `Vec` of scalar errors;
/// all entries should be zero when all constraints are satisfied.
fn residuals(vars: &[f64], constraints: &[SketchConstraint], n: usize) -> Vec<f64> {
    let pt = |i: usize| -> (f64, f64) { (vars[2 * i], vars[2 * i + 1]) };
    let seg_dir = |s: usize| -> (f64, f64) {
        let (ax, ay) = pt(s);
        let (bx, by) = pt((s + 1) % n);
        (bx - ax, by - ay)
    };

    let mut r = Vec::new();
    for c in constraints {
        match c {
            SketchConstraint::Parallel { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                // cross product = 0  ↔  ax*by - ay*bx = 0
                r.push(ax * by - ay * bx);
            }
            SketchConstraint::Perpendicular { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                // dot product = 0
                r.push(ax * bx + ay * by);
            }
            SketchConstraint::Angle { seg_a, seg_b, degrees } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                let la = (ax * ax + ay * ay).sqrt().max(1e-12);
                let lb = (bx * bx + by * by).sqrt().max(1e-12);
                let cos_actual = (ax * bx + ay * by) / (la * lb);
                let cos_target = degrees.to_radians().cos();
                r.push(cos_actual - cos_target);
            }
            SketchConstraint::Horizontal { seg } => {
                // vertical component of direction = 0
                let (_, dy) = seg_dir(*seg);
                r.push(dy);
            }
            SketchConstraint::Vertical { seg } => {
                // horizontal component of direction = 0
                let (dx, _) = seg_dir(*seg);
                r.push(dx);
            }
            SketchConstraint::EqualLength { seg_a, seg_b } => {
                let (ax, ay) = seg_dir(*seg_a);
                let (bx, by) = seg_dir(*seg_b);
                // |a|² - |b|² = 0
                r.push(ax * ax + ay * ay - bx * bx - by * by);
            }
            SketchConstraint::Coincident { pt_a, pt_b } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                r.push(ax - bx);
                r.push(ay - by);
            }
            SketchConstraint::FixedLength { seg, value } => {
                let (dx, dy) = seg_dir(*seg);
                // |seg|² - value² = 0
                r.push(dx * dx + dy * dy - value * value);
            }
            SketchConstraint::PointDistance { pt_a, pt_b, value } => {
                let (ax, ay) = pt(*pt_a);
                let (bx, by) = pt(*pt_b);
                let dx = bx - ax;
                let dy = by - ay;
                r.push(dx * dx + dy * dy - value * value);
            }
        }
    }
    r
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
