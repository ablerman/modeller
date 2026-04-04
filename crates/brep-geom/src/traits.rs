//! Core geometry traits: [`Curve3d`], [`Curve2d`], and [`Surface`].
//!
//! Geometry is purely mathematical — it knows nothing about topology.
//! All evaluation, differentiation, and bounding-box queries go through
//! these traits.  Concrete implementations live in sibling modules.

use brep_core::{Aabb, KernelError, Iso3, Point2, Point3, Vec2, Vec3};

// ── 3-D parametric curve ──────────────────────────────────────────────────────

/// A parametric curve **C(t)** in 3-D space.
///
/// # Contract
/// - `point(t)` and `derivative1(t)` must be consistent (the derivative
///   is the limit of (C(t+ε) − C(t)) / ε as ε → 0).
/// - `parameter_range()` returns `(t_min, t_max)` with `t_min ≤ t_max`.
/// - For periodic curves `t_max − t_min` equals the period.
pub trait Curve3d: Send + Sync {
    /// Name used in error messages.
    fn type_name(&self) -> &'static str;

    /// Evaluate position at parameter `t`.
    fn point(&self, t: f64) -> Point3;

    /// First derivative (tangent, not necessarily unit length).
    fn derivative1(&self, t: f64) -> Vec3;

    /// Second derivative.  Default implementation uses central finite differences.
    fn derivative2(&self, t: f64) -> Vec3 {
        let h = 1e-6;
        (self.derivative1(t + h) - self.derivative1(t - h)) / (2.0 * h)
    }

    /// Parameter range `[t_min, t_max]`.
    fn parameter_range(&self) -> (f64, f64);

    /// Returns `true` if the curve is periodic.
    fn is_periodic(&self) -> bool {
        false
    }

    /// Period of a periodic curve.  `None` for non-periodic curves.
    fn period(&self) -> Option<f64> {
        None
    }

    /// Axis-aligned bounding box over `[t0, t1]`.
    ///
    /// The default implementation samples the curve at `n` evenly-spaced
    /// points and takes the point-wise min/max.  Concrete types should
    /// override this with an exact or tighter implementation.
    fn bounding_box(&self, t0: f64, t1: f64) -> Aabb {
        let n = 32usize;
        let mut aabb = Aabb::empty();
        for i in 0..=n {
            let t = t0 + (t1 - t0) * (i as f64 / n as f64);
            aabb.include_point(&self.point(t));
        }
        aabb
    }

    /// Find the parameter value on the curve closest to `pt`.
    ///
    /// The default implementation uses a two-phase approach:
    /// 1. Sample at `n_samples` evenly-spaced parameter values.
    /// 2. Refine the best sample with up to 50 Newton iterations.
    fn closest_parameter(&self, pt: &Point3, tol: f64) -> Result<f64, KernelError> {
        let (t0, t1) = self.parameter_range();
        let n = 64usize;
        let mut best_t = t0;
        let mut best_dist2 = f64::MAX;
        for i in 0..=n {
            let t = t0 + (t1 - t0) * (i as f64 / n as f64);
            let d2 = (self.point(t) - pt).norm_squared();
            if d2 < best_dist2 {
                best_dist2 = d2;
                best_t = t;
            }
        }
        // Newton refinement: minimise ‖C(t)−P‖²  ⟹  (C−P)·C' = 0
        let mut t = best_t;
        for _ in 0..50 {
            let c = self.point(t);
            let cd = self.derivative1(t);
            let cd2 = self.derivative2(t);
            let f = (c - pt).dot(&cd);
            let df = cd.norm_squared() + (c - pt).dot(&cd2);
            if df.abs() < 1e-15 {
                break;
            }
            let dt = f / df;
            t -= dt;
            t = t.clamp(t0, t1);
            if dt.abs() < tol {
                break;
            }
        }
        Ok(t)
    }

    /// Approximate arc length from `t0` to `t1` using 16-point
    /// Gauss-Legendre quadrature.
    fn arc_length(&self, t0: f64, t1: f64) -> f64 {
        gauss_legendre_8(|t| self.derivative1(t).norm(), t0, t1)
    }

    /// Return a new curve with reversed parameterisation.
    fn reversed(&self) -> Box<dyn Curve3d>;

    /// Return a new curve transformed by `iso`.
    fn transformed(&self, iso: &Iso3) -> Box<dyn Curve3d>;
}

// ── 2-D parametric curve (pcurve on a surface) ────────────────────────────────

/// A parametric curve **C(u)** in the 2-D parameter space of a surface.
///
/// Pcurves are stored on half-edges and link topology to surface geometry.
pub trait Curve2d: Send + Sync {
    /// Name used in error messages.
    fn type_name(&self) -> &'static str;

    /// Evaluate 2-D position at parameter `u`.
    fn point(&self, u: f64) -> Point2;

    /// First derivative.
    fn derivative1(&self, u: f64) -> Vec2;

    /// Parameter range `[u_min, u_max]`.
    fn parameter_range(&self) -> (f64, f64);

    /// Returns `true` if the curve is periodic.
    fn is_periodic(&self) -> bool {
        false
    }

    /// Return a new curve with reversed parameterisation.
    fn reversed(&self) -> Box<dyn Curve2d>;
}

// ── Parametric surface ────────────────────────────────────────────────────────

/// A parametric surface **S(u, v)** in 3-D space.
pub trait Surface: Send + Sync {
    /// Name used in error messages.
    fn type_name(&self) -> &'static str;

    /// Evaluate position at `(u, v)`.
    fn point(&self, u: f64, v: f64) -> Point3;

    /// Outward-facing surface normal at `(u, v)`.
    ///
    /// Returns `Err(KernelError::DegenerateGeometry(...))` at singular points
    /// (e.g. the poles of a sphere).
    fn normal(&self, u: f64, v: f64) -> Result<Vec3, KernelError> {
        let du = self.du(u, v);
        let dv = self.dv(u, v);
        let n = du.cross(&dv);
        let len = n.norm();
        if len < 1e-14 {
            Err(KernelError::DegenerateGeometry(format!(
                "{} has zero normal at ({u}, {v})",
                self.type_name()
            )))
        } else {
            Ok(n / len)
        }
    }

    /// Partial derivative with respect to `u`.
    fn du(&self, u: f64, v: f64) -> Vec3;

    /// Partial derivative with respect to `v`.
    fn dv(&self, u: f64, v: f64) -> Vec3;

    /// Second partial derivative ∂²S/∂u².  Default: finite difference.
    fn d2u(&self, u: f64, v: f64) -> Vec3 {
        let h = 1e-6;
        (self.du(u + h, v) - self.du(u - h, v)) / (2.0 * h)
    }

    /// Second partial derivative ∂²S/∂v².  Default: finite difference.
    fn d2v(&self, u: f64, v: f64) -> Vec3 {
        let h = 1e-6;
        (self.dv(u, v + h) - self.dv(u, v - h)) / (2.0 * h)
    }

    /// Mixed partial ∂²S/∂u∂v.  Default: finite difference.
    fn duv(&self, u: f64, v: f64) -> Vec3 {
        let h = 1e-6;
        (self.du(u, v + h) - self.du(u, v - h)) / (2.0 * h)
    }

    /// Parameter range `((u_min, u_max), (v_min, v_max))`.
    fn parameter_range(&self) -> ((f64, f64), (f64, f64));

    /// Returns `true` if the surface is periodic in `u`.
    fn is_u_periodic(&self) -> bool {
        false
    }

    /// Returns `true` if the surface is periodic in `v`.
    fn is_v_periodic(&self) -> bool {
        false
    }

    /// Axis-aligned bounding box.  Default: sample at a 16×16 grid.
    fn bounding_box(&self) -> Aabb {
        let ((u0, u1), (v0, v1)) = self.parameter_range();
        let n = 16usize;
        let mut aabb = Aabb::empty();
        for i in 0..=n {
            for j in 0..=n {
                let u = u0 + (u1 - u0) * (i as f64 / n as f64);
                let v = v0 + (v1 - v0) * (j as f64 / n as f64);
                aabb.include_point(&self.point(u, v));
            }
        }
        aabb
    }

    /// Project `pt` onto the surface: find `(u, v)` minimising `‖S(u,v)−pt‖`.
    ///
    /// Default: coarse grid search followed by Newton refinement.
    fn closest_parameter(&self, pt: &Point3, tol: f64) -> Result<(f64, f64), KernelError> {
        let ((u0, u1), (v0, v1)) = self.parameter_range();
        let n = 16usize;
        let mut best_u = u0;
        let mut best_v = v0;
        let mut best_d2 = f64::MAX;
        for i in 0..=n {
            for j in 0..=n {
                let u = u0 + (u1 - u0) * (i as f64 / n as f64);
                let v = v0 + (v1 - v0) * (j as f64 / n as f64);
                let d2 = (self.point(u, v) - pt).norm_squared();
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_u = u;
                    best_v = v;
                }
            }
        }
        // Newton refinement
        let mut u = best_u;
        let mut v = best_v;
        for _ in 0..50 {
            let p = self.point(u, v);
            let su = self.du(u, v);
            let sv = self.dv(u, v);
            let diff = p - pt;
            let fu = diff.dot(&su);
            let fv = diff.dot(&sv);
            if fu.abs() < tol && fv.abs() < tol {
                break;
            }
            let suu = self.d2u(u, v);
            let svv = self.d2v(u, v);
            let suv = self.duv(u, v);
            // 2×2 Newton system: J * [du, dv]ᵀ = -[fu, fv]ᵀ
            let j00 = su.norm_squared() + diff.dot(&suu);
            let j01 = su.dot(&sv) + diff.dot(&suv);
            let j11 = sv.norm_squared() + diff.dot(&svv);
            let det = j00 * j11 - j01 * j01;
            if det.abs() < 1e-15 {
                break;
            }
            let du_ = (j11 * fu - j01 * fv) / det;
            let dv_ = (j00 * fv - j01 * fu) / det;
            u -= du_;
            v -= dv_;
            u = u.clamp(u0, u1);
            v = v.clamp(v0, v1);
            if du_.abs() < tol && dv_.abs() < tol {
                break;
            }
        }
        Ok((u, v))
    }

    /// Return a new surface transformed by `iso`.
    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface>;
}

// ── Gauss-Legendre quadrature (8-point) ──────────────────────────────────────

/// 8-point Gauss-Legendre quadrature on `[a, b]`.
///
/// Exact for polynomials of degree ≤ 15.  Sufficient for smooth
/// integrands like ‖C'(t)‖ (arc-length computation).
/// Weights sum to exactly 2 (the measure of the standard interval [-1,1]).
pub(crate) fn gauss_legendre_8<F: Fn(f64) -> f64>(f: F, a: f64, b: f64) -> f64 {
    const ABSCISSAE: [f64; 8] = [
        -0.9602898564975363,
        -0.7966664774136267,
        -0.5255324099163290,
        -0.1834346424956498,
         0.1834346424956498,
         0.5255324099163290,
         0.7966664774136267,
         0.9602898564975363,
    ];
    const WEIGHTS: [f64; 8] = [
        0.1012285362903763,
        0.2223810344533745,
        0.3137066458778873,
        0.3626837833783620,
        0.3626837833783620,
        0.3137066458778873,
        0.2223810344533745,
        0.1012285362903763,
    ];
    let mid = (a + b) * 0.5;
    let half = (b - a) * 0.5;
    ABSCISSAE
        .iter()
        .zip(WEIGHTS.iter())
        .map(|(&xi, &wi)| wi * f(mid + half * xi))
        .sum::<f64>()
        * half
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gauss_legendre_integrates_constant() {
        // ∫₀² 3 dt = 6
        let result = gauss_legendre_8(|_| 3.0, 0.0, 2.0);
        approx::assert_abs_diff_eq!(result, 6.0, epsilon = 1e-12);
    }

    #[test]
    fn gauss_legendre_integrates_quadratic() {
        // ∫₀¹ x² dx = 1/3
        let result = gauss_legendre_8(|x| x * x, 0.0, 1.0);
        approx::assert_abs_diff_eq!(result, 1.0 / 3.0, epsilon = 1e-12);
    }

    #[test]
    fn gauss_legendre_integrates_sin() {
        // ∫₀^π sin(x) dx = 2
        let result = gauss_legendre_8(f64::sin, 0.0, std::f64::consts::PI);
        approx::assert_abs_diff_eq!(result, 2.0, epsilon = 1e-10);
    }
}
