//! Concrete 3-D curve implementations.

use std::f64::consts::TAU;

use brep_core::{Aabb, Iso3, KernelError, Point3, Vec3};

use crate::traits::Curve3d;

// ── Line ──────────────────────────────────────────────────────────────────────

/// Infinite straight line: **C(t) = origin + t · direction**.
///
/// `direction` is not required to be unit length; arc-length is
/// `|direction| * |t1 - t0|`.
#[derive(Clone, Debug)]
pub struct LineCurve {
    pub origin: Point3,
    /// Direction vector (need not be unit).
    pub direction: Vec3,
}

impl LineCurve {
    pub fn new(origin: Point3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// Construct through two points.  Returns `None` if they coincide.
    pub fn through(a: Point3, b: Point3) -> Option<Self> {
        let d = b - a;
        if d.norm_squared() < 1e-20 {
            None
        } else {
            Some(Self::new(a, d))
        }
    }
}

impl Curve3d for LineCurve {
    fn type_name(&self) -> &'static str {
        "LineCurve"
    }

    fn point(&self, t: f64) -> Point3 {
        self.origin + self.direction * t
    }

    fn derivative1(&self, _t: f64) -> Vec3 {
        self.direction
    }

    fn derivative2(&self, _t: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn parameter_range(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn bounding_box(&self, t0: f64, t1: f64) -> Aabb {
        Aabb::from_corners(self.point(t0), self.point(t1))
    }

    fn arc_length(&self, t0: f64, t1: f64) -> f64 {
        self.direction.norm() * (t1 - t0).abs()
    }

    fn closest_parameter(&self, pt: &Point3, _tol: f64) -> Result<f64, KernelError> {
        let d2 = self.direction.norm_squared();
        if d2 < 1e-20 {
            return Err(KernelError::DegenerateGeometry(
                "LineCurve has zero-length direction".into(),
            ));
        }
        Ok((pt - self.origin).dot(&self.direction) / d2)
    }

    fn reversed(&self) -> Box<dyn Curve3d> {
        Box::new(LineCurve::new(self.origin, -self.direction))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Curve3d> {
        Box::new(LineCurve::new(
            iso.transform_point(&self.origin),
            iso.transform_vector(&self.direction),
        ))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Circle ───────────────────────────────────────────────────────────────────

/// Circle in 3-D space parameterised by angle:
/// **C(t) = center + radius · (cos(t) · x_axis + sin(t) · y_axis)**
///
/// `x_axis` and `y_axis` must be orthonormal and define the plane of the
/// circle.  The full circle corresponds to `t ∈ [0, 2π]`.
#[derive(Clone, Debug)]
pub struct CircleCurve {
    pub center: Point3,
    pub radius: f64,
    /// Unit vector in the plane pointing toward t=0.
    pub x_axis: Vec3,
    /// Unit vector in the plane at 90° to x_axis (t=π/2 direction).
    pub y_axis: Vec3,
}

impl CircleCurve {
    /// Construct with explicit axes.  Axes should be orthonormal.
    pub fn new(center: Point3, radius: f64, x_axis: Vec3, y_axis: Vec3) -> Self {
        Self { center, radius, x_axis, y_axis }
    }

    /// Normal to the plane of the circle (right-hand rule).
    pub fn normal(&self) -> Vec3 {
        self.x_axis.cross(&self.y_axis)
    }
}

impl Curve3d for CircleCurve {
    fn type_name(&self) -> &'static str {
        "CircleCurve"
    }

    fn point(&self, t: f64) -> Point3 {
        self.center
            + self.x_axis * (self.radius * t.cos())
            + self.y_axis * (self.radius * t.sin())
    }

    fn derivative1(&self, t: f64) -> Vec3 {
        self.x_axis * (-self.radius * t.sin()) + self.y_axis * (self.radius * t.cos())
    }

    fn derivative2(&self, t: f64) -> Vec3 {
        self.x_axis * (-self.radius * t.cos()) + self.y_axis * (-self.radius * t.sin())
    }

    fn parameter_range(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn is_periodic(&self) -> bool {
        true
    }

    fn period(&self) -> Option<f64> {
        Some(TAU)
    }

    fn bounding_box(&self, t0: f64, t1: f64) -> Aabb {
        // Sample at enough points to capture the bounding box tightly.
        let n = 64usize;
        let mut aabb = Aabb::empty();
        for i in 0..=n {
            let t = t0 + (t1 - t0) * (i as f64 / n as f64);
            aabb.include_point(&self.point(t));
        }
        aabb
    }

    fn arc_length(&self, t0: f64, t1: f64) -> f64 {
        self.radius * (t1 - t0).abs()
    }

    fn reversed(&self) -> Box<dyn Curve3d> {
        // Reverse by negating y_axis (flips t direction).
        Box::new(CircleCurve::new(
            self.center,
            self.radius,
            self.x_axis,
            -self.y_axis,
        ))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Curve3d> {
        Box::new(CircleCurve::new(
            iso.transform_point(&self.center),
            self.radius,
            iso.transform_vector(&self.x_axis),
            iso.transform_vector(&self.y_axis),
        ))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── TrimmedCurve ──────────────────────────────────────────────────────────────

/// A sub-range of another curve: **C(t)** restricted to `[t0, t1]`.
///
/// This is an adapter; the underlying curve owns its geometry.
pub struct TrimmedCurve {
    pub basis: Box<dyn Curve3d>,
    pub t0: f64,
    pub t1: f64,
}

impl TrimmedCurve {
    pub fn new(basis: Box<dyn Curve3d>, t0: f64, t1: f64) -> Self {
        Self { basis, t0, t1 }
    }
}

impl Curve3d for TrimmedCurve {
    fn type_name(&self) -> &'static str {
        "TrimmedCurve"
    }

    fn point(&self, t: f64) -> Point3 {
        self.basis.point(t)
    }

    fn derivative1(&self, t: f64) -> Vec3 {
        self.basis.derivative1(t)
    }

    fn derivative2(&self, t: f64) -> Vec3 {
        self.basis.derivative2(t)
    }

    fn parameter_range(&self) -> (f64, f64) {
        (self.t0, self.t1)
    }

    fn bounding_box(&self, t0: f64, t1: f64) -> Aabb {
        self.basis.bounding_box(t0, t1)
    }

    fn closest_parameter(&self, pt: &Point3, tol: f64) -> Result<f64, KernelError> {
        let t = self.basis.closest_parameter(pt, tol)?;
        Ok(t.clamp(self.t0, self.t1))
    }

    fn reversed(&self) -> Box<dyn Curve3d> {
        Box::new(TrimmedCurve::new(self.basis.reversed(), -self.t1, -self.t0))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Curve3d> {
        Box::new(TrimmedCurve::new(self.basis.transformed(iso), self.t0, self.t1))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── B-spline curve ────────────────────────────────────────────────────────────

/// Non-uniform rational B-spline (NURBS) curve.
///
/// If `weights` is `None` the curve is non-rational (polynomial B-spline).
///
/// # Parameterisation
/// `t ∈ [knots[degree], knots[n_poles]]`
#[derive(Clone, Debug)]
pub struct BsplineCurve {
    pub degree: usize,
    /// Knot vector.  Length = `n_poles + degree + 1`.
    pub knots: Vec<f64>,
    /// Control points.
    pub poles: Vec<Point3>,
    /// Optional weights for rational curves.
    pub weights: Option<Vec<f64>>,
}

impl BsplineCurve {
    /// Construct a non-rational B-spline curve.
    pub fn new(degree: usize, knots: Vec<f64>, poles: Vec<Point3>) -> Self {
        Self { degree, knots, poles, weights: None }
    }

    /// Construct a rational B-spline (NURBS) curve.
    pub fn new_rational(
        degree: usize,
        knots: Vec<f64>,
        poles: Vec<Point3>,
        weights: Vec<f64>,
    ) -> Self {
        Self { degree, knots, poles, weights: Some(weights) }
    }

    fn n_poles(&self) -> usize {
        self.poles.len()
    }

    /// Find the knot span index `i` such that `knots[i] ≤ t < knots[i+1]`.
    fn find_span(&self, t: f64) -> usize {
        let n = self.n_poles() - 1;
        let p = self.degree;
        let knots = &self.knots;
        // Clamp to valid range
        if t >= knots[n + 1] {
            return n;
        }
        if t <= knots[p] {
            return p;
        }
        // Binary search
        let mut lo = p;
        let mut hi = n + 1;
        let mut mid = (lo + hi) / 2;
        while t < knots[mid] || t >= knots[mid + 1] {
            if t < knots[mid] {
                hi = mid;
            } else {
                lo = mid;
            }
            mid = (lo + hi) / 2;
        }
        mid
    }

    /// Compute B-spline basis functions N[i..=i+p] at span `i`, parameter `t`.
    fn basis_functions(&self, i: usize, t: f64) -> Vec<f64> {
        let p = self.degree;
        let knots = &self.knots;
        let mut n = vec![0.0f64; p + 1];
        n[0] = 1.0;
        let mut left = vec![0.0f64; p + 1];
        let mut right = vec![0.0f64; p + 1];
        for j in 1..=p {
            left[j] = t - knots[i + 1 - j];
            right[j] = knots[i + j] - t;
            let mut saved = 0.0;
            for r in 0..j {
                let temp = n[r] / (right[r + 1] + left[j - r]);
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }
        n
    }

    /// Evaluate the curve in homogeneous coordinates, then project.
    fn eval_homogeneous(&self, t: f64) -> (Point3, f64) {
        let span = self.find_span(t);
        let n = self.basis_functions(span, t);
        let p = self.degree;
        let w_opt = self.weights.as_deref();
        let mut wx = 0.0f64;
        let mut wy = 0.0f64;
        let mut wz = 0.0f64;
        let mut wsum = 0.0f64;
        for j in 0..=p {
            let idx = span - p + j;
            let w = w_opt.map_or(1.0, |ws| ws[idx]);
            let pt = &self.poles[idx];
            let nw = n[j] * w;
            wx += nw * pt.x;
            wy += nw * pt.y;
            wz += nw * pt.z;
            wsum += nw;
        }
        (Point3::new(wx / wsum, wy / wsum, wz / wsum), wsum)
    }
}

impl Curve3d for BsplineCurve {
    fn type_name(&self) -> &'static str {
        "BsplineCurve"
    }

    fn point(&self, t: f64) -> Point3 {
        self.eval_homogeneous(t).0
    }

    fn derivative1(&self, t: f64) -> Vec3 {
        // Use central finite differences (analytic derivation is complex for NURBS).
        let h = 1e-6 * (self.parameter_range().1 - self.parameter_range().0).max(1.0);
        let (t0, t1) = self.parameter_range();
        let tp = (t + h).min(t1);
        let tm = (t - h).max(t0);
        (self.point(tp) - self.point(tm)) / (tp - tm)
    }

    fn parameter_range(&self) -> (f64, f64) {
        let p = self.degree;
        let n = self.n_poles();
        (self.knots[p], self.knots[n])
    }

    fn reversed(&self) -> Box<dyn Curve3d> {
        // Reverse by reparameterising: t' = (t_max + t_min) - t
        // Equivalent: reverse poles, negate and reverse knots.
        let (t0, t1) = self.parameter_range();
        let mut rev_knots: Vec<f64> = self.knots.iter().map(|&k| t0 + t1 - k).collect();
        rev_knots.reverse();
        let mut rev_poles = self.poles.clone();
        rev_poles.reverse();
        let rev_weights = self.weights.as_ref().map(|ws| {
            let mut w = ws.clone();
            w.reverse();
            w
        });
        Box::new(BsplineCurve {
            degree: self.degree,
            knots: rev_knots,
            poles: rev_poles,
            weights: rev_weights,
        })
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Curve3d> {
        Box::new(BsplineCurve {
            degree: self.degree,
            knots: self.knots.clone(),
            poles: self.poles.iter().map(|p| iso.transform_point(p)).collect(),
            weights: self.weights.clone(),
        })
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Line2d ────────────────────────────────────────────────────────────────────

use brep_core::{Point2, Vec2};
use crate::traits::Curve2d;

/// Straight line in 2-D parameter space.
#[derive(Clone, Debug)]
pub struct Line2d {
    pub origin: Point2,
    pub direction: Vec2,
}

impl Line2d {
    pub fn new(origin: Point2, direction: Vec2) -> Self {
        Self { origin, direction }
    }
}

impl Curve2d for Line2d {
    fn type_name(&self) -> &'static str {
        "Line2d"
    }

    fn point(&self, u: f64) -> Point2 {
        self.origin + self.direction * u
    }

    fn derivative1(&self, _u: f64) -> Vec2 {
        self.direction
    }

    fn parameter_range(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn reversed(&self) -> Box<dyn Curve2d> {
        Box::new(Line2d::new(self.origin, -self.direction))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

/// Arc in 2-D parameter space (e.g. for seam edges on cylindrical surfaces).
#[derive(Clone, Debug)]
pub struct Circle2d {
    pub center: Point2,
    pub radius: f64,
}

impl Circle2d {
    pub fn new(center: Point2, radius: f64) -> Self {
        Self { center, radius }
    }
}

impl Curve2d for Circle2d {
    fn type_name(&self) -> &'static str {
        "Circle2d"
    }

    fn point(&self, u: f64) -> Point2 {
        Point2::new(
            self.center.x + self.radius * u.cos(),
            self.center.y + self.radius * u.sin(),
        )
    }

    fn derivative1(&self, u: f64) -> Vec2 {
        Vec2::new(-self.radius * u.sin(), self.radius * u.cos())
    }

    fn parameter_range(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn is_periodic(&self) -> bool {
        true
    }

    fn reversed(&self) -> Box<dyn Curve2d> {
        // Negate the angular direction.
        Box::new(Circle2d::new(
            Point2::new(self.center.x, -self.center.y),
            self.radius,
        ))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::FRAC_PI_2;

    // ── LineCurve ────────────────────────────────────────────────────────────

    #[test]
    fn line_point_at_zero_is_origin() {
        let l = LineCurve::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 0.0, 0.0));
        let p = l.point(0.0);
        assert_abs_diff_eq!(p.x, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn line_derivative_is_direction() {
        let l = LineCurve::new(Point3::origin(), Vec3::new(2.0, 3.0, 4.0));
        let d = l.derivative1(5.0);
        assert_abs_diff_eq!(d.x, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn line_second_derivative_is_zero() {
        let l = LineCurve::new(Point3::origin(), Vec3::new(1.0, 1.0, 1.0));
        let d2 = l.derivative2(1.0);
        assert_abs_diff_eq!(d2.norm(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn line_arc_length() {
        let l = LineCurve::new(Point3::origin(), Vec3::new(1.0, 0.0, 0.0));
        assert_abs_diff_eq!(l.arc_length(0.0, 5.0), 5.0, epsilon = 1e-12);
    }

    #[test]
    fn line_closest_parameter() {
        let l = LineCurve::new(Point3::origin(), Vec3::new(1.0, 0.0, 0.0));
        let pt = Point3::new(3.0, 1.0, 0.0);
        let t = l.closest_parameter(&pt, 1e-9).unwrap();
        assert_abs_diff_eq!(t, 3.0, epsilon = 1e-9);
    }

    #[test]
    fn line_through_two_points() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let l = LineCurve::through(a, b).unwrap();
        assert_abs_diff_eq!((l.point(1.0) - b).norm(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn line_through_coincident_returns_none() {
        let a = Point3::origin();
        assert!(LineCurve::through(a, a).is_none());
    }

    #[test]
    fn line_reversed_inverts_direction() {
        let l = LineCurve::new(Point3::origin(), Vec3::new(1.0, 0.0, 0.0));
        let r = l.reversed();
        assert_abs_diff_eq!(r.derivative1(0.0).x, -1.0, epsilon = 1e-12);
    }

    // ── CircleCurve ──────────────────────────────────────────────────────────

    #[test]
    fn circle_point_at_zero_on_x_axis() {
        let c = CircleCurve::new(
            Point3::origin(),
            2.0,
            Vec3::x(),
            Vec3::y(),
        );
        let p = c.point(0.0);
        assert_abs_diff_eq!(p.x, 2.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn circle_point_at_pi_over_2() {
        let c = CircleCurve::new(Point3::origin(), 1.0, Vec3::x(), Vec3::y());
        let p = c.point(FRAC_PI_2);
        assert_abs_diff_eq!(p.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.y, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn circle_arc_length_full_circle() {
        let r = 3.0;
        let c = CircleCurve::new(Point3::origin(), r, Vec3::x(), Vec3::y());
        assert_abs_diff_eq!(c.arc_length(0.0, TAU), TAU * r, epsilon = 1e-10);
    }

    #[test]
    fn circle_is_periodic() {
        let c = CircleCurve::new(Point3::origin(), 1.0, Vec3::x(), Vec3::y());
        assert!(c.is_periodic());
        assert_eq!(c.period(), Some(TAU));
    }

    #[test]
    fn circle_derivative_tangent_to_circle() {
        let c = CircleCurve::new(Point3::origin(), 1.0, Vec3::x(), Vec3::y());
        // At t=0: point is (1,0,0), tangent should be (0,1,0)
        let d = c.derivative1(0.0);
        assert_abs_diff_eq!(d.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(d.y, 1.0, epsilon = 1e-12);
    }

    // ── BsplineCurve ─────────────────────────────────────────────────────────

    fn unit_line_bspline() -> BsplineCurve {
        // Degree-1 B-spline: straight line from (0,0,0) to (1,0,0)
        BsplineCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
        )
    }

    #[test]
    fn bspline_linear_start_end() {
        let b = unit_line_bspline();
        let p0 = b.point(0.0);
        let p1 = b.point(1.0);
        assert_abs_diff_eq!(p0.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p1.x, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn bspline_linear_midpoint() {
        let b = unit_line_bspline();
        let p = b.point(0.5);
        assert_abs_diff_eq!(p.x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn bspline_reversed_swaps_endpoints() {
        let b = unit_line_bspline();
        let rev = b.reversed();
        let (t0, t1) = rev.parameter_range();
        let p0 = rev.point(t0);
        let p1 = rev.point(t1);
        assert_abs_diff_eq!(p0.x, 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(p1.x, 0.0, epsilon = 1e-10);
    }

    // ── Line2d ───────────────────────────────────────────────────────────────

    #[test]
    fn line2d_point() {
        let l = Line2d::new(Point2::new(1.0, 2.0), Vec2::new(1.0, 0.0));
        let p = l.point(3.0);
        assert_abs_diff_eq!(p.x, 4.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.y, 2.0, epsilon = 1e-12);
    }
}
