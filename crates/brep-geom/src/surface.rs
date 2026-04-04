//! Concrete surface implementations: Plane, Cylinder, Sphere, and BSplineSurface.

use std::f64::consts::TAU;

use brep_core::{Aabb, Iso3, KernelError, Point3, Vec3};

use crate::traits::Surface;

// ── Plane ────────────────────────────────────────────────────────────────────

/// Infinite plane parameterised as:
/// **S(u, v) = origin + u · u_axis + v · v_axis**
///
/// `u_axis` and `v_axis` must be orthogonal (not necessarily unit length,
/// though unit length is conventional).
#[derive(Clone, Debug)]
pub struct Plane {
    pub origin: Point3,
    pub u_axis: Vec3,
    pub v_axis: Vec3,
}

impl Plane {
    /// Construct from origin, u_axis, and v_axis.
    pub fn new(origin: Point3, u_axis: Vec3, v_axis: Vec3) -> Self {
        Self { origin, u_axis, v_axis }
    }

    /// Construct the XY-plane at z=0.
    pub fn xy() -> Self {
        Self::new(Point3::origin(), Vec3::x(), Vec3::y())
    }

    /// Construct the YZ-plane at x=0.
    pub fn yz() -> Self {
        Self::new(Point3::origin(), Vec3::y(), Vec3::z())
    }

    /// Construct the XZ-plane at y=0.
    pub fn xz() -> Self {
        Self::new(Point3::origin(), Vec3::x(), Vec3::z())
    }

    /// Outward unit normal.
    pub fn normal_vec(&self) -> Vec3 {
        self.u_axis.cross(&self.v_axis).normalize()
    }
}

impl Surface for Plane {
    fn type_name(&self) -> &'static str {
        "Plane"
    }

    fn point(&self, u: f64, v: f64) -> Point3 {
        self.origin + self.u_axis * u + self.v_axis * v
    }

    fn du(&self, _u: f64, _v: f64) -> Vec3 {
        self.u_axis
    }

    fn dv(&self, _u: f64, _v: f64) -> Vec3 {
        self.v_axis
    }

    fn d2u(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn d2v(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn duv(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn parameter_range(&self) -> ((f64, f64), (f64, f64)) {
        (
            (f64::NEG_INFINITY, f64::INFINITY),
            (f64::NEG_INFINITY, f64::INFINITY),
        )
    }

    fn bounding_box(&self) -> Aabb {
        Aabb::empty() // infinite plane has no finite AABB
    }

    fn closest_parameter(&self, pt: &Point3, _tol: f64) -> Result<(f64, f64), KernelError> {
        let diff = pt - self.origin;
        let u2 = self.u_axis.norm_squared();
        let v2 = self.v_axis.norm_squared();
        if u2 < 1e-20 || v2 < 1e-20 {
            return Err(KernelError::DegenerateGeometry("Plane has zero-length axis".into()));
        }
        let u = diff.dot(&self.u_axis) / u2;
        let v = diff.dot(&self.v_axis) / v2;
        Ok((u, v))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface> {
        Box::new(Plane::new(
            iso.transform_point(&self.origin),
            iso.transform_vector(&self.u_axis),
            iso.transform_vector(&self.v_axis),
        ))
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Cylindrical surface ───────────────────────────────────────────────────────

/// Cylindrical surface parameterised as:
/// **S(u, v) = axis_origin + v · axis_dir + radius · (cos(u) · x + sin(u) · y)**
///
/// - `u ∈ [0, 2π]` is the angular parameter.
/// - `v` is the axial parameter (unbounded).
/// - `x` and `y` are unit vectors orthogonal to the axis.
#[derive(Clone, Debug)]
pub struct CylindricalSurface {
    pub axis_origin: Point3,
    pub axis_dir: Vec3, // unit vector
    pub radius: f64,
    pub x_axis: Vec3, // unit vector ⊥ axis_dir
    pub y_axis: Vec3, // unit vector ⊥ axis_dir and x_axis
}

impl CylindricalSurface {
    pub fn new(axis_origin: Point3, axis_dir: Vec3, radius: f64) -> Self {
        // Build an orthonormal frame from axis_dir.
        let axis_dir = axis_dir.normalize();
        let x_axis = perpendicular_unit(&axis_dir);
        let y_axis = axis_dir.cross(&x_axis).normalize();
        Self { axis_origin, axis_dir, radius, x_axis, y_axis }
    }
}

impl Surface for CylindricalSurface {
    fn type_name(&self) -> &'static str {
        "CylindricalSurface"
    }

    fn point(&self, u: f64, v: f64) -> Point3 {
        self.axis_origin
            + self.axis_dir * v
            + self.x_axis * (self.radius * u.cos())
            + self.y_axis * (self.radius * u.sin())
    }

    fn du(&self, u: f64, _v: f64) -> Vec3 {
        self.x_axis * (-self.radius * u.sin()) + self.y_axis * (self.radius * u.cos())
    }

    fn dv(&self, _u: f64, _v: f64) -> Vec3 {
        self.axis_dir
    }

    fn d2u(&self, u: f64, _v: f64) -> Vec3 {
        self.x_axis * (-self.radius * u.cos()) + self.y_axis * (-self.radius * u.sin())
    }

    fn d2v(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn duv(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn parameter_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, TAU), (f64::NEG_INFINITY, f64::INFINITY))
    }

    fn is_u_periodic(&self) -> bool {
        true
    }

    fn normal(&self, u: f64, _v: f64) -> Result<Vec3, KernelError> {
        // Normal points radially outward.
        Ok(self.x_axis * u.cos() + self.y_axis * u.sin())
    }

    fn closest_parameter(&self, pt: &Point3, _tol: f64) -> Result<(f64, f64), KernelError> {
        let prel = pt - self.axis_origin;
        let v = prel.dot(&self.axis_dir);
        let radial = prel - self.axis_dir * v;
        let u = f64::atan2(radial.dot(&self.y_axis), radial.dot(&self.x_axis));
        let u = if u < 0.0 { u + TAU } else { u };
        Ok((u, v))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface> {
        Box::new(CylindricalSurface {
            axis_origin: iso.transform_point(&self.axis_origin),
            axis_dir: iso.transform_vector(&self.axis_dir),
            radius: self.radius,
            x_axis: iso.transform_vector(&self.x_axis),
            y_axis: iso.transform_vector(&self.y_axis),
        })
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Spherical surface ─────────────────────────────────────────────────────────

/// Sphere parameterised as:
/// **S(u, v) = center + radius · (cos(v)·cos(u)·x + cos(v)·sin(u)·y + sin(v)·z)**
///
/// - `u ∈ [0, 2π]` — longitude
/// - `v ∈ [-π/2, π/2]` — latitude
#[derive(Clone, Debug)]
pub struct SphericalSurface {
    pub center: Point3,
    pub radius: f64,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
    pub z_axis: Vec3,
}

impl SphericalSurface {
    pub fn new(center: Point3, radius: f64) -> Self {
        Self {
            center,
            radius,
            x_axis: Vec3::x(),
            y_axis: Vec3::y(),
            z_axis: Vec3::z(),
        }
    }
}

impl Surface for SphericalSurface {
    fn type_name(&self) -> &'static str {
        "SphericalSurface"
    }

    fn point(&self, u: f64, v: f64) -> Point3 {
        let cv = v.cos();
        let sv = v.sin();
        let cu = u.cos();
        let su = u.sin();
        self.center
            + self.x_axis * (self.radius * cv * cu)
            + self.y_axis * (self.radius * cv * su)
            + self.z_axis * (self.radius * sv)
    }

    fn du(&self, u: f64, v: f64) -> Vec3 {
        let cv = v.cos();
        self.x_axis * (-self.radius * cv * u.sin())
            + self.y_axis * (self.radius * cv * u.cos())
    }

    fn dv(&self, u: f64, v: f64) -> Vec3 {
        self.x_axis * (-self.radius * v.sin() * u.cos())
            + self.y_axis * (-self.radius * v.sin() * u.sin())
            + self.z_axis * (self.radius * v.cos())
    }

    fn parameter_range(&self) -> ((f64, f64), (f64, f64)) {
        use std::f64::consts::FRAC_PI_2;
        ((0.0, TAU), (-FRAC_PI_2, FRAC_PI_2))
    }

    fn is_u_periodic(&self) -> bool {
        true
    }

    fn normal(&self, u: f64, v: f64) -> Result<Vec3, KernelError> {
        let p = self.point(u, v);
        let n = (p - self.center).normalize();
        Ok(n)
    }

    fn closest_parameter(&self, pt: &Point3, _tol: f64) -> Result<(f64, f64), KernelError> {
        let prel = pt - self.center;
        let x = prel.dot(&self.x_axis);
        let y = prel.dot(&self.y_axis);
        let z = prel.dot(&self.z_axis);
        let u = f64::atan2(y, x);
        let u = if u < 0.0 { u + TAU } else { u };
        let r_xy = (x * x + y * y).sqrt();
        let v = f64::atan2(z, r_xy);
        Ok((u, v))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface> {
        Box::new(SphericalSurface {
            center: iso.transform_point(&self.center),
            radius: self.radius,
            x_axis: iso.transform_vector(&self.x_axis),
            y_axis: iso.transform_vector(&self.y_axis),
            z_axis: iso.transform_vector(&self.z_axis),
        })
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Conical surface ───────────────────────────────────────────────────────────

/// Right circular cone parameterised as:
/// **S(u, v) = apex + v·(cos(half_angle)·axis + sin(half_angle)·(cos(u)·x + sin(u)·y))**
///
/// - `u ∈ [0, 2π]` — longitude around the axis
/// - `v ∈ [0, ∞)`  — distance along the slant from the apex
///
/// At `v = 0` the surface degenerates to the apex.  The base circle of radius
/// `base_radius` lies at `v = base_radius / sin(half_angle)`.
#[derive(Clone, Debug)]
pub struct ConicalSurface {
    pub apex: Point3,
    pub axis: Vec3,       // unit vector (points away from base)
    pub half_angle: f64,  // radians, 0 < half_angle < π/2
    pub x_axis: Vec3,
    pub y_axis: Vec3,
}

impl ConicalSurface {
    /// `half_angle` is in radians.  `base_radius` is used only to derive the
    /// x/y frame; it does not change the parameterisation.
    pub fn new(apex: Point3, axis: Vec3, half_angle: f64, _base_radius: f64) -> Self {
        let axis = axis.normalize();
        let x_axis = perpendicular_unit(&axis);
        let y_axis = axis.cross(&x_axis).normalize();
        Self { apex, axis, half_angle, x_axis, y_axis }
    }
}

impl Surface for ConicalSurface {
    fn type_name(&self) -> &'static str {
        "ConicalSurface"
    }

    fn point(&self, u: f64, v: f64) -> Point3 {
        let r = v * self.half_angle.sin();
        self.apex
            + self.axis * (v * self.half_angle.cos())
            + self.x_axis * (r * u.cos())
            + self.y_axis * (r * u.sin())
    }

    fn du(&self, u: f64, v: f64) -> Vec3 {
        let r = v * self.half_angle.sin();
        self.x_axis * (-r * u.sin()) + self.y_axis * (r * u.cos())
    }

    fn dv(&self, u: f64, _v: f64) -> Vec3 {
        self.axis * self.half_angle.cos()
            + self.x_axis * (self.half_angle.sin() * u.cos())
            + self.y_axis * (self.half_angle.sin() * u.sin())
    }

    fn d2u(&self, u: f64, v: f64) -> Vec3 {
        let r = v * self.half_angle.sin();
        self.x_axis * (-r * u.cos()) + self.y_axis * (-r * u.sin())
    }

    fn d2v(&self, _u: f64, _v: f64) -> Vec3 {
        Vec3::zeros()
    }

    fn duv(&self, u: f64, _v: f64) -> Vec3 {
        self.x_axis * (-self.half_angle.sin() * u.sin())
            + self.y_axis * (self.half_angle.sin() * u.cos())
    }

    fn parameter_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, TAU), (0.0, f64::INFINITY))
    }

    fn is_u_periodic(&self) -> bool {
        true
    }

    fn normal(&self, u: f64, _v: f64) -> Result<Vec3, KernelError> {
        // Outward normal on a cone: points away from the axis at angle (π/2 - half_angle).
        let n = self.x_axis * (u.cos() * self.half_angle.cos())
              + self.y_axis * (u.sin() * self.half_angle.cos())
              - self.axis   * self.half_angle.sin();
        Ok(n.normalize())
    }

    fn closest_parameter(&self, pt: &Point3, _tol: f64) -> Result<(f64, f64), KernelError> {
        let prel = pt - self.apex;
        // u = longitude around axis
        let u = f64::atan2(prel.dot(&self.y_axis), prel.dot(&self.x_axis));
        let u = if u < 0.0 { u + TAU } else { u };
        // v = slant distance from apex
        let slant_dir = self.axis * self.half_angle.cos()
            + self.x_axis * (self.half_angle.sin() * u.cos())
            + self.y_axis * (self.half_angle.sin() * u.sin());
        let v = prel.dot(&slant_dir).max(0.0);
        Ok((u, v))
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface> {
        Box::new(ConicalSurface {
            apex:       iso.transform_point(&self.apex),
            axis:       iso.transform_vector(&self.axis),
            half_angle: self.half_angle,
            x_axis:     iso.transform_vector(&self.x_axis),
            y_axis:     iso.transform_vector(&self.y_axis),
        })
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── B-spline surface ──────────────────────────────────────────────────────────

/// Bi-parametric NURBS surface.
#[derive(Clone, Debug)]
pub struct BsplineSurface {
    pub u_degree: usize,
    pub v_degree: usize,
    pub u_knots: Vec<f64>,
    pub v_knots: Vec<f64>,
    /// `poles[i][j]` for u-index `i` and v-index `j`.
    pub poles: Vec<Vec<Point3>>,
    pub weights: Option<Vec<Vec<f64>>>,
}

impl BsplineSurface {
    pub fn new(
        u_degree: usize,
        v_degree: usize,
        u_knots: Vec<f64>,
        v_knots: Vec<f64>,
        poles: Vec<Vec<Point3>>,
    ) -> Self {
        Self { u_degree, v_degree, u_knots, v_knots, poles, weights: None }
    }

    fn n_u(&self) -> usize { self.poles.len() }
    fn n_v(&self) -> usize { self.poles[0].len() }

    fn find_span(knots: &[f64], degree: usize, n: usize, t: f64) -> usize {
        if t >= knots[n + 1] { return n; }
        if t <= knots[degree] { return degree; }
        let mut lo = degree;
        let mut hi = n + 1;
        let mut mid = (lo + hi) / 2;
        while t < knots[mid] || t >= knots[mid + 1] {
            if t < knots[mid] { hi = mid; } else { lo = mid; }
            mid = (lo + hi) / 2;
        }
        mid
    }

    fn basis(knots: &[f64], degree: usize, span: usize, t: f64) -> Vec<f64> {
        let mut n = vec![0.0f64; degree + 1];
        n[0] = 1.0;
        let mut left = vec![0.0f64; degree + 1];
        let mut right = vec![0.0f64; degree + 1];
        for j in 1..=degree {
            left[j] = t - knots[span + 1 - j];
            right[j] = knots[span + j] - t;
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

    fn eval(&self, u: f64, v: f64) -> Point3 {
        let nu = Self::n_u(&self) - 1;
        let nv = Self::n_v(&self) - 1;
        let u_span = Self::find_span(&self.u_knots, self.u_degree, nu, u);
        let v_span = Self::find_span(&self.v_knots, self.v_degree, nv, v);
        let nu_b = Self::basis(&self.u_knots, self.u_degree, u_span, u);
        let nv_b = Self::basis(&self.v_knots, self.v_degree, v_span, v);
        let pu = self.u_degree;
        let pv = self.v_degree;
        let mut wx = 0.0f64;
        let mut wy = 0.0f64;
        let mut wz = 0.0f64;
        let mut wsum = 0.0f64;
        for i in 0..=pu {
            for j in 0..=pv {
                let ui = u_span - pu + i;
                let vj = v_span - pv + j;
                let w = self.weights.as_ref().map_or(1.0, |ws| ws[ui][vj]);
                let pt = &self.poles[ui][vj];
                let nw = nu_b[i] * nv_b[j] * w;
                wx += nw * pt.x;
                wy += nw * pt.y;
                wz += nw * pt.z;
                wsum += nw;
            }
        }
        Point3::new(wx / wsum, wy / wsum, wz / wsum)
    }
}

impl Surface for BsplineSurface {
    fn type_name(&self) -> &'static str {
        "BsplineSurface"
    }

    fn point(&self, u: f64, v: f64) -> Point3 {
        self.eval(u, v)
    }

    fn du(&self, u: f64, v: f64) -> Vec3 {
        let h = 1e-6;
        let (u0, u1) = (self.parameter_range().0.0, self.parameter_range().0.1);
        let up = (u + h).min(u1);
        let um = (u - h).max(u0);
        (self.eval(up, v) - self.eval(um, v)) / (up - um)
    }

    fn dv(&self, u: f64, v: f64) -> Vec3 {
        let h = 1e-6;
        let (v0, v1) = (self.parameter_range().1.0, self.parameter_range().1.1);
        let vp = (v + h).min(v1);
        let vm = (v - h).max(v0);
        (self.eval(u, vp) - self.eval(u, vm)) / (vp - vm)
    }

    fn parameter_range(&self) -> ((f64, f64), (f64, f64)) {
        let pu = self.u_degree;
        let pv = self.v_degree;
        let nu = self.poles.len();
        let nv = self.poles[0].len();
        (
            (self.u_knots[pu], self.u_knots[nu]),
            (self.v_knots[pv], self.v_knots[nv]),
        )
    }

    fn transformed(&self, iso: &Iso3) -> Box<dyn Surface> {
        Box::new(BsplineSurface {
            u_degree: self.u_degree,
            v_degree: self.v_degree,
            u_knots: self.u_knots.clone(),
            v_knots: self.v_knots.clone(),
            poles: self.poles.iter()
                .map(|row| row.iter().map(|p| iso.transform_point(p)).collect())
                .collect(),
            weights: self.weights.clone(),
        })
    }

    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Return a unit vector perpendicular to `v`.
fn perpendicular_unit(v: &Vec3) -> Vec3 {
    // Choose a basis vector not parallel to v.
    let candidate = if v.x.abs() <= v.y.abs() && v.x.abs() <= v.z.abs() {
        Vec3::x()
    } else if v.y.abs() <= v.z.abs() {
        Vec3::y()
    } else {
        Vec3::z()
    };
    v.cross(&candidate).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    // ── Plane ────────────────────────────────────────────────────────────────

    #[test]
    fn plane_xy_normal_is_z() {
        let p = Plane::xy();
        let n = p.normal_vec();
        assert_abs_diff_eq!(n.z, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn plane_point_evaluation() {
        let p = Plane::xy();
        let pt = p.point(3.0, 4.0);
        assert_abs_diff_eq!(pt.x, 3.0, epsilon = 1e-12);
        assert_abs_diff_eq!(pt.y, 4.0, epsilon = 1e-12);
        assert_abs_diff_eq!(pt.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn plane_derivatives_are_axes() {
        let p = Plane::xy();
        let du = p.du(0.0, 0.0);
        let dv = p.dv(0.0, 0.0);
        assert_abs_diff_eq!(du, Vec3::x(), epsilon = 1e-12);
        assert_abs_diff_eq!(dv, Vec3::y(), epsilon = 1e-12);
    }

    #[test]
    fn plane_second_derivatives_zero() {
        let p = Plane::xy();
        assert_abs_diff_eq!(p.d2u(1.0, 1.0).norm(), 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.d2v(1.0, 1.0).norm(), 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.duv(1.0, 1.0).norm(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn plane_closest_parameter() {
        let p = Plane::xy();
        let pt = Point3::new(5.0, 7.0, 99.0); // project ignores z
        let (u, v) = p.closest_parameter(&pt, 1e-9).unwrap();
        assert_abs_diff_eq!(u, 5.0, epsilon = 1e-9);
        assert_abs_diff_eq!(v, 7.0, epsilon = 1e-9);
    }

    // ── CylindricalSurface ───────────────────────────────────────────────────

    #[test]
    fn cylinder_point_on_equator() {
        let cyl = CylindricalSurface::new(Point3::origin(), Vec3::z(), 2.0);
        let p = cyl.point(0.0, 0.0);
        // At u=0, v=0 the point should be at (radius, 0, 0)
        assert_abs_diff_eq!(p.coords.norm(), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn cylinder_normal_is_radial() {
        let cyl = CylindricalSurface::new(Point3::origin(), Vec3::z(), 1.0);
        let n = cyl.normal(0.0, 0.0).unwrap();
        // At u=0 the normal should point in the x_axis direction
        assert_abs_diff_eq!(n.norm(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn cylinder_is_u_periodic() {
        let cyl = CylindricalSurface::new(Point3::origin(), Vec3::z(), 1.0);
        assert!(cyl.is_u_periodic());
    }

    #[test]
    fn cylinder_point_at_period_matches_start() {
        let cyl = CylindricalSurface::new(Point3::origin(), Vec3::z(), 3.0);
        let p0 = cyl.point(0.0, 1.0);
        let p1 = cyl.point(TAU, 1.0);
        assert_abs_diff_eq!((p0 - p1).norm(), 0.0, epsilon = 1e-10);
    }

    // ── SphericalSurface ─────────────────────────────────────────────────────

    #[test]
    fn sphere_point_is_on_surface() {
        let s = SphericalSurface::new(Point3::origin(), 5.0);
        for (u, v) in [(0.0, 0.0), (1.0, 0.5), (3.0, -1.0)] {
            let p = s.point(u, v);
            let d = (p - s.center).norm();
            assert_abs_diff_eq!(d, s.radius, epsilon = 1e-10);
        }
    }

    #[test]
    fn sphere_normal_is_unit_radial() {
        let s = SphericalSurface::new(Point3::origin(), 2.0);
        let n = s.normal(0.5, 0.5).unwrap();
        assert_abs_diff_eq!(n.norm(), 1.0, epsilon = 1e-12);
    }

    // ── BsplineSurface ───────────────────────────────────────────────────────

    #[test]
    fn bspline_bilinear_patch_corners() {
        // Bilinear patch (degree 1×1) from (0,0,0) to (1,1,0)
        let poles = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let knots_u = vec![0.0, 0.0, 1.0, 1.0];
        let knots_v = vec![0.0, 0.0, 1.0, 1.0];
        let surf = BsplineSurface::new(1, 1, knots_u, knots_v, poles);
        let p00 = surf.point(0.0, 0.0);
        let p11 = surf.point(1.0, 1.0);
        assert_abs_diff_eq!(p00.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p11.x, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p11.y, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn bspline_bilinear_patch_midpoint() {
        let poles = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let surf = BsplineSurface::new(
            1, 1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            poles,
        );
        let p = surf.point(0.5, 0.5);
        assert_abs_diff_eq!(p.x, 0.5, epsilon = 1e-10);
        assert_abs_diff_eq!(p.y, 0.5, epsilon = 1e-10);
    }
}
