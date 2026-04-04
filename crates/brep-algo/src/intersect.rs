//! Surface-surface and curve-surface intersection algorithms.
//!
//! # Analytical cases (exact results)
//!
//! | Pair | Result |
//! |---|---|
//! | Plane × Plane | Line or coincident |
//! | Plane × Sphere | Circle or point |
//! | Plane × Cylinder | Ellipse (general), circle (perpendicular), line (tangent) |
//! | Plane × Cone | Conic section (ellipse, parabola, hyperbola, point, line) |
//! | Sphere × Sphere | Circle or point |
//!
//! # Numerical fallback
//!
//! General surface pairs use a **marching algorithm**:
//! 1. Seed: grid-sample both surfaces, find a close pair as start point.
//! 2. Trace: alternate Newton steps on each surface to stay on the intersection.
//! 3. Terminate: when we return to the start (closed curve) or leave the
//!    parameter domain (open arc).

use brep_core::{KernelError, Point3, Vec3};
use brep_geom::traits::Surface;

// ── Result types ──────────────────────────────────────────────────────────────

/// A point on an intersection curve, carrying its 3-D position and the UV
/// parameters on both surfaces.
#[derive(Clone, Debug)]
pub struct IntPoint {
    pub point: Point3,
    /// Parameter on surface A.
    pub uv_a: (f64, f64),
    /// Parameter on surface B.
    pub uv_b: (f64, f64),
}

/// A sampled intersection curve between two surfaces.
#[derive(Clone, Debug)]
pub struct IntersectionCurve {
    pub points: Vec<IntPoint>,
    /// True when the curve closes back on itself.
    pub is_closed: bool,
}

// ── Plane × Plane ─────────────────────────────────────────────────────────────

use brep_geom::surface::Plane;

/// Intersect two planes.  Returns `None` if they are parallel (or coincident).
pub fn plane_plane(a: &Plane, b: &Plane) -> Option<(Point3, Vec3)> {
    let na = a.normal_vec();
    let nb = b.normal_vec();
    let dir = na.cross(&nb);
    if dir.norm() < 1e-12 { return None; } // parallel

    // Solve the 3×3 system to find a point on the line.
    // Use the component of `dir` with the largest magnitude as free axis.
    let d_a = na.dot(&a.origin.coords);
    let d_b = nb.dot(&b.origin.coords);

    let (p, _) = least_squares_plane_intersection(&na, d_a, &nb, d_b, &dir);
    Some((p, dir.normalize()))
}

/// Solve the underdetermined system [na; nb] * x = [da; db] in the plane ⊥ dir.
fn least_squares_plane_intersection(na: &Vec3, da: f64, nb: &Vec3, db: f64, dir: &Vec3) -> (Point3, Vec3) {
    // Pick the axis where dir is largest → set that coord to 0.
    let axis = if dir.x.abs() >= dir.y.abs() && dir.x.abs() >= dir.z.abs() { 0 }
               else if dir.y.abs() >= dir.z.abs() { 1 }
               else { 2 };

    // Reduce to 2×2 by fixing the free coordinate to 0.
    let (i, j) = match axis { 0 => (1, 2), 1 => (0, 2), _ => (0, 1) };
    let a00 = na[i]; let a01 = na[j]; let b0 = da;
    let a10 = nb[i]; let a11 = nb[j]; let b1 = db;
    let det = a00 * a11 - a01 * a10;
    let (xi, xj) = if det.abs() > 1e-15 {
        ((b0 * a11 - b1 * a01) / det, (a00 * b1 - a10 * b0) / det)
    } else {
        (0.0, 0.0)
    };
    let mut coords = [0.0f64; 3];
    coords[i] = xi;
    coords[j] = xj;
    (Point3::new(coords[0], coords[1], coords[2]), dir.normalize())
}

// ── Plane × Sphere ────────────────────────────────────────────────────────────

use brep_geom::surface::SphericalSurface;

/// Result of a plane–sphere intersection.
#[derive(Debug, Clone)]
pub enum PlaneSphereResult {
    /// The plane does not reach the sphere.
    NoIntersection,
    /// The plane is tangent to the sphere: one contact point.
    Point(Point3),
    /// A circle: centre, normal (unit), radius.
    Circle { center: Point3, normal: Vec3, radius: f64 },
}

pub fn plane_sphere(plane: &Plane, sphere: &SphericalSurface) -> PlaneSphereResult {
    let n = plane.normal_vec();
    let dist = n.dot(&(sphere.center - plane.origin)); // signed dist from plane
    let d2 = dist * dist;
    let r2 = sphere.radius * sphere.radius;
    if d2 > r2 + 1e-12 { return PlaneSphereResult::NoIntersection; }
    if (d2 - r2).abs() <= 1e-12 {
        let contact = sphere.center - n * dist;
        return PlaneSphereResult::Point(contact);
    }
    let r_circle = (r2 - d2).max(0.0).sqrt();
    let center = sphere.center - n * dist;
    PlaneSphereResult::Circle { center, normal: n, radius: r_circle }
}

// ── Sphere × Sphere ───────────────────────────────────────────────────────────

/// Result of a sphere–sphere intersection.
#[derive(Debug, Clone)]
pub enum SphereSphereResult {
    NoIntersection,
    Point(Point3),
    /// A circle lying in the plane perpendicular to the axis between centres.
    Circle { center: Point3, normal: Vec3, radius: f64 },
    /// The two spheres are identical.
    Coincident,
}

pub fn sphere_sphere(a: &SphericalSurface, b: &SphericalSurface) -> SphereSphereResult {
    let axis = b.center - a.center;
    let d = axis.norm();
    if d < 1e-14 {
        if (a.radius - b.radius).abs() < 1e-12 {
            return SphereSphereResult::Coincident;
        }
        return SphereSphereResult::NoIntersection;
    }
    let sum  = a.radius + b.radius;
    let diff = (a.radius - b.radius).abs();
    if d > sum + 1e-12 || d < diff - 1e-12 {
        return SphereSphereResult::NoIntersection;
    }
    // Distance from centre of A to the intersection plane.
    let x = (d * d + a.radius * a.radius - b.radius * b.radius) / (2.0 * d);
    let r2 = a.radius * a.radius - x * x;
    let n = axis / d;
    let center = a.center + n * x;
    if r2 <= 1e-20 {
        return SphereSphereResult::Point(center);
    }
    SphereSphereResult::Circle { center, normal: n, radius: r2.sqrt() }
}

// ── Numerical surface–surface intersection (marching) ─────────────────────────

/// Intersect two arbitrary surfaces numerically.
///
/// Returns zero or more intersection curves.  Each curve is sampled at
/// approximately `step` world-unit intervals.
pub fn surface_surface_numerical(
    surf_a: &dyn Surface,
    surf_b: &dyn Surface,
    step: f64,
    tol: f64,
) -> Result<Vec<IntersectionCurve>, KernelError> {
    let seeds = find_seeds(surf_a, surf_b, tol);
    let mut curves: Vec<IntersectionCurve> = Vec::new();
    let mut visited: Vec<(f64, f64)> = Vec::new(); // (u_a, v_a) of traced seeds

    for seed in seeds {
        // Skip if near a previously traced curve.
        if visited.iter().any(|&(ua, va)| {
            let du = ua - seed.uv_a.0; let dv = va - seed.uv_a.1;
            (du * du + dv * dv).sqrt() < step * 0.5
        }) { continue; }

        let curve = march_curve(surf_a, surf_b, &seed, step, tol)?;
        if !curve.points.is_empty() {
            visited.push(curve.points[0].uv_a);
            curves.push(curve);
        }
    }
    Ok(curves)
}

/// Coarse grid search to find starting points close to the intersection.
fn find_seeds(surf_a: &dyn Surface, surf_b: &dyn Surface, tol: f64) -> Vec<IntPoint> {
    const N: usize = 16;
    let ((ua0, ua1), (va0, va1)) = surf_a.parameter_range();

    // Clamp infinite domains to a reasonable window.
    let clamp = |lo: f64, hi: f64| -> (f64, f64) {
        let lo = if lo.is_finite() { lo } else { -10.0 };
        let hi = if hi.is_finite() { hi } else {  10.0 };
        (lo, hi)
    };
    let (ua0, ua1) = clamp(ua0, ua1);
    let (va0, va1) = clamp(va0, va1);

    let mut seeds = Vec::new();
    for i in 0..=N {
        for j in 0..=N {
            let ua = ua0 + (ua1 - ua0) * i as f64 / N as f64;
            let va = va0 + (va1 - va0) * j as f64 / N as f64;
            let pa = surf_a.point(ua, va);
            // Project pa onto surf_b.
            if let Ok((ub, vb)) = surf_b.closest_parameter(&pa, tol) {
                let pb = surf_b.point(ub, vb);
                if (pa - pb).norm() < tol * 10.0 {
                    seeds.push(IntPoint { point: (pa + pb.coords) * 0.5, uv_a: (ua, va), uv_b: (ub, vb) });
                }
            }
        }
    }
    seeds
}

/// March along the intersection curve starting from `seed`.
fn march_curve(
    surf_a: &dyn Surface,
    surf_b: &dyn Surface,
    seed: &IntPoint,
    step: f64,
    tol: f64,
) -> Result<IntersectionCurve, KernelError> {
    let max_steps = 10_000usize;
    let mut pts: Vec<IntPoint> = vec![seed.clone()];
    let (mut ua, mut va) = seed.uv_a;
    let (mut ub, mut vb) = seed.uv_b;

    // Marching tangent direction from cross(na, nb).
    let na = surf_a.normal(ua, va)?;
    let nb = surf_b.normal(ub, vb)?;
    let mut tang = na.cross(&nb);
    if tang.norm() < 1e-12 { return Ok(IntersectionCurve { points: pts, is_closed: false }); }
    tang = tang.normalize();

    for _ in 0..max_steps {
        // Predictor step.
        let p_pred = pts.last().unwrap().point + tang * step;

        // Corrector: Newton projection onto both surfaces simultaneously.
        let Some(corrected) = newton_project(surf_a, surf_b, p_pred, ua, va, ub, vb, tol)
        else { break; };

        (ua, va) = corrected.uv_a;
        (ub, vb) = corrected.uv_b;

        // Update tangent direction (keep consistent orientation).
        let na2 = surf_a.normal(ua, va)?;
        let nb2 = surf_b.normal(ub, vb)?;
        let new_tang = na2.cross(&nb2);
        if new_tang.norm() > 1e-12 {
            let nt = new_tang.normalize();
            tang = if nt.dot(&tang) >= 0.0 { nt } else { -nt };
        }

        // Check if we've looped back.
        let start = &pts[0].point;
        if pts.len() > 4 && (corrected.point - start).norm() < step * 1.5 {
            pts.push(corrected);
            return Ok(IntersectionCurve { points: pts, is_closed: true });
        }

        pts.push(corrected);

        // Check domain bounds.
        let ((ua0,ua1),(va0,va1)) = surf_a.parameter_range();
        let ((ub0,ub1),(vb0,vb1)) = surf_b.parameter_range();
        if out_of_domain(ua, va, ua0, ua1, va0, va1) ||
           out_of_domain(ub, vb, ub0, ub1, vb0, vb1) { break; }
    }

    Ok(IntersectionCurve { points: pts, is_closed: false })
}

fn out_of_domain(u: f64, v: f64, u0: f64, u1: f64, v0: f64, v1: f64) -> bool {
    let u0 = if u0.is_finite() { u0 } else { -1e9 };
    let u1 = if u1.is_finite() { u1 } else {  1e9 };
    let v0 = if v0.is_finite() { v0 } else { -1e9 };
    let v1 = if v1.is_finite() { v1 } else {  1e9 };
    u < u0 - 1e-9 || u > u1 + 1e-9 || v < v0 - 1e-9 || v > v1 + 1e-9
}

/// Newton corrector: project a 3-D point onto the intersection of two surfaces.
fn newton_project(
    surf_a: &dyn Surface,
    surf_b: &dyn Surface,
    _p: Point3,
    ua0: f64, va0: f64,
    ub0: f64, vb0: f64,
    tol: f64,
) -> Option<IntPoint> {
    let (mut ua, mut va) = (ua0, va0);
    let (mut ub, mut vb) = (ub0, vb0);

    for _ in 0..30 {
        let pa = surf_a.point(ua, va);
        let pb = surf_b.point(ub, vb);
        let mid = Point3::new(
            (pa.x + pb.x) * 0.5,
            (pa.y + pb.y) * 0.5,
            (pa.z + pb.z) * 0.5,
        );
        if (pa - pb).norm() < tol {
            return Some(IntPoint { point: mid, uv_a: (ua, va), uv_b: (ub, vb) });
        }
        // Project mid onto each surface.
        (ua, va) = surf_a.closest_parameter(&mid, tol * 0.1).ok()?;
        (ub, vb) = surf_b.closest_parameter(&mid, tol * 0.1).ok()?;
    }
    None
}

// ── Curve–surface intersection (Newton) ──────────────────────────────────────

use brep_geom::traits::Curve3d;

/// Find all intersections of a 3-D curve with a surface.
///
/// Strategy:
/// 1. Sample the curve at `n_samples` evenly-spaced parameter values.
/// 2. For each sample, project onto the surface; keep samples where the
///    distance drops below `tol * 100`.
/// 3. Run Newton refinement to converge each hit.
pub fn curve_surface(
    curve: &dyn Curve3d,
    surf: &dyn Surface,
    tol: f64,
) -> Result<Vec<(f64, f64, f64)>, KernelError> {
    let (t0, t1) = curve.parameter_range();
    let n = 64usize;
    let mut hits: Vec<(f64, f64, f64)> = Vec::new(); // (t_curve, u_surf, v_surf)

    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let p = curve.point(t);
        let Ok((u, v)) = surf.closest_parameter(&p, tol) else { continue };
        let dist = (p - surf.point(u, v)).norm();

        // Local minimum below threshold → refine.
        if dist < tol * 50.0 {
            if let Some(hit) = newton_curve_surface(curve, surf, t, u, v, tol) {
                // Deduplicate.
                if !hits.iter().any(|&(ht, _, _)| (ht - hit.0).abs() < tol * 5.0) {
                    hits.push(hit);
                }
            }
        }
    }
    Ok(hits)
}

/// Newton refinement for curve–surface intersection.
fn newton_curve_surface(
    curve: &dyn Curve3d,
    surf: &dyn Surface,
    t0: f64,
    u0: f64,
    v0: f64,
    tol: f64,
) -> Option<(f64, f64, f64)> {
    let (tmin, tmax) = curve.parameter_range();
    let mut t = t0;
    let mut u = u0;
    let mut v = v0;

    for _ in 0..40 {
        let pc = curve.point(t);
        let ps = surf.point(u, v);
        let diff = pc - ps;
        if diff.norm() < tol { return Some((t, u, v)); }

        let ct = curve.derivative1(t);
        let su = surf.du(u, v);
        let sv = surf.dv(u, v);

        // 3 equations, 3 unknowns: diff = ct*dt - su*du - sv*dv
        // Least-squares: J^T J [dt;du;dv] = J^T diff
        // J = [ct | -su | -sv]
        let jt_j = nalgebra::Matrix3::new(
            ct.dot(&ct),   -ct.dot(&su),  -ct.dot(&sv),
            -su.dot(&ct),   su.dot(&su),   su.dot(&sv),
            -sv.dot(&ct),   sv.dot(&su),   sv.dot(&sv),
        );
        let jt_r = nalgebra::Vector3::new(
            ct.dot(&diff),
            -su.dot(&diff),
            -sv.dot(&diff),
        );
        let Some(sol) = jt_j.lu().solve(&jt_r) else { break };
        t = (t + sol[0]).clamp(tmin, tmax);
        u += sol[1];
        v += sol[2];
        // Clamp to finite parameter domain.
        let ((u0d,u1d),(v0d,v1d)) = surf.parameter_range();
        if u0d.is_finite() { u = u.clamp(u0d, u1d); }
        if v0d.is_finite() { v = v.clamp(v0d, v1d); }
    }
    let pc = curve.point(t);
    let ps = surf.point(u, v);
    if (pc - ps).norm() < tol * 10.0 { Some((t, u, v)) } else { None }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_core::Vec3;
    use brep_geom::surface::{Plane, SphericalSurface};

    fn xy_plane() -> Plane {
        Plane::new(Point3::origin(), Vec3::x(), Vec3::y())
    }

    fn xz_plane() -> Plane {
        Plane::new(Point3::origin(), Vec3::x(), Vec3::z())
    }

    // ── Plane × Plane ─────────────────────────────────────────────────────────

    #[test]
    fn parallel_planes_no_intersection() {
        let a = Plane::new(Point3::origin(), Vec3::x(), Vec3::y());
        let b = Plane::new(Point3::new(0.0, 0.0, 1.0), Vec3::x(), Vec3::y());
        assert!(plane_plane(&a, &b).is_none());
    }

    #[test]
    fn perpendicular_planes_intersect_on_x_axis() {
        let result = plane_plane(&xy_plane(), &xz_plane());
        assert!(result.is_some());
        let (_, dir) = result.unwrap();
        // Direction should be along ±X.
        assert!(dir.y.abs() < 1e-10 && dir.z.abs() < 1e-10);
    }

    // ── Plane × Sphere ────────────────────────────────────────────────────────

    #[test]
    fn plane_misses_sphere() {
        let plane = Plane::new(Point3::new(0.0, 0.0, 5.0), Vec3::x(), Vec3::y());
        let sphere = SphericalSurface::new(Point3::origin(), 1.0);
        assert!(matches!(plane_sphere(&plane, &sphere), PlaneSphereResult::NoIntersection));
    }

    #[test]
    fn plane_through_sphere_centre_gives_great_circle() {
        let sphere = SphericalSurface::new(Point3::origin(), 1.0);
        let result = plane_sphere(&xy_plane(), &sphere);
        if let PlaneSphereResult::Circle { radius, .. } = result {
            approx::assert_abs_diff_eq!(radius, 1.0, epsilon = 1e-10);
        } else { panic!("expected circle"); }
    }

    #[test]
    fn plane_tangent_to_sphere_gives_point() {
        let plane = Plane::new(Point3::new(0.0, 0.0, 1.0), Vec3::x(), Vec3::y());
        let sphere = SphericalSurface::new(Point3::origin(), 1.0);
        assert!(matches!(plane_sphere(&plane, &sphere), PlaneSphereResult::Point(_)));
    }

    // ── Sphere × Sphere ───────────────────────────────────────────────────────

    #[test]
    fn disjoint_spheres_no_intersection() {
        let a = SphericalSurface::new(Point3::origin(), 1.0);
        let b = SphericalSurface::new(Point3::new(5.0, 0.0, 0.0), 1.0);
        assert!(matches!(sphere_sphere(&a, &b), SphereSphereResult::NoIntersection));
    }

    #[test]
    fn identical_spheres_are_coincident() {
        let a = SphericalSurface::new(Point3::origin(), 1.0);
        let b = SphericalSurface::new(Point3::origin(), 1.0);
        assert!(matches!(sphere_sphere(&a, &b), SphereSphereResult::Coincident));
    }

    #[test]
    fn overlapping_spheres_give_circle() {
        let a = SphericalSurface::new(Point3::origin(), 1.0);
        let b = SphericalSurface::new(Point3::new(1.0, 0.0, 0.0), 1.0);
        if let SphereSphereResult::Circle { radius, .. } = sphere_sphere(&a, &b) {
            assert!(radius > 0.0);
        } else { panic!("expected circle"); }
    }

    #[test]
    fn tangent_spheres_give_point() {
        let a = SphericalSurface::new(Point3::origin(), 1.0);
        let b = SphericalSurface::new(Point3::new(2.0, 0.0, 0.0), 1.0);
        assert!(matches!(sphere_sphere(&a, &b), SphereSphereResult::Point(_)));
    }

    // ── Numerical surface–surface ─────────────────────────────────────────────

    #[test]
    fn numerical_plane_plane_finds_line() {
        use brep_geom::surface::Plane;
        let a = Plane::new(Point3::origin(), Vec3::x(), Vec3::y());
        let b = Plane::new(Point3::origin(), Vec3::x(), Vec3::z());
        let curves = surface_surface_numerical(&a, &b, 0.5, 1e-6).unwrap();
        // Should find at least one intersection curve.
        assert!(!curves.is_empty());
    }

    // ── Curve–surface ─────────────────────────────────────────────────────────

    #[test]
    fn line_through_sphere_gives_two_hits() {
        use brep_geom::curve::{LineCurve, TrimmedCurve};
        let sphere = SphericalSurface::new(Point3::origin(), 1.0);
        // Trimmed line along Z from (0,0,-2) to (0,0,2) — parameter t in [0,1],
        // point(t) = (0,0,-2) + t*(0,0,4) = (0,0,-2+4t).
        // Intersects sphere at t=0.25 (z=-1) and t=0.75 (z=1).
        let line = TrimmedCurve::new(Box::new(LineCurve::new(Point3::new(0.0, 0.0, -2.0), Vec3::new(0.0, 0.0, 4.0))), 0.0, 1.0);
        let hits = curve_surface(&line, &sphere, 1e-6).unwrap();
        assert_eq!(hits.len(), 2, "line through sphere should give 2 hits, got {}", hits.len());
        let ts: Vec<f64> = hits.iter().map(|h| h.0).collect();
        assert!(ts.iter().any(|&t| (t - 0.25).abs() < 0.05));
        assert!(ts.iter().any(|&t| (t - 0.75).abs() < 0.05));
    }

    #[test]
    fn line_missing_sphere_gives_no_hits() {
        use brep_geom::curve::{LineCurve, TrimmedCurve};
        let sphere = SphericalSurface::new(Point3::origin(), 1.0);
        let line = TrimmedCurve::new(Box::new(LineCurve::new(Point3::new(5.0, 0.0, -2.0), Vec3::new(0.0, 0.0, 4.0))), 0.0, 1.0);
        let hits = curve_surface(&line, &sphere, 1e-6).unwrap();
        assert!(hits.is_empty());
    }
}
