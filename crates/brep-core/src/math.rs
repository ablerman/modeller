//! Math type aliases and [`Aabb`].
//!
//! All geometry uses `f64` precision.  We alias `nalgebra` types
//! rather than wrapping them so that callers can use the full
//! `nalgebra` API without friction.

use nalgebra as na;
use serde::{Deserialize, Serialize};

// ── Type aliases ──────────────────────────────────────────────────────────────

pub type Point3 = na::Point3<f64>;
pub type Point2 = na::Point2<f64>;
pub type Vec3 = na::Vector3<f64>;
pub type Vec2 = na::Vector2<f64>;
pub type Mat4 = na::Matrix4<f64>;
/// Rigid-body isometry in 3-D (rotation + translation, no scale).
pub type Iso3 = na::Isometry3<f64>;

// ── Axis-aligned bounding box ─────────────────────────────────────────────────

/// An axis-aligned bounding box in 3-D.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    pub min: Point3,
    pub max: Point3,
}

impl Aabb {
    /// Construct from two corner points (order does not matter).
    pub fn from_corners(a: Point3, b: Point3) -> Self {
        Self {
            min: Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// An empty/invalid AABB (min > max on all axes).
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::MAX, f64::MAX, f64::MAX),
            max: Point3::new(f64::MIN, f64::MIN, f64::MIN),
        }
    }

    /// Returns `true` if this AABB contains no volume.
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Smallest AABB that contains both `self` and `other`.
    pub fn union(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: Point3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Point3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }

    /// Expand every face outward by `amount`.
    pub fn expand_by(&self, amount: f64) -> Aabb {
        let d = Vec3::new(amount, amount, amount);
        Aabb { min: self.min - d, max: self.max + d }
    }

    /// Test overlap with `other` (touching counts as overlapping).
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Test whether `point` lies inside (inclusive).
    pub fn contains_point(&self, p: &Point3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Centre of the box.
    pub fn center(&self) -> Point3 {
        na::center(&self.min, &self.max)
    }

    /// Half-extents (half the side length on each axis).
    pub fn half_extents(&self) -> Vec3 {
        (self.max - self.min) * 0.5
    }

    /// Extend the box to include `point`.
    pub fn include_point(&mut self, p: &Point3) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn pt(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn from_corners_normalises_order() {
        let a = Aabb::from_corners(pt(3.0, 3.0, 3.0), pt(1.0, 1.0, 1.0));
        assert_abs_diff_eq!(a.min.x, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(a.max.x, 3.0, epsilon = 1e-12);
    }

    #[test]
    fn empty_is_empty() {
        assert!(Aabb::empty().is_empty());
    }

    #[test]
    fn non_empty_is_not_empty() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        assert!(!a.is_empty());
    }

    #[test]
    fn union_encloses_both() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        let b = Aabb::from_corners(pt(0.5, 0.5, 0.5), pt(2.0, 2.0, 2.0));
        let u = a.union(&b);
        assert_abs_diff_eq!(u.min.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(u.max.x, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn expand_by_grows_uniformly() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(2.0, 2.0, 2.0));
        let e = a.expand_by(1.0);
        assert_abs_diff_eq!(e.min.x, -1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(e.max.x, 3.0, epsilon = 1e-12);
    }

    #[test]
    fn intersects_overlapping() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(2.0, 2.0, 2.0));
        let b = Aabb::from_corners(pt(1.0, 1.0, 1.0), pt(3.0, 3.0, 3.0));
        assert!(a.intersects(&b));
    }

    #[test]
    fn intersects_touching() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        let b = Aabb::from_corners(pt(1.0, 0.0, 0.0), pt(2.0, 1.0, 1.0));
        assert!(a.intersects(&b)); // touching face counts
    }

    #[test]
    fn not_intersects_disjoint() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        let b = Aabb::from_corners(pt(2.0, 0.0, 0.0), pt(3.0, 1.0, 1.0));
        assert!(!a.intersects(&b));
    }

    #[test]
    fn contains_point_inside() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(2.0, 2.0, 2.0));
        assert!(a.contains_point(&pt(1.0, 1.0, 1.0)));
    }

    #[test]
    fn contains_point_on_boundary() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(2.0, 2.0, 2.0));
        assert!(a.contains_point(&pt(0.0, 0.0, 0.0)));
        assert!(a.contains_point(&pt(2.0, 2.0, 2.0)));
    }

    #[test]
    fn does_not_contain_outside() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        assert!(!a.contains_point(&pt(2.0, 0.5, 0.5)));
    }

    #[test]
    fn center_is_midpoint() {
        let a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(4.0, 4.0, 4.0));
        let c = a.center();
        assert_abs_diff_eq!(c.x, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn include_point_expands() {
        let mut a = Aabb::from_corners(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0));
        a.include_point(&pt(5.0, -1.0, 0.5));
        assert_abs_diff_eq!(a.max.x, 5.0, epsilon = 1e-12);
        assert_abs_diff_eq!(a.min.y, -1.0, epsilon = 1e-12);
    }
}
