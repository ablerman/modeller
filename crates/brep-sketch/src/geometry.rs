//! Sketch plane — origin + two orthonormal axes.
//!
//! No external crate dependencies; coordinates are plain `[f64; 3]`.

/// A sketch plane defined by an origin and two orthonormal axes.
///
/// Invariant: `u_axis` and `v_axis` must be unit vectors and mutually
/// orthogonal.  The crate does not enforce this at runtime — callers are
/// responsible.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Plane {
    /// World-space origin of the sketch plane.
    pub origin: [f64; 3],
    /// World-space unit vector along the sketch U axis.
    pub u_axis: [f64; 3],
    /// World-space unit vector along the sketch V axis (must be ⊥ u_axis).
    pub v_axis: [f64; 3],
}

impl Plane {
    /// Project a world-space point `[x, y, z]` to sketch UV coordinates `(u, v)`.
    #[inline]
    pub fn world_to_uv(&self, p: [f64; 3]) -> (f64, f64) {
        let rel = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        (dot3(rel, self.u_axis), dot3(rel, self.v_axis))
    }

    /// Lift UV sketch coordinates back to world space.
    #[inline]
    pub fn uv_to_world(&self, u: f64, v: f64) -> [f64; 3] {
        [
            self.origin[0] + u * self.u_axis[0] + v * self.v_axis[0],
            self.origin[1] + u * self.u_axis[1] + v * self.v_axis[1],
            self.origin[2] + u * self.u_axis[2] + v * self.v_axis[2],
        ]
    }

    /// XY plane (z = 0), origin at the world origin.
    /// U = world X, V = world Y.
    pub fn xy() -> Self {
        Self {
            origin: [0.0; 3],
            u_axis: [1.0, 0.0, 0.0],
            v_axis: [0.0, 1.0, 0.0],
        }
    }

    /// XZ plane (y = 0), origin at the world origin.
    /// U = world X, V = world Z.
    pub fn xz() -> Self {
        Self {
            origin: [0.0; 3],
            u_axis: [1.0, 0.0, 0.0],
            v_axis: [0.0, 0.0, 1.0],
        }
    }

    /// YZ plane (x = 0), origin at the world origin.
    /// U = world Y, V = world Z.
    pub fn yz() -> Self {
        Self {
            origin: [0.0; 3],
            u_axis: [0.0, 1.0, 0.0],
            v_axis: [0.0, 0.0, 1.0],
        }
    }
}

#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xy_round_trip() {
        let plane = Plane::xy();
        let world = [3.0, 4.0, 0.0];
        let (u, v) = plane.world_to_uv(world);
        assert!((u - 3.0).abs() < 1e-12);
        assert!((v - 4.0).abs() < 1e-12);
        let back = plane.uv_to_world(u, v);
        assert!((back[0] - world[0]).abs() < 1e-12);
        assert!((back[1] - world[1]).abs() < 1e-12);
    }

    #[test]
    fn xz_round_trip() {
        let plane = Plane::xz();
        let world = [2.0, 0.0, 5.0];
        let (u, v) = plane.world_to_uv(world);
        assert!((u - 2.0).abs() < 1e-12);
        assert!((v - 5.0).abs() < 1e-12);
        let back = plane.uv_to_world(u, v);
        assert!((back[0] - world[0]).abs() < 1e-12);
        assert!((back[2] - world[2]).abs() < 1e-12);
    }

    #[test]
    fn arbitrary_origin() {
        let plane = Plane {
            origin: [1.0, 2.0, 3.0],
            u_axis: [1.0, 0.0, 0.0],
            v_axis: [0.0, 1.0, 0.0],
        };
        let (u, v) = plane.world_to_uv([4.0, 6.0, 3.0]);
        assert!((u - 3.0).abs() < 1e-12);
        assert!((v - 4.0).abs() < 1e-12);
        let back = plane.uv_to_world(u, v);
        assert!((back[0] - 4.0).abs() < 1e-12);
        assert!((back[1] - 6.0).abs() < 1e-12);
    }
}
