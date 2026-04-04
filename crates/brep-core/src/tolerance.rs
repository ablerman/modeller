//! Tolerance context carried by every [`ShapeStore`].
//!
//! Tolerances are *not* global constants.  Different shapes produced
//! by different operations may require different precision levels.
//! Additionally, individual vertices and edges store their own
//! per-entity tolerance when they are the result of imprecise operations.

use serde::{Deserialize, Serialize};

/// Numerical tolerances for a shape or operation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToleranceContext {
    /// Linear distance below which two points are considered coincident (metres).
    pub linear: f64,
    /// Angular tolerance in radians.
    pub angular: f64,
    /// Parametric tolerance for curve / surface parameters.
    pub parametric: f64,
}

impl Default for ToleranceContext {
    fn default() -> Self {
        Self {
            linear: 1e-7,
            angular: 1e-9,
            parametric: 1e-9,
        }
    }
}

impl ToleranceContext {
    /// Construct with explicit values.
    pub fn new(linear: f64, angular: f64, parametric: f64) -> Self {
        Self { linear, angular, parametric }
    }

    /// Looser tolerances suitable for visualisation or rough analysis.
    pub fn coarse() -> Self {
        Self { linear: 1e-4, angular: 1e-5, parametric: 1e-5 }
    }

    /// Tighter tolerances for high-precision manufacturing.
    pub fn fine() -> Self {
        Self { linear: 1e-9, angular: 1e-12, parametric: 1e-12 }
    }

    /// Return the tolerance that is the *maximum* (least strict) of both.
    /// Used when merging entities from two differently-toleranced shapes.
    pub fn relaxed_union(&self, other: &ToleranceContext) -> ToleranceContext {
        ToleranceContext {
            linear: self.linear.max(other.linear),
            angular: self.angular.max(other.angular),
            parametric: self.parametric.max(other.parametric),
        }
    }

    /// Compute the combined vertex tolerance after snapping two vertices
    /// that are `distance` apart.
    ///
    /// ```
    /// # use brep_core::tolerance::ToleranceContext;
    /// let ctx = ToleranceContext::default();
    /// let combined = ctx.merged_vertex_tolerance(ctx.linear * 0.5, ctx.linear);
    /// assert!(combined >= ctx.linear);
    /// ```
    pub fn merged_vertex_tolerance(&self, tol_a: f64, tol_b: f64) -> f64 {
        tol_a.max(tol_b) + self.linear
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_values_are_sane() {
        let t = ToleranceContext::default();
        assert!(t.linear > 0.0 && t.linear < 1e-4);
        assert!(t.angular > 0.0 && t.angular < 1e-6);
    }

    #[test]
    fn coarse_looser_than_default() {
        let d = ToleranceContext::default();
        let c = ToleranceContext::coarse();
        assert!(c.linear > d.linear);
        assert!(c.angular > d.angular);
    }

    #[test]
    fn fine_tighter_than_default() {
        let d = ToleranceContext::default();
        let f = ToleranceContext::fine();
        assert!(f.linear < d.linear);
    }

    #[test]
    fn relaxed_union_takes_max() {
        let a = ToleranceContext::new(1e-7, 1e-9, 1e-9);
        let b = ToleranceContext::new(1e-5, 1e-8, 1e-8);
        let u = a.relaxed_union(&b);
        assert_eq!(u.linear, 1e-5);
        assert_eq!(u.angular, 1e-8);
    }

    #[test]
    fn merged_vertex_tolerance_increases() {
        let ctx = ToleranceContext::default();
        let t = ctx.merged_vertex_tolerance(ctx.linear, ctx.linear);
        assert!(t >= ctx.linear);
    }

    #[test]
    fn serde_roundtrip() {
        let t = ToleranceContext::default();
        let json = serde_json::to_string(&t).unwrap();
        let t2: ToleranceContext = serde_json::from_str(&json).unwrap();
        assert_eq!(t, t2);
    }
}
