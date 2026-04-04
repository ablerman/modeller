//! Geometry bindings: connect topology to geometry.
//!
//! A binding is the glue between a topological entity (Edge, Face) and
//! its geometric representation (a 3-D curve or surface).  Bindings carry
//! a parameter range and an orientation flag (`same_sense`) that says
//! whether the topology traverses the geometry in the natural direction.

use std::sync::Arc;

use brep_geom::traits::{Curve2d, Curve3d, Surface};

/// Geometry bound to an edge: a 3-D curve + parameter interval.
#[derive(Clone)]
pub struct CurveBinding {
    /// The underlying 3-D curve.  `Arc` allows sharing between edges
    /// that lie on the same analytic curve.
    pub curve: Arc<dyn Curve3d>,
    /// Parameter at the start vertex (in the edge's forward direction).
    pub t_start: f64,
    /// Parameter at the end vertex.
    pub t_end: f64,
    /// True when the edge traverses the curve in the curve's natural
    /// parameter direction (t_start → t_end with increasing t).
    pub same_sense: bool,
}

impl CurveBinding {
    pub fn new(curve: Arc<dyn Curve3d>, t_start: f64, t_end: f64, same_sense: bool) -> Self {
        Self { curve, t_start, t_end, same_sense }
    }

    /// Effective start parameter in curve space (accounting for same_sense).
    pub fn curve_t_start(&self) -> f64 {
        if self.same_sense { self.t_start } else { self.t_end }
    }

    /// Effective end parameter in curve space.
    pub fn curve_t_end(&self) -> f64 {
        if self.same_sense { self.t_end } else { self.t_start }
    }
}

impl std::fmt::Debug for CurveBinding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CurveBinding")
            .field("curve_type", &self.curve.type_name())
            .field("t_start", &self.t_start)
            .field("t_end", &self.t_end)
            .field("same_sense", &self.same_sense)
            .finish()
    }
}

/// Geometry bound to a face: a surface + orientation flag.
#[derive(Clone)]
pub struct SurfaceBinding {
    /// The underlying surface.
    pub surface: Arc<dyn Surface>,
    /// True when the face's outward normal agrees with the surface normal.
    pub same_sense: bool,
}

impl SurfaceBinding {
    pub fn new(surface: Arc<dyn Surface>, same_sense: bool) -> Self {
        Self { surface, same_sense }
    }
}

impl std::fmt::Debug for SurfaceBinding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SurfaceBinding")
            .field("surface_type", &self.surface.type_name())
            .field("same_sense", &self.same_sense)
            .finish()
    }
}

/// A 2-D parameter-space curve (pcurve) on a face's surface, binding a
/// half-edge to the parameter domain of the adjacent face.
///
/// Pcurves are **mandatory** for all non-degenerate half-edges on curved
/// faces.  They enable:
/// - Correct classification of points relative to face boundaries
/// - 2-D intersection computation in parameter space (more robust than 3-D)
/// - STEP compliance (AP214/AP242 requires pcurves)
#[derive(Clone)]
pub struct Pcurve {
    /// The 2-D curve in `(u, v)` parameter space.
    pub curve2d: Arc<dyn Curve2d>,
    /// Parameter at the start vertex of the half-edge.
    pub t_start: f64,
    /// Parameter at the end vertex.
    pub t_end: f64,
    /// True when the half-edge traverses the curve in the natural direction.
    pub same_sense: bool,
}

impl Pcurve {
    pub fn new(curve2d: Arc<dyn Curve2d>, t_start: f64, t_end: f64, same_sense: bool) -> Self {
        Self { curve2d, t_start, t_end, same_sense }
    }
}

impl std::fmt::Debug for Pcurve {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Pcurve")
            .field("curve2d_type", &self.curve2d.type_name())
            .field("t_start", &self.t_start)
            .field("t_end", &self.t_end)
            .field("same_sense", &self.same_sense)
            .finish()
    }
}
