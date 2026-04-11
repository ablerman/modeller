//! 2D sketch constraint solver and parametric sketcher.
//!
//! This crate provides two levels of API:
//!
//! ## High-level: [`Sketch`]
//!
//! A parametric sketch with typed IDs, profiles (polyline, arc, circle),
//! and a unified [`Constraint`] enum.  Use this when building applications.
//!
//! ```rust
//! use brep_sketch::{Sketch, Constraint, Plane};
//! use brep_sketch::profile::ProfileShape;
//!
//! let mut sketch = Sketch::new(Plane::xy(), "my sketch");
//!
//! let pid = sketch.add_polyline_profile(false);
//! let p0 = sketch.push_point(pid, (0.0, 0.0)).unwrap();
//! let p1 = sketch.push_point(pid, (1.0, 0.1)).unwrap();
//!
//! sketch.add_constraint(Constraint::Horizontal { profile: pid, seg: 0 }).unwrap();
//! let report = sketch.solve();
//! assert!(report.converged);
//! // p0 and p1 now share the same v coordinate.
//! ```
//!
//! ## Low-level: [`solve_constraints`] / [`apply_constraints`]
//!
//! The raw Gauss-Newton solver operating on a flat `Vec<[f64; 2]>` point array.
//! Use this when you already manage points and constraints yourself.
//!
//! ```rust
//! use brep_sketch::{SketchConstraint, solve_constraints, SolveResult};
//!
//! let mut pts = vec![[0.0, 0.0], [1.0, 0.2], [1.0, 1.0], [0.0, 1.0]];
//! let constraints = vec![SketchConstraint::Horizontal { seg: 0 }];
//! assert!(matches!(solve_constraints(&mut pts, &constraints, 4), SolveResult::Ok));
//! ```

pub mod arc;
pub mod constraints;
pub mod geometry;
pub mod profile;
pub mod sketch;
pub mod solver;

// ── High-level re-exports ─────────────────────────────────────────────────────
pub use constraints::Constraint;
pub use geometry::Plane;
pub use profile::{ConstraintId, GlobalPointId, PointId, Profile, ProfileId, ProfileShape};
pub use sketch::{ConstraintStatus, FullSolveReport, Sketch, SketchError, SketchResult, SketchSnapshot};

// ── Low-level re-exports (backward-compatible) ────────────────────────────────
pub use constraints::SketchConstraint;
pub use solver::{apply_constraints, ApplyResult, solve_constraints, SolveResult};
