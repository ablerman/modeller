//! 2D sketch constraint solver.
//!
//! This crate is deliberately dependency-free (just nalgebra) so it can be
//! developed and tested independently of the BRep kernel.
//!
//! # Usage
//!
//! ```rust
//! use brep_sketch::{SketchConstraint, solve_constraints, SolveResult};
//!
//! let mut pts = vec![[0.0, 0.0], [1.0, 0.2], [1.0, 1.0], [0.0, 1.0]];
//! let constraints = vec![SketchConstraint::Horizontal { seg: 0 }];
//! assert!(matches!(solve_constraints(&mut pts, &constraints, 4), SolveResult::Ok));
//! // pts[0] and pts[1] now have the same y coordinate.
//! ```

pub mod constraints;
pub mod solver;

pub use constraints::SketchConstraint;
pub use solver::{solve_constraints, SolveResult};
