//! `brep-core` — shared vocabulary for the BRep modeling kernel.
//!
//! Provides:
//! - [`EntityId`] — generational arena handle (Copy, no ownership)
//! - [`ToleranceContext`] — linear / angular / parametric tolerances
//! - [`KernelError`] — unified error type
//! - Math type aliases wrapping `nalgebra`
//! - [`Aabb`] — axis-aligned bounding box

pub mod error;
pub mod id;
pub mod math;
pub mod tolerance;

pub use error::KernelError;
pub use id::EntityId;
pub use math::{Aabb, Iso3, Mat4, Point2, Point3, Vec2, Vec3};
pub use tolerance::ToleranceContext;
