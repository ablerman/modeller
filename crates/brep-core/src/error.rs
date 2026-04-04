//! Unified error type for the BRep kernel.

use thiserror::Error;

/// All errors that can arise from the BRep kernel.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum KernelError {
    /// Topological invariant violated (e.g. open edge in closed shell).
    #[error("invalid topology: {0}")]
    InvalidTopology(String),

    /// Geometry evaluation failed at a specific parameter.
    #[error("geometry evaluation failed on {entity} at t={parameter}: {reason}")]
    GeometryEvaluation { entity: String, parameter: f64, reason: String },

    /// A numerical algorithm (Newton, marching, etc.) failed to converge.
    #[error("numerical failure in {algorithm}: {details}")]
    NumericalFailure { algorithm: String, details: String },

    /// Degenerate geometry encountered (zero-length edge, collapsed surface).
    #[error("degenerate geometry: {0}")]
    DegenerateGeometry(String),

    /// A stale or invalid [`EntityId`] was used to look up an entity.
    #[error("invalid or stale entity id")]
    InvalidEntityId,

    /// The requested operation is not supported for the given geometry types.
    #[error("operation not supported: {0}")]
    OperationNotSupported(String),

    /// I/O error from file reading/writing.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Parse error from STEP/IGES reader.
    #[error("parse error: {0}")]
    Parse(String),
}

impl KernelError {
    /// Convenience constructor for [`KernelError::NumericalFailure`].
    pub fn numerical(algorithm: impl Into<String>, details: impl Into<String>) -> Self {
        Self::NumericalFailure { algorithm: algorithm.into(), details: details.into() }
    }

    /// Convenience constructor for [`KernelError::GeometryEvaluation`].
    pub fn geom_eval(entity: impl Into<String>, parameter: f64, reason: impl Into<String>) -> Self {
        Self::GeometryEvaluation {
            entity: entity.into(),
            parameter,
            reason: reason.into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_invalid_topology() {
        let e = KernelError::InvalidTopology("open shell".into());
        assert!(e.to_string().contains("open shell"));
    }

    #[test]
    fn display_numerical_failure() {
        let e = KernelError::numerical("Newton", "did not converge after 100 iterations");
        assert!(e.to_string().contains("Newton"));
        assert!(e.to_string().contains("100 iterations"));
    }

    #[test]
    fn display_geom_eval() {
        let e = KernelError::geom_eval("BsplineCurve", 1.5, "parameter out of range");
        assert!(e.to_string().contains("BsplineCurve"));
        assert!(e.to_string().contains("1.5"));
    }

    #[test]
    fn display_invalid_entity_id() {
        let e = KernelError::InvalidEntityId;
        assert!(!e.to_string().is_empty());
    }

    #[test]
    fn from_io_error() {
        let io = std::io::Error::new(std::io::ErrorKind::NotFound, "file missing");
        let e: KernelError = io.into();
        assert!(e.to_string().contains("file missing"));
    }

    #[test]
    fn debug_format_is_not_empty() {
        let e = KernelError::InvalidEntityId;
        assert!(!format!("{:?}", e).is_empty());
    }
}
