//! Arc geometry helpers — pure 2D, no 3D or rendering dependencies.

/// Project a 2D center point onto the perpendicular bisector of (`s`, `e`).
///
/// The signed distance from the chord midpoint along the bisector is preserved,
/// so the caller's intent (which side to bow toward) is maintained.  If `s` and
/// `e` are coincident the center is returned unchanged.
pub fn project_center_to_arc_bisector(
    s: (f64, f64),
    e: (f64, f64),
    c: (f64, f64),
) -> (f64, f64) {
    let mx = (s.0 + e.0) * 0.5;
    let my = (s.1 + e.1) * 0.5;
    let chord_dx = e.0 - s.0;
    let chord_dy = e.1 - s.1;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt();
    if chord_len < 1e-10 {
        return c;
    }
    let px = -chord_dy / chord_len;
    let py =  chord_dx / chord_len;
    let t = (c.0 - mx) * px + (c.1 - my) * py;
    (mx + t * px, my + t * py)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projects_onto_bisector() {
        // Horizontal chord from (0,0) to (2,0). Bisector is the vertical x=1.
        // A center at (0.5, 3.0) should project to (1.0, 3.0).
        let result = project_center_to_arc_bisector((0.0, 0.0), (2.0, 0.0), (0.5, 3.0));
        assert!((result.0 - 1.0).abs() < 1e-10, "u={}", result.0);
        assert!((result.1 - 3.0).abs() < 1e-10, "v={}", result.1);
    }

    #[test]
    fn coincident_endpoints_returns_center_unchanged() {
        let result = project_center_to_arc_bisector((1.0, 1.0), (1.0, 1.0), (5.0, 5.0));
        assert_eq!(result, (5.0, 5.0));
    }
}
