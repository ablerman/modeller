//! `brep-mesh` — adaptive tessellation of BRep shapes.
//!
//! Converts each face of a [`ShapeStore`] into a triangle mesh.
//!
//! # Algorithm
//!
//! - **Planar faces** (Plane surface): walk the outer loop to collect the boundary
//!   polygon, then fan-triangulate from vertex 0.
//! - **Periodic / curved faces** (Cylinder, Sphere, Cone, …): sample a `N_u × N_v`
//!   UV grid.  `N_u` is derived from the chord tolerance and the u-direction
//!   curvature radius; `N_v` is derived from the v extent and chord tolerance.

use std::f64::consts::TAU;

use brep_core::{KernelError, Point3, Vec3};
use brep_topo::entity::{FaceId, Orientation};
use brep_topo::store::ShapeStore;

// ── Public types ──────────────────────────────────────────────────────────────

/// Options controlling the quality / density of the tessellation.
#[derive(Clone, Debug)]
pub struct TessellationOptions {
    /// Maximum deviation of a chord from the true curve/surface (world units).
    pub chord_tolerance: f64,
    /// Minimum number of segments per full revolution.
    pub min_segments: u32,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self { chord_tolerance: 0.01, min_segments: 8 }
    }
}

/// A single oriented triangle.
#[derive(Clone, Debug)]
pub struct Triangle {
    /// The three corner positions in CCW order (viewed from the outside of the solid).
    pub positions: [Point3; 3],
    /// Outward-facing unit normal (computed at the triangle centroid).
    pub normal: Vec3,
}

/// Tessellation of a single BRep face.
#[derive(Clone, Debug)]
pub struct FaceMesh {
    pub face_id: FaceId,
    pub triangles: Vec<Triangle>,
}

/// Tessellate every face in `store` and return one [`FaceMesh`] per face.
pub fn tessellate(
    store: &ShapeStore,
    opts: &TessellationOptions,
) -> Result<Vec<FaceMesh>, KernelError> {
    let face_ids: Vec<FaceId> = store.face_ids().collect();
    let mut meshes = Vec::with_capacity(face_ids.len());
    for fid in face_ids {
        meshes.push(tessellate_face(store, fid, opts)?);
    }
    Ok(meshes)
}

// ── Face tessellation dispatch ────────────────────────────────────────────────

fn tessellate_face(
    store: &ShapeStore,
    face_id: FaceId,
    opts: &TessellationOptions,
) -> Result<FaceMesh, KernelError> {
    let face = store.face(face_id)?;
    let surface = &*face.surface.surface;
    let flip = !face.surface.same_sense ^ matches!(face.orientation, Orientation::Reversed);

    if surface.is_u_periodic() {
        tessellate_periodic(store, face_id, surface, opts, flip)
    } else {
        tessellate_planar(store, face_id, surface, opts, flip)
    }
}

// ── Planar face: fan-triangulate the boundary polygon ─────────────────────────

fn tessellate_planar(
    store: &ShapeStore,
    face_id: FaceId,
    surface: &dyn brep_geom::traits::Surface,
    opts: &TessellationOptions,
    flip: bool,
) -> Result<FaceMesh, KernelError> {
    let face = store.face(face_id)?;

    // Collect boundary half-edges and their origin positions.
    let mut he_ids = Vec::new();
    let mut positions: Vec<Point3> = Vec::new();
    for he_result in store.loop_half_edges(face.outer_loop)? {
        let he_id = he_result?;
        he_ids.push(he_id);
        let v = store.half_edge(he_id)?.origin;
        positions.push(store.vertex(v)?.position);
    }

    // When the boundary is a single circular edge (1 half-edge in the loop),
    // sample the circle and fan-triangulate from its centre.
    if positions.len() < 3 {
        if let Some(&he_id) = he_ids.first() {
            let edge_id = store.half_edge(he_id)?.edge;
            if let Some(cb) = &store.edge(edge_id)?.curve {
                let curve = &*cb.curve;
                let r = curve.derivative1(0.0).norm().max(1e-12);
                let n_segs = segments_for_chord(r, opts.chord_tolerance, opts.min_segments) as usize;
                let pts: Vec<Point3> = (0..n_segs)
                    .map(|i| curve.point(TAU * i as f64 / n_segs as f64))
                    .collect();
                let cx = pts.iter().map(|p| p.x).sum::<f64>() / n_segs as f64;
                let cy = pts.iter().map(|p| p.y).sum::<f64>() / n_segs as f64;
                let cz = pts.iter().map(|p| p.z).sum::<f64>() / n_segs as f64;
                let center = Point3::new(cx, cy, cz);
                let raw_n = (pts[1] - pts[0]).cross(&(pts[n_segs / 4] - pts[0]));
                let normal = if raw_n.norm() > 1e-14 {
                    if flip { -raw_n.normalize() } else { raw_n.normalize() }
                } else if let Ok((u, v)) = surface.closest_parameter(&center, 1e-7) {
                    let sn = surface.normal(u, v).unwrap_or(Vec3::z());
                    if flip { -sn } else { sn }
                } else {
                    Vec3::z()
                };
                let mut triangles = Vec::with_capacity(n_segs);
                for i in 0..n_segs {
                    let a = pts[i];
                    let b = pts[(i + 1) % n_segs];
                    let tri = if flip { [center, b, a] } else { [center, a, b] };
                    triangles.push(Triangle { positions: tri, normal });
                }
                return Ok(FaceMesh { face_id, triangles });
            }
        }
        return Ok(FaceMesh { face_id, triangles: vec![] });
    }

    // Compute polygon normal from first two edges.
    let e1 = positions[1] - positions[0];
    let e2 = positions[positions.len() - 1] - positions[0];
    let raw_n = e1.cross(&e2);
    let normal = if raw_n.norm() > 1e-14 {
        if flip { -raw_n.normalize() } else { raw_n.normalize() }
    } else {
        // Fall back to surface normal at the first vertex.
        if let Ok((u, v)) = surface.closest_parameter(&positions[0], 1e-7) {
            surface.normal(u, v).unwrap_or(Vec3::z())
        } else {
            Vec3::z()
        }
    };

    // Fan triangulation: (p0, p_i, p_{i+1}).
    let mut triangles = Vec::with_capacity(positions.len() - 2);
    for i in 1..positions.len() - 1 {
        let tri = if flip {
            [positions[0], positions[i + 1], positions[i]]
        } else {
            [positions[0], positions[i], positions[i + 1]]
        };
        triangles.push(Triangle { positions: tri, normal });
    }
    Ok(FaceMesh { face_id, triangles })
}

// ── Periodic face: UV grid sampling ──────────────────────────────────────────

fn tessellate_periodic(
    store: &ShapeStore,
    face_id: FaceId,
    surface: &dyn brep_geom::traits::Surface,
    opts: &TessellationOptions,
    flip: bool,
) -> Result<FaceMesh, KernelError> {
    let face = store.face(face_id)?;

    // Collect boundary vertex positions.
    let mut boundary: Vec<Point3> = Vec::new();
    for he_result in store.loop_half_edges(face.outer_loop)? {
        let he_id = he_result?;
        let v = store.half_edge(he_id)?.origin;
        boundary.push(store.vertex(v)?.position);
    }

    // Determine the v range by projecting boundary vertices onto the surface.
    let ((_, _), (v_range_min, v_range_max)) = surface.parameter_range();
    let (v_min, v_max) = if v_range_min.is_finite() && v_range_max.is_finite() {
        (v_range_min, v_range_max)
    } else {
        // Project boundary vertices to find the actual v extent.
        let tol = opts.chord_tolerance * 1e-3;
        let mut vmin = f64::MAX;
        let mut vmax = f64::MIN;
        for p in &boundary {
            if let Ok((_, vp)) = surface.closest_parameter(p, tol) {
                if vp < vmin { vmin = vp; }
                if vp > vmax { vmax = vp; }
            }
        }
        if vmin >= vmax {
            vmin = 0.0;
            vmax = 1.0;
        }
        (vmin, vmax)
    };

    // Adaptive N_u from chord tolerance and u-curvature radius.
    // `du(u, v_mid)` has magnitude = radius of curvature in u.
    let v_mid = (v_min + v_max) * 0.5;
    let r_u = surface.du(0.0, v_mid).norm().max(1e-12);
    let n_u = segments_for_chord(r_u, opts.chord_tolerance, opts.min_segments) as usize;

    // N_v: at least 1; scale by v extent relative to chord_tolerance, but cap to keep mesh reasonable.
    let v_extent = (v_max - v_min).abs();
    let r_v = surface.dv(0.0, v_mid).norm().max(1e-12);
    let n_v = if v_extent < 1e-10 {
        1
    } else {
        segments_for_chord(r_v * (v_extent / 1.0), opts.chord_tolerance, 1) as usize
    }.max(1);

    // Build the (n_u × n_v) grid.
    // Row i has u = u0 + (TAU * i / n_u), col j has v = v_min + (v_extent * j / n_v).
    // Because the surface is periodic in u, row n_u wraps back to row 0.
    let u_coords: Vec<f64> = (0..=n_u)
        .map(|i| TAU * i as f64 / n_u as f64)
        .collect();
    let v_coords: Vec<f64> = (0..=n_v)
        .map(|j| v_min + v_extent * j as f64 / n_v as f64)
        .collect();

    // Pre-compute all grid points.
    let grid: Vec<Vec<Point3>> = u_coords.iter()
        .map(|&u| v_coords.iter().map(|&v| surface.point(u, v)).collect())
        .collect();

    // Triangulate each quad (i,j) → (i+1,j) → (i+1,j+1) → (i,j+1).
    // For the periodic seam: row n_u should use row 0's data.
    let mut triangles = Vec::with_capacity(2 * n_u * n_v);
    for i in 0..n_u {
        let next_i = (i + 1) % (n_u + 1); // wraps: n_u → 0
        for j in 0..n_v {
            let p00 = grid[i][j];
            let p10 = grid[next_i][j];
            let p01 = grid[i][j + 1];
            let p11 = grid[next_i][j + 1];

            let u_c = u_coords[i] + TAU * 0.5 / n_u as f64;
            let v_c = (v_coords[j] + v_coords[j + 1]) * 0.5;
            let normal = surface.normal(u_c, v_c)
                .unwrap_or_else(|_| triangle_normal(p00, p10, p11));
            let normal = if flip { -normal } else { normal };

            let (t0, t1) = if flip {
                ([p00, p11, p10], [p00, p01, p11])
            } else {
                ([p00, p10, p11], [p00, p11, p01])
            };
            triangles.push(Triangle { positions: t0, normal });
            triangles.push(Triangle { positions: t1, normal });
        }
    }

    Ok(FaceMesh { face_id, triangles })
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Number of segments to stay within `chord_tol` for a full circle of radius `r`.
fn segments_for_chord(r: f64, chord_tol: f64, min_segs: u32) -> u32 {
    if r < 1e-12 || chord_tol >= 2.0 * r {
        return min_segs;
    }
    let ratio = chord_tol / (2.0 * r);
    // chord = 2r sin(π/N) ≤ chord_tol  →  N ≥ π / arcsin(chord_tol/(2r))
    let n = std::f64::consts::PI / ratio.min(1.0).asin();
    (n.ceil() as u32).max(min_segs)
}

fn triangle_normal(a: Point3, b: Point3, c: Point3) -> Vec3 {
    let n = (b - a).cross(&(c - a));
    if n.norm() > 1e-14 { n.normalize() } else { Vec3::z() }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::primitives::{make_box, make_cylinder, make_cone, make_sphere};
    use brep_topo::store::ShapeStore;

    fn default_opts() -> TessellationOptions {
        TessellationOptions { chord_tolerance: 0.05, min_segments: 8 }
    }

    #[test]
    fn tessellate_box_produces_12_triangles() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let meshes = tessellate(&store, &default_opts()).unwrap();
        let total: usize = meshes.iter().map(|m| m.triangles.len()).sum();
        assert_eq!(meshes.len(), 6, "box has 6 faces");
        assert_eq!(total, 12, "box has 12 triangles (2 per face)");
    }

    #[test]
    fn tessellate_box_all_normals_nonzero() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 2.0, 3.0, 4.0).unwrap();
        for m in tessellate(&store, &default_opts()).unwrap() {
            for t in &m.triangles {
                assert!(t.normal.norm() > 0.9, "degenerate normal: {:?}", t.normal);
            }
        }
    }

    #[test]
    fn tessellate_cylinder_produces_faces() {
        let mut store = ShapeStore::new();
        make_cylinder(&mut store, 1.0, 2.0).unwrap();
        let meshes = tessellate(&store, &default_opts()).unwrap();
        assert_eq!(meshes.len(), 3);
        let total: usize = meshes.iter().map(|m| m.triangles.len()).sum();
        assert!(total > 0, "cylinder should produce triangles");
    }

    #[test]
    fn tessellate_sphere_produces_faces() {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        let meshes = tessellate(&store, &default_opts()).unwrap();
        assert_eq!(meshes.len(), 1);
        assert!(!meshes[0].triangles.is_empty());
    }

    #[test]
    fn tessellate_cone_produces_faces() {
        let mut store = ShapeStore::new();
        make_cone(&mut store, 1.0, 2.0).unwrap();
        let meshes = tessellate(&store, &default_opts()).unwrap();
        assert_eq!(meshes.len(), 2);
        let total: usize = meshes.iter().map(|m| m.triangles.len()).sum();
        assert!(total > 0);
    }

    #[test]
    fn segments_for_chord_minimum_enforced() {
        // Even a very large tolerance should not go below min_segs.
        assert_eq!(segments_for_chord(1.0, 100.0, 6), 6);
    }

    #[test]
    fn segments_for_chord_tighter_tolerance_gives_more_segments() {
        let n_coarse = segments_for_chord(1.0, 0.1, 4);
        let n_fine   = segments_for_chord(1.0, 0.01, 4);
        assert!(n_fine > n_coarse);
    }
}
