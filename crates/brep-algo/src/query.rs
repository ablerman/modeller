//! Geometric queries on [`ShapeStore`]s: point-in-solid, closest face, etc.

use brep_core::{KernelError, Point3, Vec3};
use brep_mesh::{tessellate_face, TessellationOptions};
use brep_topo::entity::{FaceId, SolidId};
use brep_topo::store::ShapeStore;

use crate::bvh::{Bvh, Ray};

// ── Point-in-solid classification ─────────────────────────────────────────────

/// Whether a point lies inside, on the boundary, or outside a solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointLocation {
    Inside,
    OnBoundary,
    Outside,
}

/// Classify `point` relative to `solid_id` using a ray-casting test.
///
/// Fires a ray in a fixed direction (+X by default) and counts how many face
/// AABBs it hits.  For each candidate face it tests whether the ray passes
/// through the face's vertex polygon (planar approximation).  An odd crossing
/// count means inside.
///
/// The direction is perturbed slightly to avoid degenerate edge/vertex hits.
pub fn point_in_solid(
    store: &ShapeStore,
    bvh: &Bvh,
    solid_id: SolidId,
    point: &Point3,
    tol: f64,
) -> Result<PointLocation, KernelError> {
    let solid = store.solid(solid_id)?;
    let shell = store.shell(solid.outer_shell)?;

    // Slightly off-axis direction to avoid grazing edges.
    let dir = Vec3::new(1.0, 1e-4, 2e-4).normalize();
    let ray = Ray::new(*point, dir);

    let candidate_faces = bvh.intersect_ray(&ray);
    let mut crossings = 0usize;

    let tess_opts = TessellationOptions { chord_tolerance: 0.05, min_segments: 8 };

    for fid in candidate_faces {
        // Only count faces belonging to this solid's shell.
        if !shell.faces.contains(&fid) { continue; }

        let face = store.face(fid)?;

        // For periodic (curved) surfaces, use tessellation for accurate crossing tests.
        if face.surface.surface.is_u_periodic() {
            if let Ok(fm) = tessellate_face(store, fid, &tess_opts) {
                for tri in &fm.triangles {
                    let pts = &tri.positions;
                    if ray_polygon_crosses(&ray, pts, &tri.normal) {
                        crossings += 1;
                        break; // count at most once per face
                    }
                }
            }
            continue;
        }

        let verts = store.face_vertices(fid)?;
        if verts.len() < 3 { continue; }

        let positions: Vec<Point3> = verts.iter()
            .map(|&vid| store.vertex(vid).map(|v| v.position))
            .collect::<Result<_, _>>()?;

        // Check if the point is on this face (on-boundary test):
        // must be on the face plane AND within the face polygon.
        let face_normal = face_plane_normal(&positions);
        let d = face_normal.dot(&(point - positions[0]));
        if d.abs() < tol {
            // Project point onto face plane and test containment.
            let projected = *point - face_normal * d;
            if point_in_polygon(&projected, &positions, &face_normal) {
                return Ok(PointLocation::OnBoundary);
            }
        }

        // Ray–triangle fan crossing test.
        if ray_polygon_crosses(&ray, &positions, &face_normal) {
            crossings += 1;
        }
    }

    Ok(if crossings % 2 == 1 { PointLocation::Inside } else { PointLocation::Outside })
}

/// Compute a face polygon normal (Newell method — robust for non-planar quads).
fn face_plane_normal(pts: &[Point3]) -> Vec3 {
    let n = pts.len();
    let mut nx = 0.0f64;
    let mut ny = 0.0f64;
    let mut nz = 0.0f64;
    for i in 0..n {
        let a = &pts[i];
        let b = &pts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
    }
    let v = Vec3::new(nx, ny, nz);
    if v.norm() > 1e-14 { v.normalize() } else { Vec3::z() }
}

/// Test whether `ray` crosses a planar polygon (fan-tested triangles).
fn ray_polygon_crosses(ray: &Ray, pts: &[Point3], normal: &Vec3) -> bool {
    let n = pts.len();
    // Ray–plane intersection.
    let denom = ray.direction.dot(normal);
    if denom.abs() < 1e-12 { return false; }
    let t = (pts[0] - ray.origin).dot(normal) / denom;
    if t < 0.0 { return false; } // behind ray origin

    let hit = ray.at(t);

    // Fan of triangles from pts[0].
    for i in 1..n - 1 {
        if point_in_triangle(&hit, &pts[0], &pts[i], &pts[i + 1], normal) {
            return true;
        }
    }
    false
}

/// Test whether `p` lies inside a polygon (fan of triangles from pts[0]).
fn point_in_polygon(p: &Point3, pts: &[Point3], normal: &Vec3) -> bool {
    let n = pts.len();
    for i in 1..n - 1 {
        if point_in_triangle(p, &pts[0], &pts[i], &pts[i + 1], normal) {
            return true;
        }
    }
    false
}

/// Barycentric test: is `p` inside triangle (a, b, c)?
fn point_in_triangle(p: &Point3, a: &Point3, b: &Point3, c: &Point3, n: &Vec3) -> bool {
    let w0 = (b - a).cross(&(p - a)).dot(n);
    let w1 = (c - b).cross(&(p - b)).dot(n);
    let w2 = (a - c).cross(&(p - c)).dot(n);
    (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0)
}

// ── Closest face ──────────────────────────────────────────────────────────────

/// Return the face of `store` whose vertex centroid is closest to `point`,
/// and the squared distance.
pub fn closest_face(
    store: &ShapeStore,
    bvh: &Bvh,
    point: &Point3,
) -> Result<Option<(FaceId, f64)>, KernelError> {
    let Some(fid) = bvh.nearest_face(point) else { return Ok(None) };
    let verts = store.face_vertices(fid)?;
    let n = verts.len() as f64;
    if n == 0.0 { return Ok(None); }
    let cx = verts.iter().map(|&v| store.vertex(v).unwrap().position.x).sum::<f64>() / n;
    let cy = verts.iter().map(|&v| store.vertex(v).unwrap().position.y).sum::<f64>() / n;
    let cz = verts.iter().map(|&v| store.vertex(v).unwrap().position.z).sum::<f64>() / n;
    let centroid = Point3::new(cx, cy, cz);
    let d2 = (centroid - point).norm_squared();
    Ok(Some((fid, d2)))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::{primitives::make_box, store::ShapeStore};

    fn box_store() -> (ShapeStore, SolidId) {
        let mut store = ShapeStore::new();
        let sid = make_box(&mut store, 2.0, 2.0, 2.0).unwrap();
        (store, sid)
    }

    #[test]
    fn point_inside_box() {
        let (store, solid_id) = box_store();
        let bvh = Bvh::build(&store);
        let loc = point_in_solid(&store, &bvh, solid_id, &Point3::new(1.0, 1.0, 1.0), 1e-7).unwrap();
        assert_eq!(loc, PointLocation::Inside);
    }

    #[test]
    fn point_outside_box() {
        let (store, solid_id) = box_store();
        let bvh = Bvh::build(&store);
        let loc = point_in_solid(&store, &bvh, solid_id, &Point3::new(5.0, 1.0, 1.0), 1e-7).unwrap();
        assert_eq!(loc, PointLocation::Outside);
    }

    #[test]
    fn closest_face_returns_some() {
        let (store, _) = box_store();
        let bvh = Bvh::build(&store);
        let result = closest_face(&store, &bvh, &Point3::new(1.0, 1.0, 3.0)).unwrap();
        assert!(result.is_some());
    }
}
