//! Bounding Volume Hierarchy (BVH) over the faces of a [`ShapeStore`].
//!
//! # Construction
//! Built with a **Surface Area Heuristic (SAH)** top-down split.  At each
//! node the longest axis is tested with a fixed number of candidate split
//! planes; the split minimising `cost = N_L * SA_L + N_R * SA_R` is chosen.
//! Leaves hold up to [`MAX_LEAF`] face IDs.
//!
//! # Queries
//! - [`Bvh::intersect_ray`] — find all faces whose AABB a ray hits.
//! - [`Bvh::intersect_aabb`] — find all faces overlapping a query AABB.
//! - [`Bvh::nearest_face`] — face whose centroid is closest to a point.

use brep_core::{Aabb, Point3, Vec3};
use brep_mesh::{tessellate_face, TessellationOptions};
use brep_topo::entity::FaceId;
use brep_topo::store::ShapeStore;

/// Maximum face count in a BVH leaf node before splitting.
const MAX_LEAF: usize = 4;
/// Number of SAH candidate split planes per axis.
const SAH_BUCKETS: usize = 12;

// ── Public types ──────────────────────────────────────────────────────────────

/// A ray with origin and (non-normalised) direction.
#[derive(Clone, Debug)]
pub struct Ray {
    pub origin: Point3,
    pub direction: Vec3,
}

impl Ray {
    pub fn new(origin: Point3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// Evaluate `origin + t * direction`.
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.direction * t
    }
}

/// A built BVH over the faces of a single [`ShapeStore`].
pub struct Bvh {
    nodes: Vec<BvhNode>,
    /// Face IDs stored in leaf order (indices into this vec are stored in leaves).
    face_ids: Vec<FaceId>,
}

impl Bvh {
    /// Build a BVH from all faces in `store`.
    pub fn build(store: &ShapeStore) -> Self {
        let mut items: Vec<(FaceId, Aabb)> = store
            .face_ids()
            .filter_map(|fid| {
                let face = store.face(fid).ok()?;
                let aabb = face.surface.surface.bounding_box();
                if aabb.is_empty() {
                    // Curved/infinite-parameter surfaces return empty AABBs.
                    // For periodic (curved) faces tessellate to get tight bounds;
                    // for planar faces fall back to vertex positions.
                    let a = if face.surface.surface.is_u_periodic() {
                        let opts = TessellationOptions { chord_tolerance: 0.05, min_segments: 8 };
                        if let Ok(fm) = tessellate_face(store, fid, &opts) {
                            let mut a = Aabb::empty();
                            for tri in &fm.triangles {
                                for p in &tri.positions { a.include_point(p); }
                            }
                            a
                        } else {
                            Aabb::empty()
                        }
                    } else {
                        let verts = store.face_vertices(fid).ok()?;
                        let mut a = Aabb::empty();
                        for vid in verts {
                            if let Ok(v) = store.vertex(vid) {
                                a.include_point(&v.position);
                            }
                        }
                        a
                    };
                    Some((fid, a))
                } else {
                    Some((fid, aabb))
                }
            })
            .collect();

        let mut face_ids: Vec<FaceId> = Vec::new();
        let mut nodes: Vec<BvhNode> = Vec::new();
        build_recursive(&mut items, &mut face_ids, &mut nodes);
        Bvh { nodes, face_ids }
    }

    /// All faces whose AABB overlaps `query`.
    pub fn intersect_aabb(&self, query: &Aabb) -> Vec<FaceId> {
        let mut result = Vec::new();
        if self.nodes.is_empty() { return result; }
        self.aabb_recursive(0, query, &mut result);
        result
    }

    /// All faces whose AABB a ray hits (slab test).  Returns face IDs in
    /// no particular order.
    pub fn intersect_ray(&self, ray: &Ray) -> Vec<FaceId> {
        let mut result = Vec::new();
        if self.nodes.is_empty() { return result; }
        let inv_dir = Vec3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );
        self.ray_recursive(0, ray, &inv_dir, &mut result);
        result
    }

    /// Face whose AABB centroid is closest to `pt`.
    pub fn nearest_face(&self, pt: &Point3) -> Option<FaceId> {
        if self.face_ids.is_empty() { return None; }
        let mut best_fid = self.face_ids[0];
        let mut best_d2 = f64::MAX;
        self.nearest_recursive(0, pt, &mut best_fid, &mut best_d2);
        Some(best_fid)
    }

    /// Total number of faces indexed.
    pub fn face_count(&self) -> usize {
        self.face_ids.len()
    }

    // ── Private traversal helpers ─────────────────────────────────────────────

    fn aabb_recursive(&self, node_idx: usize, query: &Aabb, out: &mut Vec<FaceId>) {
        let node = &self.nodes[node_idx];
        if !node.bounds.intersects(query) { return; }
        match &node.kind {
            NodeKind::Leaf { start, count } => {
                out.extend_from_slice(&self.face_ids[*start..*start + *count]);
            }
            NodeKind::Inner { left, right } => {
                self.aabb_recursive(*left, query, out);
                self.aabb_recursive(*right, query, out);
            }
        }
    }

    fn ray_recursive(&self, node_idx: usize, ray: &Ray, inv_dir: &Vec3, out: &mut Vec<FaceId>) {
        let node = &self.nodes[node_idx];
        if !ray_aabb(&node.bounds, ray, inv_dir) { return; }
        match &node.kind {
            NodeKind::Leaf { start, count } => {
                out.extend_from_slice(&self.face_ids[*start..*start + *count]);
            }
            NodeKind::Inner { left, right } => {
                self.ray_recursive(*left, ray, inv_dir, out);
                self.ray_recursive(*right, ray, inv_dir, out);
            }
        }
    }

    fn nearest_recursive(&self, node_idx: usize, pt: &Point3, best_fid: &mut FaceId, best_d2: &mut f64) {
        let node = &self.nodes[node_idx];
        // Prune: closest possible point in this node's AABB.
        if aabb_min_dist2(&node.bounds, pt) >= *best_d2 { return; }
        match &node.kind {
            NodeKind::Leaf { start, count } => {
                for &fid in &self.face_ids[*start..*start + *count] {
                    // Use AABB centroid as proxy.
                    let c = node.bounds.center();
                    let d2 = (c - pt).norm_squared();
                    if d2 < *best_d2 { *best_d2 = d2; *best_fid = fid; }
                }
            }
            NodeKind::Inner { left, right } => {
                self.nearest_recursive(*left, pt, best_fid, best_d2);
                self.nearest_recursive(*right, pt, best_fid, best_d2);
            }
        }
    }
}

// ── Internal node representation ──────────────────────────────────────────────

struct BvhNode {
    bounds: Aabb,
    kind: NodeKind,
}

enum NodeKind {
    Leaf { start: usize, count: usize },
    Inner { left: usize, right: usize },
}

// ── SAH builder ───────────────────────────────────────────────────────────────

fn build_recursive(
    items: &mut [(FaceId, Aabb)],
    face_ids: &mut Vec<FaceId>,
    nodes: &mut Vec<BvhNode>,
) -> usize {
    // Compute the enclosing AABB for all items.
    let mut bounds = Aabb::empty();
    for (_, aabb) in items.iter() {
        bounds = bounds.union(aabb);
    }

    let node_idx = nodes.len();
    nodes.push(BvhNode { bounds: bounds.clone(), kind: NodeKind::Leaf { start: 0, count: 0 } });

    if items.len() <= MAX_LEAF {
        let start = face_ids.len();
        for (fid, _) in items.iter() { face_ids.push(*fid); }
        nodes[node_idx].kind = NodeKind::Leaf { start, count: items.len() };
        return node_idx;
    }

    // SAH split: try each axis, pick best.
    let centroid_bounds = centroid_aabb(items);
    let best = best_sah_split(items, &centroid_bounds);

    match best {
        None => {
            // Degenerate: all centroids coincide — just make a leaf.
            let start = face_ids.len();
            for (fid, _) in items.iter() { face_ids.push(*fid); }
            nodes[node_idx].kind = NodeKind::Leaf { start, count: items.len() };
        }
        Some((axis, split_val)) => {
            // Partition items by centroid along `axis`.
            let mid = partition(items, axis, split_val);
            let (left_items, right_items) = items.split_at_mut(mid);
            let left_idx  = build_recursive(left_items,  face_ids, nodes);
            let right_idx = build_recursive(right_items, face_ids, nodes);
            nodes[node_idx].kind = NodeKind::Inner { left: left_idx, right: right_idx };
        }
    }
    node_idx
}

fn centroid_aabb(items: &[(FaceId, Aabb)]) -> Aabb {
    let mut a = Aabb::empty();
    for (_, aabb) in items {
        a.include_point(&aabb.center());
    }
    a
}

/// Returns `Some((axis, split_value))` for the best SAH split, or `None` if
/// all centroids are coincident.
fn best_sah_split(items: &[(FaceId, Aabb)], centroid_bounds: &Aabb) -> Option<(usize, f64)> {
    let parent_sa = surface_area(centroid_bounds);
    if parent_sa < 1e-20 { return None; }

    let mut best_cost = f64::MAX;
    let mut best_axis = 0usize;
    let mut best_split = 0.0f64;

    let extent = centroid_bounds.max - centroid_bounds.min;

    for axis in 0..3 {
        let axis_len = extent[axis];
        if axis_len < 1e-20 { continue; }

        for b in 1..SAH_BUCKETS {
            let split = centroid_bounds.min[axis] + axis_len * b as f64 / SAH_BUCKETS as f64;
            let mut left_bounds  = Aabb::empty();
            let mut right_bounds = Aabb::empty();
            let mut n_left  = 0usize;
            let mut n_right = 0usize;
            for (_, aabb) in items {
                if aabb.center()[axis] <= split {
                    left_bounds  = left_bounds.union(aabb);
                    n_left  += 1;
                } else {
                    right_bounds = right_bounds.union(aabb);
                    n_right += 1;
                }
            }
            if n_left == 0 || n_right == 0 { continue; }
            let cost = n_left  as f64 * surface_area(&left_bounds)
                     + n_right as f64 * surface_area(&right_bounds);
            if cost < best_cost {
                best_cost  = cost;
                best_axis  = axis;
                best_split = split;
            }
        }
    }

    if best_cost == f64::MAX { None } else { Some((best_axis, best_split)) }
}

/// Partition `items` in-place: items with centroid ≤ `split_val` on `axis`
/// come first.  Returns the split index (first index in right partition).
fn partition(items: &mut [(FaceId, Aabb)], axis: usize, split_val: f64) -> usize {
    let mut left = 0;
    let mut right = items.len();
    while left < right {
        if items[left].1.center()[axis] <= split_val {
            left += 1;
        } else {
            right -= 1;
            items.swap(left, right);
        }
    }
    left.max(1).min(items.len() - 1)
}

fn surface_area(aabb: &Aabb) -> f64 {
    if aabb.is_empty() { return 0.0; }
    let e = aabb.max - aabb.min;
    2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
}

// ── Ray–AABB slab test ────────────────────────────────────────────────────────

fn ray_aabb(aabb: &Aabb, ray: &Ray, inv_dir: &Vec3) -> bool {
    if aabb.is_empty() { return false; }
    let tx1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
    let tx2 = (aabb.max.x - ray.origin.x) * inv_dir.x;
    let ty1 = (aabb.min.y - ray.origin.y) * inv_dir.y;
    let ty2 = (aabb.max.y - ray.origin.y) * inv_dir.y;
    let tz1 = (aabb.min.z - ray.origin.z) * inv_dir.z;
    let tz2 = (aabb.max.z - ray.origin.z) * inv_dir.z;

    let tmin = tx1.min(tx2).max(ty1.min(ty2)).max(tz1.min(tz2));
    let tmax = tx1.max(tx2).min(ty1.max(ty2)).min(tz1.max(tz2));
    tmax >= tmin.max(0.0)
}

fn aabb_min_dist2(aabb: &Aabb, pt: &Point3) -> f64 {
    if aabb.is_empty() { return f64::MAX; }
    let dx = (aabb.min.x - pt.x).max(0.0).max(pt.x - aabb.max.x);
    let dy = (aabb.min.y - pt.y).max(0.0).max(pt.y - aabb.max.y);
    let dz = (aabb.min.z - pt.z).max(0.0).max(pt.z - aabb.max.z);
    dx * dx + dy * dy + dz * dz
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::{primitives::{make_box, make_sphere}, store::ShapeStore};

    #[test]
    fn bvh_build_box() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let bvh = Bvh::build(&store);
        assert_eq!(bvh.face_count(), 6);
    }

    #[test]
    fn bvh_build_sphere() {
        let mut store = ShapeStore::new();
        make_sphere(&mut store, 1.0).unwrap();
        let bvh = Bvh::build(&store);
        assert_eq!(bvh.face_count(), 1);
    }

    #[test]
    fn ray_hits_box() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 2.0, 2.0, 2.0).unwrap();
        let bvh = Bvh::build(&store);
        // Ray along +X through the middle of the box.
        let ray = Ray::new(Point3::new(-5.0, 1.0, 1.0), Vec3::x());
        let hits = bvh.intersect_ray(&ray);
        assert!(!hits.is_empty(), "ray along X should hit the box");
    }

    #[test]
    fn ray_misses_box() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let bvh = Bvh::build(&store);
        // Ray passing far above the box.
        let ray = Ray::new(Point3::new(-5.0, 10.0, 10.0), Vec3::x());
        let hits = bvh.intersect_ray(&ray);
        assert!(hits.is_empty(), "ray far above box should miss");
    }

    #[test]
    fn aabb_query_overlapping() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 2.0, 2.0, 2.0).unwrap();
        let bvh = Bvh::build(&store);
        let query = Aabb::from_corners(Point3::new(-0.5,-0.5,-0.5), Point3::new(0.5,0.5,0.5));
        let hits = bvh.intersect_aabb(&query);
        assert!(!hits.is_empty());
    }

    #[test]
    fn aabb_query_outside() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let bvh = Bvh::build(&store);
        let query = Aabb::from_corners(Point3::new(5.0,5.0,5.0), Point3::new(6.0,6.0,6.0));
        let hits = bvh.intersect_aabb(&query);
        assert!(hits.is_empty());
    }

    #[test]
    fn nearest_face_returns_some() {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let bvh = Bvh::build(&store);
        let fid = bvh.nearest_face(&Point3::new(0.5, 0.5, 2.0));
        assert!(fid.is_some());
    }

    #[test]
    fn empty_store_bvh() {
        let store = ShapeStore::new();
        let bvh = Bvh::build(&store);
        assert_eq!(bvh.face_count(), 0);
        let ray = Ray::new(Point3::origin(), Vec3::x());
        assert!(bvh.intersect_ray(&ray).is_empty());
        assert!(bvh.nearest_face(&Point3::origin()).is_none());
    }
}
