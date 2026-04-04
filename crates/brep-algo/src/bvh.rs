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

use std::collections::HashMap;

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

/// Precomputed tessellation for a single curved face: triangle positions + normal.
pub type FaceTri = ([Point3; 3], Vec3);

// ── Per-face triangle BVH ─────────────────────────────────────────────────────

/// A small BVH over a face's tessellated triangles for O(log N) ray crossing
/// tests inside `point_in_solid`.
pub struct TriMesh {
    tris:  Vec<FaceTri>,
    nodes: Vec<TriNode>,
}

struct TriNode {
    bounds: Aabb,
    /// Positive = leaf (start index into `tris`), negative = inner left child index.
    /// We encode leaf as `(start << 1) | 1` and inner as `(left << 1) | 0`.
    left_or_start: u32,
    right_or_count: u32,
    is_leaf: bool,
}

const TRI_LEAF: usize = 4;

impl TriMesh {
    pub fn build(tris: Vec<FaceTri>) -> Self {
        if tris.is_empty() {
            return Self { tris, nodes: Vec::new() };
        }
        // Build (tri_idx, aabb) pairs.
        let mut items: Vec<(usize, Aabb)> = tris.iter().enumerate().map(|(i, &(pos, _))| {
            let mut a = Aabb::empty();
            for p in &pos { a.include_point(p); }
            (i, a)
        }).collect();

        let mut nodes: Vec<TriNode> = Vec::new();
        let mut order: Vec<usize> = Vec::new();
        tri_build_recursive(&mut items, &mut order, &mut nodes);

        // Reorder tris to match leaf order.
        let sorted: Vec<FaceTri> = order.iter().map(|&i| tris[i]).collect();
        Self { tris: sorted, nodes }
    }

    /// Returns `true` if the ray crosses at least one triangle (front or back).
    pub fn ray_crosses(&self, ray: &Ray) -> bool {
        if self.nodes.is_empty() { return false; }
        let inv = Vec3::new(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);
        self.tri_ray_recursive(0, ray, &inv)
    }

    fn tri_ray_recursive(&self, idx: usize, ray: &Ray, inv: &Vec3) -> bool {
        let node = &self.nodes[idx];
        if !ray_aabb(&node.bounds, ray, inv) { return false; }
        if node.is_leaf {
            let start = node.left_or_start as usize;
            let count = node.right_or_count as usize;
            for &(positions, normal) in &self.tris[start..start + count] {
                if crate::query::ray_polygon_crosses_pub(ray, &positions, &normal) {
                    return true;
                }
            }
            false
        } else {
            self.tri_ray_recursive(node.left_or_start as usize, ray, inv)
                || self.tri_ray_recursive(node.right_or_count as usize, ray, inv)
        }
    }
}

fn tri_build_recursive(
    items: &mut [(usize, Aabb)],
    order: &mut Vec<usize>,
    nodes: &mut Vec<TriNode>,
) -> usize {
    let mut bounds = Aabb::empty();
    for (_, a) in items.iter() { bounds = bounds.union(a); }
    let node_idx = nodes.len();

    if items.len() <= TRI_LEAF {
        let start = order.len();
        for (i, _) in items.iter() { order.push(*i); }
        nodes.push(TriNode { bounds, left_or_start: start as u32, right_or_count: items.len() as u32, is_leaf: true });
        return node_idx;
    }

    // Split on longest axis at median.
    let ext = bounds.max - bounds.min;
    let axis = if ext.x >= ext.y && ext.x >= ext.z { 0 } else if ext.y >= ext.z { 1 } else { 2 };
    items.sort_unstable_by(|(_, a), (_, b)| a.center()[axis].partial_cmp(&b.center()[axis]).unwrap());
    let mid = items.len() / 2;
    let (left_items, right_items) = items.split_at_mut(mid);

    // Reserve placeholder.
    nodes.push(TriNode { bounds, left_or_start: 0, right_or_count: 0, is_leaf: false });
    let left  = tri_build_recursive(left_items,  order, nodes);
    let right = tri_build_recursive(right_items, order, nodes);
    nodes[node_idx].left_or_start  = left  as u32;
    nodes[node_idx].right_or_count = right as u32;
    node_idx
}

/// A built BVH over the faces of a single [`ShapeStore`].
pub struct Bvh {
    nodes: Vec<BvhNode>,
    /// Face IDs stored in leaf order (indices into this vec are stored in leaves).
    face_ids: Vec<FaceId>,
    /// Per-face triangle BVH for periodic (curved) faces.
    /// Populated once at build time; reused by `point_in_solid`.
    face_meshes: HashMap<FaceId, TriMesh>,
}

impl Bvh {
    /// Build a BVH from all faces in `store`.
    pub fn build(store: &ShapeStore) -> Self {
        // Fine tessellation: tight AABBs + accurate pis crossing tests.
        // A per-face triangle BVH (TriMesh) keeps pis O(log N) per face.
        let opts = TessellationOptions { chord_tolerance: 0.05, min_segments: 8 };
        let mut face_meshes: HashMap<FaceId, TriMesh> = HashMap::new();

        let mut items: Vec<(FaceId, Aabb)> = store
            .face_ids()
            .filter_map(|fid| {
                let face = store.face(fid).ok()?;
                // Always tessellate periodic (curved) faces so their triangle BVH
                // is cached for use by point_in_solid.
                if face.surface.surface.is_u_periodic() {
                    if let Ok(fm) = tessellate_face(store, fid, &opts) {
                        let mut aabb = Aabb::empty();
                        let tris: Vec<FaceTri> = fm.triangles.iter().map(|t| {
                            for p in &t.positions { aabb.include_point(p); }
                            (t.positions, t.normal)
                        }).collect();
                        face_meshes.insert(fid, TriMesh::build(tris));
                        if aabb.is_empty() { return None; }
                        return Some((fid, aabb));
                    }
                    return None;
                }
                // Non-periodic face: use analytical bounds if available, else vertices.
                let aabb = face.surface.surface.bounding_box();
                let a = if !aabb.is_empty() {
                    aabb
                } else {
                    let verts = store.face_vertices(fid).ok()?;
                    let mut a = Aabb::empty();
                    for vid in verts {
                        if let Ok(v) = store.vertex(vid) { a.include_point(&v.position); }
                    }
                    a
                };
                if a.is_empty() { None } else { Some((fid, a)) }
            })
            .collect();

        let mut face_ids: Vec<FaceId> = Vec::new();
        let mut nodes: Vec<BvhNode> = Vec::new();
        build_recursive(&mut items, &mut face_ids, &mut nodes);
        Bvh { nodes, face_ids, face_meshes }
    }

    /// Triangle BVH for a periodic face — `None` for planar faces.
    pub fn face_mesh(&self, fid: FaceId) -> Option<&TriMesh> {
        self.face_meshes.get(&fid)
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
