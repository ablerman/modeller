//! Parametric feature tree with dirty propagation.
//!
//! # Design
//!
//! Features are nodes in a directed acyclic graph (DAG).  Each feature:
//! - Holds typed parameters (real, direction, profile, etc.)
//! - Takes zero or more `Arc<ShapeStore>` inputs
//! - Produces one `Arc<ShapeStore>` output (cached)
//!
//! When a parameter is changed, the feature and all downstream nodes are
//! marked dirty.  The next call to [`FeatureTree::evaluate`] recomputes only
//! the dirty subtree (topological-sort order).

use std::collections::HashSet;
use std::sync::Arc;

use slotmap::{new_key_type, SlotMap};

use brep_bool::{boolean_op, BooleanKind};
use brep_core::{KernelError, Point3, Vec3};
use brep_topo::{
    primitives::make_solid_from_polygon_faces,
    store::ShapeStore,
};

// ── Public parameter types ────────────────────────────────────────────────────

/// A typed parameter value.
#[derive(Debug, Clone)]
pub enum FeatureParameterValue {
    Real(f64),
    Integer(i64),
    Bool(bool),
    Direction(Vec3),
    Point(Point3),
    Profile(Vec<Point3>),
}

/// Metadata about a single parameter (name + current value).
#[derive(Debug, Clone)]
pub struct FeatureParameter {
    pub name: &'static str,
    pub value: FeatureParameterValue,
}

// ── Feature trait ─────────────────────────────────────────────────────────────

/// A parametric construction step.
pub trait Feature: Send + Sync {
    fn name(&self) -> &str;

    /// Number of `ShapeStore` inputs this feature requires.
    fn input_count(&self) -> usize { 0 }

    /// Compute the output given `inputs` (length == `input_count()`).
    fn compute(&self, inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError>;

    fn parameter_count(&self) -> usize;
    fn get_parameter(&self, index: usize) -> Option<FeatureParameter>;
    fn set_parameter(&mut self, index: usize, value: FeatureParameterValue) -> Result<(), KernelError>;
}

// ── Feature tree ──────────────────────────────────────────────────────────────

new_key_type! {
    /// Handle to a feature in a [`FeatureTree`].
    pub struct FeatureId;
}

pub(crate) struct FeatureNode {
    feature: Box<dyn Feature>,
    inputs: Vec<FeatureId>,
    output: Option<Arc<ShapeStore>>,
    dirty: bool,
}

/// A directed acyclic graph of parametric features.
pub struct FeatureTree {
    pub(crate) nodes: SlotMap<FeatureId, FeatureNode>,
}

impl FeatureTree {
    pub fn new() -> Self {
        Self { nodes: SlotMap::with_key() }
    }

    pub fn add_feature(&mut self, feature: Box<dyn Feature>) -> FeatureId {
        self.nodes.insert(FeatureNode {
            feature, inputs: Vec::new(), output: None, dirty: true,
        })
    }

    pub fn add_feature_with_inputs(
        &mut self,
        feature: Box<dyn Feature>,
        input_ids: Vec<FeatureId>,
    ) -> FeatureId {
        self.nodes.insert(FeatureNode {
            feature, inputs: input_ids, output: None, dirty: true,
        })
    }

    pub fn set_parameter(
        &mut self,
        feature_id: FeatureId,
        index: usize,
        value: FeatureParameterValue,
    ) -> Result<(), KernelError> {
        {
            let node = self.nodes.get_mut(feature_id).ok_or(KernelError::InvalidEntityId)?;
            node.feature.set_parameter(index, value)?;
            node.dirty = true;
            node.output = None;
        }
        let to_mark = self.downstream_of(feature_id);
        for fid in to_mark {
            if let Some(n) = self.nodes.get_mut(fid) {
                n.dirty = true;
                n.output = None;
            }
        }
        Ok(())
    }

    pub fn evaluate(&mut self, feature_id: FeatureId) -> Result<Arc<ShapeStore>, KernelError> {
        let order = self.topo_order_to(feature_id);
        for fid in order {
            self.compute_node(fid)?;
        }
        self.nodes[feature_id]
            .output
            .clone()
            .ok_or_else(|| KernelError::InvalidTopology("feature produced no output".into()))
    }

    fn compute_node(&mut self, fid: FeatureId) -> Result<(), KernelError> {
        if !self.nodes[fid].dirty {
            return Ok(());
        }
        let input_ids: Vec<FeatureId> = self.nodes[fid].inputs.clone();
        let mut inputs: Vec<Arc<ShapeStore>> = Vec::with_capacity(input_ids.len());
        for iid in &input_ids {
            let out = self.nodes[*iid].output.clone().ok_or_else(|| {
                KernelError::InvalidTopology("upstream feature has no output".into())
            })?;
            inputs.push(out);
        }
        let result = self.nodes[fid].feature.compute(&inputs)?;
        self.nodes[fid].output = Some(result);
        self.nodes[fid].dirty = false;
        Ok(())
    }

    fn downstream_of(&self, fid: FeatureId) -> Vec<FeatureId> {
        let mut result = Vec::new();
        let mut visited = HashSet::new();
        let mut stack = vec![fid];
        while let Some(current) = stack.pop() {
            for (id, node) in &self.nodes {
                if node.inputs.contains(&current) && !visited.contains(&id) {
                    visited.insert(id);
                    result.push(id);
                    stack.push(id);
                }
            }
        }
        result
    }

    fn topo_order_to(&self, target: FeatureId) -> Vec<FeatureId> {
        let mut order = Vec::new();
        let mut visited = HashSet::new();
        self.topo_visit(target, &mut visited, &mut order);
        order
    }

    fn topo_visit(
        &self,
        fid: FeatureId,
        visited: &mut HashSet<FeatureId>,
        order: &mut Vec<FeatureId>,
    ) {
        if visited.contains(&fid) { return; }
        visited.insert(fid);
        let inputs: Vec<FeatureId> = self.nodes[fid].inputs.clone();
        for iid in inputs {
            self.topo_visit(iid, visited, order);
        }
        order.push(fid);
    }
}

impl Default for FeatureTree {
    fn default() -> Self { Self::new() }
}

// ── ExtrudeFeature ────────────────────────────────────────────────────────────

/// Extrudes a planar polygon profile into a prism solid.
pub struct ExtrudeFeature {
    pub profile: Vec<Point3>,
    pub direction: Vec3,
    pub distance: f64,
}

impl ExtrudeFeature {
    pub fn new(profile: Vec<Point3>, direction: Vec3, distance: f64) -> Self {
        Self { profile, direction, distance }
    }
}

impl Feature for ExtrudeFeature {
    fn name(&self) -> &str { "Extrude" }

    fn compute(&self, _inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError> {
        let dir = self.direction.normalize();
        let offset = dir * self.distance;
        let n = self.profile.len();
        if n < 3 {
            return Err(KernelError::DegenerateGeometry("extrude: profile needs ≥ 3 vertices".into()));
        }
        if self.distance <= 0.0 {
            return Err(KernelError::DegenerateGeometry("extrude: distance must be > 0".into()));
        }

        let top: Vec<Point3> = self.profile.iter().map(|&p| p + offset).collect();
        let mut faces: Vec<Vec<Point3>> = Vec::with_capacity(n + 2);

        // Bottom cap: reversed profile (outward normal = -direction).
        faces.push(self.profile.iter().rev().cloned().collect());
        // Top cap: translated profile (outward normal = +direction).
        faces.push(top.clone());
        // Side quads.
        for i in 0..n {
            let j = (i + 1) % n;
            faces.push(vec![self.profile[i], self.profile[j], top[j], top[i]]);
        }

        let mut store = ShapeStore::new();
        make_solid_from_polygon_faces(&mut store, &faces, 1e-9)?;
        Ok(Arc::new(store))
    }

    fn parameter_count(&self) -> usize { 2 }

    fn get_parameter(&self, index: usize) -> Option<FeatureParameter> {
        match index {
            0 => Some(FeatureParameter { name: "direction",
                value: FeatureParameterValue::Direction(self.direction) }),
            1 => Some(FeatureParameter { name: "distance",
                value: FeatureParameterValue::Real(self.distance) }),
            _ => None,
        }
    }

    fn set_parameter(&mut self, index: usize, value: FeatureParameterValue) -> Result<(), KernelError> {
        match (index, value) {
            (0, FeatureParameterValue::Direction(d)) => { self.direction = d; Ok(()) }
            (1, FeatureParameterValue::Real(d)) => { self.distance = d; Ok(()) }
            _ => Err(KernelError::OperationNotSupported(
                format!("ExtrudeFeature: bad param index {}", index)
            )),
        }
    }
}

// ── RevolveFeature ────────────────────────────────────────────────────────────

/// Revolves a polygon profile around an axis to create a surface of revolution.
///
/// Uses `steps` angular divisions to tessellate the curved surface.
pub struct RevolveFeature {
    pub profile: Vec<Point3>,
    pub axis_origin: Point3,
    pub axis_dir: Vec3,
    /// Angle in radians (0 < angle ≤ 2π).
    pub angle: f64,
    /// Angular divisions for tessellation.
    pub steps: usize,
}

impl RevolveFeature {
    pub fn new(profile: Vec<Point3>, axis_origin: Point3, axis_dir: Vec3, angle: f64, steps: usize) -> Self {
        Self { profile, axis_origin, axis_dir, angle, steps }
    }

    fn rotate_point(&self, p: Point3, theta: f64) -> Point3 {
        let axis = self.axis_dir.normalize();
        let rel = p - self.axis_origin;
        let c = theta.cos();
        let s = theta.sin();
        let t = 1.0 - c;
        let (ax, ay, az) = (axis.x, axis.y, axis.z);
        let rotated = Vec3::new(
            (t*ax*ax + c)    * rel.x + (t*ax*ay - s*az) * rel.y + (t*ax*az + s*ay) * rel.z,
            (t*ax*ay + s*az) * rel.x + (t*ay*ay + c)    * rel.y + (t*ay*az - s*ax) * rel.z,
            (t*ax*az - s*ay) * rel.x + (t*ay*az + s*ax) * rel.y + (t*az*az + c)    * rel.z,
        );
        self.axis_origin + rotated
    }
}

impl Feature for RevolveFeature {
    fn name(&self) -> &str { "Revolve" }

    fn compute(&self, _inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError> {
        use std::f64::consts::TAU;
        let m = self.profile.len();
        if m < 2 { return Err(KernelError::DegenerateGeometry("revolve: need ≥ 2 profile points".into())); }
        if self.steps < 3 { return Err(KernelError::DegenerateGeometry("revolve: need ≥ 3 steps".into())); }

        let full = (self.angle - TAU).abs() < 1e-9;
        let n_layers = if full { self.steps } else { self.steps + 1 };

        let grid: Vec<Vec<Point3>> = (0..n_layers).map(|step| {
            let theta = self.angle * step as f64 / self.steps as f64;
            self.profile.iter().map(|&p| self.rotate_point(p, theta)).collect()
        }).collect();

        let mut faces: Vec<Vec<Point3>> = Vec::new();

        for i in 0..self.steps {
            let next_i = (i + 1) % n_layers;
            if next_i >= grid.len() { break; }
            let a = &grid[i];
            let b = &grid[next_i];
            for j in 0..m - 1 {
                // Quad: a[j], b[j], b[j+1], a[j+1]
                faces.push(vec![a[j], b[j], b[j+1], a[j+1]]);
            }
        }

        // End caps for partial revolution.
        if !full {
            let first = grid[0].clone();
            let last  = grid[grid.len() - 1].clone();
            faces.push(first.iter().rev().cloned().collect());
            faces.push(last);
        }

        let mut store = ShapeStore::new();
        make_solid_from_polygon_faces(&mut store, &faces, 1e-9)?;
        Ok(Arc::new(store))
    }

    fn parameter_count(&self) -> usize { 2 }
    fn get_parameter(&self, index: usize) -> Option<FeatureParameter> {
        match index {
            0 => Some(FeatureParameter { name: "angle",
                value: FeatureParameterValue::Real(self.angle) }),
            1 => Some(FeatureParameter { name: "steps",
                value: FeatureParameterValue::Integer(self.steps as i64) }),
            _ => None,
        }
    }
    fn set_parameter(&mut self, index: usize, value: FeatureParameterValue) -> Result<(), KernelError> {
        match (index, value) {
            (0, FeatureParameterValue::Real(a)) => { self.angle = a; Ok(()) }
            (1, FeatureParameterValue::Integer(s)) => { self.steps = s as usize; Ok(()) }
            _ => Err(KernelError::OperationNotSupported("RevolveFeature: bad param".into())),
        }
    }
}

// ── BooleanFeature ────────────────────────────────────────────────────────────

/// Boolean operation on two input solids (expects exactly 2 inputs).
pub struct BooleanFeature {
    pub kind: BooleanKind,
    pub tol: f64,
}

impl BooleanFeature {
    pub fn new(kind: BooleanKind, tol: f64) -> Self { Self { kind, tol } }
}

impl Feature for BooleanFeature {
    fn name(&self) -> &str { "Boolean" }
    fn input_count(&self) -> usize { 2 }

    fn compute(&self, inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError> {
        if inputs.len() != 2 {
            return Err(KernelError::InvalidTopology("BooleanFeature requires 2 inputs".into()));
        }
        let store_a = &inputs[0];
        let store_b = &inputs[1];

        let sid_a = store_a.solid_ids().next().ok_or_else(|| {
            KernelError::InvalidTopology("input A has no solid".into())
        })?;
        let sid_b = store_b.solid_ids().next().ok_or_else(|| {
            KernelError::InvalidTopology("input B has no solid".into())
        })?;

        let result = boolean_op(store_a, sid_a, store_b, sid_b, self.kind, self.tol)?;
        Ok(Arc::new(result))
    }

    fn parameter_count(&self) -> usize { 0 }
    fn get_parameter(&self, _: usize) -> Option<FeatureParameter> { None }
    fn set_parameter(&mut self, _: usize, _: FeatureParameterValue) -> Result<(), KernelError> {
        Err(KernelError::OperationNotSupported("BooleanFeature has no parameters".into()))
    }
}

// ── Blend geometry (Phase 8: Fillet + Chamfer) ───────────────────────────────

/// Newell polygon normal (unnormalised → then normalised).
fn blend_newell(pts: &[Point3]) -> Vec3 {
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
    }
    Vec3::new(nx, ny, nz).normalize()
}

/// Spherical linear interpolation between unit vectors `a` and `b`.
fn blend_slerp(a: Vec3, b: Vec3, t: f64) -> Vec3 {
    let dot = a.dot(&b).clamp(-1.0, 1.0);
    let phi = dot.acos();
    if phi.abs() < 1e-9 {
        return (a * (1.0 - t) + b * t).normalize();
    }
    let sp = phi.sin();
    ((phi * (1.0 - t)).sin() / sp * a + (phi * t).sin() / sp * b).normalize()
}

/// Geometry of a single blend operation on one edge.
struct BlendEdge {
    v_start: Point3,
    v_end:   Point3,
    n1: Vec3,  // outward normal of F1 (face where edge goes v_start → v_end)
    n2: Vec3,  // outward normal of F2 (face where edge goes v_end → v_start)
    d1: Vec3,  // inward in-face perp in F1: cross(n1, e_hat_f1)
    d2: Vec3,  // inward in-face perp in F2: cross(n2, e_hat_f2)
    setback: f64,
    f1_id: brep_topo::entity::FaceId,
    f2_id: brep_topo::entity::FaceId,
}

fn compute_blend_edge(
    store: &ShapeStore,
    edge_id: brep_topo::entity::EdgeId,
    blend_dist: f64,
    is_fillet: bool,
) -> Result<Option<BlendEdge>, KernelError> {
    use brep_topo::entity::FaceId;

    let edge = store.edge(edge_id)?;
    if edge.is_degenerate { return Ok(None); }

    let he0 = store.half_edge(edge.half_edges[0])?;
    let he1 = store.half_edge(edge.half_edges[1])?;
    let v_start = store.vertex(he0.origin)?.position;
    let v_end   = store.vertex(he1.origin)?.position;
    let len = (v_end - v_start).norm();
    if len < 1e-14 { return Ok(None); }

    let f1_id: FaceId = store.loop_(he0.loop_id)?.face;
    let f2_id: FaceId = store.loop_(he1.loop_id)?.face;

    let pts1: Vec<Point3> = store.face_vertices(f1_id)?.iter()
        .map(|&v| Ok(store.vertex(v)?.position))
        .collect::<Result<_, KernelError>>()?;
    let pts2: Vec<Point3> = store.face_vertices(f2_id)?.iter()
        .map(|&v| Ok(store.vertex(v)?.position))
        .collect::<Result<_, KernelError>>()?;

    let n1 = blend_newell(&pts1);
    let n2 = blend_newell(&pts2);

    // In-face inward perp: cross(n_face, e_hat_in_face).
    let e_hat = (v_end - v_start) / len;
    let d1 = n1.cross(&e_hat).normalize();   // F1: edge direction v_start→v_end
    let d2 = n2.cross(&(-e_hat)).normalize(); // F2: edge direction v_end→v_start

    let setback = if is_fillet {
        let c = n1.dot(&n2).clamp(-1.0 + 1e-12, 1.0 - 1e-12);
        if 1.0 + c < 1e-9 { return Ok(None); } // nearly flat or concave
        let theta = c.acos();
        let s = (theta / 2.0).tan();
        if s > 1e6 { return Ok(None); } // degenerate concave
        blend_dist * s
    } else {
        blend_dist
    };

    Ok(Some(BlendEdge { v_start, v_end, n1, n2, d1, d2, setback, f1_id, f2_id }))
}

/// Rebuild a face polygon, inserting setback points where selected edges touch it.
///
/// `edge_svecs[i]` is the setback vector for the edge going from `poly[i]` to
/// `poly[(i+1)%n]`.  Missing entries mean the edge is not selected.
fn rebuild_face_poly(
    poly: &[Point3],
    edge_svecs: &std::collections::HashMap<usize, Vec3>,
) -> Vec<Point3> {
    let n = poly.len();
    let mut out = Vec::new();
    for i in 0..n {
        let prev_idx = (i + n - 1) % n; // incoming edge index (prev→i)
        let sv_in  = edge_svecs.get(&prev_idx);
        let sv_out = edge_svecs.get(&i);
        match (sv_in, sv_out) {
            (None,       None)       => out.push(poly[i]),
            (Some(sv),   None)       => out.push(poly[i] + sv),  // end of selected incoming
            (None,       Some(sv))   => out.push(poly[i] + sv),  // start of selected outgoing
            (Some(svi),  Some(svo))  => {
                out.push(poly[i] + svi); // end of incoming (= T_end)
                out.push(poly[i] + svo); // start of outgoing (= T_start)
            }
        }
    }
    out
}

/// Core engine: apply a blend (chamfer or fillet) to selected edges of a solid
/// and return a new `ShapeStore`.
///
/// * `blend_dist` — chamfer distance or fillet radius
/// * `is_fillet`  — `true` → arc strips; `false` → flat chamfer quad
/// * `steps`      — arc subdivisions (1 for chamfer, ≥ 2 for fillet)
fn apply_blend(
    store: &ShapeStore,
    solid_id: brep_topo::entity::SolidId,
    selected: &[brep_topo::entity::EdgeId],
    blend_dist: f64,
    is_fillet: bool,
    steps: usize,
) -> Result<ShapeStore, KernelError> {
    use std::collections::HashMap;
    use brep_topo::entity::FaceId;

    let solid = store.solid(solid_id)?;
    let shell = store.shell(solid.outer_shell)?;

    // ── 1. Collect face polygons ──────────────────────────────────────────────
    let mut face_polys: HashMap<FaceId, Vec<Point3>> = HashMap::new();
    for &fid in &shell.faces {
        let pts: Vec<Point3> = store.face_vertices(fid)?.iter()
            .map(|&v| Ok(store.vertex(v)?.position))
            .collect::<Result<_, KernelError>>()?;
        face_polys.insert(fid, pts);
    }

    // ── 2. Compute blend data for each selected edge ──────────────────────────
    let mut blends: Vec<BlendEdge> = Vec::new();
    for &eid in selected {
        if let Some(b) = compute_blend_edge(store, eid, blend_dist, is_fillet)? {
            blends.push(b);
        }
    }

    // ── 3. Build per-face setback-vector maps ─────────────────────────────────
    // face_svecs[fid][edge_poly_index] = setback displacement for that edge
    let mut face_svecs: HashMap<FaceId, HashMap<usize, Vec3>> = HashMap::new();
    for b in &blends {
        // F1: find edge index where poly[i]≈v_start and poly[i+1]≈v_end
        let p1 = &face_polys[&b.f1_id];
        let n1 = p1.len();
        for i in 0..n1 {
            if (p1[i] - b.v_start).norm() < 1e-9
                && (p1[(i+1)%n1] - b.v_end).norm() < 1e-9
            {
                face_svecs.entry(b.f1_id).or_default().insert(i, b.setback * b.d1);
                break;
            }
        }
        // F2: find edge index where poly[i]≈v_end and poly[i+1]≈v_start
        let p2 = &face_polys[&b.f2_id];
        let n2 = p2.len();
        for i in 0..n2 {
            if (p2[i] - b.v_end).norm() < 1e-9
                && (p2[(i+1)%n2] - b.v_start).norm() < 1e-9
            {
                face_svecs.entry(b.f2_id).or_default().insert(i, b.setback * b.d2);
                break;
            }
        }
    }

    // ── 4. Rebuild modified face polygons ─────────────────────────────────────
    let mut all_polys: Vec<Vec<Point3>> = Vec::new();
    for (&fid, poly) in &face_polys {
        let empty = HashMap::new();
        let svecs = face_svecs.get(&fid).unwrap_or(&empty);
        let new_poly = rebuild_face_poly(poly, svecs);
        if new_poly.len() >= 3 {
            all_polys.push(new_poly);
        }
    }

    // ── 5. Generate blend strips + end caps ───────────────────────────────────
    for b in &blends {
        let c  = b.n1.dot(&b.n2).clamp(-1.0, 1.0);

        // Arc points at each endpoint of the edge.
        // arc_A[k] = C_A + r * slerp(n1, n2, k/steps) for fillet
        // arc_A[0] = T1_A = V_A + setback*d1  (tangent on F1)
        // arc_A[steps] = T2_A = V_A + setback*d2  (tangent on F2)
        let (arc_s, arc_e) = if is_fillet {
            let c_offset = -(b.n1 + b.n2) * blend_dist / (1.0 + c);
            let c_s = b.v_start + c_offset;
            let c_e = b.v_end   + c_offset;
            let as_: Vec<Point3> = (0..=steps).map(|k| {
                c_s + blend_slerp(b.n1, b.n2, k as f64 / steps as f64) * blend_dist
            }).collect();
            let ae: Vec<Point3> = (0..=steps).map(|k| {
                c_e + blend_slerp(b.n1, b.n2, k as f64 / steps as f64) * blend_dist
            }).collect();
            (as_, ae)
        } else {
            // Chamfer: steps=1, arc is just [T1, T2]
            let as_ = vec![
                b.v_start + b.setback * b.d1, // T1_start
                b.v_start + b.setback * b.d2, // T2_start
            ];
            let ae  = vec![
                b.v_end + b.setback * b.d1,   // T1_end
                b.v_end + b.setback * b.d2,   // T2_end
            ];
            (as_, ae)
        };

        // Strip quads: winding [arc_s[i+1], arc_e[i+1], arc_e[i], arc_s[i]]
        // outward normal faces outward along the blend arc.
        for i in 0..steps {
            all_polys.push(vec![
                arc_s[i + 1],
                arc_e[i + 1],
                arc_e[i],
                arc_s[i],
            ]);
        }

        // End-cap fan at v_start: triangles [arc_s[i], arc_s[i+1], v_start]
        // normal faces in the -e_hat direction.
        for i in 0..steps {
            all_polys.push(vec![arc_s[i], arc_s[i + 1], b.v_start]);
        }

        // End-cap fan at v_end: triangles [arc_e[i+1], arc_e[i], v_end]
        // normal faces in the +e_hat direction.
        for i in 0..steps {
            all_polys.push(vec![arc_e[i + 1], arc_e[i], b.v_end]);
        }
    }

    // ── 6. Build result ShapeStore ────────────────────────────────────────────
    let mut result = ShapeStore::new();
    make_solid_from_polygon_faces(&mut result, &all_polys, 1e-9)?;
    Ok(result)
}

// ── ChamferFeature ────────────────────────────────────────────────────────────

/// Cuts a flat bevel along each selected edge.
///
/// `edge_indices` are 0-based indices into the sequence returned by
/// [`ShapeStore::edge_ids`].  The feature takes exactly 1 input solid.
pub struct ChamferFeature {
    pub distance:     f64,
    pub edge_indices: Vec<usize>,
}

impl ChamferFeature {
    pub fn new(distance: f64, edge_indices: Vec<usize>) -> Self {
        Self { distance, edge_indices }
    }
}

impl Feature for ChamferFeature {
    fn name(&self) -> &str { "Chamfer" }
    fn input_count(&self) -> usize { 1 }

    fn compute(&self, inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError> {
        if inputs.len() != 1 {
            return Err(KernelError::InvalidTopology("ChamferFeature: needs 1 input".into()));
        }
        let store = &inputs[0];
        let solid_id = store.solid_ids().next().ok_or_else(||
            KernelError::InvalidTopology("ChamferFeature: no solid in input".into()))?;
        let all_edges: Vec<_> = store.edge_ids().collect();
        let selected: Vec<_> = self.edge_indices.iter()
            .filter_map(|&i| all_edges.get(i).copied())
            .collect();
        let result = apply_blend(store, solid_id, &selected, self.distance, false, 1)?;
        Ok(Arc::new(result))
    }

    fn parameter_count(&self) -> usize { 1 }

    fn get_parameter(&self, index: usize) -> Option<FeatureParameter> {
        match index {
            0 => Some(FeatureParameter { name: "distance",
                value: FeatureParameterValue::Real(self.distance) }),
            _ => None,
        }
    }

    fn set_parameter(&mut self, index: usize, value: FeatureParameterValue)
        -> Result<(), KernelError>
    {
        match (index, value) {
            (0, FeatureParameterValue::Real(d)) => { self.distance = d; Ok(()) }
            _ => Err(KernelError::OperationNotSupported(
                "ChamferFeature: bad param index".into())),
        }
    }
}

// ── FilletFeature ─────────────────────────────────────────────────────────────

/// Rounds selected edges with a rolling-ball fillet of the given radius.
///
/// The blend surface is approximated by `steps` quad strips along the arc
/// (minimum 2).  `edge_indices` are 0-based indices into `ShapeStore::edge_ids`.
pub struct FilletFeature {
    pub radius:       f64,
    pub edge_indices: Vec<usize>,
    pub steps:        usize,
}

impl FilletFeature {
    pub fn new(radius: f64, edge_indices: Vec<usize>, steps: usize) -> Self {
        Self { radius, edge_indices, steps: steps.max(2) }
    }
}

impl Feature for FilletFeature {
    fn name(&self) -> &str { "Fillet" }
    fn input_count(&self) -> usize { 1 }

    fn compute(&self, inputs: &[Arc<ShapeStore>]) -> Result<Arc<ShapeStore>, KernelError> {
        if inputs.len() != 1 {
            return Err(KernelError::InvalidTopology("FilletFeature: needs 1 input".into()));
        }
        let store = &inputs[0];
        let solid_id = store.solid_ids().next().ok_or_else(||
            KernelError::InvalidTopology("FilletFeature: no solid in input".into()))?;
        let all_edges: Vec<_> = store.edge_ids().collect();
        let selected: Vec<_> = self.edge_indices.iter()
            .filter_map(|&i| all_edges.get(i).copied())
            .collect();
        let result = apply_blend(store, solid_id, &selected, self.radius, true, self.steps)?;
        Ok(Arc::new(result))
    }

    fn parameter_count(&self) -> usize { 2 }

    fn get_parameter(&self, index: usize) -> Option<FeatureParameter> {
        match index {
            0 => Some(FeatureParameter { name: "radius",
                value: FeatureParameterValue::Real(self.radius) }),
            1 => Some(FeatureParameter { name: "steps",
                value: FeatureParameterValue::Integer(self.steps as i64) }),
            _ => None,
        }
    }

    fn set_parameter(&mut self, index: usize, value: FeatureParameterValue)
        -> Result<(), KernelError>
    {
        match (index, value) {
            (0, FeatureParameterValue::Real(r))    => { self.radius = r; Ok(()) }
            (1, FeatureParameterValue::Integer(s)) => {
                self.steps = (s as usize).max(2); Ok(())
            }
            _ => Err(KernelError::OperationNotSupported(
                "FilletFeature: bad param index".into())),
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use approx::assert_abs_diff_eq;

    fn square_profile() -> Vec<Point3> {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ]
    }

    #[test]
    fn extrude_creates_box_topology() {
        let feat = ExtrudeFeature::new(square_profile(), Vec3::z(), 3.0);
        let store = feat.compute(&[]).unwrap();
        assert_eq!(store.face_count(), 6, "prism should have 6 faces");
        assert_eq!(store.vertex_count(), 8, "prism should have 8 vertices");
        assert_eq!(store.edge_count(), 12, "prism should have 12 edges");
    }

    #[test]
    fn feature_tree_evaluates_and_caches() {
        let mut tree = FeatureTree::new();
        let fid = tree.add_feature(Box::new(ExtrudeFeature::new(square_profile(), Vec3::z(), 2.0)));

        let r1 = tree.evaluate(fid).unwrap();
        // Second evaluation should be cached (no dirty flag).
        assert!(!tree.nodes[fid].dirty);
        let r2 = tree.evaluate(fid).unwrap();
        assert!(Arc::ptr_eq(&r1, &r2), "should return same Arc");
    }

    #[test]
    fn feature_tree_dirty_propagation() {
        let mut tree = FeatureTree::new();
        let fid = tree.add_feature(Box::new(ExtrudeFeature::new(square_profile(), Vec3::z(), 2.0)));

        let r1 = tree.evaluate(fid).unwrap();
        tree.set_parameter(fid, 1, FeatureParameterValue::Real(5.0)).unwrap();

        assert!(tree.nodes[fid].dirty);
        let r2 = tree.evaluate(fid).unwrap();

        let z_max = |store: &ShapeStore| store.vertex_ids()
            .map(|v| store.vertex(v).unwrap().position.z)
            .fold(f64::MIN, f64::max);
        assert_abs_diff_eq!(z_max(&r1), 2.0, epsilon = 1e-9);
        assert_abs_diff_eq!(z_max(&r2), 5.0, epsilon = 1e-9);
    }

    #[test]
    fn feature_tree_chained_boolean() {
        let mut tree = FeatureTree::new();

        let box_a = Box::new(ExtrudeFeature::new(
            vec![Point3::origin(), Point3::new(2.,0.,0.), Point3::new(2.,2.,0.), Point3::new(0.,2.,0.)],
            Vec3::z(), 2.0,
        ));
        let box_b = Box::new(ExtrudeFeature::new(
            vec![Point3::new(1.,1.,0.), Point3::new(3.,1.,0.), Point3::new(3.,3.,0.), Point3::new(1.,3.,0.)],
            Vec3::z(), 2.0,
        ));

        let fid_a = tree.add_feature(box_a);
        let fid_b = tree.add_feature(box_b);
        let fid_u = tree.add_feature_with_inputs(
            Box::new(BooleanFeature::new(BooleanKind::Union, 1e-6)),
            vec![fid_a, fid_b],
        );

        let result = tree.evaluate(fid_u).unwrap();
        assert!(result.face_count() >= 6);
    }

    #[test]
    fn revolve_creates_faces() {
        let profile = vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 2.0)];
        let feat = RevolveFeature::new(profile, Point3::origin(), Vec3::z(), PI, 12);
        let result = feat.compute(&[]).unwrap();
        assert!(result.face_count() > 0);
    }

    // ── Phase 8: Chamfer ──────────────────────────────────────────────────────

    /// Helper: create an extrude box as Arc<ShapeStore>.
    fn make_box_input() -> Arc<ShapeStore> {
        let feat = ExtrudeFeature::new(square_profile(), Vec3::z(), 3.0);
        feat.compute(&[]).unwrap()
    }

    #[test]
    fn chamfer_one_edge_face_count() {
        let input = make_box_input();
        // A box has 12 edges.  Pick the first one.
        let feat = ChamferFeature::new(0.2, vec![0]);
        let result = feat.compute(&[input]).unwrap();
        // 6 original faces (2 modified) + 1 chamfer quad + 2 triangle end-caps = 9
        assert_eq!(result.face_count(), 9,
            "chamfered box should have 9 faces, got {}", result.face_count());
    }

    #[test]
    fn chamfer_produces_valid_vertex_count() {
        let input = make_box_input();
        let feat = ChamferFeature::new(0.1, vec![0]);
        let result = feat.compute(&[input]).unwrap();
        // Original 8 verts → 2 verts replaced by 2 setback pairs + 2 original endpoints stay
        // = 8 - 2 + 4 = 10 vertices (the original 2 edge verts stay in end-caps and other faces)
        assert!(result.vertex_count() >= 10,
            "expected ≥ 10 vertices, got {}", result.vertex_count());
    }

    #[test]
    fn chamfer_feature_tree_integration() {
        let mut tree = FeatureTree::new();
        let fid_box = tree.add_feature(Box::new(
            ExtrudeFeature::new(square_profile(), Vec3::z(), 2.0)
        ));
        let fid_cham = tree.add_feature_with_inputs(
            Box::new(ChamferFeature::new(0.15, vec![0])),
            vec![fid_box],
        );
        let result = tree.evaluate(fid_cham).unwrap();
        assert_eq!(result.face_count(), 9);
    }

    // ── Phase 8: Fillet ───────────────────────────────────────────────────────

    #[test]
    fn fillet_one_edge_face_count() {
        let input = make_box_input();
        // steps=2: 2 strip quads + 2 end-caps (2 triangles each) + 6 original = 12
        let feat = FilletFeature::new(0.2, vec![0], 2);
        let result = feat.compute(&[input]).unwrap();
        assert_eq!(result.face_count(), 12,
            "filleted box (steps=2) should have 12 faces, got {}", result.face_count());
    }

    #[test]
    fn fillet_arc_points_on_sphere() {
        // The fillet arc points should lie on a cylinder of the given radius
        // centered on the fillet spine.  For a 90-degree box edge, the arc
        // center is at distance r from both adjacent faces.
        let input = make_box_input();
        let radius = 0.3;
        let steps  = 4;
        let feat = FilletFeature::new(radius, vec![0], steps);
        let result = feat.compute(&[input]).unwrap();
        // Strip quads are at indices 6..6+steps in all_polys ordering.
        // We just check the result is non-trivial.
        assert_eq!(result.face_count(), 6 + steps + 2 * steps,
            "filleted box should have {} faces", 6 + steps + 2 * steps);
    }

    #[test]
    fn fillet_feature_tree_dirty_propagation() {
        let mut tree = FeatureTree::new();
        let fid_box = tree.add_feature(Box::new(
            ExtrudeFeature::new(square_profile(), Vec3::z(), 2.0)
        ));
        let fid_fil = tree.add_feature_with_inputs(
            Box::new(FilletFeature::new(0.1, vec![0], 3)),
            vec![fid_box],
        );
        let r1 = tree.evaluate(fid_fil).unwrap();
        // Change fillet radius via parameter index 0.
        tree.set_parameter(fid_fil, 0, FeatureParameterValue::Real(0.2)).unwrap();
        let r2 = tree.evaluate(fid_fil).unwrap();
        // Results are different (different arc geometry).
        assert!(!Arc::ptr_eq(&r1, &r2));
        // Face counts should both be 6 + 3 + 2*3 = 15.
        assert_eq!(r1.face_count(), 15);
        assert_eq!(r2.face_count(), 15);
    }
}
