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
}
