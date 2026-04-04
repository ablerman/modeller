//! STEP (ISO 10303-21) reader and writer for BRep solids.
//!
//! Supports a subset of AP214 sufficient to round-trip polyhedral solids and
//! simple analytical surfaces (Plane, CylindricalSurface, SphericalSurface,
//! ConicalSurface) with LineCurve and CircleCurve edges.
//!
//! # Entity coverage
//!
//! **Write**: Plane, CylindricalSurface, SphericalSurface, ConicalSurface,
//! LineCurve, CircleCurve.  BsplineSurface / BsplineCurve are not yet supported.
//!
//! **Read**: the same set of entities.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::Arc;

use brep_core::{EntityId, KernelError, Point3, Vec3};
use brep_geom::curve::{CircleCurve, LineCurve};
use brep_geom::surface::{ConicalSurface, CylindricalSurface, Plane, SphericalSurface};
use brep_topo::{
    binding::{CurveBinding, SurfaceBinding},
    entity::{
        Edge, EdgeId, Face, HalfEdge, HalfEdgeId, Loop, LoopId,
        LoopKind, Orientation, SolidId, Vertex, VertexId,
    },
    euler::make_solid_shell,
    store::ShapeStore,
};

// ── Public write API ──────────────────────────────────────────────────────────

/// Write `solid_id` from `store` to `w` as a STEP AP214 Physical File.
pub fn write_step<W: Write>(
    store: &ShapeStore,
    solid_id: SolidId,
    w: &mut W,
) -> Result<(), KernelError> {
    let mut writer = StepWriter::new();
    writer.write_solid(store, solid_id)?;
    let content = writer.finish();
    w.write_all(content.as_bytes())
        .map_err(|e| KernelError::Io(e))
}

// ── Public read API ───────────────────────────────────────────────────────────

/// Parse a STEP file from `r`, reconstruct a `ShapeStore`.
///
/// Returns the store and the ID of the first `MANIFOLD_SOLID_BREP` found.
pub fn read_step<R: Read>(r: &mut R) -> Result<(ShapeStore, SolidId), KernelError> {
    let mut buf = String::new();
    r.read_to_string(&mut buf)
        .map_err(|e| KernelError::Io(e))?;
    parse_step(&buf)
}

// ── Writer ────────────────────────────────────────────────────────────────────

struct StepWriter {
    next_id: u32,
    entities: Vec<(u32, String)>, // (id, entity string without #N=)
}

impl StepWriter {
    fn new() -> Self {
        Self { next_id: 1, entities: Vec::new() }
    }

    fn alloc(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn emit(&mut self, id: u32, s: String) {
        self.entities.push((id, s));
    }

    fn write_point(&mut self, p: Point3) -> u32 {
        let id = self.alloc();
        self.emit(id, format!("CARTESIAN_POINT('',({:.15},{:.15},{:.15}))", p.x, p.y, p.z));
        id
    }

    fn write_direction(&mut self, v: Vec3) -> u32 {
        let n = v.normalize();
        let id = self.alloc();
        self.emit(id, format!("DIRECTION('',({:.15},{:.15},{:.15}))", n.x, n.y, n.z));
        id
    }

    fn write_vector(&mut self, v: Vec3) -> u32 {
        let mag = v.norm();
        let dir_id = self.write_direction(v);
        let id = self.alloc();
        self.emit(id, format!("VECTOR('',#{},{:.15})", dir_id, mag));
        id
    }

    /// AXIS2_PLACEMENT_3D(name, origin, z_axis, x_axis)
    fn write_axis2(&mut self, origin: Point3, z_axis: Vec3, x_axis: Vec3) -> u32 {
        let pt_id  = self.write_point(origin);
        let z_id   = self.write_direction(z_axis);
        let x_id   = self.write_direction(x_axis);
        let id = self.alloc();
        self.emit(id, format!("AXIS2_PLACEMENT_3D('',#{},#{},#{})", pt_id, z_id, x_id));
        id
    }

    fn write_plane(&mut self, p: &Plane) -> u32 {
        let n = p.normal_vec();
        let x = p.u_axis.normalize();
        let ax = self.write_axis2(p.origin, n, x);
        let id = self.alloc();
        self.emit(id, format!("PLANE('',#{})", ax));
        id
    }

    fn write_cylindrical_surface(&mut self, c: &CylindricalSurface) -> u32 {
        let z = c.axis_dir.normalize();
        let x = c.x_axis.normalize();
        let ax = self.write_axis2(c.axis_origin, z, x);
        let id = self.alloc();
        self.emit(id, format!("CYLINDRICAL_SURFACE('',#{},{:.15})", ax, c.radius));
        id
    }

    fn write_spherical_surface(&mut self, s: &SphericalSurface) -> u32 {
        let z = s.z_axis.normalize();
        let x = s.x_axis.normalize();
        let ax = self.write_axis2(s.center, z, x);
        let id = self.alloc();
        self.emit(id, format!("SPHERICAL_SURFACE('',#{},{:.15})", ax, s.radius));
        id
    }

    fn write_conical_surface(&mut self, c: &ConicalSurface) -> u32 {
        let z = c.axis.normalize();
        let x = c.x_axis.normalize();
        let ax = self.write_axis2(c.apex, z, x);
        let id = self.alloc();
        self.emit(id, format!("CONICAL_SURFACE('',#{},{:.15},{:.15})", ax, 0.0_f64, c.half_angle));
        id
    }

    fn write_line_curve(&mut self, l: &LineCurve) -> u32 {
        let pt_id  = self.write_point(l.origin);
        let vec_id = self.write_vector(l.direction);
        let id = self.alloc();
        self.emit(id, format!("LINE('',#{},#{})", pt_id, vec_id));
        id
    }

    fn write_circle_curve(&mut self, c: &CircleCurve) -> u32 {
        let z = c.x_axis.cross(&c.y_axis).normalize();
        let ax = self.write_axis2(c.center, z, c.x_axis);
        let id = self.alloc();
        self.emit(id, format!("CIRCLE('',#{},{:.15})", ax, c.radius));
        id
    }

    fn write_surface_binding(
        &mut self,
        store: &ShapeStore,
        face_id: brep_topo::entity::FaceId,
    ) -> Result<u32, KernelError> {
        let face = store.face(face_id)?;
        let surf = &face.surface.surface;
        if let Some(p) = surf.as_any().downcast_ref::<Plane>() {
            Ok(self.write_plane(p))
        } else if let Some(c) = surf.as_any().downcast_ref::<CylindricalSurface>() {
            Ok(self.write_cylindrical_surface(c))
        } else if let Some(s) = surf.as_any().downcast_ref::<SphericalSurface>() {
            Ok(self.write_spherical_surface(s))
        } else if let Some(c) = surf.as_any().downcast_ref::<ConicalSurface>() {
            Ok(self.write_conical_surface(c))
        } else {
            Err(KernelError::InvalidTopology(format!(
                "STEP write: unsupported surface type '{}'",
                surf.type_name()
            )))
        }
    }

    fn write_edge_curve_for_edge(
        &mut self,
        store: &ShapeStore,
        edge_id: EdgeId,
        v_start_step: u32,
        v_end_step: u32,
    ) -> Result<u32, KernelError> {
        let edge = store.edge(edge_id)?;
        let curve_id = if let Some(cb) = &edge.curve {
            let c = &cb.curve;
            if let Some(l) = c.as_any().downcast_ref::<LineCurve>() {
                self.write_line_curve(l)
            } else if let Some(ci) = c.as_any().downcast_ref::<CircleCurve>() {
                self.write_circle_curve(ci)
            } else {
                return Err(KernelError::InvalidTopology(format!(
                    "STEP write: unsupported curve type '{}'",
                    c.type_name()
                )));
            }
        } else {
            // No curve binding — write a line between vertices.
            let [he0, _] = edge.half_edges;
            let he = store.half_edge(he0)?;
            let from = store.vertex(he.origin)?.position;
            let next = store.half_edge(he.next)?.origin;
            let to = store.vertex(next)?.position;
            let l = LineCurve::through(from, to).unwrap_or(LineCurve::new(from, Vec3::x()));
            self.write_line_curve(&l)
        };
        let id = self.alloc();
        self.emit(
            id,
            format!(
                "EDGE_CURVE('',#{},#{},#{}, .T.)",
                v_start_step, v_end_step, curve_id
            ),
        );
        Ok(id)
    }

    fn write_solid(
        &mut self,
        store: &ShapeStore,
        solid_id: SolidId,
    ) -> Result<(), KernelError> {
        let solid = store.solid(solid_id)?;
        let shell = store.shell(solid.outer_shell)?;

        // Map from VertexId → STEP entity id.
        let mut vertex_step: HashMap<u64, u32> = HashMap::new();
        let vid_key = |v: VertexId| -> u64 {
            (v.index() as u64) << 32 | v.generation() as u64
        };

        // Collect and write all unique vertices.
        let face_ids: Vec<_> = shell.faces.clone();
        for &fid in &face_ids {
            for vid in store.face_vertices(fid)? {
                let key = vid_key(vid);
                if !vertex_step.contains_key(&key) {
                    let pos = store.vertex(vid)?.position;
                    let pt_id = self.write_point(pos);
                    let vp_id = self.alloc();
                    self.emit(vp_id, format!("VERTEX_POINT('',#{})", pt_id));
                    vertex_step.insert(key, vp_id);
                }
            }
        }

        // Map EdgeId → STEP EDGE_CURVE id.
        let mut edge_step: HashMap<u64, u32> = HashMap::new();
        let eid_key = |e: EdgeId| -> u64 {
            (e.index() as u64) << 32 | e.generation() as u64
        };

        // Build edge step IDs.
        let edge_ids: Vec<EdgeId> = store.edge_ids().collect();
        for eid in edge_ids {
            let edge = store.edge(eid)?;
            let [he0, he1] = edge.half_edges;
            let start_vid = store.half_edge(he0)?.origin;
            let end_vid   = store.half_edge(he1)?.origin;

            // Get the STEP vertex IDs.
            let sv = *vertex_step.get(&vid_key(start_vid)).ok_or_else(|| {
                KernelError::InvalidEntityId
            })?;
            let ev = *vertex_step.get(&vid_key(end_vid)).ok_or_else(|| {
                KernelError::InvalidEntityId
            })?;

            let ec_id = self.write_edge_curve_for_edge(store, eid, sv, ev)?;
            edge_step.insert(eid_key(eid), ec_id);
        }

        // Write faces.
        let mut face_step_ids: Vec<u32> = Vec::new();
        for &fid in &face_ids {
            let face = store.face(fid)?;
            let surf_id = self.write_surface_binding(store, fid)?;

            // Walk the outer loop.
            let loop_he_ids = self.write_loop(store, face.outer_loop, &edge_step, &vertex_step)?;
            let loop_id = self.alloc();
            let he_list: String = loop_he_ids
                .iter()
                .map(|id| format!("#{}", id))
                .collect::<Vec<_>>()
                .join(",");
            self.emit(loop_id, format!("EDGE_LOOP('', ({}))", he_list));

            let bound_id = self.alloc();
            self.emit(bound_id, format!("FACE_OUTER_BOUND('',#{}, .T.)", loop_id));

            let face_sense = if face.surface.same_sense { ".T." } else { ".F." };
            let adv_id = self.alloc();
            self.emit(
                adv_id,
                format!("ADVANCED_FACE('', (#{}),#{},{})", bound_id, surf_id, face_sense),
            );
            face_step_ids.push(adv_id);
        }

        // Write shell.
        let face_list: String = face_step_ids
            .iter()
            .map(|id| format!("#{}", id))
            .collect::<Vec<_>>()
            .join(",");
        let shell_id_step = self.alloc();
        self.emit(shell_id_step, format!("CLOSED_SHELL('', ({}))", face_list));

        let solid_step = self.alloc();
        self.emit(solid_step, format!("MANIFOLD_SOLID_BREP('solid', #{})", shell_id_step));

        Ok(())
    }

    fn write_loop(
        &mut self,
        store: &ShapeStore,
        loop_id: LoopId,
        edge_step: &HashMap<u64, u32>,
        _vertex_step: &HashMap<u64, u32>,
    ) -> Result<Vec<u32>, KernelError> {
        let eid_key = |e: EdgeId| -> u64 {
            (e.index() as u64) << 32 | e.generation() as u64
        };

        let hes: Vec<HalfEdgeId> = store.loop_half_edges(loop_id)?.collect::<Result<_, _>>()?;
        let mut oriented_ids = Vec::new();

        for he_id in hes {
            let he = store.half_edge(he_id)?;
            let edge_id = he.edge;
            let ec_id = edge_step.get(&eid_key(edge_id)).copied().unwrap_or(0);

            // Sense: if this half-edge is the first (he[0]) in the edge, it's .T., else .F.
            let edge = store.edge(edge_id)?;
            let sense = if edge.half_edges[0] == he_id { ".T." } else { ".F." };

            let oe_id = self.alloc();
            self.emit(oe_id, format!("ORIENTED_EDGE('',*,*,#{},{})", ec_id, sense));
            oriented_ids.push(oe_id);
        }

        Ok(oriented_ids)
    }

    fn finish(self) -> String {
        let mut out = String::new();
        out.push_str("ISO-10303-21;\n");
        out.push_str("HEADER;\n");
        out.push_str("FILE_DESCRIPTION(('BRep kernel STEP export'),'2;1');\n");
        out.push_str("FILE_NAME('','',(''),(''),'brep-kernel','','');\n");
        out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));\n");
        out.push_str("ENDSEC;\n");
        out.push_str("DATA;\n");

        let mut sorted = self.entities;
        sorted.sort_by_key(|(id, _)| *id);
        for (id, s) in sorted {
            out.push_str(&format!("#{} = {};\n", id, s));
        }

        out.push_str("ENDSEC;\n");
        out.push_str("END-ISO-10303-21;\n");
        out
    }
}

// ── Parser ────────────────────────────────────────────────────────────────────

/// A parsed STEP attribute value.
#[derive(Debug, Clone)]
enum Attr {
    Ref(u32),
    Number(f64),
    Str(#[allow(dead_code)] String),
    Enum(String),
    List(Vec<Attr>),
    Derived,  // *
    Unset,    // $
}

/// A parsed STEP entity: type name + attribute list.
#[derive(Debug, Clone)]
struct StepEntity {
    etype: String,
    attrs: Vec<Attr>,
}

/// Parse the DATA section of a STEP file into a map from entity ID to StepEntity.
fn parse_data(input: &str) -> Result<HashMap<u32, StepEntity>, KernelError> {
    let mut map = HashMap::new();

    // Extract just the DATA section.
    let data_start = input.find("DATA;").ok_or_else(|| {
        KernelError::InvalidTopology("STEP: missing DATA; section".into())
    })? + 5;
    // Find the ENDSEC that comes *after* DATA; (not the one ending HEADER).
    let data_end = input[data_start..].find("ENDSEC;").ok_or_else(|| {
        KernelError::InvalidTopology("STEP: missing ENDSEC; after DATA".into())
    })? + data_start;
    let data = &input[data_start..data_end];

    for line in data.lines() {
        let line = line.trim();
        if line.is_empty() || !line.starts_with('#') {
            continue;
        }
        // Remove trailing ';'
        let line = line.trim_end_matches(';');

        // Split on ' = '
        let eq_pos = line.find(" = ").ok_or_else(|| {
            KernelError::InvalidTopology(format!("STEP: malformed line: {}", line))
        })?;
        let id_str = &line[1..eq_pos]; // strip leading '#'
        let id: u32 = id_str.parse().map_err(|_| {
            KernelError::InvalidTopology(format!("STEP: invalid entity id: {}", id_str))
        })?;
        let rest = &line[eq_pos + 3..]; // after " = "

        // rest is "ENTITY_NAME(attrs...)"
        let paren = rest.find('(').ok_or_else(|| {
            KernelError::InvalidTopology(format!("STEP: missing '(' in: {}", rest))
        })?;
        let etype = rest[..paren].trim().to_uppercase();
        let attrs_str = &rest[paren + 1..rest.rfind(')').unwrap_or(rest.len())];
        let attrs = parse_attrs(attrs_str)?;

        map.insert(id, StepEntity { etype, attrs });
    }

    Ok(map)
}

fn parse_attrs(s: &str) -> Result<Vec<Attr>, KernelError> {
    let mut attrs = Vec::new();
    let chars: Vec<char> = s.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        // Skip whitespace and commas between attrs
        while i < chars.len() && (chars[i] == ' ' || chars[i] == '\t' || chars[i] == ',') {
            i += 1;
        }
        if i >= chars.len() {
            break;
        }
        let (attr, new_i) = parse_single_attr(&chars, i)?;
        attrs.push(attr);
        i = new_i;
    }
    Ok(attrs)
}

fn parse_single_attr(chars: &[char], mut i: usize) -> Result<(Attr, usize), KernelError> {
    if i >= chars.len() {
        return Ok((Attr::Unset, i));
    }
    match chars[i] {
        '#' => {
            i += 1;
            let start = i;
            while i < chars.len() && chars[i].is_ascii_digit() {
                i += 1;
            }
            let s: String = chars[start..i].iter().collect();
            let n: u32 = s.parse().map_err(|_| {
                KernelError::InvalidTopology(format!("STEP: bad ref #{}", s))
            })?;
            Ok((Attr::Ref(n), i))
        }
        '\'' => {
            i += 1;
            let start = i;
            while i < chars.len() && chars[i] != '\'' {
                i += 1;
            }
            let s: String = chars[start..i].iter().collect();
            if i < chars.len() { i += 1; } // skip closing '
            Ok((Attr::Str(s), i))
        }
        '.' => {
            i += 1;
            let start = i;
            while i < chars.len() && chars[i] != '.' {
                i += 1;
            }
            let s: String = chars[start..i].iter().collect();
            if i < chars.len() { i += 1; } // skip closing .
            Ok((Attr::Enum(s), i))
        }
        '(' => {
            i += 1;
            let mut items = Vec::new();
            while i < chars.len() && chars[i] != ')' {
                while i < chars.len() && (chars[i] == ' ' || chars[i] == ',') {
                    i += 1;
                }
                if i >= chars.len() || chars[i] == ')' { break; }
                let (item, ni) = parse_single_attr(chars, i)?;
                items.push(item);
                i = ni;
            }
            if i < chars.len() { i += 1; } // skip ')'
            Ok((Attr::List(items), i))
        }
        '*' => Ok((Attr::Derived, i + 1)),
        '$' => Ok((Attr::Unset, i + 1)),
        c if c == '-' || c.is_ascii_digit() => {
            let start = i;
            if chars[i] == '-' { i += 1; }
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.' || chars[i] == 'E' || chars[i] == 'e' || (i > start && (chars[i] == '+' || chars[i] == '-'))) {
                i += 1;
            }
            let s: String = chars[start..i].iter().collect();
            let n: f64 = s.parse().map_err(|_| {
                KernelError::InvalidTopology(format!("STEP: bad number: {}", s))
            })?;
            Ok((Attr::Number(n), i))
        }
        _ => {
            // Unknown — skip to next comma or closing paren
            while i < chars.len() && chars[i] != ',' && chars[i] != ')' {
                i += 1;
            }
            Ok((Attr::Unset, i))
        }
    }
}

// ── Attribute extraction helpers ──────────────────────────────────────────────

fn attr_ref(a: &Attr) -> Option<u32> {
    if let Attr::Ref(n) = a { Some(*n) } else { None }
}

fn attr_number(a: &Attr) -> Option<f64> {
    if let Attr::Number(n) = a { Some(*n) } else { None }
}

fn attr_list(a: &Attr) -> Option<&[Attr]> {
    if let Attr::List(v) = a { Some(v) } else { None }
}

fn attr_enum(a: &Attr) -> Option<&str> {
    if let Attr::Enum(s) = a { Some(s.as_str()) } else { None }
}

fn get_point(map: &HashMap<u32, StepEntity>, id: u32) -> Result<Point3, KernelError> {
    let e = map.get(&id).ok_or(KernelError::InvalidEntityId)?;
    if e.etype != "CARTESIAN_POINT" {
        return Err(KernelError::InvalidTopology(format!("expected CARTESIAN_POINT, got {}", e.etype)));
    }
    // attrs[1] is the coordinate list
    let coords = attr_list(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
        .ok_or(KernelError::InvalidEntityId)?;
    let x = attr_number(coords.get(0).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let y = attr_number(coords.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let z = attr_number(coords.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    Ok(Point3::new(x, y, z))
}

fn get_direction(map: &HashMap<u32, StepEntity>, id: u32) -> Result<Vec3, KernelError> {
    let e = map.get(&id).ok_or(KernelError::InvalidEntityId)?;
    if e.etype != "DIRECTION" {
        return Err(KernelError::InvalidTopology(format!("expected DIRECTION, got {}", e.etype)));
    }
    let coords = attr_list(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
        .ok_or(KernelError::InvalidEntityId)?;
    let x = attr_number(coords.get(0).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let y = attr_number(coords.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let z = attr_number(coords.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    Ok(Vec3::new(x, y, z))
}

/// Returns (origin, z_axis, x_axis) from an AXIS2_PLACEMENT_3D.
fn get_axis2(map: &HashMap<u32, StepEntity>, id: u32) -> Result<(Point3, Vec3, Vec3), KernelError> {
    let e = map.get(&id).ok_or(KernelError::InvalidEntityId)?;
    if e.etype != "AXIS2_PLACEMENT_3D" {
        return Err(KernelError::InvalidTopology(format!("expected AXIS2_PLACEMENT_3D, got {}", e.etype)));
    }
    let pt_id  = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let z_id   = attr_ref(e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let x_id   = attr_ref(e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
    let origin = get_point(map, pt_id)?;
    let z      = get_direction(map, z_id)?;
    let x      = get_direction(map, x_id)?;
    Ok((origin, z, x))
}

// ── Reconstructor ─────────────────────────────────────────────────────────────

fn parse_step(input: &str) -> Result<(ShapeStore, SolidId), KernelError> {
    let map = parse_data(input)?;

    // Find MANIFOLD_SOLID_BREP.
    let solid_entry = map
        .iter()
        .find(|(_, e)| e.etype == "MANIFOLD_SOLID_BREP")
        .ok_or_else(|| KernelError::InvalidTopology("STEP: no MANIFOLD_SOLID_BREP found".into()))?;
    let solid_step_id = *solid_entry.0;
    let shell_ref = attr_ref(
        solid_entry.1.attrs.get(1).ok_or(KernelError::InvalidEntityId)?,
    )
    .ok_or(KernelError::InvalidEntityId)?;

    let shell_entry = map.get(&shell_ref).ok_or(KernelError::InvalidEntityId)?;
    if shell_entry.etype != "CLOSED_SHELL" {
        return Err(KernelError::InvalidTopology(
            "STEP: expected CLOSED_SHELL".into(),
        ));
    }
    let face_refs: Vec<u32> = attr_list(
        shell_entry.attrs.get(1).ok_or(KernelError::InvalidEntityId)?,
    )
    .ok_or(KernelError::InvalidEntityId)?
    .iter()
    .filter_map(|a| attr_ref(a))
    .collect();

    // Build a new ShapeStore.
    let mut store = ShapeStore::new();
    let result = make_solid_shell(&mut store);
    let solid_id = result.solid_id;
    let shell_id = result.shell_id;

    let sentinel_he: HalfEdgeId = EntityId::from_raw(0, 0);
    let _sentinel_e:  EdgeId     = EntityId::from_raw(0, 0);
    let sentinel_l:  LoopId     = EntityId::from_raw(0, 0);

    // Map from STEP entity ID to our VertexId.
    let mut step_to_vertex: HashMap<u32, VertexId> = HashMap::new();
    // Map from STEP EDGE_CURVE id to our EdgeId (to avoid duplicates).
    let mut step_to_edge: HashMap<u32, EdgeId> = HashMap::new();
    // Map from STEP EDGE_CURVE id to (start_vertex_step_id, end_vertex_step_id).
    let mut edge_curve_vertices: HashMap<u32, (u32, u32)> = HashMap::new();

    // First pass: create vertices.
    for &face_ref in &face_refs {
        let face_e = map.get(&face_ref).ok_or(KernelError::InvalidEntityId)?;
        if face_e.etype != "ADVANCED_FACE" {
            continue;
        }
        // attrs[1] = list of bounds
        let bounds_list = attr_list(face_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
            .ok_or(KernelError::InvalidEntityId)?;
        for bound_attr in bounds_list {
            let bound_ref = attr_ref(bound_attr).ok_or(KernelError::InvalidEntityId)?;
            let bound_e = map.get(&bound_ref).ok_or(KernelError::InvalidEntityId)?;
            // Get edge loop.
            let loop_ref = attr_ref(bound_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                .ok_or(KernelError::InvalidEntityId)?;
            let loop_e = map.get(&loop_ref).ok_or(KernelError::InvalidEntityId)?;
            let oe_list = attr_list(loop_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                .ok_or(KernelError::InvalidEntityId)?;
            for oe_attr in oe_list {
                let oe_ref = attr_ref(oe_attr).ok_or(KernelError::InvalidEntityId)?;
                let oe_e = map.get(&oe_ref).ok_or(KernelError::InvalidEntityId)?;
                // ORIENTED_EDGE attrs: name, *, *, edge_curve_ref, sense
                let ec_ref = attr_ref(oe_e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?)
                    .ok_or(KernelError::InvalidEntityId)?;
                let ec_e = map.get(&ec_ref).ok_or(KernelError::InvalidEntityId)?;
                // EDGE_CURVE attrs: name, v_start, v_end, curve, same_sense
                let vs_ref = attr_ref(ec_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                    .ok_or(KernelError::InvalidEntityId)?;
                let ve_ref = attr_ref(ec_e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?)
                    .ok_or(KernelError::InvalidEntityId)?;
                edge_curve_vertices.insert(ec_ref, (vs_ref, ve_ref));

                for &vp_ref in &[vs_ref, ve_ref] {
                    if step_to_vertex.contains_key(&vp_ref) { continue; }
                    let vp_e = map.get(&vp_ref).ok_or(KernelError::InvalidEntityId)?;
                    let pt_ref = attr_ref(vp_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                        .ok_or(KernelError::InvalidEntityId)?;
                    let pos = get_point(&map, pt_ref)?;
                    let vid = store.insert_vertex(Vertex::new(pos, store.tolerance.linear));
                    step_to_vertex.insert(vp_ref, vid);
                }
            }
        }
    }

    // Second pass: create edges.
    for (&ec_ref, &(vs_ref, ve_ref)) in &edge_curve_vertices {
        if step_to_edge.contains_key(&ec_ref) { continue; }
        let ec_e = map.get(&ec_ref).ok_or(KernelError::InvalidEntityId)?;
        let curve_ref = attr_ref(ec_e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?)
            .ok_or(KernelError::InvalidEntityId)?;
        let curve = build_curve(&map, curve_ref)?;

        let start_vid = *step_to_vertex.get(&vs_ref).ok_or(KernelError::InvalidEntityId)?;
        let end_vid   = *step_to_vertex.get(&ve_ref).ok_or(KernelError::InvalidEntityId)?;
        let from = store.vertex(start_vid)?.position;
        let to   = store.vertex(end_vid)?.position;

        // Create a placeholder edge with sentinel half-edges; we'll fill in half-edges
        // after creating them during face construction.
        let eid = store.insert_edge(Edge {
            half_edges: [sentinel_he, sentinel_he],
            curve: Some(CurveBinding::new(Arc::from(curve), 0.0, 1.0, true)),
            tolerance: store.tolerance.linear,
            is_degenerate: (from - to).norm() < 1e-12,
        });
        step_to_edge.insert(ec_ref, eid);
    }

    // Third pass: create faces, loops, half-edges.
    for &face_ref in &face_refs {
        let face_e = map.get(&face_ref).ok_or(KernelError::InvalidEntityId)?;
        if face_e.etype != "ADVANCED_FACE" { continue; }

        let surf_ref = attr_ref(face_e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?)
            .ok_or(KernelError::InvalidEntityId)?;
        let surf = build_surface(&map, surf_ref)?;

        let same_sense_attr = face_e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?;
        let same_sense = attr_enum(same_sense_attr).map(|s| s == "T").unwrap_or(true);

        let surface_binding = SurfaceBinding::new(Arc::from(surf), same_sense);
        let orientation = if same_sense { Orientation::Same } else { Orientation::Reversed };

        let face_id = store.insert_face(Face {
            outer_loop: sentinel_l,
            inner_loops: smallvec::SmallVec::new(),
            surface: surface_binding,
            orientation,
            shell: shell_id,
            tolerance: store.tolerance.linear,
        });

        let bounds_list = attr_list(face_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
            .ok_or(KernelError::InvalidEntityId)?
            .to_vec();

        for (bi, bound_attr) in bounds_list.iter().enumerate() {
            let bound_ref = attr_ref(bound_attr).ok_or(KernelError::InvalidEntityId)?;
            let bound_e = map.get(&bound_ref).ok_or(KernelError::InvalidEntityId)?;
            let is_outer = bound_e.etype == "FACE_OUTER_BOUND";
            let loop_ref = attr_ref(bound_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                .ok_or(KernelError::InvalidEntityId)?;
            let loop_e = map.get(&loop_ref).ok_or(KernelError::InvalidEntityId)?;
            let oe_list = attr_list(loop_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?)
                .ok_or(KernelError::InvalidEntityId)?
                .to_vec();

            let loop_id = store.insert_loop(Loop {
                first_half_edge: sentinel_he,
                loop_kind: if is_outer { LoopKind::Outer } else { LoopKind::Inner },
                face: face_id,
            });

            if bi == 0 {
                store.face_mut(face_id)?.outer_loop = loop_id;
            } else {
                store.face_mut(face_id)?.inner_loops.push(loop_id);
            }

            // Create half-edges for this loop.
            let mut he_ids: Vec<HalfEdgeId> = Vec::new();
            let mut he_edge_refs: Vec<(EdgeId, bool)> = Vec::new(); // (edge_id, is_forward)

            for oe_attr in &oe_list {
                let oe_ref = attr_ref(oe_attr).ok_or(KernelError::InvalidEntityId)?;
                let oe_e = map.get(&oe_ref).ok_or(KernelError::InvalidEntityId)?;
                let ec_ref = attr_ref(oe_e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?)
                    .ok_or(KernelError::InvalidEntityId)?;
                let sense_str = attr_enum(oe_e.attrs.get(4).ok_or(KernelError::InvalidEntityId)?)
                    .unwrap_or("T");
                let oe_sense = sense_str == "T";

                let (vs_ref, ve_ref) = *edge_curve_vertices.get(&ec_ref).ok_or(KernelError::InvalidEntityId)?;
                let edge_id = *step_to_edge.get(&ec_ref).ok_or(KernelError::InvalidEntityId)?;

                // Pick origin vertex based on sense.
                let origin_ref = if oe_sense { vs_ref } else { ve_ref };
                let origin_vid = *step_to_vertex.get(&origin_ref).ok_or(KernelError::InvalidEntityId)?;

                let he_id = store.insert_half_edge(HalfEdge {
                    origin: origin_vid,
                    twin: sentinel_he,
                    next: sentinel_he,
                    prev: sentinel_he,
                    loop_id,
                    edge: edge_id,
                    pcurve: None,
                });
                he_ids.push(he_id);
                he_edge_refs.push((edge_id, oe_sense));
            }

            // Wire next/prev.
            let n = he_ids.len();
            for idx in 0..n {
                store.half_edge_mut(he_ids[idx])?.next = he_ids[(idx + 1) % n];
                store.half_edge_mut(he_ids[idx])?.prev = he_ids[(idx + n - 1) % n];
            }
            if !he_ids.is_empty() {
                store.loop_mut(loop_id)?.first_half_edge = he_ids[0];
            }

            // Assign half-edges to edges (first = forward, second = reverse).
            for (idx, (edge_id, is_forward)) in he_edge_refs.iter().enumerate() {
                let edge = store.edge_mut(*edge_id)?;
                if *is_forward {
                    edge.half_edges[0] = he_ids[idx];
                } else {
                    edge.half_edges[1] = he_ids[idx];
                }
            }
        }

        store.shell_mut(shell_id)?.faces.push(face_id);
    }

    // Fourth pass: wire twins.
    let all_he_ids: Vec<HalfEdgeId> = {
        let shell = store.shell(shell_id)?;
        let mut hes = Vec::new();
        for &fid in &shell.faces {
            let face = store.face(fid)?;
            for he in store.loop_half_edges(face.outer_loop)? {
                hes.push(he?);
            }
        }
        hes
    };

    for &he_id in &all_he_ids {
        let edge_id = store.half_edge(he_id)?.edge;
        let edge = store.edge(edge_id)?;
        let [he0, he1] = edge.half_edges;
        if he0 == he_id && he1 != sentinel_he {
            store.half_edge_mut(he_id)?.twin = he1;
        } else if he1 == he_id && he0 != sentinel_he {
            store.half_edge_mut(he_id)?.twin = he0;
        }
    }

    store.shell_mut(shell_id)?.is_closed = true;
    let _ = solid_step_id; // used only for finding the entity
    Ok((store, solid_id))
}

fn build_curve(map: &HashMap<u32, StepEntity>, id: u32) -> Result<Box<dyn brep_geom::traits::Curve3d>, KernelError> {
    let e = map.get(&id).ok_or(KernelError::InvalidEntityId)?;
    match e.etype.as_str() {
        "LINE" => {
            let pt_ref  = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let vec_ref = attr_ref(e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let origin  = get_point(map, pt_ref)?;
            let vec_e   = map.get(&vec_ref).ok_or(KernelError::InvalidEntityId)?;
            let dir_ref = attr_ref(vec_e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let mag     = attr_number(vec_e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let dir     = get_direction(map, dir_ref)?;
            Ok(Box::new(LineCurve::new(origin, dir * mag)))
        }
        "CIRCLE" => {
            let ax_ref = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let radius = attr_number(e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let (center, z, x) = get_axis2(map, ax_ref)?;
            let y = z.cross(&x).normalize();
            Ok(Box::new(CircleCurve::new(center, radius, x, y)))
        }
        _ => Err(KernelError::InvalidTopology(format!("STEP read: unsupported curve type '{}'", e.etype))),
    }
}

fn build_surface(map: &HashMap<u32, StepEntity>, id: u32) -> Result<Box<dyn brep_geom::traits::Surface>, KernelError> {
    let e = map.get(&id).ok_or(KernelError::InvalidEntityId)?;
    match e.etype.as_str() {
        "PLANE" => {
            let ax_ref = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let (origin, z, x) = get_axis2(map, ax_ref)?;
            let y = z.cross(&x).normalize();
            Ok(Box::new(Plane::new(origin, x, y)))
        }
        "CYLINDRICAL_SURFACE" => {
            let ax_ref = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let radius = attr_number(e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let (origin, z, x) = get_axis2(map, ax_ref)?;
            let y = z.cross(&x).normalize();
            Ok(Box::new(CylindricalSurface {
                axis_origin: origin,
                axis_dir: z,
                radius,
                x_axis: x,
                y_axis: y,
            }))
        }
        "SPHERICAL_SURFACE" => {
            let ax_ref = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let radius = attr_number(e.attrs.get(2).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let (center, z, x) = get_axis2(map, ax_ref)?;
            let y = z.cross(&x).normalize();
            Ok(Box::new(SphericalSurface { center, radius, x_axis: x, y_axis: y, z_axis: z }))
        }
        "CONICAL_SURFACE" => {
            let ax_ref      = attr_ref(e.attrs.get(1).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let half_angle  = attr_number(e.attrs.get(3).ok_or(KernelError::InvalidEntityId)?).ok_or(KernelError::InvalidEntityId)?;
            let (apex, z, x) = get_axis2(map, ax_ref)?;
            let y = z.cross(&x).normalize();
            Ok(Box::new(ConicalSurface { apex, axis: z, half_angle, x_axis: x, y_axis: y }))
        }
        _ => Err(KernelError::InvalidTopology(format!("STEP read: unsupported surface type '{}'", e.etype))),
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use brep_topo::primitives::make_box;

    #[test]
    fn write_step_box() {
        let mut store = ShapeStore::new();
        let solid_id = make_box(&mut store, 2.0, 3.0, 4.0).unwrap();
        let mut buf = Vec::new();
        write_step(&store, solid_id, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.contains("ISO-10303-21"));
        assert!(text.contains("MANIFOLD_SOLID_BREP"));
        assert!(text.contains("CLOSED_SHELL"));
        assert!(text.contains("ADVANCED_FACE"));
    }

    #[test]
    fn round_trip_box() {
        let mut store = ShapeStore::new();
        let solid_id = make_box(&mut store, 2.0, 3.0, 4.0).unwrap();

        // Write to STEP.
        let mut buf = Vec::new();
        write_step(&store, solid_id, &mut buf).unwrap();

        // Read back.
        let (store2, _solid2) = read_step(&mut buf.as_slice()).unwrap();

        // Basic topology checks.
        assert_eq!(store2.face_count(), 6, "box should have 6 faces");
        assert_eq!(store2.vertex_count(), 8, "box should have 8 vertices");
    }

    #[test]
    fn step_file_has_correct_entities() {
        let mut store = ShapeStore::new();
        let solid_id = make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        let mut buf = Vec::new();
        write_step(&store, solid_id, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();

        // Should have 6 ADVANCED_FACE entities.
        let count = text.matches("ADVANCED_FACE").count();
        assert_eq!(count, 6, "expected 6 ADVANCED_FACE, got {}", count);

        // Should have 8 VERTEX_POINT entities.
        let count = text.matches("VERTEX_POINT").count();
        assert_eq!(count, 8, "expected 8 VERTEX_POINT, got {}", count);
    }
}
