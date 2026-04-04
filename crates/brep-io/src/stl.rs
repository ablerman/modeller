//! STL (stereolithography) file export.
//!
//! Supports both ASCII and binary STL formats.  The input is a [`FaceMesh`]
//! slice from `brep-mesh`.  Every triangle is written as-is; no merging of
//! shared vertices is performed (STL does not require it).
//!
//! # Example
//! ```no_run
//! use brep_io::stl::{write_stl_binary, write_stl_ascii};
//! use brep_mesh::{tessellate, TessellationOptions};
//! use brep_topo::{primitives::make_box, store::ShapeStore};
//!
//! let mut store = ShapeStore::new();
//! make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
//! let meshes = tessellate(&store, &TessellationOptions::default()).unwrap();
//!
//! let mut buf = Vec::new();
//! write_stl_binary(&meshes, &mut buf).unwrap();
//! ```

use std::io::{self, Write};

use brep_mesh::FaceMesh;

/// Write `meshes` as binary STL to `writer`.
///
/// Binary STL format:
/// - 80-byte header (ASCII, padded with zeros)
/// - 4-byte triangle count (u32 LE)
/// - Per triangle: 12 floats (normal + 3 vertices) + 2-byte attribute (always 0)
pub fn write_stl_binary(meshes: &[FaceMesh], writer: &mut dyn Write) -> io::Result<()> {
    // Count total triangles.
    let total: u32 = meshes.iter().map(|m| m.triangles.len() as u32).sum();

    // 80-byte header.
    let header = b"Binary STL written by brep-io                                                   ";
    writer.write_all(&header[..80])?;

    // Triangle count.
    writer.write_all(&total.to_le_bytes())?;

    for mesh in meshes {
        for tri in &mesh.triangles {
            // Normal.
            writer.write_all(&(tri.normal.x as f32).to_le_bytes())?;
            writer.write_all(&(tri.normal.y as f32).to_le_bytes())?;
            writer.write_all(&(tri.normal.z as f32).to_le_bytes())?;
            // Vertices.
            for p in &tri.positions {
                writer.write_all(&(p.x as f32).to_le_bytes())?;
                writer.write_all(&(p.y as f32).to_le_bytes())?;
                writer.write_all(&(p.z as f32).to_le_bytes())?;
            }
            // Attribute byte count (always 0).
            writer.write_all(&0u16.to_le_bytes())?;
        }
    }
    Ok(())
}

/// Write `meshes` as ASCII STL to `writer`.
pub fn write_stl_ascii(meshes: &[FaceMesh], name: &str, writer: &mut dyn Write) -> io::Result<()> {
    writeln!(writer, "solid {name}")?;
    for mesh in meshes {
        for tri in &mesh.triangles {
            writeln!(
                writer,
                "  facet normal {:.8e} {:.8e} {:.8e}",
                tri.normal.x, tri.normal.y, tri.normal.z
            )?;
            writeln!(writer, "    outer loop")?;
            for p in &tri.positions {
                writeln!(writer, "      vertex {:.8e} {:.8e} {:.8e}", p.x, p.y, p.z)?;
            }
            writeln!(writer, "    endloop")?;
            writeln!(writer, "  endfacet")?;
        }
    }
    writeln!(writer, "endsolid {name}")?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use brep_mesh::{tessellate, TessellationOptions};
    use brep_topo::{primitives::make_box, store::ShapeStore};

    fn box_meshes() -> Vec<FaceMesh> {
        let mut store = ShapeStore::new();
        make_box(&mut store, 1.0, 1.0, 1.0).unwrap();
        tessellate(&store, &TessellationOptions::default()).unwrap()
    }

    #[test]
    fn binary_stl_header_is_80_bytes() {
        let meshes = box_meshes();
        let mut buf = Vec::new();
        write_stl_binary(&meshes, &mut buf).unwrap();
        assert_eq!(&buf[..80], b"Binary STL written by brep-io                                                   ");
    }

    #[test]
    fn binary_stl_triangle_count_matches() {
        let meshes = box_meshes();
        let total: u32 = meshes.iter().map(|m| m.triangles.len() as u32).sum();
        let mut buf = Vec::new();
        write_stl_binary(&meshes, &mut buf).unwrap();
        let count = u32::from_le_bytes(buf[80..84].try_into().unwrap());
        assert_eq!(count, total);
        // Each triangle = 50 bytes (12 f32s + 2 attr bytes).
        assert_eq!(buf.len(), 84 + total as usize * 50);
    }

    #[test]
    fn ascii_stl_starts_and_ends_correctly() {
        let meshes = box_meshes();
        let mut buf = Vec::new();
        write_stl_ascii(&meshes, "test_box", &mut buf).unwrap();
        let s = std::str::from_utf8(&buf).unwrap();
        assert!(s.starts_with("solid test_box\n"));
        assert!(s.ends_with("endsolid test_box\n"));
    }

    #[test]
    fn ascii_stl_facet_count_matches() {
        let meshes = box_meshes();
        let total: usize = meshes.iter().map(|m| m.triangles.len()).sum();
        let mut buf = Vec::new();
        write_stl_ascii(&meshes, "box", &mut buf).unwrap();
        let s = std::str::from_utf8(&buf).unwrap();
        let count = s.matches("facet normal").count();
        assert_eq!(count, total);
    }
}
