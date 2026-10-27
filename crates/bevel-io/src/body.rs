// SPDX-License-Identifier: GPL-3.0-or-later

//! Converting kernel tessellations into export meshes.

use crate::{ExportMesh, IoError};
use bevel_kernel::BodyMesh;

/// One export mesh for a tessellated body: the triangles of all its faces,
/// with positions in millimetres.
///
/// Faces share their boundary points exactly (the tessellation discretizes
/// every edge once), so the result is watertight once positions are welded,
/// as the 3MF writer does.
///
/// # Errors
///
/// [`IoError::InvalidMesh`] if the body has more vertices than 32-bit
/// indices can address.
pub fn export_mesh(
    name: &str,
    color: Option<[u8; 4]>,
    mesh: &BodyMesh,
) -> Result<ExportMesh, IoError> {
    let mut out = ExportMesh {
        name: name.to_owned(),
        color,
        positions: Vec::new(),
        triangles: Vec::new(),
    };
    for face in &mesh.faces {
        let base = u32::try_from(out.positions.len()).map_err(|_| IoError::InvalidMesh {
            mesh: name.to_owned(),
            reason: "too many vertices for 32-bit indices".to_owned(),
        })?;
        out.positions
            .extend(face.positions.iter().map(bevel_math::Point3::to_array));
        out.triangles.extend(
            face.triangles
                .iter()
                .map(|t| [t[0] + base, t[1] + base, t[2] + base]),
        );
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevel_kernel::{
        CylinderParams, OpContext, OpTag, SphereParams, TessellationOptions, cylinder, sphere,
        tessellate,
    };
    use bevel_math::Frame3;

    fn meshes() -> Vec<ExportMesh> {
        let mut cx = OpContext::new();
        let bodies = [
            cylinder(
                &CylinderParams {
                    frame: Frame3::WORLD,
                    radius: 10.0,
                    height: 20.0,
                },
                OpTag(1),
                &mut cx,
            )
            .unwrap(),
            sphere(
                &SphereParams {
                    frame: Frame3::WORLD,
                    radius: 8.0,
                },
                OpTag(2),
                &mut cx,
            )
            .unwrap(),
        ];
        bodies
            .iter()
            .enumerate()
            .map(|(i, out)| {
                let body = &out.bodies[0];
                let options = TessellationOptions::export(&body.bounds());
                let mesh = tessellate(body, &options, &cx).unwrap();
                export_mesh(&format!("Body {i}"), None, &mesh).unwrap()
            })
            .collect()
    }

    #[test]
    fn tessellated_primitives_export_watertight() {
        for mesh in meshes() {
            mesh.validate().unwrap();
            assert!(mesh.is_watertight(), "{} is not watertight", mesh.name);
        }
    }

    #[test]
    fn tessellated_primitives_round_trip_through_3mf() {
        let meshes = meshes();
        let mut bytes = std::io::Cursor::new(Vec::new());
        crate::threemf::write(&meshes, &mut bytes).unwrap();
        let read = crate::threemf::read(bytes.get_ref()).unwrap();
        assert_eq!(read.len(), meshes.len());
        for (a, b) in meshes.iter().zip(&read) {
            assert_eq!(a.name, b.name);
            assert_eq!(a.triangles.len(), b.triangles.len());
            assert!(b.is_watertight());
        }
    }
}
