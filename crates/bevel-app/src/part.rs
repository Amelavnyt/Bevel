// SPDX-License-Identifier: GPL-3.0-or-later

//! The part shown until documents exist: the five kernel primitives as real
//! B-rep bodies. It turns bodies into the renderer's plain buffers, keeps the
//! table that maps pick ids back to faces and edges, describes selections for
//! the properties panel, and exports meshes.

use bevel_geom::Surface;
use bevel_kernel::{
    Body, BodyMesh, BoxParams, ConeParams, CylinderParams, EdgeKey, FaceKey, KernelError,
    MassProperties, OpContext, OpTag, Role, SphereParams, TessellationOptions, TopoName,
    TorusParams, mass_properties, tessellate,
};
use bevel_math::{Frame3, Point3, Vector3};
use bevel_render::{LineSet, TriangleMesh};
use bevel_ui::{ExportFormat, SelectionInfo};
use std::sync::Arc;

/// Linear colour of bodies (a neutral grey).
const BODY: [f32; 4] = [0.48, 0.52, 0.54, 1.0];
/// sRGB colour bodies are exported with.
const BODY_EXPORT: [u8; 4] = [184, 190, 194, 255];
/// Linear colour of B-rep edges.
const EDGE: [f32; 4] = [0.02, 0.02, 0.03, 1.0];

/// One body with everything derived from it.
pub struct PartBody {
    /// Display name.
    pub name: String,
    /// The B-rep.
    pub body: Arc<Body>,
    /// Display tessellation.
    pub mesh: BodyMesh,
    /// Volume, area, centre of mass and inertia.
    pub mass: MassProperties,
}

/// Something that can be picked or selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// A face of a body (by index into [`Part::bodies`]).
    Face { body: usize, face: FaceKey },
    /// An edge of a body.
    Edge { body: usize, edge: EdgeKey },
}

impl Target {
    /// Index of the body the target belongs to.
    pub fn body(self) -> usize {
        match self {
            Self::Face { body, .. } | Self::Edge { body, .. } => body,
        }
    }
}

/// The buffers for one frame of the scene and the pick table that goes
/// with them: id `i + 1` is `targets[i]`.
pub struct RenderScene {
    pub meshes: Vec<TriangleMesh>,
    pub lines: Vec<LineSet>,
    pub targets: Vec<Target>,
}

impl RenderScene {
    /// The target with pick id `id`.
    pub fn target(&self, id: u32) -> Option<Target> {
        let index = usize::try_from(id).ok()?.checked_sub(1)?;
        self.targets.get(index).copied()
    }

    /// The pick id of `target`, if it is in this scene.
    pub fn id(&self, target: Target) -> Option<u32> {
        let index = self.targets.iter().position(|&t| t == target)?;
        u32::try_from(index + 1).ok()
    }
}

/// The bodies of the part.
pub struct Part {
    pub bodies: Vec<PartBody>,
}

fn at(x: f64, y: f64, z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(x, y, z),
        ..Frame3::WORLD
    }
}

impl Part {
    /// The demonstration part: one of each primitive, side by side.
    ///
    /// # Errors
    ///
    /// Any error from building, tessellating or measuring the bodies.
    pub fn demo() -> Result<Self, KernelError> {
        let mut cx = OpContext::new();
        let outputs = [
            (
                "Box",
                bevel_kernel::box_(
                    &BoxParams {
                        frame: at(-150.0, -20.0, 0.0),
                        size: Vector3::splat(40.0),
                    },
                    OpTag(1),
                    &mut cx,
                )?,
            ),
            (
                "Cylinder",
                bevel_kernel::cylinder(
                    &CylinderParams {
                        frame: at(-75.0, 0.0, 0.0),
                        radius: 20.0,
                        height: 40.0,
                    },
                    OpTag(2),
                    &mut cx,
                )?,
            ),
            (
                "Sphere",
                bevel_kernel::sphere(
                    &SphereParams {
                        frame: at(0.0, 0.0, 22.0),
                        radius: 22.0,
                    },
                    OpTag(3),
                    &mut cx,
                )?,
            ),
            (
                "Cone",
                bevel_kernel::cone(
                    &ConeParams {
                        frame: at(75.0, 0.0, 0.0),
                        bottom_radius: 22.0,
                        top_radius: 10.0,
                        height: 40.0,
                    },
                    OpTag(4),
                    &mut cx,
                )?,
            ),
            (
                "Torus",
                bevel_kernel::torus(
                    &TorusParams {
                        frame: at(150.0, 0.0, 9.0),
                        major_radius: 21.0,
                        minor_radius: 9.0,
                    },
                    OpTag(5),
                    &mut cx,
                )?,
            ),
        ];
        let mut bodies = Vec::new();
        for (name, output) in outputs {
            for body in output.bodies {
                let options = TessellationOptions::fine(&body.bounds());
                let mesh = tessellate(&body, &options, &cx)?;
                let mass = mass_properties(&body, &cx)?;
                bodies.push(PartBody {
                    name: name.to_owned(),
                    body,
                    mesh,
                    mass,
                });
            }
        }
        Ok(Self { bodies })
    }

    fn visible<'a>(&'a self, visible: &'a [bool]) -> impl Iterator<Item = (usize, &'a PartBody)> {
        self.bodies
            .iter()
            .enumerate()
            .filter(|(i, _)| visible.get(*i).copied().unwrap_or(true))
    }

    /// Render buffers for the visible bodies, with one pick id per face and
    /// per edge.
    pub fn render_scene(&self, visible: &[bool]) -> RenderScene {
        let mut scene = RenderScene {
            meshes: Vec::new(),
            lines: Vec::new(),
            targets: Vec::new(),
        };
        let next_id = |scene: &mut RenderScene, target: Target| {
            scene.targets.push(target);
            u32::try_from(scene.targets.len()).unwrap_or(0)
        };
        for (index, part_body) in self.visible(visible) {
            for face in &part_body.mesh.faces {
                let id = next_id(
                    &mut scene,
                    Target::Face {
                        body: index,
                        face: face.face,
                    },
                );
                scene.meshes.push(TriangleMesh {
                    positions: face
                        .positions
                        .iter()
                        .map(|p| p.as_vec3().to_array())
                        .collect(),
                    normals: face
                        .normals
                        .iter()
                        .map(|n| n.as_vec3().to_array())
                        .collect(),
                    indices: face.triangles.iter().flatten().copied().collect(),
                    ids: vec![id; face.positions.len()],
                    color: BODY,
                });
            }
            let mut lines = LineSet {
                color: EDGE,
                ..LineSet::default()
            };
            // Seams are where a periodic surface's parameters wrap around,
            // not edges anyone drew; they are neither shown nor pickable.
            for polyline in &part_body.mesh.edges {
                if part_body.body.is_seam(polyline.edge) {
                    continue;
                }
                let id = next_id(
                    &mut scene,
                    Target::Edge {
                        body: index,
                        edge: polyline.edge,
                    },
                );
                for pair in polyline.points.windows(2) {
                    lines.positions.push(pair[0].as_vec3().to_array());
                    lines.positions.push(pair[1].as_vec3().to_array());
                    lines.ids.push(id);
                }
            }
            scene.lines.push(lines);
        }
        scene
    }

    /// The faces of body `index`, as targets.
    pub fn body_faces(&self, index: usize) -> Vec<Target> {
        self.bodies
            .get(index)
            .map(|b| {
                b.mesh
                    .faces
                    .iter()
                    .map(|f| Target::Face {
                        body: index,
                        face: f.face,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// What the properties panel shows for `selection`.
    pub fn selection_info(&self, selection: &[Target]) -> Option<SelectionInfo> {
        let first = *selection.first()?;
        let part_body = self.bodies.get(first.body())?;
        let mut rows = Vec::new();
        let title = if selection.iter().all(|t| t.body() == first.body())
            && selection.len() == part_body.mesh.faces.len()
            && selection.iter().all(|t| matches!(t, Target::Face { .. }))
        {
            part_body.name.clone()
        } else if selection.len() > 1 {
            format!("{} items", selection.len())
        } else {
            let body = &part_body.body;
            match first {
                Target::Face { face, .. } => {
                    let face = body.face(face)?;
                    rows.push(("Face".to_owned(), readable_name(&face.name)));
                    rows.push(("Surface".to_owned(), surface_kind(&face.surface).to_owned()));
                    format!("{} · face", part_body.name)
                }
                Target::Edge { edge, .. } => {
                    let edge = body.edge(edge)?;
                    rows.push(("Edge".to_owned(), readable_name(&edge.name)));
                    format!("{} · edge", part_body.name)
                }
            }
        };
        let m = &part_body.mass;
        rows.push(("Body".to_owned(), part_body.name.clone()));
        rows.push(("Volume".to_owned(), format!("{} mm³", fixed(m.volume))));
        rows.push(("Area".to_owned(), format!("{} mm²", fixed(m.area))));
        let c = m.centroid;
        rows.push((
            "Centre of mass".to_owned(),
            format!("{}, {}, {} mm", fixed(c.x), fixed(c.y), fixed(c.z)),
        ));
        // Unit density, so moments of inertia are in mm^5.
        for (axis, moment) in ["I1", "I2", "I3"]
            .iter()
            .zip(m.principal_moments.to_array())
        {
            rows.push((
                format!("Principal moment {axis}"),
                format!("{moment:.4e} mm⁵"),
            ));
        }
        Some(SelectionInfo { title, rows })
    }

    /// The visible bodies as a mesh file.
    ///
    /// # Errors
    ///
    /// A message if tessellating or writing fails.
    pub fn export(&self, visible: &[bool], format: ExportFormat) -> Result<Vec<u8>, String> {
        let cx = OpContext::new();
        let mut meshes = Vec::new();
        for (_, part_body) in self.visible(visible) {
            let options = TessellationOptions::export(&part_body.body.bounds());
            let mesh = tessellate(&part_body.body, &options, &cx).map_err(|e| e.to_string())?;
            meshes.push(
                bevel_io::export_mesh(&part_body.name, Some(BODY_EXPORT), &mesh)
                    .map_err(|e| e.to_string())?,
            );
        }
        let mut bytes = std::io::Cursor::new(Vec::new());
        match format {
            ExportFormat::Stl => bevel_io::stl::write_binary(&meshes, &mut bytes),
            ExportFormat::ThreeMf => bevel_io::threemf::write(&meshes, &mut bytes),
        }
        .map_err(|e| e.to_string())?;
        Ok(bytes.into_inner())
    }
}

/// `value` with two decimals, without a minus sign on values that round to zero.
fn fixed(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    // Adding zero turns -0.0 into +0.0.
    format!("{:.2}", rounded + 0.0)
}

/// A short name for the properties panel: the role an entity plays
/// ("Top", "Side 2", "Seam 1"), or its full persistent name when it has no
/// single role.
fn readable_name(name: &TopoName) -> String {
    match name {
        TopoName::Generated { role, .. } => match role {
            Role::Side(i) => format!("Side {i}"),
            Role::Seam(i) => format!("Seam {i}"),
            Role::Pole(i) => format!("Pole {i}"),
            Role::Edge(i) => format!("Edge {i}"),
            Role::Vertex(i) => format!("Vertex {i}"),
            other => format!("{other:?}"),
        },
        other => other.to_string(),
    }
}

fn surface_kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::Cylinder(_) => "Cylinder",
        Surface::Cone(_) => "Cone",
        Surface::Sphere(_) => "Sphere",
        Surface::Torus(_) => "Torus",
        Surface::Nurbs(_) => "NURBS",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_part_has_five_valid_bodies() {
        let part = Part::demo().unwrap();
        assert_eq!(part.bodies.len(), 5);
        for b in &part.bodies {
            assert!(
                bevel_kernel::validate(&b.body).is_empty(),
                "{} invalid",
                b.name
            );
            assert!(b.mass.volume > 0.0);
        }
    }

    #[test]
    fn pick_ids_map_back_to_their_targets() {
        let part = Part::demo().unwrap();
        let scene = part.render_scene(&[true; 5]);
        assert!(!scene.targets.is_empty());
        for (i, &target) in scene.targets.iter().enumerate() {
            let id = u32::try_from(i + 1).unwrap();
            assert_eq!(scene.target(id), Some(target));
            assert_eq!(scene.id(target), Some(id));
        }
        assert_eq!(scene.target(0), None);
        // Every mesh vertex carries its face's id; every segment its edge's.
        for mesh in &scene.meshes {
            assert_eq!(mesh.ids.len(), mesh.positions.len());
        }
        for lines in &scene.lines {
            assert_eq!(lines.ids.len() * 2, lines.positions.len());
        }
    }

    #[test]
    fn hidden_bodies_are_not_drawn_or_exported() {
        let part = Part::demo().unwrap();
        let visible = [true, false, false, false, false];
        let scene = part.render_scene(&visible);
        assert!(scene.targets.iter().all(|t| t.body() == 0));
        let stl = part.export(&visible, ExportFormat::Stl).unwrap();
        let triangles = bevel_io::stl::read(&stl).unwrap();
        assert_eq!(triangles.len(), 12, "only the box's 12 triangles");
    }

    #[test]
    fn exported_3mf_is_watertight_per_body() {
        let part = Part::demo().unwrap();
        let bytes = part.export(&[true; 5], ExportFormat::ThreeMf).unwrap();
        let meshes = bevel_io::threemf::read(&bytes).unwrap();
        assert_eq!(meshes.len(), 5);
        assert!(meshes.iter().all(bevel_io::ExportMesh::is_watertight));
    }

    #[test]
    fn values_that_round_to_zero_have_no_sign() {
        assert_eq!(fixed(-0.001), "0.00");
        assert_eq!(fixed(-0.0), "0.00");
        assert_eq!(fixed(-1.234), "-1.23");
    }

    #[test]
    fn selecting_all_faces_of_a_body_names_the_body() {
        let part = Part::demo().unwrap();
        let info = part.selection_info(&part.body_faces(2)).unwrap();
        assert_eq!(info.title, "Sphere");
        assert!(info.rows.iter().any(|(label, _)| label == "Volume"));
    }

    #[test]
    fn seams_are_neither_drawn_nor_pickable() {
        let part = Part::demo().unwrap();
        let scene = part.render_scene(&[true; 5]);
        for target in &scene.targets {
            if let Target::Edge { body, edge } = *target {
                assert!(!part.bodies[body].body.is_seam(edge));
            }
        }
        // The sphere's only face is bounded by a seam and two poles, so it
        // shows no edges at all.
        assert!(
            !scene
                .targets
                .iter()
                .any(|t| matches!(t, Target::Edge { body: 2, .. }))
        );
    }
}
