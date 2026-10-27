// SPDX-License-Identifier: GPL-3.0-or-later

//! The bodies of the last rebuild as the viewport sees them. It turns bodies
//! into the renderer's plain buffers, keeps the table that maps pick ids
//! back to faces and edges, measures selections for the Measure dialog, and
//! exports meshes.

use crate::rebuild::{Built, Mass};
use bevel_geom::{Curve3, Surface};
use bevel_kernel::{
    Body, BodyMesh, EdgeKey, FaceKey, OpContext, Role, TessellationOptions, TopoName, edge_length,
    face_area, tessellate,
};
use bevel_model::BodyId;
use bevel_render::{LineSet, TriangleMesh};
use bevel_ui::{ExportFormat, MeasureValue, Measurement, SelectionInfo};
use std::sync::Arc;

/// Linear colour of bodies (a neutral grey).
const BODY: [f32; 4] = [0.48, 0.52, 0.54, 1.0];
/// sRGB colour bodies are exported with.
const BODY_EXPORT: [u8; 4] = [184, 190, 194, 255];
/// Linear colour of B-rep edges.
const EDGE: [f32; 4] = [0.02, 0.02, 0.03, 1.0];

/// One body with everything derived from it.
pub struct PartBody {
    /// Stable id.
    pub id: BodyId,
    /// Display name.
    pub name: String,
    /// The B-rep.
    pub body: Arc<Body>,
    /// Display tessellation.
    pub mesh: BodyMesh,
    /// Volume, area and centre of mass.
    pub mass: Mass,
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

impl Part {
    /// The bodies of a rebuild, named by `name`.
    pub fn from_built(built: &Built, name: &dyn Fn(BodyId) -> String) -> Self {
        Self {
            bodies: built
                .bodies
                .iter()
                .map(|b| PartBody {
                    id: b.id,
                    name: name(b.id),
                    body: Arc::clone(&b.body),
                    mesh: b.mesh.clone(),
                    mass: b.mass,
                })
                .collect(),
        }
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

    /// What the Measure dialog shows for `selection`.
    pub fn selection_info(&self, selection: &[Target]) -> Option<SelectionInfo> {
        let first = *selection.first()?;
        let part_body = self.bodies.get(first.body())?;
        let whole_body = selection.iter().all(|t| t.body() == first.body())
            && selection.len() == part_body.mesh.faces.len()
            && selection.iter().all(|t| matches!(t, Target::Face { .. }));
        if whole_body {
            return Some(body_info(part_body));
        }
        let cx = OpContext::new();
        if let [only] = selection {
            return self.single_info(*only, &cx);
        }
        let (mut faces, mut edges, mut area, mut length) = (0, 0, 0.0, 0.0);
        for target in selection {
            let part_body = self.bodies.get(target.body())?;
            match *target {
                Target::Face { face, .. } => {
                    faces += 1;
                    area += face_area(&part_body.body, face, &cx).ok()?;
                }
                Target::Edge { edge, .. } => {
                    edges += 1;
                    length += edge_length(&part_body.body, edge).ok()?;
                }
            }
        }
        let mut measurements = Vec::new();
        let mut parts = Vec::new();
        if faces > 0 {
            parts.push(count(faces, "face"));
            measurements.push(numbers("Total area", vec![area], "mm²"));
        }
        if edges > 0 {
            parts.push(count(edges, "edge"));
            measurements.push(numbers("Total length", vec![length], "mm"));
        }
        Some(SelectionInfo {
            title: parts.join(", "),
            measurements,
        })
    }

    fn single_info(&self, target: Target, cx: &OpContext) -> Option<SelectionInfo> {
        let part_body = self.bodies.get(target.body())?;
        let body = &part_body.body;
        Some(match target {
            Target::Face { face: key, .. } => {
                let face = body.face(key)?;
                SelectionInfo {
                    title: format!("{} · {} face", part_body.name, readable_name(&face.name)),
                    measurements: vec![
                        text("Type", surface_kind(&face.surface)),
                        numbers("Area", vec![face_area(body, key, cx).ok()?], "mm²"),
                    ],
                }
            }
            Target::Edge { edge: key, .. } => {
                let edge = body.edge(key)?;
                SelectionInfo {
                    title: format!("{} · edge", part_body.name),
                    measurements: vec![
                        text("Type", curve_kind(&edge.curve)),
                        numbers("Length", vec![edge_length(body, key).ok()?], "mm"),
                    ],
                }
            }
        })
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

/// Volume, area and centre of mass of a whole body.
fn body_info(part_body: &PartBody) -> SelectionInfo {
    let m = &part_body.mass;
    SelectionInfo {
        title: part_body.name.clone(),
        measurements: vec![
            text("Type", "Solid body"),
            numbers("Volume", vec![m.volume], "mm³"),
            numbers("Area", vec![m.area], "mm²"),
            numbers("Centre of mass", m.centroid.to_array().to_vec(), "mm"),
        ],
    }
}

fn numbers(label: &str, values: Vec<f64>, unit: &'static str) -> Measurement {
    Measurement {
        label: label.to_owned(),
        value: MeasureValue::Numbers { values, unit },
    }
}

fn text(label: &str, value: &str) -> Measurement {
    Measurement {
        label: label.to_owned(),
        value: MeasureValue::Text(value.to_owned()),
    }
}

/// "1 face", "3 edges".
fn count(n: usize, noun: &str) -> String {
    if n == 1 {
        format!("1 {noun}")
    } else {
        format!("{n} {noun}s")
    }
}

/// A short name for the Measure dialog: the role an entity plays
/// ("Top", "Side", "Seam 1"), or its full persistent name when it has no
/// single role.
fn readable_name(name: &TopoName) -> String {
    match name {
        TopoName::Generated { role, .. } => match role {
            Role::Side(_) => "Side".to_owned(),
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
        Surface::Plane(_) => "Planar",
        Surface::Cylinder(_) => "Cylindrical",
        Surface::Cone(_) => "Conical",
        Surface::Sphere(_) => "Spherical",
        Surface::Torus(_) => "Toroidal",
        Surface::Nurbs(_) => "NURBS",
    }
}

fn curve_kind(curve: &Curve3) -> &'static str {
    match curve {
        Curve3::Line(_) => "Line",
        Curve3::Circle(_) => "Circle",
        Curve3::Ellipse(_) => "Ellipse",
        Curve3::Nurbs(_) => "NURBS",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sample document's five primitives, built as the app builds them.
    fn sample() -> Part {
        let state = bevel_doc::sample::primitives();
        let built =
            crate::rebuild::build(&mut bevel_model::Regenerator::new(), &state.part, 1, None)
                .unwrap();
        Part::from_built(&built, &|id| state.body_name(id))
    }

    #[test]
    fn demo_part_has_five_valid_bodies() {
        let part = sample();
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
        let part = sample();
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
        let part = sample();
        let visible = [true, false, false, false, false];
        let scene = part.render_scene(&visible);
        assert!(scene.targets.iter().all(|t| t.body() == 0));
        let stl = part.export(&visible, ExportFormat::Stl).unwrap();
        let triangles = bevel_io::stl::read(&stl).unwrap();
        assert_eq!(triangles.len(), 12, "only the box's 12 triangles");
    }

    #[test]
    fn exported_3mf_is_watertight_per_body() {
        let part = sample();
        let bytes = part.export(&[true; 5], ExportFormat::ThreeMf).unwrap();
        let meshes = bevel_io::threemf::read(&bytes).unwrap();
        assert_eq!(meshes.len(), 5);
        assert!(meshes.iter().all(bevel_io::ExportMesh::is_watertight));
    }

    #[test]
    fn selecting_all_faces_of_a_body_names_the_body() {
        let part = sample();
        let info = part.selection_info(&part.body_faces(2)).unwrap();
        assert_eq!(info.title, "Sphere1");
        assert!(info.measurements.iter().any(|m| m.label == "Volume"));
    }

    fn value(info: &SelectionInfo, label: &str) -> f64 {
        match &info
            .measurements
            .iter()
            .find(|m| m.label == label)
            .unwrap()
            .value
        {
            MeasureValue::Numbers { values, .. } => values[0],
            MeasureValue::Text(_) => panic!("{label} is text"),
        }
    }

    #[test]
    fn seams_are_neither_drawn_nor_pickable() {
        let part = sample();
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

    #[test]
    fn a_face_and_an_edge_are_measured() {
        let part = sample();
        let cylinder = &part.bodies[1].body;
        let (side, _) = cylinder
            .faces()
            .find(|(_, f)| matches!(f.surface, Surface::Cylinder(_)))
            .unwrap();
        let info = part
            .selection_info(&[Target::Face {
                body: 1,
                face: side,
            }])
            .unwrap();
        let expected = 2.0 * std::f64::consts::PI * 20.0 * 40.0;
        assert!((value(&info, "Area") - expected).abs() < 1e-6 * expected);
        let (circle, _) = cylinder
            .edges()
            .find(|(_, e)| matches!(e.curve, Curve3::Circle(_)))
            .unwrap();
        let both = [
            Target::Face {
                body: 1,
                face: side,
            },
            Target::Edge {
                body: 1,
                edge: circle,
            },
        ];
        let info = part.selection_info(&both).unwrap();
        assert_eq!(info.title, "1 face, 1 edge");
        let circumference = 2.0 * std::f64::consts::PI * 20.0;
        assert!((value(&info, "Total length") - circumference).abs() < 1e-9);
    }
}
