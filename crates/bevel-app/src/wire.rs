// SPDX-License-Identifier: GPL-3.0-or-later

//! Messages between the app and its rebuild worker in the browser. A
//! request carries the part as a `.bevel` file; a reply carries the bodies
//! as kernel JSON with their meshes and masses. Mesh faces and edges refer
//! to their B-rep entities by position in the body's face and edge lists,
//! which deserializing the body preserves, because slotmap keys do not
//! survive the trip.

use crate::rebuild::{Built, BuiltBody, Mass};
use bevel_doc::DocState;
use bevel_kernel::{Body, BodyMesh, EdgePolyline, FaceMesh};
use bevel_math::{Point3, Vector3};
use bevel_model::{BodyId, FeatureId, Part};
use bevel_ui::StepState;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A rebuild request.
#[derive(Serialize, Deserialize)]
pub struct Request {
    /// Echoed in the reply.
    pub generation: u64,
    /// The part as a `.bevel` file.
    pub file: Vec<u8>,
}

#[derive(Serialize, Deserialize)]
enum State {
    Ok,
    Warning(String),
    Error(String),
    Skipped(String),
    Suppressed,
    RolledBack,
}

#[derive(Serialize, Deserialize)]
struct Face {
    /// Position of the face in the body's face list.
    index: usize,
    positions: Vec<[f64; 3]>,
    normals: Vec<[f32; 3]>,
    triangles: Vec<[u32; 3]>,
}

#[derive(Serialize, Deserialize)]
struct Edge {
    /// Position of the edge in the body's edge list.
    index: usize,
    points: Vec<[f64; 3]>,
}

#[derive(Serialize, Deserialize)]
struct WireBody {
    id: BodyId,
    body: Body,
    faces: Vec<Face>,
    edges: Vec<Edge>,
    volume: f64,
    area: f64,
    centroid: [f64; 3],
}

/// A rebuild reply.
#[derive(Serialize, Deserialize)]
pub struct Reply {
    generation: u64,
    statuses: Vec<(FeatureId, State)>,
    parameters: Vec<(String, Result<String, String>)>,
    bodies: Vec<WireBody>,
}

/// The request for rebuilding `part`.
///
/// # Errors
///
/// A message if the part cannot be written.
pub fn request(generation: u64, part: Part) -> Result<String, String> {
    let state = DocState {
        part: Arc::new(part),
        ..DocState::default()
    };
    let file = bevel_doc::file::write(&state).map_err(|e| e.to_string())?;
    serde_json::to_string(&Request { generation, file }).map_err(|e| e.to_string())
}

/// The part a request asks to rebuild.
///
/// # Errors
///
/// A message if the request is malformed.
pub fn read_request(json: &str) -> Result<(u64, Part), String> {
    let request: Request = serde_json::from_str(json).map_err(|e| e.to_string())?;
    let state = bevel_doc::file::read(&request.file).map_err(|e| e.to_string())?;
    Ok((request.generation, state.part.as_ref().clone()))
}

fn state_to_wire(s: &StepState) -> State {
    match s {
        StepState::Ok => State::Ok,
        StepState::Warning(w) => State::Warning(w.clone()),
        StepState::Error(e) => State::Error(e.clone()),
        StepState::Skipped(r) => State::Skipped(r.clone()),
        StepState::Suppressed => State::Suppressed,
        StepState::RolledBack => State::RolledBack,
    }
}

fn state_from_wire(s: State) -> StepState {
    match s {
        State::Ok => StepState::Ok,
        State::Warning(w) => StepState::Warning(w),
        State::Error(e) => StepState::Error(e),
        State::Skipped(r) => StepState::Skipped(r),
        State::Suppressed => StepState::Suppressed,
        State::RolledBack => StepState::RolledBack,
    }
}

fn body_to_wire(b: &BuiltBody) -> WireBody {
    let faces: BTreeMap<_, usize> = b
        .body
        .faces()
        .enumerate()
        .map(|(i, (k, _))| (k, i))
        .collect();
    let edges: BTreeMap<_, usize> = b
        .body
        .edges()
        .enumerate()
        .map(|(i, (k, _))| (k, i))
        .collect();
    WireBody {
        id: b.id,
        body: b.body.as_ref().clone(),
        faces: b
            .mesh
            .faces
            .iter()
            .filter_map(|f| {
                Some(Face {
                    index: *faces.get(&f.face)?,
                    positions: f.positions.iter().map(Point3::to_array).collect(),
                    normals: f.normals.iter().map(|n| n.as_vec3().to_array()).collect(),
                    triangles: f.triangles.clone(),
                })
            })
            .collect(),
        edges: b
            .mesh
            .edges
            .iter()
            .filter_map(|e| {
                Some(Edge {
                    index: *edges.get(&e.edge)?,
                    points: e.points.iter().map(Point3::to_array).collect(),
                })
            })
            .collect(),
        volume: b.mass.volume,
        area: b.mass.area,
        centroid: b.mass.centroid.to_array(),
    }
}

fn body_from_wire(w: WireBody) -> Option<BuiltBody> {
    let faces: Vec<_> = w.body.faces().map(|(k, _)| k).collect();
    let edges: Vec<_> = w.body.edges().map(|(k, _)| k).collect();
    let mesh = BodyMesh {
        faces: w
            .faces
            .into_iter()
            .map(|f| {
                Some(FaceMesh {
                    face: *faces.get(f.index)?,
                    positions: f.positions.into_iter().map(Point3::from_array).collect(),
                    normals: f
                        .normals
                        .into_iter()
                        .map(|n| Vector3::from_array(n.map(f64::from)))
                        .collect(),
                    triangles: f.triangles,
                })
            })
            .collect::<Option<_>>()?,
        edges: w
            .edges
            .into_iter()
            .map(|e| {
                Some(EdgePolyline {
                    edge: *edges.get(e.index)?,
                    points: e.points.into_iter().map(Point3::from_array).collect(),
                })
            })
            .collect::<Option<_>>()?,
    };
    Some(BuiltBody {
        id: w.id,
        body: Arc::new(w.body),
        mesh,
        mass: Mass {
            volume: w.volume,
            area: w.area,
            centroid: Point3::from_array(w.centroid),
        },
    })
}

/// The reply carrying `built`.
///
/// # Errors
///
/// A message if it cannot be serialized.
pub fn reply(built: &Built) -> Result<String, String> {
    let reply = Reply {
        generation: built.generation,
        statuses: built
            .statuses
            .iter()
            .map(|(id, s)| (*id, state_to_wire(s)))
            .collect(),
        parameters: built.parameters.clone(),
        bodies: built.bodies.iter().map(body_to_wire).collect(),
    };
    serde_json::to_string(&reply).map_err(|e| e.to_string())
}

/// The rebuild a reply carries.
///
/// # Errors
///
/// A message if the reply is malformed or a body's mesh does not match it.
pub fn read_reply(json: &str) -> Result<Built, String> {
    let reply: Reply = serde_json::from_str(json).map_err(|e| e.to_string())?;
    Ok(Built {
        generation: reply.generation,
        statuses: reply
            .statuses
            .into_iter()
            .map(|(id, s)| (id, state_from_wire(s)))
            .collect(),
        parameters: reply.parameters,
        bodies: reply
            .bodies
            .into_iter()
            .map(body_from_wire)
            .collect::<Option<_>>()
            .ok_or_else(|| "a body's mesh does not match the body".to_owned())?,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use bevel_model::Regenerator;

    #[test]
    fn rebuilds_survive_the_trip_with_their_keys() {
        let state = bevel_doc::sample::primitives();
        let (generation, part) =
            read_request(&request(7, state.part.as_ref().clone()).unwrap()).unwrap();
        assert_eq!(generation, 7);
        let built =
            crate::rebuild::build(&mut Regenerator::new(), &part, generation, None).unwrap();
        let back = read_reply(&reply(&built).unwrap()).unwrap();
        assert_eq!(back.generation, 7);
        assert_eq!(back.statuses, built.statuses);
        assert_eq!(back.bodies.len(), built.bodies.len());
        for (a, b) in built.bodies.iter().zip(&back.bodies) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.mesh.faces.len(), b.mesh.faces.len());
            // Every mesh face and edge still belongs to the same named entity.
            for (fa, fb) in a.mesh.faces.iter().zip(&b.mesh.faces) {
                let na = &a.body.face(fa.face).unwrap().name;
                let nb = &b.body.face(fb.face).unwrap().name;
                assert_eq!(na, nb);
                assert_eq!(fa.positions, fb.positions);
            }
            for (ea, eb) in a.mesh.edges.iter().zip(&b.mesh.edges) {
                let na = &a.body.edge(ea.edge).unwrap().name;
                let nb = &b.body.edge(eb.edge).unwrap().name;
                assert_eq!(na, nb);
            }
        }
    }
}
