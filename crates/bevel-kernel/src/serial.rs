// SPDX-License-Identifier: GPL-3.0-or-later

//! Serialization of bodies: the flat, index-based form described under
//! "Serialization" on [`Body`], the conversions to and from it, and the
//! `Serialize`, `Deserialize` and `PartialEq` impls built on them.
//! Deserialization goes through [`BodyBuilder`] and validation, so it never
//! produces an invalid body.

use bevel_geom::{Curve2, Curve3, Surface};
use bevel_math::{Interval, Point3};
use serde::de::Error as _;
use serde::ser::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use slotmap::SecondaryMap;

use crate::KernelError;
use crate::builder::BodyBuilder;
use crate::naming::TopoName;
use crate::topology::{Body, BodyKind, Edge, EdgeKey, FaceKey, VertexKey};

/// The serialized form of a body (see the [module documentation](self)).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct BodyData {
    kind: BodyKind,
    vertices: Vec<VertexData>,
    edges: Vec<EdgeData>,
    faces: Vec<FaceData>,
    shells: Vec<ShellData>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct VertexData {
    name: TopoName,
    point: Point3,
    tol: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EdgeData {
    name: TopoName,
    start: usize,
    end: usize,
    curve: Curve3,
    range: Interval,
    degenerate: bool,
    tol: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FaceData {
    name: TopoName,
    surface: Surface,
    reversed: bool,
    loops: Vec<Vec<CoedgeData>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoedgeData {
    edge: usize,
    reversed: bool,
    pcurve: Curve2,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ShellData {
    faces: Vec<usize>,
    closed: bool,
}

/// A reference in a body that cannot be written as an index.
const DANGLING: KernelError = KernelError::InvalidInput {
    what: "body",
    reason: "a reference points to an entity that does not exist",
};

/// A reference in serialized data that is out of range.
const OUT_OF_RANGE: KernelError = KernelError::InvalidInput {
    what: "serialized body",
    reason: "an index refers past the end of its list",
};

impl Body {
    /// The flat form of the body.
    fn to_data(&self) -> Result<BodyData, KernelError> {
        let mut vertex_index: SecondaryMap<VertexKey, usize> = SecondaryMap::new();
        let vertices = self
            .vertices
            .iter()
            .enumerate()
            .map(|(i, (k, v))| {
                vertex_index.insert(k, i);
                VertexData {
                    name: v.name.clone(),
                    point: v.point,
                    tol: v.tol,
                }
            })
            .collect();
        let mut edge_index: SecondaryMap<EdgeKey, usize> = SecondaryMap::new();
        let mut edges = Vec::with_capacity(self.edges.len());
        for (i, (k, e)) in self.edges.iter().enumerate() {
            edge_index.insert(k, i);
            let index = |v: VertexKey| vertex_index.get(v).copied().ok_or(DANGLING);
            edges.push(EdgeData {
                name: e.name.clone(),
                start: index(e.start)?,
                end: index(e.end)?,
                curve: e.curve.clone(),
                range: e.range,
                degenerate: e.degenerate,
                tol: e.tol,
            });
        }
        let mut face_index: SecondaryMap<FaceKey, usize> = SecondaryMap::new();
        let mut faces = Vec::with_capacity(self.faces.len());
        for (i, (k, f)) in self.faces.iter().enumerate() {
            face_index.insert(k, i);
            let mut loops = Vec::with_capacity(f.loops.len());
            for &lk in &f.loops {
                let l = self.loops.get(lk).ok_or(DANGLING)?;
                let mut coedges = Vec::with_capacity(l.coedges.len());
                for &ck in &l.coedges {
                    let c = self.coedges.get(ck).ok_or(DANGLING)?;
                    coedges.push(CoedgeData {
                        edge: edge_index.get(c.edge).copied().ok_or(DANGLING)?,
                        reversed: c.reversed,
                        pcurve: c.pcurve.clone(),
                    });
                }
                loops.push(coedges);
            }
            faces.push(FaceData {
                name: f.name.clone(),
                surface: f.surface.clone(),
                reversed: f.reversed,
                loops,
            });
        }
        let mut shells = Vec::with_capacity(self.shells.len());
        for &sk in &self.shells {
            let s = self.shell_arena.get(sk).ok_or(DANGLING)?;
            let faces = s
                .faces
                .iter()
                .map(|&f| face_index.get(f).copied().ok_or(DANGLING))
                .collect::<Result<_, _>>()?;
            shells.push(ShellData {
                faces,
                closed: s.closed,
            });
        }
        Ok(BodyData {
            kind: self.kind,
            vertices,
            edges,
            faces,
            shells,
        })
    }

    /// Rebuilds and validates a body from its flat form.
    fn from_data(data: BodyData) -> Result<Self, KernelError> {
        let mut b = BodyBuilder::new();
        let vertices: Vec<VertexKey> = data
            .vertices
            .into_iter()
            .map(|v| b.add_vertex(v.point, v.tol, v.name))
            .collect();
        let vertex = |i: usize| vertices.get(i).copied().ok_or(OUT_OF_RANGE);
        let mut edges: Vec<EdgeKey> = Vec::with_capacity(data.edges.len());
        for e in data.edges {
            edges.push(b.insert_edge(Edge {
                curve: e.curve,
                range: e.range,
                start: vertex(e.start)?,
                end: vertex(e.end)?,
                degenerate: e.degenerate,
                tol: e.tol,
                name: e.name,
            })?);
        }
        let mut faces: Vec<FaceKey> = Vec::with_capacity(data.faces.len());
        let mut face_loops = Vec::with_capacity(data.faces.len());
        for f in data.faces {
            faces.push(b.add_face(f.surface, f.reversed, f.name));
            face_loops.push(f.loops);
        }
        for (&face, loops) in faces.iter().zip(face_loops) {
            for coedges in loops {
                let coedges = coedges
                    .into_iter()
                    .map(|c| {
                        let edge = edges.get(c.edge).copied().ok_or(OUT_OF_RANGE)?;
                        Ok((edge, c.reversed, c.pcurve))
                    })
                    .collect::<Result<Vec<_>, KernelError>>()?;
                b.add_loop(face, &coedges)?;
            }
        }
        for s in data.shells {
            let shell_faces = s
                .faces
                .iter()
                .map(|&i| faces.get(i).copied().ok_or(OUT_OF_RANGE))
                .collect::<Result<Vec<_>, _>>()?;
            b.add_shell(&shell_faces, s.closed)?;
        }
        b.finish(data.kind)
    }
}

impl Serialize for Body {
    /// Writes the flat form described under "Serialization" on [`Body`].
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.to_data()
            .map_err(S::Error::custom)?
            .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Body {
    /// Reads the flat form described under "Serialization" on [`Body`],
    /// rebuilds the body and validates it; an invalid body is an error.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let data = BodyData::deserialize(deserializer)?;
        Self::from_data(data).map_err(D::Error::custom)
    }
}

impl PartialEq for Body {
    /// Compares the flat serialized form: the same entities in the same
    /// construction order, with the same geometry, names and references.
    /// Keys are not compared, so a body equals its round trip through
    /// serialization. A body with dangling references equals nothing.
    fn eq(&self, other: &Self) -> bool {
        match (self.to_data(), other.to_data()) {
            (Ok(a), Ok(b)) => a == b,
            _ => false,
        }
    }
}
