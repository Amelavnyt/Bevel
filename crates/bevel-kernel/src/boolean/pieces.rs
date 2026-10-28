// SPDX-License-Identifier: GPL-3.0-or-later

//! The pieces the result is built from: vertices shared by position, and
//! edge pieces (the parts of the inputs' edges between split points, and
//! the parts of intersection curves inside both faces), each with its
//! pcurve on every face that uses it.

use crate::naming::TopoName;
use crate::topology::{CoedgeKey, EdgeKey, FaceKey};
use bevel_geom::{Curve2, Curve3, CurveGeom};
use bevel_math::{Interval, Point3};

/// A face of one of the inputs: `(0, key)` for the target, `(1, key)` for
/// the tool.
pub(crate) type FaceRef = (usize, FaceKey);

/// A vertex of the result.
#[derive(Clone, Debug)]
pub(crate) struct Vertex {
    pub point: Point3,
    /// The input vertex it is, if any.
    pub name: Option<TopoName>,
}

/// Vertices merged by position.
#[derive(Clone, Debug, Default)]
pub(crate) struct Vertices {
    pub list: Vec<Vertex>,
    tol: f64,
}

impl Vertices {
    pub(crate) fn new(tol: f64) -> Self {
        Self {
            list: Vec::new(),
            tol,
        }
    }

    /// The vertex at `p`, created if no vertex lies within the tolerance;
    /// `name` is kept for input vertices.
    pub(crate) fn at(&mut self, p: Point3, name: Option<&TopoName>) -> usize {
        if let Some(i) = self
            .list
            .iter()
            .position(|v| (v.point - p).length() <= self.tol)
        {
            if let (Some(n), None) = (name, &self.list[i].name) {
                self.list[i].name = Some(n.clone());
            }
            return i;
        }
        self.list.push(Vertex {
            point: p,
            name: name.cloned(),
        });
        self.list.len() - 1
    }
}

/// Where a piece comes from.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Origin {
    /// Part `index` (of `count`) of an input edge.
    Edge {
        body: usize,
        edge: EdgeKey,
        name: TopoName,
        index: u32,
        count: u32,
    },
    /// Part of the intersection of two faces.
    Cut { faces: [TopoName; 2] },
}

/// A pcurve of a piece on one face. Input edges carry one per coedge (a
/// seam has two on the same face, with the travel direction of each);
/// intersection pieces one per face, used both ways.
#[derive(Clone, Debug)]
pub(crate) struct Use {
    pub face: FaceRef,
    pub pcurve: Curve2,
    /// For input edges: the coedge.
    pub coedge: Option<CoedgeKey>,
    /// Whether the piece runs the same way as the coedge's own edge (not
    /// so after merging with a piece of the other input running the other
    /// way).
    pub along: bool,
    /// Where the piece starts along the coedge's own edge, for ordering
    /// the pieces of a coedge.
    pub order: f64,
}

/// An edge of the result, before sewing.
#[derive(Clone, Debug)]
pub(crate) struct Piece {
    pub curve: Curve3,
    pub range: Interval,
    pub start: usize,
    pub end: usize,
    pub degenerate: bool,
    pub tol: f64,
    pub origin: Origin,
    pub uses: Vec<Use>,
}

impl Piece {
    /// The point halfway along the parameter.
    pub(crate) fn midpoint(&self) -> Point3 {
        self.curve.point(self.range.mid())
    }
}
