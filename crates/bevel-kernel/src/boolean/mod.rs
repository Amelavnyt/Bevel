// SPDX-License-Identifier: GPL-3.0-or-later

//! Boolean operations: union, subtraction and intersection of solids.
//!
//! [`boolean`] runs the pipeline of `docs/KERNEL.md`:
//!
//! 1. **Cut** ([`cut`]): every pair of faces whose boxes overlap is
//!    intersected; intersection curves are split where they cross the
//!    faces' edges and kept where they run inside both faces; the edges are
//!    split there too. Coincident faces imprint their edges on each other.
//! 2. **Imprint** ([`imprint`]): each face is split into fragments along
//!    the pieces crossing it.
//! 3. **Classify** ([`classify`]): a point inside each fragment is tested
//!    against the other solid: inside, outside or on its boundary (then
//!    with the same or the opposite orientation).
//! 4. **Select**: union keeps what lies outside the other solid, intersection
//!    what lies inside, subtraction the target outside the tool and the tool
//!    inside the target, turned over. Of two coincident fragments with the
//!    same orientation union and intersection keep one; with opposite
//!    orientation subtraction keeps the target's.
//! 5. **Sew** ([`sew`]): the kept fragments become faces of new bodies,
//!    validated by the builder.
//!
//! Each failure names its stage, and nothing invalid is ever returned.

mod classify;
mod cut;
mod domain;
mod imprint;
mod pieces;
mod sew;

use crate::naming::{History, OpContext, OpOutput, OpTag};
use crate::topology::Body;
use crate::{KernelError, TessellationOptions};
use bevel_geom::SurfaceGeom;
use classify::{Containment, Solid};
use std::collections::BTreeMap;

/// Which boolean operation to run.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub enum BooleanOp {
    /// Everything in either body.
    Union,
    /// The target without the tool.
    Subtract,
    /// What both bodies share.
    Intersect,
}

/// Where a boolean operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanStage {
    /// Preparing the inputs (face domains).
    Prepare,
    /// Intersecting faces.
    Cut,
    /// Splitting faces into fragments.
    Imprint,
    /// Telling which fragments lie inside the other body.
    Classify,
    /// Sewing the result.
    Sew,
}

impl std::fmt::Display for BooleanStage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Prepare => "preparing the inputs",
            Self::Cut => "intersecting faces",
            Self::Imprint => "splitting faces",
            Self::Classify => "classifying fragments",
            Self::Sew => "sewing the result",
        })
    }
}

fn at(stage: BooleanStage) -> impl Fn(KernelError) -> KernelError {
    move |e| match e {
        e @ (KernelError::Boolean { .. } | KernelError::Cancelled) => e,
        e => KernelError::Boolean {
            stage,
            message: e.to_string(),
        },
    }
}

/// Combines `target` and `tool` by `op`. The result may be several bodies
/// (a subtraction can cut the target in two) or none (an intersection of
/// bodies that do not meet).
///
/// # Errors
///
/// [`KernelError::Boolean`] with the stage that failed, or
/// [`KernelError::Cancelled`].
pub fn boolean(
    target: &Body,
    tool: &Body,
    op: BooleanOp,
    tag: OpTag,
    cx: &mut OpContext,
) -> Result<OpOutput, KernelError> {
    let tol = cx.tolerance;
    let bounds = target.bounds().union(tool.bounds());
    let options = TessellationOptions::export(&bounds);
    let solids = [
        Solid::new(target, &options, tol).map_err(at(BooleanStage::Prepare))?,
        Solid::new(tool, &options, tol).map_err(at(BooleanStage::Prepare))?,
    ];
    let inputs = cut::Inputs {
        bodies: [target, tool],
        solids,
        tol,
    };
    cx.check_cancelled()?;
    let cuts = cut::cut(&inputs).map_err(at(BooleanStage::Cut))?;
    let mut vertices = pieces::Vertices::new(tol);
    let pieces = cut::pieces(&inputs, &cuts, &mut vertices).map_err(at(BooleanStage::Cut))?;
    cx.check_cancelled()?;
    let fragments = imprint::fragments(&inputs, &pieces).map_err(at(BooleanStage::Imprint))?;
    cx.check_cancelled()?;

    // Rank the fragments of each input face by their interior points.
    let mut per_face: BTreeMap<pieces::FaceRef, Vec<usize>> = BTreeMap::new();
    for (i, f) in fragments.iter().enumerate() {
        per_face.entry(f.face).or_default().push(i);
    }
    let mut rank = vec![(0u32, 1u32); fragments.len()];
    for list in per_face.values_mut() {
        list.sort_by(|&a, &b| {
            let (p, q) = (fragments[a].point, fragments[b].point);
            p.x.total_cmp(&q.x)
                .then(p.y.total_cmp(&q.y))
                .then(p.z.total_cmp(&q.z))
        });
        let of = u32::try_from(list.len()).unwrap_or(u32::MAX);
        for (r, &i) in list.iter().enumerate() {
            rank[i] = (u32::try_from(r).unwrap_or(u32::MAX), of);
        }
    }

    let mut kept = Vec::new();
    for (i, f) in fragments.iter().enumerate() {
        let side = f.face.0;
        let other = &inputs.solids[1 - side];
        let containment = other
            .classify(f.point)
            .map_err(at(BooleanStage::Classify))?;
        let same_sense = if containment == Containment::On {
            Some(same_orientation(&inputs, f, other).map_err(at(BooleanStage::Classify))?)
        } else {
            None
        };
        let keep = select(op, side, containment, same_sense);
        if let Some(flip) = keep {
            kept.push(sew::Kept {
                fragment: f,
                flip,
                rank: rank[i].0,
                of: rank[i].1,
            });
        }
    }
    cx.check_cancelled()?;
    let mut history = History::new();
    let bodies = sew::sew(&inputs, &pieces, &vertices, &kept, tag, &mut history)
        .map_err(at(BooleanStage::Sew))?;
    Ok(OpOutput { bodies, history })
}

/// Whether to keep a fragment of input `side` (0 target, 1 tool), and if
/// so whether to turn it over.
fn select(op: BooleanOp, side: usize, c: Containment, same_sense: Option<bool>) -> Option<bool> {
    use BooleanOp::{Intersect, Subtract, Union};
    use Containment::{In, On, Out};
    match (op, c) {
        (Union, Out) | (Intersect, In) => Some(false),
        (Union | Intersect, On) => (same_sense == Some(true) && side == 0).then_some(false),
        (Subtract, Out) if side == 0 => Some(false),
        (Subtract, In) if side == 1 => Some(true),
        (Subtract, On) => (same_sense == Some(false) && side == 0).then_some(false),
        _ => None,
    }
}

/// Whether a fragment lying on the other solid's boundary faces the same
/// way as the face it lies on.
fn same_orientation(
    inputs: &cut::Inputs<'_>,
    f: &imprint::Fragment,
    other: &Solid<'_>,
) -> Result<bool, KernelError> {
    let face = inputs.face(f.face)?;
    let Some((ok, ouv)) = other.on_face(f.point)? else {
        return Ok(true);
    };
    let of = inputs.face((1 - f.face.0, ok))?;
    let n = face.surface.normal(f.uv.x, f.uv.y);
    let m = of.surface.normal(ouv.x, ouv.y);
    Ok(match (n, m) {
        (Some(n), Some(m)) => {
            let n = if face.reversed { -n } else { n };
            let m = if of.reversed { -m } else { m };
            n.dot(m) > 0.0
        }
        _ => true,
    })
}
