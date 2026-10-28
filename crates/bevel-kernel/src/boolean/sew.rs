// SPDX-License-Identifier: GPL-3.0-or-later

//! Sewing the kept fragments into bodies, with names and history.
//!
//! Fragments that share pieces form shells. A shell made only of the
//! tool's turned-over faces, sharing nothing with the target's, is a void
//! (a tool subtracted from deep inside the target) and joins the shell
//! around it; every other shell is the outer boundary of its own body.
//!
//! Names: an input entity that is not split keeps its name. A split edge's
//! pieces are `Modified` fragments of it, in order along the edge; a split
//! face's fragments are `Modified` fragments in the order of their
//! interior points. Intersection edges are `Between` their two input
//! faces, numbered in the order of their midpoints, and new vertices are
//! numbered in the order of their positions.

use super::cut::Inputs;
use super::imprint::Fragment;
use super::pieces::{Origin, Piece, Vertices};
use crate::KernelError;
use crate::builder::BodyBuilder;
use crate::naming::{EntityKind, History, OpTag, Role, Source, TopoName};
use crate::topology::{Body, BodyKind, EdgeKey, FaceKey, VertexKey};
use bevel_math::Point3;
use std::collections::BTreeMap;
use std::sync::Arc;

/// A kept fragment and whether it is turned over (the tool's fragments in
/// a subtraction).
pub(crate) struct Kept<'f> {
    pub fragment: &'f Fragment,
    pub flip: bool,
    /// Its index among all fragments of its input face, for naming.
    pub rank: u32,
    /// How many fragments its input face split into.
    pub of: u32,
}

fn order(a: Point3, b: Point3) -> std::cmp::Ordering {
    a.x.total_cmp(&b.x)
        .then(a.y.total_cmp(&b.y))
        .then(a.z.total_cmp(&b.z))
}

/// Names for pieces and new vertices.
struct Names {
    pieces: Vec<TopoName>,
    vertices: Vec<TopoName>,
}

fn names(tag: OpTag, pieces: &[Piece], vertices: &Vertices) -> Names {
    // New vertices in the order of their positions.
    let mut fresh: Vec<usize> = (0..vertices.list.len())
        .filter(|&i| vertices.list[i].name.is_none())
        .collect();
    fresh.sort_by(|&a, &b| order(vertices.list[a].point, vertices.list[b].point));
    let mut vnames: Vec<Option<TopoName>> = vertices.list.iter().map(|v| v.name.clone()).collect();
    for (k, &i) in fresh.iter().enumerate() {
        vnames[i] = Some(TopoName::generated(
            tag,
            Source::Primitive,
            Role::Vertex(u32::try_from(k).unwrap_or(u32::MAX)),
        ));
    }
    // Intersection pieces between the same two faces, in midpoint order.
    let mut groups: BTreeMap<[TopoName; 2], Vec<usize>> = BTreeMap::new();
    for (i, p) in pieces.iter().enumerate() {
        if let Origin::Cut { faces } = &p.origin {
            let mut key = faces.clone();
            key.sort();
            groups.entry(key).or_default().push(i);
        }
    }
    let mut cut_index = vec![0u32; pieces.len()];
    for list in groups.values_mut() {
        list.sort_by(|&a, &b| order(pieces[a].midpoint(), pieces[b].midpoint()));
        for (k, &i) in list.iter().enumerate() {
            cut_index[i] = u32::try_from(k).unwrap_or(u32::MAX);
        }
    }
    let pnames = pieces
        .iter()
        .enumerate()
        .map(|(i, p)| match &p.origin {
            Origin::Edge {
                name, index, count, ..
            } => {
                if *count == 1 {
                    name.clone()
                } else {
                    TopoName::modified(tag, name.clone(), *index)
                }
            }
            Origin::Cut { faces } => {
                TopoName::between(tag, faces[0].clone(), faces[1].clone(), cut_index[i])
            }
        })
        .collect();
    Names {
        pieces: pnames,
        // Every vertex has a name by now; the fallback is never used.
        vertices: vnames
            .into_iter()
            .map(|n| {
                n.unwrap_or(TopoName::generated(
                    tag,
                    Source::Primitive,
                    Role::Vertex(u32::MAX),
                ))
            })
            .collect(),
    }
}

/// The key of the result edge a half-edge of fragment `k` lies on: the
/// piece, and for a piece more than two kept faces use (bodies touching
/// along an edge), also the input body, so each body keeps its own edge
/// and the bodies stay apart.
type EdgeId = (usize, Option<usize>);

fn edge_id(kept: &[Kept<'_>], crowded: &BTreeMap<usize, usize>, k: usize, piece: usize) -> EdgeId {
    let side = (crowded.get(&piece).copied().unwrap_or(0) > 2).then_some(kept[k].fragment.face.0);
    (piece, side)
}

/// How many kept coedges use each piece.
fn uses(kept: &[Kept<'_>]) -> BTreeMap<usize, usize> {
    let mut n = BTreeMap::new();
    for f in kept {
        for h in f.fragment.cycles.iter().flatten() {
            *n.entry(h.piece).or_insert(0) += 1;
        }
    }
    n
}

/// Groups fragments into shells: connected through shared edges.
fn shells(kept: &[Kept<'_>], crowded: &BTreeMap<usize, usize>) -> Vec<Vec<usize>> {
    fn find(p: &mut [usize], i: usize) -> usize {
        let mut i = i;
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..kept.len()).collect();
    let mut owner: BTreeMap<EdgeId, usize> = BTreeMap::new();
    for (k, f) in kept.iter().enumerate() {
        for h in f.fragment.cycles.iter().flatten() {
            let id = edge_id(kept, crowded, k, h.piece);
            match owner.get(&id) {
                Some(&o) => {
                    let (a, b) = (find(&mut parent, o), find(&mut parent, k));
                    if a != b {
                        parent[a.max(b)] = a.min(b);
                    }
                }
                None => {
                    owner.insert(id, k);
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for k in 0..kept.len() {
        let r = find(&mut parent, k);
        groups.entry(r).or_default().push(k);
    }
    groups.into_values().collect()
}

/// Builds the result bodies from the kept fragments.
///
/// # Errors
///
/// [`KernelError::InvalidBody`] (or another builder error) if the
/// fragments do not close up into valid solids.
pub(crate) fn sew(
    inputs: &Inputs<'_>,
    pieces: &[Piece],
    vertices: &Vertices,
    kept: &[Kept<'_>],
    tag: OpTag,
    history: &mut History,
) -> Result<Vec<Arc<Body>>, KernelError> {
    let names = names(tag, pieces, vertices);
    let crowded = uses(kept);
    // Every edge of a closed shell has two coedges: a piece kept by one
    // fragment means the cut or the classification went wrong.
    // (A pole's degenerate edge has one coedge by definition.)
    if let Some((&pi, _)) = crowded
        .iter()
        .find(|(pi, n)| **n == 1 && !pieces[**pi].degenerate)
    {
        let p = &pieces[pi];
        let what = match &p.origin {
            Origin::Edge { body, name, .. } => {
                format!(
                    "{} edge {name}",
                    if *body == 0 { "a target" } else { "a tool" }
                )
            }
            Origin::Cut { faces } => format!("the intersection of {} and {}", faces[0], faces[1]),
        };
        let (a, b) = (vertices.list[p.start].point, vertices.list[p.end].point);
        return Err(KernelError::Boolean {
            stage: super::BooleanStage::Sew,
            message: format!("{what} from {a:?} to {b:?} has a face on only one side"),
        });
    }
    let groups = shells(kept, &crowded);
    // Voids: shells of turned-over tool faces only. They join the first
    // outer shell (a subtraction that both splits the target and leaves a
    // void inside one of the pieces is not told apart yet).
    let is_void = |g: &Vec<usize>| g.iter().all(|&k| kept[k].flip);
    let outers: Vec<&Vec<usize>> = groups.iter().filter(|g| !is_void(g)).collect();
    let mut voids_of: Vec<Vec<&Vec<usize>>> = vec![Vec::new(); outers.len()];
    for g in groups.iter().filter(|g| is_void(g)) {
        match voids_of.first_mut() {
            Some(v) => v.push(g),
            None => {
                return Err(KernelError::InvalidInput {
                    what: "boolean",
                    reason: "a void lies outside every result body",
                });
            }
        }
    }
    let mut bodies = Vec::new();
    for (i, outer) in outers.iter().enumerate() {
        let mut b = BodyBuilder::new();
        let mut vkeys: BTreeMap<usize, VertexKey> = BTreeMap::new();
        let mut ekeys: BTreeMap<EdgeId, EdgeKey> = BTreeMap::new();
        let mut shell_faces: Vec<Vec<FaceKey>> = Vec::new();
        for group in std::iter::once(*outer).chain(voids_of[i].iter().copied()) {
            let mut faces = Vec::new();
            for &k in group {
                faces.push(add_fragment(
                    inputs,
                    pieces,
                    vertices,
                    &names,
                    &kept[k],
                    tag,
                    &mut b,
                    &mut vkeys,
                    &mut ekeys,
                    history,
                    &|piece| edge_id(kept, &crowded, k, piece),
                )?);
            }
            shell_faces.push(faces);
        }
        for faces in &shell_faces {
            b.add_shell(faces, true)?;
        }
        bodies.push(Arc::new(b.finish(BodyKind::Solid)?));
    }
    Ok(bodies)
}

#[expect(clippy::too_many_arguments, reason = "the state of one sewing pass")]
fn add_fragment(
    inputs: &Inputs<'_>,
    pieces: &[Piece],
    vertices: &Vertices,
    names: &Names,
    kept: &Kept<'_>,
    tag: OpTag,
    b: &mut BodyBuilder,
    vkeys: &mut BTreeMap<usize, VertexKey>,
    ekeys: &mut BTreeMap<EdgeId, EdgeKey>,
    history: &mut History,
    id_of: &dyn Fn(usize) -> EdgeId,
) -> Result<FaceKey, KernelError> {
    let frag = kept.fragment;
    let face = inputs.face(frag.face)?;
    let name = if kept.of == 1 {
        face.name.clone()
    } else {
        let n = TopoName::modified(tag, face.name.clone(), kept.rank);
        history.record(n.clone(), EntityKind::Face, vec![face.name.clone()])?;
        n
    };
    let fk = b.add_face(face.surface.clone(), face.reversed ^ kept.flip, name);
    for cycle in &frag.cycles {
        let mut coedges = Vec::with_capacity(cycle.len());
        for h in cycle {
            let piece = &pieces[h.piece];
            let id = id_of(h.piece);
            let ek = if let Some(&k) = ekeys.get(&id) {
                k
            } else {
                let mut vkey =
                    |vid: usize, b: &mut BodyBuilder| -> Result<VertexKey, KernelError> {
                        if let Some(&k) = vkeys.get(&vid) {
                            return Ok(k);
                        }
                        let v = &vertices.list[vid];
                        let n = names.vertices[vid].clone();
                        if v.name.is_none() {
                            history.record(n.clone(), EntityKind::Vertex, Vec::new())?;
                        }
                        let k = b.add_vertex(v.point, inputs.tol, n);
                        vkeys.insert(vid, k);
                        Ok(k)
                    };
                let s = vkey(piece.start, b)?;
                let name = match id.1 {
                    // The second body's copy of a shared edge.
                    Some(1) => TopoName::modified(tag, names.pieces[h.piece].clone(), 1),
                    _ => names.pieces[h.piece].clone(),
                };
                let k = if piece.degenerate {
                    b.add_degenerate_edge(s, piece.range, piece.tol, name.clone())?
                } else {
                    let e = vkey(piece.end, b)?;
                    b.add_edge(
                        piece.curve.clone(),
                        piece.range,
                        s,
                        e,
                        piece.tol,
                        name.clone(),
                    )?
                };
                let from = match &piece.origin {
                    Origin::Edge {
                        name: base, count, ..
                    } if *count > 1 => Some(vec![base.clone()]),
                    Origin::Cut { faces } => Some(faces.to_vec()),
                    Origin::Edge { .. } => None,
                };
                if let Some(from) = from {
                    history.record(name, EntityKind::Edge, from)?;
                }
                ekeys.insert(id, k);
                k
            };
            coedges.push((
                ek,
                !h.forward ^ kept.flip,
                piece.uses[h.use_].pcurve.clone(),
            ));
        }
        if kept.flip {
            coedges.reverse();
        }
        b.add_loop(fk, &coedges)?;
    }
    Ok(fk)
}
