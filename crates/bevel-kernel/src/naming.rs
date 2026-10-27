// SPDX-License-Identifier: GPL-3.0-or-later

//! Persistent topological naming, operation history and the context every
//! operation runs in.
//!
//! # Names
//!
//! Every vertex, edge and face carries a [`TopoName`] derived from how it
//! was created, never from where it happens to sit in an arena:
//!
//! - [`TopoName::Generated`]: created by operation `op` from a [`Source`]
//!   (a primitive, a profile segment, an input entity, a tool face) in a
//!   [`Role`] (top, bottom, side `i`, seam `i`, ...).
//! - [`TopoName::Modified`]: fragment `fragment` of an older entity `base`
//!   that operation `op` split.
//! - [`TopoName::Merged`]: the merge of several older entities (for
//!   example coplanar faces after a boolean).
//! - [`TopoName::Between`]: an edge or vertex named by the two faces it
//!   separates, with an index for when the same pair meets more than once.
//!
//! Names are plain values: they compare, order and hash structurally, so
//! two names are equal exactly when they describe the same derivation.
//! The total order is the derived one (variant first, then fields in
//! declaration order), which is deterministic and stable across runs and
//! platforms. Names are canonical: [`TopoName::merged`] sorts and
//! de-duplicates its parts and [`TopoName::between`] sorts its two faces,
//! and deserialization refuses names that are not in this form, so equal
//! derivations always produce equal names.
//!
//! Within one body every name is unique (the validator reports duplicates);
//! the kind of entity (vertex, edge or face) is not part of the name, so a
//! face and an edge never share one either.
//!
//! # Roles used by the primitives
//!
//! | Primitive | Faces | Edges | Vertices |
//! |---|---|---|---|
//! | box | `Bottom` `Top` `Front` `Back` `Left` `Right` | `Edge(0..12)` | `Vertex(0..8)` |
//! | cylinder | `Bottom` `Top` `Side(0)` | `Edge(0)` bottom circle, `Edge(1)` top circle, `Seam(0)` | `Vertex(0)` bottom, `Vertex(1)` top |
//! | cone | `Bottom` `Top` (no cap at an apex) `Side(0)` | `Edge(0)`, `Edge(1)` or `Apex`, `Seam(0)` | `Vertex(0)` bottom, `Vertex(1)` top |
//! | sphere | `Side(0)` | `Seam(0)`, `Pole(0)` south, `Pole(1)` north | `Vertex(0)` south, `Vertex(1)` north |
//! | torus | `Side(0)` | `Seam(0)` (the tube circle at `u = 0`), `Seam(1)` (the outer equator) | `Vertex(0)` |
//!
//! The numbering of box edges and vertices is documented in
//! [`crate::primitives`]; `docs/KERNEL.md` ("Topology conventions") has the
//! full table.
//!
//! # History
//!
//! [`History`] maps every entity an operation created to the names of the
//! input entities it was generated or modified from; primitives generate
//! everything from nothing. [`OpOutput`] carries the result bodies and the
//! history; [`OpContext`] carries the tolerance and the cancellation token.

use std::collections::BTreeSet;
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use bevel_math::tol;
use serde::{Deserialize, Serialize};

use crate::KernelError;
use crate::topology::Body;

/// The operation that created an entity: an opaque number chosen by the
/// caller (`bevel-model` passes the feature id), so the kernel can name
/// what it creates without knowing about features.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[serde(transparent)]
pub struct OpTag(pub u64);

impl fmt::Display for OpTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// What a [`TopoName::Generated`] entity was generated from.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Source {
    /// A primitive (box, cylinder, ...) created it from its parameters.
    Primitive,
    /// A segment of a sketch profile, by the sketch's opaque segment id
    /// (for example the side face an extrude sweeps from that segment).
    ProfileSegment(u64),
    /// An entity of an input body (for example the cap an extrude makes
    /// from a planar face).
    Entity(Arc<TopoName>),
    /// A face of a boolean tool body.
    ToolFace(Arc<TopoName>),
}

/// The part an entity plays in what generated it.
///
/// The primitives use the roles in the table of the
/// [module documentation](self); operations add their own uses (`Start`
/// and `End` caps of an extrude, `Side(i)` for the face swept from profile
/// segment `i`, and so on).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Role {
    /// The face (or cap) at the top, along `+z` of the placement frame.
    Top,
    /// The face (or cap) at the bottom, along `-z` of the placement frame.
    Bottom,
    /// A side face; the index tells several apart (`Side(0)` for the one
    /// curved face of a cylinder, cone, sphere or torus).
    Side(u32),
    /// The face along `-y` of the placement frame.
    Front,
    /// The face along `+y` of the placement frame.
    Back,
    /// The face along `-x` of the placement frame.
    Left,
    /// The face along `+x` of the placement frame.
    Right,
    /// The start cap of a sweep.
    Start,
    /// The end cap of a sweep.
    End,
    /// A seam edge of a periodic face; the index tells several apart.
    Seam(u32),
    /// The degenerate edge at a pole of a sphere (`0` south, `1` north).
    Pole(u32),
    /// The degenerate edge at the apex of a cone.
    Apex,
    /// An edge, by its index in the generator's documented numbering.
    Edge(u32),
    /// A vertex, by its index in the generator's documented numbering.
    Vertex(u32),
}

/// The persistent name of a vertex, edge or face (see the
/// [module documentation](self)).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "TopoNameRaw")]
pub enum TopoName {
    /// Created by `op` from `source` in the given `role`.
    Generated {
        /// The creating operation.
        op: OpTag,
        /// What it was generated from.
        source: Source,
        /// The part it plays.
        role: Role,
    },
    /// Fragment `fragment` of the older entity `base`, split by `op`.
    /// Fragment indices follow a canonical geometric order chosen by the
    /// operation, so they are stable across rebuilds.
    Modified {
        /// The splitting operation.
        op: OpTag,
        /// The name of the entity that was split.
        base: Arc<TopoName>,
        /// Which piece this is.
        fragment: u32,
    },
    /// The merge of several older entities by `op`; `parts` is sorted and
    /// free of duplicates.
    Merged {
        /// The merging operation.
        op: OpTag,
        /// The merged entities' names, sorted.
        parts: Vec<Arc<TopoName>>,
    },
    /// An edge or vertex named by the two faces it lies between; `faces`
    /// is sorted, and `index` tells apart several such entities between
    /// the same pair of faces.
    Between {
        /// The creating operation.
        op: OpTag,
        /// The two faces, sorted.
        faces: [Arc<TopoName>; 2],
        /// Which of the entities between these faces this is.
        index: u32,
    },
}

impl TopoName {
    /// A [`TopoName::Generated`] name.
    pub fn generated(op: OpTag, source: Source, role: Role) -> Self {
        Self::Generated { op, source, role }
    }

    /// A [`TopoName::Modified`] name.
    pub fn modified(op: OpTag, base: TopoName, fragment: u32) -> Self {
        Self::Modified {
            op,
            base: Arc::new(base),
            fragment,
        }
    }

    /// A [`TopoName::Merged`] name with its parts sorted and de-duplicated.
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidInput`] if `parts` is empty.
    pub fn merged(
        op: OpTag,
        parts: impl IntoIterator<Item = TopoName>,
    ) -> Result<Self, KernelError> {
        let parts: BTreeSet<TopoName> = parts.into_iter().collect();
        if parts.is_empty() {
            return Err(KernelError::InvalidInput {
                what: "merged name",
                reason: "a merge needs at least one part",
            });
        }
        Ok(Self::Merged {
            op,
            parts: parts.into_iter().map(Arc::new).collect(),
        })
    }

    /// A [`TopoName::Between`] name with its two faces sorted.
    pub fn between(op: OpTag, a: TopoName, b: TopoName, index: u32) -> Self {
        let (a, b) = if b < a { (b, a) } else { (a, b) };
        Self::Between {
            op,
            faces: [Arc::new(a), Arc::new(b)],
            index,
        }
    }

    /// The operation that created this name.
    pub fn op(&self) -> OpTag {
        match self {
            Self::Generated { op, .. }
            | Self::Modified { op, .. }
            | Self::Merged { op, .. }
            | Self::Between { op, .. } => *op,
        }
    }

    /// Every operation that appears anywhere in this name's lineage,
    /// itself included (what a timeline needs to know which features a
    /// reference depends on).
    pub fn ops(&self) -> BTreeSet<OpTag> {
        let mut out = BTreeSet::new();
        self.collect_ops(&mut out);
        out
    }

    fn collect_ops(&self, out: &mut BTreeSet<OpTag>) {
        out.insert(self.op());
        match self {
            Self::Generated { source, .. } => match source {
                Source::Entity(n) | Source::ToolFace(n) => n.collect_ops(out),
                Source::Primitive | Source::ProfileSegment(_) => {}
            },
            Self::Modified { base, .. } => base.collect_ops(out),
            Self::Merged { parts, .. } => parts.iter().for_each(|p| p.collect_ops(out)),
            Self::Between { faces, .. } => faces.iter().for_each(|p| p.collect_ops(out)),
        }
    }
}

impl fmt::Display for TopoName {
    /// A compact, human-readable form for logs and messages, for example
    /// `gen(#7, primitive, Bottom)`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Generated { op, source, role } => {
                write!(f, "gen({op}, ")?;
                match source {
                    Source::Primitive => write!(f, "primitive")?,
                    Source::ProfileSegment(s) => write!(f, "segment {s}")?,
                    Source::Entity(n) => write!(f, "from {n}")?,
                    Source::ToolFace(n) => write!(f, "tool {n}")?,
                }
                write!(f, ", {role:?})")
            }
            Self::Modified { op, base, fragment } => write!(f, "mod({op}, {base}, {fragment})"),
            Self::Merged { op, parts } => {
                write!(f, "merge({op}")?;
                for p in parts {
                    write!(f, ", {p}")?;
                }
                write!(f, ")")
            }
            Self::Between { op, faces, index } => {
                write!(f, "between({op}, {}, {}, {index})", faces[0], faces[1])
            }
        }
    }
}

/// The serialized form of a [`TopoName`], checked for canonical form.
#[derive(Deserialize)]
enum TopoNameRaw {
    Generated {
        op: OpTag,
        source: Source,
        role: Role,
    },
    Modified {
        op: OpTag,
        base: Arc<TopoName>,
        fragment: u32,
    },
    Merged {
        op: OpTag,
        parts: Vec<Arc<TopoName>>,
    },
    Between {
        op: OpTag,
        faces: [Arc<TopoName>; 2],
        index: u32,
    },
}

impl TryFrom<TopoNameRaw> for TopoName {
    type Error = KernelError;

    fn try_from(raw: TopoNameRaw) -> Result<Self, KernelError> {
        Ok(match raw {
            TopoNameRaw::Generated { op, source, role } => Self::Generated { op, source, role },
            TopoNameRaw::Modified { op, base, fragment } => Self::Modified { op, base, fragment },
            TopoNameRaw::Merged { op, parts } => {
                if parts.is_empty() || parts.windows(2).any(|w| w[1] <= w[0]) {
                    return Err(KernelError::InvalidInput {
                        what: "merged name",
                        reason: "the parts must be non-empty, sorted and distinct",
                    });
                }
                Self::Merged { op, parts }
            }
            TopoNameRaw::Between { op, faces, index } => {
                if faces[1] < faces[0] {
                    return Err(KernelError::InvalidInput {
                        what: "between name",
                        reason: "the two faces must be sorted",
                    });
                }
                Self::Between { op, faces, index }
            }
        })
    }
}

/// The kind of a named entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum EntityKind {
    /// A vertex.
    Vertex,
    /// An edge.
    Edge,
    /// A face.
    Face,
}

impl fmt::Display for EntityKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Vertex => "vertex",
            Self::Edge => "edge",
            Self::Face => "face",
        })
    }
}

/// What one created entity derives from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoryEntry {
    /// The created entity's name.
    pub name: TopoName,
    /// Whether it is a vertex, an edge or a face.
    pub kind: EntityKind,
    /// The names of the input entities it was generated or modified from;
    /// empty when it was generated from nothing (primitives).
    pub from: Vec<TopoName>,
}

/// The history of one operation: for every entity it created, the input
/// entities it came from.
///
/// Entries are kept sorted by name (one entry per name), so iteration and
/// serialization are deterministic. Serialized as the sorted list of
/// entries; deserialization refuses unsorted or duplicate names.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "Vec<HistoryEntry>", into = "Vec<HistoryEntry>")]
pub struct History {
    /// Invariant: sorted by name, names distinct.
    entries: Vec<HistoryEntry>,
}

impl History {
    /// An empty history.
    pub fn new() -> Self {
        Self::default()
    }

    /// The history of an operation that generated every vertex, edge and
    /// face of `body` from nothing, as the primitives do.
    ///
    /// # Errors
    ///
    /// [`KernelError::DuplicateHistory`] if two entities share a name.
    pub fn generated(body: &Body) -> Result<Self, KernelError> {
        let mut h = Self::new();
        for (_, v) in body.vertices() {
            h.record(v.name.clone(), EntityKind::Vertex, Vec::new())?;
        }
        for (_, e) in body.edges() {
            h.record(e.name.clone(), EntityKind::Edge, Vec::new())?;
        }
        for (_, f) in body.faces() {
            h.record(f.name.clone(), EntityKind::Face, Vec::new())?;
        }
        Ok(h)
    }

    /// Records that the entity `name` of kind `kind` was created from the
    /// entities named in `from`.
    ///
    /// # Errors
    ///
    /// [`KernelError::DuplicateHistory`] if `name` is already recorded.
    pub fn record(
        &mut self,
        name: TopoName,
        kind: EntityKind,
        from: Vec<TopoName>,
    ) -> Result<(), KernelError> {
        match self.entries.binary_search_by(|e| e.name.cmp(&name)) {
            Ok(_) => Err(KernelError::DuplicateHistory(Box::new(name))),
            Err(at) => {
                self.entries.insert(at, HistoryEntry { name, kind, from });
                Ok(())
            }
        }
    }

    /// The entry for `name`, if the operation created it.
    pub fn get(&self, name: &TopoName) -> Option<&HistoryEntry> {
        self.entries
            .binary_search_by(|e| e.name.cmp(name))
            .ok()
            .and_then(|i| self.entries.get(i))
    }

    /// The entries in name order.
    pub fn iter(&self) -> impl Iterator<Item = &HistoryEntry> {
        self.entries.iter()
    }

    /// The number of recorded entities.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns `true` if nothing is recorded.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl TryFrom<Vec<HistoryEntry>> for History {
    type Error = KernelError;

    fn try_from(entries: Vec<HistoryEntry>) -> Result<Self, KernelError> {
        if entries.windows(2).any(|w| w[1].name <= w[0].name) {
            return Err(KernelError::InvalidInput {
                what: "history",
                reason: "the entries must be sorted by name, with distinct names",
            });
        }
        Ok(Self { entries })
    }
}

impl From<History> for Vec<HistoryEntry> {
    fn from(h: History) -> Self {
        h.entries
    }
}

/// What an operation returns: the result bodies and the history of what
/// it created.
#[derive(Clone, Debug)]
pub struct OpOutput {
    /// The result bodies, immutable and shared.
    pub bodies: Vec<Arc<Body>>,
    /// Where every created entity came from.
    pub history: History,
}

/// The context an operation runs in.
#[derive(Clone, Debug)]
pub struct OpContext {
    /// The modelling tolerance in millimetres, given to every vertex and
    /// edge an operation creates. It must be finite, at least
    /// [`tol::LINEAR`] and at most [`OpContext::MAX_TOLERANCE`].
    pub tolerance: f64,
    /// Set to `true` from another thread to ask the operation to stop;
    /// long loops call [`OpContext::check_cancelled`].
    pub cancel: Option<Arc<AtomicBool>>,
}

impl Default for OpContext {
    fn default() -> Self {
        Self {
            tolerance: tol::LINEAR,
            cancel: None,
        }
    }
}

impl OpContext {
    /// The largest tolerance an operation accepts, 0.1 mm.
    pub const MAX_TOLERANCE: f64 = 0.1;

    /// A context with the default tolerance ([`tol::LINEAR`]) and no
    /// cancellation token.
    pub fn new() -> Self {
        Self::default()
    }

    /// A context with the default tolerance that watches `token`.
    pub fn with_cancel(token: Arc<AtomicBool>) -> Self {
        Self {
            cancel: Some(token),
            ..Self::default()
        }
    }

    /// `Ok` unless the cancellation token is set.
    ///
    /// # Errors
    ///
    /// [`KernelError::Cancelled`] once the token is set.
    pub fn check_cancelled(&self) -> Result<(), KernelError> {
        match &self.cancel {
            Some(token) if token.load(Ordering::Relaxed) => Err(KernelError::Cancelled),
            _ => Ok(()),
        }
    }

    /// The tolerance, checked to be finite and within
    /// `[tol::LINEAR, OpContext::MAX_TOLERANCE]`.
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidParameter`] otherwise.
    pub fn checked_tolerance(&self) -> Result<f64, KernelError> {
        let t = self.tolerance;
        if t.is_finite() && (tol::LINEAR..=Self::MAX_TOLERANCE).contains(&t) {
            Ok(t)
        } else {
            Err(KernelError::InvalidParameter {
                name: "tolerance",
                value: t,
                reason: "must be finite, at least 1e-6 mm and at most 0.1 mm",
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g(op: u64, role: Role) -> TopoName {
        TopoName::generated(OpTag(op), Source::Primitive, role)
    }

    #[test]
    fn names_order_by_variant_then_fields() {
        let mut names = [
            TopoName::between(OpTag(1), g(1, Role::Top), g(1, Role::Bottom), 0),
            g(2, Role::Top),
            g(1, Role::Side(3)),
            g(1, Role::Side(1)),
            TopoName::modified(OpTag(1), g(1, Role::Top), 0),
        ];
        names.sort();
        assert_eq!(names[0], g(1, Role::Side(1)));
        assert_eq!(names[1], g(1, Role::Side(3)));
        assert_eq!(names[2], g(2, Role::Top));
        assert!(matches!(names[3], TopoName::Modified { .. }));
        assert!(matches!(names[4], TopoName::Between { .. }));
    }

    #[test]
    fn constructors_are_canonical() {
        let a = TopoName::between(OpTag(1), g(1, Role::Top), g(1, Role::Bottom), 0);
        let b = TopoName::between(OpTag(1), g(1, Role::Bottom), g(1, Role::Top), 0);
        assert_eq!(a, b);
        let m = TopoName::merged(
            OpTag(3),
            [g(1, Role::Top), g(1, Role::Bottom), g(1, Role::Top)],
        )
        .unwrap();
        let TopoName::Merged { parts, .. } = &m else {
            panic!("not merged")
        };
        assert_eq!(parts.len(), 2);
        assert!(parts[0] < parts[1]);
        assert!(TopoName::merged(OpTag(3), []).is_err());
    }

    #[test]
    fn names_round_trip_and_refuse_non_canonical_forms() {
        let n = TopoName::modified(
            OpTag(9),
            TopoName::between(OpTag(4), g(1, Role::Top), g(2, Role::Edge(3)), 1),
            2,
        );
        let json = serde_json::to_string(&n).unwrap();
        assert_eq!(serde_json::from_str::<TopoName>(&json).unwrap(), n);
        assert_eq!(
            n.ops(),
            BTreeSet::from([OpTag(1), OpTag(2), OpTag(4), OpTag(9)])
        );

        let generated = r#"{"Generated":{"op":7,"source":"Primitive","role":{"Side":0}}}"#;
        assert_eq!(
            serde_json::from_str::<TopoName>(generated).unwrap(),
            g(7, Role::Side(0))
        );
        let top = r#"{"Generated":{"op":1,"source":"Primitive","role":"Top"}}"#;
        let bottom = r#"{"Generated":{"op":1,"source":"Primitive","role":"Bottom"}}"#;
        let unsorted = format!(r#"{{"Between":{{"op":1,"faces":[{bottom},{top}],"index":0}}}}"#);
        assert!(serde_json::from_str::<TopoName>(&unsorted).is_err());
        let unsorted = format!(r#"{{"Merged":{{"op":1,"parts":[{bottom},{top}]}}}}"#);
        assert!(serde_json::from_str::<TopoName>(&unsorted).is_err());
        let sorted = format!(r#"{{"Merged":{{"op":1,"parts":[{top},{bottom}]}}}}"#);
        assert!(serde_json::from_str::<TopoName>(&sorted).is_ok());
        assert!(serde_json::from_str::<TopoName>(r#"{"Merged":{"op":1,"parts":[]}}"#).is_err());
    }

    #[test]
    fn display_is_compact() {
        assert_eq!(g(7, Role::Bottom).to_string(), "gen(#7, primitive, Bottom)");
        let b = TopoName::between(OpTag(2), g(1, Role::Top), g(1, Role::Bottom), 0);
        assert_eq!(
            b.to_string(),
            "between(#2, gen(#1, primitive, Top), gen(#1, primitive, Bottom), 0)"
        );
    }

    #[test]
    fn history_keeps_entries_sorted_and_unique() {
        let mut h = History::new();
        h.record(g(1, Role::Top), EntityKind::Face, vec![]).unwrap();
        h.record(g(1, Role::Bottom), EntityKind::Face, vec![])
            .unwrap();
        assert!(matches!(
            h.record(g(1, Role::Top), EntityKind::Face, vec![]),
            Err(KernelError::DuplicateHistory(_))
        ));
        assert_eq!(h.len(), 2);
        assert_eq!(h.iter().next().unwrap().name, g(1, Role::Top));
        assert_eq!(h.get(&g(1, Role::Bottom)).unwrap().kind, EntityKind::Face);
        assert!(h.get(&g(2, Role::Bottom)).is_none());
        let json = serde_json::to_string(&h).unwrap();
        assert_eq!(serde_json::from_str::<History>(&json).unwrap(), h);
        let entry = |role: &str| {
            format!(
                r#"{{"name":{{"Generated":{{"op":1,"source":"Primitive","role":"{role}"}}}},"kind":"Face","from":[]}}"#
            )
        };
        let unsorted = format!("[{},{}]", entry("Bottom"), entry("Top"));
        assert!(serde_json::from_str::<History>(&unsorted).is_err());
    }

    #[test]
    fn context_checks_tolerance_and_cancellation() {
        let token = Arc::new(AtomicBool::new(false));
        let mut cx = OpContext::with_cancel(Arc::clone(&token));
        assert!(cx.check_cancelled().is_ok());
        token.store(true, Ordering::Relaxed);
        assert_eq!(cx.check_cancelled(), Err(KernelError::Cancelled));
        assert!(cx.checked_tolerance().is_ok());
        for bad in [0.0, 1e-9, f64::NAN, 1.0] {
            cx.tolerance = bad;
            assert!(cx.checked_tolerance().is_err(), "{bad}");
        }
    }
}
