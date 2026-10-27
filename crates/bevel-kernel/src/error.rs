// SPDX-License-Identifier: GPL-3.0-or-later

//! Errors returned by kernel operations.

use bevel_geom::GeomError;
use thiserror::Error;

use crate::naming::TopoName;
use crate::topology::EntityKey;
use crate::validate::Issue;

/// Why a kernel operation failed.
///
/// Every message is plain English; the variants carry the technical
/// detail (keys, values, the validator's issues).
#[derive(Clone, Debug, PartialEq, Error)]
pub enum KernelError {
    /// A numeric parameter is out of range.
    #[error("invalid {name} {value}: {reason}")]
    InvalidParameter {
        /// Which parameter.
        name: &'static str,
        /// The rejected value.
        value: f64,
        /// What the value must satisfy.
        reason: &'static str,
    },

    /// A placement frame is not finite, orthonormal and right-handed (see
    /// [`Frame3::is_valid`](bevel_math::Frame3::is_valid)), or its origin
    /// lies outside the modelling space.
    #[error(
        "the placement frame must be finite, orthonormal and right-handed, with its origin inside the modelling space"
    )]
    InvalidFrame,

    /// The result would reach outside the modelling space
    /// (`±`[`MODEL_EXTENT`](bevel_math::tol::MODEL_EXTENT) on every axis).
    #[error(
        "the result would reach {extent} mm from the origin along an axis, outside the modelling space of ±{max} mm",
        max = bevel_math::tol::MODEL_EXTENT
    )]
    OutsideModelSpace {
        /// The largest absolute coordinate the result would have.
        extent: f64,
    },

    /// Constructing a curve or surface failed.
    #[error("geometry error: {0}")]
    Geometry(#[from] GeomError),

    /// A builder step referred to an entity that does not exist.
    #[error("{0} does not exist in the body being built")]
    UnknownKey(EntityKey),

    /// A builder step would give an entity a second owner (a face added
    /// to a second shell, for example).
    #[error("{entity} already belongs to {owner}")]
    AlreadyOwned {
        /// The entity.
        entity: EntityKey,
        /// Its existing owner.
        owner: EntityKey,
    },

    /// The coedges given for a loop do not join up: coedge `index` ends at
    /// a different vertex than the next one (cyclically) starts at.
    #[error("the coedges of the loop do not join up after coedge {index}")]
    OpenLoop {
        /// Position in the given list of the coedge whose end is wrong.
        index: usize,
    },

    /// Some other input is malformed; the message says what and why.
    #[error("invalid {what}: {reason}")]
    InvalidInput {
        /// What was malformed.
        what: &'static str,
        /// Why.
        reason: &'static str,
    },

    /// A finished body failed validation; `issues` lists everything the
    /// validator found.
    #[error("the body is invalid: {}", summary(.issues))]
    InvalidBody {
        /// The validator's findings, never empty.
        issues: Vec<Issue>,
    },

    /// A name was recorded twice in an operation's history.
    #[error("{0} is recorded twice in the history")]
    DuplicateHistory(Box<TopoName>),

    /// The operation was cancelled through its context.
    #[error("the operation was cancelled")]
    Cancelled,

    /// Tessellating `entity` (an edge or a face) to the requested
    /// tolerances would need more than `limit` points. Loosen the
    /// tolerances; the limit keeps a tiny tolerance from exhausting
    /// memory.
    #[error(
        "tessellating {entity} needs more than {limit} points; loosen the tessellation tolerances"
    )]
    PointLimit {
        /// The edge or face being tessellated.
        entity: EntityKey,
        /// The largest number of points allowed.
        limit: usize,
    },

    /// `entity` (an edge or a face) cannot be tessellated; the message
    /// says why (for example a face boundary that crosses itself in
    /// `(u, v)` once discretized).
    #[error("cannot tessellate {entity}: {reason}")]
    Tessellation {
        /// The edge or face.
        entity: EntityKey,
        /// Why.
        reason: &'static str,
    },
}

/// "N issues; first: ..." for [`KernelError::InvalidBody`].
fn summary(issues: &[Issue]) -> String {
    match issues.first() {
        None => "no issues were reported".to_owned(),
        Some(first) if issues.len() == 1 => first.to_string(),
        Some(first) => format!("{} issues, the first: {first}", issues.len()),
    }
}
