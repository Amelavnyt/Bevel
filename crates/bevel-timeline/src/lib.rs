// SPDX-License-Identifier: GPL-3.0-or-later

//! Generic history timeline.
//!
//! One engine serves both the part timeline (features) and every sketch
//! timeline (sketch ops): ordered steps, a rollback marker, suppression,
//! dependency-checked reordering and a replay cache keyed by step content.
//!
//! - [`Timeline`] holds the steps and the marker and validates every edit:
//!   a step may never come before a step it depends on.
//! - [`replay`] runs the steps before the marker through an [`Evaluator`],
//!   reusing cached states, and reports a [`StepStatus`] for every step. A
//!   step whose input failed, was suppressed or was deleted is skipped, and
//!   everything that does not depend on it still runs.
//! - [`ReplayCache`] keeps the state after each step, keyed by a hash of the
//!   step, its resolved inputs and everything before it, so editing step `k`
//!   replays from `k` and scrubbing the marker over cached steps evaluates
//!   nothing.

mod digest;
mod replay;
mod timeline;

pub use digest::{Digest, DigestBuilder};
pub use replay::{
    Applied, Cancelled, Evaluator, Replay, ReplayCache, SkipReason, StepStatus, replay,
};
pub use timeline::{EditError, Step, Timeline};

/// One operation in a timeline: a feature, or a sketch op.
pub trait TimelineOp {
    /// Stable identifier, unique within a timeline.
    type Id: Copy + Ord + std::hash::Hash + std::fmt::Debug;

    /// This op's identifier.
    fn id(&self) -> Self::Id;

    /// The steps whose outputs this op reads. For features this includes
    /// every feature named in a stored geometry reference, so reordering
    /// understands geometry references too.
    fn deps(&self) -> Vec<Self::Id>;

    /// Hash of everything that defines the op, including its id.
    fn content_hash(&self) -> Digest;
}
