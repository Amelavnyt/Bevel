// SPDX-License-Identifier: GPL-3.0-or-later

//! Replaying a timeline through an evaluator, with a cache of states.

use crate::{Digest, DigestBuilder, Timeline, TimelineOp};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

/// Runs ops: turns the state before a step into the state after it.
pub trait Evaluator<Op: TimelineOp> {
    /// What the timeline builds, e.g. the bodies and sketches of a part.
    /// Cloned when cached, so it should share its contents (`Arc`).
    type State: Clone;
    /// A warning or error about one step.
    type Diagnostic: Clone + std::fmt::Debug;

    /// The state before the first step.
    fn initial_state(&self) -> Self::State;

    /// A hash of the inputs `op` reads from outside the timeline, such as
    /// resolved parameter values. Part of the cache key, so changing a
    /// parameter re-runs exactly the steps that read it.
    fn input_hash(&self, op: &Op) -> Digest {
        let _ = op;
        Digest::default()
    }

    /// Runs `op` on `state`.
    ///
    /// # Errors
    ///
    /// The reason the step failed; replay keeps the previous state and skips
    /// the steps that depend on this one.
    fn apply(
        &self,
        state: &Self::State,
        op: &Op,
    ) -> Result<Applied<Self::State, Self::Diagnostic>, Self::Diagnostic>;

    /// The approximate memory a cached state costs, in bytes, for the
    /// cache budget.
    fn state_cost(&self, state: &Self::State) -> usize {
        let _ = state;
        1
    }
}

/// The result of running one step.
#[derive(Clone, Debug)]
pub struct Applied<S, D> {
    /// The state after the step.
    pub state: S,
    /// Warnings; the step still counts as done.
    pub warnings: Vec<D>,
}

impl<S, D> Applied<S, D> {
    /// A result without warnings.
    pub fn ok(state: S) -> Self {
        Self {
            state,
            warnings: Vec::new(),
        }
    }
}

/// Why a step was not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SkipReason<Id> {
    /// The step depends on a step that failed, was skipped or is suppressed.
    InputUnavailable(Id),
    /// The step depends on a step that was deleted.
    InputMissing(Id),
}

/// What happened to a step in the last replay.
#[derive(Clone, Debug, PartialEq)]
pub enum StepStatus<Id, D> {
    /// Ran without problems.
    Ok,
    /// Ran, with warnings.
    Warning(Vec<D>),
    /// Failed; the state is as before the step.
    Error(D),
    /// Not run because an input is unavailable.
    Skipped(SkipReason<Id>),
    /// Suppressed by the user.
    Suppressed,
    /// After the marker.
    AfterMarker,
}

impl<Id, D> StepStatus<Id, D> {
    /// Whether the step contributed to the state.
    pub fn ran(&self) -> bool {
        matches!(self, Self::Ok | Self::Warning(_))
    }
}

/// Replay was cancelled before it finished.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("replay cancelled")]
pub struct Cancelled;

/// The result of a replay.
#[derive(Clone, Debug)]
pub struct Replay<Id, S, D> {
    /// The state at the marker.
    pub state: S,
    /// The status of every step, in timeline order.
    pub statuses: Vec<(Id, StepStatus<Id, D>)>,
    /// How many steps were evaluated rather than taken from the cache.
    pub evaluated: usize,
}

impl<Id: Copy + PartialEq, S, D> Replay<Id, S, D> {
    /// The status of step `id`.
    pub fn status(&self, id: Id) -> Option<&StepStatus<Id, D>> {
        self.statuses.iter().find(|(i, _)| *i == id).map(|(_, s)| s)
    }
}

/// What is known after a step, apart from the state: its status and the
/// steps unavailable to later ones. Cheap, so kept even when the state is
/// evicted.
struct Meta<Id, D> {
    status: StepStatus<Id, D>,
    unavailable: Arc<BTreeSet<Id>>,
}

/// Cached states after steps, keyed by a hash of the step and everything
/// before it, evicted least recently used first once over budget.
pub struct ReplayCache<Id, S, D> {
    states: BTreeMap<Digest, (S, usize, u64)>,
    meta: BTreeMap<Digest, Meta<Id, D>>,
    budget: usize,
    used: usize,
    clock: u64,
}

/// More status entries than this and the status table is cleared; they are
/// tiny, so this only bounds pathological sessions.
const META_LIMIT: usize = 100_000;

impl<Id: Copy + Ord, S: Clone, D: Clone> ReplayCache<Id, S, D> {
    /// A cache holding states up to a total cost of `budget` (see
    /// [`Evaluator::state_cost`]).
    pub fn new(budget: usize) -> Self {
        Self {
            states: BTreeMap::new(),
            meta: BTreeMap::new(),
            budget,
            used: 0,
            clock: 0,
        }
    }

    /// The number of cached states.
    pub fn len(&self) -> usize {
        self.states.len()
    }

    /// Whether no state is cached.
    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }

    /// Forgets everything.
    pub fn clear(&mut self) {
        self.states.clear();
        self.meta.clear();
        self.used = 0;
    }

    fn get_state(&mut self, key: &Digest) -> Option<S> {
        self.clock += 1;
        let clock = self.clock;
        self.states.get_mut(key).map(|(s, _, last)| {
            *last = clock;
            s.clone()
        })
    }

    fn put(&mut self, key: Digest, state: S, cost: usize, meta: Meta<Id, D>) {
        if self.meta.len() >= META_LIMIT {
            self.meta.clear();
        }
        self.meta.insert(key, meta);
        if cost > self.budget {
            return;
        }
        self.clock += 1;
        if let Some((_, old, _)) = self.states.insert(key, (state, cost, self.clock)) {
            self.used -= old;
        }
        self.used += cost;
        while self.used > self.budget {
            let Some(oldest) = self
                .states
                .iter()
                .min_by_key(|(_, (_, _, last))| *last)
                .map(|(k, _)| *k)
            else {
                break;
            };
            if let Some((_, c, _)) = self.states.remove(&oldest) {
                self.used -= c;
            }
        }
    }
}

type ReplayOf<Op, E> =
    Replay<<Op as TimelineOp>::Id, <E as Evaluator<Op>>::State, <E as Evaluator<Op>>::Diagnostic>;

/// The cache key after each step before the marker.
fn keys<Op: TimelineOp, E: Evaluator<Op>>(timeline: &Timeline<Op>, evaluator: &E) -> Vec<Digest> {
    let mut key = Digest::of(b"bevel-timeline replay v1");
    let mut out = Vec::with_capacity(timeline.marker());
    for step in &timeline.steps()[..timeline.marker()] {
        key = DigestBuilder::new("step")
            .digest(&key)
            .digest(&step.op().content_hash())
            .digest(&evaluator.input_hash(step.op()))
            .u64(u64::from(step.suppressed()))
            .finish();
        out.push(key);
    }
    out
}

/// Replays `timeline` up to its marker.
///
/// Starts from the latest step whose state is cached (with everything
/// before it known), evaluates the rest and caches each new state.
/// `cancelled` is polled before each evaluated step.
///
/// # Errors
///
/// [`Cancelled`] if `cancelled` returned `true`; the cache keeps the steps
/// finished so far.
pub fn replay<Op: TimelineOp, E: Evaluator<Op>>(
    timeline: &Timeline<Op>,
    evaluator: &E,
    cache: &mut ReplayCache<Op::Id, E::State, E::Diagnostic>,
    cancelled: &dyn Fn() -> bool,
) -> Result<ReplayOf<Op, E>, Cancelled> {
    let steps = timeline.steps();
    let marker = timeline.marker();
    let keys = keys(timeline, evaluator);

    // The longest prefix with all statuses known whose last state is cached.
    let known = keys
        .iter()
        .take_while(|k| cache.meta.contains_key(*k))
        .count();
    let mut start = 0;
    let mut state = None;
    for j in (1..=known).rev() {
        if let Some(s) = cache.get_state(&keys[j - 1]) {
            start = j;
            state = Some(s);
            break;
        }
    }
    let mut state = state.unwrap_or_else(|| evaluator.initial_state());

    let mut statuses = Vec::with_capacity(steps.len());
    let mut unavailable = Arc::new(BTreeSet::new());
    for (step, key) in steps.iter().zip(&keys).take(start) {
        if let Some(meta) = cache.meta.get(key) {
            statuses.push((step.op().id(), meta.status.clone()));
            unavailable = Arc::clone(&meta.unavailable);
        }
    }

    let present: BTreeSet<Op::Id> = steps.iter().map(|s| s.op().id()).collect();
    let mut evaluated = 0;
    for (step, key) in steps.iter().zip(&keys).skip(start) {
        let op = step.op();
        let id = op.id();
        let deps = op.deps();
        let status = if step.suppressed() {
            StepStatus::Suppressed
        } else if let Some(missing) = deps.iter().find(|d| !present.contains(d)) {
            StepStatus::Skipped(SkipReason::InputMissing(*missing))
        } else if let Some(down) = deps.iter().find(|d| unavailable.contains(d)) {
            StepStatus::Skipped(SkipReason::InputUnavailable(*down))
        } else {
            if cancelled() {
                return Err(Cancelled);
            }
            evaluated += 1;
            match evaluator.apply(&state, op) {
                Ok(applied) => {
                    state = applied.state;
                    if applied.warnings.is_empty() {
                        StepStatus::Ok
                    } else {
                        StepStatus::Warning(applied.warnings)
                    }
                }
                Err(d) => StepStatus::Error(d),
            }
        };
        if !status.ran() {
            Arc::make_mut(&mut unavailable).insert(id);
        }
        let meta = Meta {
            status: status.clone(),
            unavailable: Arc::clone(&unavailable),
        };
        cache.put(*key, state.clone(), evaluator.state_cost(&state), meta);
        statuses.push((id, status));
    }
    for step in &steps[marker..] {
        statuses.push((step.op().id(), StepStatus::AfterMarker));
    }
    Ok(Replay {
        state,
        statuses,
        evaluated,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::timeline::tests::{Toy, toy};
    use std::cell::Cell;

    /// Sums the values of the steps that ran.
    struct Sum {
        offset: i64,
    }

    impl Evaluator<Toy> for Sum {
        type State = i64;
        type Diagnostic = String;
        fn initial_state(&self) -> i64 {
            0
        }
        fn input_hash(&self, _: &Toy) -> Digest {
            DigestBuilder::new("sum")
                .u64(self.offset.cast_unsigned())
                .finish()
        }
        fn apply(&self, state: &i64, op: &Toy) -> Result<Applied<i64, String>, String> {
            if op.fail {
                Err(format!("step {} failed", op.id))
            } else {
                Ok(Applied::ok(state + op.value + self.offset))
            }
        }
    }

    fn run(
        t: &Timeline<Toy>,
        cache: &mut ReplayCache<u32, i64, String>,
    ) -> Replay<u32, i64, String> {
        replay(t, &Sum { offset: 0 }, cache, &|| false).unwrap()
    }

    #[test]
    fn failures_skip_only_their_dependents() {
        let mut t = Timeline::new();
        t.insert(toy(1, &[])).unwrap();
        t.insert(Toy {
            fail: true,
            ..toy(2, &[1])
        })
        .unwrap();
        t.insert(toy(3, &[2])).unwrap();
        t.insert(toy(4, &[1])).unwrap();
        t.insert(toy(5, &[3])).unwrap();
        let r = run(&t, &mut ReplayCache::new(100));
        assert_eq!(r.state, 1 + 4);
        assert_eq!(r.status(1), Some(&StepStatus::Ok));
        assert!(matches!(r.status(2), Some(StepStatus::Error(_))));
        assert_eq!(
            r.status(3),
            Some(&StepStatus::Skipped(SkipReason::InputUnavailable(2)))
        );
        assert_eq!(
            r.status(5),
            Some(&StepStatus::Skipped(SkipReason::InputUnavailable(3)))
        );
        assert_eq!(r.status(4), Some(&StepStatus::Ok));
    }

    #[test]
    fn suppressed_deleted_and_rolled_back_steps() {
        let mut t = Timeline::new();
        for (id, deps) in [(1, &[][..]), (2, &[1]), (3, &[2]), (4, &[])] {
            t.insert(toy(id, deps)).unwrap();
        }
        t.set_suppressed(1, true).unwrap();
        t.set_marker(3).unwrap();
        let r = run(&t, &mut ReplayCache::new(100));
        assert_eq!(r.state, 0);
        assert_eq!(r.status(1), Some(&StepStatus::Suppressed));
        assert_eq!(
            r.status(2),
            Some(&StepStatus::Skipped(SkipReason::InputUnavailable(1)))
        );
        assert_eq!(r.status(4), Some(&StepStatus::AfterMarker));
        t.set_suppressed(1, false).unwrap();
        t.remove(2).unwrap();
        let r = run(&t, &mut ReplayCache::new(100));
        assert_eq!(
            r.status(3),
            Some(&StepStatus::Skipped(SkipReason::InputMissing(2)))
        );
        assert_eq!(r.state, 1);
    }

    #[test]
    fn edits_replay_from_the_edited_step() {
        let mut t = Timeline::new();
        for id in 1..=5 {
            t.insert(toy(id, &[])).unwrap();
        }
        let mut cache = ReplayCache::new(100);
        assert_eq!(run(&t, &mut cache).evaluated, 5);
        assert_eq!(run(&t, &mut cache).evaluated, 0);
        t.replace(Toy {
            value: 10,
            ..toy(4, &[])
        })
        .unwrap();
        let r = run(&t, &mut cache);
        assert_eq!(r.evaluated, 2);
        assert_eq!(r.state, 1 + 2 + 3 + 10 + 5);
        // Scrubbing back over cached steps evaluates nothing.
        t.set_marker(2).unwrap();
        assert_eq!(run(&t, &mut cache).evaluated, 0);
        // Changing an outside input re-runs everything that reads it.
        let r = replay(&t, &Sum { offset: 1 }, &mut cache, &|| false).unwrap();
        assert_eq!(r.evaluated, 2);
    }

    #[test]
    fn eviction_keeps_results_correct() {
        let mut t = Timeline::new();
        for id in 1..=8 {
            t.insert(toy(id, &[])).unwrap();
        }
        let mut cache = ReplayCache::new(3);
        run(&t, &mut cache);
        assert_eq!(cache.len(), 3);
        t.set_marker(2).unwrap();
        let r = run(&t, &mut cache);
        assert_eq!(r.state, 3);
        assert_eq!(r.evaluated, 2, "early states were evicted");
    }

    #[test]
    fn cancellation_stops_between_steps() {
        let mut t = Timeline::new();
        for id in 1..=4 {
            t.insert(toy(id, &[])).unwrap();
        }
        let polls = Cell::new(0);
        let mut cache = ReplayCache::new(100);
        let r = replay(&t, &Sum { offset: 0 }, &mut cache, &|| {
            polls.set(polls.get() + 1);
            polls.get() > 2
        });
        assert_eq!(r.unwrap_err(), Cancelled);
        // The finished steps are cached.
        assert_eq!(run(&t, &mut cache).evaluated, 2);
    }
}
