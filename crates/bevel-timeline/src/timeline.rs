// SPDX-License-Identifier: GPL-3.0-or-later

//! The ordered steps and the rollback marker.

use crate::TimelineOp;
use std::collections::BTreeSet;
use std::sync::Arc;

/// One step: an op and whether it is suppressed.
#[derive(Debug)]
pub struct Step<Op> {
    op: Arc<Op>,
    suppressed: bool,
}

impl<Op> Clone for Step<Op> {
    fn clone(&self) -> Self {
        Self {
            op: Arc::clone(&self.op),
            suppressed: self.suppressed,
        }
    }
}

impl<Op> Step<Op> {
    /// A step running `op`.
    pub fn new(op: Arc<Op>, suppressed: bool) -> Self {
        Self { op, suppressed }
    }

    /// The op.
    pub fn op(&self) -> &Arc<Op> {
        &self.op
    }

    /// Whether the step is suppressed (kept, but not run).
    pub fn suppressed(&self) -> bool {
        self.suppressed
    }
}

/// Why an edit was refused. The timeline is unchanged.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum EditError<Id: std::fmt::Debug> {
    /// No step has this id.
    #[error("no step {0:?}")]
    NotFound(Id),
    /// A step with this id already exists.
    #[error("step {0:?} already exists")]
    Duplicate(Id),
    /// The op depends on a step that is not in the timeline.
    #[error("step {step:?} depends on {dependency:?}, which does not exist")]
    MissingDependency {
        /// The step being added or edited.
        step: Id,
        /// The dependency that does not exist.
        dependency: Id,
    },
    /// The step would come before a step it depends on.
    #[error("step {step:?} would come before {dependency:?}, which it depends on")]
    WouldPrecedeDependency {
        /// The step being placed.
        step: Id,
        /// The step it depends on.
        dependency: Id,
    },
    /// The step would come after a step that depends on it.
    #[error("step {step:?} would come after {dependent:?}, which depends on it")]
    WouldFollowDependent {
        /// The step being placed.
        step: Id,
        /// The step that depends on it.
        dependent: Id,
    },
    /// A replacement op has a different id from the step it replaces.
    #[error("a replacement for {expected:?} has id {found:?}")]
    IdMismatch {
        /// The id of the step being replaced.
        expected: Id,
        /// The id of the replacement.
        found: Id,
    },
    /// An index is past the end of the timeline.
    #[error("index {index} is past the end ({len} steps)")]
    OutOfRange {
        /// The index asked for.
        index: usize,
        /// The number of steps.
        len: usize,
    },
}

/// Ordered steps with a rollback marker.
///
/// The marker sits between steps: the first `marker` steps run, the rest
/// are rolled back. New steps go in at the marker. Every edit keeps the
/// invariant that a step comes after all the steps it depends on that
/// exist; deleting a step without its dependents leaves them with a missing
/// input, and replay skips them.
#[derive(Debug)]
pub struct Timeline<Op> {
    steps: Vec<Step<Op>>,
    marker: usize,
}

impl<Op> Clone for Timeline<Op> {
    fn clone(&self) -> Self {
        Self {
            steps: self.steps.clone(),
            marker: self.marker,
        }
    }
}

impl<Op> Default for Timeline<Op> {
    fn default() -> Self {
        Self {
            steps: Vec::new(),
            marker: 0,
        }
    }
}

type Result<T, Op> = std::result::Result<T, EditError<<Op as TimelineOp>::Id>>;

impl<Op: TimelineOp> Timeline<Op> {
    /// An empty timeline.
    pub fn new() -> Self {
        Self::default()
    }

    /// A timeline from stored steps and marker, checking ids and order.
    ///
    /// # Errors
    ///
    /// The first problem found: a duplicate id, a step before one of its
    /// dependencies, or a marker past the end. Missing dependencies are
    /// allowed (the dependency was deleted).
    pub fn from_steps(steps: Vec<Step<Op>>, marker: usize) -> Result<Self, Op> {
        let mut seen = BTreeSet::new();
        let all: BTreeSet<Op::Id> = steps.iter().map(|s| s.op.id()).collect();
        for step in &steps {
            let id = step.op.id();
            for dep in step.op.deps() {
                if all.contains(&dep) && !seen.contains(&dep) {
                    return Err(EditError::WouldPrecedeDependency {
                        step: id,
                        dependency: dep,
                    });
                }
            }
            if !seen.insert(id) {
                return Err(EditError::Duplicate(id));
            }
        }
        if marker > steps.len() {
            return Err(EditError::OutOfRange {
                index: marker,
                len: steps.len(),
            });
        }
        Ok(Self { steps, marker })
    }

    /// The steps in order.
    pub fn steps(&self) -> &[Step<Op>] {
        &self.steps
    }

    /// The number of steps.
    pub fn len(&self) -> usize {
        self.steps.len()
    }

    /// Whether there are no steps.
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// The marker: how many steps run.
    pub fn marker(&self) -> usize {
        self.marker
    }

    /// The index of step `id`.
    pub fn position(&self, id: Op::Id) -> Option<usize> {
        self.steps.iter().position(|s| s.op.id() == id)
    }

    /// Step `id`.
    pub fn get(&self, id: Op::Id) -> Option<&Step<Op>> {
        self.steps.iter().find(|s| s.op.id() == id)
    }

    fn index(&self, id: Op::Id) -> Result<usize, Op> {
        self.position(id).ok_or(EditError::NotFound(id))
    }

    /// The steps that depend on `id`, directly or through other steps, in
    /// timeline order.
    pub fn dependents(&self, id: Op::Id) -> Vec<Op::Id> {
        let mut reached = BTreeSet::from([id]);
        let mut out = Vec::new();
        for step in &self.steps {
            let sid = step.op.id();
            if sid != id && step.op.deps().iter().any(|d| reached.contains(d)) {
                reached.insert(sid);
                out.push(sid);
            }
        }
        out
    }

    /// Checks that `op` can sit at `index` in `steps`: every dependency is
    /// earlier and every dependent later. Dependencies in `may_be_missing`
    /// (those the step already had) may be absent.
    fn check_place(
        steps: &[Step<Op>],
        index: usize,
        op: &Op,
        may_be_missing: &[Op::Id],
    ) -> Result<(), Op> {
        let id = op.id();
        for dep in op.deps() {
            match steps.iter().position(|s| s.op.id() == dep) {
                None if may_be_missing.contains(&dep) => {}
                None => {
                    return Err(EditError::MissingDependency {
                        step: id,
                        dependency: dep,
                    });
                }
                Some(i) if i >= index => {
                    return Err(EditError::WouldPrecedeDependency {
                        step: id,
                        dependency: dep,
                    });
                }
                Some(_) => {}
            }
        }
        for (i, step) in steps.iter().enumerate() {
            if i < index && step.op.deps().contains(&id) {
                return Err(EditError::WouldFollowDependent {
                    step: id,
                    dependent: step.op.id(),
                });
            }
        }
        Ok(())
    }

    /// Inserts `op` at the marker and moves the marker past it.
    ///
    /// # Errors
    ///
    /// The id is taken, or a dependency is missing or rolled back.
    pub fn insert(&mut self, op: Op) -> Result<(), Op> {
        self.insert_at(self.marker, op)
    }

    /// Inserts `op` at `index`. The marker moves with the steps after it.
    ///
    /// # Errors
    ///
    /// The id is taken, `index` is past the end, or a dependency is missing
    /// or not before `index`.
    pub fn insert_at(&mut self, index: usize, op: Op) -> Result<(), Op> {
        if index > self.steps.len() {
            return Err(EditError::OutOfRange {
                index,
                len: self.steps.len(),
            });
        }
        if self.position(op.id()).is_some() {
            return Err(EditError::Duplicate(op.id()));
        }
        Self::check_place(&self.steps, index, &op, &[])?;
        self.steps.insert(index, Step::new(Arc::new(op), false));
        if index <= self.marker {
            self.marker += 1;
        }
        Ok(())
    }

    /// Replaces step `op.id()` in place, keeping its suppression, and
    /// returns the old op. Dependencies the step already had may be missing
    /// (they were deleted), so a step with a lost input can still be edited.
    ///
    /// # Errors
    ///
    /// No such step, or a new dependency is missing, or a dependency is not
    /// earlier.
    pub fn replace(&mut self, op: Op) -> Result<Arc<Op>, Op> {
        let id = op.id();
        let index = self.index(id)?;
        let old_deps = self.steps[index].op.deps();
        Self::check_place(&self.steps, index, &op, &old_deps)?;
        let step = &mut self.steps[index];
        Ok(std::mem::replace(&mut step.op, Arc::new(op)))
    }

    /// Removes step `id`; its dependents stay and will be skipped.
    ///
    /// # Errors
    ///
    /// No such step.
    pub fn remove(&mut self, id: Op::Id) -> Result<Step<Op>, Op> {
        let index = self.index(id)?;
        if index < self.marker {
            self.marker -= 1;
        }
        Ok(self.steps.remove(index))
    }

    /// Removes step `id` and everything that depends on it, returning the
    /// removed steps in timeline order.
    ///
    /// # Errors
    ///
    /// No such step.
    pub fn remove_with_dependents(&mut self, id: Op::Id) -> Result<Vec<Step<Op>>, Op> {
        self.index(id)?;
        let mut doomed: BTreeSet<Op::Id> = self.dependents(id).into_iter().collect();
        doomed.insert(id);
        let mut removed = Vec::new();
        let mut kept = Vec::new();
        let mut marker = self.marker;
        for (i, step) in std::mem::take(&mut self.steps).into_iter().enumerate() {
            if doomed.contains(&step.op.id()) {
                if i < self.marker {
                    marker -= 1;
                }
                removed.push(step);
            } else {
                kept.push(step);
            }
        }
        self.steps = kept;
        self.marker = marker;
        Ok(removed)
    }

    /// Moves step `id` so that it ends up at index `to`.
    ///
    /// A step that ran before the move keeps running if it lands at or
    /// before the marker, and a rolled-back step stays rolled back if it
    /// lands at or after the marker; the other steps keep their side of the
    /// marker.
    ///
    /// # Errors
    ///
    /// No such step, `to` is past the end, or the step would come before a
    /// dependency or after a dependent.
    pub fn move_to(&mut self, id: Op::Id, to: usize) -> Result<(), Op> {
        let from = self.index(id)?;
        if to >= self.steps.len() {
            return Err(EditError::OutOfRange {
                index: to,
                len: self.steps.len(),
            });
        }
        let mut rest = self.steps.clone();
        let step = rest.remove(from);
        Self::check_place(&rest, to, &step.op, &step.op.deps())?;
        let was_active = from < self.marker;
        // The marker among the other steps.
        let boundary = self.marker - usize::from(was_active);
        let active = to < boundary || (to == boundary && was_active);
        rest.insert(to, step);
        self.steps = rest;
        self.marker = boundary + usize::from(active);
        Ok(())
    }

    /// Suppresses or unsuppresses step `id`.
    ///
    /// # Errors
    ///
    /// No such step.
    pub fn set_suppressed(&mut self, id: Op::Id, suppressed: bool) -> Result<(), Op> {
        let index = self.index(id)?;
        self.steps[index].suppressed = suppressed;
        Ok(())
    }

    /// Moves the marker so that the first `marker` steps run.
    ///
    /// # Errors
    ///
    /// `marker` is past the end.
    pub fn set_marker(&mut self, marker: usize) -> Result<(), Op> {
        if marker > self.steps.len() {
            return Err(EditError::OutOfRange {
                index: marker,
                len: self.steps.len(),
            });
        }
        self.marker = marker;
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::{Digest, DigestBuilder};

    /// A toy op: adds `value` to the state; fails if `fail`.
    #[derive(Clone, Debug, PartialEq)]
    pub(crate) struct Toy {
        pub id: u32,
        pub deps: Vec<u32>,
        pub value: i64,
        pub fail: bool,
    }

    impl TimelineOp for Toy {
        type Id = u32;
        fn id(&self) -> u32 {
            self.id
        }
        fn deps(&self) -> Vec<u32> {
            self.deps.clone()
        }
        fn content_hash(&self) -> Digest {
            let mut b = DigestBuilder::new("toy");
            b.u64(u64::from(self.id))
                .u64(self.value.cast_unsigned())
                .u64(u64::from(self.fail));
            for d in &self.deps {
                b.u64(u64::from(*d));
            }
            b.finish()
        }
    }

    pub(crate) fn toy(id: u32, deps: &[u32]) -> Toy {
        Toy {
            id,
            deps: deps.to_vec(),
            value: i64::from(id),
            fail: false,
        }
    }

    fn ids(t: &Timeline<Toy>) -> Vec<u32> {
        t.steps().iter().map(|s| s.op().id).collect()
    }

    fn abc() -> Timeline<Toy> {
        let mut t = Timeline::new();
        t.insert(toy(1, &[])).unwrap();
        t.insert(toy(2, &[1])).unwrap();
        t.insert(toy(3, &[])).unwrap();
        t.insert(toy(4, &[2])).unwrap();
        t
    }

    #[test]
    fn inserts_go_in_at_the_marker() {
        let mut t = abc();
        t.set_marker(2).unwrap();
        t.insert(toy(5, &[1])).unwrap();
        assert_eq!(ids(&t), [1, 2, 5, 3, 4]);
        assert_eq!(t.marker(), 3);
        // Step 4 is rolled back, so nothing new may depend on it.
        assert_eq!(
            t.insert(toy(6, &[4])),
            Err(EditError::WouldPrecedeDependency {
                step: 6,
                dependency: 4
            })
        );
        assert_eq!(t.insert(toy(1, &[])), Err(EditError::Duplicate(1)));
        assert_eq!(
            t.insert(toy(7, &[9])),
            Err(EditError::MissingDependency {
                step: 7,
                dependency: 9
            })
        );
        assert_eq!(ids(&t), [1, 2, 5, 3, 4], "refused edits change nothing");
    }

    #[test]
    fn moves_respect_dependencies() {
        let mut t = abc();
        assert_eq!(
            t.move_to(2, 0),
            Err(EditError::WouldPrecedeDependency {
                step: 2,
                dependency: 1
            })
        );
        assert_eq!(
            t.move_to(2, 3),
            Err(EditError::WouldFollowDependent {
                step: 2,
                dependent: 4
            })
        );
        t.move_to(3, 0).unwrap();
        assert_eq!(ids(&t), [3, 1, 2, 4]);
        t.move_to(3, 2).unwrap();
        assert_eq!(ids(&t), [1, 2, 3, 4]);
        assert_eq!(t.marker(), 4);
    }

    #[test]
    fn moves_keep_steps_on_their_side_of_the_marker() {
        let mut t = abc();
        t.set_marker(2).unwrap(); // 1 2 | 3 4
        // A rolled-back step moved in front of running steps runs.
        t.move_to(3, 0).unwrap();
        assert_eq!(ids(&t), [3, 1, 2, 4]);
        assert_eq!(t.marker(), 3);
        // Landing right at the marker keeps the step's side: 1 2 3 | 4.
        t.move_to(3, 2).unwrap();
        assert_eq!(ids(&t), [1, 2, 3, 4]);
        assert_eq!(t.marker(), 3);
        // A running step moved behind rolled-back steps is rolled back.
        t.move_to(3, 3).unwrap(); // 1 2 | 4 3
        assert_eq!(ids(&t), [1, 2, 4, 3]);
        assert_eq!(t.marker(), 2);
    }

    #[test]
    fn removing_with_dependents_takes_the_chain() {
        let mut t = abc();
        assert_eq!(t.dependents(1), [2, 4]);
        let removed = t.remove_with_dependents(1).unwrap();
        assert_eq!(removed.len(), 3);
        assert_eq!(ids(&t), [3]);
        assert_eq!(t.marker(), 1);
    }

    #[test]
    fn removing_alone_leaves_dependents() {
        let mut t = abc();
        t.remove(2).unwrap();
        assert_eq!(ids(&t), [1, 3, 4]);
        assert_eq!(t.marker(), 3);
        // A stored timeline with a missing dependency still loads.
        let steps = t.steps().to_vec();
        assert!(Timeline::from_steps(steps, 3).is_ok());
    }

    #[test]
    fn replace_checks_new_dependencies() {
        let mut t = abc();
        assert_eq!(
            t.replace(toy(1, &[3])),
            Err(EditError::WouldPrecedeDependency {
                step: 1,
                dependency: 3
            })
        );
        let old = t.replace(toy(3, &[2])).unwrap();
        assert!(old.deps.is_empty());
        assert_eq!(t.get(3).unwrap().op().deps, [2]);
    }

    #[test]
    fn stored_order_is_checked() {
        let steps = vec![
            Step::new(Arc::new(toy(2, &[1])), false),
            Step::new(Arc::new(toy(1, &[])), false),
        ];
        assert!(matches!(
            Timeline::from_steps(steps, 0),
            Err(EditError::WouldPrecedeDependency { .. })
        ));
    }
}
