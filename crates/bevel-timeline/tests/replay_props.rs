// SPDX-License-Identifier: GPL-3.0-or-later

//! Random edit sequences: the timeline never breaks its ordering invariant,
//! and a replay through a long-lived cache always equals a fresh replay.

#![allow(clippy::unwrap_used, reason = "tests")]

use bevel_timeline::{
    Applied, Digest, DigestBuilder, Evaluator, Replay, ReplayCache, Timeline, TimelineOp, replay,
};
use proptest::prelude::*;
use std::collections::BTreeSet;

#[derive(Clone, Debug)]
struct Op {
    id: u32,
    deps: Vec<u32>,
    value: i64,
    fail: bool,
}

impl TimelineOp for Op {
    type Id = u32;
    fn id(&self) -> u32 {
        self.id
    }
    fn deps(&self) -> Vec<u32> {
        self.deps.clone()
    }
    fn content_hash(&self) -> Digest {
        let mut b = DigestBuilder::new("op");
        b.u64(u64::from(self.id))
            .u64(self.value.cast_unsigned())
            .u64(u64::from(self.fail));
        for d in &self.deps {
            b.u64(u64::from(*d));
        }
        b.finish()
    }
}

/// State: the ids that ran, in order, and a checksum.
struct Eval;

impl Evaluator<Op> for Eval {
    type State = (Vec<u32>, i64);
    type Diagnostic = u32;
    fn initial_state(&self) -> Self::State {
        (Vec::new(), 0)
    }
    fn apply(&self, state: &Self::State, op: &Op) -> Result<Applied<Self::State, u32>, u32> {
        if op.fail {
            return Err(op.id);
        }
        let mut ran = state.0.clone();
        ran.push(op.id);
        Ok(Applied::ok((
            ran,
            state.1.wrapping_mul(31).wrapping_add(op.value),
        )))
    }
}

#[derive(Clone, Debug)]
enum Edit {
    Insert {
        deps: Vec<prop::sample::Index>,
        value: i64,
        fail: bool,
    },
    Replace {
        at: prop::sample::Index,
        value: i64,
        fail: bool,
    },
    Move {
        at: prop::sample::Index,
        to: prop::sample::Index,
    },
    Remove {
        at: prop::sample::Index,
        cascade: bool,
    },
    Suppress {
        at: prop::sample::Index,
        on: bool,
    },
    Marker {
        to: prop::sample::Index,
    },
}

fn edit() -> impl Strategy<Value = Edit> {
    use prop::sample::Index;
    prop_oneof![
        4 => (prop::collection::vec(any::<Index>(), 0..3), -5i64..5, prop::bool::weighted(0.15))
            .prop_map(|(deps, value, fail)| Edit::Insert { deps, value, fail }),
        2 => (any::<Index>(), -5i64..5, prop::bool::weighted(0.2))
            .prop_map(|(at, value, fail)| Edit::Replace { at, value, fail }),
        2 => (any::<Index>(), any::<Index>()).prop_map(|(at, to)| Edit::Move { at, to }),
        1 => (any::<Index>(), any::<bool>()).prop_map(|(at, cascade)| Edit::Remove { at, cascade }),
        1 => (any::<Index>(), any::<bool>()).prop_map(|(at, on)| Edit::Suppress { at, on }),
        1 => any::<Index>().prop_map(|to| Edit::Marker { to }),
    ]
}

fn apply(t: &mut Timeline<Op>, next_id: &mut u32, e: &Edit) {
    let ids: Vec<u32> = t.steps().iter().map(|s| s.op().id).collect();
    let pick = |i: &prop::sample::Index| ids[i.index(ids.len())];
    match e {
        Edit::Insert { deps, value, fail } => {
            let deps: BTreeSet<u32> = if ids.is_empty() {
                BTreeSet::new()
            } else {
                deps.iter().map(pick).collect()
            };
            *next_id += 1;
            // May be refused (a dependency is rolled back); that's fine.
            let _ = t.insert(Op {
                id: *next_id,
                deps: deps.into_iter().collect(),
                value: *value,
                fail: *fail,
            });
        }
        _ if ids.is_empty() => {}
        Edit::Replace { at, value, fail } => {
            let old = t.get(pick(at)).unwrap().op().as_ref().clone();
            t.replace(Op {
                value: *value,
                fail: *fail,
                ..old
            })
            .unwrap();
        }
        Edit::Move { at, to } => {
            let _ = t.move_to(pick(at), to.index(ids.len()));
        }
        Edit::Remove { at, cascade } => {
            if *cascade {
                t.remove_with_dependents(pick(at)).unwrap();
            } else {
                t.remove(pick(at)).unwrap();
            }
        }
        Edit::Suppress { at, on } => t.set_suppressed(pick(at), *on).unwrap(),
        Edit::Marker { to } => t.set_marker(to.index(ids.len() + 1)).unwrap(),
    }
}

fn check_order(t: &Timeline<Op>) {
    let present: BTreeSet<u32> = t.steps().iter().map(|s| s.op().id).collect();
    let mut seen = BTreeSet::new();
    for s in t.steps() {
        for d in &s.op().deps {
            assert!(
                !present.contains(d) || seen.contains(d),
                "{} before its dependency {d}",
                s.op().id
            );
        }
        seen.insert(s.op().id);
    }
    assert!(t.marker() <= t.len());
}

type R = Replay<u32, (Vec<u32>, i64), u32>;

fn same(a: &R, b: &R) {
    assert_eq!(a.state, b.state);
    assert_eq!(a.statuses, b.statuses);
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        rng_seed: prop::test_runner::RngSeed::Fixed(0x7133_11E0),
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn cached_replay_equals_fresh_replay(edits in prop::collection::vec(edit(), 1..40), budget in 1usize..12) {
        let mut t = Timeline::new();
        let mut next_id = 0;
        let mut cache = ReplayCache::new(budget);
        for e in &edits {
            apply(&mut t, &mut next_id, e);
            check_order(&t);
            let cached = replay(&t, &Eval, &mut cache, &|| false).unwrap();
            let fresh = replay(&t, &Eval, &mut ReplayCache::new(0), &|| false).unwrap();
            same(&cached, &fresh);
            // Only steps whose inputs all ran contribute to the state.
            for (id, status) in &cached.statuses {
                prop_assert_eq!(status.ran(), cached.state.0.contains(id));
            }
        }
    }
}
