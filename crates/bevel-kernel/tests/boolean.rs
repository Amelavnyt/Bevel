// SPDX-License-Identifier: GPL-3.0-or-later

//! Boolean operations on primitives: the named edge cases and volume
//! identities.

#![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

use bevel_kernel::{
    Body, BooleanOp, BoxParams, CylinderParams, OpContext, OpTag, SphereParams, boolean, box_,
    cylinder, mass_properties, sphere, validate,
};
use bevel_math::{Frame3, Point3, Vector3};
use std::f64::consts::PI;
use std::sync::Arc;

fn block(origin: [f64; 3], size: [f64; 3], tag: u64) -> Arc<Body> {
    let params = BoxParams {
        frame: Frame3 {
            origin: Point3::from_array(origin),
            ..Frame3::WORLD
        },
        size: Vector3::from_array(size),
    };
    box_(&params, OpTag(tag), &mut OpContext::new())
        .unwrap()
        .bodies[0]
        .clone()
}

fn volume(bodies: &[Arc<Body>]) -> f64 {
    bodies
        .iter()
        .map(|b| mass_properties(b, &OpContext::new()).unwrap().volume)
        .sum()
}

fn run(a: &Body, b: &Body, op: BooleanOp) -> Vec<Arc<Body>> {
    let out = boolean(a, b, op, OpTag(99), &mut OpContext::new())
        .unwrap_or_else(|e| panic!("{op:?}: {e}"));
    for body in &out.bodies {
        assert!(validate(body).is_empty(), "{op:?}: {:?}", validate(body));
    }
    out.bodies
}

#[test]
fn overlapping_boxes() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    let b = block([5.0, 5.0, 5.0], [10.0, 10.0, 10.0], 2);
    let u = volume(&run(&a, &b, BooleanOp::Union));
    let s = volume(&run(&a, &b, BooleanOp::Subtract));
    let i = volume(&run(&a, &b, BooleanOp::Intersect));
    assert!((i - 125.0).abs() < 1e-6, "intersect {i}");
    assert!((s - 875.0).abs() < 1e-6, "subtract {s}");
    assert!((u - 1875.0).abs() < 1e-6, "union {u}");
}

fn cylinder_at(base: [f64; 3], radius: f64, height: f64, tag: u64) -> Arc<Body> {
    let params = CylinderParams {
        frame: Frame3 {
            origin: Point3::from_array(base),
            ..Frame3::WORLD
        },
        radius,
        height,
    };
    cylinder(&params, OpTag(tag), &mut OpContext::new())
        .unwrap()
        .bodies[0]
        .clone()
}

fn sphere_at(centre: [f64; 3], radius: f64, tag: u64) -> Arc<Body> {
    let params = SphereParams {
        frame: Frame3 {
            origin: Point3::from_array(centre),
            ..Frame3::WORLD
        },
        radius,
    };
    sphere(&params, OpTag(tag), &mut OpContext::new())
        .unwrap()
        .bodies[0]
        .clone()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-6 * b.abs().max(1.0)
}

#[test]
fn box_with_a_hole() {
    let a = block([0.0; 3], [20.0, 20.0, 10.0], 1);
    let c = cylinder_at([10.0, 10.0, -5.0], 4.0, 20.0, 2);
    let out = run(&a, &c, BooleanOp::Subtract);
    assert_eq!(out.len(), 1);
    let expected = 4000.0 - PI * 16.0 * 10.0;
    assert!(
        close(volume(&out), expected),
        "{} vs {expected}",
        volume(&out)
    );
    let shell = out[0].shell_keys()[0];
    assert_eq!(
        out[0].euler_counts(shell).unwrap().genus(),
        Some(1),
        "a through hole"
    );
    let i = volume(&run(&a, &c, BooleanOp::Intersect));
    assert!(close(i, PI * 16.0 * 10.0), "{i}");
}

#[test]
fn coplanar_faces() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    let b = block([5.0, 5.0, 0.0], [10.0, 10.0, 10.0], 2);
    assert!(close(volume(&run(&a, &b, BooleanOp::Union)), 1750.0));
    assert!(close(volume(&run(&a, &b, BooleanOp::Subtract)), 750.0));
    assert!(close(volume(&run(&a, &b, BooleanOp::Intersect)), 250.0));
}

#[test]
fn boxes_sharing_a_face() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    let b = block([10.0, 0.0, 0.0], [10.0, 10.0, 10.0], 2);
    let u = run(&a, &b, BooleanOp::Union);
    assert_eq!(u.len(), 1, "one body");
    assert!(close(volume(&u), 2000.0));
    assert!(close(volume(&run(&a, &b, BooleanOp::Subtract)), 1000.0));
    assert!(run(&a, &b, BooleanOp::Intersect).is_empty());
}

#[test]
fn boxes_touching_along_an_edge_and_at_a_corner() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    for b in [
        block([10.0, 10.0, 0.0], [10.0, 10.0, 10.0], 2),
        block([10.0, 10.0, 10.0], [10.0, 10.0, 10.0], 3),
    ] {
        let u = run(&a, &b, BooleanOp::Union);
        assert_eq!(u.len(), 2, "touching bodies stay apart");
        assert!(close(volume(&u), 2000.0));
        assert!(close(volume(&run(&a, &b, BooleanOp::Subtract)), 1000.0));
        assert!(run(&a, &b, BooleanOp::Intersect).is_empty());
    }
}

#[test]
fn coaxial_cylinders() {
    let c = cylinder_at([0.0; 3], 5.0, 10.0, 1);
    let d = cylinder_at([0.0, 0.0, 5.0], 5.0, 10.0, 2);
    assert!(close(
        volume(&run(&c, &d, BooleanOp::Union)),
        PI * 25.0 * 15.0
    ));
    assert!(close(
        volume(&run(&c, &d, BooleanOp::Intersect)),
        PI * 25.0 * 5.0
    ));
    let thin = cylinder_at([0.0, 0.0, -5.0], 3.0, 20.0, 3);
    let tube = run(&c, &thin, BooleanOp::Subtract);
    assert!(close(volume(&tube), PI * 16.0 * 10.0));
}

#[test]
fn tangent_spheres() {
    let s = sphere_at([0.0; 3], 5.0, 1);
    let t = sphere_at([10.0, 0.0, 0.0], 5.0, 2);
    let one = 4.0 / 3.0 * PI * 125.0;
    let u = run(&s, &t, BooleanOp::Union);
    assert_eq!(u.len(), 2);
    assert!(close(volume(&u), 2.0 * one));
    assert!(close(volume(&run(&s, &t, BooleanOp::Subtract)), one));
    assert!(run(&s, &t, BooleanOp::Intersect).is_empty());
}

#[test]
fn sphere_through_a_box() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    let s = sphere_at([10.0, 5.0, 5.0], 4.0, 2);
    let half = 2.0 / 3.0 * PI * 64.0;
    assert!(close(
        volume(&run(&a, &s, BooleanOp::Subtract)),
        1000.0 - half
    ));
    assert!(close(volume(&run(&a, &s, BooleanOp::Union)), 1000.0 + half));
    assert!(close(volume(&run(&a, &s, BooleanOp::Intersect)), half));
}

#[test]
fn near_tolerance_slivers_never_give_broken_bodies() {
    let a = block([0.0; 3], [10.0, 10.0, 10.0], 1);
    for gap in [-3e-6, -5e-7, 5e-7, 3e-6] {
        let b = block([0.0, 0.0, 10.0 + gap], [10.0, 10.0, 10.0], 2);
        for op in [BooleanOp::Union, BooleanOp::Subtract, BooleanOp::Intersect] {
            if let Ok(out) = boolean(&a, &b, op, OpTag(99), &mut OpContext::new()) {
                for body in &out.bodies {
                    assert!(validate(body).is_empty(), "{op:?} {gap}");
                }
            }
        }
    }
}

mod props {
    use super::*;
    use bevel_math::{DQuat, DVec3};
    use proptest::prelude::*;
    use proptest::test_runner::{Config, RngSeed};
    use std::f64::consts::TAU;

    /// A placement at `at` turned by `q`.
    fn frame(at: [f64; 3], q: DQuat) -> Frame3 {
        Frame3 {
            origin: Point3::from_array(at),
            x: q * Vector3::X,
            y: q * Vector3::Y,
            z: q * Vector3::Z,
        }
    }

    /// A random primitive: 0 box, 1 cylinder, 2 sphere.
    fn primitive(kind: u8, f: Frame3, size: f64, tag: u64) -> Arc<Body> {
        let cx = &mut OpContext::new();
        let out = match kind {
            0 => box_(
                &BoxParams {
                    frame: f,
                    size: Vector3::new(size, size * 0.8, size * 1.3),
                },
                OpTag(tag),
                cx,
            ),
            1 => cylinder(
                &CylinderParams {
                    frame: f,
                    radius: size * 0.5,
                    height: size * 1.4,
                },
                OpTag(tag),
                cx,
            ),
            _ => sphere(
                &SphereParams {
                    frame: f,
                    radius: size * 0.6,
                },
                OpTag(tag),
                cx,
            ),
        };
        out.unwrap().bodies[0].clone()
    }

    /// Fixed seed; `BEVEL_PROPTEST_SEED` and `BEVEL_PROPTEST_CASES` override
    /// it for exploratory runs.
    fn config() -> Config {
        let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
        Config {
            cases: env("BEVEL_PROPTEST_CASES").map_or(64, |c| u32::try_from(c).unwrap_or(64)),
            rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0xB001_EA45)),
            failure_persistence: None,
            ..Config::default()
        }
    }

    fn rotation() -> impl Strategy<Value = DQuat> {
        (prop::array::uniform3(-1.0f64..1.0), 0.0f64..TAU).prop_map(|(a, angle)| {
            let axis = DVec3::from_array(a).try_normalize().unwrap_or(DVec3::Z);
            DQuat::from_axis_angle(axis, angle)
        })
    }

    proptest! {
        #![proptest_config(config())]

        #[test]
        fn volumes_add_up(
            ka in 0u8..3, kb in 0u8..3,
            at in prop::array::uniform3(-6.0..6.0f64),
            turn in rotation(),
            sa in 6.0..12.0f64, sb in 4.0..10.0f64,
        ) {
            if let Err(e) = identities(ka, kb, at, turn, sa, sb) {
                prop_assert!(false, "{}", e);
            }
        }
    }

    /// The volume identities for one placement, as an error message when
    /// they fail.
    fn identities(
        ka: u8,
        kb: u8,
        at: [f64; 3],
        turn: DQuat,
        sa: f64,
        sb: f64,
    ) -> Result<(), String> {
        let a = primitive(ka, Frame3::WORLD, sa, 1);
        let b = primitive(kb, frame(at, turn), sb, 2);
        let (va, vb) = (
            volume(std::slice::from_ref(&a)),
            volume(std::slice::from_ref(&b)),
        );
        let u = volume(&run(&a, &b, BooleanOp::Union));
        let s = volume(&run(&a, &b, BooleanOp::Subtract));
        let i = volume(&run(&a, &b, BooleanOp::Intersect));
        let scale = va.max(vb);
        if (u - (va + vb - i)).abs() > 1e-6 * scale {
            return Err(format!("union {u} vs {}", va + vb - i));
        }
        if (s - (va - i)).abs() > 1e-6 * scale {
            return Err(format!("subtract {s} vs {}", va - i));
        }
        if !(i >= -1e-9 && i <= va.min(vb) + 1e-6 * scale) {
            return Err(format!("intersect {i}"));
        }
        Ok(())
    }

    /// Placements that once failed, each for its own reason.
    #[test]
    #[allow(clippy::too_many_lines, reason = "a table of cases")]
    fn past_failures() {
        let q = DQuat::from_xyzw;
        let cases = [
            // An intersection curve through both poles of a sphere: it
            // must not be joined into one part across a pole.
            (
                2,
                2,
                [
                    -0.833_100_636_250_418_5,
                    3.399_078_553_151_119,
                    3.191_650_828_319_979,
                ],
                q(
                    -0.107_793_100_120_073_02,
                    -0.248_840_666_381_355_16,
                    -0.208_963_727_462_308_62,
                    0.939_570_716_298_909,
                ),
                6.354_876_716_213_283,
                4.869_021_698_270_052,
            ),
            // Edges whose pcurves bend in (u, v) more than their curves
            // in space: tessellation refined without end.
            (
                0,
                1,
                [0.0, 1.755_157_798_101_971_5, 0.0],
                q(
                    0.272_236_559_384_533_5,
                    0.0,
                    -0.322_390_785_006_406_44,
                    0.906_615_374_609_003_9,
                ),
                11.715_710_342_465_384,
                4.0,
            ),
            (
                1,
                2,
                [
                    -0.565_891_472_811_432_6,
                    3.867_239_889_586_163_5,
                    5.972_320_537_571_316,
                ],
                q(
                    0.759_121_484_080_614_4,
                    0.644_450_419_002_730_5,
                    0.078_765_346_401_172_57,
                    -0.047_055_818_564_275_95,
                ),
                9.203_320_704_033_175,
                4.0,
            ),
            // A branch leaving through a seam right next to a pole.
            (
                1,
                2,
                [
                    -0.565_891_472_811_432_6,
                    3.867_239_889_586_163_5,
                    5.972_320_537_571_316,
                ],
                q(
                    0.759_121_484_080_614_4,
                    0.644_450_419_002_730_5,
                    0.078_765_346_401_172_57,
                    -0.047_055_818_564_275_95,
                ),
                9.203_320_704_033_175,
                6.288_912_469_695_748,
            ),
            // A pcurve swinging round a pole within one edge span: the
            // volume integral missed part of it.
            (
                0,
                2,
                [5.666_150_312_032_847, -1.835_035_551_692_220_3, 0.0],
                q(
                    -0.548_134_424_648_046_6,
                    0.343_702_469_624_318_66,
                    0.756_531_656_535_432_1,
                    -0.095_273_908_021_316_83,
                ),
                6.0,
                7.303_064_886_984_16,
            ),
            // Pcurve ends near a pole, off in u by more than the node
            // tolerance: the face's regions were not traced.
            (
                2,
                1,
                [
                    -5.394_146_879_048_097,
                    1.315_935_194_363_1,
                    -3.626_121_921_501_262_3,
                ],
                q(
                    -0.015_853_824_070_026_96,
                    -0.041_624_238_320_157_39,
                    0.0,
                    -0.999_007_547_041_874_7,
                ),
                11.054_568_803_147_97,
                9.379_909_810_315_15,
            ),
            (
                2,
                1,
                [
                    -5.394_146_879_048_097,
                    1.315_935_194_363_1,
                    -3.626_121_921_501_262_3,
                ],
                q(
                    -0.014_996_103_671_952_959,
                    -0.039_372_292_158_537_62,
                    -0.014_452_004_643_627_154,
                    -0.999_007_547_041_874_7,
                ),
                11.054_568_803_147_97,
                9.379_909_810_315_15,
            ),
            // A cut a thousandth of a millimetre inside a circular edge.
            (
                1,
                1,
                [
                    0.376_175_277_905_398_34,
                    -3.970_662_692_415_346,
                    1.172_694_101_569_217_4,
                ],
                q(
                    -0.494_732_776_708_079_36,
                    -0.256_423_944_894_637_9,
                    0.614_654_959_150_882_2,
                    -0.558_288_027_210_518_5,
                ),
                8.146_527_414_675_989,
                7.749_057_599_548_871,
            ),
        ];
        for (ka, kb, at, turn, sa, sb) in cases {
            identities(ka, kb, at, turn, sa, sb)
                .unwrap_or_else(|e| panic!("{ka} {kb} {at:?}: {e}"));
        }
    }
}
