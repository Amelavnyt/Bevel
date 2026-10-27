// SPDX-License-Identifier: GPL-3.0-or-later

#![allow(clippy::unwrap_used, missing_docs, reason = "tests")]

use std::f64::consts::{FRAC_PI_4, FRAC_PI_6, PI};

use bevel_expr::{
    AngleUnit, Dim, EvalContext, Expr, ExprError, ExprErrorKind, LengthUnit, ParamError, ParamKind,
    Parameter, ParameterTable, Quantity, Units, eval_as, format_quantity,
};
use proptest::prelude::*;

const MM_RAD: Units = Units {
    length: LengthUnit::Millimetre,
    angle: AngleUnit::Radian,
};

fn none(_: &str) -> Option<Quantity> {
    None
}

fn run_with(src: &str, units: Units) -> Result<Quantity, ExprError> {
    Expr::parse(src)?.eval(&EvalContext {
        units,
        lookup: &none,
    })
}

fn run(src: &str) -> Result<Quantity, ExprError> {
    run_with(src, Units::default())
}

fn ok(src: &str) -> Quantity {
    run(src).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * (1.0 + b.abs())
}

fn assert_q(src: &str, value: f64, dim: Dim) {
    let q = ok(src);
    assert_eq!(q.dim, dim, "{src}");
    assert!(close(q.value, value), "{src}: {} != {value}", q.value);
}

fn kind(src: &str) -> (std::ops::Range<usize>, ExprErrorKind) {
    let e = run(src).unwrap_err();
    (e.span, e.kind)
}

#[test]
fn arithmetic_and_units() {
    assert_q("10 mm + 2 cm", 30.0, Dim::LENGTH);
    assert_q("1 in", 25.4, Dim::LENGTH);
    assert_q("1 ft", 304.8, Dim::LENGTH);
    assert_q("1 m + 5 um", 1000.005, Dim::LENGTH);
    assert_q("2 * 3 mm", 6.0, Dim::LENGTH);
    assert_q("(2 mm)^2", 4.0, Dim::AREA);
    assert_q("10 mm ^ 2", 100.0, Dim::AREA);
    assert_q("-2^2", -4.0, Dim::UNITLESS);
    assert_q("2^3^2", 512.0, Dim::UNITLESS);
    assert_q("2^-1", 0.5, Dim::UNITLESS);
    assert_q("1e3 + .5 + 2.5E-2", 1000.525, Dim::UNITLESS);
    assert_q("+3 - -2", 5.0, Dim::UNITLESS);
    assert_q("pi", PI, Dim::UNITLESS);
    assert_q("180 deg", PI, Dim::ANGLE);
    assert_q("1 rad", 1.0, Dim::ANGLE);
    assert_q("6 mm / 2 mm", 3.0, Dim::UNITLESS);
    assert_q("6 mm * 2 mm", 12.0, Dim::AREA);
    assert_q("2 mm * 2 mm * 2 mm", 8.0, Dim::VOLUME);
}

#[test]
fn unit_binds_to_literal_first() {
    // `16 mm^2` is (16 mm)^2 = 256 mm^2, whose root is 16 mm.
    assert_q("16 mm^2", 256.0, Dim::AREA);
    assert_q("sqrt(16 mm^2)", 16.0, Dim::LENGTH);
    assert_q("sqrt(16 mm * 1 mm)", 4.0, Dim::LENGTH);
}

#[test]
fn functions() {
    assert_q("sin(30)", 0.5, Dim::UNITLESS);
    assert!(close(run_with("sin(30 deg)", MM_RAD).unwrap().value, 0.5));
    assert!(close(run_with("sin(pi / 2)", MM_RAD).unwrap().value, 1.0));
    assert_q("cos(60 deg)", 0.5, Dim::UNITLESS);
    assert_q("tan(45)", 1.0, Dim::UNITLESS);
    assert_q("atan2(1 mm, 1 mm)", FRAC_PI_4, Dim::ANGLE);
    assert_q("asin(0.5)", FRAC_PI_6, Dim::ANGLE);
    assert_q("acos(1)", 0.0, Dim::ANGLE);
    assert_q("atan(1)", FRAC_PI_4, Dim::ANGLE);
    assert_q("abs(-3 mm)", 3.0, Dim::LENGTH);
    assert_q("min(3 mm, 1 cm, 2 mm)", 2.0, Dim::LENGTH);
    assert_q("max(3, 1, 2)", 3.0, Dim::UNITLESS);
    assert_q("floor(2.7)", 2.0, Dim::UNITLESS);
    assert_q("ceil(2.1)", 3.0, Dim::UNITLESS);
    assert_q("round(2.5)", 3.0, Dim::UNITLESS);
    assert_q("sqrt(9)", 3.0, Dim::UNITLESS);
}

#[test]
fn rounding_uses_default_unit() {
    let inch = Units {
        length: LengthUnit::Inch,
        angle: AngleUnit::Degree,
    };
    let q = run_with("round(1.26 in)", inch).unwrap();
    assert!(close(q.value, 25.4));
    let q = run_with("round(1.26 in)", Units::default()).unwrap();
    assert!(close(q.value, 32.0)); // 32.004 mm rounds to 32 mm
    let q = run_with("floor(44.9 deg)", Units::default()).unwrap();
    assert!(close(q.value, 44f64.to_radians()));
}

#[test]
fn error_spans() {
    let (span, k) = kind("10 mm + 30 deg");
    assert_eq!(span, 0..14);
    assert!(matches!(k, ExprErrorKind::InvalidOperand(_)));
    assert_eq!(
        run("10 mm + 30 deg").unwrap_err().to_string(),
        "cannot add length and angle"
    );
    assert_eq!(kind("2 * (1 mm + 1)").0, 4..14);

    let (span, k) = kind("width * 2");
    assert_eq!(span, 0..5);
    assert_eq!(k, ExprErrorKind::UnknownParameter("width".into()));
    assert_eq!(
        run("widht").unwrap_err().to_string(),
        "unknown parameter `widht`"
    );

    assert_eq!(kind("1 / 0"), (0..5, ExprErrorKind::DivisionByZero));
    assert_eq!(kind("2 +"), (3..3, ExprErrorKind::UnexpectedEnd));
    assert_eq!(
        kind("foo(1)"),
        (0..3, ExprErrorKind::UnknownFunction("foo".into()))
    );
    let (span, k) = kind("min(1)");
    assert_eq!(span, 0..6);
    assert!(matches!(k, ExprErrorKind::WrongArgCount { found: 1, .. }));
    assert!(matches!(
        kind("atan2(1)").1,
        ExprErrorKind::WrongArgCount { .. }
    ));
    assert!(matches!(
        kind("sin(1, 2)").1,
        ExprErrorKind::WrongArgCount { .. }
    ));
    assert_eq!(kind("2 $ 3"), (2..3, ExprErrorKind::UnexpectedChar('$')));
    assert_eq!(run("(2").unwrap_err().kind, ExprErrorKind::UnexpectedEnd);
    assert_eq!(
        run("(2 3").unwrap_err().to_string(),
        "expected `)`, found `3`"
    );
    assert!(matches!(
        kind("2 3").1,
        ExprErrorKind::UnexpectedToken { .. }
    ));
    assert!(matches!(
        kind("1 +* 2").1,
        ExprErrorKind::UnexpectedToken { .. }
    ));
    assert!(matches!(kind("").1, ExprErrorKind::UnexpectedEnd));
}

#[test]
fn dimension_errors() {
    assert!(matches!(
        kind("1 mm + 1").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("2 ^ (1 mm)").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("(1 mm) ^ 0.5").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("((1 mm) ^ 100) ^ 2").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert_eq!(
        run("sqrt(4 mm)").unwrap_err().to_string(),
        "sqrt needs an even power of length"
    );
    assert!(matches!(kind("sqrt(-1)").1, ExprErrorKind::Domain(_)));
    assert!(matches!(kind("asin(2)").1, ExprErrorKind::Domain(_)));
    assert!(matches!(kind("acos(-2)").1, ExprErrorKind::Domain(_)));
    assert!(matches!(
        kind("sin(1 mm)").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("asin(1 mm)").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("atan2(1 mm, 1)").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(
        kind("min(1 mm, 1)").1,
        ExprErrorKind::InvalidOperand(_)
    ));
    assert!(matches!(kind("(-8) ^ 0.5").1, ExprErrorKind::Domain(_)));
    assert_eq!(kind("0 ^ -1").1, ExprErrorKind::DivisionByZero);
    assert_eq!(kind("10 ^ 400").1, ExprErrorKind::NonFinite);
    assert_eq!(kind("1e999").1, ExprErrorKind::NonFinite);
}

#[test]
fn deep_nesting_is_refused() {
    let src = "(".repeat(300);
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("{}1{}", "(".repeat(300), ")".repeat(300));
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("{}1", "-".repeat(300));
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("{}1{}", "sin(".repeat(300), ")".repeat(300));
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("1{}", "^1".repeat(300));
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("1{}", "+1".repeat(100_000));
    assert_eq!(run(&src).unwrap_err().kind, ExprErrorKind::TooDeep);
    let src = format!("{}1{}", "(".repeat(200), ")".repeat(200));
    assert_q(&src, 1.0, Dim::UNITLESS);
}

#[test]
fn eval_as_applies_default_units() {
    let inch = Units {
        length: LengthUnit::Inch,
        angle: AngleUnit::Degree,
    };
    let cx = |units| EvalContext {
        units,
        lookup: &none,
    };
    let e = Expr::parse("40").unwrap();
    let q = eval_as(&e, Dim::LENGTH, &cx(inch)).unwrap();
    assert!(close(q.value, 1016.0));
    assert_eq!(q.dim, Dim::LENGTH);
    let q = eval_as(&e, Dim::LENGTH, &cx(Units::default())).unwrap();
    assert!(close(q.value, 40.0));

    let e = Expr::parse("30").unwrap();
    let q = eval_as(&e, Dim::ANGLE, &cx(Units::default())).unwrap();
    assert!(close(q.value, FRAC_PI_6));
    assert_eq!(q.dim, Dim::ANGLE);
    let q = eval_as(&e, Dim::UNITLESS, &cx(Units::default())).unwrap();
    assert!(close(q.value, 30.0));

    let e = Expr::parse("10 deg").unwrap();
    let err = eval_as(&e, Dim::LENGTH, &cx(Units::default())).unwrap_err();
    assert_eq!(err.span, 0..6);
    assert_eq!(
        err.kind,
        ExprErrorKind::DimensionMismatch {
            expected: Dim::LENGTH,
            found: Dim::ANGLE
        }
    );
    let e = Expr::parse("10 mm").unwrap();
    assert!(eval_as(&e, Dim::UNITLESS, &cx(Units::default())).is_err());
    assert!(eval_as(&e, Dim::AREA, &cx(Units::default())).is_err());
}

#[test]
fn identifiers_and_lookup() {
    let e = Expr::parse("b * 2 + a + b + sin(c) + pi").unwrap();
    let ids: Vec<_> = e.identifiers().into_iter().collect();
    assert_eq!(ids, ["a", "b", "c"]);
    let lookup = |n: &str| match n {
        "a" => Some(Quantity::length(1.0)),
        "b" => Some(Quantity::length(2.0)),
        _ => None,
    };
    let cx = EvalContext {
        units: Units::default(),
        lookup: &lookup,
    };
    let e = Expr::parse("a + b * 2").unwrap();
    assert!(close(e.eval(&cx).unwrap().value, 5.0));
    assert_eq!(e.source(), "a + b * 2");
    // Parameters named like units are fine.
    let e = Expr::parse("m + in").unwrap();
    assert_eq!(e.identifiers().len(), 2);
    // `m` right after a number is a unit.
    assert!(Expr::parse("2 m").unwrap().identifiers().is_empty());
}

fn param(name: &str, expr: &str) -> Parameter {
    Parameter {
        name: name.into(),
        expression: expr.into(),
        comment: String::new(),
        kind: ParamKind::User,
    }
}

fn table(items: &[(&str, &str)]) -> ParameterTable {
    let mut t = ParameterTable::new();
    for (n, e) in items {
        t.add(param(n, e)).unwrap();
    }
    t
}

#[test]
fn parameter_chain() {
    let t = table(&[("c", "b + a"), ("b", "a * 2"), ("a", "10 mm")]);
    let ev = t.evaluate(Units::default());
    assert_eq!(ev.value("a"), Some(Quantity::length(10.0)));
    assert_eq!(ev.value("b"), Some(Quantity::length(20.0)));
    assert_eq!(ev.value("c"), Some(Quantity::length(30.0)));
    assert_eq!(ev.iter().count(), 3);
    assert!(ev.get("zzz").is_none());
    let order: Vec<_> = t.params().iter().map(|p| p.name.as_str()).collect();
    assert_eq!(order, ["c", "b", "a"]);
    assert_eq!(
        t.dependents("a").into_iter().collect::<Vec<_>>(),
        ["b", "c"]
    );
    assert!(t.dependents("c").is_empty());
}

#[test]
fn bare_number_parameter_is_unitless() {
    let t = table(&[("n", "40")]);
    let ev = t.evaluate(Units::default());
    assert_eq!(ev.value("n"), Some(Quantity::unitless(40.0)));
}

#[test]
fn cycles_and_failures() {
    let t = table(&[
        ("x", "y"),
        ("y", "x"),
        ("z", "x + 1 mm"),
        ("s", "s + 1"),
        ("ok", "1 mm"),
        ("bad", "1 +"),
        ("w", "bad * 2"),
        ("u", "nope"),
    ]);
    let ev = t.evaluate(Units::default());
    let cyc = |n: &str| match &ev.get(n).unwrap().as_ref().unwrap_err().kind {
        ExprErrorKind::Cycle(c) => c.clone(),
        other => panic!("{n}: {other:?}"),
    };
    assert_eq!(cyc("x"), ["x", "y"]);
    assert_eq!(cyc("y"), ["x", "y"]);
    assert_eq!(cyc("s"), ["s"]);
    let z = ev.get("z").unwrap().as_ref().unwrap_err();
    assert_eq!(z.kind, ExprErrorKind::DependsOnError("x".into()));
    assert_eq!(z.span, 0..1);
    assert_eq!(ev.value("ok"), Some(Quantity::length(1.0)));
    assert_eq!(
        ev.get("bad").unwrap().as_ref().unwrap_err().kind,
        ExprErrorKind::UnexpectedEnd
    );
    assert_eq!(
        ev.get("w").unwrap().as_ref().unwrap_err().kind,
        ExprErrorKind::DependsOnError("bad".into())
    );
    assert_eq!(
        ev.get("u").unwrap().as_ref().unwrap_err().kind,
        ExprErrorKind::UnknownParameter("nope".into())
    );
}

#[test]
fn three_cycle_order() {
    let t = table(&[("c", "a"), ("a", "b"), ("b", "c")]);
    let ev = t.evaluate(Units::default());
    for n in ["a", "b", "c"] {
        assert_eq!(
            ev.get(n).unwrap().as_ref().unwrap_err().kind,
            ExprErrorKind::Cycle(vec!["a".into(), "b".into(), "c".into()])
        );
    }
}

#[test]
fn rename_rewrites_text() {
    let mut t = table(&[
        ("a", "10 mm"),
        ("b", "a * 2"),
        ("c", "max(a,  b)+a  /* ? */"),
        ("d", "2 a"),
        ("m", "3 m + a"),
    ]);
    t.rename("a", "width").unwrap();
    assert_eq!(t.get("b").unwrap().expression, "width * 2");
    // Unparsable text is untouched.
    assert_eq!(t.get("c").unwrap().expression, "max(a,  b)+a  /* ? */");
    assert_eq!(t.get("d").unwrap().expression, "2 a");
    // The unit `m` is not an identifier.
    assert_eq!(t.get("m").unwrap().expression, "3 m + width");
    assert!(t.get("a").is_none());
    assert!(t.get("width").is_some());
    assert_eq!(t.params()[0].name, "width");

    t.rename("m", "k").unwrap();
    assert_eq!(t.get("m"), None);
    assert!(matches!(t.rename("zz", "q"), Err(ParamError::NotFound(_))));
    assert!(matches!(
        t.rename("b", "width"),
        Err(ParamError::Duplicate(_))
    ));
    assert!(matches!(
        t.rename("b", "1x"),
        Err(ParamError::InvalidName(_))
    ));
    assert!(matches!(
        t.rename("b", "pi"),
        Err(ParamError::InvalidName(_))
    ));

    let mut t = table(&[("a", "1"), ("b", "a+a+sin(a)")]);
    t.rename("a", "ab").unwrap();
    assert_eq!(t.get("b").unwrap().expression, "ab+ab+sin(ab)");
}

#[test]
fn remove_and_names() {
    let mut t = table(&[("a", "1"), ("b", "a + 1"), ("s", "s")]);
    match t.remove("a") {
        Err(ParamError::InUse { name, users }) => {
            assert_eq!(name, "a");
            assert_eq!(users, ["b"]);
        }
        other => panic!("{other:?}"),
    }
    assert!(t.remove("s").is_ok());
    assert!(t.remove("b").is_ok());
    assert_eq!(t.remove("a").unwrap().name, "a");
    assert!(matches!(t.remove("a"), Err(ParamError::NotFound(_))));

    let mut t = table(&[("d1", "1"), ("d3", "1")]);
    assert_eq!(t.next_model_name(), "d2");
    t.add(param("d2", "1")).unwrap();
    assert_eq!(t.next_model_name(), "d4");

    for bad in ["", "1a", "a b", "pi", "sin", "x-y", "é"] {
        assert!(
            matches!(t.add(param(bad, "1")), Err(ParamError::InvalidName(_))),
            "{bad}"
        );
    }
    assert!(matches!(
        t.add(param("d1", "2")),
        Err(ParamError::Duplicate(_))
    ));
    t.add(param("m", "1")).unwrap();
    t.set_expression("m", "2 mm".into()).unwrap();
    t.set_comment("m", "hi".into()).unwrap();
    assert_eq!(t.get("m").unwrap().expression, "2 mm");
    assert_eq!(t.get("m").unwrap().comment, "hi");
    assert!(t.set_expression("nope", String::new()).is_err());
    assert!(t.set_comment("nope", String::new()).is_err());
}

#[test]
fn formatting() {
    let u = Units::default();
    assert_eq!(format_quantity(Quantity::length(40.0), u, 2), "40.00 mm");
    assert_eq!(
        format_quantity(Quantity::angle(30f64.to_radians()), u, 1),
        "30.0 deg"
    );
    let area = ok("40 mm * 40 mm");
    assert_eq!(format_quantity(area, u, 2), "1600.00 mm^2");
    assert_eq!(format_quantity(ok("2 mm * 2 mm * 2 mm"), u, 0), "8 mm^3");
    assert_eq!(format_quantity(Quantity::unitless(3.0), u, 1), "3.0");
    assert_eq!(format_quantity(Quantity::length(-0.0), u, 2), "0.00 mm");
    assert_eq!(format_quantity(Quantity::length(-1.5), u, 1), "-1.5 mm");
}

#[test]
fn table_serde_round_trip() {
    let t = table(&[("a", "10 mm"), ("b", "a * 2")]);
    let json = serde_json::to_string(&t).unwrap();
    let back: ParameterTable = serde_json::from_str(&json).unwrap();
    assert_eq!(t, back);
    let u = Units {
        length: LengthUnit::Inch,
        angle: AngleUnit::Radian,
    };
    let back: Units = serde_json::from_str(&serde_json::to_string(&u).unwrap()).unwrap();
    assert_eq!(u, back);
}

fn expr_strategy() -> impl Strategy<Value = String> {
    let leaf = prop_oneof![
        (
            0u32..1000,
            prop::sample::select(vec!["", "mm", "cm", "in", "deg", "rad", "ft"])
        )
            .prop_map(|(n, u)| format!("{n} {u}")),
        Just("pi".to_owned()),
        Just("0".to_owned()),
    ];
    leaf.prop_recursive(5, 48, 3, |inner| {
        prop_oneof![
            (
                inner.clone(),
                prop::sample::select(vec!["+", "-", "*", "/", "^"]),
                inner.clone()
            )
                .prop_map(|(a, o, b)| format!("({a}) {o} ({b})")),
            inner.clone().prop_map(|a| format!("-({a})")),
            (
                prop::sample::select(vec![
                    "sin", "cos", "tan", "asin", "acos", "atan", "sqrt", "abs", "round", "floor",
                    "ceil"
                ]),
                inner.clone()
            )
                .prop_map(|(f, a)| format!("{f}({a})")),
            (inner.clone(), inner).prop_map(|(a, b)| format!("min({a}, {b})")),
        ]
    })
}

proptest! {
    #[test]
    fn well_formed_never_panics(src in expr_strategy()) {
        let e = Expr::parse(&src).unwrap();
        match run(&src) {
            Ok(q) => prop_assert!(q.value.is_finite()),
            Err(err) => {
                prop_assert!(err.span.start <= err.span.end);
                prop_assert!(err.span.end <= src.len());
            }
        }
        prop_assert_eq!(e.source(), src.as_str());
    }

    #[test]
    fn arbitrary_text_never_panics(src in ".{0,64}") {
        if let Err(err) = Expr::parse(&src) {
            prop_assert!(err.span.start <= err.span.end);
            prop_assert!(err.span.end <= src.len());
            prop_assert!(src.is_char_boundary(err.span.start));
            prop_assert!(src.is_char_boundary(err.span.end));
        }
    }
}
