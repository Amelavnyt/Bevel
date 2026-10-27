# Kernel conventions

This document collects the conventions that every piece of geometry code in Bevel follows, plus the robustness log for the kernel. The overall design is in [ARCHITECTURE.md](ARCHITECTURE.md); the reasoning behind the main choices is in [DECISIONS.md](DECISIONS.md).

## Units

Internal length is the millimetre and angle is the radian. All geometry is `f64`; `f32` is used only for GPU buffers. Display units (mm, cm, m, in) affect only the UI and the default unit of expressions, so nothing in the kernel converts units.

## Tolerances

Tolerance constants live in `crates/bevel-math/src/tol.rs`:

| Constant | Value | Meaning |
|---|---|---|
| `LINEAR` | 1e-6 mm | Two points closer than this are the same point |
| `ANGULAR` | 1e-11 rad | Two directions closer than this are parallel |
| `MODEL_EXTENT` | 1e5 mm | Half-size of the modeling space; geometry outside `[-MODEL_EXTENT, MODEL_EXTENT]` on any axis, or non-finite, is rejected with a clear error |

Rules:

- Never compare floats with `==`. Use the helpers in `bevel_math::tol` (`eq_length`, `eq_point`, `parallel`, `in_model_space`, and the ones added later).
- Parametric tolerances are derived per curve or surface from `LINEAR` and the local parametric speed, not hard-coded.
- Edges and vertices carry their own tolerance, which is needed for imported STEP data. Healing may merge vertices within tolerance but must not widen a tolerance silently; a body that needs a larger tolerance than the operation allows is reported as an issue.
- Library code returns typed errors for out-of-range input; it never panics.

## NURBS first, analytic fast paths second

Every curve and surface is exactly representable as rational NURBS, and every algorithm has a correct general-NURBS implementation first. Analytic types (line, circle, ellipse, plane, cylinder, cone, sphere, torus) keep their identity so they can be intersected, measured and exported exactly. Analytic fast paths (plane-plane, plane-quadric, sphere-sphere, coaxial cases) are optional optimizations and each one is cross-checked against the general NURBS path in tests. Helices and offsets are not exactly NURBS: they are approximated within tolerance and the error bound is recorded.

## Operations

Kernel operations share one signature shape:

```rust
fn op(inputs, opts, tag: OpTag, cx: &mut OpContext) -> Result<OpOutput, KernelError>
```

This is a planned design. `OpOutput { bodies, history }` returns the resulting bodies and a map from each new entity to what it was generated or modified from. `OpContext` carries tolerances, a cancellation token and debug-dump settings. `OpTag` is an opaque `u64` supplied by the caller (`bevel-model` passes the `FeatureId`), so the kernel can name what it creates without knowing about features. Operations that cut or add through an intermediate volume also return that tool body, so feature patterns can reuse it. Long loops check the cancellation token.

## Boolean pipeline

Union, subtract and intersect (one target against many tools) run in these stages:

1. Broad phase: a face BVH yields candidate face pairs.
2. Face-face surface intersection produces curves and pcurves, clipped to each face's trimmed domain by 2D classification against the loops using robust predicates.
3. Imprint: edges and faces are split along the curves, using a 2D arrangement in each face's parameter space.
4. Classify each fragment as IN, OUT or ON the other body, using multi-ray point-in-solid tests with tolerance.
5. Select fragments for the operation and flip orientation where needed (subtract).
6. Sew the selected fragments into shells, merging same-domain faces (for example coplanar pieces) and redundant edges.
7. Heal and validate. On failure the operation returns `KernelError::Boolean { stage, .. }` together with a debug dump; it never returns a broken body.

Coplanar faces, contact along an edge or at a vertex, coaxial cylinders, tangent spheres and near-tolerance slivers are each a named test case from the first boolean commit.

## Persistent naming

Every face, edge and vertex carries a `TopoName` derived from how it was created: `Generated` (from an operation, a source and a role), `Modified` (a fragment of an older entity), `Merged` (a same-domain merge) or `Between` (an edge or vertex named by its adjacent faces). Fragment indices come from a canonical, deterministic geometric ordering so names are stable across rebuilds. `bevel-model` stores references as `EntityRef` (a `TopoName` plus a geometric hint) and resolves them by exact name, then by hint, then through the lineage, otherwise reporting the reference as lost. Kernel operations are required to produce names and a history map; an operation that cannot is incomplete. A regression suite covers edits that split or merge faces underneath dependent features.

## Robustness program

- Every kernel failure can dump its inputs (bodies, operation and options) to a `bevel-debug-*.json` file, which can be replayed with `bevel-cli kernel-repro <file>`.
- Property tests (`proptest`) apply random primitives under random transforms and check that results validate and that volume identities hold: `V(A union B) = V(A) + V(B) - V(A intersect B)` and `V(A - B) = V(A) - V(A intersect B)`.
- `validate(&Body)` is run on results in debug builds and tests. It checks closed 2-manifold shells, consistent orientation, loop closure, pcurve and 3D curve agreement, vertices on their edges, Euler-Poincare and, in debug and tests only, absence of self-intersections.
- Every kernel bug becomes a regression test and gets a row in the robustness log below.

## Robustness log

Add a row for each kernel bug, newest last. The regression test column names the test that fails without the fix.

| Date | Case | Cause | Fix | Regression test |
|---|---|---|---|---|
| 2026-10-26 | Knot refinement moved the curve | A5.4 port read `U[i-l+1]` instead of `U[i-p+l]` | Corrected the index | `refinement_matches_repeated_insertion` |
| 2026-10-26 | A rational quartic could not remove a knot it had just inserted | Removal bound mixed point and weight parts, overestimating the deviation | Track the point and weight parts separately | `removal_bound_keeps_point_and_weight_parts_apart` |
| 2026-10-26 | Closest point wrong next to a C0 corner | Newton refined a corner-knot seed in the wrong span | Keep each refinement in its seed's span and seed both sides of corners | `closest_point_refines_corner_seeds_on_both_sides` |
| 2026-10-26 | Closest point missed the near side of a quintic hairpin | Sampling stepped over the narrow turn | Branch-and-bound pass over Bézier segments with a sound lower bound | `closest_point_finds_the_far_side_of_a_hairpin` |
| 2026-10-26 | Arc length off by 5e-6 with a 1e-6 tolerance | 8- and 16-point Gauss rules agreed by accident on a steep speed profile | Also require the 16-point result to match the sum of its halves | `arc_length_is_not_fooled_by_agreeing_rules` |
| 2026-10-26 | Basis functions slightly negative one ulp past the domain end | Public basis functions did not clamp the parameter | Clamp to the domain | `basis_is_a_non_negative_partition_of_unity` |
| 2026-10-27 | Principal curvatures off by 5e-9 on a sphere of radius 2 | Solving k² − 2Hk + K loses half the digits near umbilic points | Closed-form eigen-decomposition of the second fundamental form in an orthonormal tangent basis | `curvatures_stay_accurate_near_an_umbilic_point` |
| 2026-10-27 | A NaN range end gave a valid-looking zero-length interval | `f64::min`/`max` ignore NaN in `Interval::new` | Propagate NaN to both ends | `nan_end_is_not_swallowed` |

## Algorithm notes

Conventions in `bevel-geom` that every caller relies on:

- **Homogeneous storage** ([ADR-0016](DECISIONS.md#adr-0016-nurbs-stored-in-homogeneous-coordinates-one-implementation-for-2d-and-3d)): control points are stored weighted, `(w·P, w)`, and weights must be finite and positive. Every algorithm is written once for 2D and 3D through the `Homogeneous` trait.
- **Knots** closer than `KnotVector::tolerance()` (1e-12 of the knot range, at least 4 ulps) are snapped together on construction, so multiplicities are well defined. Parameters outside a curve's or surface's domain are clamped.
- **Representation changes** (knot insertion, refinement and removal, degree elevation, Bézier decomposition, split, reparametrization) keep the geometry unchanged; degree reduction and knot removal report a rigorous error bound and stay within the requested tolerance.
- **Closest point** on curves and surfaces seeds by sampling every knot span or patch, refines with the Newton iteration of The NURBS Book section 6.1 (step-controlled to decrease the distance), and runs a branch-and-bound pass over Bézier segments or patches with a sound lower bound, so it never returns anything worse than sampling.
- **Principal curvatures** are signed relative to the surface normal: positive where the surface bends towards the normal. A sphere with an outward normal therefore reports `-1/r`. They are computed as the eigen-decomposition of the second fundamental form in an orthonormal tangent basis, which stays accurate near umbilic points.
- **Conics** use the angle as their parameter range: a circle or ellipse arc over `[start, end]` agrees with the angle at its knots. Between knots the NURBS parameter is not the angle; `ParamMap` converts between the two exactly, and `to_nurbs_mapped` returns the map with the converted geometry.
- **Analytic transforms**: rigid motions and uniform scaling keep analytic types. Under non-uniform scaling a circle becomes an ellipse; a sphere or torus becomes its exact NURBS form; a cylinder or cone stays analytic only if the map is conformal across its axis. Singular and (for analytic types) projective matrices are rejected.
- **Fitting**: least-squares error is not monotone in the number of control points, because the knot vectors for different counts are not nested. `approximate_to_tolerance` finds a count that meets the tolerance, not necessarily the smallest.

References for the kernel and geometry crates. Source code is only ported from GPL-compatible projects, with attribution and the original notices kept; otherwise implementations are written from the books and papers.

- Piegl and Tiller, *The NURBS Book* (2nd ed.): NURBS algorithms. Doc comments cite algorithm numbers.
- Patrikalakis and Maekawa, *Shape Interrogation for Computer Aided Design and Manufacturing*: intersections, offsets, robustness.
- Hoffmann, *Geometric and Solid Modeling*: booleans and robustness.
- Mantyla, *An Introduction to Solid Modeling*, and Stroud, *Boundary Representation Modelling Techniques*: B-rep data structures and Euler operators.
- Shewchuk, *Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates*: the basis of the orient and incircle predicates.
- Barnhill and Kersey, *A marching method for parametric surface/surface intersection*: surface-surface intersection tracing.
- Wang et al., *Computation of Rotation Minimizing Frames*: the double reflection method used for sweeps.
- Truck and Fornjot are Rust kernels worth reading for ideas.
