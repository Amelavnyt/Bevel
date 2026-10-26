# Architecture

This is the living architecture document for Bevel, an open-source, parametric, history-based 3D CAD application written in Rust. It describes the layers, the data flow, the crates and the main subsystems. Decisions with trade-offs are recorded separately in [DECISIONS.md](DECISIONS.md); the order of work is in [ROADMAP.md](ROADMAP.md). Subsystem details live in [KERNEL.md](KERNEL.md), [FILE_FORMAT.md](FILE_FORMAT.md) and [SCRIPTING.md](SCRIPTING.md).

Rust snippets below that define types are planned designs, not code that exists today. They show the intended shape and will be updated as the real types land.

## Current state

Phase 0 is in place: the Cargo workspace with every crate compiling, the renderer with grid, axes and a lit test cube, the egui layout with empty panels, and the app running on both desktop and web. Everything below the renderer in the layer diagram (kernel, sketching, timeline, model, document, I/O, scripting) is still stubs and is described here as the target design. See [ROADMAP.md](ROADMAP.md) for what comes next.

## Glossary

| Term | Meaning |
|---|---|
| Document | A `.bevel` file: version store plus working state |
| Part | Bodies, sketches, reference geometry, parameters and one part timeline (v1: exactly one part per document) |
| Body | One B-rep solid with a stable `BodyId` |
| Feature | One step of the part timeline (sketch, extrude, fillet, ...) |
| Sketch op | One step of a sketch's own timeline (add line, add constraint, trim, ...) |
| Rollback marker | Position in a timeline: replay stops there, new steps are inserted there |
| Regeneration | Replaying the timeline up to the marker to produce the model |
| TopoName | Persistent name of a face, edge or vertex, derived from how it was created |
| EntityRef | Stored reference to geometry: a `TopoName` plus a geometric hint |
| Command | Serializable document edit; the only way to change a document |
| Commit / Version / Branch | Git-like snapshots; a version is a named commit |

## Layers

```
L7  bevel-app (eframe: desktop + web)              bevel-cli (headless)
L6  bevel-ui (egui panels, timelines, tools, dialogs)
L5  bevel-script (Rhai)
L4  bevel-doc (commands, undo/redo, versions, .bevel format)
L3  bevel-model (features, regeneration, references, bodies)          bevel-io (STL, 3MF, STEP)
L2  bevel-kernel (B-rep, ops, booleans, tessellation, naming)          bevel-sketch (sketch + its timeline)
L1  bevel-geom (NURBS + analytic, intersections)   bevel-solver   bevel-expr   bevel-timeline   bevel-render (wgpu)
L0  bevel-math
```

**Layering rule:** a crate may depend only on crates in lower layers, never sideways and never up.

## Data flow

Input goes to the active Tool (ui), which produces a `Command` (doc). Applying it yields a new immutable `DocState`. Incremental regeneration runs on a worker (model) and produces a `ModelState`: bodies with TopoNames, solved sketches and diagnostics. Bodies are tessellated (kernel) and uploaded as GPU buffers (render). Picking returns ids, which are mapped to `EntityRef`s and stored in the next Command.

## Clean seams

- `bevel-kernel` knows nothing about features, sketches or documents. Callers pass an opaque `OpTag` so the kernel can name what it creates.
- `bevel-sketch` knows nothing about 3D bodies. The Sketch feature in bevel-model resolves the sketch plane and projected edges and hands them in as plain inputs.
- `bevel-render` knows nothing about kernel or model types; it draws plain mesh and line buffers carrying ids. The adapter lives in `bevel-app`.
- `bevel-io` depends only on kernel-level crates (kernel, geom, math), never on model or doc. Importing means io parses STEP into kernel bodies and the app issues an `ImportBody` command.
- All `cfg(target_arch = "wasm32")` code lives in `bevel-app/src/platform/`.

## Crates

| Crate | Responsibility | Depends on |
|---|---|---|
| bevel-math | f64 vectors/points/transforms (glam), boxes, intervals, tolerances, robust predicates, root finding, Newton, quadrature | none |
| bevel-geom | Curves and surfaces (NURBS + analytic), evaluation, projection, fitting, offsets, all intersections | math |
| bevel-solver | Generic nonlinear constraint solver: decomposition, LM/dogleg, DOF and conflict analysis | math |
| bevel-expr | Units-aware expression language, parameter tables, dependency graph | math |
| bevel-timeline | Generic timeline: order, marker, suppress, reorder validation, replay cache | none |
| bevel-render | wgpu renderer: meshes, edges, picking, section, overlays, camera | math |
| bevel-kernel | B-rep topology, builders, validation, modeling ops, booleans, blends, tessellation, mass properties, persistent naming | math, geom |
| bevel-sketch | Sketch entities, constraints, dimensions, sketch ops and timeline, profile regions | math, geom, solver, expr, timeline |
| bevel-model | Parts, features, regeneration, EntityRef resolution, bodies, reference geometry | math, geom, kernel, sketch, expr, timeline |
| bevel-io | STL, 3MF, STEP read/write | math, geom, kernel |
| bevel-doc | DocState, Commands, undo/redo, object store, commits/branches, `.bevel` format, migrations | model (and its dependencies) |
| bevel-script | Rhai engine, API bindings, script features | doc, model, kernel |
| bevel-ui | egui panels, widgets, tools, dialogs, timeline widgets | doc, model, render, script |
| bevel-app | eframe app, app state, platform layer, mesh adapter | everything |
| bevel-cli | Headless commands for CI and power users | doc, model, io, script |

Repository layout: `crates/` holds the crates above; `xtask/` holds `cargo xtask` helpers (pure-Rust guard, commit check, wasm build, fixtures, release); `web/` holds the Trunk setup for the wasm build; `assets/` holds icons, standards tables and example scripts; `fixtures/` holds documents, STEP files and golden results used by tests.

## Cross-cutting conventions

- **Units:** internal length is the millimetre, angle is the radian, and all geometry is `f64` (`f32` only on the GPU). Display units (mm, cm, m, in) affect only the UI and the default unit of expressions.
- **Tolerances** live in `bevel_math::tol`; see [KERNEL.md](KERNEL.md). Floats are never compared with `==`. Modeling space is +-1e5 mm; larger geometry is refused with a clear error.
- **IDs:** typed newtypes (`FeatureId`, `SketchId`, `SketchOpId`, `SketchEntityId`, `ConstraintId`, `ParamId`, `BodyId`, `CommitId`, ...) allocated from a per-document monotonic counter stored in the document and never reused. Ids created by procedural ops (for example the pieces of a trim) derive deterministically from (op id, index).
- **Collections** that reach output or serialization use `BTreeMap` or `IndexMap`, never `HashMap` iteration order.
- **Deterministic regeneration:** the same document always yields the same bodies and the same topological names. No unseeded randomness.
- **Errors:** `thiserror` in libraries, `anyhow` only in binaries. Library crates never panic on user input or bad geometry. Every user-facing error has a plain-English message plus technical detail.
- **Logging:** `tracing` everywhere, shown in an in-app log panel.
- **Concurrency:** regeneration runs off the UI thread on native (std threads and channels; `rayon` for data-parallel work such as tessellation). On web it starts single-threaded, time-sliced if needed; a Web Worker is a later decision. Long kernel loops check a cancellation token.
- **Licensing:** `GPL-3.0-or-later`, with an SPDX header at the top of every source file.

## Geometry kernel

### bevel-math

glam f64 types (`DVec2`, `DVec3`, `DVec4`, `DMat4`, `DQuat`) behind aliases (`Point3`, `Vector3`, `Transform3`, ...), `Aabb2/3`, `Interval`, `Frame3` (origin plus orthonormal axes), tolerance helpers, Shewchuk adaptive predicates (the `robust` crate: orient2d, orient3d, incircle), polynomial roots (closed form up to quartic plus a Sturm/bisection fallback), damped Newton in 1-D/2-D/3-D, and Gauss-Legendre quadrature. `nalgebra` provides dense linear algebra for the solver and fitting.

### bevel-geom: curves and surfaces

Every curve and surface is exactly representable as (rational) NURBS. Analytic types keep their identity so they can be intersected, measured and exported exactly.

Planned design:

```rust
pub enum Curve2 { Line(Line2), Circle(Circle2), Ellipse(Ellipse2), Nurbs(NurbsCurve2) }
pub enum Curve3 { Line(Line3), Circle(Circle3), Ellipse(Ellipse3), Nurbs(NurbsCurve3) }
pub enum Surface { Plane(Plane), Cylinder(Cylinder), Cone(Cone), Sphere(Sphere), Torus(Torus), Nurbs(NurbsSurface) }
// Bounds (trims, parameter ranges) live in topology, not in these types.
```

- Common traits: `CurveGeom` (eval, derivatives to order n, tangent, curvature, domain, closest point, bbox, transform, reverse, `to_nurbs()`) and `SurfaceGeom` (eval, partials, normal, principal curvatures, domain, periodicity, point inversion, isocurves, bbox, transform, `to_nurbs()`).
- The NURBS core follows *The NURBS Book*, with algorithm numbers cited in doc comments: knot vectors, basis functions and derivatives, curve and surface points and derivatives (rational and not), knot insertion/refinement/removal, degree elevation and reduction, Bezier decomposition, split, reverse, reparametrization, exact circles/arcs/conics as rational quadratics, exact surfaces of revolution and extrusion, interpolation and least-squares fitting, point inversion and projection, arc length and curvature. Helices and offsets are not exactly NURBS; they are approximated within tolerance and the error bound is recorded.
- Every algorithm gets a correct general-NURBS path first. Analytic fast paths (plane-plane, plane-quadric, sphere-sphere, coaxial cases) are optional optimizations, cross-checked against the NURBS path in tests.
- Intersections: curve-curve (Bezier subdivision with hull/box culling, then Newton, with overlap detection); curve-surface (subdivision plus Newton); surface-surface by seed points from recursive patch subdivision, predictor-corrector tracing with curvature-based step size, handling of closed loops, boundary exits and near-tangent points, fitting each branch as a 3D NURBS curve plus a pcurve on each surface, and explicit result kinds for tangential contact and coincident surfaces.

### bevel-kernel: topology

Planned design:

```rust
// Arena-based B-rep (slotmap). Bodies are immutable values shared via Arc; ops return new bodies.
pub struct Vertex { point: Point3, tol: f64, name: TopoName }
pub struct Edge   { curve: Curve3, range: Interval, verts: [VertexKey; 2], tol: f64, name: TopoName }
pub struct Coedge { edge: EdgeKey, reversed: bool, pcurve: Curve2, loop_: LoopKey }   // half-edge / fin
pub struct Loop   { coedges: Vec<CoedgeKey> }              // a face's first loop is its outer loop
pub struct Face   { surface: Surface, reversed: bool, loops: Vec<LoopKey>, name: TopoName }
pub struct Shell  { faces: Vec<FaceKey>, closed: bool }
pub struct Solid  { shells: Vec<ShellKey> }                // shells[0] is outer, the rest are voids
```

- Euler-operator-style builders and healing utilities (merge vertices within tolerance, fix orientation, compute missing pcurves).
- `validate(&Body) -> Vec<Issue>` checks closed 2-manifold shells, consistent orientation, loop closure, pcurve and 3D curve agreement, vertices on their edges, Euler-Poincare and, in debug and tests only, absence of self-intersections.
- Bodies serialize with serde (caches, imported bodies stored in documents, fixtures).
- Open sheet bodies may exist internally as intermediate results; the UI exposes solids only.

Operations: primitives (box, cylinder, sphere, cone, torus), extrude, revolve, sweep, loft, booleans, fillet, chamfer, shell, draft, transform/copy, mirror, split, and helix and thread geometry. Ops that cut or add through an intermediate volume also return that tool body so feature patterns can reuse it. The operation signature, the boolean pipeline and the robustness program are described in [KERNEL.md](KERNEL.md).

### Persistent naming

Persistent naming is designed in from the start. Planned design:

```rust
pub struct OpTag(pub u64);   // opaque; bevel-model passes the FeatureId
pub enum TopoName {
    Generated { op: OpTag, source: Source, role: Role },           // e.g. side face from profile segment #7, start cap, end cap
    Modified  { op: OpTag, base: Arc<TopoName>, fragment: u32 },   // a split piece of an older entity
    Merged    { op: OpTag, parts: Vec<Arc<TopoName>> },             // same-domain merge
    Between   { op: OpTag, faces: [Arc<TopoName>; 2], index: u32 }, // edge/vertex named by its adjacent faces
}
```

- `Source` identifies the generator: an opaque profile-segment id from the sketch, an input entity's name, a primitive role (`Top`, `Bottom`, `Side`, ...), a boolean tool face, and so on.
- Fragment indices come from a canonical, deterministic geometric ordering, so they are stable across rebuilds.
- `OpOutput.history` maps each new entity to what it was generated or modified from.
- In bevel-model, `EntityRef { body: BodyId, name: TopoName, hint: GeomHint }` stores the centroid, normal or tangent, and area or length at selection time. Resolution tries the exact name; if several entities match, the nearest by hint; otherwise it searches the lineage (ancestors and descendants) together with the hint; otherwise the reference is unresolved. An unresolved feature turns red with "Reference lost - reselect", and reselecting issues a Command that updates the ref.
- A regression suite covers the classic cases, for example a sketch on the top face of an extrude followed by an edit of the base sketch that splits or merges faces: dependent features must stay on the intended faces and edges.

### Tessellation

Per face in parameter space. Each edge is discretized once and shared by both adjacent faces, so meshes are watertight by construction, to a chordal and angular tolerance. Interior points are added adaptively by curvature, a constrained Delaunay triangulation in UV (`spade`) fills the face, and points map to 3D with exact surface normals. A coarse preset is used while dragging, a fine one at rest, and separate presets for export. Output keeps ids: triangle to face, polyline to edge, point to vertex.

### Mass properties and measurement

Volume, area, centre of mass and the inertia tensor come from Gauss quadrature over trimmed faces, cross-checked against tessellation. Measurement covers distances, angles, radii and the minimum distance between any two entities.

## Sketching

### Sketch model (bevel-sketch)

- A sketch lives on a plane, either a reference plane or a planar face, with its own 2D frame. The sketch stores `ExternalRefId`s for its plane and projected geometry; the Sketch feature in bevel-model owns the actual `EntityRef`s, resolves them at regeneration and passes in a plane frame plus projected 2D curves.
- Entities: point, line, arc, circle, ellipse and elliptical arc, and NURBS spline (control-point and fit-point modes), each with a `construction` flag. Rectangles (corner, center, 3-point), slots and polygons are tools that create entities and constraints, not entity types.
- Projected entities (edges and vertices projected from 3D) are refreshed every regeneration and are fixed for the solver.
- Constraints: coincident, point-on-curve, horizontal, vertical, parallel, perpendicular, tangent (G1), curvature (G2, splines), equal, symmetric, midpoint, concentric, collinear, fix.
- Dimensions: distance (point-point, point-line, line-line), horizontal and vertical distance, length, radius, diameter, angle, arc length. A dimension is either driving (an expression) or driven (read-only reference).

### Each sketch has its own timeline

- A sketch's content is the replay of its sketch ops, using the same generic engine as the part timeline: `AddEntity`, `AddConstraint`, `AddDimension`, `Trim`, `Extend`, `Split`, `Offset`, `Mirror`, `SketchPattern`, `SketchFillet`, `SketchChamfer`, `Project`, `SetConstruction`, `Delete` (needed only for entities produced by procedural ops; deleting a plainly added entity removes its step).
- It has the same powers as the part timeline: edit in place, rollback marker, dependency-checked reorder and suppress.
- Edits happen in place. Changing a dimension from 20 to 25, dragging geometry or editing a constraint modifies the existing step (undoably) instead of adding a new one, so the sketch timeline stays a clean recipe of how the sketch is built.
- Replay semantics: each op applies to the state left by the previous ops. Procedural ops (trim, offset, mirror, fillet, ...) solve first, then act on solved geometry, creating entities with derived ids. Per-op results are cached so editing late steps is instant.
- In the part timeline a sketch is one feature. While a sketch is being edited, its own timeline replaces the part timeline in the bottom bar; finishing or leaving the sketch brings the part timeline back.

### Solver (bevel-solver)

The solver is generic and knows nothing about sketches.

- Variables (coordinates, radii, angles, control points, ...) and constraints are residual functions with analytic Jacobians.
- The system is split into independent clusters (connected components first; structural decomposition such as Dulmage-Mendelsohn later), and each cluster is solved with Levenberg-Marquardt or a dogleg trust region with line search.
- While dragging, solving regularizes toward previous positions so geometry moves predictably.
- Degrees-of-freedom analysis uses Jacobian rank (QR/SVD): each entity is under-, fully or over-constrained, and the redundant or conflicting constraint set is reported.
- Target: a 500-entity sketch stays interactive (under 16 ms per drag frame on desktop).
- A test suite of classic constraint puzzles covers tangent chains, symmetric slots, fully constrained brackets and deliberate conflicts.

### Profiles

After solving, a 2D arrangement of the non-construction curves (using robust predicates) yields closed regions with holes. Region ids derive from their bounding entity ids, so an extrude's selected regions survive sketch edits. Open or overlapping geometry is flagged, not fatal.

### Sketch UX

The view looks at the plane on entry; there is a grid with snapping; inference shows glyphs (horizontal, vertical, coincident, tangent, midpoint); dimensions are entered on the canvas and accept expressions. Colours: under-constrained blue, fully constrained white (dark theme), conflicting red. Constraint glyphs can be clicked and deleted.

## Timeline engine (bevel-timeline)

The timeline engine is generic. Planned design:

```rust
pub trait TimelineOp: Clone {
    type Id: Copy + Ord + std::hash::Hash;
    fn id(&self) -> Self::Id;
    fn deps(&self) -> Vec<Self::Id>;      // steps whose outputs this step reads
    fn content_hash(&self) -> [u8; 32];
}
pub struct Timeline<Op: TimelineOp> { steps: Vec<Step<Op>>, marker: usize }
pub struct Step<Op> { op: Arc<Op>, suppressed: bool, label: Option<String> }
```

- Edits: insert at the marker, edit in place, delete (dependents are flagged, or deleted after confirmation), move (rejected with a reason if a step would land before something it depends on), suppress and unsuppress, set marker.
- For features, dependencies include every feature whose `OpTag` appears in a referenced `TopoName`, so reorder validation understands geometry references too.
- Replay cache: a state snapshot after each step, keyed by hash(step + previous snapshot hash). Editing step k replays from k; scrubbing the marker is instant when cached; memory is bounded by an LRU budget.
- Step status: Ok, Warning, Error, Skipped (an input failed or is suppressed), Suppressed, AfterMarker.
- Editing a feature temporarily rolls the marker to just before it and restores it afterwards.
- The engine is used twice: for the part timeline (features) and for every sketch timeline (sketch ops).

## Features and regeneration (bevel-model)

Planned design:

```rust
pub struct Feature { id: FeatureId, name: String, def: FeatureDef }

#[derive(Clone, Serialize, Deserialize)]
pub enum FeatureDef {
    Sketch(SketchDef), Primitive(PrimitiveDef), Extrude(ExtrudeDef), Revolve(RevolveDef),
    Sweep(SweepDef), Loft(LoftDef), Fillet(FilletDef), Chamfer(ChamferDef), Shell(ShellDef),
    Draft(DraftDef), Hole(HoleDef), Thread(ThreadDef), Pattern(PatternDef), Mirror(MirrorDef),
    Combine(CombineDef), SplitBody(SplitBodyDef), MoveBody(MoveBodyDef),
    RefPlane(RefPlaneDef), RefAxis(RefAxisDef), RefPoint(RefPointDef),
    Import(ImportDef), Script(ScriptFeatureDef),
}

pub trait FeatureImpl {
    fn inputs(&self) -> Vec<Dependency>;                         // parameters, sketches, EntityRefs, bodies
    fn schema() -> &'static FeatureSchema where Self: Sized;     // drives dialogs, scripting, docs
    fn regenerate(&self, cx: &mut RegenContext) -> Result<FeatureOutput, FeatureError>;
}
```

| Feature | Key inputs | Kernel work |
|---|---|---|
| Sketch | plane or planar face | none (sketch replay and solve) |
| Primitive | type, sizes, placement, body mode | primitive (+ boolean) |
| Extrude / Revolve | regions, extent or axis and angle, taper, body mode | extrude / revolve (+ boolean) |
| Sweep / Loft | profiles, path, options, body mode | sweep / loft (+ boolean) |
| Fillet / Chamfer | edges, sizes | blend |
| Shell / Draft | faces, thickness or angle and neutral plane | shell / draft |
| Hole | face and points, standard, type, depth, thread | tool body and subtract (+ thread) |
| Thread | cylindrical face, standard, cosmetic or modeled | helix sweep, or metadata only |
| Pattern / Mirror | features or bodies, counts and spacing or path, or plane | transformed tool bodies and the same boolean, or body copies |
| Combine | target, tools, join/cut/intersect, keep tools | boolean |
| Split / Move-Copy body | body and plane or face, or transform | split / transform |
| Ref plane / axis / point | references, offsets and angles | construction geometry only |
| Import | stored body blob (from STEP) | none: base body, no history |
| Script | script and declared parameters | via the Rhai API |

- **Bodies:** `BodyId` derives from (creating FeatureId, index); a body has a name, colour and visibility. Body modes are New, Join, Cut and Intersect with explicit target bodies (default: the bodies the new geometry touches).
- **Regeneration:** evaluate parameters, then replay features up to the marker through the timeline cache, producing `ModelState { bodies, sketches, reference_geometry, diagnostics }`. This runs on a worker with cancellation; the UI keeps showing the last good state plus progress.
- **Previews:** an open feature dialog regenerates a temporary feature on top of the cached state and renders it translucent; OK turns it into a Command.
- **Failures:** features that do not depend on a failed feature still run; dependents become Skipped.
- **Script features** run through a `ScriptRunner` trait defined in bevel-model and implemented in bevel-script (dependency inversion; the app or CLI injects it).
- **Origin:** every part has a non-deletable origin point, XY/XZ/YZ planes and X/Y/Z axes.
- **Assemblies seam:** `Document { parts: Vec<Part> }` with exactly one part in v1. `Component`, `Occurrence` and `Joint` are reserved, not implemented.

## Parameters and expressions (bevel-expr)

- Grammar: numbers with units (`10 mm`, `2in`, `45 deg`), identifiers, `+ - * / ^`, parentheses, the functions `sin cos tan asin acos atan2 sqrt abs min max round floor ceil`, and the constant `pi`. A hand-written Pratt parser keeps source spans for precise errors.
- Dimensional analysis covers length, angle, area and unitless values. `width * 2` is fine; `width + 30 deg` is an error.
- The parameter table holds user parameters (name, expression, comment) plus model parameters created automatically for every dimension and numeric feature input (`d1`, `d2`, ..., renameable). Every numeric field in the UI accepts an expression.
- A dependency graph with cycle detection means that changing a parameter regenerates from the first dependent feature.

## Document, commands, history and versions (bevel-doc)

### State

`DocState` is immutable and cheap to clone (`Arc` per feature, sketch and parameter table); every edit produces a new `DocState` sharing all unchanged parts. A `DocSession` holds the object store, refs, the working `DocState` and the undo stack.

### Commands

Planned design:

```rust
pub enum Command {
    AddFeature { def: FeatureDef },                  // inserted at the marker
    EditFeature { id: FeatureId, def: FeatureDef },
    DeleteFeature { id: FeatureId, cascade: bool },
    MoveFeature { id: FeatureId, to: usize },
    SetSuppressed { id: FeatureId, suppressed: bool },
    SetMarker { index: usize },
    RenameFeature { id: FeatureId, name: String },
    Sketch { sketch: SketchId, edit: SketchEdit },   // add/edit/move/suppress sketch ops, drag, set dimension, sketch marker
    Parameters(ParamEdit),
    SetBodyProps { body: BodyId, props: BodyProps },
    ImportBody { blob: BodyBlob, name: String },
    SetUnits(Units),
    Batch { label: String, cmds: Vec<Command> },
}
```

Commands are validated, applied atomically, serializable and labelled ("Edit Extrude 3"). The UI, Rhai scripts and tests all go through them; the UI never mutates the document directly.

### Undo and redo

The undo stack holds previous `DocState`s, which is cheap because of sharing, capped by memory. Sketch edits are undoable like everything else. Marker moves are saved with the document but do not clutter the undo stack. Undo history lives per session; versions are durable.

### Versions and branches

- A content-addressed object store: features, sketches, parameter tables and body blobs are serialized and hashed (`blake3`, `pure` feature), and unchanged objects are shared between versions.
- `Commit { tree, parents, message, timestamp }`; refs are branches (`main` by default) and tags (named versions); there is a `HEAD` and an uncommitted working state.
- UI: save a version with a message, a version list, checkout, create and switch branch, and a feature-level diff between two versions (added, removed, modified, moved).
- Merge (Phase 12b): three-way merge per feature by stable ids, order merge by stable ids, and per-feature conflict resolution (keep mine or keep theirs).
- The store backs `.bevel` from the first save format, so no migration is needed when the versions UI arrives.

### File format

A `.bevel` file is a ZIP container (pure-Rust deflate only) with `manifest.json` (format name, `format_version`, app version, HEAD), `objects/<hash>` (JSON definitions; body blobs as JSON or compact binary), `refs/heads/*`, `refs/tags/*` and `working.json`, with `thumbnail.png` to follow. The format is versioned from day one: every change bumps `format_version` and ships a migration on `serde_json::Value` plus a fixture test that loads every past version. See [FILE_FORMAT.md](FILE_FORMAT.md). `bevel-cli unpack` and `pack` convert to and from a plain directory for debugging and git-friendly storage.

### Autosave

Native builds autosave periodically to the platform data directory and offer recovery on launch. The web build uses browser storage plus a "download .bevel" action.

## Rendering (bevel-render)

- wgpu, drawing into offscreen targets that are shown in egui (see [ADR-0012](DECISIONS.md#adr-0012-viewport-renders-to-an-offscreen-texture-shown-by-egui)).
- Planned passes: shaded faces (simple lit material, optional matcap); B-rep edges as screen-space thick lines (instanced quads, since wgpu lines are 1 px wide); silhouettes of curved faces (post-process on depth and normals); grid and origin axes; translucent previews; hover and selection highlight; section plane (clip plus capped fill); sketch overlay (entities, points, constraint glyphs; dimension text is drawn by egui at projected positions).
- Picking: an id buffer pass (`R32Uint`: body, face, edge, vertex and sketch-entity ids) read back asynchronously under the cursor, plus a CPU ray cast against a BVH for exact hit points and snapping.
- Camera: orbit around the point under the cursor, pan, zoom to cursor, fit all and fit selection, standard views, perspective and orthographic projection, smooth transitions, and a clickable view cube.
- GPU buffers are kept per body and re-uploaded only when that body's tessellation hash changes.

### Viewport rendering

This describes what the code does today.

- **Offscreen targets.** `ViewportRenderer` owns its colour and depth targets and renders into them rather than into egui's pass. Colour is multisampled (4x MSAA where the adapter supports it for both colour and depth formats, otherwise no MSAA) and resolved into an `Rgba8Unorm` texture. Depth is `Depth32Float`.
- **Colour space.** egui composites in gamma space and samples textures as stored. The resolve target therefore holds sRGB-encoded values in a plain UNORM format: shaders work in linear colour and encode their output with `to_srgb`. Theme colours are given as sRGB bytes and converted to linear on the CPU.
- **Hand-off to egui.** `bevel-app` (`viewport.rs`) calls the renderer each frame with the viewport size in pixels and the camera, then registers the resolved texture with egui's wgpu renderer (`register_native_texture`). When the targets are recreated, for example after a resize, the existing egui texture id is updated in place (`update_egui_texture_from_wgpu_texture`). The renderer clamps target edges to 8192 px.
- **World space.** Z-up, right-handed, in millimetres, with the ground grid in the XY plane. CPU-side camera math is `f64`; values are converted to `f32` only when written to GPU buffers.
- **Depth.** Reverse-Z (1 at the near plane, 0 at infinity) with an infinite far plane for perspective projection; see [ADR-0013](DECISIONS.md#adr-0013-z-up-world-and-reverse-z-depth).
- **Camera.** A turntable camera orbiting a target point: the eye sits at `target + distance * dir(yaw, pitch)`, with pitch clamped just short of the poles. Orbit is right-drag or Shift+middle-drag; pan is middle-drag or Shift+right-drag; zoom is toward the cursor with the wheel or a pinch. Home is Fit All. The View menu toggles between perspective and orthographic projection; the orthographic view height matches what the perspective view shows at the target distance, so switching keeps the framing.

## App and UI (bevel-app, bevel-ui)

### Layout

The layout is dark by default and modelled on common parametric CAD workspaces.

```
┌──────────────────────────────────────────────────────────────────────────┐
│ File Edit View │ [SOLID] [SKETCH] [INSPECT] [SCRIPTS]  tools… │ main ▾   │
├───────────────┬──────────────────────────────────────────┬───────────────┤
│ Browser       │                                          │ Properties /  │
│ ▸ Origin      │              3D viewport                 │ feature panel │
│ ▸ Parameters  │       view cube · grid · axes            │ (schema-driven│
│ ▸ Sketches    │                                          │  fields,      │
│ ▸ Bodies      │                                          │  expressions, │
│ ▸ Construction│                                          │  OK / Cancel) │
├───────────────┴──────────────────────────────────────────┴───────────────┤
│ ⏮ ◀ ▶ ⏭  [Sketch1][Extrude1][Fillet1] ▌ [Hole1][Pattern1]   part timeline│
└──────────────────────────────────────────────────────────────────────────┘

While a sketch is open, the same bar shows that sketch's timeline instead:
┌──────────────────────────────────────────────────────────────────────────┐
│ Part ▸ Sketch1 │ ⏮ ◀ ▶ ⏭  [Line][Line][Coincident][d1][Trim] ▌  [Finish] │
└──────────────────────────────────────────────────────────────────────────┘
```

### App state and tools

- Planned: `AppState { session, model: Arc<ModelState>, selection, mode, active_tool, camera, prefs }` with `mode = Model | SketchEdit(SketchId) | FeatureDialog(..)`.
- Tools are state machines (`trait Tool { fn on_input(..); fn preview(..); fn overlay(..); fn finish(..) -> Option<Command>; }`): sketch tools (line, arc, circle, rectangle, slot, polygon, spline, dimension, constraints, trim, extend, offset, ...), feature tools and measure.
- Widgets emit `UiAction`s and the app turns them into Commands. No widget touches `DocState`.

### Selection

Filters cover faces, edges, vertices, bodies, sketch entities, regions and planes or axes. Hovering pre-highlights, click and shift-click select, and dialogs request typed selections ("Edges: 3 selected"). Stored selections become `EntityRef`s.

### Feature dialogs

Dialogs are generated from `FeatureSchema` (length, angle, count, enum, selection and bool fields). Numeric fields take expressions, there is a live preview with OK and Cancel, and editing reopens the same dialog. On-canvas drag handles (such as the extrude arrow) arrive in Phase 14.

### Timeline widgets

Steps have icons and status colours (error red, warning yellow, suppressed struck through, after-marker greyed). The marker is draggable and steps can be reordered by dragging with live validity feedback. The context menu offers edit, rename, suppress, delete and roll to here, and hovering highlights the geometry a step produced. The sketch timeline is the same widget instantiated for sketch ops; it replaces the part timeline while a sketch is open, with a breadcrumb (`Part ▸ Sketch1`) and a Finish Sketch action to return.

### Shortcuts

Planned defaults (configurable later): L line, A arc, C circle, R rectangle, D dimension, T trim, E extrude, F fillet, H hole, M measure, Home fit all, Esc cancel or exit sketch, Delete, Ctrl+Z, Ctrl+Shift+Z, Ctrl+S.

### Platform layer

`bevel-app/src/platform/` is the only place with `cfg(target_arch = "wasm32")` code. It will cover file open and save (rfd on native, using the XDG portal on Linux rather than GTK; the browser file picker and download on web), storage (autosave, preferences), task spawning, clipboard and time. Today it provides the native and web entry points and the shared log filter.

## I/O, scripting and CLI

### STL and 3MF (bevel-io)

Binary and ASCII STL; the 3MF core spec (ZIP plus XML via `quick-xml`) with millimetre units, one object per body, names and colours. Quality presets reuse the tessellation settings. Exported meshes must be watertight, and this is tested.

### STEP (ISO 10303-21; AP242, AP214-compatible)

- A Part 21 tokenizer, parser and writer of our own (small, no code generation) plus an entity-mapping layer.
- Export covers the header, product, units and context boilerplate; `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, `CLOSED_SHELL`, `ADVANCED_FACE`, `FACE_OUTER_BOUND`/`FACE_BOUND`, `EDGE_LOOP`, `ORIENTED_EDGE`, `EDGE_CURVE`, `VERTEX_POINT`, `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (and rational complex entities), `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE` and `B_SPLINE_SURFACE_WITH_KNOTS` (and rational). One solid is written per body and body names are kept.
- Import reads the same entities plus `SURFACE_OF_REVOLUTION`, `SURFACE_OF_LINEAR_EXTRUSION`, `OFFSET_SURFACE` (converted to NURBS), `TRIMMED_CURVE`, `PCURVE` and `SEAM_CURVE`. Assemblies are flattened into separate bodies with transforms applied, and healing handles tolerant vertices and edges, orientation and missing pcurves. Unsupported entities become warnings in an import report, never crashes. Imported bodies are stored in the document and appear as an `Import` feature.
- Tests: round trips (export, import, same topology counts, volume and area within tolerance), a fixture corpus (our own exports plus freely licensed STEP files, each checked for its terms), and a manual checklist for opening our exports in other CAD tools.

### Scripting (bevel-script)

Rhai scripts are sandboxed: no file or network access, operation and call-depth limits, and deterministic behaviour. The API mirrors Commands and model queries. Macros run once and every call becomes a normal Command, so results land in the timeline as ordinary features. Script features are timeline steps that re-run on every regeneration with declared parameters (for example a gear generator), and scripts label their outputs so TopoNames stay stable. A script panel offers an editor with basic highlighting, run, errors with line numbers, and an API reference generated from the bindings. Examples live in `assets/scripts/`. See [SCRIPTING.md](SCRIPTING.md).

### CLI (bevel-cli)

`rebuild <file>` (diagnostics; non-zero exit on errors), `export <file> --format stl|3mf|step [--body NAME] -o OUT`, `import <file.step> -o OUT.bevel`, `validate <file>`, `run <script.rhai> [-o OUT.bevel]`, `info <file>` (features, parameters, bodies, volumes), `unpack` and `pack`, and `kernel-repro <dump.json>`. Integration tests drive the CLI over `fixtures/`.
