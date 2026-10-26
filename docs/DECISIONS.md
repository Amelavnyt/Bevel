# Decision log

This file records the architecture decisions that shape Bevel. Each entry is an ADR (architecture decision record) with the same structure:

- **Context:** the forces and constraints behind the decision.
- **Decision:** what was chosen.
- **Consequences:** what follows from it, including the costs.

Every ADR has a number and a status: Accepted, Proposed or Superseded. Numbers are never reused or renumbered. A decision that is replaced stays in the log, its status changes to Superseded and it links to the ADR that replaces it. A change to the architecture or to the order of the [roadmap](ROADMAP.md) is proposed as a new ADR with status Proposed before the work starts.

See [ARCHITECTURE.md](ARCHITECTURE.md) for the design these decisions produce.

## Index

| ADR | Title | Status |
|---|---|---|
| 0001 | 100% Rust, no C/C++ in the build | Accepted |
| 0002 | egui/eframe UI with our own wgpu renderer | Accepted |
| 0003 | Own B-rep kernel, NURBS-first with analytic identity | Accepted |
| 0004 | Persistent naming through operation history | Accepted |
| 0005 | Commands are the only mutation path | Accepted |
| 0006 | One generic timeline engine for part and sketch timelines | Accepted |
| 0007 | Content-addressed object store; `.bevel` is a versioned ZIP | Accepted |
| 0008 | Internal units, f64 and tolerance policy | Accepted |
| 0009 | License GPL-3.0-or-later, no CLA for now | Accepted |
| 0010 | Rhai for scripting, script features via an injected runner | Accepted |
| 0011 | Conventional Commits with a footer allow-list | Accepted |
| 0012 | Viewport renders to an offscreen texture shown by egui | Accepted |
| 0013 | Z-up world and reverse-Z depth | Accepted |
| 0014 | Exceptions to the dependency rules for the application shell | Accepted |
| 0015 | One design workspace with an SVG icon set | Accepted |

## ADR-0001: 100% Rust, no C/C++ in the build

Status: Accepted (2026-10-26)

**Context.** A CAD application that builds on every desktop platform and in the browser is much simpler to ship when the whole dependency tree is Rust. Native C and C++ libraries (OpenCascade, Dear ImGui, SDL) bring system toolchains, cross-compilation problems and a hard wasm32 story. Rust crates that only call OS or graphics APIs (winit, wgpu, rfd, web-sys) do not have these problems.

**Decision.** Nothing written in C or C++ is compiled into Bevel. Crates whose build runs `cc`, `cmake`, `bindgen` or `pkg-config` are not allowed. CI enforces this with `cargo xtask check-pure-rust`, which fails if any of those appear in the dependency tree; exceptions go in an explicit allow-list that needs maintainer approval.

**Consequences.** The same code builds for desktop and wasm32 without a native toolchain. Some capabilities that are one dependency away in C++ (geometry kernels, STEP translators) have to be written in Rust. The dependency choice is narrower, which is why every new dependency is checked for maintenance, license and pure-Rust status.

## ADR-0002: egui/eframe UI with our own wgpu renderer

Status: Accepted (2026-10-26)

**Context.** Bevel targets desktop and the web from one codebase. It needs an immediate-mode-friendly UI for dense tool panels and timelines, and a 3D viewport with control over passes, picking and edge rendering that a general UI toolkit does not provide.

**Decision.** The UI is egui via eframe, which runs natively and on wasm32. The 3D viewport is our own renderer built on wgpu, in the `bevel-render` crate, which knows nothing about kernel or model types and draws plain mesh and line buffers. The adapter between model data and those buffers lives in `bevel-app`.

**Consequences.** One UI and rendering stack for both targets, and full control of the viewport. The cost is that we own the renderer: thick edges, silhouettes, picking and sections are our work. How the viewport is embedded in egui is recorded in [ADR-0012](#adr-0012-viewport-renders-to-an-offscreen-texture-shown-by-egui).

## ADR-0003: Own B-rep kernel, NURBS-first with analytic identity

Status: Accepted (2026-10-26)

**Context.** The modeling kernel is the core of a parametric CAD program, and its robustness decides the quality of everything above it. Existing Rust kernels are young, and C++ kernels conflict with [ADR-0001](#adr-0001-100-rust-no-cc-in-the-build). Fillets, lofts, sweeps and STEP data all need free-form surfaces, so adding NURBS later would force a rewrite.

**Decision.** Bevel has its own B-rep kernel (`bevel-geom` and `bevel-kernel`). Every curve and surface is exactly representable as rational NURBS, and analytic types (line, circle, ellipse, plane, cylinder, cone, sphere, torus) keep their identity so they can be intersected, measured and exported exactly. Each algorithm gets a correct general-NURBS path first; analytic fast paths are optional and are cross-checked against it in tests.

**Consequences.** One general code path covers every case, at the price of slower and more delicate general algorithms such as surface-surface intersection. Fast paths can be added where profiling shows a need, without changing results. Kernel robustness becomes a standing program (see [KERNEL.md](KERNEL.md)).

## ADR-0004: Persistent naming through operation history

Status: Accepted (2026-10-26)

**Context.** In a history-based modeler, later features refer to faces and edges produced by earlier ones. If those references are index-based or purely geometric, editing an early feature breaks or silently retargets the later ones (the topological naming problem). Retrofitting a naming scheme onto an existing kernel is hard.

**Decision.** Naming is part of the kernel design from the start. Every face, edge and vertex carries a `TopoName` derived from how it was created (generated, modified, merged, or named by its adjacent faces), with an opaque `OpTag` identifying the creating operation. Operations return a history map from new entities to their sources. `bevel-model` stores references as `EntityRef` (a `TopoName` plus a geometric hint) and resolves them by exact name, then by hint, then through the lineage, and otherwise reports the reference as lost.

**Consequences.** Every kernel operation must produce names and history, which adds work to each one. References survive edits that split or merge faces, and failures are explicit ("Reference lost, reselect") rather than silent. A regression suite for naming cases is required.

## ADR-0005: Commands are the only mutation path

Status: Accepted (2026-10-26)

**Context.** Undo/redo, versions, scripting and tests all need a single, reliable description of what changed in a document. If panels or tools mutate state directly, each of those features needs its own bookkeeping.

**Decision.** A document changes only by applying a serializable `Command`. `DocState` is immutable and cheap to clone, with `Arc` sharing per feature, sketch and parameter table; applying a command yields a new `DocState`. Undo is a stack of previous states, capped by memory. The UI emits actions that the app turns into Commands, Rhai scripts call the same Commands, and tests drive them directly.

**Consequences.** Undo, redo, labels and scripting come from one mechanism, and command sequences are easy to test and replay. Every edit must be expressible as a Command, including drags and sketch edits. Undo history is per session; durable history belongs to versions ([ADR-0007](#adr-0007-content-addressed-object-store-bevel-is-a-versioned-zip)).

## ADR-0006: One generic timeline engine for part and sketch timelines

Status: Accepted (2026-10-26)

**Context.** Both the part (features) and every sketch (sketch ops) need the same behaviour: ordered steps, a rollback marker, in-place editing, reordering with dependency checks, suppression, and cached replay. Implementing it twice would duplicate the hardest logic.

**Decision.** `bevel-timeline` is a generic engine over a `TimelineOp` trait (id, dependencies, content hash). It is used for the part timeline and for each sketch's timeline. Edits happen in place: changing a dimension modifies the existing step instead of appending a new one, so a timeline stays a clean recipe. Replay results are cached per step, keyed by the hash of the step and the previous snapshot.

**Consequences.** Editing step k replays only from k, and scrubbing the marker is cheap. The engine must stay independent of features and sketches, and ops must supply honest dependency lists and content hashes. In the UI, the sketch timeline replaces the part timeline while a sketch is open.

## ADR-0007: Content-addressed object store; `.bevel` is a versioned ZIP

Status: Accepted (2026-10-26)

**Context.** Version history with branches and diffs is a core feature, and a document format that is later retrofitted for it would need a migration. The format should also be easy to inspect and to keep in git when needed.

**Decision.** Features, sketches, parameter tables and body blobs are serialized and hashed (`blake3`, pure-Rust mode) into a content-addressed object store, with git-like commits, branches and tags on top. A `.bevel` file is a ZIP container holding a manifest, the objects, the refs and the working state. The format carries a `format_version`; every change bumps it and ships a migration plus a fixture test that loads every past version.

**Consequences.** Unchanged objects are shared between versions, so history is cheap. The versions UI can arrive later without changing the save format. Migrations are mandatory work whenever the format changes. The layout is specified in [FILE_FORMAT.md](FILE_FORMAT.md).

## ADR-0008: Internal units, f64 and tolerance policy

Status: Accepted (2026-10-26)

**Context.** Geometry code needs one unit system and a consistent rule for when two numbers are equal; scattered ad-hoc epsilons are a common source of kernel bugs. Imported STEP data can be less precise than modeled geometry.

**Decision.** Internal length is the millimetre and angle is the radian, in `f64` throughout (`f32` only on the GPU). Display units affect only the UI and the default unit of expressions. Tolerances live in `bevel_math::tol` (linear 1e-6 mm, angular 1e-11 rad, modeling space +-1e5 mm), floats are never compared with `==`, and parametric tolerances are derived per curve or surface. Edges and vertices also carry their own tolerance. Geometry outside the modeling space is refused with a clear error.

**Consequences.** All crates agree on what "equal" means, and imported data can be represented faithfully. The bounded modeling space gives the renderer and kernel a known range (see [ADR-0013](#adr-0013-z-up-world-and-reverse-z-depth)). Values from other unit systems must be converted at the boundary. Details are in [KERNEL.md](KERNEL.md).

## ADR-0009: License GPL-3.0-or-later, no CLA for now

Status: Accepted (2026-10-26)

**Context.** Bevel should stay open source, and porting well-understood techniques from other open-source CAD projects (for example the GPL-3.0 SolveSpace or the LGPL FreeCAD) should be possible with attribution. A contributor license agreement adds friction before there are outside contributors.

**Decision.** The project is licensed GPL-3.0-or-later, declared in the workspace manifest and in an SPDX header at the top of every source file. Dependencies must have GPL-3.0-compatible licenses, checked by `cargo deny` in CI. There is no CLA for now; this will be revisited before outside contributions are accepted.

**Consequences.** Code from compatible projects can be ported with its notices kept, while incompatible or proprietary sources are off limits. Changing the license later would require the agreement of contributors unless a CLA is introduced first.

## ADR-0010: Rhai for scripting, script features via an injected runner

Status: Accepted (2026-10-26)

**Context.** Users need macros for repetitive work and parametric generators (gears, nuts, enclosures). The scripting engine must be pure Rust ([ADR-0001](#adr-0001-100-rust-no-cc-in-the-build)), sandboxable and deterministic, and it should not tie the model layer to a particular engine.

**Decision.** Scripting uses Rhai in `bevel-script`. Macros run once, and every call becomes a normal Command. Script features are timeline steps that re-run on every regeneration with declared parameters. `bevel-model` defines a `ScriptRunner` trait, `bevel-script` implements it, and the app or CLI injects it. Scripts have no file or network access and run under operation and call-depth limits.

**Consequences.** Script output lands in the timeline as ordinary features, and regeneration stays deterministic. The model layer does not depend on the scripting layer. Script features must label their outputs so topological names stay stable. See [SCRIPTING.md](SCRIPTING.md).

## ADR-0011: Conventional Commits with a footer allow-list

Status: Accepted (2026-10-26)

**Context.** A public project benefits from a consistent, readable history that tooling can parse for changelogs and review.

**Decision.** Commit messages follow Conventional Commits 1.0 (`type(scope): subject`), with the scope being the crate name without the `bevel-` prefix or one of `ci`, `deps` and `docs`. Subjects are imperative, lower case after the colon and at most 72 characters. Footers are limited to an allow-list (`Fixes #N`, `Refs #N`, `BREAKING CHANGE: ...`). `cargo xtask check-commits` checks every commit on a branch in CI, and `.githooks/commit-msg` runs the same check locally.

**Consequences.** History is uniform and machine-readable, and malformed messages fail early. Contributors need to enable the hook (`git config core.hooksPath .githooks`) or fix messages after CI reports them. The checker enforces format only.

## ADR-0012: Viewport renders to an offscreen texture shown by egui

Status: Accepted (2026-10-26)

**Context.** The 3D viewport has to appear inside an egui panel. There are two ways to do this with eframe's wgpu backend: record the scene inside egui's own render pass through an `egui_wgpu` paint callback, or render into textures that we own and display the result as an image.

**Decision.** The renderer draws into its own offscreen targets and the app shows the result as an egui texture. The alternative, an `egui_wgpu` paint callback into egui's render pass, was rejected because later passes (an id buffer for picking, silhouettes from depth and normals, section caps) need our own depth and colour targets, and MSAA settings independent of egui. Colour is 4x multisampled where supported and resolved into an `Rgba8Unorm` texture that holds sRGB-encoded values, because egui composites in gamma space. `bevel-app` registers that texture with egui's renderer and updates the registration when the targets are recreated.

**Consequences.** There is one extra texture sample per frame when egui draws the image. The texture is re-registered whenever the viewport is resized. In return, every pass can read depth and colour freely and the renderer stays independent of how egui configures its own pass. Shaders must encode their output to sRGB themselves.

## ADR-0013: Z-up world and reverse-Z depth

Status: Accepted (2026-10-26)

**Context.** CAD users expect Z to be up and the ground plane to be XY, which also matches how sketch planes and standard views are named. The modeling space spans +-1e5 mm, and a conventional depth buffer has poor precision far from the near plane at that range. wgpu line topologies do not allow a depth bias, which is a common way to keep edges from z-fighting with the faces they outline.

**Decision.** World space is Z-up and right-handed, in millimetres, with the grid in the XY plane. Depth is reverse-Z (1 at the near plane, 0 at infinity) with an infinite far plane, which gives even depth precision across the modeling space. Edges are pulled towards the camera in the line shader instead of using depth bias, which WebGPU forbids for line topologies.

**Consequences.** Depth precision is good at any distance and no far-plane tuning is needed in perspective views. All depth comparisons and clears in the renderer use the reversed convention, and any new pass must follow it. The edge offset is part of the shader and has to be tuned in clip or view space rather than through pipeline state.

## ADR-0014: Exceptions to the dependency rules for the application shell

Status: Accepted (2026-10-26)

**Context.** The application shell (eframe, winit, wgpu) brings in a few dependencies that the rules in [ADR-0001](#adr-0001-100-rust-no-cc-in-the-build) and the cargo-deny policy flag. Each was checked to see whether it could be removed.

**Decision.** Three edges stay on the pure-Rust guard's allow-list (`xtask/pure-rust-allowlist.txt`). All are Linux-only, and none builds or links C code:

- `wayland-backend` lists `cc`, but its build script uses it only with the `log` feature, which nothing enables.
- `wayland-sys` lists `pkg-config`, but with the `dlopen` feature (enabled by winit and wgpu) it loads the Wayland libraries at run time and its build script returns before probing.
- `x11-dl` runs the `pkg-config` program, if installed, only to record X11 library directories as strings for run-time loading. It compiles nothing and links only libdl.

Removing them would mean dropping Wayland or X11 support. cargo-deny ignores RUSTSEC-2026-0370, -0371 and -0372, which mark `zvariant`, `zvariant_derive` and `zbus_names` as unmaintained (merged into zbus 6). They are not vulnerabilities, and they come from accesskit's Linux screen-reader adapter, whose latest release still uses zbus 5. Removing them would mean dropping screen-reader support.

Two dependencies outside the starting set are accepted: `tracing-subscriber`, the standard companion to `tracing`, which routes events to stderr and the in-app log panel; and `wasm-bindgen-futures`, which starts the async web runner and is already in the graph through wgpu. The web platform layer uses eframe's re-export of `web-sys` rather than depending on it directly.

**Consequences.** The guard and cargo-deny stay strict for everything else. Each exception is reviewed when the dependency that brings it in is upgraded: the advisory ignores go once accesskit moves to zbus 6, and the guard warns when an allow-list entry is no longer needed. Content hashing for the object store will need the same review: `blake3` keeps `cc` as a build dependency even with its `pure` feature, which disables the C and assembly code.

## ADR-0015: One design workspace with an SVG icon set

Status: Accepted (2026-10-26)

**Context.** The first layout had workspace tabs (SOLID, SKETCH, INSPECT, SCRIPTS) and text buttons. Sketching, measuring and section views are part of everyday modeling, so putting them in separate tabs forced needless mode switches. Text-only buttons are slow to scan and look unlike the CAD tools users know.

**Decision.** Bevel has a single design workspace. The toolbar groups tools as CREATE, MODIFY, CONSTRUCT, INSPECT, INSERT and UTILITIES; sketch tools replace the toolbar's contents only while a sketch is open. Tools are shown as icons from a project-owned SVG set in `assets/icons/`, rendered with egui_extras' `svg` feature (resvg, usvg and tiny-skia: pure Rust, MIT/Apache, accepted by the pure-Rust guard and cargo-deny). Icons are drawn for Bevel rather than taken from another product's set.

**Consequences.** New tools need an icon in the same style (32×32, flat bodies in three shades of light neutral grey with one outline, the affected part in the signature teal `#2D8985`; keep icons simple enough to read at 16 px). Teal is the dominant UI colour; the accent green `#77C89D` is secondary. SVGs are rasterized at run time, which costs a little start-up time and keeps icons sharp at any scale. Workspace tabs can return later for genuinely different environments (assemblies, drawings) without changing how part modeling is organized.
