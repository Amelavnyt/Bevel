# Roadmap

Each phase ends with CI green and a demo that can be tried by hand. Items are checked off as they land. The order of phases changes only through a decision record in [DECISIONS.md](DECISIONS.md). Architecture details are in [ARCHITECTURE.md](ARCHITECTURE.md).

## Phase 0: Skeleton

- [x] Cargo workspace with all crates compiling
- [x] CI
- [x] Contributor docs
- [x] LICENSE
- [x] xtask with the pure-Rust guard and the commit check
- [x] commit-msg hook
- [x] eframe app with the full layout (empty panels)
- [x] wgpu viewport with grid, axes, orbit/pan/zoom camera and a lit test cube
- [x] The same app runs in the browser via Trunk

## Phase 1: Math and NURBS

- [x] bevel-math: vectors, transforms, boxes, intervals, tolerances, robust predicates, root finding, Newton, quadrature
- [x] NURBS curves and surfaces: knot vectors, basis functions, evaluation and derivatives, knot insertion/refinement/removal, degree elevation and reduction, Bezier decomposition, split, reverse, reparametrize
- [x] Exact circles, arcs and conics as rational quadratics; exact surfaces of revolution and extrusion
- [x] Interpolation and least-squares fitting for curves and surfaces
- [x] Point inversion, projection, arc length and curvature
- [x] Analytic types (line, circle, ellipse, plane, cylinder, cone, sphere, torus) with `to_nurbs`
- [x] A debug view that displays test curves and surfaces

Helices and offset curves/surfaces, which are not exactly NURBS and are approximated within a tolerance, moved to the phases that need them: offsets to Phase 8 (fillets and shells), helices to Phase 10 (threads).

## Phase 2: B-rep and display

- [x] Topology, builders, validation and serialization
- [x] The five primitives: box, cylinder, sphere, cone, torus
- [x] Tessellation
- [x] Rendering with edges
- [x] GPU picking with hover and selection
- [x] Mass properties, face areas and edge lengths
- [x] Measure dialog for the selection: area, length, volume, centre of mass
- [x] STL export
- [x] 3MF export

The CPU ray cast for exact hit points and silhouette edges of curved faces moved to Phase 7.

## Phase 3: Document and timeline

- [x] bevel-timeline: generic timeline with marker, reorder, suppress and replay cache
- [x] bevel-expr: expressions, units and parameter tables
- [x] bevel-model: primitive features, origin, regeneration with cache
- [x] bevel-doc: commands, undo/redo, object store
- [x] `.bevel` v1 with migration scaffold
- [x] Browser tree
- [x] Part timeline widget (marker, reorder, suppress, edit)
- [x] Parameters dialog
- [x] CLI `rebuild`, `info` and `export`

## Phase 4: Booleans

- [x] Curve-curve and curve-surface intersection
- [x] Surface-surface intersection (general NURBS path and analytic fast paths)
- [x] Boolean pipeline with the edge-case suite and property tests
- [x] Kernel debug dumps and `kernel-repro`
- [ ] Body modes on primitives
- [ ] Combine
- [ ] Multi-body UI (visibility, colour, rename)

## Phase 5: Sketcher

- [ ] Constraint solver
- [ ] Sketch entities
- [ ] Constraints and dimensions
- [ ] The sketch timeline and its strip
- [ ] Sketch-edit mode: the sketch tools and Finish Sketch replace the SOLID tools while a sketch is open
- [ ] Profiles and regions
- [ ] Tools with inference
- [ ] Degrees-of-freedom colours
- [ ] Conflict reporting

## Phase 6: Extrude, revolve and naming

- [ ] Extrude with all body modes
- [ ] Revolve with all body modes
- [ ] Reference planes, axes and points
- [ ] Sketch on a planar face
- [ ] EntityRef resolution
- [ ] "Reference lost" UX
- [ ] Topological-naming regression suite

## Phase 7: Inspect

- [ ] CPU ray cast against a bounding volume hierarchy, for exact hit points and snapping (moved from Phase 2)
- [ ] Silhouette edges of curved faces (moved from Phase 2)
- [ ] Measure distances, angles and radii between selections
- [ ] Section view
- [ ] Mass Properties tool (inertia tensor, principal moments and axes)

## Phase 8: Blends and offsets

- [ ] Offset curves and surfaces, approximated within a recorded tolerance
- [ ] Fillet
- [ ] Chamfer
- [ ] Shell
- [ ] Draft

## Phase 9: Sweeps and patterns

- [ ] Sweep
- [ ] Loft
- [ ] Linear, circular and path patterns of features and bodies
- [ ] Mirror
- [ ] Move/copy body
- [ ] Split body

## Phase 10: Holes and threads

- [ ] Helices, approximated within a recorded tolerance
- [ ] ISO metric data: clearance per ISO 273, tap drills, counterbores for ISO 4762, 90-degree countersinks for ISO 10642, M1.6 to M24 first (UNC/UNF later)
- [ ] Simple, counterbore, countersink and tapped holes on faces or sketch points
- [ ] Cosmetic threads
- [ ] Modeled threads (ISO 68-1 profile) with a clearance option for 3D printing

## Phase 11: STEP

- [ ] Part 21 parser and writer
- [ ] STEP export
- [ ] STEP import with healing
- [ ] Import reports
- [ ] Fixture corpus
- [ ] Round-trip tests

## Phase 12: Versions and branches

Part a: commits, tags, branches and diff.

- [ ] Commits and tags
- [ ] Branches and checkout
- [ ] Diff UI

Part b: merge.

- [ ] Three-way merge
- [ ] Conflict UI

## Phase 13: Scripting

- [ ] Rhai engine
- [ ] API
- [ ] Macros
- [ ] Script features
- [ ] Script panel
- [ ] Examples (gear, hex nut, enclosure)

## Phase 14: Polish and release

- [ ] On-canvas handles
- [ ] Preferences
- [ ] Keymap
- [ ] Autosave and recovery
- [ ] Performance pass
- [ ] Native installers
- [ ] Web deploy (GitHub Pages)
- [ ] User docs

## Later

Not scheduled yet: assemblies and joints, then 2D drawings, a surface-modeling workspace, text and emboss, design variants and configurations, a command palette, 3D-print analysis, SVG and DXF import, sheet metal, collaboration, and plugins.
