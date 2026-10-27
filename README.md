# Bevel

Bevel is an open-source, parametric, history-based 3D CAD application, written entirely in Rust. Every modeling step becomes an entry in a timeline that you can scrub, edit, reorder, suppress and branch. It runs as a native desktop app (Windows, macOS, Linux) and in the browser from the same code.

Bevel has its own geometry kernel: a boundary-representation kernel built on NURBS, with exact analytic curves and surfaces. No C or C++ code is compiled into it.

**Status:** early development. Phase 0 (the skeleton) is done: the workspace, the application shell with its full layout and a 3D viewport that shows a test cube. The kernel, sketcher and modeling features come next; see [docs/ROADMAP.md](docs/ROADMAP.md).

## Building

You need a Rust toolchain from [rustup](https://rustup.rs). The repository pins the exact version in `rust-toolchain.toml`, and rustup installs it, along with the `wasm32-unknown-unknown` target, the first time you build.

```sh
# Desktop app
cargo run -p bevel-app --release

# Headless command-line tool
cargo run -p bevel-cli -- --help

# Tests
cargo test --workspace
```

On Linux, Bevel loads X11 or Wayland and the graphics drivers at run time, so no development packages are needed to build it.

### In the browser

The web build uses [Trunk](https://trunkrs.dev):

```sh
cargo install --locked trunk
trunk serve --config web/Trunk.toml --open
```

Open <http://localhost:8080>. A browser with WebGPU or WebGL2 is required. `trunk build --release --config web/Trunk.toml` writes a static site to `dist/`.

## Modeling

Bevel is history-based: every feature you create is a step on the timeline at the bottom of the window.

- **Create a primitive**: click Box in the CREATE group (Cylinder, Sphere, Cone and Torus are in the CREATE menu). A dialog shows its dimensions and placement with a live preview; every field takes an expression such as `40 mm`, `width * 2` or `1.5 in`, and a bare number uses the document's unit. OK adds the feature.
- **Edit**: double-click a feature on the timeline, or right-click it for Edit, Rename, Suppress, Roll marker here and Delete.
- **Timeline**: drag a feature to reorder it, or drag the teal marker to roll the model back to any point; the playback buttons step the marker.
- **Parameters**: Edit > Change Parameters (or Change Parameters in the MODIFY menu) lists every dimension (`d1`, `d2`, ... ) and lets you add your own named parameters and use them in any field.
- **Undo and redo**: Ctrl+Z and Ctrl+Shift+Z (or Ctrl+Y).
- **Files**: File > New, Open, Save and Save As (Ctrl+N, Ctrl+O, Ctrl+S, Ctrl+Shift+S) work with `.bevel` files; in the browser, Open uses the file picker and Save downloads the file. Help > Open Sample loads a document with one of each primitive.

## Using the viewport

| Action | Mouse |
|---|---|
| Orbit | Right drag, or Shift + middle drag |
| Pan | Middle drag, or Shift + right drag |
| Zoom (towards the cursor) | Scroll wheel or pinch |
| Fit all | Home, or View > Fit All |
| Select a face or edge | Click; Shift-click adds or removes; Esc clears |
| Select a body | Click its name under Bodies in the browser |

Press M (or click Measure in the INSPECT group) to open the Measure dialog: it shows the area of a selected face, the length of an edge, the volume, area and centre of mass of a whole body, and totals for several faces or edges, each with a copy button. File > Export writes the visible bodies as STL or 3MF (a save dialog on the desktop, a download in the browser). Click a face of the view cube (top right of the viewport) to look from that side. The navigation bar at the bottom of the viewport has Home view, Fit All, the perspective/orthographic switch and the grid and origin toggles. Help > Viewport Controls lists all of this in the app.

## Repository layout

```
crates/      the Bevel crates, one per layer of the architecture
docs/        architecture, roadmap, design decisions, kernel notes, file format
web/         index.html and Trunk configuration for the browser build
xtask/       repository automation (cargo xtask)
assets/      icons, standards tables and example scripts
fixtures/    documents and models used by tests
```

Start with [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for how the pieces fit together, and [docs/DECISIONS.md](docs/DECISIONS.md) for why.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for coding standards, the checks every change must pass, and the commit message format.

## License

Bevel is free software, licensed under the GNU General Public License, version 3 or (at your option) any later version. See [LICENSE](LICENSE).
