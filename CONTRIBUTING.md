# Contributing to Bevel

Thanks for your interest in Bevel. This document covers how to set up, the rules the code follows, the checks every change must pass, and how commits and pull requests are written.

## Setup

1. Install Rust with [rustup](https://rustup.rs). The toolchain version and the `wasm32-unknown-unknown` target come from `rust-toolchain.toml`.
2. Enable the commit message hook:

   ```sh
   git config core.hooksPath .githooks
   ```

3. For the browser build, install Trunk: `cargo install --locked trunk`.

## Before every commit

Run the full local check:

```sh
cargo xtask ci
```

It runs, in order:

- `cargo fmt --all --check`
- `cargo clippy --workspace --all-targets -- -D warnings`
- `cargo clippy -p bevel-app --target wasm32-unknown-unknown -- -D warnings`
- `cargo test --workspace`
- the wasm build of `bevel-app` (`cargo xtask wasm`)
- the pure-Rust guard (`cargo xtask check-pure-rust`)
- the layer check (`cargo xtask check-layers`)

CI runs the same checks, plus `cargo deny check` (licenses, bans, advisories), the commit message check and a Trunk build of the web app. Windows and macOS builds run weekly and on release branches.

## Hard rules

- **Pure Rust.** Nothing written in C or C++ is compiled into Bevel: no crates whose build runs `cc`, `cmake` or `bindgen`, or links system libraries found with `pkg-config`. Crates that call OS or graphics APIs from Rust (winit, wgpu, web-sys) are fine. `cargo xtask check-pure-rust` enforces this. Exceptions are listed, with reasons, in `xtask/pure-rust-allowlist.txt` and need a maintainer's approval.
- **Our own kernel.** No third-party CAD kernels. Code ported from GPL-compatible projects must keep its notices and attribution; clean implementations from books and papers are preferred. Never copy from incompatible or proprietary sources.
- **Layering.** A crate may depend only on crates in lower layers (see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)). `cargo xtask check-layers` enforces this; a new crate must be added to its table.
- **Commands only.** The UI never changes the document directly. Every change is a `Command`, which is what makes undo/redo, versions, scripting and tests work.
- **No panics in libraries.** Library crates return typed errors for bad input or bad geometry. No `unwrap`, `expect` or `panic!` outside tests, benchmarks and binary start-up code; clippy denies them.
- **Deterministic regeneration.** The same document always produces the same bodies and the same topological names. Collections whose order reaches output or serialization are `BTreeMap` or `IndexMap`, never `HashMap`; randomness is always seeded.
- **Platform code stays in one place.** All `cfg(target_arch = "wasm32")` code lives in `crates/bevel-app/src/platform/`.
- **Architecture changes are proposed, not made silently.** Write a decision record in [docs/DECISIONS.md](docs/DECISIONS.md) and discuss it first.

## Coding standards

- Rust edition 2024, the stable toolchain pinned in `rust-toolchain.toml`.
- `unsafe` code is forbidden. Clippy's `pedantic` group is on as warnings; the short allow-list in the workspace `Cargo.toml` says why each exception exists.
- Every source file starts with `// SPDX-License-Identifier: GPL-3.0-or-later`.
- Every public item has rustdoc. Module docs explain the algorithm and cite its source (for NURBS code, the algorithm number in *The NURBS Book*, such as A5.1).
- **Units:** lengths in millimetres, angles in radians, `f64` for all geometry (`f32` only on the GPU). Display units affect only the UI.
- **Tolerances** live in `bevel_math::tol`. Never compare floats with `==`; use the helpers. The modeling space is ±1e5 mm.
- **IDs** are typed newtypes allocated from a per-document counter and never reused. IDs created by procedural operations derive deterministically from the operation and an index.
- **Errors:** `thiserror` in libraries, `anyhow` only in binaries. User-facing errors have a plain-English message plus technical detail.
- **Logging:** `tracing` everywhere; the app shows it in its log panel.
- **Tests:** unit tests next to the code; `proptest` for math, kernel and solver; `insta` snapshots for serialized formats and diagnostics; golden-model tests through the CLI over `fixtures/`; UI smoke tests with `egui_kittest`; `criterion` benchmarks for performance-sensitive code.
- **Dependencies** are few, mature, pure Rust and GPL-3.0-compatible. Add them to `[workspace.dependencies]` with a pinned version. Propose new ones in the pull request with their license and maintenance status.
- Keep [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) and [docs/ROADMAP.md](docs/ROADMAP.md) current, and record anything non-obvious you learn in the relevant doc.

## Commit messages

Bevel uses [Conventional Commits 1.0](https://www.conventionalcommits.org/):

```
type(scope): subject

Optional body explaining what changed and why.

Optional footers.
```

- **Types:** `feat`, `fix`, `perf`, `refactor`, `test`, `docs`, `build`, `ci`, `chore`, `style`, `revert`.
- **Scope:** the crate name without `bevel-` (`kernel`, `geom`, `sketch`, `solver`, `ui`, `render`, `doc`, `io`, ...) or `ci`, `deps`, `docs`. Omit it for repository-wide changes.
- **Subject:** imperative mood ("add", not "added" or "adds"), lower case after the colon, no trailing period. The whole header is at most 72 characters; aim for about 50.
- **Body,** when the change isn't obvious: a blank line after the subject, lines wrapped at 72 columns, explaining what changed and why rather than listing files.
- **Footers** are limited to `Fixes #N`, `Refs #N` and `BREAKING CHANGE: ...`. Other trailers are rejected.
- **Tone:** plain and specific, one maintainer writing to another. No emojis, no filler such as "This commit...".
- **One logical change per commit.** Each commit builds and passes the tests. Don't mix refactors with behaviour changes.

`cargo xtask check-commits` checks every commit on a branch in CI, and the `commit-msg` hook runs the same check locally.

Examples:

```
feat(geom): add knot insertion for NURBS curves

Implements Boehm's algorithm (The NURBS Book, A5.1). Curve splitting
and Bézier decomposition both build on it.
```

```
fix(kernel): keep face orientation when merging coplanar fragments
```

```
test(solver): cover conflicting tangent constraints
```

## Branches and pull requests

- Work on a branch named like the commit type and topic: `feat/sketch-solver`, `fix/coplanar-union`.
- Pull request titles use the same Conventional Commits form. The description gives a short summary, the reason for the change, and how it was tested.
- Pull requests are merged with a merge commit once CI is green.

## License

By contributing, you agree that your contributions are licensed under the GNU General Public License, version 3 or later.
