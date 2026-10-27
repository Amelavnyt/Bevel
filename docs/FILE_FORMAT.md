# File format

A Bevel document is a `.bevel` file: a ZIP container (deflate, written by pure-Rust code) around a content-addressed object store. This page describes format version 1. The implementation is `bevel_doc::file`; see also [ADR-0007](DECISIONS.md#adr-0007-content-addressed-object-store-bevel-is-a-versioned-zip).

## Container

| Entry | Contents |
|---|---|
| `manifest.json` | What the file is and which format version it uses |
| `working.json` | The working state: the timeline, pointing at objects |
| `objects/<hash>` | One JSON object per entry, named by the BLAKE3 hash of its bytes (64 lower-case hex digits) |

Coming with later phases: `refs/heads/*` and `refs/tags/*` (branches and named versions, Phase 12), body blobs for imported geometry (Phase 11) and `thumbnail.png`. Readers ignore entries they do not know.

Writing is deterministic: entries are written in the order above (objects sorted by hash) with a fixed timestamp, so the same document always gives the same bytes. That keeps fixtures stable and diffs meaningful.

## manifest.json

```json
{
  "format": "bevel",
  "format_version": 1,
  "app_version": "0.1.0",
  "head": "refs/heads/main"
}
```

`head` names the branch the working state belongs to. In version 1 no commits exist yet, so the ref itself is absent.

## working.json

```json
{
  "features": [
    { "object": "<hash>", "suppressed": false }
  ],
  "marker": 1,
  "parameters": "<hash>",
  "units": { "length": "Millimetre", "angle": "Degree" },
  "body_props": [
    { "body": { "feature": 1, "index": 0 },
      "props": { "name": "Lid", "color": [200, 40, 40], "visible": true } }
  ],
  "next_feature_id": 2
}
```

- `features`: the timeline in order. `marker` is how many features run; the rest are rolled back.
- `parameters`: the object holding the parameter table.
- `units`: the default units for bare numbers. Internally everything is millimetres and radians.
- `body_props`: display properties that differ from the defaults (name from the creating feature, default grey, visible).
- `next_feature_id`: feature ids are never reused, even after deletion. A reader uses at least one more than the largest id present.

## Objects

**Feature.** Its `id`, display `name` and definition:

```json
{
  "id": 1,
  "name": "Box1",
  "def": {
    "type": "primitive",
    "kind": "box",
    "plane": "xy",
    "inputs": { "length": "d1", "width": "d2", "height": "d3", "x": "d4", "y": "d5", "z": "d6" },
    "body_mode": "new_body"
  }
}
```

Each input names a parameter. Primitive kinds and their input keys:

| Kind | Inputs |
|---|---|
| `box` | `length`, `width`, `height` (corner at the placement point) |
| `cylinder` | `radius`, `height` |
| `sphere` | `radius` (centred on the placement point) |
| `cone` | `bottom_radius`, `top_radius` (zero for an apex), `height` |
| `torus` | `major_radius`, `minor_radius` |

All kinds also have `x`, `y`, `z`: the placement point's offset from the origin, in the plane's frame. `plane` is `xy`, `xz` (normal −Y) or `yz` (normal +X).

**Parameter table.** The parameters in insertion order:

```json
{ "params": [
  { "name": "d1", "expression": "width * 2", "comment": "", "kind": "Model" },
  { "name": "width", "expression": "20 mm", "comment": "overall", "kind": "User" }
] }
```

`Model` parameters belong to a feature input and are deleted with their feature; `User` parameters are created in the Parameters dialog. Expressions are stored as typed. The syntax supports units (`mm cm m um in ft deg rad`), `+ - * / ^`, parentheses, `pi` and the functions `sin cos tan asin acos atan atan2 sqrt abs min max round floor ceil`.

## Reading

1. Read `manifest.json`; refuse anything whose `format` is not `bevel`.
2. Parse `working.json` as JSON and pass it through `migrate(value, format_version)`.
3. Load each object and check that its bytes hash to its name; a mismatch means the file is damaged.
4. Rebuild the timeline, checking that ids are unique and every feature comes after the features it depends on.

## Versions and migrations

Every change to the format bumps `format_version` and adds a step to `bevel_doc::file::migrate`, which rewrites the JSON of the older version into the next one. A file from a newer version is refused with a message saying a newer Bevel is needed. `fixtures/bevel/v<N>/` keeps sample files written by each version, and a test loads and regenerates every one of them, so old files keep opening. Run the doc tests with `BEVEL_WRITE_FIXTURES=1` to write the current version's fixtures.

`bevel-cli unpack` and `pack`, which convert to and from a plain directory, arrive later.
