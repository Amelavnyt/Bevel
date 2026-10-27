// SPDX-License-Identifier: GPL-3.0-or-later
// Per-frame uniforms shared by every viewport pass. Keep in sync with the
// `globals` offsets in pack.rs (a test compares them with this struct).
//
// Uniform arrays need a 16-byte element stride to work on WebGL2, hence the
// selected ids are packed four to a vec4.

struct Globals {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    // xyz: eye position in world space.
    eye: vec4<f32>,
    // x: minor cell size, y: major cell size, z: fade radius, w: half extent.
    grid: vec4<f32>,
    // xy: grid centre in the XY plane.
    grid_center: vec4<f32>,
    // xy: viewport size in pixels, z: 1 for orthographic projection.
    viewport: vec4<f32>,
    background_top: vec4<f32>,
    background_bottom: vec4<f32>,
    grid_minor: vec4<f32>,
    grid_major: vec4<f32>,
    axis_x: vec4<f32>,
    axis_y: vec4<f32>,
    // Highlight tints, linear: hovered, selected, selected and hovered.
    hover: vec4<f32>,
    selected: vec4<f32>,
    selected_hover: vec4<f32>,
    // x: hovered id (0 for none).
    highlight: vec4<u32>,
    // Up to 64 selected ids, padded with 0.
    selected_ids: array<vec4<u32>, 16>,
};

@group(0) @binding(0) var<uniform> globals: Globals;

// The render target stores sRGB-encoded values in a UNORM format (see
// OUTPUT_FORMAT in renderer.rs), so every pass encodes its linear result.
fn to_srgb(color: vec4<f32>) -> vec4<f32> {
    let c = clamp(color.rgb, vec3<f32>(0.0), vec3<f32>(1.0));
    let low = c * 12.92;
    let high = 1.055 * pow(c, vec3<f32>(1.0 / 2.4)) - 0.055;
    return vec4<f32>(select(high, low, c <= vec3<f32>(0.0031308)), color.a);
}

// How strongly faces take on a highlight tint.
const FACE_MIX_HOVER: f32 = 0.45;
const FACE_MIX_SELECTED: f32 = 0.6;
const FACE_MIX_SELECTED_HOVER: f32 = 0.7;

const HIGHLIGHT_HOVERED: u32 = 1u;
const HIGHLIGHT_SELECTED: u32 = 2u;

// Bit 0: the id is hovered. Bit 1: the id is selected. Id 0 is never
// highlighted.
fn highlight_state(id: u32) -> u32 {
    if id == 0u {
        return 0u;
    }
    var state = 0u;
    if id == globals.highlight.x {
        state = HIGHLIGHT_HOVERED;
    }
    for (var i = 0u; i < 16u; i++) {
        if any(globals.selected_ids[i] == vec4<u32>(id)) {
            state |= HIGHLIGHT_SELECTED;
            break;
        }
    }
    return state;
}

// A face colour tinted for the highlight state (linear in, linear out).
fn tint_face(color: vec3<f32>, state: u32) -> vec3<f32> {
    if state == (HIGHLIGHT_HOVERED | HIGHLIGHT_SELECTED) {
        return mix(color, globals.selected_hover.rgb, FACE_MIX_SELECTED_HOVER);
    }
    if state == HIGHLIGHT_SELECTED {
        return mix(color, globals.selected.rgb, FACE_MIX_SELECTED);
    }
    if state == HIGHLIGHT_HOVERED {
        return mix(color, globals.hover.rgb, FACE_MIX_HOVER);
    }
    return color;
}

// An edge colour replaced by the highlight colour at full strength.
fn tint_edge(color: vec3<f32>, state: u32) -> vec3<f32> {
    if state == (HIGHLIGHT_HOVERED | HIGHLIGHT_SELECTED) {
        return globals.selected_hover.rgb;
    }
    if state == HIGHLIGHT_SELECTED {
        return globals.selected.rgb;
    }
    if state == HIGHLIGHT_HOVERED {
        return globals.hover.rgb;
    }
    return color;
}
