// SPDX-License-Identifier: GPL-3.0-or-later
// Ground grid in the XY plane. Lines are computed per pixel from the world
// position with screen-space derivatives, which keeps them one pixel wide and
// anti-aliased at any zoom. Minor lines fade out before they get denser than
// a few pixels, and the whole grid fades with distance from its centre.

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) world: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> VertexOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, 1.0),
    );
    let world = globals.grid_center.xy + corners[index] * globals.grid.w;
    var out: VertexOut;
    out.position = globals.view_proj * vec4<f32>(world, 0.0, 1.0);
    out.world = world;
    return out;
}

// Coverage of the grid lines with spacing `cell` at position `p`, and how many
// cells one pixel spans (used to fade out lines that would be too dense).
fn grid_coverage(p: vec2<f32>, cell: f32) -> vec2<f32> {
    let c = p / cell;
    let w = max(fwidth(c), vec2<f32>(1e-6));
    let distance_in_pixels = abs(fract(c - 0.5) - 0.5) / w;
    let line = 1.0 - min(min(distance_in_pixels.x, distance_in_pixels.y), 1.0);
    return vec2<f32>(line, max(w.x, w.y));
}

// Coverage of the line `coordinate == 0`.
fn axis_coverage(coordinate: f32) -> f32 {
    let w = max(fwidth(coordinate), 1e-6);
    return 1.0 - min(abs(coordinate) / (1.5 * w), 1.0);
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    let minor = grid_coverage(in.world, globals.grid.x);
    let major = grid_coverage(in.world, globals.grid.y);
    let x_axis = axis_coverage(in.world.y);
    let y_axis = axis_coverage(in.world.x);

    let minor_fade = 1.0 - smoothstep(0.15, 0.4, minor.y);
    let major_fade = 1.0 - smoothstep(0.15, 0.4, major.y);
    let r = length(in.world - globals.grid_center.xy);
    let distance_fade = 1.0 - smoothstep(0.45 * globals.grid.z, globals.grid.z, r);

    var color = globals.grid_minor;
    color.a *= minor.x * minor_fade;
    let major_a = globals.grid_major.a * major.x * major_fade;
    color = vec4<f32>(mix(color.rgb, globals.grid_major.rgb, step(color.a, major_a)), max(color.a, major_a));
    let x_a = globals.axis_x.a * x_axis;
    color = vec4<f32>(mix(color.rgb, globals.axis_x.rgb, step(color.a, x_a)), max(color.a, x_a));
    let y_a = globals.axis_y.a * y_axis;
    color = vec4<f32>(mix(color.rgb, globals.axis_y.rgb, step(color.a, y_a)), max(color.a, y_a));

    color.a *= distance_fade;
    if color.a < 0.002 {
        discard;
    }
    return to_srgb(color);
}
