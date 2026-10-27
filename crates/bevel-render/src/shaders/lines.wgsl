// SPDX-License-Identifier: GPL-3.0-or-later
// Thick line segments (edges, axes). Each segment is one instance that the
// vertex shader expands to a screen-space quad of the requested pixel width,
// extended by half the width at both ends so polylines join without gaps.
// Lines are pulled slightly towards the camera so edges win the depth test
// against the faces they bound.
//
// The `Instance` attributes must match LINE_INSTANCE_FORMATS in pack.rs.

struct Instance {
    @location(0) start: vec3<f32>,
    @location(1) end: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) id: u32,
    @location(4) width: f32,
};

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) @interpolate(flat) id: u32,
};

// Moves a clip-space point a little towards the camera.
fn pull_towards_camera(clip: vec4<f32>) -> vec4<f32> {
    var out = clip;
    if globals.viewport.z > 0.5 {
        // Orthographic depth is linear: offset by a fixed fraction of the range.
        out.z += 2e-5 * out.w;
    } else {
        // Reverse-Z perspective depth is n / distance: scaling it moves the
        // point a fixed fraction of its distance towards the eye.
        out.z *= 1.0015;
    }
    return out;
}

// Clip-space position of one corner of the segment's quad. `extra_width` is
// added to the segment's width (the id pass makes lines easier to hit).
fn expand(segment: Instance, index: u32, extra_width: f32) -> vec4<f32> {
    // Corners as (0 = start / 1 = end, side -1 / +1) for the two triangles.
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(0.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0),
    );
    let corner = corners[index];

    var a = globals.view_proj * vec4<f32>(segment.start, 1.0);
    var b = globals.view_proj * vec4<f32>(segment.end, 1.0);
    // A point is in front of the near plane when w - z >= 0 (reverse-Z).
    // Cut the segment there, otherwise the screen-space maths below breaks
    // for segments that reach behind the camera.
    let da = a.w - a.z;
    let db = b.w - b.z;
    if da < 0.0 && db < 0.0 {
        return vec4<f32>(2.0, 2.0, 2.0, 1.0);
    }
    if da < 0.0 {
        a = mix(a, b, da / (da - db));
    } else if db < 0.0 {
        b = mix(b, a, db / (db - da));
    }
    a = pull_towards_camera(a);
    b = pull_towards_camera(b);

    // Work in pixels, centred on the viewport.
    let half_size = 0.5 * globals.viewport.xy;
    let pa = a.xy / a.w * half_size;
    let pb = b.xy / b.w * half_size;
    let along = pb - pa;
    let length_px = length(along);
    var dir = vec2<f32>(1.0, 0.0);
    if length_px > 1e-4 {
        dir = along / length_px;
    }
    let normal = vec2<f32>(-dir.y, dir.x);

    let at_end = corner.x > 0.5;
    let anchor = select(a, b, at_end);
    let anchor_px = select(pa, pb, at_end);
    let half_width = 0.5 * (segment.width + extra_width);
    let toward_end = corner.x * 2.0 - 1.0;
    let px = anchor_px + normal * (corner.y * half_width) + dir * (toward_end * half_width);
    return vec4<f32>(px / half_size * anchor.w, anchor.z, anchor.w);
}

@vertex
fn vs_main(@builtin(vertex_index) index: u32, segment: Instance) -> VertexOut {
    var out: VertexOut;
    out.position = expand(segment, index, 0.0);
    out.color = segment.color;
    out.id = segment.id;
    return out;
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    return to_srgb(vec4<f32>(tint_edge(in.color.rgb, highlight_state(in.id)), in.color.a));
}
