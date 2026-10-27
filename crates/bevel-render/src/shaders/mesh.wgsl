// SPDX-License-Identifier: GPL-3.0-or-later
// Shaded triangle meshes: camera-relative key and fill lights plus a soft
// specular highlight, so faces stay readable from every direction.
//
// The `VertexIn` attributes must match MESH_VERTEX_FORMATS in pack.rs.

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) id: u32,
};

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal_view: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) @interpolate(flat) id: u32,
};

@vertex
fn vs_main(in: VertexIn) -> VertexOut {
    var out: VertexOut;
    out.position = globals.view_proj * vec4<f32>(in.position, 1.0);
    out.normal_view = (globals.view * vec4<f32>(in.normal, 0.0)).xyz;
    out.color = in.color;
    out.id = in.id;
    return out;
}

@fragment
fn fs_main(in: VertexOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    var n = normalize(in.normal_view);
    if !front {
        n = -n;
    }
    let key = normalize(vec3<f32>(-0.35, 0.55, 1.0));
    let fill = normalize(vec3<f32>(0.6, -0.3, 0.5));
    let diffuse = 0.62 * max(dot(n, key), 0.0) + 0.18 * max(dot(n, fill), 0.0) + 0.24;
    let half_vector = normalize(key + vec3<f32>(0.0, 0.0, 1.0));
    let specular = 0.18 * pow(max(dot(n, half_vector), 0.0), 48.0);
    let shaded = in.color.rgb * diffuse + vec3<f32>(specular);
    return to_srgb(vec4<f32>(tint_face(shaded, highlight_state(in.id)), in.color.a));
}

// Id pass: the face id of each pixel, for picking. Written to an R32Uint
// target, 0 where nothing pickable is drawn.
@fragment
fn fs_id(in: VertexOut) -> @location(0) vec4<u32> {
    return vec4<u32>(in.id, 0u, 0u, 0u);
}
