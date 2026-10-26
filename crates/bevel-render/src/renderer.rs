// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{Camera, LineSet, Projection, TriangleMesh};
use bevel_math::{Aabb3, Point3};
use wgpu::util::DeviceExt as _;

/// Colour format of the texture the renderer resolves into.
///
/// The UI composites in gamma space and samples textures as stored, so the
/// target holds sRGB-encoded values in a plain UNORM format: the shaders
/// encode their linear results with `to_srgb` (shaders/globals.wgsl).
const OUTPUT_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// Number of `f32`s in the `Globals` uniform block (see shaders/globals.wgsl).
const GLOBALS_FLOATS: usize = 16 + 16 + 4 * 10;
/// Largest render target edge we ask for; larger viewports are clamped.
const MAX_TARGET_SIZE: u32 = 8192;

/// Mesh vertex: position, normal, colour.
const MESH_VERTEX_FLOATS: usize = 3 + 3 + 4;
/// Line vertex: position, colour.
const LINE_VERTEX_FLOATS: usize = 3 + 4;

/// Viewport colours, as sRGB bytes with straight alpha.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ViewportTheme {
    /// Background at the top of the viewport.
    pub background_top: [u8; 4],
    /// Background at the bottom of the viewport.
    pub background_bottom: [u8; 4],
    /// Minor grid lines.
    pub grid_minor: [u8; 4],
    /// Major grid lines (every tenth line).
    pub grid_major: [u8; 4],
    /// X axis.
    pub axis_x: [u8; 4],
    /// Y axis.
    pub axis_y: [u8; 4],
    /// Z axis.
    pub axis_z: [u8; 4],
}

impl ViewportTheme {
    /// The default dark theme.
    pub const DARK: Self = Self {
        background_top: [58, 62, 70, 255],
        background_bottom: [26, 28, 32, 255],
        grid_minor: [120, 126, 136, 70],
        grid_major: [140, 146, 158, 150],
        axis_x: [214, 72, 72, 230],
        axis_y: [92, 186, 92, 230],
        axis_z: [80, 128, 232, 255],
    };
}

/// Converts an sRGB byte colour with straight alpha to linear floats.
fn linear(color: [u8; 4]) -> [f32; 4] {
    let channel = |c: u8| {
        let c = f32::from(c) / 255.0;
        if c <= 0.040_45 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };
    [
        channel(color[0]),
        channel(color[1]),
        channel(color[2]),
        f32::from(color[3]) / 255.0,
    ]
}

fn f32_bytes(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

fn u32_bytes(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// The texture a frame was rendered into.
pub struct RenderOutput<'a> {
    /// View of the resolved colour texture: [`wgpu::TextureFormat::Rgba8Unorm`]
    /// holding sRGB-encoded colours with straight alpha.
    pub view: &'a wgpu::TextureView,
    /// `true` if the texture was recreated this frame (for example after a
    /// resize), so any binding of the previous texture must be refreshed.
    pub recreated: bool,
}

struct Targets {
    size: [u32; 2],
    /// Multisampled colour target, `None` when rendering without MSAA.
    msaa: Option<wgpu::TextureView>,
    output: wgpu::TextureView,
    depth: wgpu::TextureView,
}

struct GpuMesh {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
}

struct GpuLines {
    vertices: wgpu::Buffer,
    vertex_count: u32,
}

/// Renders the 3D viewport into an offscreen texture.
pub struct ViewportRenderer {
    sample_count: u32,
    theme: ViewportTheme,
    globals: wgpu::Buffer,
    globals_bind_group: wgpu::BindGroup,
    pipelines: Pipelines,
    meshes: Vec<GpuMesh>,
    lines: Option<GpuLines>,
    /// Z axis segment; its length follows the grid, so it is rewritten every frame.
    axis_lines: wgpu::Buffer,
    scene_bounds: Aabb3,
    targets: Option<Targets>,
}

impl ViewportRenderer {
    /// Creates the renderer's pipelines on `device`.
    ///
    /// `adapter` is only queried for multisampling support; the renderer
    /// falls back to no MSAA where 4x is unavailable.
    pub fn new(device: &wgpu::Device, adapter: &wgpu::Adapter) -> Self {
        let sample_count = {
            let color = adapter.get_texture_format_features(OUTPUT_FORMAT).flags;
            let depth = adapter.get_texture_format_features(DEPTH_FORMAT).flags;
            if color.sample_count_supported(4)
                && depth.sample_count_supported(4)
                && color.contains(wgpu::TextureFormatFeatureFlags::MULTISAMPLE_RESOLVE)
            {
                4
            } else {
                1
            }
        };
        tracing::info!(sample_count, "viewport renderer created");

        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("viewport globals"),
            size: (GLOBALS_FLOATS * 4) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport globals"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let globals_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport globals"),
            layout: &globals_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals.as_entire_binding(),
            }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport"),
            bind_group_layouts: &[Some(&globals_layout)],
            immediate_size: 0,
        });

        let pipelines = Pipelines::new(device, &layout, sample_count);

        let axis_lines = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("viewport axis lines"),
            size: (2 * LINE_VERTEX_FLOATS * 4) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            sample_count,
            theme: ViewportTheme::DARK,
            globals,
            globals_bind_group,
            pipelines,
            meshes: Vec::new(),
            lines: None,
            axis_lines,
            scene_bounds: Aabb3::EMPTY,
            targets: None,
        }
    }

    /// Sets the viewport colours.
    pub fn set_theme(&mut self, theme: ViewportTheme) {
        self.theme = theme;
    }

    /// Bounding box of everything set with [`ViewportRenderer::set_scene`].
    pub fn scene_bounds(&self) -> Aabb3 {
        self.scene_bounds
    }

    /// Replaces the drawn geometry. Meshes or line sets that don't fit
    /// 32-bit indices are skipped with a warning.
    pub fn set_scene(&mut self, device: &wgpu::Device, meshes: &[TriangleMesh], lines: &[LineSet]) {
        self.scene_bounds = meshes
            .iter()
            .map(TriangleMesh::bounds)
            .chain(lines.iter().map(LineSet::bounds))
            .fold(Aabb3::EMPTY, Aabb3::union);

        self.meshes = meshes
            .iter()
            .filter_map(|mesh| {
                if mesh.normals.len() != mesh.positions.len() {
                    tracing::warn!("skipping mesh: normal count does not match position count");
                    return None;
                }
                let vertex_count = u32::try_from(mesh.positions.len()).ok()?;
                if mesh.indices.iter().any(|&i| i >= vertex_count) {
                    tracing::warn!("skipping mesh: index out of range");
                    return None;
                }
                let index_count = u32::try_from(mesh.indices.len()).ok()?;
                let mut vertices = Vec::with_capacity(mesh.positions.len() * MESH_VERTEX_FLOATS);
                for (p, n) in mesh.positions.iter().zip(&mesh.normals) {
                    vertices.extend_from_slice(p);
                    vertices.extend_from_slice(n);
                    vertices.extend_from_slice(&mesh.color);
                }
                Some(GpuMesh {
                    vertices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("viewport mesh vertices"),
                        contents: &f32_bytes(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    }),
                    indices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("viewport mesh indices"),
                        contents: &u32_bytes(&mesh.indices),
                        usage: wgpu::BufferUsages::INDEX,
                    }),
                    index_count,
                })
            })
            .collect();

        let mut line_vertices = Vec::new();
        for set in lines {
            for p in &set.positions {
                line_vertices.extend_from_slice(p);
                line_vertices.extend_from_slice(&set.color);
            }
        }
        self.lines = u32::try_from(line_vertices.len() / LINE_VERTEX_FLOATS)
            .ok()
            .filter(|&n| n > 0)
            .map(|vertex_count| GpuLines {
                vertices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("viewport line vertices"),
                    contents: &f32_bytes(&line_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
                vertex_count,
            });
    }

    /// Renders one frame of `size` pixels as seen by `camera`.
    ///
    /// The size is clamped to at least 1x1 and at most the renderer's
    /// maximum target size.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        size: [u32; 2],
        camera: &Camera,
    ) -> RenderOutput<'_> {
        let size = size.map(|s| s.clamp(1, MAX_TARGET_SIZE));
        let (targets, recreated) = match self.targets.take() {
            Some(targets) if targets.size == size => (targets, false),
            _ => (self.create_targets(device, size), true),
        };

        let grid = GridLayout::for_camera(camera);
        queue.write_buffer(
            &self.globals,
            0,
            &f32_bytes(&self.globals_data(camera, size, &grid)),
        );
        let axis_z = linear(self.theme.axis_z);
        let z_top = grid.fade_radius as f32 * 0.5;
        let axis_vertices = [
            0.0, 0.0, 0.0, axis_z[0], axis_z[1], axis_z[2], axis_z[3], //
            0.0, 0.0, z_top, axis_z[0], axis_z[1], axis_z[2], axis_z[3],
        ];
        queue.write_buffer(&self.axis_lines, 0, &f32_bytes(&axis_vertices));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("viewport"),
        });
        {
            let (view, resolve_target) = match &targets.msaa {
                Some(msaa) => (msaa, Some(&targets.output)),
                None => (&targets.output, None),
            };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth,
                    depth_ops: Some(wgpu::Operations {
                        // Reverse-Z: 0 is infinitely far away.
                        load: wgpu::LoadOp::Clear(0.0),
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.globals_bind_group, &[]);

            pass.set_pipeline(&self.pipelines.background);
            pass.draw(0..3, 0..1);

            pass.set_pipeline(&self.pipelines.mesh);
            for mesh in &self.meshes {
                pass.set_vertex_buffer(0, mesh.vertices.slice(..));
                pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, 0..1);
            }

            pass.set_pipeline(&self.pipelines.lines);
            if let Some(lines) = &self.lines {
                pass.set_vertex_buffer(0, lines.vertices.slice(..));
                pass.draw(0..lines.vertex_count, 0..1);
            }
            pass.set_vertex_buffer(0, self.axis_lines.slice(..));
            pass.draw(0..2, 0..1);

            // Translucent grid last so it blends over what's already drawn.
            pass.set_pipeline(&self.pipelines.grid);
            pass.draw(0..6, 0..1);
        }
        queue.submit([encoder.finish()]);

        let targets = self.targets.insert(targets);
        RenderOutput {
            view: &targets.output,
            recreated,
        }
    }

    fn create_targets(&self, device: &wgpu::Device, size: [u32; 2]) -> Targets {
        let extent = wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        };
        let texture = |label, format, sample_count, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: extent,
                    mip_level_count: 1,
                    sample_count,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&wgpu::TextureViewDescriptor::default())
        };
        let output = texture(
            "viewport output",
            OUTPUT_FORMAT,
            1,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        );
        let msaa = (self.sample_count > 1).then(|| {
            texture(
                "viewport msaa colour",
                OUTPUT_FORMAT,
                self.sample_count,
                wgpu::TextureUsages::RENDER_ATTACHMENT,
            )
        });
        let depth = texture(
            "viewport depth",
            DEPTH_FORMAT,
            self.sample_count,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        Targets {
            size,
            msaa,
            output,
            depth,
        }
    }

    fn globals_data(
        &self,
        camera: &Camera,
        size: [u32; 2],
        grid: &GridLayout,
    ) -> [f32; GLOBALS_FLOATS] {
        let aspect = f64::from(size[0]) / f64::from(size[1]);
        let eye = camera.eye().as_vec3();
        let orthographic = if camera.projection == Projection::Orthographic {
            1.0
        } else {
            0.0
        };
        let mut data = [0.0f32; GLOBALS_FLOATS];
        let vectors: [[f32; 4]; 10] = [
            [eye.x, eye.y, eye.z, 1.0],
            [
                grid.minor as f32,
                grid.major as f32,
                grid.fade_radius as f32,
                grid.half_extent as f32,
            ],
            [grid.center.x as f32, grid.center.y as f32, 0.0, 0.0],
            [size[0] as f32, size[1] as f32, orthographic, 0.0],
            linear(self.theme.background_top),
            linear(self.theme.background_bottom),
            linear(self.theme.grid_minor),
            linear(self.theme.grid_major),
            linear(self.theme.axis_x),
            linear(self.theme.axis_y),
        ];
        data[..16].copy_from_slice(&camera.view_projection(aspect).as_mat4().to_cols_array());
        data[16..32].copy_from_slice(&camera.view().as_mat4().to_cols_array());
        for (chunk, v) in data[32..].chunks_exact_mut(4).zip(vectors) {
            chunk.copy_from_slice(&v);
        }
        data
    }
}

/// Grid spacing and extent chosen for the current view.
#[derive(Clone, Copy, Debug, PartialEq)]
struct GridLayout {
    minor: f64,
    major: f64,
    fade_radius: f64,
    half_extent: f64,
    center: Point3,
}

impl GridLayout {
    fn for_camera(camera: &Camera) -> Self {
        // A power of ten that gives a few dozen minor cells across the view.
        let minor = 10f64
            .powf((camera.half_height() * 0.08).log10().floor())
            .clamp(1e-3, 1e4);
        let major = 10.0 * minor;
        let fade_radius = (camera.distance * 6.0).max(major * 10.0);
        // Centre the grid under the target, snapped to major cells so the
        // fade doesn't slide relative to the lines.
        let snap = |v: f64| (v / major).round() * major;
        Self {
            minor,
            major,
            fade_radius,
            half_extent: fade_radius,
            center: Point3::new(snap(camera.target.x), snap(camera.target.y), 0.0),
        }
    }
}

/// The render pipelines of the viewport passes.
struct Pipelines {
    background: wgpu::RenderPipeline,
    mesh: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    grid: wgpu::RenderPipeline,
}

impl Pipelines {
    fn new(device: &wgpu::Device, layout: &wgpu::PipelineLayout, sample_count: u32) -> Self {
        let builder = PipelineBuilder {
            device,
            layout,
            sample_count,
        };
        let background = builder.build(&PipelineSpec {
            label: "viewport background",
            source: include_str!("shaders/background.wgsl"),
            vertex_layout: None,
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: false,
            depth_compare: wgpu::CompareFunction::Always,
            blend: None,
        });
        let mesh = builder.build(&PipelineSpec {
            label: "viewport meshes",
            source: include_str!("shaders/mesh.wgsl"),
            vertex_layout: Some(&[
                wgpu::VertexFormat::Float32x3,
                wgpu::VertexFormat::Float32x3,
                wgpu::VertexFormat::Float32x4,
            ]),
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::Greater,
            blend: None,
        });
        let lines = builder.build(&PipelineSpec {
            label: "viewport lines",
            source: include_str!("shaders/lines.wgsl"),
            vertex_layout: Some(&[wgpu::VertexFormat::Float32x3, wgpu::VertexFormat::Float32x4]),
            topology: wgpu::PrimitiveTopology::LineList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::GreaterEqual,
            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
        });
        let grid = builder.build(&PipelineSpec {
            label: "viewport grid",
            source: include_str!("shaders/grid.wgsl"),
            vertex_layout: None,
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: false,
            depth_compare: wgpu::CompareFunction::GreaterEqual,
            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
        });
        Self {
            background,
            mesh,
            lines,
            grid,
        }
    }
}

struct PipelineBuilder<'a> {
    device: &'a wgpu::Device,
    layout: &'a wgpu::PipelineLayout,
    sample_count: u32,
}

struct PipelineSpec<'a> {
    label: &'a str,
    source: &'a str,
    /// Attribute formats of the single vertex buffer, `None` for passes
    /// that generate their vertices from the vertex index.
    vertex_layout: Option<&'a [wgpu::VertexFormat]>,
    topology: wgpu::PrimitiveTopology,
    depth_write: bool,
    depth_compare: wgpu::CompareFunction,
    blend: Option<wgpu::BlendState>,
}

impl PipelineBuilder<'_> {
    fn build(&self, spec: &PipelineSpec<'_>) -> wgpu::RenderPipeline {
        let source = format!("{}\n{}", include_str!("shaders/globals.wgsl"), spec.source);
        let module = self
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(spec.label),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });

        let mut attributes = Vec::new();
        let mut stride = 0;
        for (location, &format) in spec.vertex_layout.unwrap_or_default().iter().enumerate() {
            attributes.push(wgpu::VertexAttribute {
                format,
                offset: stride,
                shader_location: location as u32,
            });
            stride += format.size();
        }
        let buffers = [Some(wgpu::VertexBufferLayout {
            array_stride: stride,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &attributes,
        })];
        let buffers: &[Option<wgpu::VertexBufferLayout<'_>>] = if spec.vertex_layout.is_some() {
            &buffers
        } else {
            &[]
        };

        self.device
            .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(spec.label),
                layout: Some(self.layout),
                vertex: wgpu::VertexState {
                    module: &module,
                    entry_point: Some("vs_main"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    buffers,
                },
                primitive: wgpu::PrimitiveState {
                    topology: spec.topology,
                    ..wgpu::PrimitiveState::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(spec.depth_write),
                    depth_compare: Some(spec.depth_compare),
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: self.sample_count,
                    ..wgpu::MultisampleState::default()
                },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some("fs_main"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: OUTPUT_FORMAT,
                        blend: spec.blend,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn srgb_conversion_hits_known_values() {
        assert!(linear([0, 0, 0, 0]).iter().all(|c| c.abs() < f32::EPSILON));
        let mid = linear([188, 188, 188, 255]);
        assert!((mid[0] - 0.5).abs() < 0.01);
        assert!((mid[3] - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn grid_spacing_is_a_power_of_ten_that_scales_with_zoom() {
        let near = GridLayout::for_camera(&Camera {
            distance: 100.0,
            ..Camera::default()
        });
        let far = GridLayout::for_camera(&Camera {
            distance: 10_000.0,
            ..Camera::default()
        });
        assert!((near.major / near.minor - 10.0).abs() < 1e-12);
        assert!((far.minor / near.minor - 100.0).abs() < 1e-9);
        let exponent = near.minor.log10();
        assert!((exponent - exponent.round()).abs() < 1e-12);
    }
}
