// SPDX-License-Identifier: GPL-3.0-or-later

//! The render pipelines of the viewport passes.

use crate::pack::{self, LINE_INSTANCE_FORMATS, MESH_VERTEX_FORMATS};

/// Colour format of the texture the renderer resolves into.
///
/// The UI composites in gamma space and samples textures as stored, so the
/// target holds sRGB-encoded values in a plain UNORM format: the shaders
/// encode their linear results with `to_srgb` (shaders/globals.wgsl).
pub(crate) const OUTPUT_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
pub(crate) const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Format of the id buffer the picking pass renders into.
pub(crate) const ID_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Uint;

/// The pipelines of the main (colour) pass and of the id pass.
pub(crate) struct Pipelines {
    pub(crate) background: wgpu::RenderPipeline,
    pub(crate) mesh: wgpu::RenderPipeline,
    /// Thick lines: one instance per segment, six vertices per instance.
    pub(crate) lines: wgpu::RenderPipeline,
    pub(crate) grid: wgpu::RenderPipeline,
    /// Meshes into the id buffer.
    pub(crate) id_mesh: wgpu::RenderPipeline,
    /// Wider lines into the id buffer.
    pub(crate) id_lines: wgpu::RenderPipeline,
}

impl Pipelines {
    pub(crate) fn new(
        device: &wgpu::Device,
        layout: &wgpu::PipelineLayout,
        sample_count: u32,
    ) -> Self {
        let builder = PipelineBuilder {
            device,
            layout,
            sample_count,
            target_format: OUTPUT_FORMAT,
        };
        // The id buffer is never multisampled: ids can't be averaged.
        let id_builder = PipelineBuilder {
            device,
            layout,
            sample_count: 1,
            target_format: ID_FORMAT,
        };
        let background = builder.build(&PipelineSpec {
            label: "viewport background",
            source: include_str!("shaders/background.wgsl"),
            entry_points: MAIN_ENTRY_POINTS,
            vertex_layout: None,
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: false,
            depth_compare: wgpu::CompareFunction::Always,
            blend: None,
        });
        let mesh = builder.build(&PipelineSpec {
            label: "viewport meshes",
            source: include_str!("shaders/mesh.wgsl"),
            entry_points: MAIN_ENTRY_POINTS,
            vertex_layout: Some(VertexLayout {
                formats: &MESH_VERTEX_FORMATS,
                step_mode: wgpu::VertexStepMode::Vertex,
            }),
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::Greater,
            blend: None,
        });
        let lines = builder.build(&PipelineSpec {
            label: "viewport lines",
            source: include_str!("shaders/lines.wgsl"),
            entry_points: MAIN_ENTRY_POINTS,
            vertex_layout: Some(VertexLayout {
                formats: &LINE_INSTANCE_FORMATS,
                step_mode: wgpu::VertexStepMode::Instance,
            }),
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::GreaterEqual,
            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
        });
        let grid = builder.build(&PipelineSpec {
            label: "viewport grid",
            source: include_str!("shaders/grid.wgsl"),
            entry_points: MAIN_ENTRY_POINTS,
            vertex_layout: None,
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: false,
            depth_compare: wgpu::CompareFunction::GreaterEqual,
            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
        });
        let id_mesh = id_builder.build(&PipelineSpec {
            label: "viewport id meshes",
            source: include_str!("shaders/mesh.wgsl"),
            entry_points: EntryPoints {
                vertex: "vs_main",
                fragment: "fs_id",
            },
            vertex_layout: Some(VertexLayout {
                formats: &MESH_VERTEX_FORMATS,
                step_mode: wgpu::VertexStepMode::Vertex,
            }),
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::Greater,
            blend: None,
        });
        let id_lines = id_builder.build(&PipelineSpec {
            label: "viewport id lines",
            source: include_str!("shaders/lines.wgsl"),
            entry_points: EntryPoints {
                vertex: "vs_id",
                fragment: "fs_id",
            },
            vertex_layout: Some(VertexLayout {
                formats: &LINE_INSTANCE_FORMATS,
                step_mode: wgpu::VertexStepMode::Instance,
            }),
            topology: wgpu::PrimitiveTopology::TriangleList,
            depth_write: true,
            depth_compare: wgpu::CompareFunction::GreaterEqual,
            blend: None,
        });
        Self {
            background,
            mesh,
            lines,
            grid,
            id_mesh,
            id_lines,
        }
    }
}

struct PipelineBuilder<'a> {
    device: &'a wgpu::Device,
    layout: &'a wgpu::PipelineLayout,
    sample_count: u32,
    /// Format of the single colour target.
    target_format: wgpu::TextureFormat,
}

/// The names of the shader's vertex and fragment entry points.
#[derive(Clone, Copy)]
struct EntryPoints {
    vertex: &'static str,
    fragment: &'static str,
}

const MAIN_ENTRY_POINTS: EntryPoints = EntryPoints {
    vertex: "vs_main",
    fragment: "fs_main",
};

/// The attributes of the single vertex buffer of a pipeline.
#[derive(Clone, Copy)]
struct VertexLayout<'a> {
    /// Attribute formats in shader location order, tightly packed.
    formats: &'a [wgpu::VertexFormat],
    /// Whether the buffer advances per vertex or per instance.
    step_mode: wgpu::VertexStepMode,
}

struct PipelineSpec<'a> {
    label: &'a str,
    source: &'a str,
    entry_points: EntryPoints,
    /// The vertex buffer layout, `None` for passes that generate their
    /// vertices from the vertex index.
    vertex_layout: Option<VertexLayout<'a>>,
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
        for (location, &format) in spec
            .vertex_layout
            .map(|l| l.formats)
            .unwrap_or_default()
            .iter()
            .enumerate()
        {
            attributes.push(wgpu::VertexAttribute {
                format,
                offset: stride,
                shader_location: location as u32,
            });
            stride += format.size();
        }
        debug_assert_eq!(
            stride,
            spec.vertex_layout.map_or(0, |l| pack::stride_of(l.formats))
        );
        let buffers = [Some(wgpu::VertexBufferLayout {
            array_stride: stride,
            step_mode: spec
                .vertex_layout
                .map_or(wgpu::VertexStepMode::Vertex, |l| l.step_mode),
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
                    entry_point: Some(spec.entry_points.vertex),
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
                    entry_point: Some(spec.entry_points.fragment),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: self.target_format,
                        blend: spec.blend,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
    }
}
