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

/// The pipelines of the main (colour) pass.
pub(crate) struct Pipelines {
    pub(crate) background: wgpu::RenderPipeline,
    pub(crate) mesh: wgpu::RenderPipeline,
    /// Thick lines: one instance per segment, six vertices per instance.
    pub(crate) lines: wgpu::RenderPipeline,
    pub(crate) grid: wgpu::RenderPipeline,
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
