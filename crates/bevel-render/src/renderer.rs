// SPDX-License-Identifier: GPL-3.0-or-later

use crate::pack::{self, HIGHLIGHT_WORDS, LINE_INSTANCE_WORDS, MAX_SELECTED, globals};
use crate::pick::{PickResult, Picker};
use crate::pipelines::{DEPTH_FORMAT, ID_FORMAT, OUTPUT_FORMAT, Pipelines};
use crate::{Camera, LineSet, Projection, TriangleMesh};
use bevel_math::{Aabb3, Point3};
use wgpu::util::DeviceExt as _;

/// Largest render target edge we ask for; larger viewports are clamped.
const MAX_TARGET_SIZE: u32 = 8192;
/// Vertices drawn per line segment instance: two triangles.
const LINE_QUAD_VERTICES: u32 = 6;
/// Width of the Z axis, in pixels.
const AXIS_Z_WIDTH: f32 = 1.5;

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
    /// Tint of hovered faces and edges (the light signature teal). Faces mix
    /// it in at about 45%, edges take it at full strength.
    pub hover: [u8; 4],
    /// Tint of selected faces and edges (the signature teal). Faces mix it in
    /// at about 60%.
    pub selected: [u8; 4],
    /// Tint of faces and edges that are both selected and hovered, slightly
    /// brighter than [`ViewportTheme::selected`].
    pub selected_hover: [u8; 4],
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
        // #58AAA5, #2D8985 and a step brighter than the latter.
        hover: [88, 170, 165, 255],
        selected: [45, 137, 133, 255],
        selected_hover: [62, 158, 153, 255],
    };
}

/// Which helpers the viewport draws around the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ViewportOverlays {
    /// The ground grid in the XY plane.
    pub grid: bool,
    /// The X, Y and Z axes through the origin.
    pub axes: bool,
}

impl Default for ViewportOverlays {
    fn default() -> Self {
        Self {
            grid: true,
            axes: true,
        }
    }
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

/// The colour, or fully transparent when `visible` is false.
fn shown(color: [f32; 4], visible: bool) -> [f32; 4] {
    if visible {
        color
    } else {
        [color[0], color[1], color[2], 0.0]
    }
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
    /// The id buffer, created the first time a pick needs it.
    ids: Option<IdTargets>,
}

/// The targets of the id pass: a non-multisampled id texture with its own
/// depth buffer, the size of the viewport.
struct IdTargets {
    /// Kept for the texel copy into the readback buffer.
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    depth: wgpu::TextureView,
}

struct GpuMesh {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
}

/// Line segments as instances of the thick line pipeline.
struct GpuLines {
    instances: wgpu::Buffer,
    instance_count: u32,
}

/// Renders the 3D viewport into an offscreen texture.
pub struct ViewportRenderer {
    sample_count: u32,
    theme: ViewportTheme,
    overlays: ViewportOverlays,
    /// Hovered and selected ids, packed for the globals uniform.
    highlight: [u32; HIGHLIGHT_WORDS],
    globals: wgpu::Buffer,
    globals_bind_group: wgpu::BindGroup,
    pipelines: Pipelines,
    meshes: Vec<GpuMesh>,
    lines: Option<GpuLines>,
    /// Z axis segment (one line instance); its length follows the grid, so it
    /// is rewritten every frame.
    axis_lines: wgpu::Buffer,
    /// Contents of `axis_lines`, to rebuild it only when they change.
    axis_instance: [u32; LINE_INSTANCE_WORDS],
    scene_bounds: Aabb3,
    targets: Option<Targets>,
    picker: Picker,
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
            size: (globals::WORDS * 4) as u64,
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

        let axis_instance = [0u32; LINE_INSTANCE_WORDS];
        let axis_lines = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("viewport axis lines"),
            contents: &pack::words_to_bytes(&axis_instance),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            sample_count,
            theme: ViewportTheme::DARK,
            overlays: ViewportOverlays::default(),
            highlight: [0; HIGHLIGHT_WORDS],
            globals,
            globals_bind_group,
            pipelines,
            meshes: Vec::new(),
            lines: None,
            axis_lines,
            axis_instance,
            scene_bounds: Aabb3::EMPTY,
            targets: None,
            picker: Picker::new(device),
        }
    }

    /// Sets the viewport colours.
    pub fn set_theme(&mut self, theme: ViewportTheme) {
        self.theme = theme;
    }

    /// Chooses which helpers (grid, axes) are drawn.
    pub fn set_overlays(&mut self, overlays: ViewportOverlays) {
        self.overlays = overlays;
    }

    /// Sets which faces and edges are drawn highlighted: the one under the
    /// cursor (`hovered`) and the `selected` ones, by id.
    ///
    /// At most 64 selected ids are used; extra ids are ignored with a
    /// warning. Id 0 ("not pickable") is never highlighted. The state stays
    /// until the next call and survives [`ViewportRenderer::set_scene`].
    pub fn set_highlight(&mut self, hovered: Option<u32>, selected: &[u32]) {
        let (words, ignored) = pack::pack_highlight(hovered, selected);
        if ignored > 0 {
            tracing::warn!(
                ignored,
                "more than {MAX_SELECTED} selected ids; ignoring the extra ones"
            );
        }
        self.highlight = words;
    }

    /// Asks which id is under `pixel` (viewport pixels from the top left,
    /// in the size passed to [`ViewportRenderer::render`]).
    ///
    /// The id pass is rendered as part of the next `render` call, and
    /// [`ViewportRenderer::poll_pick`] returns the result a little later.
    /// Only one pick is in flight at a time: a newer request replaces an
    /// older one that has not been rendered yet, and a request made while a
    /// readback is still in flight waits until that result has been
    /// collected. A pixel outside the viewport reads the nearest edge pixel.
    pub fn request_pick(&mut self, pixel: [u32; 2]) {
        self.picker.request(pixel);
    }

    /// The result of the pick in flight, once it is ready. Never blocks;
    /// returns `None` until the GPU has finished, so call it every frame.
    ///
    /// On native targets this polls the device without waiting. On the web
    /// the browser completes the readback by itself.
    pub fn poll_pick(&mut self) -> Option<PickResult> {
        self.picker.poll()
    }

    /// Bounding box of everything set with [`ViewportRenderer::set_scene`].
    pub fn scene_bounds(&self) -> Aabb3 {
        self.scene_bounds
    }

    /// Replaces the drawn geometry.
    ///
    /// Meshes and line sets that can't be drawn are skipped with a warning:
    /// mismatched normal, id or position counts, indices out of range, or
    /// counts that don't fit 32-bit indices. Each mesh needs one id per
    /// vertex and each line set one id per segment.
    pub fn set_scene(&mut self, device: &wgpu::Device, meshes: &[TriangleMesh], lines: &[LineSet]) {
        let meshes: Vec<&TriangleMesh> = meshes
            .iter()
            .filter(|mesh| match mesh.check() {
                Ok(()) => true,
                Err(reason) => {
                    tracing::warn!("skipping mesh: {reason}");
                    false
                }
            })
            .collect();
        let lines: Vec<LineSet> = lines
            .iter()
            .filter(|set| match set.check() {
                Ok(()) => true,
                Err(reason) => {
                    tracing::warn!("skipping line set: {reason}");
                    false
                }
            })
            .cloned()
            .collect();

        self.scene_bounds = meshes
            .iter()
            .map(|mesh| mesh.bounds())
            .chain(lines.iter().map(LineSet::bounds))
            .fold(Aabb3::EMPTY, Aabb3::union);

        self.meshes = meshes
            .iter()
            .filter_map(|mesh| {
                let index_count = u32::try_from(mesh.indices.len()).ok()?;
                Some(GpuMesh {
                    vertices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("viewport mesh vertices"),
                        contents: &pack::words_to_bytes(&pack::pack_mesh_vertices(mesh)),
                        usage: wgpu::BufferUsages::VERTEX,
                    }),
                    indices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("viewport mesh indices"),
                        contents: &pack::words_to_bytes(&mesh.indices),
                        usage: wgpu::BufferUsages::INDEX,
                    }),
                    index_count,
                })
            })
            .collect();

        let (instances, count) = pack::pack_line_instances(&lines);
        self.lines = u32::try_from(count)
            .ok()
            .filter(|&n| n > 0)
            .map(|instance_count| GpuLines {
                instances: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("viewport line instances"),
                    contents: &pack::words_to_bytes(&instances),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
                instance_count,
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
        let (mut targets, recreated) = match self.targets.take() {
            Some(targets) if targets.size == size => (targets, false),
            _ => (self.create_targets(device, size), true),
        };
        // The id pass costs a full extra pass, so it only runs for a pick.
        let picking = self.picker.wants_pass();
        if picking && targets.ids.is_none() {
            targets.ids = Some(Self::create_id_targets(device, size));
        }

        let grid = GridLayout::for_camera(camera);
        queue.write_buffer(
            &self.globals,
            0,
            &pack::words_to_bytes(&self.globals_data(camera, size, &grid)),
        );
        let axis_z = linear(self.theme.axis_z);
        let z_top = grid.fade_radius as f32 * 0.5;
        let axis_instance =
            pack::pack_line_instance([0.0; 3], [0.0, 0.0, z_top], axis_z, 0, AXIS_Z_WIDTH);
        // Rebuild the (tiny) axis instance buffer when it changes instead of
        // writing into a buffer the same frame draws from as vertex input:
        // Vulkan synchronization validation flags that write as a hazard.
        if self.axis_instance != axis_instance {
            self.axis_lines = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("viewport axis lines"),
                contents: &pack::words_to_bytes(&axis_instance),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.axis_instance = axis_instance;
        }

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
                pass.set_vertex_buffer(0, lines.instances.slice(..));
                pass.draw(0..LINE_QUAD_VERTICES, 0..lines.instance_count);
            }
            if self.overlays.axes {
                pass.set_vertex_buffer(0, self.axis_lines.slice(..));
                pass.draw(0..LINE_QUAD_VERTICES, 0..1);
            }

            // Translucent grid last so it blends over what's already drawn.
            // The same pass draws the X and Y axes.
            if self.overlays.grid || self.overlays.axes {
                pass.set_pipeline(&self.pipelines.grid);
                pass.draw(0..6, 0..1);
            }
        }
        let mut copied = false;
        if let Some(ids) = targets.ids.as_ref().filter(|_| picking) {
            self.draw_ids(&mut encoder, ids);
            copied = self.picker.copy_pixel(&mut encoder, &ids.texture, size);
        }
        queue.submit([encoder.finish()]);
        if copied {
            self.picker.map();
        }

        let targets = self.targets.insert(targets);
        RenderOutput {
            view: &targets.output,
            recreated,
        }
    }

    /// Draws meshes (face ids) and lines (edge ids) into the id buffer.
    /// Edges are drawn wider than in the colour pass and pulled towards the
    /// camera the same way, so they win over the faces they bound.
    fn draw_ids(&self, encoder: &mut wgpu::CommandEncoder, ids: &IdTargets) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("viewport ids"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &ids.view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    // Id 0 means nothing pickable.
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &ids.depth,
                depth_ops: Some(wgpu::Operations {
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

        pass.set_pipeline(&self.pipelines.id_mesh);
        for mesh in &self.meshes {
            pass.set_vertex_buffer(0, mesh.vertices.slice(..));
            pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..mesh.index_count, 0, 0..1);
        }

        pass.set_pipeline(&self.pipelines.id_lines);
        if let Some(lines) = &self.lines {
            pass.set_vertex_buffer(0, lines.instances.slice(..));
            pass.draw(0..LINE_QUAD_VERTICES, 0..lines.instance_count);
        }
    }

    fn create_id_targets(device: &wgpu::Device, size: [u32; 2]) -> IdTargets {
        let extent = wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        };
        let create = |label, format, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: extent,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let texture = create(
            "viewport ids",
            ID_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = create(
            "viewport id depth",
            DEPTH_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        IdTargets {
            view: texture.create_view(&wgpu::TextureViewDescriptor::default()),
            depth: depth.create_view(&wgpu::TextureViewDescriptor::default()),
            texture,
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
            ids: None,
        }
    }

    fn globals_data(
        &self,
        camera: &Camera,
        size: [u32; 2],
        grid: &GridLayout,
    ) -> [u32; globals::WORDS] {
        let aspect = f64::from(size[0]) / f64::from(size[1]);
        let eye = camera.eye().as_vec3();
        let orthographic = if camera.projection == Projection::Orthographic {
            1.0
        } else {
            0.0
        };
        let vectors: [[f32; 4]; globals::VECTOR_COUNT] = [
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
            shown(linear(self.theme.grid_minor), self.overlays.grid),
            shown(linear(self.theme.grid_major), self.overlays.grid),
            shown(linear(self.theme.axis_x), self.overlays.axes),
            shown(linear(self.theme.axis_y), self.overlays.axes),
        ];
        let highlight_colors = [
            linear(self.theme.hover),
            linear(self.theme.selected),
            linear(self.theme.selected_hover),
        ];

        let mut data = [0u32; globals::WORDS];
        let mut put = |at: usize, values: &[f32]| {
            for (word, value) in data[at..].iter_mut().zip(values) {
                *word = value.to_bits();
            }
        };
        put(
            globals::VIEW_PROJ,
            &camera.view_projection(aspect).as_mat4().to_cols_array(),
        );
        put(globals::VIEW, &camera.view().as_mat4().to_cols_array());
        for (i, v) in vectors.iter().enumerate() {
            put(globals::VECTORS + 4 * i, v);
        }
        for (i, v) in highlight_colors.iter().enumerate() {
            put(globals::HIGHLIGHT_COLORS + 4 * i, v);
        }
        data[globals::HIGHLIGHT..].copy_from_slice(&self.highlight);
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
