// SPDX-License-Identifier: GPL-3.0-or-later

//! Glue between the renderer and egui: renders the scene into a texture
//! and keeps that texture registered with egui's wgpu renderer.

use bevel_math::Aabb3;
use bevel_render::{Camera, ViewportOverlays, ViewportRenderer};
use eframe::egui_wgpu::{self, wgpu};

pub struct Viewport {
    renderer: ViewportRenderer,
    texture: Option<egui::TextureId>,
}

impl Viewport {
    pub fn new(render_state: &egui_wgpu::RenderState) -> Self {
        let mut renderer = ViewportRenderer::new(&render_state.device, &render_state.adapter);
        let (meshes, lines) = crate::scene::test_scene();
        renderer.set_scene(&render_state.device, &meshes, &lines);
        Self {
            renderer,
            texture: None,
        }
    }

    pub fn set_overlays(&mut self, overlays: ViewportOverlays) {
        self.renderer.set_overlays(overlays);
    }

    pub fn scene_bounds(&self) -> Aabb3 {
        self.renderer.scene_bounds()
    }

    /// Renders a frame and returns the egui texture showing it.
    pub fn render(
        &mut self,
        render_state: &egui_wgpu::RenderState,
        size_px: [u32; 2],
        camera: &Camera,
    ) -> egui::TextureId {
        let output =
            self.renderer
                .render(&render_state.device, &render_state.queue, size_px, camera);
        let mut egui_renderer = render_state.renderer.write();
        match self.texture {
            Some(id) if !output.recreated => id,
            Some(id) => {
                egui_renderer.update_egui_texture_from_wgpu_texture(
                    &render_state.device,
                    output.view,
                    wgpu::FilterMode::Linear,
                    id,
                );
                id
            }
            None => {
                let id = egui_renderer.register_native_texture(
                    &render_state.device,
                    output.view,
                    wgpu::FilterMode::Linear,
                );
                self.texture = Some(id);
                id
            }
        }
    }
}
