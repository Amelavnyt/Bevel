// SPDX-License-Identifier: GPL-3.0-or-later

use crate::viewport::Viewport;
use bevel_render::Camera;
use bevel_ui::{LogBuffer, UiAction, UiState};

/// Application state: UI state, camera and the viewport renderer.
pub struct BevelApp {
    ui: UiState,
    camera: Camera,
    /// `None` when eframe runs without a wgpu render state.
    viewport: Option<Viewport>,
    log: LogBuffer,
    /// Aspect ratio of the viewport last frame, used by actions such as Fit All.
    aspect: f64,
    /// Fit the scene once the viewport's size is known.
    fit_pending: bool,
}

impl BevelApp {
    /// Creates the app; `can_quit` is false on the web, where there is no window to close.
    pub fn new(cc: &eframe::CreationContext<'_>, log: LogBuffer, can_quit: bool) -> Self {
        cc.egui_ctx.set_theme(egui::Theme::Dark);
        let viewport = cc.wgpu_render_state.as_ref().map(Viewport::new);
        if viewport.is_none() {
            tracing::error!("no wgpu render state; the 3D viewport is disabled");
        }
        tracing::info!(version = env!("CARGO_PKG_VERSION"), "Bevel started");
        Self {
            ui: UiState {
                can_quit,
                ..UiState::default()
            },
            camera: Camera::default(),
            viewport,
            log,
            aspect: 1.0,
            fit_pending: true,
        }
    }

    fn apply(&mut self, action: UiAction, ctx: &egui::Context) {
        match action {
            UiAction::FitAll => self.fit_all(),
            UiAction::SetProjection(projection) => {
                self.camera.projection = projection;
                self.ui.projection = projection;
            }
            UiAction::Quit => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
        }
    }

    fn fit_all(&mut self) {
        if let Some(viewport) = &self.viewport {
            self.camera.fit(&viewport.scene_bounds(), self.aspect);
        }
    }
}

impl eframe::App for BevelApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let render_state = frame.wgpu_render_state();
        let camera = &mut self.camera;
        let viewport = &mut self.viewport;
        let aspect = &mut self.aspect;
        let actions = bevel_ui::workbench(ui, &mut self.ui, &self.log, |ui| {
            let area = bevel_ui::viewport::interact(ui, camera);
            *aspect = area.aspect();
            let texture = match (viewport.as_mut(), render_state) {
                (Some(viewport), Some(render_state)) => {
                    Some(viewport.render(render_state, area.size_px, camera))
                }
                _ => None,
            };
            bevel_ui::viewport::paint(ui, &area, texture);
        });

        if std::mem::take(&mut self.fit_pending) {
            self.fit_all();
            ui.ctx().request_repaint();
        }
        for action in actions {
            self.apply(action, ui.ctx());
        }
    }
}
