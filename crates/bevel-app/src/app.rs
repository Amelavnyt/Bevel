// SPDX-License-Identifier: GPL-3.0-or-later

use crate::viewport::Viewport;
use bevel_render::{Camera, StandardView, ViewportOverlays};
use bevel_ui::{LogBuffer, UiAction, UiState};

/// Length of animated view changes, in seconds.
const TRANSITION_SECONDS: f64 = 0.4;

/// An animated move of the camera to a new view.
struct Transition {
    from: Camera,
    to: Camera,
    start: f64,
}

/// Application state: UI state, camera and the viewport renderer.
pub struct BevelApp {
    ui: UiState,
    camera: Camera,
    transition: Option<Transition>,
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
        bevel_ui::install(&cc.egui_ctx);
        let viewport = cc.wgpu_render_state.as_ref().map(Viewport::new);
        if viewport.is_none() {
            tracing::error!("no wgpu render state; the 3D viewport is disabled");
        }
        tracing::info!(version = env!("CARGO_PKG_VERSION"), "Bevel started");
        let mut ui = UiState::default();
        ui.can_quit = can_quit;
        Self {
            ui,
            camera: Camera::default(),
            transition: None,
            viewport,
            log,
            aspect: 1.0,
            fit_pending: true,
        }
    }

    fn apply(&mut self, action: UiAction, ctx: &egui::Context) {
        match action {
            UiAction::FitAll => {
                let target = self.fitted(self.camera.clone());
                self.animate_to(target, ctx);
            }
            UiAction::SetView(view) => {
                let mut target = self.camera.clone();
                target.set_view(view);
                if view == StandardView::Home {
                    target = self.fitted(target);
                }
                self.animate_to(target, ctx);
            }
            UiAction::SetProjection(projection) => {
                self.camera.projection = projection;
                self.ui.projection = projection;
            }
            UiAction::Quit => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
        }
    }

    /// `camera` moved so the whole scene is in view.
    fn fitted(&self, mut camera: Camera) -> Camera {
        if let Some(viewport) = &self.viewport {
            camera.fit(&viewport.scene_bounds(), self.aspect);
        }
        camera
    }

    fn animate_to(&mut self, to: Camera, ctx: &egui::Context) {
        self.transition = Some(Transition {
            from: self.camera.clone(),
            to,
            start: ctx.input(|i| i.time),
        });
        ctx.request_repaint();
    }

    /// Advances a running view transition. Any navigation input cancels it.
    fn step_transition(&mut self, ctx: &egui::Context) {
        let Some(transition) = &self.transition else {
            return;
        };
        let now = ctx.input(|i| i.time);
        let t = ((now - transition.start) / TRANSITION_SECONDS).clamp(0.0, 1.0);
        let eased = t * t * (3.0 - 2.0 * t);
        self.camera = Camera::interpolate(&transition.from, &transition.to, eased);
        if t >= 1.0 {
            self.transition = None;
        } else {
            ctx.request_repaint();
        }
    }
}

impl eframe::App for BevelApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.step_transition(ui.ctx());
        let before = self.camera.clone();

        let render_state = frame.wgpu_render_state();
        let camera = &mut self.camera;
        let viewport = &mut self.viewport;
        let aspect = &mut self.aspect;
        let fit_pending = &mut self.fit_pending;
        let actions = bevel_ui::workbench(ui, &mut self.ui, &self.log, |ui, state, actions| {
            let area = bevel_ui::viewport::interact(ui, camera);
            *aspect = area.aspect();
            let texture = match (viewport.as_mut(), render_state) {
                (Some(viewport), Some(render_state)) => {
                    viewport.set_overlays(ViewportOverlays {
                        grid: state.show_grid,
                        axes: state.show_origin,
                    });
                    match viewport
                        .show_geometry_debug(&render_state.device, state.show_geometry_debug)
                    {
                        Ok(changed) => *fit_pending |= changed,
                        Err(err) => {
                            tracing::error!(%err, "geometry debug scene failed");
                            state.show_geometry_debug = false;
                            state.notify(format!("The geometry debug scene failed: {err}"));
                        }
                    }
                    Some(viewport.render(render_state, area.size_px, camera))
                }
                _ => None,
            };
            bevel_ui::viewport::paint(ui, &area, texture);
            bevel_ui::viewport::overlays(ui, &area, camera, state, actions);
        });

        // The user took over the camera: stop animating it.
        if self.transition.is_some() && self.camera != before {
            self.transition = None;
        }
        if std::mem::take(&mut self.fit_pending) {
            self.camera = self.fitted(self.camera.clone());
            ui.ctx().request_repaint();
        }
        for action in actions {
            self.apply(action, ui.ctx());
        }
    }
}
