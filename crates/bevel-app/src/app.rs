// SPDX-License-Identifier: GPL-3.0-or-later

use crate::part::{Part, RenderScene, Target};
use crate::platform::{self, SaveOutcome};
use crate::viewport::Viewport;
use bevel_render::{Camera, StandardView, ViewportOverlays};
use bevel_ui::{BodyEntry, ExportFormat, LogBuffer, UiAction, UiState};

/// Length of animated view changes, in seconds.
const TRANSITION_SECONDS: f64 = 0.4;

/// An animated move of the camera to a new view.
struct Transition {
    from: Camera,
    to: Camera,
    start: f64,
}

/// Which scene the viewport shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shown {
    /// Nothing uploaded yet.
    Nothing,
    /// The part's bodies, with the given visibility.
    Part,
    /// View > Geometry Debug.
    GeometryDebug,
}

/// Application state: UI state, the part, the camera and the viewport renderer.
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
    /// `None` if the demonstration part could not be built.
    part: Option<Part>,
    /// The scene currently uploaded, and the visibility it was built with.
    shown: Shown,
    shown_visibility: Vec<bool>,
    /// Pick table of the uploaded part scene.
    scene: Option<RenderScene>,
    /// Last pixel a pick was requested for, to avoid repeating it.
    last_pick_px: Option<[u32; 2]>,
    hovered: Option<Target>,
    /// The pixel the current `hovered` value was read at.
    hovered_px: Option<[u32; 2]>,
    /// A click waiting for the pick result at its pixel, with Shift.
    pending_click: Option<([u32; 2], bool)>,
    selected: Vec<Target>,
}

impl BevelApp {
    /// Creates the app; `can_quit` is false on the web, where there is no window to close.
    pub fn new(cc: &eframe::CreationContext<'_>, log: LogBuffer, can_quit: bool) -> Self {
        bevel_ui::install(&cc.egui_ctx);
        let viewport = cc.wgpu_render_state.as_ref().map(Viewport::new);
        if viewport.is_none() {
            tracing::error!("no wgpu render state; the 3D viewport is disabled");
        }
        let mut ui = UiState::default();
        ui.can_quit = can_quit;
        let part = match Part::demo() {
            Ok(part) => Some(part),
            Err(err) => {
                tracing::error!(%err, "could not build the demonstration part");
                ui.notify(format!("Could not build the bodies: {err}"));
                None
            }
        };
        if let Some(part) = &part {
            ui.bodies = part
                .bodies
                .iter()
                .map(|b| BodyEntry {
                    name: b.name.clone(),
                    visible: true,
                })
                .collect();
        }
        tracing::info!(version = env!("CARGO_PKG_VERSION"), "Bevel started");
        Self {
            ui,
            camera: Camera::default(),
            transition: None,
            viewport,
            log,
            aspect: 1.0,
            fit_pending: true,
            part,
            shown: Shown::Nothing,
            shown_visibility: Vec::new(),
            scene: None,
            last_pick_px: None,
            hovered: None,
            hovered_px: None,
            pending_click: None,
            selected: Vec::new(),
        }
    }

    fn visibility(&self) -> Vec<bool> {
        self.ui.bodies.iter().map(|b| b.visible).collect()
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
            UiAction::Export(format) => self.export(format),
            UiAction::SelectBody(index) => {
                if let Some(part) = &self.part {
                    self.selected = part.body_faces(index);
                    self.update_selection_info();
                }
            }
        }
    }

    fn export(&mut self, format: ExportFormat) {
        let Some(part) = &self.part else {
            return;
        };
        let visible = self.visibility();
        let count = visible.iter().filter(|v| **v).count();
        let result = part.export(&visible, format).and_then(|bytes| {
            let name = format!("bevel.{}", format.extension());
            let description = format!("{} mesh", format.label());
            platform::save_file(&name, format.extension(), &description, &bytes)
        });
        match result {
            Ok(SaveOutcome::Saved(place)) => {
                tracing::info!(%place, count, "exported {}", format.label());
                self.ui.notify(format!(
                    "Exported {count} bodies as {} to {place}",
                    format.label()
                ));
            }
            Ok(SaveOutcome::Cancelled) => {}
            Err(err) => {
                tracing::error!(%err, "export failed");
                self.ui.notify(format!("Export failed: {err}"));
            }
        }
    }

    fn update_selection_info(&mut self) {
        self.ui.selection = self
            .part
            .as_ref()
            .and_then(|part| part.selection_info(&self.selected));
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

    /// Uploads the scene the UI asks for, if it differs from what is shown.
    fn sync_scene(&mut self, device: &eframe::egui_wgpu::wgpu::Device) {
        let Some(viewport) = self.viewport.as_mut() else {
            return;
        };
        let visibility: Vec<bool> = self.ui.bodies.iter().map(|b| b.visible).collect();
        let wanted = if self.ui.show_geometry_debug {
            Shown::GeometryDebug
        } else {
            Shown::Part
        };
        if wanted == self.shown && (wanted != Shown::Part || visibility == self.shown_visibility) {
            return;
        }
        let first = self.shown == Shown::Nothing || wanted != self.shown;
        match wanted {
            Shown::GeometryDebug => match crate::geometry_debug::scene() {
                Ok((meshes, lines)) => {
                    viewport.set_scene(device, &meshes, &lines);
                    self.scene = None;
                }
                Err(err) => {
                    tracing::error!(%err, "geometry debug scene failed");
                    self.ui.show_geometry_debug = false;
                    self.ui
                        .notify(format!("The geometry debug scene failed: {err}"));
                    return;
                }
            },
            Shown::Part | Shown::Nothing => {
                let scene = self
                    .part
                    .as_ref()
                    .map(|part| part.render_scene(&visibility));
                if let Some(scene) = &scene {
                    viewport.set_scene(device, &scene.meshes, &scene.lines);
                }
                self.scene = scene;
                // Drop selections of bodies that are no longer shown.
                let before = self.selected.len();
                self.selected
                    .retain(|t| visibility.get(t.body()).copied().unwrap_or(false));
                if self.selected.len() != before {
                    self.update_selection_info();
                }
            }
        }
        self.shown = wanted;
        self.shown_visibility = visibility;
        self.hovered = None;
        self.hovered_px = None;
        self.pending_click = None;
        self.last_pick_px = None;
        self.fit_pending |= first;
    }

    /// Hover, click-to-select and highlighting for the part scene.
    ///
    /// Picks are read back asynchronously, so a click made right after the
    /// pointer moved waits for the pick result at the click's own pixel
    /// rather than using whatever was hovered a frame earlier.
    fn pick(&mut self, area: &bevel_ui::viewport::ViewportArea, escape: bool) {
        let Some(viewport) = self.viewport.as_mut() else {
            return;
        };
        let Some(scene) = &self.scene else {
            viewport.set_highlight(None, &[]);
            return;
        };
        let mut changed = false;
        if let Some(result) = viewport.poll_pick() {
            self.hovered = result.id.and_then(|id| scene.target(id));
            self.hovered_px = Some(result.pixel);
            if let Some((px, shift)) = self.pending_click
                && px == result.pixel
            {
                self.pending_click = None;
                changed |= select(&mut self.selected, self.hovered, shift);
            }
        }
        match area.hover_px {
            Some(px) if self.last_pick_px != Some(px) => {
                viewport.request_pick(px);
                self.last_pick_px = Some(px);
            }
            Some(_) => {}
            None => {
                self.hovered = None;
                self.hovered_px = None;
                self.last_pick_px = None;
            }
        }
        if area.clicked {
            match area.hover_px {
                Some(px) if self.hovered_px == Some(px) => {
                    changed |= select(&mut self.selected, self.hovered, area.shift);
                }
                Some(px) => self.pending_click = Some((px, area.shift)),
                None => changed |= select(&mut self.selected, None, area.shift),
            }
        }
        if escape && !self.selected.is_empty() {
            self.selected.clear();
            self.pending_click = None;
            changed = true;
        }
        let hovered = self.hovered.and_then(|t| scene.id(t));
        let selected: Vec<u32> = self.selected.iter().filter_map(|&t| scene.id(t)).collect();
        viewport.set_highlight(hovered, &selected);
        if changed {
            self.update_selection_info();
        }
    }
}

/// Applies a click on `target` (or on empty space) to `selection`: a plain
/// click selects only the target or clears the selection; Shift toggles the
/// target and ignores empty space. Returns whether the selection changed.
fn select(selection: &mut Vec<Target>, target: Option<Target>, shift: bool) -> bool {
    match (target, shift) {
        (Some(target), true) => {
            if let Some(i) = selection.iter().position(|&t| t == target) {
                selection.remove(i);
            } else {
                selection.push(target);
            }
            true
        }
        (Some(target), false) => {
            let changed = *selection != [target];
            *selection = vec![target];
            changed
        }
        (None, false) => {
            let changed = !selection.is_empty();
            selection.clear();
            changed
        }
        (None, true) => false,
    }
}

impl eframe::App for BevelApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.step_transition(ui.ctx());
        if let Some(render_state) = frame.wgpu_render_state() {
            self.sync_scene(&render_state.device);
        }
        let before = self.camera.clone();
        let escape = ui.input(|i| i.key_pressed(egui::Key::Escape));

        let render_state = frame.wgpu_render_state();
        let camera = &mut self.camera;
        let viewport = &mut self.viewport;
        let aspect = &mut self.aspect;
        let mut viewport_area = None;
        let actions = bevel_ui::workbench(ui, &mut self.ui, &self.log, |ui, state, actions| {
            let area = bevel_ui::viewport::interact(ui, camera);
            *aspect = area.aspect();
            let texture = match (viewport.as_mut(), render_state) {
                (Some(viewport), Some(render_state)) => {
                    viewport.set_overlays(ViewportOverlays {
                        grid: state.show_grid,
                        axes: state.show_origin,
                    });
                    Some(viewport.render(render_state, area.size_px, camera))
                }
                _ => None,
            };
            bevel_ui::viewport::paint(ui, &area, texture);
            bevel_ui::viewport::overlays(ui, &area, camera, state, actions);
            viewport_area = Some(area);
        });
        if let Some(area) = viewport_area {
            self.pick(&area, escape);
            // Keep frames coming while a pick result is on its way.
            if area.hover_px.is_some() {
                ui.ctx().request_repaint();
            }
        }

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

#[cfg(test)]
mod tests {
    use super::*;
    use bevel_kernel::FaceKey;

    fn face(n: u64) -> Target {
        let mut keys = slotmap::SlotMap::<FaceKey, ()>::with_key();
        let mut key = keys.insert(());
        for _ in 0..n {
            key = keys.insert(());
        }
        Target::Face { body: 0, face: key }
    }

    #[test]
    fn plain_click_replaces_and_empty_click_clears() {
        let mut selection = vec![face(0)];
        assert!(select(&mut selection, Some(face(1)), false));
        assert_eq!(selection, vec![face(1)]);
        assert!(!select(&mut selection, Some(face(1)), false));
        assert!(select(&mut selection, None, false));
        assert!(selection.is_empty());
    }

    #[test]
    fn shift_click_toggles_and_ignores_empty_space() {
        let mut selection = vec![face(0)];
        assert!(select(&mut selection, Some(face(1)), true));
        assert_eq!(selection, vec![face(0), face(1)]);
        assert!(select(&mut selection, Some(face(0)), true));
        assert_eq!(selection, vec![face(1)]);
        assert!(!select(&mut selection, None, true));
        assert_eq!(selection, vec![face(1)]);
    }
}
