// SPDX-License-Identifier: GPL-3.0-or-later

//! egui panels, widgets, tools and timeline widgets for Bevel.
//!
//! Widgets never touch the document. They read state and emit
//! [`UiAction`]s, which the application turns into commands or view
//! changes. [`workbench`] lays out the whole window:
//!
//! ```text
//! menu bar | workspace tabs | tools                   | branch
//! browser  |              3D viewport                 | properties
//! log (optional)
//! timeline
//! ```
//!
//! Status: the layout exists with empty panels; panel contents arrive with
//! the features they show (see the roadmap).

mod browser;
mod log;
mod menu;
mod properties;
mod timeline;
mod toolbar;
pub mod viewport;

pub use log::{LogBuffer, LogLayer, LogLine};

use bevel_render::Projection;

/// A group of tools shown in the toolbar, like the workspaces of other CAD tools.
///
/// Sketching is part of solid modeling, not a workspace of its own: Create
/// Sketch lives in the SOLID tools, and while a sketch is open the sketch
/// tools replace the toolbar's contents until the sketch is finished.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Workspace {
    /// Solid modeling: sketches, primitives and features.
    #[default]
    Solid,
    /// Measurement and analysis.
    Inspect,
    /// Macros and script features.
    Scripts,
}

impl Workspace {
    /// All workspaces, in toolbar order.
    pub const ALL: [Self; 3] = [Self::Solid, Self::Inspect, Self::Scripts];

    /// Label shown on the workspace tab.
    pub fn label(self) -> &'static str {
        match self {
            Self::Solid => "SOLID",
            Self::Inspect => "INSPECT",
            Self::Scripts => "SCRIPTS",
        }
    }
}

/// Something the user asked for that the application has to carry out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UiAction {
    /// Frame everything in the viewport.
    FitAll,
    /// Switch between perspective and orthographic projection.
    SetProjection(Projection),
    /// Quit the application (desktop only).
    Quit,
}

/// UI-only state: what is shown, not what is modeled.
#[derive(Clone, Debug)]
pub struct UiState {
    /// Active toolbar workspace.
    pub workspace: Workspace,
    /// Whether the log panel is open.
    pub show_log: bool,
    /// Whether the About window is open.
    pub show_about: bool,
    /// Current projection, mirrored from the camera for the View menu.
    pub projection: Projection,
    /// Name of the checked-out branch.
    pub branch: String,
    /// Whether the platform can quit (false on the web).
    pub can_quit: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            workspace: Workspace::default(),
            show_log: false,
            show_about: false,
            projection: Projection::Perspective,
            branch: "main".to_owned(),
            can_quit: true,
        }
    }
}

/// Lays out the whole window and calls `viewport` to fill the central area.
///
/// Returns the actions the user triggered this frame.
pub fn workbench(
    ui: &mut egui::Ui,
    state: &mut UiState,
    log: &LogBuffer,
    viewport: impl FnOnce(&mut egui::Ui),
) -> Vec<UiAction> {
    let mut actions = Vec::new();

    egui::Panel::top("menu_bar").show(ui, |ui| menu::show(ui, state, &mut actions));
    egui::Panel::top("toolbar").show(ui, |ui| toolbar::show(ui, state));
    egui::Panel::bottom("timeline")
        .resizable(false)
        .show(ui, timeline::show);
    if state.show_log {
        egui::Panel::bottom("log")
            .resizable(true)
            .default_size(160.0)
            .show(ui, |ui| log::show(ui, log));
    }
    egui::Panel::left("browser")
        .resizable(true)
        .default_size(220.0)
        .show(ui, browser::show);
    egui::Panel::right("properties")
        .resizable(true)
        .default_size(260.0)
        .show(ui, properties::show);
    egui::CentralPanel::no_frame().show(ui, viewport);

    if state.show_about {
        about_window(ui.ctx(), &mut state.show_about);
    }
    if ui.input(|i| i.key_pressed(egui::Key::Home)) {
        actions.push(UiAction::FitAll);
    }
    actions
}

fn about_window(ctx: &egui::Context, open: &mut bool) {
    egui::Window::new("About Bevel")
        .open(open)
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.heading("Bevel");
            ui.label(concat!("Version ", env!("CARGO_PKG_VERSION")));
            ui.label("Open-source parametric 3D CAD.");
            ui.label("Licensed under the GNU GPL, version 3 or later.");
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable as _;

    #[test]
    fn workbench_shows_every_panel() {
        let log = LogBuffer::default();
        let mut state = UiState::default();
        let mut harness = Harness::new_ui(|ui| {
            workbench(ui, &mut state, &log, |ui| {
                ui.label("viewport goes here");
            });
        });
        harness.run();
        for label in [
            "Browser",
            "Properties",
            "SOLID",
            "SCRIPTS",
            "viewport goes here",
        ] {
            harness.get_by_label(label);
        }
    }
}
