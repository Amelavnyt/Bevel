// SPDX-License-Identifier: GPL-3.0-or-later

//! egui panels, widgets, tools and timeline widgets for Bevel.
//!
//! Widgets never touch the document. They read state and emit
//! [`UiAction`]s, which the application turns into commands or view
//! changes. [`workbench`] lays out the whole window:
//!
//! ```text
//! app bar:  logo  menus  new open save  undo redo   <title>   branch  log
//! toolbar:  CREATE | MODIFY | CONSTRUCT | INSPECT | INSERT | UTILITIES
//! browser |          3D viewport (view cube, nav bar)         | properties
//! log (optional)
//! timeline
//! ```
//!
//! There is a single design workspace: sketching, construction geometry and
//! inspection are tools in it, as in other parametric CAD tools. While a
//! sketch is open, the sketch tools take over the toolbar.
//!
//! Status: the layout and viewport controls exist; panel contents and tools
//! arrive with the features they use (see the roadmap).

mod appbar;
mod browser;
pub mod icons;
mod log;
mod properties;
pub mod theme;
mod timeline;
mod toolbar;
pub mod tools;
mod viewcube;
pub mod viewport;
mod widgets;

pub use log::{LogBuffer, LogLayer, LogLine};

use bevel_render::{Projection, StandardView};

/// How long notices stay on screen, in seconds.
const NOTICE_SECONDS: f64 = 4.0;

/// Something the user asked for that the application has to carry out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UiAction {
    /// Frame everything in the viewport.
    FitAll,
    /// Turn the camera to a standard view.
    SetView(StandardView),
    /// Switch between perspective and orthographic projection.
    SetProjection(Projection),
    /// Quit the application (desktop only).
    Quit,
    /// Export the visible bodies to a file.
    Export(ExportFormat),
    /// Select a body (by its index in [`UiState::bodies`]).
    SelectBody(usize),
}

/// A mesh file format for export.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportFormat {
    /// Binary STL.
    Stl,
    /// 3MF (3D Manufacturing Format).
    ThreeMf,
}

impl ExportFormat {
    /// File extension, without the dot.
    pub fn extension(self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::ThreeMf => "3mf",
        }
    }

    /// Name for menus and file dialogs.
    pub fn label(self) -> &'static str {
        match self {
            Self::Stl => "STL",
            Self::ThreeMf => "3MF",
        }
    }
}

/// A body as the browser shows it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BodyEntry {
    /// Display name.
    pub name: String,
    /// Whether it is drawn and exported.
    pub visible: bool,
}

/// What the properties panel shows for the current selection.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelectionInfo {
    /// Heading, e.g. the body or face name.
    pub title: String,
    /// Label and value rows, already formatted with units.
    pub rows: Vec<(String, String)>,
}

/// A short message shown over the viewport.
#[derive(Clone, Debug)]
pub struct Notice {
    /// The message.
    pub text: String,
    /// Time (egui input time, seconds) at which it disappears.
    pub until: f64,
}

/// UI-only state: what is shown, not what is modeled.
#[derive(Clone, Debug)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "independent show/hide toggles, not a state machine"
)]
pub struct UiState {
    /// Whether the log panel is open.
    pub show_log: bool,
    /// Whether the properties panel is open.
    pub show_properties: bool,
    /// Whether the About window is open.
    pub show_about: bool,
    /// Whether the viewport controls window is open.
    pub show_controls: bool,
    /// Whether the ground grid is drawn.
    pub show_grid: bool,
    /// Whether the origin axes are drawn.
    pub show_origin: bool,
    /// Whether the viewport shows the geometry debug scene (sample NURBS
    /// curves and surfaces) instead of the test cube.
    pub show_geometry_debug: bool,
    /// Current projection, mirrored from the camera.
    pub projection: Projection,
    /// Name of the checked-out branch.
    pub branch: String,
    /// Whether the platform can quit (false on the web).
    pub can_quit: bool,
    /// Message shown over the viewport, if any.
    pub notice: Option<Notice>,
    /// The part's bodies, in browser order; the app keeps this in sync.
    pub bodies: Vec<BodyEntry>,
    /// What is selected, for the properties panel; `None` when nothing is.
    pub selection: Option<SelectionInfo>,
    /// Current time, for timing notices; updated every frame.
    now: f64,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_log: false,
            show_properties: true,
            show_about: false,
            show_controls: false,
            show_grid: true,
            show_origin: true,
            show_geometry_debug: false,
            projection: Projection::Perspective,
            branch: "main".to_owned(),
            can_quit: true,
            notice: None,
            bodies: Vec::new(),
            selection: None,
            now: 0.0,
        }
    }
}

impl UiState {
    /// Shows a notice for a few seconds.
    pub fn notify(&mut self, text: impl Into<String>) {
        self.notice = Some(Notice {
            text: text.into(),
            until: self.now + NOTICE_SECONDS,
        });
    }

    /// Tells the user that `name` doesn't exist yet and when it arrives.
    fn notify_unavailable(&mut self, name: &str, phase: &str) {
        self.notify(format!(
            "{name} isn't available yet. It arrives in {phase}."
        ));
    }
}

/// Prepares `ctx` for the Bevel UI: installs the image loaders the icons
/// need and applies the theme. Call once at start-up.
pub fn install(ctx: &egui::Context) {
    egui_extras::install_image_loaders(ctx);
    theme::apply(ctx);
}

/// Lays out the whole window and calls `viewport` to fill the central area.
/// The viewport closure gets the state and the action list so its overlays
/// can use them.
///
/// Returns the actions the user triggered this frame.
pub fn workbench(
    ui: &mut egui::Ui,
    state: &mut UiState,
    log: &LogBuffer,
    viewport: impl FnOnce(&mut egui::Ui, &mut UiState, &mut Vec<UiAction>),
) -> Vec<UiAction> {
    let mut actions = Vec::new();
    state.now = ui.input(|i| i.time);

    let bar_frame = |margin: egui::Margin| {
        egui::Frame::new()
            .fill(theme::BAR)
            .inner_margin(margin)
            .stroke(egui::Stroke::new(1.0, theme::BORDER))
    };
    egui::Panel::top("app_bar")
        .frame(bar_frame(egui::Margin::symmetric(8, 4)))
        .exact_size(36.0)
        .show(ui, |ui| appbar::show(ui, state, &mut actions));
    egui::Panel::top("toolbar")
        .frame(bar_frame(egui::Margin::symmetric(8, 4)))
        .show(ui, |ui| toolbar::show(ui, state));
    egui::Panel::bottom("timeline")
        .frame(bar_frame(egui::Margin::symmetric(8, 5)))
        .resizable(false)
        .exact_size(40.0)
        .show(ui, timeline::show);
    if state.show_log {
        egui::Panel::bottom("log")
            .resizable(true)
            .default_size(160.0)
            .show(ui, |ui| log::show(ui, log));
    }
    let side_frame = egui::Frame::new()
        .fill(theme::PANEL)
        .inner_margin(egui::Margin::symmetric(8, 6));
    egui::Panel::left("browser")
        .frame(side_frame)
        .resizable(true)
        .default_size(230.0)
        .show(ui, |ui| browser::show(ui, state, &mut actions));
    if state.show_properties {
        egui::Panel::right("properties")
            .frame(side_frame)
            .resizable(true)
            .default_size(300.0)
            .size_range(260.0..=440.0)
            .show(ui, |ui| properties::show(ui, state.selection.as_ref()));
    }
    egui::CentralPanel::no_frame().show(ui, |ui| viewport(ui, state, &mut actions));

    if state.show_about {
        about_window(ui.ctx(), &mut state.show_about);
    }
    if state.show_controls {
        controls_window(ui.ctx(), &mut state.show_controls);
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
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.add(icons::Icon::Logo.image(48.0));
                ui.vertical(|ui| {
                    ui.heading("Bevel");
                    ui.label(concat!("Version ", env!("CARGO_PKG_VERSION")));
                });
            });
            ui.add_space(6.0);
            ui.label("Open-source parametric 3D CAD.");
            ui.label("Licensed under the GNU GPL, version 3 or later.");
        });
}

fn controls_window(ctx: &egui::Context, open: &mut bool) {
    egui::Window::new("Viewport Controls")
        .open(open)
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            egui::Grid::new("controls").striped(true).show(ui, |ui| {
                for (action, how) in [
                    ("Orbit", "Right drag, or Shift + middle drag"),
                    ("Pan", "Middle drag, or Shift + right drag"),
                    ("Zoom", "Scroll wheel or pinch, towards the cursor"),
                    ("Fit all", "Home"),
                    ("Standard views", "Click a face of the view cube"),
                ] {
                    ui.strong(action);
                    ui.label(how);
                    ui.end_row();
                }
            });
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable as _;

    fn harness(state: &mut UiState) -> Harness<'_> {
        let log = LogBuffer::default();
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1400.0, 900.0))
            .build_ui(move |ui| {
                install(ui.ctx());
                workbench(ui, state, &log, |ui, _, _| {
                    ui.label("viewport goes here");
                });
            });
        harness.run();
        harness
    }

    #[test]
    fn workbench_shows_every_region() {
        let mut state = UiState::default();
        let harness = harness(&mut state);
        for label in [
            "BROWSER",
            "PROPERTIES",
            "Untitled",
            "Origin",
            "Create Sketch",
            "Measure",
            "Section Analysis",
            "viewport goes here",
        ] {
            harness.get_by_label(label);
        }
    }

    #[test]
    fn sketching_and_inspection_are_tools_not_workspaces() {
        let mut state = UiState::default();
        let harness = harness(&mut state);
        for tab in ["SOLID", "SKETCH", "INSPECT ", "SCRIPTS"] {
            assert!(
                harness.query_by_label(tab).is_none(),
                "unexpected tab {tab}"
            );
        }
    }

    #[test]
    fn unavailable_tool_explains_when_it_arrives() {
        let mut state = UiState::default();
        {
            let mut harness = harness(&mut state);
            harness.get_by_label("Extrude").click();
            harness.run();
        }
        let notice = state.notice.expect("clicking a tool shows a notice");
        assert!(notice.text.contains("Extrude"));
        assert!(notice.text.contains("Phase 6"));
    }

    fn with_bodies() -> UiState {
        UiState {
            bodies: vec![
                BodyEntry {
                    name: "Box".to_owned(),
                    visible: true,
                },
                BodyEntry {
                    name: "Torus".to_owned(),
                    visible: false,
                },
            ],
            ..UiState::default()
        }
    }

    #[test]
    fn clicking_a_body_selects_it() {
        let mut state = with_bodies();
        let mut actions = Vec::new();
        {
            let log = LogBuffer::default();
            let mut harness = Harness::builder()
                .with_size(egui::vec2(1400.0, 900.0))
                .build_ui(|ui| {
                    install(ui.ctx());
                    actions.extend(workbench(ui, &mut state, &log, |_, _, _| {}));
                });
            harness.run();
            harness.get_by_label("Torus").click();
            harness.run();
        }
        assert!(actions.contains(&UiAction::SelectBody(1)));
    }

    #[test]
    fn properties_show_the_selection() {
        let mut state = with_bodies();
        state.selection = Some(SelectionInfo {
            title: "Box".to_owned(),
            rows: vec![("Volume".to_owned(), "8000 mm³".to_owned())],
        });
        let harness = harness(&mut state);
        harness.get_by_label("Volume");
        harness.get_by_label("8000 mm³");
        assert!(harness.query_by_label("Nothing selected").is_none());
    }
}
