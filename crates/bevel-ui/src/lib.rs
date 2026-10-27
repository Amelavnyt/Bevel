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
//! browser |   3D viewport (view cube, nav bar, floating tool dialogs)
//! log (optional)
//! timeline
//! ```
//!
//! There is a single design workspace: sketching, construction geometry and
//! inspection are tools in it, as in other parametric CAD tools. While a
//! sketch is open, the sketch tools take over the toolbar.

mod appbar;
mod browser;
mod feature_dialog;
pub mod icons;
mod log;
mod measure;
mod parameters;
pub mod theme;
mod timeline;
mod toolbar;
pub mod tools;
mod viewcube;
pub mod viewport;
mod widgets;

pub use log::{LogBuffer, LogLayer, LogLine};

pub use bevel_doc::Command;
pub use bevel_model::{BodyId, FeatureId, OriginPlane, PrimitiveKind};
use bevel_render::{Projection, StandardView};

/// How long notices stay on screen, in seconds.
const NOTICE_SECONDS: f64 = 4.0;

/// Something the user asked for that the application has to carry out.
#[derive(Clone, Debug, PartialEq)]
pub enum UiAction {
    /// Run a document command (the only way the document changes).
    Command(Command),
    /// Undo the last document change.
    Undo,
    /// Redo the last undone change.
    Redo,
    /// Start a new, empty document.
    NewDocument,
    /// Open a `.bevel` file.
    Open,
    /// Save to the current file (asks for one the first time).
    Save,
    /// Save to a new file.
    SaveAs,
    /// Open the sample document.
    OpenSample,
    /// Open the dialog for a new primitive.
    NewPrimitive(PrimitiveKind),
    /// Open the dialog for an existing feature.
    EditFeature(FeatureId),
    /// The open feature dialog's inputs changed: re-check and preview.
    FeatureDialogChanged,
    /// Accept the open feature dialog.
    FeatureDialogOk,
    /// Close the open feature dialog without changes.
    FeatureDialogCancel,
    /// Show or hide a body.
    SetBodyVisible(BodyId, bool),
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
    /// Stable id.
    pub id: BodyId,
    /// Display name.
    pub name: String,
    /// Whether it is drawn and exported.
    pub visible: bool,
}

/// How a timeline step fared in the last rebuild.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StepState {
    /// Built.
    Ok,
    /// Built, with a warning.
    Warning(String),
    /// Failed, with the reason.
    Error(String),
    /// Not built because an input is missing or failed, with the reason.
    Skipped(String),
    /// Suppressed by the user.
    Suppressed,
    /// After the rollback marker.
    RolledBack,
}

/// A feature as the timeline shows it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TimelineEntry {
    /// Stable id.
    pub id: FeatureId,
    /// Display name, e.g. "Box1".
    pub name: String,
    /// The feature's kind, for its icon.
    pub kind: PrimitiveKind,
    /// The features it depends on; it cannot move before them.
    pub deps: Vec<FeatureId>,
    /// Whether it is suppressed.
    pub suppressed: bool,
    /// The last rebuild's outcome.
    pub state: StepState,
}

/// A parameter as the Parameters dialog shows it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParamRow {
    /// Name.
    pub name: String,
    /// Expression as typed.
    pub expression: String,
    /// The formatted value, or the error.
    pub value: Result<String, String>,
    /// Comment.
    pub comment: String,
    /// Whether it belongs to a feature input (`d1`, ...) rather than the user.
    pub model: bool,
    /// The features and parameters that read it.
    pub used_by: Vec<String>,
}

/// The document as the UI shows it; the app refreshes it after every
/// change and rebuild.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DocView {
    /// The document's name, e.g. "Untitled" or the file name.
    pub title: String,
    /// Whether there are unsaved changes.
    pub modified: bool,
    /// The features in timeline order.
    pub timeline: Vec<TimelineEntry>,
    /// How many features run.
    pub marker: usize,
    /// User parameters first, then model parameters.
    pub parameters: Vec<ParamRow>,
    /// Symbol of the default length unit, e.g. "mm".
    pub units: String,
    /// Label of the change Undo would undo.
    pub undo: Option<String>,
    /// Label of the change Redo would redo.
    pub redo: Option<String>,
    /// Whether a rebuild is running.
    pub rebuilding: bool,
}

/// The open dialog for a new or edited primitive. The dialog edits the
/// texts; the app checks them and fills in `errors`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FeatureDialog {
    /// The feature being edited, or `None` for a new one.
    pub editing: Option<FeatureId>,
    /// Which primitive.
    pub kind: PrimitiveKind,
    /// The plane it stands on.
    pub plane: OriginPlane,
    /// One expression per field of [`PrimitiveKind::fields`].
    pub values: Vec<String>,
    /// The error for each field, if any, filled in by the app.
    pub errors: Vec<Option<String>>,
    /// An error for the whole feature (e.g. the kernel refused it).
    pub error: Option<String>,
}

impl FeatureDialog {
    /// A dialog for a new primitive with the default values.
    pub fn new(kind: PrimitiveKind) -> Self {
        let fields = kind.fields();
        Self {
            editing: None,
            kind,
            plane: OriginPlane::Xy,
            values: fields.iter().map(|f| f.default.to_owned()).collect(),
            errors: vec![None; fields.len()],
            error: None,
        }
    }

    /// Whether every field is valid and the feature builds.
    pub fn is_valid(&self) -> bool {
        self.error.is_none() && self.errors.iter().all(Option::is_none)
    }
}

/// One result in the Measure dialog.
#[derive(Clone, Debug, PartialEq)]
pub enum MeasureValue {
    /// One or more numbers in a unit, e.g. an area or a point.
    Numbers {
        /// The values; a point has three.
        values: Vec<f64>,
        /// Unit symbol, e.g. "mm²".
        unit: &'static str,
    },
    /// A description, e.g. the type of a surface.
    Text(String),
}

/// A labelled result in the Measure dialog.
#[derive(Clone, Debug, PartialEq)]
pub struct Measurement {
    /// What was measured, e.g. "Area".
    pub label: String,
    /// The result.
    pub value: MeasureValue,
}

/// What the Measure dialog shows for the current selection.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelectionInfo {
    /// What is selected, e.g. "Box · Top face".
    pub title: String,
    /// The results.
    pub measurements: Vec<Measurement>,
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
    /// What is selected, for the Measure dialog; `None` when nothing is.
    pub selection: Option<SelectionInfo>,
    /// Whether the Measure dialog is open.
    pub measure_open: bool,
    /// Decimal places the Measure dialog shows.
    pub measure_decimals: usize,
    /// The document, as the timeline, browser and dialogs show it.
    pub doc: DocView,
    /// The open feature dialog, if any.
    pub feature_dialog: Option<FeatureDialog>,
    /// Whether the Parameters dialog is open.
    pub show_parameters: bool,
    /// The feature under the cursor in the timeline, for highlighting what
    /// it built.
    pub hovered_feature: Option<FeatureId>,
    /// Current time, for timing notices; updated every frame.
    now: f64,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_log: false,
            measure_open: false,
            measure_decimals: 2,
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
            doc: DocView {
                title: "Untitled".to_owned(),
                units: "mm".to_owned(),
                ..DocView::default()
            },
            feature_dialog: None,
            show_parameters: false,
            hovered_feature: None,
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

    /// Runs a toolbar tool: opens it if it exists, otherwise says when it
    /// arrives.
    fn activate(&mut self, tool: &tools::Tool, actions: &mut Vec<UiAction>) {
        if tool.name == tools::MEASURE {
            self.measure_open = true;
        } else if tool.name == tools::PARAMETERS {
            self.show_parameters = true;
        } else if let Some(kind) = tools::primitive(tool) {
            actions.push(UiAction::NewPrimitive(kind));
        } else {
            self.notify_unavailable(tool.name, tool.phase);
        }
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
        .show(ui, |ui| toolbar::show(ui, state, &mut actions));
    egui::Panel::bottom("timeline")
        .frame(bar_frame(egui::Margin::symmetric(8, 5)))
        .resizable(false)
        .exact_size(40.0)
        .show(ui, |ui| timeline::show(ui, state, &mut actions));
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
    egui::CentralPanel::no_frame().show(ui, |ui| viewport(ui, state, &mut actions));

    if state.measure_open {
        measure::show(ui.ctx(), state);
    }
    if state.feature_dialog.is_some() {
        feature_dialog::show(ui.ctx(), state, &mut actions);
    }
    if state.show_parameters {
        parameters::show(ui.ctx(), state, &mut actions);
    }
    if state.show_about {
        about_window(ui.ctx(), &mut state.show_about);
    }
    if state.show_controls {
        controls_window(ui.ctx(), &mut state.show_controls);
    }
    let typing = ui.ctx().egui_wants_keyboard_input();
    if ui.input(|i| i.key_pressed(egui::Key::Home)) {
        actions.push(UiAction::FitAll);
    }
    shortcuts(ui, typing, &mut actions);
    if !typing && ui.input(|i| i.key_pressed(egui::Key::M)) {
        state.measure_open = !state.measure_open;
    }
    actions
}

/// Document shortcuts: undo, redo, save, new and open.
fn shortcuts(ui: &egui::Ui, typing: bool, actions: &mut Vec<UiAction>) {
    use egui::{Key, KeyboardShortcut, Modifiers};
    let shortcut = |mods, key| {
        ui.ctx()
            .input_mut(|i| i.consume_shortcut(&KeyboardShortcut::new(mods, key)))
    };
    let cmd_shift = Modifiers::COMMAND | Modifiers::SHIFT;
    if shortcut(cmd_shift, Key::S) {
        actions.push(UiAction::SaveAs);
    } else if shortcut(Modifiers::COMMAND, Key::S) {
        actions.push(UiAction::Save);
    }
    if shortcut(Modifiers::COMMAND, Key::O) {
        actions.push(UiAction::Open);
    }
    if shortcut(Modifiers::COMMAND, Key::N) {
        actions.push(UiAction::NewDocument);
    }
    // Text fields keep their own undo.
    if !typing {
        if shortcut(cmd_shift, Key::Z) || shortcut(Modifiers::COMMAND, Key::Y) {
            actions.push(UiAction::Redo);
        } else if shortcut(Modifiers::COMMAND, Key::Z) {
            actions.push(UiAction::Undo);
        }
    }
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
                    ("Select", "Click; Shift + click adds or removes"),
                    ("Measure", "M"),
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
                    id: BodyId {
                        feature: FeatureId(1),
                        index: 0,
                    },
                    name: "Box".to_owned(),
                    visible: true,
                },
                BodyEntry {
                    id: BodyId {
                        feature: FeatureId(2),
                        index: 0,
                    },
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
    fn there_is_no_properties_panel() {
        let mut state = with_bodies();
        let harness = harness(&mut state);
        assert!(harness.query_by_label("PROPERTIES").is_none());
    }

    #[test]
    fn measure_tool_opens_the_dialog_with_results() {
        let mut state = with_bodies();
        state.selection = Some(SelectionInfo {
            title: "Box · Top face".to_owned(),
            measurements: vec![Measurement {
                label: "Area".to_owned(),
                value: MeasureValue::Numbers {
                    values: vec![1600.0],
                    unit: "mm²",
                },
            }],
        });
        {
            let mut harness = harness(&mut state);
            harness.get_by_label("Measure").click();
            harness.run();
            harness.get_by_label("MEASURE");
            harness.get_by_label("Box · Top face");
            harness.get_by_label("1,600.00 mm²");
        }
        assert!(state.measure_open);
    }
}
