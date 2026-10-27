// SPDX-License-Identifier: GPL-3.0-or-later

//! The model browser: the document's origin, parameters, sketches, bodies
//! and construction geometry as a tree with visibility toggles.

use crate::icons::Icon;
use crate::{UiAction, UiState, theme, widgets};

const ROW_ICON: f32 = 16.0;

pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    widgets::panel_heading(ui, "BROWSER");
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            node(
                ui,
                "doc",
                true,
                None,
                Icon::Document,
                &state.doc.title.clone(),
                |ui, _| {
                    node(
                        ui,
                        "origin",
                        false,
                        Some(&mut state.show_origin),
                        Icon::Origin,
                        "Origin",
                        |ui, visible| {
                            for (icon, name) in [
                                (Icon::Point, "O"),
                                (Icon::Plane, "XY"),
                                (Icon::Plane, "XZ"),
                                (Icon::Plane, "YZ"),
                                (Icon::Axis, "X"),
                                (Icon::Axis, "Y"),
                                (Icon::Axis, "Z"),
                            ] {
                                leaf(ui, icon, name, visible);
                            }
                        },
                    );
                    parameters_node(ui, state);
                    node(
                        ui,
                        "sketches",
                        false,
                        None,
                        Icon::Folder,
                        "Sketches",
                        |ui, _| {
                            empty_note(ui, "No sketches");
                        },
                    );
                    bodies_node(ui, state, actions);
                    node(
                        ui,
                        "construction",
                        false,
                        None,
                        Icon::Folder,
                        "Construction",
                        |ui, _| {
                            empty_note(ui, "No construction geometry");
                        },
                    );
                },
            );
        });
}

/// The user parameters with their values, and a link to the dialog.
fn parameters_node(ui: &mut egui::Ui, state: &mut UiState) {
    let parameters = &state.doc.parameters;
    let mut open = false;
    node(
        ui,
        "parameters",
        false,
        None,
        Icon::Parameters,
        "Parameters",
        |ui, _| {
            let user: Vec<_> = parameters.iter().filter(|p| !p.model).collect();
            if user.is_empty() {
                empty_note(ui, "No user parameters");
            }
            for p in user {
                let text = match &p.value {
                    Ok(v) => format!("{} = {v}", p.name),
                    Err(_) => format!("{} = ?", p.name),
                };
                leaf(ui, Icon::Parameters, &text, true);
            }
            ui.horizontal(|ui| {
                ui.add_space(24.0);
                let link = egui::Link::new(egui::RichText::new("Change parameters…").small());
                open = ui.add(link).clicked();
            });
        },
    );
    if open {
        state.show_parameters = true;
    }
}

/// The bodies with visibility toggles; clicking a name selects the body.
fn bodies_node(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    let bodies = &mut state.bodies;
    node(ui, "bodies", true, None, Icon::Folder, "Bodies", |ui, _| {
        if bodies.is_empty() {
            empty_note(ui, "No bodies");
        }
        for (index, body) in bodies.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.add_space(4.0);
                let was = body.visible;
                visibility_toggle(ui, &mut body.visible, &body.name);
                if body.visible != was {
                    actions.push(UiAction::SetBodyVisible(body.id, body.visible));
                }
                let tint = if body.visible {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_white_alpha(90)
                };
                ui.add(Icon::Body.image(ROW_ICON).tint(tint));
                let color = if body.visible {
                    theme::TEXT
                } else {
                    theme::TEXT_WEAK
                };
                let label = egui::Label::new(egui::RichText::new(&body.name).color(color))
                    .sense(egui::Sense::click());
                if ui.add(label).clicked() {
                    actions.push(UiAction::SelectBody(index));
                }
            });
        }
    });
}

/// A collapsible tree row with an optional visibility toggle. `body`
/// receives whether the node is visible, for dimming its children.
fn node(
    ui: &mut egui::Ui,
    id: &str,
    default_open: bool,
    visible: Option<&mut bool>,
    icon: Icon,
    name: &str,
    body: impl FnOnce(&mut egui::Ui, bool),
) {
    let id = ui.make_persistent_id(("browser", id));
    let shown = visible.as_deref().copied().unwrap_or(true);
    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, default_open)
        .show_header(ui, |ui| {
            if let Some(visible) = visible {
                visibility_toggle(ui, visible, name);
            }
            row_label(ui, icon, name, shown);
        })
        .body(|ui| body(ui, shown));
}

fn leaf(ui: &mut egui::Ui, icon: Icon, name: &str, visible: bool) {
    ui.horizontal(|ui| {
        ui.add_space(18.0);
        row_label(ui, icon, name, visible);
    });
}

fn row_label(ui: &mut egui::Ui, icon: Icon, name: &str, visible: bool) {
    let tint = if visible {
        egui::Color32::WHITE
    } else {
        egui::Color32::from_white_alpha(90)
    };
    ui.add(icon.image(ROW_ICON).tint(tint));
    let color = if visible {
        theme::TEXT
    } else {
        theme::TEXT_WEAK
    };
    ui.label(egui::RichText::new(name).color(color));
}

fn visibility_toggle(ui: &mut egui::Ui, visible: &mut bool, name: &str) {
    let (icon, tip) = if *visible {
        (Icon::Eye, format!("Hide {name}"))
    } else {
        (Icon::EyeOff, format!("Show {name}"))
    };
    if ui
        .add(
            egui::Button::image(icon.image(14.0).alt_text(&tip))
                .frame_when_inactive(false)
                .min_size(egui::vec2(18.0, 16.0)),
        )
        .on_hover_text(tip)
        .clicked()
    {
        *visible = !*visible;
    }
}

fn empty_note(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).italics().color(theme::TEXT_WEAK));
}
