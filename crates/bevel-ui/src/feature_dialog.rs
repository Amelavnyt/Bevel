// SPDX-License-Identifier: GPL-3.0-or-later

//! The dialog for creating or editing a primitive: a floating panel like
//! Measure, with one expression field per input, the plane, and OK/Cancel.
//! Every edit asks the app to re-check and preview; OK is enabled only when
//! every field is valid.

use crate::icons::Icon;
use crate::{FeatureDialog, OriginPlane, UiAction, UiState, theme};

const WIDTH: f32 = 300.0;

pub(crate) fn show(ctx: &egui::Context, state: &mut UiState, actions: &mut Vec<UiAction>) {
    let Some(dialog) = state.feature_dialog.as_mut() else {
        return;
    };
    let content = ctx.content_rect();
    let mut changed = false;
    let mut ok = false;
    let mut cancel = false;
    egui::Window::new("Feature")
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .fixed_size(egui::vec2(WIDTH, 0.0))
        .pivot(egui::Align2::RIGHT_TOP)
        .default_pos(egui::pos2(content.right() - 16.0, content.top() + 120.0))
        .frame(
            egui::Frame::window(&ctx.global_style())
                .fill(theme::PANEL)
                .inner_margin(egui::Margin::same(0)),
        )
        .show(ctx, |ui| {
            cancel |= header(ui, dialog);
            egui::Frame::new()
                .inner_margin(egui::Margin::symmetric(12, 10))
                .show(ui, |ui| {
                    changed |= fields(ui, dialog);
                    if let Some(error) = &dialog.error {
                        ui.add_space(6.0);
                        ui.label(egui::RichText::new(error).color(theme::ERROR));
                    }
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let valid = dialog.is_valid();
                            let button = egui::Button::new("OK").fill(if valid {
                                theme::ACCENT
                            } else {
                                theme::BAR
                            });
                            ok |= ui
                                .add_enabled(valid, button)
                                .on_disabled_hover_text("Fix the highlighted fields first")
                                .clicked();
                            cancel |= ui.button("Cancel").clicked();
                        });
                    });
                });
        });
    let (enter, escape) = ctx.input(|i| {
        (
            i.key_pressed(egui::Key::Enter),
            i.key_pressed(egui::Key::Escape),
        )
    });
    if changed {
        actions.push(UiAction::FeatureDialogChanged);
    }
    if cancel || escape {
        actions.push(UiAction::FeatureDialogCancel);
    } else if (ok || enter) && dialog.is_valid() && !changed {
        actions.push(UiAction::FeatureDialogOk);
    }
}

/// Draws the header; returns whether the close button was clicked.
fn header(ui: &mut egui::Ui, dialog: &FeatureDialog) -> bool {
    let mut close = false;
    egui::Frame::new()
        .fill(theme::BAR)
        .inner_margin(egui::Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(WIDTH - 20.0);
            ui.horizontal(|ui| {
                ui.add(Icon::primitive(dialog.kind).image(18.0));
                let title = match dialog.editing {
                    Some(_) => format!("EDIT {}", dialog.kind.name().to_uppercase()),
                    None => dialog.kind.name().to_uppercase(),
                };
                ui.label(egui::RichText::new(title).strong().color(theme::TEXT));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let button = egui::Button::image(Icon::Close.image(12.0).alt_text("Close"))
                        .frame_when_inactive(false);
                    close = ui.add(button).on_hover_text("Cancel (Esc)").clicked();
                });
            });
        });
    close
}

/// Draws the plane and the inputs; returns whether anything changed.
fn fields(ui: &mut egui::Ui, dialog: &mut FeatureDialog) -> bool {
    let mut changed = false;
    egui::Grid::new("feature_fields")
        .num_columns(2)
        .spacing(egui::vec2(10.0, 6.0))
        .show(ui, |ui| {
            let label = ui.label(egui::RichText::new("Plane").color(theme::TEXT_WEAK));
            let before = dialog.plane;
            egui::ComboBox::from_id_salt("feature_plane")
                .selected_text(dialog.plane.name())
                .width(150.0)
                .show_ui(ui, |ui| {
                    for plane in OriginPlane::ALL {
                        ui.selectable_value(&mut dialog.plane, plane, plane.name());
                    }
                })
                .response
                .labelled_by(label.id);
            changed |= dialog.plane != before;
            ui.end_row();

            for (i, field) in dialog.kind.fields().iter().enumerate() {
                let (Some(value), Some(error)) = (dialog.values.get_mut(i), dialog.errors.get(i))
                else {
                    continue;
                };
                let label = ui.label(egui::RichText::new(field.label).color(theme::TEXT_WEAK));
                let mut edit = egui::TextEdit::singleline(value)
                    .font(egui::TextStyle::Monospace)
                    .desired_width(150.0);
                if error.is_some() {
                    edit = edit.text_color(theme::ERROR);
                }
                let response = ui.add(edit).labelled_by(label.id);
                let response = match error {
                    Some(e) => response.on_hover_text(egui::RichText::new(e).color(theme::ERROR)),
                    None => response,
                };
                changed |= response.changed();
                ui.end_row();
            }
        });
    if let Some((label, message)) = dialog
        .kind
        .fields()
        .iter()
        .zip(&dialog.errors)
        .find_map(|(f, e)| e.as_ref().map(|e| (f.label, e)))
    {
        ui.label(
            egui::RichText::new(format!("{label}: {message}"))
                .small()
                .color(theme::ERROR),
        );
    }
    changed
}
