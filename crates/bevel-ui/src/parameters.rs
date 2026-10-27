// SPDX-License-Identifier: GPL-3.0-or-later

//! The Parameters dialog: user and model parameters with their
//! expressions, values and comments.
//!
//! Every edit is pushed as a [`ParamEdit`] command; text being typed lives
//! in egui memory until it is committed.

use bevel_doc::ParamEdit;
use bevel_expr::{ParamKind, Parameter};
use egui::{Id, Sense};

use crate::icons::Icon;
use crate::{Command, ParamRow, UiAction, UiState, theme};

const WIDTH: f32 = 560.0;

pub(crate) fn show(ctx: &egui::Context, state: &mut UiState, actions: &mut Vec<UiAction>) {
    let content = ctx.content_rect();
    let mut open = true;
    egui::Window::new("Parameters")
        .title_bar(false)
        .resizable(true)
        .collapsible(false)
        .default_size(egui::vec2(WIDTH, 380.0))
        .pivot(egui::Align2::CENTER_CENTER)
        .default_pos(content.center())
        .frame(
            egui::Frame::window(&ctx.global_style())
                .fill(theme::PANEL)
                .inner_margin(egui::Margin::same(0)),
        )
        .show(ctx, |ui| {
            header(ui, &mut open);
            egui::Frame::new()
                .inner_margin(egui::Margin::symmetric(12, 10))
                .show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink(false)
                        .show(ui, |ui| body(ui, state, actions));
                });
        });
    if !open {
        state.show_parameters = false;
    }
}

fn header(ui: &mut egui::Ui, open: &mut bool) {
    egui::Frame::new()
        .fill(theme::BAR)
        .inner_margin(egui::Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.add(Icon::Parameters.image(18.0));
                ui.label(
                    egui::RichText::new("PARAMETERS")
                        .strong()
                        .color(theme::TEXT),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let close = egui::Button::image(Icon::Close.image(12.0).alt_text("Close"))
                        .frame_when_inactive(false);
                    if ui.add(close).on_hover_text("Close").clicked() {
                        *open = false;
                    }
                });
            });
        });
}

fn weak(text: &str) -> egui::RichText {
    egui::RichText::new(text).small().color(theme::TEXT_WEAK)
}

fn body(ui: &mut egui::Ui, state: &UiState, actions: &mut Vec<UiAction>) {
    let (model, user): (Vec<&ParamRow>, Vec<&ParamRow>) =
        state.doc.parameters.iter().partition(|row| row.model);
    ui.label(weak("User parameters"));
    if user.is_empty() && model.is_empty() {
        ui.label(
            egui::RichText::new(
                "No parameters yet. Every dimension you type in a feature becomes one.",
            )
            .italics()
            .color(theme::TEXT_WEAK),
        );
    }
    if !user.is_empty() {
        grid(ui, "user_parameters", &user, actions);
    }
    add_row(ui, actions);
    if !model.is_empty() {
        ui.add_space(10.0);
        egui::CollapsingHeader::new(weak("Model parameters"))
            .default_open(true)
            .show(ui, |ui| grid(ui, "model_parameters", &model, actions));
    }
}

/// A table of parameters; user rows can be renamed and deleted, model rows
/// show the feature that owns them instead.
fn grid(ui: &mut egui::Ui, id: &str, rows: &[&ParamRow], actions: &mut Vec<UiAction>) {
    egui::Grid::new(id)
        .num_columns(5)
        .spacing(egui::vec2(10.0, 4.0))
        .show(ui, |ui| {
            ui.label(weak("Name"));
            let expression_h = ui.label(weak("Expression"));
            ui.label(weak("Value"));
            let comment_h = ui.label(weak("Comment"));
            ui.label(weak(if rows.first().is_some_and(|r| r.model) {
                "Feature"
            } else {
                ""
            }));
            ui.end_row();
            for row in rows {
                name_cell(ui, row, actions);
                if let Some(expression) = text_cell(
                    ui,
                    Id::unique(("param_expression", &row.name)),
                    &row.expression,
                    160.0,
                    true,
                    expression_h.id,
                ) {
                    push(
                        actions,
                        ParamEdit::SetExpression {
                            name: row.name.clone(),
                            expression,
                        },
                    );
                }
                value_cell(ui, row);
                if let Some(comment) = text_cell(
                    ui,
                    Id::unique(("param_comment", &row.name)),
                    &row.comment,
                    140.0,
                    false,
                    comment_h.id,
                ) {
                    push(
                        actions,
                        ParamEdit::SetComment {
                            name: row.name.clone(),
                            comment,
                        },
                    );
                }
                last_cell(ui, row, actions);
                ui.end_row();
            }
        });
}

fn push(actions: &mut Vec<UiAction>, edit: ParamEdit) {
    actions.push(UiAction::Command(Command::Parameters(edit)));
}

/// The name: a label, or for user parameters a text box after a double click.
fn name_cell(ui: &mut egui::Ui, row: &ParamRow, actions: &mut Vec<UiAction>) {
    if row.model {
        ui.label(egui::RichText::new(&row.name).monospace());
        return;
    }
    let id = Id::unique(("param_rename", &row.name));
    let editing: Option<(String, bool)> = ui.data(|d| d.get_temp(id));
    let Some((mut text, fresh)) = editing else {
        let label = ui.add(
            egui::Label::new(egui::RichText::new(&row.name).monospace()).sense(Sense::click()),
        );
        if label.double_clicked() {
            ui.data_mut(|d| d.insert_temp(id, (row.name.clone(), true)));
        }
        label.on_hover_text("Double-click to rename");
        return;
    };
    let response = ui.add(
        egui::TextEdit::singleline(&mut text)
            .id(id.with("edit"))
            .desired_width(90.0)
            .font(egui::TextStyle::Monospace),
    );
    if fresh {
        response.request_focus();
    } else if response.lost_focus() {
        let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
        let new = text.trim().to_owned();
        if enter && !new.is_empty() && new != row.name {
            push(
                actions,
                ParamEdit::Rename {
                    old: row.name.clone(),
                    new,
                },
            );
        }
        ui.data_mut(|d| d.remove_temp::<(String, bool)>(id));
        return;
    }
    ui.data_mut(|d| d.insert_temp(id, (text, false)));
}

/// A text box that edits a buffer and returns the text when the user
/// commits it (Enter or leaving the box) and it differs from `current`.
/// While the box has no focus it shows `current`, so undo and redo show.
fn text_cell(
    ui: &mut egui::Ui,
    id: Id,
    current: &str,
    width: f32,
    monospace: bool,
    label: Id,
) -> Option<String> {
    let edit_id = id.with("edit");
    let focused = ui.memory(|m| m.has_focus(edit_id));
    let mut text = if focused {
        ui.data(|d| d.get_temp::<String>(id))
            .unwrap_or_else(|| current.to_owned())
    } else {
        current.to_owned()
    };
    let mut edit = egui::TextEdit::singleline(&mut text)
        .id(edit_id)
        .desired_width(width);
    if monospace {
        edit = edit.font(egui::TextStyle::Monospace);
    }
    let response = ui.add(edit).labelled_by(label);
    let cancelled = ui.input(|i| i.key_pressed(egui::Key::Escape));
    let commit = response.lost_focus() && !cancelled && text != current;
    ui.data_mut(|d| d.insert_temp(id, text.clone()));
    commit.then_some(text)
}

fn value_cell(ui: &mut egui::Ui, row: &ParamRow) {
    match &row.value {
        Ok(value) => {
            ui.label(egui::RichText::new(value).color(theme::TEXT));
        }
        Err(message) => {
            ui.label(egui::RichText::new("error").color(theme::ERROR))
                .on_hover_text(message);
        }
    }
}

/// The last column: the delete button of a user parameter, or the owning
/// feature of a model parameter.
fn last_cell(ui: &mut egui::Ui, row: &ParamRow, actions: &mut Vec<UiAction>) {
    if row.model {
        ui.label(weak(&row.used_by.join(", ")));
        return;
    }
    let delete = egui::Button::image(
        Icon::Close
            .image(12.0)
            .alt_text(format!("Delete {}", row.name)),
    )
    .frame_when_inactive(false);
    let response = ui
        .add_enabled(row.used_by.is_empty(), delete)
        .on_disabled_hover_text(format!("Used by {}", row.used_by.join(", ")));
    if response.on_hover_text("Delete").clicked() {
        push(
            actions,
            ParamEdit::Remove {
                name: row.name.clone(),
            },
        );
    }
}

/// The row that adds a user parameter.
fn add_row(ui: &mut egui::Ui, actions: &mut Vec<UiAction>) {
    let id = Id::unique("param_add");
    let (mut name, mut expression): (String, String) =
        ui.data(|d| d.get_temp(id)).unwrap_or_default();
    ui.add_space(8.0);
    ui.label(weak("Add parameter"));
    let mut add = false;
    ui.horizontal(|ui| {
        let name_edit = egui::TextEdit::singleline(&mut name)
            .hint_text("name")
            .font(egui::TextStyle::Monospace)
            .desired_width(90.0);
        let name_response = ui.add(name_edit);
        name_response.widget_info(|| {
            egui::WidgetInfo::labeled(egui::Role::TextInput, true, "New parameter name")
        });
        let expression_edit = egui::TextEdit::singleline(&mut expression)
            .hint_text("e.g. 20 mm")
            .font(egui::TextStyle::Monospace)
            .desired_width(160.0);
        let expression_response = ui.add(expression_edit);
        expression_response.widget_info(|| {
            egui::WidgetInfo::labeled(egui::Role::TextInput, true, "New parameter expression")
        });
        let ready = !name.trim().is_empty() && !expression.trim().is_empty();
        add |= ui.add_enabled(ready, egui::Button::new("Add")).clicked();
        let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
        add |= ready && enter && (name_response.lost_focus() || expression_response.lost_focus());
    });
    if add {
        push(
            actions,
            ParamEdit::Add(Parameter {
                name: name.trim().to_owned(),
                expression: expression.trim().to_owned(),
                comment: String::new(),
                kind: ParamKind::User,
            }),
        );
        name.clear();
        expression.clear();
    }
    ui.data_mut(|d| d.insert_temp(id, (name, expression)));
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::{LogBuffer, install, workbench};
    use egui_kittest::Harness;
    use egui_kittest::kittest::{NodeT as _, Queryable as _};

    fn row(name: &str, expression: &str, value: &str, model: bool) -> ParamRow {
        ParamRow {
            name: name.to_owned(),
            expression: expression.to_owned(),
            value: Ok(value.to_owned()),
            comment: String::new(),
            model,
            used_by: vec!["Box1".to_owned()],
        }
    }

    fn state() -> UiState {
        let mut state = UiState::default();
        state.doc.parameters = vec![
            row("width", "20 mm", "20.00 mm", false),
            row("d1", "width * 2", "40.00 mm", true),
        ];
        state.show_parameters = true;
        state
    }

    fn run(state: &mut UiState, interact: impl FnOnce(&mut Harness<'_>)) -> Vec<UiAction> {
        let mut actions = Vec::new();
        {
            let log = LogBuffer::default();
            let mut harness = Harness::builder()
                .with_size(egui::vec2(1400.0, 900.0))
                .build_ui(|ui| {
                    install(ui.ctx());
                    actions.extend(workbench(ui, state, &log, |_, _, _| {}));
                });
            harness.run();
            interact(&mut harness);
            harness.run();
        }
        actions
    }

    #[test]
    fn rows_show_names_and_values() {
        let mut state = state();
        run(&mut state, |h| {
            for label in ["PARAMETERS", "width", "d1", "20.00 mm", "40.00 mm"] {
                h.get_by_label(label);
            }
        });
    }

    #[test]
    fn a_used_parameter_cannot_be_deleted() {
        let mut state = state();
        run(&mut state, |h| {
            let delete = h.get_by_label("Delete width");
            assert!(delete.accesskit_node().is_disabled());
        });
    }

    #[test]
    fn an_unused_parameter_can_be_deleted() {
        let mut state = state();
        state.doc.parameters[0].used_by.clear();
        let actions = run(&mut state, |h| h.get_by_label("Delete width").click());
        assert!(
            actions.contains(&UiAction::Command(Command::Parameters(ParamEdit::Remove {
                name: "width".to_owned()
            })))
        );
    }

    #[test]
    fn adding_a_parameter_pushes_the_edit() {
        let mut state = state();
        let actions = run(&mut state, |h| {
            h.get_by_label("New parameter name").focus();
            h.run();
            h.get_by_label("New parameter name").type_text("depth");
            h.run();
            h.get_by_label("New parameter expression").focus();
            h.run();
            h.get_by_label("New parameter expression").type_text("5 mm");
            h.run();
            h.get_by_label("Add").click();
        });
        let added = actions.iter().find_map(|a| match a {
            UiAction::Command(Command::Parameters(ParamEdit::Add(p))) => Some(p),
            _ => None,
        });
        let added = added.expect("an Add edit");
        assert_eq!(added.name, "depth");
        assert_eq!(added.expression, "5 mm");
        assert_eq!(added.kind, ParamKind::User);
    }

    #[test]
    fn the_close_button_hides_the_dialog() {
        let mut state = state();
        run(&mut state, |h| h.get_by_label("Close").click());
        assert!(!state.show_parameters);
    }

    #[test]
    fn an_empty_table_explains_itself() {
        let mut state = UiState {
            show_parameters: true,
            ..UiState::default()
        };
        run(&mut state, |h| {
            h.get_by_label("No parameters yet. Every dimension you type in a feature becomes one.");
        });
    }
}
