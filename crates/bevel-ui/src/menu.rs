// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{UiAction, UiState};
use bevel_render::Projection;

/// File, Edit, View and Help menus, and the branch selector.
pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    egui::MenuBar::new().ui(ui, |ui| {
        ui.menu_button("File", |ui| {
            // Document commands arrive with the document model.
            for item in ["New", "Open…", "Save", "Save As…", "Export…"] {
                ui.add_enabled(false, egui::Button::new(item));
            }
            if state.can_quit {
                ui.separator();
                if ui.button("Quit").clicked() {
                    actions.push(UiAction::Quit);
                }
            }
        });
        ui.menu_button("Edit", |ui| {
            ui.add_enabled(false, egui::Button::new("Undo").shortcut_text("Ctrl+Z"));
            ui.add_enabled(
                false,
                egui::Button::new("Redo").shortcut_text("Ctrl+Shift+Z"),
            );
        });
        ui.menu_button("View", |ui| {
            if ui
                .add(egui::Button::new("Fit All").shortcut_text("Home"))
                .clicked()
            {
                actions.push(UiAction::FitAll);
            }
            ui.separator();
            for (projection, label) in [
                (Projection::Perspective, "Perspective"),
                (Projection::Orthographic, "Orthographic"),
            ] {
                if ui.radio(state.projection == projection, label).clicked() {
                    actions.push(UiAction::SetProjection(projection));
                }
            }
            ui.separator();
            ui.checkbox(&mut state.show_log, "Log");
        });
        ui.menu_button("Help", |ui| {
            if ui.button("About Bevel").clicked() {
                state.show_about = true;
            }
        });
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            // Branches arrive with versions; the selector shows where they will live.
            ui.add_enabled_ui(false, |ui| {
                egui::ComboBox::from_label("Branch")
                    .selected_text(state.branch.as_str())
                    .show_ui(ui, |_| {});
            });
        });
    });
}
