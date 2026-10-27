// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{SelectionInfo, theme, widgets};

/// Properties of the selection, or the open feature's dialog.
pub(crate) fn show(ui: &mut egui::Ui, selection: Option<&SelectionInfo>) {
    widgets::panel_heading(ui, "PROPERTIES");
    let Some(selection) = selection else {
        ui.add_space(24.0);
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new("Nothing selected").color(theme::TEXT_WEAK));
            ui.add_space(4.0);
            ui.label(
                egui::RichText::new("Select a feature, body or sketch to see its properties.")
                    .small()
                    .color(theme::TEXT_WEAK),
            );
        });
        return;
    };
    ui.add_space(4.0);
    ui.strong(&selection.title);
    ui.add_space(6.0);
    egui::Grid::new("selection_properties")
        .num_columns(2)
        .spacing(egui::vec2(12.0, 4.0))
        .striped(true)
        .show(ui, |ui| {
            for (label, value) in &selection.rows {
                ui.label(egui::RichText::new(label).color(theme::TEXT_WEAK));
                ui.label(egui::RichText::new(value).monospace());
                ui.end_row();
            }
        });
}
