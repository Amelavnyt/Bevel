// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{theme, widgets};

/// Properties of the selection, or the open feature's dialog.
pub(crate) fn show(ui: &mut egui::Ui) {
    widgets::panel_heading(ui, "PROPERTIES");
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
}
