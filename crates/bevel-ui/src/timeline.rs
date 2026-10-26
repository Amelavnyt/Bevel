// SPDX-License-Identifier: GPL-3.0-or-later

/// The part timeline strip with its playback controls.
pub(crate) fn show(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.add_enabled_ui(false, |ui| {
            for (icon, tip) in [
                ("⏮", "Go to start"),
                ("⏴", "Step back"),
                ("⏵", "Step forward"),
                ("⏭", "Go to end"),
            ] {
                ui.button(icon).on_disabled_hover_text(tip);
            }
        });
        ui.separator();
        ui.weak("The timeline is empty");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.weak("Part timeline");
        });
    });
}
