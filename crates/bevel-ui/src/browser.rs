// SPDX-License-Identifier: GPL-3.0-or-later

/// The model browser: origin, parameters, sketches, bodies and construction geometry.
pub(crate) fn show(ui: &mut egui::Ui) {
    ui.strong("Browser");
    ui.separator();
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::CollapsingHeader::new("Origin")
            .default_open(true)
            .show(ui, |ui| {
                for item in [
                    "O (origin)",
                    "XY plane",
                    "XZ plane",
                    "YZ plane",
                    "X axis",
                    "Y axis",
                    "Z axis",
                ] {
                    ui.label(item);
                }
            });
        for (section, empty) in [
            ("Parameters", "No parameters"),
            ("Sketches", "No sketches"),
            ("Bodies", "No bodies"),
            ("Construction", "No construction geometry"),
        ] {
            egui::CollapsingHeader::new(section).show(ui, |ui| {
                ui.weak(empty);
            });
        }
    });
}
