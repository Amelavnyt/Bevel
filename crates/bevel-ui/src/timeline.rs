// SPDX-License-Identifier: GPL-3.0-or-later

//! The part timeline strip: playback controls, the step track with its
//! rollback marker, and the document units.

use crate::icons::Icon;
use crate::{theme, widgets};

pub(crate) fn show(ui: &mut egui::Ui) {
    ui.horizontal_centered(|ui| {
        ui.add_enabled_ui(false, |ui| {
            for (icon, tip) in [
                (Icon::First, "Go to start"),
                (Icon::Prev, "Step back"),
                (Icon::Next, "Step forward"),
                (Icon::Last, "Go to end"),
            ] {
                widgets::icon_button(ui, icon, 14.0, tip).on_disabled_hover_text(tip);
            }
        });
        ui.add_space(4.0);

        let units_width = 44.0;
        let track_width = (ui.available_width() - units_width - 12.0).max(40.0);
        let (track, _) =
            ui.allocate_exact_size(egui::vec2(track_width, 26.0), egui::Sense::hover());
        let painter = ui.painter();
        painter.rect_filled(track, 5.0, theme::RECESSED);
        painter.rect_stroke(
            track,
            5.0,
            egui::Stroke::new(1.0, theme::BORDER),
            egui::StrokeKind::Inside,
        );
        // The rollback marker sits at the end of the (empty) history.
        let x = track.left() + 10.0;
        painter.vline(
            x,
            track.y_range().shrink(4.0),
            egui::Stroke::new(2.5, theme::ACCENT_LIGHT),
        );
        painter.text(
            egui::pos2(x + 12.0, track.center().y),
            egui::Align2::LEFT_CENTER,
            "Features appear here as you model",
            egui::FontId::proportional(12.0),
            theme::TEXT_WEAK,
        );

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(egui::RichText::new("mm").color(theme::TEXT_WEAK))
                .on_hover_text("Document units");
        });
    });
}
