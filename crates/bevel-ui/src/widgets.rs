// SPDX-License-Identifier: GPL-3.0-or-later

//! Small widgets shared by the panels.

use crate::icons::Icon;
use crate::theme;

/// A frameless icon button that shows a frame on hover.
pub fn icon_button(ui: &mut egui::Ui, icon: Icon, size: f32, tooltip: &str) -> egui::Response {
    ui.add(
        egui::Button::image(icon.image(size).alt_text(tooltip))
            .frame_when_inactive(false)
            .min_size(egui::vec2(size + 8.0, size + 6.0)),
    )
    .on_hover_text(tooltip)
}

/// An icon button that stays highlighted while `on`.
pub fn toggle_icon(
    ui: &mut egui::Ui,
    icon: Icon,
    size: f32,
    on: bool,
    tooltip: &str,
) -> egui::Response {
    ui.add(
        egui::Button::image(icon.image(size).alt_text(tooltip))
            .selected(on)
            .frame_when_inactive(on)
            .min_size(egui::vec2(size + 8.0, size + 6.0)),
    )
    .on_hover_text(tooltip)
}

/// A thin vertical line between groups in a horizontal layout.
pub fn vertical_separator(ui: &mut egui::Ui) {
    let height = ui.available_height().clamp(16.0, 52.0);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(1.0, height), egui::Sense::hover());
    ui.painter().vline(
        rect.center().x,
        rect.y_range().shrink(3.0),
        egui::Stroke::new(1.0, theme::BORDER),
    );
}

/// A small upper-case heading at the top of a panel.
pub fn panel_heading(ui: &mut egui::Ui, text: &str) {
    ui.add_space(2.0);
    ui.label(
        egui::RichText::new(text)
            .font(theme::caption_font())
            .color(theme::TEXT_WEAK)
            .strong(),
    );
    ui.add_space(2.0);
}
