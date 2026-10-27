// SPDX-License-Identifier: GPL-3.0-or-later

//! The Measure dialog: a small floating panel over the viewport, in the
//! style of the feature dialogs to come, showing results for the current
//! selection with a copy button per value and a precision setting.

use crate::icons::Icon;
use crate::{MeasureValue, UiState, theme};

const WIDTH: f32 = 300.0;

/// Formats `value` with `decimals` places and thousands separators.
pub(crate) fn format_number(value: f64, decimals: usize) -> String {
    let text = format!("{:.*}", decimals, value + 0.0);
    let (sign, digits) = text
        .strip_prefix('-')
        .map_or(("", text.as_str()), |d| ("-", d));
    let (whole, fraction) = digits
        .split_once('.')
        .map_or((digits, None), |(w, f)| (w, Some(f)));
    let mut grouped = String::new();
    for (i, c) in whole.chars().enumerate() {
        if i > 0 && (whole.len() - i) % 3 == 0 {
            grouped.push(',');
        }
        grouped.push(c);
    }
    // "-0.00" reads as a negative value that isn't there.
    let sign = if grouped
        .chars()
        .chain(fraction.unwrap_or("").chars())
        .all(|c| c == '0' || c == ',')
    {
        ""
    } else {
        sign
    };
    match fraction {
        Some(f) => format!("{sign}{grouped}.{f}"),
        None => format!("{sign}{grouped}"),
    }
}

/// The text of a result, as shown and copied.
pub(crate) fn format_value(value: &MeasureValue, decimals: usize) -> String {
    match value {
        MeasureValue::Numbers { values, unit } => {
            let numbers: Vec<String> = values.iter().map(|v| format_number(*v, decimals)).collect();
            format!("{} {unit}", numbers.join(", "))
        }
        MeasureValue::Text(text) => text.clone(),
    }
}

pub(crate) fn show(ctx: &egui::Context, state: &mut UiState) {
    let content = ctx.content_rect();
    let mut open = true;
    egui::Window::new("Measure")
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .fixed_size(egui::vec2(WIDTH, 0.0))
        .pivot(egui::Align2::RIGHT_TOP)
        .default_pos(egui::pos2(content.right() - 16.0, content.top() + 280.0))
        .frame(
            egui::Frame::window(&ctx.global_style())
                .fill(theme::PANEL)
                .inner_margin(egui::Margin::same(0)),
        )
        .show(ctx, |ui| {
            header(ui, &mut open);
            egui::Frame::new()
                .inner_margin(egui::Margin::symmetric(12, 10))
                .show(ui, |ui| body(ui, state));
        });
    if !open {
        state.measure_open = false;
    }
}

fn header(ui: &mut egui::Ui, open: &mut bool) {
    egui::Frame::new()
        .fill(theme::BAR)
        .inner_margin(egui::Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(WIDTH - 20.0);
            ui.horizontal(|ui| {
                ui.add(Icon::Measure.image(18.0));
                ui.label(egui::RichText::new("MEASURE").strong().color(theme::TEXT));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let close = egui::Button::image(Icon::Close.image(12.0).alt_text("Close"))
                        .frame_when_inactive(false);
                    if ui.add(close).on_hover_text("Close (M)").clicked() {
                        *open = false;
                    }
                });
            });
        });
}

fn body(ui: &mut egui::Ui, state: &mut UiState) {
    ui.label(
        egui::RichText::new("Selection")
            .small()
            .color(theme::TEXT_WEAK),
    );
    let Some(selection) = &state.selection else {
        ui.label(
            egui::RichText::new("Select faces, edges or bodies to measure.")
                .italics()
                .color(theme::TEXT_WEAK),
        );
        precision(ui, &mut state.measure_decimals);
        return;
    };
    ui.label(egui::RichText::new(&selection.title).strong());
    ui.add_space(8.0);
    ui.label(
        egui::RichText::new("Results")
            .small()
            .color(theme::TEXT_WEAK),
    );
    let decimals = state.measure_decimals;
    egui::Grid::new("measure_results")
        .num_columns(3)
        .spacing(egui::vec2(10.0, 4.0))
        .show(ui, |ui| {
            for m in &selection.measurements {
                let text = format_value(&m.value, decimals);
                ui.label(egui::RichText::new(&m.label).color(theme::TEXT_WEAK));
                ui.label(egui::RichText::new(&text).monospace());
                let copy = egui::Button::image(Icon::Copy.image(12.0).alt_text("Copy"))
                    .frame_when_inactive(false);
                if ui.add(copy).on_hover_text("Copy").clicked() {
                    ui.ctx().copy_text(text);
                }
                ui.end_row();
            }
        });
    precision(ui, &mut state.measure_decimals);
}

fn precision(ui: &mut egui::Ui, decimals: &mut usize) {
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        let label = ui.label(
            egui::RichText::new("Precision")
                .small()
                .color(theme::TEXT_WEAK),
        );
        egui::ComboBox::from_id_salt("measure_precision")
            .selected_text(format_number(0.0, *decimals))
            .width(80.0)
            .show_ui(ui, |ui| {
                for d in 0..=4 {
                    ui.selectable_value(decimals, d, format_number(0.0, d));
                }
            })
            .response
            .labelled_by(label.id);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_get_separators_and_no_negative_zero() {
        assert_eq!(format_number(1600.0, 2), "1,600.00");
        assert_eq!(format_number(-1_234_567.891, 1), "-1,234,567.9");
        assert_eq!(format_number(-0.001, 2), "0.00");
        assert_eq!(format_number(12.5, 0), "12");
        assert_eq!(format_number(999.999, 2), "1,000.00");
    }

    #[test]
    fn points_are_joined() {
        let value = MeasureValue::Numbers {
            values: vec![1.0, -0.0, 2.5],
            unit: "mm",
        };
        assert_eq!(format_value(&value, 1), "1.0, 0.0, 2.5 mm");
    }
}
