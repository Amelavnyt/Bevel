// SPDX-License-Identifier: GPL-3.0-or-later

//! The toolbar: one row of tool groups (CREATE, MODIFY, CONSTRUCT, INSPECT,
//! INSERT, UTILITIES). Each group shows its pinned tools as large icons and
//! lists all of its tools in a menu under the group label.

use crate::tools::{self, Tool, ToolGroup};
use crate::{UiState, theme, widgets};

const ICON: f32 = 28.0;
const MENU_ICON: f32 = 18.0;
/// Size of a toolbar button around its icon.
const BUTTON: egui::Vec2 = egui::vec2(40.0, 38.0);

pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 2.0;
        for (i, group) in ToolGroup::ALL.into_iter().enumerate() {
            if i > 0 {
                ui.add_space(6.0);
                widgets::vertical_separator(ui);
                ui.add_space(6.0);
            }
            group_ui(ui, state, group);
        }
    });
}

fn group_ui(ui: &mut egui::Ui, state: &mut UiState, group: ToolGroup) {
    let label = egui::RichText::new(format!("{} ⏷", group.label()))
        .font(theme::caption_font())
        .color(theme::TEXT_WEAK);
    let label_width = ui
        .painter()
        .layout_no_wrap(
            format!("{} ⏷", group.label()),
            theme::caption_font(),
            theme::TEXT_WEAK,
        )
        .size()
        .x
        + 2.0 * ui.spacing().button_padding.x;
    let pinned = tools::in_group(group).filter(|t| t.pinned).count() as f32;
    let row_width = pinned * BUTTON.x + (pinned - 1.0).max(0.0) * 2.0;
    let width = row_width.max(label_width);

    ui.allocate_ui_with_layout(
        egui::vec2(width, BUTTON.y + 20.0),
        egui::Layout::top_down(egui::Align::Center),
        |ui| {
            ui.spacing_mut().item_spacing.y = 0.0;
            ui.allocate_ui_with_layout(
                egui::vec2(width, BUTTON.y),
                egui::Layout::left_to_right(egui::Align::Center)
                    .with_main_align(egui::Align::Center),
                |ui| {
                    for tool in tools::in_group(group).filter(|t| t.pinned) {
                        let response = ui
                            .add(
                                egui::Button::image(tool.icon.image(ICON).alt_text(tool.name))
                                    .frame_when_inactive(false)
                                    .min_size(BUTTON),
                            )
                            .on_hover_ui(|ui| tooltip(ui, tool));
                        if response.clicked() {
                            state.activate(tool);
                        }
                    }
                },
            );
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            // Group labels are flat until hovered.
            let widgets = &mut ui.visuals_mut().widgets;
            widgets.inactive.weak_bg_fill = egui::Color32::TRANSPARENT;
            widgets.inactive.bg_stroke = egui::Stroke::NONE;
            egui::containers::menu::MenuButton::new(label).ui(ui, |ui| {
                for tool in tools::in_group(group) {
                    let mut button = egui::Button::image_and_text(
                        tool.icon.image(MENU_ICON).alt_text(tool.name),
                        tool.name,
                    );
                    if let Some(key) = tool.shortcut {
                        button = button.shortcut_text(key);
                    }
                    if ui.add(button).on_hover_ui(|ui| tooltip(ui, tool)).clicked() {
                        state.activate(tool);
                    }
                }
            });
        },
    );
}

fn tooltip(ui: &mut egui::Ui, tool: &Tool) {
    ui.set_max_width(280.0);
    ui.horizontal(|ui| {
        ui.add(tool.icon.image(32.0));
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.strong(tool.name);
                if let Some(key) = tool.shortcut {
                    ui.label(egui::RichText::new(key).monospace().color(theme::TEXT_WEAK));
                }
            });
            ui.label(tool.description);
        });
    });
    if !tools::is_available(tool) {
        ui.add_space(2.0);
        ui.label(
            egui::RichText::new(format!("Coming in {}", tool.phase))
                .small()
                .color(theme::HIGHLIGHT),
        );
    }
}
