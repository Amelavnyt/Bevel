// SPDX-License-Identifier: GPL-3.0-or-later

//! The application bar: menus, quick-access buttons, the document title and
//! the branch selector.

use crate::icons::Icon;
use crate::{UiAction, UiState, theme, widgets};
use bevel_render::{Projection, StandardView};

const DOCUMENTS_PHASE: &str = "Phase 3: Document & timeline";

pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    let bar_rect = ui.max_rect();
    ui.horizontal_centered(|ui| {
        ui.add(Icon::Logo.image(20.0));
        ui.label(
            egui::RichText::new("Bevel")
                .strong()
                .size(14.5)
                .color(theme::ACCENT_LIGHT),
        );
        ui.add_space(6.0);
        ui.scope(|ui| menus(ui, state, actions));
        widgets::vertical_separator(ui);

        for (icon, name, shortcut) in [
            (Icon::New, "New", "Ctrl+N"),
            (Icon::Open, "Open", "Ctrl+O"),
            (Icon::Save, "Save", "Ctrl+S"),
        ] {
            let tip = format!("{name} ({shortcut})");
            if widgets::icon_button(ui, icon, 18.0, &tip).clicked() {
                state.notify_unavailable(name, DOCUMENTS_PHASE);
            }
        }
        widgets::vertical_separator(ui);
        ui.add_enabled_ui(false, |ui| {
            widgets::icon_button(ui, Icon::Undo, 18.0, "Undo (Ctrl+Z)")
                .on_disabled_hover_text("Nothing to undo");
            widgets::icon_button(ui, Icon::Redo, 18.0, "Redo (Ctrl+Shift+Z)")
                .on_disabled_hover_text("Nothing to redo");
        });

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if widgets::toggle_icon(ui, Icon::Log, 18.0, state.show_log, "Log").clicked() {
                state.show_log = !state.show_log;
            }
            // Branches arrive with versions; the selector shows where they will live.
            ui.add_enabled_ui(false, |ui| {
                ui.add(egui::Button::image_and_text(
                    Icon::Branch.image(16.0).alt_text("Branch"),
                    format!("{} ⏷", state.branch),
                ))
                .on_disabled_hover_text("Branches and versions arrive in Phase 12");
            });
        });
    });

    // The document title, centred on the whole bar.
    ui.painter().text(
        bar_rect.center(),
        egui::Align2::CENTER_CENTER,
        "Untitled",
        egui::FontId::proportional(13.5),
        theme::TEXT,
    );
}

fn menus(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    // Menu titles are flat until hovered, as in a menu bar.
    let widgets = &mut ui.visuals_mut().widgets;
    widgets.inactive.weak_bg_fill = egui::Color32::TRANSPARENT;
    widgets.inactive.bg_stroke = egui::Stroke::NONE;
    ui.menu_button("File", |ui| {
        for (name, shortcut) in [
            ("New", "Ctrl+N"),
            ("Open…", "Ctrl+O"),
            ("Save", "Ctrl+S"),
            ("Save As…", "Ctrl+Shift+S"),
        ] {
            ui.add_enabled(false, egui::Button::new(name).shortcut_text(shortcut))
                .on_disabled_hover_text(format!("Arrives in {DOCUMENTS_PHASE}"));
        }
        ui.separator();
        ui.add_enabled(false, egui::Button::new("Export…"))
            .on_disabled_hover_text("STL and 3MF export arrive in Phase 2");
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
            .add(egui::Button::image_and_text(
                Icon::Home.image(16.0),
                "Home View",
            ))
            .clicked()
        {
            actions.push(UiAction::SetView(StandardView::Home));
        }
        if ui
            .add(
                egui::Button::image_and_text(Icon::Fit.image(16.0), "Fit All")
                    .shortcut_text("Home"),
            )
            .clicked()
        {
            actions.push(UiAction::FitAll);
        }
        ui.menu_button("Standard Views", |ui| {
            for (view, name) in [
                (StandardView::Top, "Top"),
                (StandardView::Front, "Front"),
                (StandardView::Right, "Right"),
                (StandardView::Back, "Back"),
                (StandardView::Left, "Left"),
                (StandardView::Bottom, "Bottom"),
            ] {
                if ui.button(name).clicked() {
                    actions.push(UiAction::SetView(view));
                }
            }
        });
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
        ui.checkbox(&mut state.show_grid, "Grid");
        ui.checkbox(&mut state.show_origin, "Origin axes");
        ui.checkbox(&mut state.show_properties, "Properties panel");
        ui.checkbox(&mut state.show_geometry_debug, "Geometry Debug")
            .on_hover_text("Sample NURBS curves and surfaces from the geometry kernel");
        ui.checkbox(&mut state.show_log, "Log");
    });
    ui.menu_button("Help", |ui| {
        if ui.button("Viewport Controls").clicked() {
            state.show_controls = true;
        }
        if ui.button("About Bevel").clicked() {
            state.show_about = true;
        }
    });
}
