// SPDX-License-Identifier: GPL-3.0-or-later

//! The application bar: menus, quick-access buttons, the document title and
//! the branch selector.

use crate::icons::Icon;
use crate::{ExportFormat, UiAction, UiState, theme, widgets};
use bevel_render::{Projection, StandardView};

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

        for (icon, name, shortcut, action) in [
            (Icon::New, "New", "Ctrl+N", UiAction::NewDocument),
            (Icon::Open, "Open", "Ctrl+O", UiAction::Open),
            (Icon::Save, "Save", "Ctrl+S", UiAction::Save),
        ] {
            let tip = format!("{name} ({shortcut})");
            if widgets::icon_button(ui, icon, 18.0, &tip).clicked() {
                actions.push(action);
            }
        }
        widgets::vertical_separator(ui);
        let undo = state.doc.undo.clone();
        let redo = state.doc.redo.clone();
        ui.add_enabled_ui(undo.is_some(), |ui| {
            let tip = undo.map_or_else(
                || "Undo (Ctrl+Z)".to_owned(),
                |l| format!("Undo {l} (Ctrl+Z)"),
            );
            if widgets::icon_button(ui, Icon::Undo, 18.0, &tip)
                .on_disabled_hover_text("Nothing to undo")
                .clicked()
            {
                actions.push(UiAction::Undo);
            }
        });
        ui.add_enabled_ui(redo.is_some(), |ui| {
            let tip = redo.map_or_else(
                || "Redo (Ctrl+Shift+Z)".to_owned(),
                |l| format!("Redo {l} (Ctrl+Shift+Z)"),
            );
            if widgets::icon_button(ui, Icon::Redo, 18.0, &tip)
                .on_disabled_hover_text("Nothing to redo")
                .clicked()
            {
                actions.push(UiAction::Redo);
            }
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
        if state.doc.modified {
            format!("{} •", state.doc.title)
        } else {
            state.doc.title.clone()
        },
        egui::FontId::proportional(13.5),
        theme::TEXT,
    );
}

fn menus(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    // Menu titles are flat until hovered, as in a menu bar.
    let widgets = &mut ui.visuals_mut().widgets;
    widgets.inactive.weak_bg_fill = egui::Color32::TRANSPARENT;
    widgets.inactive.bg_stroke = egui::Stroke::NONE;
    ui.menu_button("File", |ui| file_menu(ui, state, actions));
    ui.menu_button("Edit", |ui| edit_menu(ui, state, actions));
    ui.menu_button("View", |ui| view_menu(ui, state, actions));
    ui.menu_button("Help", |ui| help_menu(ui, state, actions));
}

fn file_menu(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    for (name, shortcut, action) in [
        ("New", "Ctrl+N", UiAction::NewDocument),
        ("Open…", "Ctrl+O", UiAction::Open),
        ("Save", "Ctrl+S", UiAction::Save),
        ("Save As…", "Ctrl+Shift+S", UiAction::SaveAs),
    ] {
        if ui
            .add(egui::Button::new(name).shortcut_text(shortcut))
            .clicked()
        {
            actions.push(action);
        }
    }
    ui.separator();
    let has_bodies = state.bodies.iter().any(|b| b.visible);
    ui.add_enabled_ui(has_bodies, |ui| {
        ui.menu_button("Export", |ui| {
            for format in [ExportFormat::Stl, ExportFormat::ThreeMf] {
                if ui.button(format!("{}…", format.label())).clicked() {
                    actions.push(UiAction::Export(format));
                }
            }
        })
        .response
        .on_disabled_hover_text("No visible bodies to export");
    });
    if state.can_quit {
        ui.separator();
        if ui.button("Quit").clicked() {
            actions.push(UiAction::Quit);
        }
    }
}

fn edit_menu(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    let undo = state
        .doc
        .undo
        .as_ref()
        .map_or_else(|| "Undo".to_owned(), |l| format!("Undo {l}"));
    if ui
        .add_enabled(
            state.doc.undo.is_some(),
            egui::Button::new(undo).shortcut_text("Ctrl+Z"),
        )
        .clicked()
    {
        actions.push(UiAction::Undo);
    }
    let redo = state
        .doc
        .redo
        .as_ref()
        .map_or_else(|| "Redo".to_owned(), |l| format!("Redo {l}"));
    if ui
        .add_enabled(
            state.doc.redo.is_some(),
            egui::Button::new(redo).shortcut_text("Ctrl+Shift+Z"),
        )
        .clicked()
    {
        actions.push(UiAction::Redo);
    }
    ui.separator();
    if ui.button("Change Parameters…").clicked() {
        state.show_parameters = true;
    }
}

fn view_menu(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
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
        .add(egui::Button::image_and_text(Icon::Fit.image(16.0), "Fit All").shortcut_text("Home"))
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
    ui.checkbox(&mut state.show_geometry_debug, "Geometry Debug")
        .on_hover_text("Sample NURBS curves and surfaces from the geometry kernel");
    ui.checkbox(&mut state.show_log, "Log");
}

fn help_menu(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    if ui
        .button("Open Sample")
        .on_hover_text("A document with one of each primitive")
        .clicked()
    {
        actions.push(UiAction::OpenSample);
    }
    ui.separator();
    if ui.button("Viewport Controls").clicked() {
        state.show_controls = true;
    }
    if ui.button("About Bevel").clicked() {
        state.show_about = true;
    }
}
