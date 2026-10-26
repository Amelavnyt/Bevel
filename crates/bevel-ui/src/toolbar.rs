// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{UiState, Workspace};

/// Tool groups per workspace. Every tool is disabled until its feature exists.
///
/// Sketch tools (line, arc, dimension, trim, ...) are not listed here: they
/// replace these tools while a sketch is open.
fn tool_groups(workspace: Workspace) -> &'static [&'static [&'static str]] {
    match workspace {
        Workspace::Solid => &[
            &["Create Sketch"],
            &["Box", "Cylinder", "Sphere", "Cone", "Torus"],
            &["Extrude", "Revolve", "Sweep", "Loft"],
            &["Fillet", "Chamfer", "Shell", "Draft"],
            &["Combine", "Pattern", "Mirror"],
        ],
        Workspace::Inspect => &[&["Measure", "Section", "Mass Properties"]],
        Workspace::Scripts => &[&["Run Script", "Script Editor"]],
    }
}

/// Workspace tabs and the active workspace's tools.
pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState) {
    ui.horizontal_wrapped(|ui| {
        for workspace in Workspace::ALL {
            let text = egui::RichText::new(workspace.label()).strong();
            if ui
                .add(egui::Button::selectable(state.workspace == workspace, text))
                .clicked()
            {
                state.workspace = workspace;
            }
        }
        ui.separator();
        for (i, group) in tool_groups(state.workspace).iter().enumerate() {
            if i > 0 {
                ui.separator();
            }
            for tool in *group {
                ui.add_enabled(false, egui::Button::new(*tool));
            }
        }
    });
}
