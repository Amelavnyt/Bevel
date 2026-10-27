// SPDX-License-Identifier: GPL-3.0-or-later

//! The part timeline strip: playback controls, the step track with its
//! rollback marker, and the document units.
//!
//! The strip only reads the [`DocView`](crate::DocView) and pushes
//! [`UiAction`]s; the document changes when the app runs them.

use egui::{Align2, Color32, CornerRadius, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};

use crate::icons::Icon;
use crate::{Command, FeatureId, StepState, TimelineEntry, UiAction, UiState, theme, widgets};

/// Height of the track.
const TRACK_HEIGHT: f32 = 26.0;
/// Side of a chip.
const CHIP: f32 = 24.0;
/// Space between chips.
const GAP: f32 = 4.0;
/// Space before the first chip.
const PAD: f32 = 8.0;
/// Distance between the left edges of neighbouring chips.
const STEP: f32 = CHIP + GAP;
/// Width of the marker's grab handle.
const HANDLE: f32 = 9.0;

/// A chip drag in progress, for drawing the drop target.
struct DragPreview {
    entry: usize,
    gap: usize,
    allowed: bool,
    pointer: Pos2,
}

/// The inline rename popup, kept in egui memory.
#[derive(Clone, Default)]
struct Renaming(Option<RenameState>);

#[derive(Clone)]
struct RenameState {
    id: FeatureId,
    original: String,
    text: String,
    pos: Pos2,
    fresh: bool,
}

pub(crate) fn show(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) {
    ui.horizontal_centered(|ui| {
        playback(ui, state, actions);
        ui.add_space(4.0);
        let units_width = 44.0;
        let track_width = (ui.available_width() - units_width - 12.0).max(40.0);
        track(ui, state, actions, track_width);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(egui::RichText::new(&state.doc.units).color(theme::TEXT_WEAK))
                .on_hover_text("Document units");
            if state.doc.rebuilding {
                ui.add(egui::Spinner::new().size(12.0))
                    .on_hover_text("Rebuilding…");
            }
        });
    });
}

/// The four marker buttons.
fn playback(ui: &mut egui::Ui, state: &UiState, actions: &mut Vec<UiAction>) {
    let len = state.doc.timeline.len();
    let marker = state.doc.marker.min(len);
    let buttons = [
        (Icon::First, "Go to start", 0, marker > 0),
        (
            Icon::Prev,
            "Step back",
            marker.saturating_sub(1),
            marker > 0,
        ),
        (Icon::Next, "Step forward", marker + 1, marker < len),
        (Icon::Last, "Go to end", len, marker < len),
    ];
    for (icon, tip, index, enabled) in buttons {
        let clicked = ui
            .add_enabled_ui(enabled, |ui| widgets::icon_button(ui, icon, 14.0, tip))
            .inner
            .on_disabled_hover_text(tip)
            .clicked();
        if clicked {
            actions.push(UiAction::Command(Command::SetMarker { index }));
        }
    }
}

/// The recessed track with the chips and the marker.
fn track(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>, width: f32) {
    let (track, _) = ui.allocate_exact_size(egui::vec2(width, TRACK_HEIGHT), Sense::hover());
    ui.painter().rect_filled(track, 5.0, theme::RECESSED);
    ui.painter().rect_stroke(
        track,
        5.0,
        Stroke::new(1.0, theme::BORDER),
        StrokeKind::Inside,
    );
    if state.doc.timeline.is_empty() {
        state.hovered_feature = None;
        // The rollback marker sits at the end of the (empty) history.
        let x = track.left() + 10.0;
        ui.painter().vline(
            x,
            track.y_range().shrink(4.0),
            Stroke::new(2.5, theme::ACCENT_LIGHT),
        );
        let hint = Rect::from_min_max(egui::pos2(x + 12.0, track.top()), track.right_bottom());
        ui.put(
            hint,
            egui::Label::new(
                egui::RichText::new("Features appear here as you model")
                    .size(12.0)
                    .color(theme::TEXT_WEAK),
            )
            .truncate(),
        );
        return;
    }
    let mut child = ui.new_child(egui::UiBuilder::new().max_rect(track.shrink(1.0)));
    let mut hovered = None;
    egui::ScrollArea::horizontal()
        .id_salt("timeline_track")
        .auto_shrink(false)
        .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysHidden)
        .show(&mut child, |ui| {
            hovered = chips(ui, state, actions);
        });
    state.hovered_feature = hovered;
}

/// Converts a count to a coordinate step; timelines are short.
fn count_f(n: usize) -> f32 {
    f32::from(u16::try_from(n).unwrap_or(u16::MAX))
}

fn chip_rect(content: Rect, index: usize) -> Rect {
    let x = content.left() + PAD + count_f(index) * STEP;
    Rect::from_min_size(
        egui::pos2(x, content.center().y - CHIP / 2.0),
        Vec2::splat(CHIP),
    )
}

/// The x of the gap before chip `gap` (`len` is after the last).
fn gap_x(content: Rect, gap: usize) -> f32 {
    content.left() + PAD - GAP / 2.0 + count_f(gap) * STEP
}

/// The gap nearest to `x`.
fn gap_at(content: Rect, x: f32, len: usize) -> usize {
    (0..=len)
        .min_by(|a, b| {
            (gap_x(content, *a) - x)
                .abs()
                .total_cmp(&(gap_x(content, *b) - x).abs())
        })
        .unwrap_or(0)
}

/// Where entry `k` ends up when it is dropped into `gap`.
fn final_index(k: usize, gap: usize) -> usize {
    if gap <= k { gap } else { gap - 1 }
}

/// Whether moving entry `k` to final index `to` keeps every feature after
/// the ones it depends on.
fn move_allowed(entries: &[TimelineEntry], k: usize, to: usize) -> Result<(), String> {
    let Some(moved) = entries.get(k) else {
        return Ok(());
    };
    // Where entry `j` sits once `k` has been taken out.
    let without = |j: usize| if j < k { j } else { j - 1 };
    for (j, other) in entries.iter().enumerate() {
        if j == k {
            continue;
        }
        if moved.deps.contains(&other.id) && without(j) >= to {
            return Err(format!("Can't move {} before {}", moved.name, other.name));
        }
        if other.deps.contains(&moved.id) && without(j) < to {
            return Err(format!("Can't move {} after {}", moved.name, other.name));
        }
    }
    Ok(())
}

/// Draws the chips and the marker; returns the hovered feature.
fn chips(ui: &mut egui::Ui, state: &mut UiState, actions: &mut Vec<UiAction>) -> Option<FeatureId> {
    let entries = state.doc.timeline.clone();
    let marker_index = state.doc.marker.min(entries.len());
    let width = PAD * 2.0 + count_f(entries.len()) * STEP - GAP;
    let (content, _) = ui.allocate_exact_size(egui::vec2(width, CHIP), Sense::hover());
    let rename_id = ui.scope_id().with("rename");
    let mut hovered = None;
    let mut preview = None;
    for (i, entry) in entries.iter().enumerate() {
        let rect = chip_rect(content, i);
        let id = ui.scope_id().with(("chip", entry.id.0));
        let response = ui.interact(rect, id, Sense::click_and_drag());
        response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Button, true, &entry.name));
        let active = response.hovered() || response.dragged();
        if active {
            hovered = Some(entry.id);
        }
        paint_chip(ui, rect, entry, active, 1.0);
        if !response.dragged() {
            response.clone().on_hover_ui(|ui| tooltip(ui, entry));
        }
        if response.double_clicked() {
            actions.push(UiAction::EditFeature(entry.id));
        }
        context_menu(&response, entry, i, rename_id, actions);
        if let Some(p) = chip_drag(ui, state, actions, &response, content, &entries, i) {
            preview = Some(p);
        }
    }
    marker(ui, content, marker_index, entries.len(), actions);
    if let Some(p) = preview {
        paint_preview(ui, content, &entries, &p);
    }
    rename_popup(ui, rename_id, actions);
    hovered
}

fn tooltip(ui: &mut egui::Ui, entry: &TimelineEntry) {
    ui.label(egui::RichText::new(&entry.name).strong());
    ui.label(match &entry.state {
        StepState::Ok => "Built".to_owned(),
        StepState::Warning(w) => format!("Warning: {w}"),
        StepState::Error(e) => format!("Error: {e}"),
        StepState::Skipped(s) => format!("Skipped: {s}"),
        StepState::Suppressed => "Suppressed".to_owned(),
        StepState::RolledBack => "Rolled back".to_owned(),
    });
    ui.label(
        egui::RichText::new("Double-click to edit · drag to reorder")
            .small()
            .color(theme::TEXT_WEAK),
    );
}

/// Draws one chip; `fade` scales its opacity (for the drag ghost).
fn paint_chip(ui: &egui::Ui, rect: Rect, entry: &TimelineEntry, hovered: bool, fade: f32) {
    let painter = ui.painter();
    let alpha = match entry.state {
        StepState::Ok | StepState::Warning(_) | StepState::Error(_) => 1.0,
        StepState::Skipped(_) => 0.5,
        StepState::Suppressed => 0.4,
        StepState::RolledBack => 0.35,
    } * fade;
    let corner = CornerRadius::same(4);
    if hovered {
        painter.rect_filled(rect, corner, theme::HOVER.gamma_multiply(fade));
    }
    let icon_rect = Rect::from_center_size(rect.center(), Vec2::splat(18.0));
    Icon::primitive(entry.kind)
        .image(18.0)
        .tint(Color32::WHITE.gamma_multiply(alpha))
        .paint_at(ui, icon_rect);
    match entry.state {
        StepState::Warning(_) => {
            painter.hline(
                rect.x_range().shrink(3.0),
                rect.bottom() - 2.0,
                Stroke::new(2.0, theme::WARNING.gamma_multiply(fade)),
            );
        }
        StepState::Error(_) => {
            painter.rect_stroke(
                rect,
                corner,
                Stroke::new(1.5, theme::ERROR.gamma_multiply(fade)),
                StrokeKind::Inside,
            );
        }
        StepState::Skipped(_) => {
            painter.rect_stroke(
                rect,
                corner,
                Stroke::new(1.0, theme::WARNING.gamma_multiply(fade)),
                StrokeKind::Inside,
            );
        }
        StepState::Suppressed => {
            painter.line_segment(
                [
                    rect.left_bottom() + egui::vec2(3.0, -3.0),
                    rect.right_top() + egui::vec2(-3.0, 3.0),
                ],
                Stroke::new(1.5, theme::TEXT_WEAK.gamma_multiply(fade)),
            );
        }
        StepState::Ok | StepState::RolledBack => {}
    }
}

fn context_menu(
    response: &egui::Response,
    entry: &TimelineEntry,
    index: usize,
    rename_id: egui::Id,
    actions: &mut Vec<UiAction>,
) {
    let id = entry.id;
    response.context_menu(|ui| {
        if ui.button("Edit").clicked() {
            actions.push(UiAction::EditFeature(id));
            ui.close();
        }
        if ui.button("Rename…").clicked() {
            let state = RenameState {
                id,
                original: entry.name.clone(),
                text: entry.name.clone(),
                pos: response.rect.left_top() - egui::vec2(0.0, 4.0),
                fresh: true,
            };
            ui.data_mut(|d| d.insert_temp(rename_id, Renaming(Some(state))));
            ui.close();
        }
        let suppress = if entry.suppressed {
            "Unsuppress"
        } else {
            "Suppress"
        };
        if ui.button(suppress).clicked() {
            actions.push(UiAction::Command(Command::SetSuppressed {
                id,
                suppressed: !entry.suppressed,
            }));
            ui.close();
        }
        if ui.button("Roll marker here").clicked() {
            actions.push(UiAction::Command(Command::SetMarker { index: index + 1 }));
            ui.close();
        }
        ui.separator();
        if ui.button("Delete").clicked() {
            actions.push(UiAction::Command(Command::DeleteFeature {
                id,
                cascade: false,
            }));
            ui.close();
        }
    });
}

/// The small text box that renames a feature.
fn rename_popup(ui: &egui::Ui, rename_id: egui::Id, actions: &mut Vec<UiAction>) {
    let Some(Renaming(Some(mut rename))) = ui.data(|d| d.get_temp::<Renaming>(rename_id)) else {
        return;
    };
    let mut done = None;
    egui::Area::new(rename_id.with("area"))
        .order(egui::Order::Foreground)
        .pivot(Align2::LEFT_BOTTOM)
        .fixed_pos(rename.pos)
        .show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                let edit = egui::TextEdit::singleline(&mut rename.text)
                    .desired_width(140.0)
                    .hint_text("Name");
                let response = ui.add(edit);
                response.widget_info(|| {
                    egui::WidgetInfo::labeled(egui::Role::TextInput, true, "Rename")
                });
                if rename.fresh {
                    response.request_focus();
                    rename.fresh = false;
                } else if response.lost_focus() {
                    let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
                    done = Some(enter);
                }
            });
        });
    if done == Some(true) {
        let name = rename.text.trim().to_owned();
        if !name.is_empty() && name != rename.original {
            actions.push(UiAction::Command(Command::RenameFeature {
                id: rename.id,
                name,
            }));
        }
    }
    let state = if done.is_some() { None } else { Some(rename) };
    ui.data_mut(|d| d.insert_temp(rename_id, Renaming(state)));
}

/// Handles dragging a chip: returns the preview while dragging and pushes
/// the move when it is dropped.
fn chip_drag(
    ui: &egui::Ui,
    state: &mut UiState,
    actions: &mut Vec<UiAction>,
    response: &egui::Response,
    content: Rect,
    entries: &[TimelineEntry],
    index: usize,
) -> Option<DragPreview> {
    if !response.dragged() && !response.drag_stopped() {
        return None;
    }
    let pointer = ui.input(|i| i.pointer.latest_pos())?;
    let gap = gap_at(content, pointer.x, entries.len());
    let to = final_index(index, gap);
    let verdict = move_allowed(entries, index, to);
    if response.drag_stopped() {
        match verdict {
            Ok(()) if to != index => {
                actions.push(UiAction::Command(Command::MoveFeature {
                    id: entries[index].id,
                    to,
                }));
            }
            Ok(()) => {}
            Err(message) => state.notify(message),
        }
        return None;
    }
    Some(DragPreview {
        entry: index,
        gap,
        allowed: verdict.is_ok(),
        pointer,
    })
}

fn paint_preview(ui: &egui::Ui, content: Rect, entries: &[TimelineEntry], preview: &DragPreview) {
    let colour = if preview.allowed {
        theme::ACCENT
    } else {
        theme::ERROR
    };
    ui.painter().vline(
        gap_x(content, preview.gap),
        content.y_range().shrink(1.0),
        Stroke::new(2.0, colour),
    );
    let ghost = Rect::from_center_size(
        egui::pos2(preview.pointer.x, content.center().y),
        Vec2::splat(CHIP),
    );
    paint_chip(ui, ghost, &entries[preview.entry], false, 0.7);
}

/// The rollback marker: a bar with a grab handle, draggable between chips.
fn marker(
    ui: &mut egui::Ui,
    content: Rect,
    marker: usize,
    len: usize,
    actions: &mut Vec<UiAction>,
) {
    let x = gap_x(content, marker);
    let hit = Rect::from_center_size(
        egui::pos2(x, content.center().y),
        egui::vec2(HANDLE, content.height()),
    );
    let response = ui
        .interact(hit, ui.scope_id().with("marker"), Sense::drag())
        .on_hover_cursor(egui::CursorIcon::ResizeHorizontal);
    response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Button, true, "Rollback marker"));
    let draw = |ui: &egui::Ui, x: f32, colour: Color32| {
        let bar = Rect::from_x_y_ranges(x - 1.5..=x + 1.5, content.y_range());
        ui.painter().rect_filled(bar, 1.5, colour);
        let cap = Rect::from_min_size(
            egui::pos2(x - HANDLE / 2.0, content.top()),
            egui::vec2(HANDLE, 6.0),
        );
        ui.painter().rect_filled(cap, CornerRadius::same(3), colour);
    };
    draw(ui, x, theme::ACCENT);
    let Some(pointer) = ui.input(|i| i.pointer.latest_pos()) else {
        return;
    };
    let gap = gap_at(content, pointer.x, len);
    if response.dragged() {
        draw(ui, gap_x(content, gap), theme::ACCENT.gamma_multiply(0.5));
    } else if response.drag_stopped() && gap != marker {
        actions.push(UiAction::Command(Command::SetMarker { index: gap }));
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::{LogBuffer, PrimitiveKind, install, workbench};
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable as _;

    fn entry(id: u64, name: &str, deps: &[u64], state: StepState) -> TimelineEntry {
        TimelineEntry {
            id: FeatureId(id),
            name: name.to_owned(),
            kind: PrimitiveKind::Box,
            deps: deps.iter().map(|d| FeatureId(*d)).collect(),
            suppressed: false,
            state,
        }
    }

    fn chain() -> Vec<TimelineEntry> {
        // C depends on B, B depends on A, D is independent.
        vec![
            entry(1, "A", &[], StepState::Ok),
            entry(2, "B", &[1], StepState::Ok),
            entry(3, "C", &[2], StepState::Ok),
            entry(4, "D", &[], StepState::Ok),
        ]
    }

    #[test]
    fn final_index_accounts_for_removal() {
        assert_eq!(final_index(2, 0), 0);
        assert_eq!(final_index(2, 2), 2);
        assert_eq!(final_index(2, 3), 2);
        assert_eq!(final_index(2, 4), 3);
    }

    #[test]
    fn independent_features_move_freely() {
        let entries = chain();
        for to in 0..4 {
            assert!(move_allowed(&entries, 3, to).is_ok(), "to {to}");
        }
    }

    #[test]
    fn a_feature_cannot_precede_its_inputs() {
        let entries = chain();
        let err = move_allowed(&entries, 2, 0).unwrap_err();
        assert_eq!(err, "Can't move C before B");
        assert!(
            move_allowed(&entries, 2, 1)
                .unwrap_err()
                .contains("before B")
        );
        assert!(move_allowed(&entries, 2, 3).is_ok());
    }

    #[test]
    fn a_feature_cannot_follow_its_dependents() {
        let entries = chain();
        let err = move_allowed(&entries, 0, 2).unwrap_err();
        assert_eq!(err, "Can't move A after B");
        assert!(move_allowed(&entries, 0, 0).is_ok());
        assert!(move_allowed(&entries, 1, 1).is_ok());
    }

    fn run(state: &mut UiState, interact: impl FnOnce(&mut Harness<'_>)) -> Vec<UiAction> {
        let mut actions = Vec::new();
        {
            let log = LogBuffer::default();
            let mut harness = Harness::builder()
                .with_size(egui::vec2(1400.0, 900.0))
                .with_step_dt(0.05)
                .build_ui(|ui| {
                    install(ui.ctx());
                    actions.extend(workbench(ui, state, &log, |_, _, _| {}));
                });
            harness.run();
            interact(&mut harness);
            harness.run();
        }
        actions
    }

    fn three() -> UiState {
        let mut state = UiState::default();
        state.doc.timeline = vec![
            entry(1, "Box1", &[], StepState::Ok),
            entry(2, "Cyl1", &[], StepState::Error("bad".to_owned())),
            entry(3, "Sph1", &[], StepState::RolledBack),
        ];
        state.doc.marker = 2;
        state
    }

    #[test]
    fn every_feature_is_a_labelled_chip() {
        let mut state = three();
        run(&mut state, |h| {
            for name in ["Box1", "Cyl1", "Sph1"] {
                h.get_by_label(name);
            }
        });
    }

    #[test]
    fn playback_buttons_move_the_marker() {
        let mut state = three();
        let actions = run(&mut state, |h| h.get_by_label("Go to start").click());
        assert!(actions.contains(&UiAction::Command(Command::SetMarker { index: 0 })));
        let mut state = three();
        let actions = run(&mut state, |h| h.get_by_label("Go to end").click());
        assert!(actions.contains(&UiAction::Command(Command::SetMarker { index: 3 })));
    }

    #[test]
    fn double_clicking_a_chip_edits_it() {
        let mut state = three();
        let actions = run(&mut state, |h| {
            let chip = h.get_by_label("Cyl1");
            chip.click();
            h.step();
            h.get_by_label("Cyl1").click();
            h.step();
        });
        assert!(actions.contains(&UiAction::EditFeature(FeatureId(2))));
    }

    #[test]
    fn empty_timeline_shows_a_hint() {
        let mut state = UiState::default();
        run(&mut state, |h| {
            h.get_by_label("Features appear here as you model");
        });
    }
}
