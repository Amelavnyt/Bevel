// SPDX-License-Identifier: GPL-3.0-or-later

//! The 3D viewport area: camera navigation input, the rendered image and the
//! controls floating over it (view cube, navigation bar, axis triad,
//! notices).
//!
//! Navigation:
//! - orbit: right drag, or Shift + middle drag
//! - pan: middle drag, or Shift + right drag
//! - zoom: scroll wheel or pinch, towards the cursor

use crate::icons::Icon;
use crate::{UiAction, UiState, theme, viewcube, widgets};
use bevel_math::DVec2;
use bevel_render::{Camera, Projection, StandardView};

/// Zoom exponent per scrolled point; scrolling 100 points zooms by about e^0.15.
const ZOOM_PER_SCROLL_POINT: f64 = 0.0015;

/// The viewport's place on screen this frame.
#[derive(Clone, Copy, Debug)]
pub struct ViewportArea {
    /// Area in UI points.
    pub rect: egui::Rect,
    /// Size of the area in physical pixels, at least 1x1.
    pub size_px: [u32; 2],
}

impl ViewportArea {
    /// Width divided by height.
    pub fn aspect(&self) -> f64 {
        f64::from(self.size_px[0]) / f64::from(self.size_px[1])
    }
}

/// Claims all remaining space for the viewport and applies camera navigation.
pub fn interact(ui: &mut egui::Ui, camera: &mut Camera) -> ViewportArea {
    let (rect, response) =
        ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
    let pixels_per_point = ui.ctx().pixels_per_point();
    #[expect(
        clippy::cast_sign_loss,
        reason = "clamped to at least 1 before the cast"
    )]
    let to_px = |points: f32| (points * pixels_per_point).round().max(1.0) as u32;
    let size_px = [to_px(rect.width()), to_px(rect.height())];
    let area = ViewportArea { rect, size_px };
    let height_points = f64::from(rect.height().max(1.0));

    let shift = ui.input(|i| i.modifiers.shift);
    let delta = response.drag_delta();
    let delta = DVec2::new(f64::from(delta.x), f64::from(delta.y));
    if delta != DVec2::ZERO {
        let right = response.dragged_by(egui::PointerButton::Secondary);
        let middle = response.dragged_by(egui::PointerButton::Middle);
        if (right && !shift) || (middle && shift) {
            camera.orbit(delta);
        } else if middle || right {
            camera.pan(delta, height_points);
        }
    }
    if response.dragged_by(egui::PointerButton::Secondary)
        || response.dragged_by(egui::PointerButton::Middle)
    {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
    }

    if let Some(pointer) = response.hover_pos() {
        let (scroll, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
        let factor = (-f64::from(scroll) * ZOOM_PER_SCROLL_POINT).exp() / f64::from(pinch);
        if (factor - 1.0).abs() > 1e-9 {
            let ndc = DVec2::new(
                f64::from((pointer.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0,
                1.0 - f64::from((pointer.y - rect.top()) / rect.height().max(1.0)) * 2.0,
            );
            camera.zoom_about(factor, ndc, area.aspect());
        }
    }
    area
}

/// Draws the rendered viewport image, or a placeholder when there is none.
pub fn paint(ui: &egui::Ui, area: &ViewportArea, texture: Option<egui::TextureId>) {
    let painter = ui.painter_at(area.rect);
    if let Some(texture) = texture {
        let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
        painter.image(texture, area.rect, uv, egui::Color32::WHITE);
    } else {
        painter.rect_filled(area.rect, 0.0, ui.visuals().extreme_bg_color);
        painter.text(
            area.rect.center(),
            egui::Align2::CENTER_CENTER,
            "3D viewport unavailable: no GPU renderer",
            egui::FontId::proportional(14.0),
            ui.visuals().weak_text_color(),
        );
    }
}

/// Draws the controls that float over the viewport and collects the actions
/// they trigger.
pub fn overlays(
    ui: &mut egui::Ui,
    area: &ViewportArea,
    camera: &Camera,
    state: &mut UiState,
    actions: &mut Vec<UiAction>,
) {
    if let Some(view) = viewcube::show(ui, area.rect, camera) {
        actions.push(UiAction::SetView(view));
    }
    viewcube::axis_triad(ui, area.rect, camera);
    let bar = navigation_bar(ui, area.rect, state, actions);
    notice(ui, area.rect, bar.top(), state);
}

/// The navigation bar centred at the bottom of the viewport. Returns its rect.
fn navigation_bar(
    ui: &mut egui::Ui,
    viewport: egui::Rect,
    state: &mut UiState,
    actions: &mut Vec<UiAction>,
) -> egui::Rect {
    let size = egui::vec2(236.0, 34.0);
    let rect = egui::Rect::from_center_size(
        egui::pos2(viewport.center().x, viewport.bottom() - 12.0 - size.y / 2.0),
        size,
    );
    let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect));
    egui::Frame::new()
        .fill(theme::OVERLAY)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(8.0)
        .inner_margin(egui::Margin::symmetric(6, 3))
        .show(&mut child, |ui| {
            ui.horizontal_centered(|ui| {
                ui.spacing_mut().item_spacing.x = 2.0;
                if widgets::icon_button(ui, Icon::Home, 18.0, "Home view").clicked() {
                    actions.push(UiAction::SetView(StandardView::Home));
                }
                if widgets::icon_button(ui, Icon::Fit, 18.0, "Fit all (Home key)").clicked() {
                    actions.push(UiAction::FitAll);
                }
                widgets::vertical_separator(ui);
                for (projection, icon, tip) in [
                    (Projection::Perspective, Icon::Perspective, "Perspective"),
                    (Projection::Orthographic, Icon::Orthographic, "Orthographic"),
                ] {
                    let on = state.projection == projection;
                    if widgets::toggle_icon(ui, icon, 18.0, on, tip).clicked() {
                        actions.push(UiAction::SetProjection(projection));
                    }
                }
                widgets::vertical_separator(ui);
                if widgets::toggle_icon(ui, Icon::Grid, 18.0, state.show_grid, "Grid").clicked() {
                    state.show_grid = !state.show_grid;
                }
                if widgets::toggle_icon(ui, Icon::Origin, 18.0, state.show_origin, "Origin axes")
                    .clicked()
                {
                    state.show_origin = !state.show_origin;
                }
            });
        });
    rect
}

/// A short-lived message above the navigation bar.
fn notice(ui: &egui::Ui, viewport: egui::Rect, above: f32, state: &mut UiState) {
    let now = ui.input(|i| i.time);
    let Some(notice) = &state.notice else {
        return;
    };
    let remaining = notice.until - now;
    if remaining <= 0.0 {
        state.notice = None;
        return;
    }
    let alpha = (remaining / 0.4).clamp(0.0, 1.0) as f32;
    let painter = ui.painter_at(viewport);
    let galley = painter.layout(
        notice.text.clone(),
        egui::FontId::proportional(13.0),
        theme::TEXT.gamma_multiply(alpha),
        (viewport.width() - 40.0).min(520.0),
    );
    let rect = egui::Rect::from_center_size(
        egui::pos2(viewport.center().x, above - 14.0 - galley.size().y / 2.0),
        galley.size() + egui::vec2(28.0, 16.0),
    );
    painter.rect(
        rect,
        8.0,
        theme::OVERLAY.gamma_multiply(alpha),
        egui::Stroke::new(1.0, theme::ACCENT_LIGHT.gamma_multiply(alpha)),
        egui::StrokeKind::Inside,
    );
    painter.galley(rect.min + egui::vec2(14.0, 8.0), galley, theme::TEXT);
    ui.ctx().request_repaint();
}
