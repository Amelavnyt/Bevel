// SPDX-License-Identifier: GPL-3.0-or-later

//! The 3D viewport area: camera navigation input and showing the rendered image.
//!
//! Navigation:
//! - orbit: right drag, or Shift + middle drag
//! - pan: middle drag, or Shift + right drag
//! - zoom: scroll wheel or pinch, towards the cursor

use bevel_math::DVec2;
use bevel_render::Camera;

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
