// SPDX-License-Identifier: GPL-3.0-or-later

//! Colours, spacing and text sizes of the Bevel UI.

use egui::{Color32, CornerRadius, FontFamily, FontId, Margin, Stroke, TextStyle};

/// Window and panel background.
pub const PANEL: Color32 = Color32::from_rgb(0x25, 0x27, 0x2c);
/// The toolbar and app bar background, slightly lighter than panels.
pub const BAR: Color32 = Color32::from_rgb(0x2d, 0x30, 0x36);
/// Recessed areas: text fields, the timeline track.
pub const RECESSED: Color32 = Color32::from_rgb(0x1b, 0x1d, 0x21);
/// Borders between regions.
pub const BORDER: Color32 = Color32::from_rgb(0x3a, 0x3e, 0x46);
/// Normal text.
pub const TEXT: Color32 = Color32::from_rgb(0xd8, 0xdd, 0xe3);
/// Secondary text: hints, group labels.
pub const TEXT_WEAK: Color32 = Color32::from_rgb(0x8f, 0x97, 0xa1);
/// Bevel's signature teal: selection, focus, pressed and toggled controls.
pub const ACCENT: Color32 = Color32::from_rgb(0x2d, 0x89, 0x85);
/// A lighter teal for text and lines that must read on dark backgrounds.
pub const ACCENT_LIGHT: Color32 = Color32::from_rgb(0x58, 0xaa, 0xa5);
/// The light green accent: highlights, matching the green in the icons.
pub const HIGHLIGHT: Color32 = Color32::from_rgb(0x77, 0xc8, 0x9d);
/// Hovered widget background.
pub const HOVER: Color32 = Color32::from_rgb(0x3a, 0x3f, 0x47);
/// Pressed or selected widget background (dark signature teal).
pub const ACTIVE: Color32 = Color32::from_rgb(0x1f, 0x5c, 0x59);
/// Translucent background of controls floating over the viewport.
pub const OVERLAY: Color32 = Color32::from_rgba_premultiplied(0x22, 0x25, 0x2a, 0xe6);

/// Font for small upper-case section and group labels.
pub fn caption_font() -> FontId {
    FontId::new(10.5, FontFamily::Proportional)
}

/// Applies the Bevel look to `ctx`.
pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    ctx.style_mut_of(egui::Theme::Dark, |style| {
        style.text_styles = [
            (
                TextStyle::Heading,
                FontId::new(16.0, FontFamily::Proportional),
            ),
            (TextStyle::Body, FontId::new(13.5, FontFamily::Proportional)),
            (
                TextStyle::Button,
                FontId::new(13.5, FontFamily::Proportional),
            ),
            (
                TextStyle::Small,
                FontId::new(11.0, FontFamily::Proportional),
            ),
            (
                TextStyle::Monospace,
                FontId::new(12.5, FontFamily::Monospace),
            ),
        ]
        .into();

        let spacing = &mut style.spacing;
        spacing.item_spacing = egui::vec2(6.0, 4.0);
        spacing.button_padding = egui::vec2(7.0, 3.0);
        spacing.menu_margin = Margin::same(6);
        spacing.window_margin = Margin::same(10);
        spacing.indent = 16.0;
        spacing.interact_size.y = 22.0;

        let v = &mut style.visuals;
        v.dark_mode = true;
        v.override_text_color = None;
        v.panel_fill = PANEL;
        v.window_fill = PANEL;
        v.window_stroke = Stroke::new(1.0, BORDER);
        v.window_corner_radius = CornerRadius::same(8);
        v.menu_corner_radius = CornerRadius::same(6);
        v.extreme_bg_color = RECESSED;
        v.faint_bg_color = Color32::from_rgb(0x2a, 0x2d, 0x32);
        v.code_bg_color = RECESSED;
        v.hyperlink_color = ACCENT_LIGHT;
        v.warn_fg_color = Color32::from_rgb(0xe8, 0xb0, 0x4a);
        v.selection.bg_fill = ACTIVE;
        v.selection.stroke = Stroke::new(1.0, ACCENT);

        let corner = CornerRadius::same(5);
        let w = &mut v.widgets;
        w.noninteractive.bg_fill = PANEL;
        w.noninteractive.weak_bg_fill = PANEL;
        w.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
        w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
        w.noninteractive.corner_radius = corner;

        w.inactive.bg_fill = Color32::from_rgb(0x33, 0x37, 0x3e);
        w.inactive.weak_bg_fill = Color32::from_rgb(0x33, 0x37, 0x3e);
        w.inactive.bg_stroke = Stroke::NONE;
        w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
        w.inactive.corner_radius = corner;

        w.hovered.bg_fill = HOVER;
        w.hovered.weak_bg_fill = HOVER;
        w.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x4d, 0x54, 0x5e));
        w.hovered.fg_stroke = Stroke::new(1.5, Color32::WHITE);
        w.hovered.corner_radius = corner;
        w.hovered.expansion = 0.0;

        w.active.bg_fill = ACTIVE;
        w.active.weak_bg_fill = ACTIVE;
        w.active.bg_stroke = Stroke::new(1.0, ACCENT);
        w.active.fg_stroke = Stroke::new(1.5, Color32::WHITE);
        w.active.corner_radius = corner;
        w.active.expansion = 0.0;

        w.open.bg_fill = HOVER;
        w.open.weak_bg_fill = HOVER;
        w.open.bg_stroke = Stroke::new(1.0, BORDER);
        w.open.fg_stroke = Stroke::new(1.0, TEXT);
        w.open.corner_radius = corner;
    });
}
