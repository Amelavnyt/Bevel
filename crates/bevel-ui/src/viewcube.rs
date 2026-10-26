// SPDX-License-Identifier: GPL-3.0-or-later

//! The view cube: a small cube in the corner of the viewport that turns with
//! the camera. Clicking a face looks at the model from that side.

use crate::theme;
use bevel_math::{DVec2, Vector3};
use bevel_render::{Camera, StandardView};

/// Half the cube's edge, in points.
const HALF: f32 = 32.0;
/// Size of the square the cube is drawn in.
const AREA: f32 = 124.0;

struct Face {
    normal: Vector3,
    /// Reading direction of the label on the face.
    u: Vector3,
    /// "Up" direction of the label on the face.
    v: Vector3,
    label: &'static str,
    view: StandardView,
}

const FACES: [Face; 6] = [
    Face {
        normal: Vector3::Z,
        u: Vector3::X,
        v: Vector3::Y,
        label: "TOP",
        view: StandardView::Top,
    },
    Face {
        normal: Vector3::NEG_Z,
        u: Vector3::X,
        v: Vector3::NEG_Y,
        label: "BOTTOM",
        view: StandardView::Bottom,
    },
    Face {
        normal: Vector3::NEG_Y,
        u: Vector3::X,
        v: Vector3::Z,
        label: "FRONT",
        view: StandardView::Front,
    },
    Face {
        normal: Vector3::Y,
        u: Vector3::NEG_X,
        v: Vector3::Z,
        label: "BACK",
        view: StandardView::Back,
    },
    Face {
        normal: Vector3::X,
        u: Vector3::Y,
        v: Vector3::Z,
        label: "RIGHT",
        view: StandardView::Right,
    },
    Face {
        normal: Vector3::NEG_X,
        u: Vector3::NEG_Y,
        v: Vector3::Z,
        label: "LEFT",
        view: StandardView::Left,
    },
];

/// Screen offset (y down) of world direction `w` for the camera's orientation.
fn project(camera: &Camera, w: Vector3) -> egui::Vec2 {
    let p = DVec2::new(w.dot(camera.right()), -w.dot(camera.up()));
    egui::vec2(p.x as f32, p.y as f32)
}

/// Draws the cube in the top-right corner of `viewport` and returns the view
/// the user clicked, if any.
pub fn show(ui: &mut egui::Ui, viewport: egui::Rect, camera: &Camera) -> Option<StandardView> {
    let area = egui::Rect::from_min_size(
        egui::pos2(viewport.right() - AREA - 8.0, viewport.top() + 8.0),
        egui::vec2(AREA, AREA),
    );
    let center = area.center() + egui::vec2(0.0, 4.0);
    let response = ui.interact(area, ui.scope_id().with("view_cube"), egui::Sense::click());
    let back = camera.back();

    // Faces that point towards the camera, farthest first.
    let mut visible: Vec<(&Face, f32, Vec<egui::Pos2>)> = FACES
        .iter()
        .filter_map(|face| {
            let facing = face.normal.dot(back) as f32;
            (facing > 0.02).then(|| {
                let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(a, b)| {
                    center + HALF * project(camera, face.normal + a * face.u + b * face.v)
                });
                (face, facing, corners.to_vec())
            })
        })
        .collect();
    visible.sort_by(|a, b| a.1.total_cmp(&b.1));

    let hovered = response
        .hover_pos()
        .and_then(|p| visible.iter().rev().find(|(_, _, poly)| contains(poly, p)))
        .map(|(face, _, _)| face.view);

    let painter = ui.painter_at(viewport);
    // Soft shadow under the cube.
    painter.circle_filled(
        center + egui::vec2(0.0, 2.0),
        HALF * 1.55,
        egui::Color32::from_black_alpha(40),
    );
    for (face, facing, poly) in &visible {
        let shade = 0.78 + 0.22 * facing;
        let base = if hovered == Some(face.view) {
            egui::Color32::from_rgb(0x7f, 0xb6, 0xea)
        } else {
            egui::Color32::from_rgb(214, 222, 232).lerp_to_gamma(egui::Color32::BLACK, 1.0 - shade)
        };
        painter.add(egui::Shape::convex_polygon(
            poly.clone(),
            base,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(0x55, 0x60, 0x6e)),
        ));
    }
    // Labels after all faces, so no later face covers part of a label.
    for (face, facing, _) in &visible {
        if *facing > 0.25 {
            let mut angle = project(camera, face.u).angle();
            // Keep labels from reading upside down.
            if angle.abs() > std::f32::consts::FRAC_PI_2 {
                angle += std::f32::consts::PI;
            }
            // Fit the label to 70% of the face's width on screen, measured
            // from the laid-out text rather than guessed from its length.
            let face_width = 2.0 * HALF * project(camera, face.u).length();
            let reference = painter
                .layout_no_wrap(
                    face.label.to_owned(),
                    egui::FontId::proportional(10.0),
                    egui::Color32::WHITE,
                )
                .size()
                .x;
            let size = (10.0 * 0.7 * face_width / reference.max(1.0)).clamp(5.0, 11.5);
            let alpha = ((facing - 0.25) / 0.35).clamp(0.0, 1.0);
            let color = egui::Color32::from_rgb(0x2a, 0x33, 0x3e).gamma_multiply(alpha);
            let galley = painter.layout_no_wrap(
                face.label.to_owned(),
                egui::FontId::proportional(size),
                color,
            );
            let face_center = center + HALF * project(camera, face.normal);
            // `pos` is the text's top-left corner; the anchor only picks the
            // pivot, so place the box centred on the face, then rotate.
            let top_left = face_center - galley.size() / 2.0;
            painter.add(
                egui::epaint::TextShape::new(top_left, galley, color)
                    .with_angle_and_anchor(angle, egui::Align2::CENTER_CENTER),
            );
        }
    }

    let response = response.on_hover_text_at_pointer(match hovered {
        Some(view) => format!("{view:?} view"),
        None => "Click a face to look from that side".to_owned(),
    });
    if hovered.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    if response.clicked() { hovered } else { None }
}

/// Point-in-convex-polygon test, for either winding.
fn contains(poly: &[egui::Pos2], p: egui::Pos2) -> bool {
    let mut sign = 0i8;
    for (i, a) in poly.iter().enumerate() {
        let b = poly[(i + 1) % poly.len()];
        let cross = (b - *a).x * (p - *a).y - (b - *a).y * (p - *a).x;
        if cross.abs() < f32::EPSILON {
            continue;
        }
        let side = if cross > 0.0 { 1 } else { -1 };
        if sign == 0 {
            sign = side;
        } else if side != sign {
            return false;
        }
    }
    true
}

/// Draws a small X/Y/Z axis triad in the bottom-left corner of `viewport`.
pub fn axis_triad(ui: &egui::Ui, viewport: egui::Rect, camera: &Camera) {
    let origin = egui::pos2(viewport.left() + 40.0, viewport.bottom() - 40.0);
    let back = camera.back();
    let mut axes = [
        (Vector3::X, "X", egui::Color32::from_rgb(0xe0, 0x55, 0x55)),
        (Vector3::Y, "Y", egui::Color32::from_rgb(0x5f, 0xc0, 0x5f)),
        (Vector3::Z, "Z", egui::Color32::from_rgb(0x5b, 0x8d, 0xe8)),
    ];
    // Draw the axis pointing away from the camera first.
    axes.sort_by(|a, b| a.0.dot(back).total_cmp(&b.0.dot(back)));
    let painter = ui.painter_at(viewport);
    for (dir, label, color) in axes {
        let tip = origin + 24.0 * project(camera, dir);
        painter.line_segment([origin, tip], egui::Stroke::new(2.0, color));
        painter.text(
            origin + 33.0 * project(camera, dir),
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::proportional(11.0),
            color,
        );
    }
    painter.circle_filled(origin, 2.5, theme::TEXT);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_frames_are_right_handed_and_outward() {
        for face in &FACES {
            assert!(
                (face.u.cross(face.v) - face.normal).length() < 1e-12,
                "{}",
                face.label
            );
        }
    }

    #[test]
    fn standard_view_looks_straight_at_its_face() {
        let mut camera = Camera::default();
        for face in &FACES {
            camera.set_view(face.view);
            assert!(face.normal.dot(camera.back()) > 0.999, "{}", face.label);
        }
    }

    #[test]
    fn polygon_containment_handles_both_windings() {
        let square = [
            egui::pos2(0.0, 0.0),
            egui::pos2(1.0, 0.0),
            egui::pos2(1.0, 1.0),
            egui::pos2(0.0, 1.0),
        ];
        let mut reversed = square;
        reversed.reverse();
        for poly in [&square[..], &reversed[..]] {
            assert!(contains(poly, egui::pos2(0.5, 0.5)));
            assert!(!contains(poly, egui::pos2(1.5, 0.5)));
        }
    }
}
