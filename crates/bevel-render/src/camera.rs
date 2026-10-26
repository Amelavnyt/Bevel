// SPDX-License-Identifier: GPL-3.0-or-later

use bevel_math::dcamera::rh::{proj::directx, view};
use bevel_math::{Aabb3, DMat4, DVec2, Point3, Vector3};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

/// How the camera projects the scene.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Projection {
    /// Perspective projection with the camera's vertical field of view.
    Perspective,
    /// Orthographic projection; the view height matches what the perspective
    /// view shows at the target distance, so switching keeps the framing.
    Orthographic,
}

/// A named viewing direction, as offered by the view cube.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandardView {
    /// Looking down onto the XY plane, X to the right and Y up the screen.
    Top,
    /// Looking up at the XY plane from below.
    Bottom,
    /// Looking along +Y, X to the right and Z up.
    Front,
    /// Looking along -Y.
    Back,
    /// Looking along +X, from the -X side.
    Left,
    /// Looking along -X, Y to the right and Z up.
    Right,
    /// Isometric view from the front, right and top.
    Home,
}

impl StandardView {
    /// Every standard view.
    pub const ALL: [Self; 7] = [
        Self::Top,
        Self::Bottom,
        Self::Front,
        Self::Back,
        Self::Left,
        Self::Right,
        Self::Home,
    ];

    /// Camera `(yaw, pitch)` for this view; see [`Camera`].
    pub fn yaw_pitch(self) -> (f64, f64) {
        use std::f64::consts::PI;
        match self {
            Self::Top => (-FRAC_PI_2, MAX_PITCH),
            Self::Bottom => (-FRAC_PI_2, -MAX_PITCH),
            Self::Front => (-FRAC_PI_2, 0.0),
            Self::Back => (FRAC_PI_2, 0.0),
            Self::Left => (PI, 0.0),
            Self::Right => (0.0, 0.0),
            // The direction (1, -1, 1): equal angles to all three axes.
            Self::Home => (-FRAC_PI_4, (1.0 / 3f64.sqrt()).asin()),
        }
    }
}

/// A turntable camera orbiting a target point in a Z-up world.
///
/// The eye sits at `target + distance * dir(yaw, pitch)`, where `yaw` is
/// measured in the XY plane from +X and `pitch` is the elevation above it.
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    /// Point the camera looks at and orbits around.
    pub target: Point3,
    /// Distance from the eye to the target, in millimetres.
    pub distance: f64,
    /// Azimuth of the eye around the target, in radians.
    pub yaw: f64,
    /// Elevation of the eye above the XY plane through the target, in radians.
    pub pitch: f64,
    /// Vertical field of view, in radians.
    pub fov_y: f64,
    /// Perspective or orthographic projection.
    pub projection: Projection,
}

/// Closest the eye may get to the target, in millimetres.
const MIN_DISTANCE: f64 = 1e-3;
/// Farthest the eye may get from the target, in millimetres.
const MAX_DISTANCE: f64 = 1e6;
/// Keeps the eye off the poles, where the turntable's up vector degenerates.
const MAX_PITCH: f64 = FRAC_PI_2 - 1e-3;
/// Orbit speed in radians per pixel of drag.
const ORBIT_RADIANS_PER_PIXEL: f64 = 0.008;

impl Default for Camera {
    fn default() -> Self {
        let (yaw, pitch) = StandardView::Home.yaw_pitch();
        Self {
            target: Point3::ZERO,
            distance: 150.0,
            yaw,
            pitch,
            fov_y: 40f64.to_radians(),
            projection: Projection::Perspective,
        }
    }
}

impl Camera {
    /// Points the camera along a standard view, keeping target and distance.
    pub fn set_view(&mut self, view: StandardView) {
        (self.yaw, self.pitch) = view.yaw_pitch();
    }

    /// The camera a fraction `t` (0 to 1) of the way from `from` to `to`, for
    /// animated transitions. Yaw turns the short way round, distance changes
    /// geometrically so zooming feels even, and the projection switches at
    /// the end.
    pub fn interpolate(from: &Self, to: &Self, t: f64) -> Self {
        use std::f64::consts::{PI, TAU};
        let t = t.clamp(0.0, 1.0);
        let yaw_delta = (to.yaw - from.yaw + PI).rem_euclid(TAU) - PI;
        Self {
            target: from.target.lerp(to.target, t),
            distance: from.distance * (to.distance / from.distance).powf(t),
            yaw: from.yaw + t * yaw_delta,
            pitch: from.pitch + t * (to.pitch - from.pitch),
            fov_y: from.fov_y + t * (to.fov_y - from.fov_y),
            projection: if t < 1.0 {
                from.projection
            } else {
                to.projection
            },
        }
    }

    /// Unit vector from the target towards the eye.
    pub fn back(&self) -> Vector3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vector3::new(cp * cy, cp * sy, sp)
    }

    /// Unit vector pointing to the right of the screen.
    pub fn right(&self) -> Vector3 {
        Vector3::Z.cross(self.back()).normalize()
    }

    /// Unit vector pointing up the screen.
    pub fn up(&self) -> Vector3 {
        self.back().cross(self.right())
    }

    /// Eye position in world space.
    pub fn eye(&self) -> Point3 {
        self.target + self.distance * self.back()
    }

    /// Half the height of the view at the target distance, in millimetres.
    pub fn half_height(&self) -> f64 {
        self.distance * (0.5 * self.fov_y).tan()
    }

    /// Size in millimetres, at the target distance, of one pixel of a
    /// viewport that is `viewport_height` pixels tall.
    pub fn world_per_pixel(&self, viewport_height: f64) -> f64 {
        2.0 * self.half_height() / viewport_height.max(1.0)
    }

    /// World-to-view transform.
    pub fn view(&self) -> DMat4 {
        view::look_at_mat4(self.eye(), self.target, Vector3::Z)
    }

    /// View-to-clip transform with reverse-Z depth.
    pub fn projection_matrix(&self, aspect: f64) -> DMat4 {
        let aspect = aspect.max(1e-6);
        match self.projection {
            Projection::Perspective => {
                let near = (self.distance * 1e-3).max(1e-4);
                directx::perspective_infinite_reverse(self.fov_y, aspect, near)
            }
            Projection::Orthographic => {
                let h = self.half_height();
                let w = h * aspect;
                // Depth range relative to the eye; swapping near and far in
                // the call gives reverse-Z. Negative near keeps geometry
                // behind the eye visible, which an orthographic view expects.
                let reach = 20.0 * self.distance;
                directx::orthographic(-w, w, -h, h, reach, -reach)
            }
        }
    }

    /// World-to-clip transform.
    pub fn view_projection(&self, aspect: f64) -> DMat4 {
        self.projection_matrix(aspect) * self.view()
    }

    /// Rotates the eye around the target by a screen drag of `delta` pixels.
    pub fn orbit(&mut self, delta: DVec2) {
        self.yaw = (self.yaw - delta.x * ORBIT_RADIANS_PER_PIXEL).rem_euclid(std::f64::consts::TAU);
        self.pitch = (self.pitch + delta.y * ORBIT_RADIANS_PER_PIXEL).clamp(-MAX_PITCH, MAX_PITCH);
    }

    /// Moves the camera so the scene follows a screen drag of `delta`
    /// pixels in a viewport `viewport_height` pixels tall.
    pub fn pan(&mut self, delta: DVec2, viewport_height: f64) {
        let s = self.world_per_pixel(viewport_height);
        self.target += s * (-delta.x * self.right() + delta.y * self.up());
    }

    /// Scales the distance to the target by `factor` (below 1 zooms in)
    /// while keeping the point under `ndc` (normalized device coordinates,
    /// y up) fixed on screen.
    pub fn zoom_about(&mut self, factor: f64, ndc: DVec2, aspect: f64) {
        let factor = factor.clamp(MIN_DISTANCE / self.distance, MAX_DISTANCE / self.distance);
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        // The point under the cursor on the plane through the target facing the
        // camera. Scaling the camera about it (a homothety) keeps it in place.
        let h = self.half_height();
        let pivot = self.target + ndc.x * h * aspect * self.right() + ndc.y * h * self.up();
        self.target = pivot + factor * (self.target - pivot);
        self.distance *= factor;
    }

    /// Frames `bounds` in a viewport with the given aspect ratio, keeping
    /// the viewing direction. Does nothing for an empty box.
    pub fn fit(&mut self, bounds: &Aabb3, aspect: f64) {
        if bounds.is_empty() {
            return;
        }
        let radius = bounds.bounding_radius().max(1.0);
        let half_fov_y = 0.5 * self.fov_y;
        let half_fov_x = (half_fov_y.tan() * aspect.max(1e-6)).atan();
        let half_fov = half_fov_y.min(half_fov_x);
        self.target = bounds.center();
        // A sphere of this radius just touches the narrower frustum edges;
        // the margin leaves some room around the model.
        self.distance = (1.5 * radius / half_fov.sin()).clamp(MIN_DISTANCE, MAX_DISTANCE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevel_math::DVec4;

    fn ndc_of(camera: &Camera, p: Point3, aspect: f64) -> DVec2 {
        let c = camera.view_projection(aspect) * DVec4::new(p.x, p.y, p.z, 1.0);
        DVec2::new(c.x / c.w, c.y / c.w)
    }

    fn close(a: DVec2, b: DVec2) -> bool {
        (a - b).length() < 1e-9
    }

    #[test]
    fn target_projects_to_screen_centre() {
        let mut camera = Camera {
            target: Point3::new(5.0, -3.0, 2.0),
            ..Camera::default()
        };
        for projection in [Projection::Perspective, Projection::Orthographic] {
            camera.projection = projection;
            assert!(close(ndc_of(&camera, camera.target, 1.5), DVec2::ZERO));
        }
    }

    #[test]
    fn camera_basis_is_orthonormal() {
        let camera = Camera::default();
        let (b, r, u) = (camera.back(), camera.right(), camera.up());
        assert!((b.length() - 1.0).abs() < 1e-12);
        assert!((r.length() - 1.0).abs() < 1e-12);
        assert!(b.dot(r).abs() < 1e-12 && r.dot(u).abs() < 1e-12);
        assert!(
            u.z > 0.0,
            "up vector should point up for a camera above the grid"
        );
    }

    #[test]
    fn standard_views_put_the_expected_axes_on_screen() {
        let close3 = |a: Vector3, b: Vector3| (a - b).length() < 1e-2;
        let mut camera = Camera::default();
        for (view, right, up) in [
            (StandardView::Front, Vector3::X, Vector3::Z),
            (StandardView::Right, Vector3::Y, Vector3::Z),
            (StandardView::Back, -Vector3::X, Vector3::Z),
            (StandardView::Left, -Vector3::Y, Vector3::Z),
            (StandardView::Top, Vector3::X, Vector3::Y),
            (StandardView::Bottom, Vector3::X, -Vector3::Y),
        ] {
            camera.set_view(view);
            assert!(
                close3(camera.right(), right),
                "{view:?}: right {}",
                camera.right()
            );
            assert!(close3(camera.up(), up), "{view:?}: up {}", camera.up());
        }
        camera.set_view(StandardView::Home);
        assert!(close3(
            camera.back(),
            Vector3::new(1.0, -1.0, 1.0).normalize()
        ));
    }

    #[test]
    fn interpolation_hits_both_ends_and_turns_the_short_way() {
        let from = Camera {
            yaw: 3.0,
            distance: 10.0,
            ..Camera::default()
        };
        let to = Camera {
            yaw: -3.0,
            distance: 1000.0,
            target: Point3::new(1.0, 2.0, 3.0),
            ..Camera::default()
        };
        assert_eq!(Camera::interpolate(&from, &to, 0.0), from);
        let end = Camera::interpolate(&from, &to, 1.0);
        assert!(
            (end.yaw.sin() - to.yaw.sin()).abs() < 1e-12
                && (end.yaw.cos() - to.yaw.cos()).abs() < 1e-12
        );
        assert!((end.distance - 1000.0).abs() < 1e-9);
        let mid = Camera::interpolate(&from, &to, 0.5);
        // 3 and -3 radians are 0.28 apart across +-pi, not 6 apart through 0.
        assert!(mid.yaw.cos() < -0.98, "went the long way: yaw {}", mid.yaw);
        assert!((mid.distance - 100.0).abs() < 1e-9);
    }

    #[test]
    fn orbit_keeps_distance_and_clamps_pitch() {
        let mut camera = Camera::default();
        let before = camera.eye().distance(camera.target);
        camera.orbit(DVec2::new(123.0, 10_000.0));
        assert!((camera.eye().distance(camera.target) - before).abs() < 1e-9);
        assert!(camera.pitch <= MAX_PITCH);
    }

    #[test]
    fn zoom_keeps_cursor_point_fixed() {
        let aspect = 1.6;
        let ndc = DVec2::new(0.4, -0.25);
        for projection in [Projection::Perspective, Projection::Orthographic] {
            let mut camera = Camera {
                projection,
                ..Camera::default()
            };
            let h = camera.half_height();
            let pivot =
                camera.target + ndc.x * h * aspect * camera.right() + ndc.y * h * camera.up();
            assert!(close(ndc_of(&camera, pivot, aspect), ndc));
            camera.zoom_about(0.5, ndc, aspect);
            assert!(close(ndc_of(&camera, pivot, aspect), ndc));
            assert!((camera.distance - 75.0).abs() < 1e-9);
        }
    }

    #[test]
    fn pan_moves_scene_with_the_cursor() {
        let mut camera = Camera::default();
        let height = 800.0;
        let p = camera.target;
        let before = ndc_of(&camera, p, 1.0);
        camera.pan(DVec2::new(40.0, 0.0), height);
        let after = ndc_of(&camera, p, 1.0);
        // 40 px to the right of an 800 px tall square viewport is 0.1 in NDC.
        assert!(close(after - before, DVec2::new(0.1, 0.0)));
    }

    #[test]
    fn fit_puts_every_corner_on_screen() {
        let bounds = Aabb3::new(
            Point3::new(-10.0, -10.0, 0.0),
            Point3::new(30.0, 10.0, 20.0),
        );
        let aspect = 0.5;
        let mut camera = Camera::default();
        camera.fit(&bounds, aspect);
        for i in 0..8 {
            let corner = Point3::new(
                if i & 1 == 0 {
                    bounds.min.x
                } else {
                    bounds.max.x
                },
                if i & 2 == 0 {
                    bounds.min.y
                } else {
                    bounds.max.y
                },
                if i & 4 == 0 {
                    bounds.min.z
                } else {
                    bounds.max.z
                },
            );
            let ndc = ndc_of(&camera, corner, aspect);
            assert!(ndc.abs().max_element() < 1.0, "corner {corner} at {ndc}");
        }
    }

    #[test]
    fn reverse_z_puts_nearer_points_at_larger_depth() {
        let camera = Camera::default();
        let vp = camera.view_projection(1.0);
        let depth = |p: Point3| {
            let c = vp * DVec4::new(p.x, p.y, p.z, 1.0);
            c.z / c.w
        };
        let near = camera.target + 10.0 * camera.back();
        let far = camera.target - 10.0 * camera.back();
        assert!(depth(near) > depth(camera.target));
        assert!(depth(camera.target) > depth(far));
    }
}
