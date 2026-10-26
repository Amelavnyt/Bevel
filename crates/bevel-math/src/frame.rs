// SPDX-License-Identifier: GPL-3.0-or-later

//! Orthonormal coordinate frames in 3D.

use crate::{DMat3, DMat4, MathError, Point3, Vector3};

/// Vectors shorter than this have no usable direction.
const MIN_LENGTH: f64 = 1e-12;

/// A right-handed orthonormal coordinate frame: an origin and three
/// mutually perpendicular unit axes with `x × y = z`.
///
/// "Local" coordinates are measured along the frame's axes from its origin;
/// "world" coordinates are model-space coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame3 {
    /// Origin of the frame in world coordinates.
    pub origin: Point3,
    /// Unit x axis.
    pub x: Vector3,
    /// Unit y axis.
    pub y: Vector3,
    /// Unit z axis (the normal for a plane frame).
    pub z: Vector3,
}

impl Frame3 {
    /// The world frame: origin at zero, axes along X, Y, Z.
    pub const WORLD: Self = Self {
        origin: Point3::ZERO,
        x: Vector3::X,
        y: Vector3::Y,
        z: Vector3::Z,
    };

    /// A frame at `origin` whose z axis is `normal` (which need not be unit length).
    ///
    /// The x axis is chosen deterministically: take the world axis (X, Y or
    /// Z) along which `normal` has the smallest absolute component (the
    /// earliest of X, Y, Z on ties), and set `x = normalize(axis × normal)`.
    /// Then `y = z × x`.
    ///
    /// # Errors
    /// [`MathError::DegenerateVector`] if `normal` is zero, too short or not finite.
    pub fn from_normal(origin: Point3, normal: Vector3) -> Result<Self, MathError> {
        let z = unit(normal)?;
        let a = z.abs();
        let axis = if a.x <= a.y && a.x <= a.z {
            Vector3::X
        } else if a.y <= a.z {
            Vector3::Y
        } else {
            Vector3::Z
        };
        let x = unit(axis.cross(z))?;
        let y = z.cross(x);
        Ok(Self { origin, x, y, z })
    }

    /// A frame at `origin` with x axis along `x` and the y axis in the plane
    /// of `x` and `y_hint`, on the side of `y_hint` (Gram–Schmidt).
    /// `z = x × y`.
    ///
    /// # Errors
    /// [`MathError::DegenerateVector`] if either vector is zero or not finite,
    /// or if `y_hint` is parallel to `x`.
    pub fn from_axes(origin: Point3, x: Vector3, y_hint: Vector3) -> Result<Self, MathError> {
        let x = unit(x)?;
        let hint = unit(y_hint)?;
        let y = unit(hint - x * x.dot(hint))?;
        let z = x.cross(y);
        Ok(Self { origin, x, y, z })
    }

    /// World coordinates of the point with local coordinates `local`.
    pub fn to_world(&self, local: Point3) -> Point3 {
        self.origin + self.vector_to_world(local)
    }

    /// Local coordinates of the world point `p`.
    pub fn to_local(&self, p: Point3) -> Point3 {
        self.vector_to_local(p - self.origin)
    }

    /// Rotates a local direction or displacement into world coordinates.
    pub fn vector_to_world(&self, v: Vector3) -> Vector3 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    /// Rotates a world direction or displacement into local coordinates.
    pub fn vector_to_local(&self, v: Vector3) -> Vector3 {
        Vector3::new(v.dot(self.x), v.dot(self.y), v.dot(self.z))
    }

    /// The matrix taking local coordinates to world coordinates.
    pub fn matrix(&self) -> DMat4 {
        DMat4::from_cols(
            self.x.extend(0.0),
            self.y.extend(0.0),
            self.z.extend(0.0),
            self.origin.extend(1.0),
        )
    }

    /// The matrix taking world coordinates to local coordinates.
    pub fn inverse_matrix(&self) -> DMat4 {
        // The rotation part is orthonormal, so its inverse is its transpose.
        let rot = DMat3::from_cols(self.x, self.y, self.z).transpose();
        let t = -(rot * self.origin);
        DMat4::from_cols(
            rot.x_axis.extend(0.0),
            rot.y_axis.extend(0.0),
            rot.z_axis.extend(0.0),
            t.extend(1.0),
        )
    }
}

/// `v` scaled to unit length, or an error if it has no usable direction.
fn unit(v: Vector3) -> Result<Vector3, MathError> {
    let len = v.length();
    if v.is_finite() && len > MIN_LENGTH {
        Ok(v / len)
    } else {
        Err(MathError::DegenerateVector)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn assert_close(a: Vector3, b: Vector3, tol: f64) {
        assert!((a - b).length() <= tol, "{a:?} vs {b:?}");
    }

    fn assert_orthonormal_right_handed(f: &Frame3) {
        assert!((f.x.length() - 1.0).abs() < 1e-12);
        assert!((f.y.length() - 1.0).abs() < 1e-12);
        assert!((f.z.length() - 1.0).abs() < 1e-12);
        assert!(f.x.dot(f.y).abs() < 1e-12);
        assert!(f.x.dot(f.z).abs() < 1e-12);
        assert!(f.y.dot(f.z).abs() < 1e-12);
        assert_close(f.x.cross(f.y), f.z, 1e-12);
    }

    #[test]
    fn world_is_identity() {
        let p = Point3::new(1.0, -2.0, 3.0);
        assert_close(Frame3::WORLD.to_local(p), p, 0.0);
        assert_close(Frame3::WORLD.to_world(p), p, 0.0);
        assert_eq!(Frame3::WORLD.matrix(), DMat4::IDENTITY);
    }

    #[test]
    fn from_normal_is_deterministic_and_right_handed() {
        let f = Frame3::from_normal(Point3::ZERO, Vector3::new(0.0, 0.0, 5.0)).unwrap();
        assert_orthonormal_right_handed(&f);
        assert_close(f.z, Vector3::Z, 1e-15);
        // Least aligned world axis is X (earliest tie), so x = X × Z normalized = -Y.
        assert_close(f.x, -Vector3::Y, 1e-15);
        assert_close(f.y, Vector3::X, 1e-15);
        let g = Frame3::from_normal(Point3::ZERO, Vector3::new(0.0, 0.0, 5.0)).unwrap();
        assert_eq!(f, g);
    }

    #[test]
    fn from_normal_rejects_degenerate_input() {
        assert_eq!(
            Frame3::from_normal(Point3::ZERO, Vector3::ZERO),
            Err(MathError::DegenerateVector)
        );
        assert_eq!(
            Frame3::from_normal(Point3::ZERO, Vector3::new(f64::NAN, 0.0, 1.0)),
            Err(MathError::DegenerateVector)
        );
    }

    #[test]
    fn from_axes_orthogonalizes_the_hint() {
        let f = Frame3::from_axes(
            Point3::new(1.0, 2.0, 3.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 3.0, 0.0),
        )
        .unwrap();
        assert_orthonormal_right_handed(&f);
        assert_close(f.x, Vector3::X, 1e-15);
        assert_close(f.y, Vector3::Y, 1e-15);
        assert_close(f.z, Vector3::Z, 1e-15);
        assert_eq!(
            Frame3::from_axes(Point3::ZERO, Vector3::X, Vector3::new(-2.0, 0.0, 0.0)),
            Err(MathError::DegenerateVector)
        );
    }

    #[test]
    fn matrices_agree_with_methods() {
        let f =
            Frame3::from_normal(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 2.0, 2.0)).unwrap();
        let p = Point3::new(-4.0, 5.0, 0.5);
        assert_close(f.matrix().transform_point3(p), f.to_world(p), 1e-12);
        assert_close(f.inverse_matrix().transform_point3(p), f.to_local(p), 1e-12);
        let id = f.matrix() * f.inverse_matrix();
        assert!(id.abs_diff_eq(DMat4::IDENTITY, 1e-12));
    }

    proptest! {
        #[test]
        fn round_trip(
            n in (-10.0..10.0f64, -10.0..10.0f64, -10.0..10.0f64),
            o in (-100.0..100.0f64, -100.0..100.0f64, -100.0..100.0f64),
            p in (-100.0..100.0f64, -100.0..100.0f64, -100.0..100.0f64),
        ) {
            let normal = Vector3::new(n.0, n.1, n.2);
            prop_assume!(normal.length() > 1e-3);
            let f = Frame3::from_normal(Point3::new(o.0, o.1, o.2), normal).unwrap();
            assert_orthonormal_right_handed(&f);
            let p = Point3::new(p.0, p.1, p.2);
            prop_assert!((f.to_world(f.to_local(p)) - p).length() <= 1e-12);
            prop_assert!((f.to_local(f.to_world(p)) - p).length() <= 1e-12);
        }
    }
}
