// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{Point3, Vector3};

/// An axis-aligned bounding box in 3D.
///
/// An empty box (no points added) has `min > max` on every axis; it is the
/// identity for [`Aabb3::union`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb3 {
    /// Corner with the smallest coordinates.
    pub min: Point3,
    /// Corner with the largest coordinates.
    pub max: Point3,
}

impl Aabb3 {
    /// The empty box.
    pub const EMPTY: Self = Self {
        min: Point3::splat(f64::INFINITY),
        max: Point3::splat(f64::NEG_INFINITY),
    };

    /// A box spanning the two given corners, in any order.
    pub fn new(a: Point3, b: Point3) -> Self {
        Self {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// The smallest box containing all `points`; [`Aabb3::EMPTY`] if there are none.
    pub fn from_points(points: impl IntoIterator<Item = Point3>) -> Self {
        points.into_iter().fold(Self::EMPTY, Self::include)
    }

    /// Returns `true` if the box contains no points.
    pub fn is_empty(&self) -> bool {
        self.min.cmpgt(self.max).any()
    }

    /// The box grown to contain `p`.
    #[must_use]
    pub fn include(self, p: Point3) -> Self {
        Self {
            min: self.min.min(p),
            max: self.max.max(p),
        }
    }

    /// The smallest box containing both boxes.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Centre of the box. Meaningless for an empty box.
    pub fn center(&self) -> Point3 {
        0.5 * (self.min + self.max)
    }

    /// Edge lengths of the box; zero for an empty box.
    pub fn size(&self) -> Vector3 {
        if self.is_empty() {
            Vector3::ZERO
        } else {
            self.max - self.min
        }
    }

    /// Radius of the smallest sphere around [`Aabb3::center`] that contains the box.
    pub fn bounding_radius(&self) -> f64 {
        0.5 * self.size().length()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_box_is_identity_for_union() {
        let b = Aabb3::new(Point3::ZERO, Point3::ONE);
        assert!(Aabb3::EMPTY.is_empty());
        assert_eq!(Aabb3::EMPTY.union(b), b);
        assert_eq!(Aabb3::EMPTY.size(), Vector3::ZERO);
    }

    #[test]
    fn from_points_spans_all_points() {
        let b = Aabb3::from_points([Point3::new(1.0, -2.0, 3.0), Point3::new(-1.0, 4.0, 0.0)]);
        assert_eq!(b.min, Point3::new(-1.0, -2.0, 0.0));
        assert_eq!(b.max, Point3::new(1.0, 4.0, 3.0));
        assert_eq!(b.center(), Point3::new(0.0, 1.0, 1.5));
    }
}
