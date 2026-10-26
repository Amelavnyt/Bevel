// SPDX-License-Identifier: GPL-3.0-or-later

//! Axis-aligned bounding boxes in 2D.
//!
//! The 2D counterpart of [`bevel_math::Aabb3`]. It lives here until
//! `bevel-math` grows its own `Aabb2`, at which point this becomes a
//! re-export.

use bevel_math::{Point2, Vector2};

/// An axis-aligned bounding box in 2D.
///
/// An empty box (no points added) has `min > max` on every axis; it is the
/// identity for [`Aabb2::union`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb2 {
    /// Corner with the smallest coordinates.
    pub min: Point2,
    /// Corner with the largest coordinates.
    pub max: Point2,
}

impl Aabb2 {
    /// The empty box.
    pub const EMPTY: Self = Self {
        min: Point2::splat(f64::INFINITY),
        max: Point2::splat(f64::NEG_INFINITY),
    };

    /// A box spanning the two given corners, in any order.
    pub fn new(a: Point2, b: Point2) -> Self {
        Self {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// The smallest box containing all `points`; [`Aabb2::EMPTY`] if there are none.
    pub fn from_points(points: impl IntoIterator<Item = Point2>) -> Self {
        points.into_iter().fold(Self::EMPTY, Self::include)
    }

    /// Returns `true` if the box contains no points.
    pub fn is_empty(&self) -> bool {
        self.min.cmpgt(self.max).any()
    }

    /// The box grown to contain `p`.
    #[must_use]
    pub fn include(self, p: Point2) -> Self {
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

    /// Returns `true` if `p` lies inside the box or on its boundary.
    pub fn contains(&self, p: Point2) -> bool {
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    /// Centre of the box. Meaningless for an empty box.
    pub fn center(&self) -> Point2 {
        0.5 * (self.min + self.max)
    }

    /// Edge lengths of the box; zero for an empty box.
    pub fn size(&self) -> Vector2 {
        if self.is_empty() {
            Vector2::ZERO
        } else {
            self.max - self.min
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_points_spans_all_points() {
        let b = Aabb2::from_points([Point2::new(1.0, -2.0), Point2::new(-1.0, 4.0)]);
        assert_eq!(b.min, Point2::new(-1.0, -2.0));
        assert_eq!(b.max, Point2::new(1.0, 4.0));
        assert!(b.contains(Point2::ZERO));
        assert!(!b.contains(Point2::new(2.0, 0.0)));
        assert!(Aabb2::EMPTY.is_empty());
        assert_eq!(Aabb2::EMPTY.union(b), b);
        assert_eq!(Aabb2::EMPTY.size(), Vector2::ZERO);
    }
}
