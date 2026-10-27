// SPDX-License-Identifier: GPL-3.0-or-later

use serde::{Deserialize, Serialize};

use crate::{MathError, Point2, Point3, Vector2, Vector3};

/// An axis-aligned bounding box in 3D.
///
/// An empty box (no points added) has `min > max` on every axis; it is the
/// identity for [`Aabb3::union`].
///
/// Serialized as `{ "min": [x, y, z], "max": [x, y, z] }`; deserialization
/// rejects NaN coordinates. Formats without infinities (JSON) cannot hold
/// the empty box.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "Aabb3Raw")]
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

    /// Returns `true` if `p` lies inside the box or on its surface.
    pub fn contains(&self, p: Point3) -> bool {
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    /// Returns `true` if the two boxes share at least one point (touching counts).
    /// An empty box intersects nothing.
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.cmple(other.max).all() && other.min.cmple(self.max).all()
    }

    /// Radius of the smallest sphere around [`Aabb3::center`] that contains the box.
    pub fn bounding_radius(&self) -> f64 {
        0.5 * self.size().length()
    }
}

/// An axis-aligned bounding box in 2D.
///
/// An empty box (no points added) has `min > max` on both axes; it is the
/// identity for [`Aabb2::union`].
///
/// Serialized as `{ "min": [x, y], "max": [x, y] }`; deserialization
/// rejects NaN coordinates. Formats without infinities (JSON) cannot hold
/// the empty box.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "Aabb2Raw")]
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

    /// Returns `true` if `p` lies inside the box or on its boundary.
    pub fn contains(&self, p: Point2) -> bool {
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    /// Returns `true` if the two boxes share at least one point (touching counts).
    /// An empty box intersects nothing.
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.cmple(other.max).all() && other.min.cmple(self.max).all()
    }
}

/// The serialized form of an [`Aabb3`], checked before it becomes one.
#[derive(Deserialize)]
struct Aabb3Raw {
    min: Point3,
    max: Point3,
}

impl TryFrom<Aabb3Raw> for Aabb3 {
    type Error = MathError;

    fn try_from(raw: Aabb3Raw) -> Result<Self, MathError> {
        if raw.min.is_nan() || raw.max.is_nan() {
            return Err(MathError::InvalidInput("a box corner is NaN"));
        }
        Ok(Self {
            min: raw.min,
            max: raw.max,
        })
    }
}

/// The serialized form of an [`Aabb2`], checked before it becomes one.
#[derive(Deserialize)]
struct Aabb2Raw {
    min: Point2,
    max: Point2,
}

impl TryFrom<Aabb2Raw> for Aabb2 {
    type Error = MathError;

    fn try_from(raw: Aabb2Raw) -> Result<Self, MathError> {
        if raw.min.is_nan() || raw.max.is_nan() {
            return Err(MathError::InvalidInput("a box corner is NaN"));
        }
        Ok(Self {
            min: raw.min,
            max: raw.max,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boxes_round_trip_through_serde() {
        let b = Aabb3::new(Point3::new(-1.0, 0.5, 2.0), Point3::new(3.0, 4.0, 5.0));
        let json = serde_json::to_string(&b).unwrap();
        assert_eq!(json, r#"{"min":[-1.0,0.5,2.0],"max":[3.0,4.0,5.0]}"#);
        assert_eq!(serde_json::from_str::<Aabb3>(&json).unwrap(), b);
        let c = Aabb2::new(Point2::new(-1.0, 0.5), Point2::new(3.0, 4.0));
        let json = serde_json::to_string(&c).unwrap();
        assert_eq!(serde_json::from_str::<Aabb2>(&json).unwrap(), c);
        assert!(serde_json::from_str::<Aabb2>(r#"{"min":[0.0],"max":[1.0,1.0]}"#).is_err());
    }

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

    #[test]
    fn aabb3_contains_and_intersects() {
        let b = Aabb3::new(Point3::ZERO, Point3::ONE);
        assert!(b.contains(Point3::splat(0.5)) && b.contains(Point3::ONE));
        assert!(!b.contains(Point3::new(0.5, 0.5, 1.5)));
        let touching = Aabb3::new(Point3::ONE, Point3::splat(2.0));
        assert!(b.intersects(&touching));
        assert!(!b.intersects(&Aabb3::new(Point3::splat(1.5), Point3::splat(2.0))));
        assert!(!Aabb3::EMPTY.intersects(&b));
        assert!(!Aabb3::EMPTY.contains(Point3::ZERO));
    }

    #[test]
    fn aabb2_mirrors_aabb3() {
        let b = Aabb2::from_points([Point2::new(1.0, -2.0), Point2::new(-1.0, 4.0)]);
        assert_eq!(b.min, Point2::new(-1.0, -2.0));
        assert_eq!(b.max, Point2::new(1.0, 4.0));
        assert_eq!(b.center(), Point2::new(0.0, 1.0));
        assert_eq!(b.size(), Vector2::new(2.0, 6.0));
        assert!(Aabb2::EMPTY.is_empty());
        assert_eq!(Aabb2::EMPTY.size(), Vector2::ZERO);
        assert_eq!(Aabb2::EMPTY.union(b), b);
        assert!(b.contains(Point2::ZERO) && !b.contains(Point2::new(2.0, 0.0)));
        assert!(b.intersects(&Aabb2::new(Point2::new(1.0, 4.0), Point2::new(5.0, 5.0))));
        assert!(!b.intersects(&Aabb2::new(Point2::new(2.0, 4.0), Point2::new(5.0, 5.0))));
        assert!(!Aabb2::EMPTY.intersects(&b));
    }
}
