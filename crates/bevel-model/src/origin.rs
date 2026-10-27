// SPDX-License-Identifier: GPL-3.0-or-later

//! The origin every part has: a point, three planes and three axes.

use bevel_math::{Frame3, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// One of the three origin planes.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum OriginPlane {
    /// The XY plane; its normal is +Z.
    #[default]
    Xy,
    /// The XZ plane; its normal is -Y, so X and Z read as on a front view.
    Xz,
    /// The YZ plane; its normal is +X.
    Yz,
}

impl OriginPlane {
    /// All three, in browser order.
    pub const ALL: [Self; 3] = [Self::Xy, Self::Xz, Self::Yz];

    /// Display name: "XY", "XZ", "YZ".
    pub fn name(self) -> &'static str {
        match self {
            Self::Xy => "XY",
            Self::Xz => "XZ",
            Self::Yz => "YZ",
        }
    }

    /// The plane's frame at the world origin: its x and y axes span the
    /// plane, z is the normal.
    pub fn frame(self) -> Frame3 {
        let (x, y) = match self {
            Self::Xy => (Vector3::X, Vector3::Y),
            Self::Xz => (Vector3::X, Vector3::Z),
            Self::Yz => (Vector3::Y, Vector3::Z),
        };
        Frame3 {
            origin: Point3::ZERO,
            x,
            y,
            z: x.cross(y),
        }
    }
}

/// One of the three origin axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OriginAxis {
    /// The X axis.
    X,
    /// The Y axis.
    Y,
    /// The Z axis.
    Z,
}

impl OriginAxis {
    /// All three.
    pub const ALL: [Self; 3] = [Self::X, Self::Y, Self::Z];

    /// Display name: "X", "Y", "Z".
    pub fn name(self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
        }
    }

    /// The unit direction.
    pub fn direction(self) -> Vector3 {
        match self {
            Self::X => Vector3::X,
            Self::Y => Vector3::Y,
            Self::Z => Vector3::Z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_frames_are_valid() {
        for p in OriginPlane::ALL {
            assert!(p.frame().is_valid(), "{p:?}");
        }
        assert_eq!(OriginPlane::Xz.frame().z, -Vector3::Y);
        assert_eq!(OriginPlane::Yz.frame().z, Vector3::X);
    }
}
