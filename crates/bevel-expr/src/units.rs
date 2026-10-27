// SPDX-License-Identifier: GPL-3.0-or-later

//! Dimensions, quantities and unit conversion.

use std::fmt;

use serde::{Deserialize, Serialize};

/// Physical dimension as exponents of length and angle (area = length^2).
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
pub struct Dim {
    /// Exponent of length.
    pub length: i8,
    /// Exponent of angle.
    pub angle: i8,
}

impl Dim {
    /// No dimension.
    pub const UNITLESS: Dim = Dim {
        length: 0,
        angle: 0,
    };
    /// Length.
    pub const LENGTH: Dim = Dim {
        length: 1,
        angle: 0,
    };
    /// Area (length squared).
    pub const AREA: Dim = Dim {
        length: 2,
        angle: 0,
    };
    /// Volume (length cubed).
    pub const VOLUME: Dim = Dim {
        length: 3,
        angle: 0,
    };
    /// Angle.
    pub const ANGLE: Dim = Dim {
        length: 0,
        angle: 1,
    };
}

fn push_power(parts: &mut Vec<String>, name: &str, exp: i8) {
    match exp {
        0 => {}
        1 => parts.push(name.to_owned()),
        e => parts.push(format!("{name}^{e}")),
    }
}

impl fmt::Display for Dim {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let named = [
            (Dim::UNITLESS, "unitless"),
            (Dim::LENGTH, "length"),
            (Dim::AREA, "area"),
            (Dim::VOLUME, "volume"),
            (Dim::ANGLE, "angle"),
        ];
        if let Some((_, n)) = named.iter().find(|(d, _)| d == self) {
            return f.write_str(n);
        }
        let mut parts = Vec::new();
        push_power(&mut parts, "length", self.length);
        push_power(&mut parts, "angle", self.angle);
        f.write_str(&parts.join("·"))
    }
}

/// A value in internal units (mm, rad) with its dimension.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Quantity {
    /// Value in internal units.
    pub value: f64,
    /// Dimension of the value.
    pub dim: Dim,
}

impl Quantity {
    /// A unitless quantity.
    #[must_use]
    pub const fn unitless(value: f64) -> Self {
        Self {
            value,
            dim: Dim::UNITLESS,
        }
    }

    /// A length in millimetres.
    #[must_use]
    pub const fn length(mm: f64) -> Self {
        Self {
            value: mm,
            dim: Dim::LENGTH,
        }
    }

    /// An angle in radians.
    #[must_use]
    pub const fn angle(rad: f64) -> Self {
        Self {
            value: rad,
            dim: Dim::ANGLE,
        }
    }
}

/// A unit of length.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LengthUnit {
    /// Millimetre.
    #[default]
    Millimetre,
    /// Centimetre.
    Centimetre,
    /// Metre.
    Metre,
    /// Micrometre.
    Micrometre,
    /// Inch (25.4 mm exactly).
    Inch,
    /// Foot (304.8 mm exactly).
    Foot,
}

impl LengthUnit {
    /// Short symbol, e.g. `mm`.
    #[must_use]
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Millimetre => "mm",
            Self::Centimetre => "cm",
            Self::Metre => "m",
            Self::Micrometre => "um",
            Self::Inch => "in",
            Self::Foot => "ft",
        }
    }

    /// Looks a unit up by its symbol.
    #[must_use]
    pub fn from_symbol(s: &str) -> Option<Self> {
        Some(match s {
            "mm" => Self::Millimetre,
            "cm" => Self::Centimetre,
            "m" => Self::Metre,
            "um" => Self::Micrometre,
            "in" => Self::Inch,
            "ft" => Self::Foot,
            _ => return None,
        })
    }

    fn mm_per_unit(self) -> f64 {
        match self {
            Self::Millimetre => 1.0,
            Self::Centimetre => 10.0,
            Self::Metre => 1000.0,
            Self::Micrometre => 0.001,
            Self::Inch => 25.4,
            Self::Foot => 304.8,
        }
    }

    /// Converts a value in this unit to millimetres.
    #[must_use]
    pub fn to_internal(self, v: f64) -> f64 {
        v * self.mm_per_unit()
    }

    /// Converts millimetres to this unit.
    #[must_use]
    pub fn from_internal(self, mm: f64) -> f64 {
        mm / self.mm_per_unit()
    }
}

/// A unit of angle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum AngleUnit {
    /// Degree.
    #[default]
    Degree,
    /// Radian.
    Radian,
}

impl AngleUnit {
    /// Short symbol, e.g. `deg`.
    #[must_use]
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Degree => "deg",
            Self::Radian => "rad",
        }
    }

    /// Looks a unit up by its symbol.
    #[must_use]
    pub fn from_symbol(s: &str) -> Option<Self> {
        match s {
            "deg" => Some(Self::Degree),
            "rad" => Some(Self::Radian),
            _ => None,
        }
    }

    /// Converts a value in this unit to radians.
    #[must_use]
    pub fn to_internal(self, v: f64) -> f64 {
        match self {
            Self::Degree => v.to_radians(),
            Self::Radian => v,
        }
    }

    /// Converts radians to this unit.
    #[must_use]
    pub fn from_internal(self, rad: f64) -> f64 {
        match self {
            Self::Degree => rad.to_degrees(),
            Self::Radian => rad,
        }
    }
}

/// Document default units: bare numbers in a length field mean this unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Units {
    /// Default length unit.
    pub length: LengthUnit,
    /// Default angle unit.
    pub angle: AngleUnit,
}

/// Formats a quantity in the default units, e.g. `40.00 mm`, `30.0 deg`,
/// `1600.00 mm^2`. Unitless values carry no unit; negative zero prints
/// without a sign.
#[must_use]
pub fn format_quantity(q: Quantity, units: Units, decimals: usize) -> String {
    let l = units.length.to_internal(1.0);
    let a = units.angle.to_internal(1.0);
    let scale = l.powi(i32::from(q.dim.length)) * a.powi(i32::from(q.dim.angle));
    let mut s = format!("{:.*}", decimals, q.value / scale);
    if s.strip_prefix('-')
        .is_some_and(|rest| rest.chars().all(|c| c == '0' || c == '.'))
    {
        s.remove(0);
    }
    let mut parts = Vec::new();
    push_power(&mut parts, units.length.symbol(), q.dim.length);
    push_power(&mut parts, units.angle.symbol(), q.dim.angle);
    if parts.is_empty() {
        s
    } else {
        format!("{s} {}", parts.join("·"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dim_display() {
        assert_eq!(Dim::AREA.to_string(), "area");
        assert_eq!(Dim::UNITLESS.to_string(), "unitless");
        let d = Dim {
            length: 4,
            angle: -1,
        };
        assert_eq!(d.to_string(), "length^4·angle^-1");
    }

    #[test]
    fn formatting() {
        let u = Units::default();
        assert_eq!(format_quantity(Quantity::length(40.0), u, 2), "40.00 mm");
        assert_eq!(
            format_quantity(Quantity::angle(30f64.to_radians()), u, 1),
            "30.0 deg"
        );
        let area = Quantity {
            value: 1600.0,
            dim: Dim::AREA,
        };
        assert_eq!(format_quantity(area, u, 2), "1600.00 mm^2");
        assert_eq!(format_quantity(Quantity::unitless(2.5), u, 1), "2.5");
        assert_eq!(format_quantity(Quantity::length(-0.0), u, 2), "0.00 mm");
        assert_eq!(format_quantity(Quantity::length(-0.0001), u, 2), "0.00 mm");
        let inch = Units {
            length: LengthUnit::Inch,
            angle: AngleUnit::Radian,
        };
        assert_eq!(format_quantity(Quantity::length(25.4), inch, 1), "1.0 in");
        let vol = Quantity {
            value: 25.4_f64.powi(3),
            dim: Dim::VOLUME,
        };
        assert_eq!(format_quantity(vol, inch, 1), "1.0 in^3");
    }
}
