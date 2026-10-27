// SPDX-License-Identifier: GPL-3.0-or-later

//! Features: the steps of a part's timeline.

use crate::origin::OriginPlane;
use bevel_timeline::{Digest, DigestBuilder, TimelineOp};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

/// Identifies a feature for the life of a document. Ids come from a
/// per-document counter and are never reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct FeatureId(pub u64);

impl fmt::Display for FeatureId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "F{}", self.0)
    }
}

/// One step of a part's timeline.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    /// Stable id.
    pub id: FeatureId,
    /// Display name, e.g. "Box1".
    pub name: String,
    /// What the feature does.
    pub def: FeatureDef,
}

/// What a feature does. More kinds arrive with their phases.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum FeatureDef {
    /// A box, cylinder, sphere, cone or torus.
    Primitive(PrimitiveDef),
}

impl FeatureDef {
    /// The parameters the feature reads, sorted, without duplicates.
    pub fn parameters(&self) -> Vec<&str> {
        let mut out: Vec<&str> = match self {
            Self::Primitive(p) => p.inputs.values().map(String::as_str).collect(),
        };
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The kind of feature, for names and icons: "Box", "Cylinder", ...
    pub fn kind_name(&self) -> &'static str {
        match self {
            Self::Primitive(p) => p.kind.name(),
        }
    }
}

impl TimelineOp for Feature {
    type Id = FeatureId;

    fn id(&self) -> FeatureId {
        self.id
    }

    fn deps(&self) -> Vec<FeatureId> {
        // Primitives start from nothing; features that read geometry list
        // the features their references name.
        Vec::new()
    }

    fn content_hash(&self) -> Digest {
        // The name is display only and does not change the result.
        let def = serde_json::to_vec(&self.def).unwrap_or_default();
        DigestBuilder::new("bevel-model feature")
            .u64(self.id.0)
            .bytes(&def)
            .finish()
    }
}

/// How a feature's new geometry combines with existing bodies. Join, cut
/// and intersect arrive with booleans.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyMode {
    /// The geometry becomes new bodies.
    #[default]
    NewBody,
}

/// The kinds of primitive.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrimitiveKind {
    /// A box with a corner at the placement point.
    Box,
    /// A cylinder standing on the placement point.
    Cylinder,
    /// A sphere centred on the placement point.
    Sphere,
    /// A cone or frustum standing on the placement point.
    Cone,
    /// A torus centred on the placement point.
    Torus,
}

/// The kind of value a field holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldKind {
    /// A length (mm internally).
    Length,
    /// An angle (radians internally).
    Angle,
}

/// One numeric input of a feature, for dialogs, scripting and docs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field {
    /// Key in [`PrimitiveDef::inputs`].
    pub key: &'static str,
    /// Label shown in dialogs.
    pub label: &'static str,
    /// The kind of value.
    pub kind: FieldKind,
    /// The expression a new feature starts with.
    pub default: &'static str,
}

const fn length(key: &'static str, label: &'static str, default: &'static str) -> Field {
    Field {
        key,
        label,
        kind: FieldKind::Length,
        default,
    }
}

const PLACEMENT: [Field; 3] = [
    length("x", "X offset", "0 mm"),
    length("y", "Y offset", "0 mm"),
    length("z", "Z offset", "0 mm"),
];

const BOX: [Field; 6] = [
    length("length", "Length", "40 mm"),
    length("width", "Width", "40 mm"),
    length("height", "Height", "40 mm"),
    PLACEMENT[0],
    PLACEMENT[1],
    PLACEMENT[2],
];
const CYLINDER: [Field; 5] = [
    length("radius", "Radius", "20 mm"),
    length("height", "Height", "40 mm"),
    PLACEMENT[0],
    PLACEMENT[1],
    PLACEMENT[2],
];
const SPHERE: [Field; 4] = [
    length("radius", "Radius", "20 mm"),
    PLACEMENT[0],
    PLACEMENT[1],
    PLACEMENT[2],
];
const CONE: [Field; 6] = [
    length("bottom_radius", "Bottom radius", "20 mm"),
    length("top_radius", "Top radius", "0 mm"),
    length("height", "Height", "40 mm"),
    PLACEMENT[0],
    PLACEMENT[1],
    PLACEMENT[2],
];
const TORUS: [Field; 5] = [
    length("major_radius", "Major radius", "20 mm"),
    length("minor_radius", "Minor radius", "6 mm"),
    PLACEMENT[0],
    PLACEMENT[1],
    PLACEMENT[2],
];

impl PrimitiveKind {
    /// Every kind, in menu order.
    pub const ALL: [Self; 5] = [
        Self::Box,
        Self::Cylinder,
        Self::Sphere,
        Self::Cone,
        Self::Torus,
    ];

    /// Display name: "Box", "Cylinder", ...
    pub fn name(self) -> &'static str {
        match self {
            Self::Box => "Box",
            Self::Cylinder => "Cylinder",
            Self::Sphere => "Sphere",
            Self::Cone => "Cone",
            Self::Torus => "Torus",
        }
    }

    /// The inputs, in dialog order; the last three place the primitive.
    pub fn fields(self) -> &'static [Field] {
        match self {
            Self::Box => &BOX,
            Self::Cylinder => &CYLINDER,
            Self::Sphere => &SPHERE,
            Self::Cone => &CONE,
            Self::Torus => &TORUS,
        }
    }
}

/// A primitive feature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrimitiveDef {
    /// Which primitive.
    pub kind: PrimitiveKind,
    /// The origin plane it stands on.
    pub plane: OriginPlane,
    /// For each field key of [`PrimitiveKind::fields`], the name of the
    /// parameter holding its value.
    pub inputs: BTreeMap<String, String>,
    /// How the result combines with existing bodies.
    #[serde(default)]
    pub body_mode: BodyMode,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_field_key_is_unique_per_kind() {
        for kind in PrimitiveKind::ALL {
            let mut keys: Vec<_> = kind.fields().iter().map(|f| f.key).collect();
            keys.sort_unstable();
            keys.dedup();
            assert_eq!(keys.len(), kind.fields().len(), "{kind:?}");
        }
    }

    #[test]
    fn the_name_does_not_change_the_hash() {
        let def = FeatureDef::Primitive(PrimitiveDef {
            kind: PrimitiveKind::Box,
            plane: OriginPlane::Xy,
            inputs: BTreeMap::from([("length".to_owned(), "d1".to_owned())]),
            body_mode: BodyMode::NewBody,
        });
        let a = Feature {
            id: FeatureId(1),
            name: "Box1".to_owned(),
            def: def.clone(),
        };
        let b = Feature {
            name: "Lid".to_owned(),
            ..a.clone()
        };
        assert_eq!(a.content_hash(), b.content_hash());
        let c = Feature {
            id: FeatureId(2),
            ..a.clone()
        };
        assert_ne!(a.content_hash(), c.content_hash());
    }
}
