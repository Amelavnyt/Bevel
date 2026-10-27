// SPDX-License-Identifier: GPL-3.0-or-later

//! Parts, features and regeneration.
//!
//! A [`Part`] owns its features (in a [`bevel_timeline::Timeline`]), its
//! parameters and its default units. A [`Regenerator`] replays the features
//! up to the rollback marker and produces a [`ModelState`]: the bodies,
//! each with a stable [`BodyId`]. A feature that fails does not stop the
//! others; the features that depend on it are skipped.
//!
//! Every numeric input of a feature names a parameter in the part's
//! parameter table, so each dimension can be edited in the Parameters
//! dialog and driven by expressions. Every part also has an origin: a
//! point, the [`OriginPlane`]s and the [`OriginAxis`]es, which are not
//! features and cannot be deleted.

mod feature;
mod origin;
mod regen;

pub use feature::{
    BodyMode, Feature, FeatureDef, FeatureId, Field, FieldKind, PrimitiveDef, PrimitiveKind,
};
pub use origin::{OriginAxis, OriginPlane};
pub use regen::{
    BodyId, FeatureError, FeatureStatus, ModelBody, ModelState, Part, Regenerated, Regenerator,
    coerce,
};
