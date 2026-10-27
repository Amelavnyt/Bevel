// SPDX-License-Identifier: GPL-3.0-or-later

//! Regeneration: replaying a part's features into bodies.

use crate::feature::{Feature, FeatureDef, FeatureId, FieldKind, PrimitiveDef, PrimitiveKind};
use bevel_expr::{Dim, Evaluated, ParameterTable, Quantity, Units};
use bevel_kernel::{
    Body, BoxParams, ConeParams, CylinderParams, KernelError, OpContext, OpTag, SphereParams,
    TorusParams,
};
use bevel_math::{Point3, Vector3};
use bevel_timeline::{
    Applied, Cancelled, Digest, DigestBuilder, Evaluator, ReplayCache, StepStatus, Timeline,
};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A part: its features in timeline order, its parameters and units.
#[derive(Clone, Debug, Default)]
pub struct Part {
    /// The features and the rollback marker.
    pub features: Timeline<Feature>,
    /// User and model parameters.
    pub parameters: ParameterTable,
    /// Default units for bare numbers.
    pub units: Units,
}

/// Identifies a body: the feature that created it and its index among that
/// feature's results.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct BodyId {
    /// The creating feature.
    pub feature: FeatureId,
    /// The index among its results.
    pub index: u32,
}

/// A body in the model.
#[derive(Clone, Debug)]
pub struct ModelBody {
    /// Stable id.
    pub id: BodyId,
    /// The B-rep.
    pub body: Arc<Body>,
}

/// What regeneration produces at the marker.
#[derive(Clone, Debug, Default)]
pub struct ModelState {
    /// The bodies, in creation order.
    pub bodies: Arc<Vec<ModelBody>>,
}

impl ModelState {
    /// Body `id`.
    pub fn body(&self, id: BodyId) -> Option<&ModelBody> {
        self.bodies.iter().find(|b| b.id == id)
    }
}

/// Why a feature failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FeatureError {
    /// An input is not set.
    #[error("{field} is not set")]
    MissingInput {
        /// The field's label.
        field: &'static str,
    },
    /// An input's parameter does not exist or could not be evaluated.
    #[error("{field}: {message}")]
    Parameter {
        /// The field's label.
        field: &'static str,
        /// What went wrong.
        message: String,
    },
    /// The kernel refused the operation.
    #[error("{0}")]
    Kernel(String),
}

/// A feature's status after regeneration.
pub type FeatureStatus = StepStatus<FeatureId, FeatureError>;

/// The result of regenerating a part.
#[derive(Clone, Debug)]
pub struct Regenerated {
    /// The model at the marker.
    pub state: ModelState,
    /// Every feature's status, in timeline order.
    pub statuses: Vec<(FeatureId, FeatureStatus)>,
    /// Every parameter's value or error.
    pub parameters: Evaluated,
    /// How many features were run rather than taken from the cache.
    pub evaluated: usize,
}

/// Regenerates parts, keeping a cache of model states between runs.
pub struct Regenerator {
    cache: ReplayCache<FeatureId, ModelState, FeatureError>,
}

impl Default for Regenerator {
    fn default() -> Self {
        Self::new()
    }
}

/// States kept by default; each shares its bodies with its neighbours, so
/// a state costs little more than a list of pointers.
const DEFAULT_BUDGET: usize = 512;

impl Regenerator {
    /// A regenerator with an empty cache.
    pub fn new() -> Self {
        Self {
            cache: ReplayCache::new(DEFAULT_BUDGET),
        }
    }

    /// Replays `part` up to its marker. Features that read parameters whose
    /// values changed are re-run; the rest come from the cache.
    ///
    /// # Errors
    ///
    /// [`Cancelled`] once `cancel` is set.
    pub fn regenerate(
        &mut self,
        part: &Part,
        cancel: Option<&Arc<AtomicBool>>,
    ) -> Result<Regenerated, Cancelled> {
        let parameters = part.parameters.evaluate(part.units);
        let eval = Eval {
            parameters: &parameters,
            units: part.units,
            cancel: cancel.cloned(),
        };
        let cancelled = || cancel.is_some_and(|c| c.load(Ordering::Relaxed));
        let result = bevel_timeline::replay(&part.features, &eval, &mut self.cache, &cancelled);
        if cancelled() {
            // A kernel operation stopped part-way may have been recorded as
            // a failure; forget it.
            self.cache.clear();
            return Err(Cancelled);
        }
        let replay = result?;
        Ok(Regenerated {
            state: replay.state,
            statuses: replay.statuses,
            parameters,
            evaluated: replay.evaluated,
        })
    }
}

struct Eval<'a> {
    parameters: &'a Evaluated,
    units: Units,
    cancel: Option<Arc<AtomicBool>>,
}

impl Evaluator<Feature> for Eval<'_> {
    type State = ModelState;
    type Diagnostic = FeatureError;

    fn initial_state(&self) -> ModelState {
        ModelState::default()
    }

    fn input_hash(&self, op: &Feature) -> Digest {
        let mut b = DigestBuilder::new("bevel-model inputs");
        for name in op.def.parameters() {
            b.bytes(name.as_bytes());
            match self.parameters.value(name) {
                Some(q) => {
                    b.f64(q.value)
                        .u64(u64::from(q.dim.length.cast_unsigned()))
                        .u64(u64::from(q.dim.angle.cast_unsigned()));
                }
                None => {
                    b.bytes(b"unavailable");
                }
            }
        }
        // Bare numbers read in the default units.
        b.bytes(format!("{:?}", self.units).as_bytes());
        b.finish()
    }

    fn apply(
        &self,
        state: &ModelState,
        op: &Feature,
    ) -> Result<Applied<ModelState, FeatureError>, FeatureError> {
        let mut cx = match &self.cancel {
            Some(token) => OpContext::with_cancel(Arc::clone(token)),
            None => OpContext::default(),
        };
        let bodies = match &op.def {
            FeatureDef::Primitive(p) => self.primitive(p, OpTag(op.id.0), &mut cx)?,
        };
        let mut all = state.bodies.as_ref().clone();
        for (index, body) in (0u32..).zip(bodies) {
            all.push(ModelBody {
                id: BodyId {
                    feature: op.id,
                    index,
                },
                body,
            });
        }
        Ok(Applied::ok(ModelState {
            bodies: Arc::new(all),
        }))
    }
}

impl Eval<'_> {
    /// The value of field `key` of `kind`, in internal units.
    fn input(&self, def: &PrimitiveDef, key: &str) -> Result<f64, FeatureError> {
        let field = def
            .kind
            .fields()
            .iter()
            .find(|f| f.key == key)
            .ok_or(FeatureError::MissingInput { field: "input" })?;
        let name = def
            .inputs
            .get(key)
            .ok_or(FeatureError::MissingInput { field: field.label })?;
        let value = match self.parameters.get(name) {
            None => {
                return Err(FeatureError::Parameter {
                    field: field.label,
                    message: format!("parameter `{name}` does not exist"),
                });
            }
            Some(Err(e)) => {
                return Err(FeatureError::Parameter {
                    field: field.label,
                    message: format!("`{name}`: {e}"),
                });
            }
            Some(Ok(q)) => *q,
        };
        coerce(value, field.kind, self.units).map_err(|message| FeatureError::Parameter {
            field: field.label,
            message: format!("`{name}` {message}"),
        })
    }

    fn primitive(
        &self,
        def: &PrimitiveDef,
        tag: OpTag,
        cx: &mut OpContext,
    ) -> Result<Vec<Arc<Body>>, FeatureError> {
        let get = |key| self.input(def, key);
        let mut frame = def.plane.frame();
        frame.origin = Point3::ZERO + Vector3::new(get("x")?, get("y")?, get("z")?);
        let output = match def.kind {
            PrimitiveKind::Box => bevel_kernel::box_(
                &BoxParams {
                    frame,
                    size: Vector3::new(get("length")?, get("width")?, get("height")?),
                },
                tag,
                cx,
            ),
            PrimitiveKind::Cylinder => bevel_kernel::cylinder(
                &CylinderParams {
                    frame,
                    radius: get("radius")?,
                    height: get("height")?,
                },
                tag,
                cx,
            ),
            PrimitiveKind::Sphere => bevel_kernel::sphere(
                &SphereParams {
                    frame,
                    radius: get("radius")?,
                },
                tag,
                cx,
            ),
            PrimitiveKind::Cone => bevel_kernel::cone(
                &ConeParams {
                    frame,
                    bottom_radius: get("bottom_radius")?,
                    top_radius: get("top_radius")?,
                    height: get("height")?,
                },
                tag,
                cx,
            ),
            PrimitiveKind::Torus => bevel_kernel::torus(
                &TorusParams {
                    frame,
                    major_radius: get("major_radius")?,
                    minor_radius: get("minor_radius")?,
                },
                tag,
                cx,
            ),
        };
        output
            .map(|o| o.bodies)
            .map_err(|e: KernelError| FeatureError::Kernel(e.to_string()))
    }
}

/// A parameter value as a field of `kind` expects it, in internal units. A
/// unitless value is read in the document's default unit (typing `40` in a
/// length field means 40 mm in a millimetre document).
///
/// # Errors
///
/// A short message if the value has the wrong dimension.
pub fn coerce(value: Quantity, kind: FieldKind, units: Units) -> Result<f64, String> {
    let expected = match kind {
        FieldKind::Length => Dim::LENGTH,
        FieldKind::Angle => Dim::ANGLE,
    };
    if value.dim == expected {
        Ok(value.value)
    } else if value.dim == Dim::UNITLESS {
        Ok(match kind {
            FieldKind::Length => units.length.to_internal(value.value),
            FieldKind::Angle => units.angle.to_internal(value.value),
        })
    } else {
        Err(format!("is a {}, not a {expected}", value.dim))
    }
}
