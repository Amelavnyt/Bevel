// SPDX-License-Identifier: GPL-3.0-or-later

//! Rebuilding a part into displayable bodies, away from the UI thread.
//!
//! [`build`] regenerates the part, tessellates every body and measures it.
//! The platform layer runs it on a background thread (desktop) or in a Web
//! Worker (browser) and hands back a [`Built`]; both talk to it through
//! the same [`Rebuilder`](crate::platform::Rebuilder) interface. A newer
//! request cancels an older one.

use bevel_expr::{Units, format_quantity};
use bevel_kernel::{Body, BodyMesh, OpContext, TessellationOptions, mass_properties, tessellate};
use bevel_math::Point3;
use bevel_model::{BodyId, FeatureId, FeatureStatus, Part, Regenerator};
use bevel_timeline::{Cancelled, SkipReason, StepStatus};
use bevel_ui::StepState;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// Volume, area and centre of mass of a body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mass {
    /// Volume in mm³.
    pub volume: f64,
    /// Area in mm².
    pub area: f64,
    /// Centre of mass.
    pub centroid: Point3,
}

/// A body ready to show: its B-rep, display mesh and mass.
#[derive(Clone, Debug)]
pub struct BuiltBody {
    /// Stable id.
    pub id: BodyId,
    /// The B-rep.
    pub body: Arc<Body>,
    /// The display tessellation.
    pub mesh: BodyMesh,
    /// Volume, area and centre of mass.
    pub mass: Mass,
}

/// The result of one rebuild.
#[derive(Clone, Debug)]
pub struct Built {
    /// Which request this answers.
    pub generation: u64,
    /// Every feature's status, in timeline order.
    pub statuses: Vec<(FeatureId, StepState)>,
    /// Every parameter's formatted value or error.
    pub parameters: Vec<(String, Result<String, String>)>,
    /// The bodies at the marker.
    pub bodies: Vec<BuiltBody>,
}

/// A status as the timeline shows it; `name` looks up feature names.
pub fn step_state(status: &FeatureStatus, name: &dyn Fn(FeatureId) -> String) -> StepState {
    match status {
        StepStatus::Ok => StepState::Ok,
        StepStatus::Warning(w) => StepState::Warning(
            w.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join("; "),
        ),
        StepStatus::Error(e) => StepState::Error(e.to_string()),
        StepStatus::Skipped(SkipReason::InputUnavailable(id)) => {
            StepState::Skipped(format!("{} did not build", name(*id)))
        }
        StepStatus::Skipped(SkipReason::InputMissing(id)) => {
            StepState::Skipped(format!("{} was deleted", name(*id)))
        }
        StepStatus::Suppressed => StepState::Suppressed,
        StepStatus::AfterMarker => StepState::RolledBack,
    }
}

/// Formats a parameter value in the document's units.
pub fn format_value(q: bevel_expr::Quantity, units: Units) -> String {
    format_quantity(q, units, 2)
}

/// Regenerates `part` and prepares every body for display.
///
/// # Errors
///
/// [`Cancelled`] once `cancel` is set.
pub fn build(
    regenerator: &mut Regenerator,
    part: &Part,
    generation: u64,
    cancel: Option<&Arc<AtomicBool>>,
) -> Result<Built, Cancelled> {
    let regenerated = regenerator.regenerate(part, cancel)?;
    let name = |id: FeatureId| {
        part.features
            .get(id)
            .map_or_else(|| id.to_string(), |s| s.op().name.clone())
    };
    let statuses = regenerated
        .statuses
        .iter()
        .map(|(id, s)| (*id, step_state(s, &name)))
        .collect();
    let parameters = part
        .parameters
        .params()
        .iter()
        .map(|p| {
            let value = match regenerated.parameters.get(&p.name) {
                Some(Ok(q)) => Ok(format_value(*q, part.units)),
                Some(Err(e)) => Err(e.to_string()),
                None => Err("not evaluated".to_owned()),
            };
            (p.name.clone(), value)
        })
        .collect();
    let cx = match cancel {
        Some(token) => OpContext::with_cancel(Arc::clone(token)),
        None => OpContext::default(),
    };
    let mut bodies = Vec::new();
    for model_body in regenerated.state.bodies.iter() {
        let body = &model_body.body;
        let options = TessellationOptions::fine(&body.bounds());
        // A body that cannot be shown or measured is left out rather than
        // failing the whole rebuild; cancellation is checked below.
        let (Ok(mesh), Ok(mass)) = (tessellate(body, &options, &cx), mass_properties(body, &cx))
        else {
            tracing::warn!(body = ?model_body.id, "could not tessellate or measure a body");
            continue;
        };
        bodies.push(BuiltBody {
            id: model_body.id,
            body: Arc::clone(body),
            mesh,
            mass: Mass {
                volume: mass.volume,
                area: mass.area,
                centroid: mass.centroid,
            },
        });
    }
    if cancel.is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed)) {
        return Err(Cancelled);
    }
    Ok(Built {
        generation,
        statuses,
        parameters,
        bodies,
    })
}
