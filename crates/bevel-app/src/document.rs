// SPDX-License-Identifier: GPL-3.0-or-later

//! The open document: its session (state and undo history), where it was
//! saved, and its rebuilds. Also turns the document into what the UI shows
//! and the feature dialog's texts into commands.

use crate::platform::Rebuilder;
use crate::rebuild::Built;
use bevel_doc::{Command, CommandError, DocSession, DocState, ParamEdit, add_primitive};
use bevel_expr::{EvalContext, Evaluated, Expr};
use bevel_model::{Feature, FeatureDef, FeatureId, Part, PrimitiveDef, coerce};
use bevel_ui::{DocView, FeatureDialog, ParamRow, StepState, TimelineEntry};
use std::collections::{BTreeMap, BTreeSet};

/// The open document.
pub struct Document {
    session: DocSession,
    /// Where the document was last saved or opened from, if anywhere.
    pub location: Option<String>,
    rebuilder: Rebuilder,
    /// The generation of the latest rebuild request.
    requested: u64,
    /// The latest finished rebuild (possibly of a preview).
    built: Option<Built>,
}

impl Document {
    /// A document holding `state`, with a rebuild requested.
    pub fn new(ctx: &egui::Context, state: DocState) -> Self {
        let mut doc = Self {
            session: DocSession::new(state),
            location: None,
            rebuilder: Rebuilder::new(ctx),
            requested: 0,
            built: None,
        };
        doc.rebuild(None);
        doc
    }

    /// Replaces the document (New, Open), keeping the rebuild worker.
    pub fn replace(&mut self, state: DocState, location: Option<String>) {
        self.session = DocSession::new(state);
        self.location = location;
        self.rebuild(None);
    }

    /// The current state.
    pub fn state(&self) -> &DocState {
        self.session.state()
    }

    /// The session, for undo and saving.
    pub fn session_mut(&mut self) -> &mut DocSession {
        &mut self.session
    }

    /// The latest finished rebuild.
    pub fn built(&self) -> Option<&Built> {
        self.built.as_ref()
    }

    /// Whether a rebuild newer than the shown one is running.
    pub fn rebuilding(&self) -> bool {
        self.built
            .as_ref()
            .is_none_or(|b| b.generation < self.requested)
    }

    /// Runs `cmd` and rebuilds.
    ///
    /// # Errors
    ///
    /// Why the command was refused.
    pub fn execute(&mut self, cmd: &Command) -> Result<(), CommandError> {
        self.session.execute(cmd)?;
        self.rebuild(None);
        Ok(())
    }

    /// Requests a rebuild of the document, or of `preview` (the document
    /// with a dialog's command applied) instead.
    pub fn rebuild(&mut self, preview: Option<&DocState>) {
        self.requested += 1;
        let part: Part = preview
            .unwrap_or_else(|| self.session.state())
            .part
            .as_ref()
            .clone();
        self.rebuilder.request(self.requested, part);
    }

    /// Takes a finished rebuild; returns whether one arrived.
    pub fn poll(&mut self) -> bool {
        match self.rebuilder.poll() {
            Some(built) => {
                self.built = Some(built);
                true
            }
            None => false,
        }
    }

    /// The document as the UI shows it.
    pub fn view(&self) -> DocView {
        let state = self.session.state();
        let part = &state.part;
        let statuses: BTreeMap<FeatureId, &StepState> = self
            .built
            .iter()
            .flat_map(|b| b.statuses.iter().map(|(id, s)| (*id, s)))
            .collect();
        let marker = part.features.marker();
        let timeline = part
            .features
            .steps()
            .iter()
            .enumerate()
            .map(|(i, step)| {
                let f = step.op();
                let state = if i >= marker {
                    StepState::RolledBack
                } else if step.suppressed() {
                    StepState::Suppressed
                } else {
                    statuses.get(&f.id).map_or(StepState::Ok, |s| (*s).clone())
                };
                TimelineEntry {
                    id: f.id,
                    name: f.name.clone(),
                    kind: kind(f),
                    deps: bevel_timeline::TimelineOp::deps(f.as_ref()),
                    suppressed: step.suppressed(),
                    state,
                }
            })
            .collect();
        let values: BTreeMap<&str, &Result<String, String>> = self
            .built
            .iter()
            .flat_map(|b| b.parameters.iter().map(|(n, v)| (n.as_str(), v)))
            .collect();
        let mut parameters: Vec<ParamRow> = part
            .parameters
            .params()
            .iter()
            .map(|p| ParamRow {
                name: p.name.clone(),
                expression: p.expression.clone(),
                value: values
                    .get(p.name.as_str())
                    .map_or_else(|| Err("not built yet".to_owned()), |v| (*v).clone()),
                comment: p.comment.clone(),
                model: p.kind == bevel_expr::ParamKind::Model,
                used_by: used_by(state, &p.name),
            })
            .collect();
        parameters.sort_by_key(|p| p.model);
        DocView {
            title: self.title(),
            modified: self.session.is_modified(),
            timeline,
            marker,
            parameters,
            units: part.units.length.symbol().to_owned(),
            undo: self.session.undo_label().map(str::to_owned),
            redo: self.session.redo_label().map(str::to_owned),
            rebuilding: self.rebuilding(),
        }
    }

    /// The document's name: its file name without the extension.
    pub fn title(&self) -> String {
        self.location.as_deref().map_or_else(
            || "Untitled".to_owned(),
            |place| {
                let name = place.rsplit(['/', '\\']).next().unwrap_or(place);
                name.strip_suffix(".bevel").unwrap_or(name).to_owned()
            },
        )
    }

    /// The dialog for editing feature `id`, filled with its expressions.
    pub fn edit_dialog(&self, id: FeatureId) -> Option<FeatureDialog> {
        let state = self.session.state();
        let feature = state.feature(id)?;
        let FeatureDef::Primitive(p) = &feature.def;
        let fields = p.kind.fields();
        let values = fields
            .iter()
            .map(|f| {
                p.inputs
                    .get(f.key)
                    .and_then(|name| state.part.parameters.get(name))
                    .map_or_else(|| f.default.to_owned(), |param| param.expression.clone())
            })
            .collect();
        Some(FeatureDialog {
            editing: Some(id),
            kind: p.kind,
            plane: p.plane,
            values,
            errors: vec![None; fields.len()],
            error: None,
        })
    }

    /// The command the dialog stands for: adding the primitive, or changing
    /// the edited feature's parameters and plane.
    pub fn dialog_command(&self, dialog: &FeatureDialog) -> Option<Command> {
        let state = self.session.state();
        let Some(id) = dialog.editing else {
            let values = dialog
                .kind
                .fields()
                .iter()
                .zip(&dialog.values)
                .map(|(f, v)| (f.key.to_owned(), v.clone()))
                .collect();
            return Some(add_primitive(state, dialog.kind, dialog.plane, &values));
        };
        let feature = state.feature(id)?;
        let FeatureDef::Primitive(p) = &feature.def;
        let mut cmds = Vec::new();
        for (field, value) in p.kind.fields().iter().zip(&dialog.values) {
            let name = p.inputs.get(field.key)?;
            let param = state.part.parameters.get(name)?;
            if &param.expression != value {
                cmds.push(Command::Parameters(ParamEdit::SetExpression {
                    name: name.clone(),
                    expression: value.clone(),
                }));
            }
        }
        if p.plane != dialog.plane {
            cmds.push(Command::EditFeature {
                id,
                def: FeatureDef::Primitive(PrimitiveDef {
                    plane: dialog.plane,
                    ..p.clone()
                }),
            });
        }
        Some(Command::Batch {
            label: format!("Edit {}", feature.name),
            cmds,
        })
    }

    /// Checks the dialog's fields against the current parameter values and
    /// fills in its errors; when they are all valid, requests a preview
    /// rebuild with the dialog applied.
    pub fn check_dialog(&mut self, dialog: &mut FeatureDialog) {
        let state = self.session.state();
        let part = &state.part;
        let evaluated = part.parameters.evaluate(part.units);
        // A field must not read the parameter it sets.
        let own: Vec<Option<String>> = match dialog.editing.and_then(|id| state.feature(id)) {
            Some(Feature {
                def: FeatureDef::Primitive(p),
                ..
            }) => dialog
                .kind
                .fields()
                .iter()
                .map(|f| p.inputs.get(f.key).cloned())
                .collect(),
            _ => vec![None; dialog.values.len()],
        };
        dialog.errors = dialog
            .kind
            .fields()
            .iter()
            .zip(&dialog.values)
            .zip(&own)
            .map(|((field, text), own)| {
                field_error(text, field.kind, &evaluated, part, own.as_deref())
            })
            .collect();
        dialog.error = None;
        if !dialog.errors.iter().all(Option::is_none) {
            return;
        }
        let preview = self
            .dialog_command(dialog)
            .and_then(|cmd| cmd.apply(state).ok());
        match preview {
            Some(preview) => self.rebuild(Some(&preview)),
            None => dialog.error = Some("These values cannot be applied".to_owned()),
        }
    }

    /// After a preview rebuild arrives, the error the dialog's feature hit,
    /// if any.
    pub fn preview_error(&self, dialog: &FeatureDialog) -> Option<String> {
        let id = dialog
            .editing
            .unwrap_or(FeatureId(self.session.state().next_feature_id));
        let built = self.built.as_ref()?;
        if built.generation < self.requested {
            return None;
        }
        built
            .statuses
            .iter()
            .find(|(f, _)| *f == id)
            .and_then(|(_, s)| match s {
                StepState::Error(e) | StepState::Skipped(e) => Some(e.clone()),
                _ => None,
            })
    }
}

fn kind(feature: &Feature) -> bevel_model::PrimitiveKind {
    match &feature.def {
        FeatureDef::Primitive(p) => p.kind,
    }
}

/// The features and parameters that read parameter `name`.
fn used_by(state: &DocState, name: &str) -> Vec<String> {
    let part = &state.part;
    let mut users: Vec<String> = part
        .features
        .steps()
        .iter()
        .filter(|s| s.op().def.parameters().contains(&name))
        .map(|s| s.op().name.clone())
        .collect();
    let params: BTreeSet<String> = part.parameters.dependents(name);
    users.extend(params);
    users
}

/// Why `text` is not a valid value for a field of `kind`, if it is not.
fn field_error(
    text: &str,
    kind: bevel_model::FieldKind,
    evaluated: &Evaluated,
    part: &Part,
    own: Option<&str>,
) -> Option<String> {
    let expr = match Expr::parse(text) {
        Ok(e) => e,
        Err(e) => return Some(e.to_string()),
    };
    if let Some(own) = own
        && (expr.identifiers().contains(own)
            || expr
                .identifiers()
                .iter()
                .any(|i| part.parameters.dependents(own).contains(i)))
    {
        return Some(format!("cannot refer to its own parameter `{own}`"));
    }
    let lookup = |name: &str| evaluated.value(name);
    let cx = EvalContext {
        units: part.units,
        lookup: &lookup,
    };
    let value = match expr.eval(&cx) {
        Ok(v) => v,
        Err(e) => return Some(e.to_string()),
    };
    coerce(value, kind, part.units).err()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

    use super::*;
    use bevel_model::PrimitiveKind;
    use std::time::{Duration, Instant};

    fn doc() -> Document {
        Document::new(&egui::Context::default(), bevel_doc::sample::primitives())
    }

    /// Waits for the background rebuild to catch up.
    fn settle(doc: &mut Document) {
        let start = Instant::now();
        while doc.rebuilding() {
            doc.poll();
            assert!(
                start.elapsed() < Duration::from_secs(30),
                "rebuild never finished"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn the_view_shows_features_parameters_and_statuses() {
        let mut doc = doc();
        settle(&mut doc);
        let view = doc.view();
        assert_eq!(view.title, "Untitled");
        assert_eq!(view.timeline.len(), 5);
        assert!(view.timeline.iter().all(|e| e.state == StepState::Ok));
        assert_eq!(view.marker, 5);
        let d1 = view.parameters.iter().find(|p| p.name == "d1").unwrap();
        assert_eq!(d1.value, Ok("40.00 mm".to_owned()));
        assert_eq!(d1.used_by, ["Box1"]);
        assert!(!view.rebuilding);
    }

    #[test]
    fn dialog_fields_are_checked() {
        let mut doc = doc();
        let mut dialog = FeatureDialog::new(PrimitiveKind::Box);
        dialog.values[0] = "2 *".to_owned();
        dialog.values[1] = "30 deg".to_owned();
        dialog.values[2] = "d1 / 2".to_owned();
        doc.check_dialog(&mut dialog);
        assert!(dialog.errors[0].is_some());
        assert!(dialog.errors[1].as_ref().unwrap().contains("angle"));
        assert!(
            dialog.errors[2].is_none(),
            "existing parameters can be used"
        );
        // An edited feature's field cannot read its own parameter.
        let mut edit = doc.edit_dialog(FeatureId(1)).unwrap();
        assert_eq!(edit.values[0], "40 mm");
        edit.values[0] = "d1 + 1 mm".to_owned();
        doc.check_dialog(&mut edit);
        assert!(edit.errors[0].as_ref().unwrap().contains("own parameter"));
    }

    #[test]
    fn editing_changes_only_what_changed() {
        let mut doc = doc();
        let mut edit = doc.edit_dialog(FeatureId(2)).unwrap();
        edit.values[0] = "25 mm".to_owned();
        let Some(Command::Batch { label, cmds }) = doc.dialog_command(&edit) else {
            panic!("expected a batch");
        };
        assert_eq!(label, "Edit Cylinder1");
        assert_eq!(cmds.len(), 1, "{cmds:?}");
        doc.execute(&Command::Batch { label, cmds }).unwrap();
        settle(&mut doc);
        let built = doc.built().unwrap();
        let cylinder = built
            .bodies
            .iter()
            .find(|b| b.id.feature == FeatureId(2))
            .unwrap();
        let expected = std::f64::consts::PI * 25.0 * 25.0 * 40.0;
        assert!((cylinder.mass.volume - expected).abs() < 1e-6 * expected);
    }

    #[test]
    fn previews_report_kernel_errors() {
        let mut doc = doc();
        let mut dialog = FeatureDialog::new(PrimitiveKind::Torus);
        // A tube wider than the ring is refused by the kernel, not the parser.
        dialog.values[1] = "30 mm".to_owned();
        doc.check_dialog(&mut dialog);
        assert!(dialog.errors.iter().all(Option::is_none));
        settle(&mut doc);
        let error = doc.preview_error(&dialog).unwrap();
        assert!(error.contains("minor"), "{error}");
    }
}
