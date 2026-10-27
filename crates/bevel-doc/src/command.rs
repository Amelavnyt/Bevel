// SPDX-License-Identifier: GPL-3.0-or-later

//! Commands: the only way to change a document.

use crate::state::{BodyProps, DocState};
use bevel_expr::{ParamKind, Parameter, Units};
use bevel_model::{
    BodyId, BodyMode, Feature, FeatureDef, FeatureId, OriginPlane, PrimitiveDef, PrimitiveKind,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

/// A change to the document. Commands are validated and applied as a whole
/// or not at all, can be serialized, and carry a label for the undo menu.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum Command {
    /// Adds a feature at the marker. The id is the document's next one.
    AddFeature {
        /// What the feature does.
        def: FeatureDef,
        /// Its name; `None` numbers it by kind ("Box1", "Box2", ...).
        name: Option<String>,
    },
    /// Replaces a feature's definition.
    EditFeature {
        /// The feature.
        id: FeatureId,
        /// The new definition.
        def: FeatureDef,
    },
    /// Deletes a feature, and its model parameters that nothing else uses.
    DeleteFeature {
        /// The feature.
        id: FeatureId,
        /// Also delete the features that depend on it; otherwise they stay
        /// and are skipped.
        cascade: bool,
    },
    /// Moves a feature to another place in the timeline.
    MoveFeature {
        /// The feature.
        id: FeatureId,
        /// Its index afterwards.
        to: usize,
    },
    /// Suppresses or unsuppresses a feature.
    SetSuppressed {
        /// The feature.
        id: FeatureId,
        /// Suppressed or not.
        suppressed: bool,
    },
    /// Moves the rollback marker.
    SetMarker {
        /// How many features run.
        index: usize,
    },
    /// Renames a feature.
    RenameFeature {
        /// The feature.
        id: FeatureId,
        /// The new name.
        name: String,
    },
    /// Edits the parameter table.
    Parameters(ParamEdit),
    /// Sets a body's name, colour or visibility.
    SetBodyProps {
        /// The body.
        body: BodyId,
        /// The new properties.
        props: BodyProps,
    },
    /// Sets the document's default units.
    SetUnits(Units),
    /// Several commands as one undo step; all apply or none do.
    Batch {
        /// The undo label.
        label: String,
        /// The commands, in order.
        cmds: Vec<Command>,
    },
}

/// A change to the parameter table.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "edit", rename_all = "snake_case")]
pub enum ParamEdit {
    /// Adds a parameter.
    Add(Parameter),
    /// Sets a parameter's expression.
    SetExpression {
        /// The parameter.
        name: String,
        /// The new expression.
        expression: String,
    },
    /// Sets a parameter's comment.
    SetComment {
        /// The parameter.
        name: String,
        /// The new comment.
        comment: String,
    },
    /// Renames a parameter and every reference to it.
    Rename {
        /// The current name.
        old: String,
        /// The new name.
        new: String,
    },
    /// Removes a parameter nothing uses.
    Remove {
        /// The parameter.
        name: String,
    },
}

/// Why a command was refused. The document is unchanged.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum CommandError {
    /// No feature has this id.
    #[error("there is no feature {0}")]
    NoFeature(FeatureId),
    /// The timeline refused the edit.
    #[error("{0}")]
    Timeline(String),
    /// The parameter table refused the edit.
    #[error("{0}")]
    Parameter(String),
    /// A parameter is still used by features.
    #[error("parameter `{name}` is used by {users}")]
    ParameterInUse {
        /// The parameter.
        name: String,
        /// The features using it, comma-separated.
        users: String,
    },
    /// A feature names a parameter that does not exist.
    #[error("feature {feature} uses parameter `{name}`, which does not exist")]
    MissingParameter {
        /// The feature.
        feature: String,
        /// The parameter.
        name: String,
    },
    /// A name is empty or only whitespace.
    #[error("names cannot be empty")]
    EmptyName,
}

fn timeline_err<E: std::fmt::Display>(e: E) -> CommandError {
    CommandError::Timeline(e.to_string())
}

fn param_err<E: std::fmt::Display>(e: E) -> CommandError {
    CommandError::Parameter(e.to_string())
}

impl Command {
    /// The undo label, e.g. "Add Box1" or "Edit Cylinder2".
    pub fn label(&self, state: &DocState) -> String {
        let name = |id: &FeatureId| {
            state
                .feature(*id)
                .map_or_else(|| id.to_string(), |f| f.name.clone())
        };
        match self {
            Self::AddFeature { def, name: n } => match n {
                Some(n) => format!("Add {n}"),
                None => format!("Add {}", def.kind_name()),
            },
            Self::EditFeature { id, .. } => format!("Edit {}", name(id)),
            Self::DeleteFeature { id, .. } => format!("Delete {}", name(id)),
            Self::MoveFeature { id, .. } => format!("Move {}", name(id)),
            Self::SetSuppressed { id, suppressed } => {
                let verb = if *suppressed {
                    "Suppress"
                } else {
                    "Unsuppress"
                };
                format!("{verb} {}", name(id))
            }
            Self::SetMarker { .. } => "Move marker".to_owned(),
            Self::RenameFeature { id, .. } => format!("Rename {}", name(id)),
            Self::Parameters(edit) => match edit {
                ParamEdit::Add(p) => format!("Add parameter {}", p.name),
                ParamEdit::SetExpression { name, .. } | ParamEdit::SetComment { name, .. } => {
                    format!("Change parameter {name}")
                }
                ParamEdit::Rename { old, .. } => format!("Rename parameter {old}"),
                ParamEdit::Remove { name } => format!("Delete parameter {name}"),
            },
            Self::SetBodyProps { .. } => "Change body".to_owned(),
            Self::SetUnits(_) => "Change units".to_owned(),
            Self::Batch { label, .. } => label.clone(),
        }
    }

    /// Whether the command only moves the marker (kept out of undo).
    pub fn is_marker_move(&self) -> bool {
        matches!(self, Self::SetMarker { .. })
    }

    /// Applies the command to `state`.
    ///
    /// # Errors
    ///
    /// Why the command was refused; `state` is not changed.
    pub fn apply(&self, state: &DocState) -> Result<DocState, CommandError> {
        let mut next = state.clone();
        self.apply_in_place(&mut next)?;
        Ok(next)
    }

    fn apply_in_place(&self, s: &mut DocState) -> Result<(), CommandError> {
        match self {
            Self::AddFeature { def, name } => {
                check_parameters(s, def, None)?;
                let id = FeatureId(s.next_feature_id);
                let name = match name {
                    Some(n) => non_empty(n)?,
                    None => s.next_feature_name(def.kind_name()),
                };
                let feature = Feature {
                    id,
                    name,
                    def: def.clone(),
                };
                s.part_mut()
                    .features
                    .insert(feature)
                    .map_err(timeline_err)?;
                s.next_feature_id += 1;
            }
            Self::EditFeature { id, def } => {
                let old = s.feature(*id).ok_or(CommandError::NoFeature(*id))?.clone();
                check_parameters(s, def, Some(&old.name))?;
                let feature = Feature {
                    def: def.clone(),
                    ..old
                };
                s.part_mut()
                    .features
                    .replace(feature)
                    .map_err(timeline_err)?;
            }
            Self::DeleteFeature { id, cascade } => {
                s.feature(*id).ok_or(CommandError::NoFeature(*id))?;
                let removed: Vec<Arc<Feature>> = if *cascade {
                    s.part_mut()
                        .features
                        .remove_with_dependents(*id)
                        .map_err(timeline_err)?
                        .into_iter()
                        .map(|st| Arc::clone(st.op()))
                        .collect()
                } else {
                    vec![Arc::clone(
                        s.part_mut()
                            .features
                            .remove(*id)
                            .map_err(timeline_err)?
                            .op(),
                    )]
                };
                remove_orphan_model_parameters(s, &removed);
                let gone: BTreeSet<FeatureId> = removed.iter().map(|f| f.id).collect();
                Arc::make_mut(&mut s.body_props).retain(|b, _| !gone.contains(&b.feature));
            }
            Self::MoveFeature { id, to } => {
                s.part_mut()
                    .features
                    .move_to(*id, *to)
                    .map_err(timeline_err)?;
            }
            Self::SetSuppressed { id, suppressed } => {
                s.part_mut()
                    .features
                    .set_suppressed(*id, *suppressed)
                    .map_err(timeline_err)?;
            }
            Self::SetMarker { index } => {
                s.part_mut()
                    .features
                    .set_marker(*index)
                    .map_err(timeline_err)?;
            }
            Self::RenameFeature { id, name } => {
                let name = non_empty(name)?;
                let old = s.feature(*id).ok_or(CommandError::NoFeature(*id))?.clone();
                s.part_mut()
                    .features
                    .replace(Feature { name, ..old })
                    .map_err(timeline_err)?;
            }
            Self::Parameters(edit) => apply_param_edit(s, edit)?,
            Self::SetBodyProps { body, props } => {
                Arc::make_mut(&mut s.body_props).insert(*body, props.clone());
            }
            Self::SetUnits(units) => s.part_mut().units = *units,
            Self::Batch { cmds, .. } => {
                for c in cmds {
                    c.apply_in_place(s)?;
                }
            }
        }
        Ok(())
    }
}

fn non_empty(name: &str) -> Result<String, CommandError> {
    let name = name.trim();
    if name.is_empty() {
        Err(CommandError::EmptyName)
    } else {
        Ok(name.to_owned())
    }
}

/// Every parameter `def` reads must exist.
fn check_parameters(
    s: &DocState,
    def: &FeatureDef,
    feature: Option<&str>,
) -> Result<(), CommandError> {
    for name in def.parameters() {
        if s.part.parameters.get(name).is_none() {
            return Err(CommandError::MissingParameter {
                feature: feature.unwrap_or(def.kind_name()).to_owned(),
                name: name.to_owned(),
            });
        }
    }
    Ok(())
}

/// The features that read parameter `name`.
fn features_using<'a>(s: &'a DocState, name: &str) -> Vec<&'a str> {
    s.part
        .features
        .steps()
        .iter()
        .filter(|st| st.op().def.parameters().contains(&name))
        .map(|st| st.op().name.as_str())
        .collect()
}

fn apply_param_edit(s: &mut DocState, edit: &ParamEdit) -> Result<(), CommandError> {
    match edit {
        ParamEdit::Add(p) => s.part_mut().parameters.add(p.clone()).map_err(param_err),
        ParamEdit::SetExpression { name, expression } => s
            .part_mut()
            .parameters
            .set_expression(name, expression.clone())
            .map_err(param_err),
        ParamEdit::SetComment { name, comment } => s
            .part_mut()
            .parameters
            .set_comment(name, comment.clone())
            .map_err(param_err),
        ParamEdit::Rename { old, new } => {
            s.part_mut()
                .parameters
                .rename(old, new)
                .map_err(param_err)?;
            // Features name their parameters directly; follow the rename.
            let renamed: Vec<Feature> = s
                .part
                .features
                .steps()
                .iter()
                .filter(|st| st.op().def.parameters().contains(&old.as_str()))
                .map(|st| {
                    let mut f = st.op().as_ref().clone();
                    rename_input(&mut f.def, old, new);
                    f
                })
                .collect();
            for f in renamed {
                s.part_mut().features.replace(f).map_err(timeline_err)?;
            }
            Ok(())
        }
        ParamEdit::Remove { name } => {
            let users = features_using(s, name);
            if !users.is_empty() {
                return Err(CommandError::ParameterInUse {
                    name: name.clone(),
                    users: users.join(", "),
                });
            }
            s.part_mut()
                .parameters
                .remove(name)
                .map(drop)
                .map_err(param_err)
        }
    }
}

fn rename_input(def: &mut FeatureDef, old: &str, new: &str) {
    match def {
        FeatureDef::Primitive(p) => {
            for value in p.inputs.values_mut() {
                if value == old {
                    new.clone_into(value);
                }
            }
        }
    }
}

/// After deleting `removed`, deletes the model parameters only they used
/// and that no other parameter reads.
fn remove_orphan_model_parameters(s: &mut DocState, removed: &[Arc<Feature>]) {
    let candidates: BTreeSet<String> = removed
        .iter()
        .flat_map(|f| f.def.parameters().into_iter().map(str::to_owned))
        .collect();
    for name in candidates {
        let is_model = s
            .part
            .parameters
            .get(&name)
            .is_some_and(|p| p.kind == ParamKind::Model);
        if is_model
            && features_using(s, &name).is_empty()
            && s.part.parameters.dependents(&name).is_empty()
        {
            // Cannot fail: the parameter exists and nothing uses it.
            let _ = s.part_mut().parameters.remove(&name);
        }
    }
}

/// The command that adds a primitive with the given field expressions
/// (missing fields get their defaults): one model parameter per field
/// ("d1", "d2", ...) and the feature, as one undo step.
pub fn add_primitive(
    state: &DocState,
    kind: PrimitiveKind,
    plane: OriginPlane,
    expressions: &BTreeMap<String, String>,
) -> Command {
    let mut table = state.part.parameters.clone();
    let mut cmds = Vec::new();
    let mut inputs = BTreeMap::new();
    for field in kind.fields() {
        let name = table.next_model_name();
        let expression = expressions
            .get(field.key)
            .cloned()
            .unwrap_or_else(|| field.default.to_owned());
        let param = Parameter {
            name: name.clone(),
            expression,
            comment: String::new(),
            kind: ParamKind::Model,
        };
        // Reserve the name so the next field gets the next one.
        let _ = table.add(param.clone());
        cmds.push(Command::Parameters(ParamEdit::Add(param)));
        inputs.insert(field.key.to_owned(), name);
    }
    cmds.push(Command::AddFeature {
        def: FeatureDef::Primitive(PrimitiveDef {
            kind,
            plane,
            inputs,
            body_mode: BodyMode::NewBody,
        }),
        name: None,
    });
    Command::Batch {
        label: format!("Add {}", kind.name()),
        cmds,
    }
}
