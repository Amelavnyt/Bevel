// SPDX-License-Identifier: GPL-3.0-or-later

//! Named parameters and their dependency graph.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::error::{ExprError, ExprErrorKind};
use crate::eval::{EvalContext, Expr};
use crate::parser::is_reserved;
use crate::units::{Quantity, Units};

/// Who created a parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ParamKind {
    /// Created by the user.
    User,
    /// Created for a feature input (`d1`, `d2`, ...).
    Model,
}

/// A named expression.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    /// Unique identifier-like name.
    pub name: String,
    /// Expression source text.
    pub expression: String,
    /// Free-form comment.
    pub comment: String,
    /// Who created it.
    pub kind: ParamKind,
}

/// Errors from editing a [`ParameterTable`].
#[derive(Clone, Debug, PartialEq, Error)]
pub enum ParamError {
    /// Not an identifier, or reserved (`pi`, a function name).
    #[error("`{0}` is not a valid parameter name")]
    InvalidName(String),
    /// The name is already taken.
    #[error("a parameter named `{0}` already exists")]
    Duplicate(String),
    /// No parameter has that name.
    #[error("no parameter named `{0}`")]
    NotFound(String),
    /// Other parameters read the parameter.
    #[error("`{name}` is used by {}", .users.join(", "))]
    InUse {
        /// The parameter that cannot be removed.
        name: String,
        /// The parameters that read it, sorted.
        users: Vec<String>,
    },
}

fn is_valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    let ok_first = chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_');
    ok_first && chars.all(|c| c.is_ascii_alphanumeric() || c == '_') && !is_reserved(name)
}

/// An ordered set of parameters.
///
/// Parameter values are evaluated with [`Expr::eval`], not
/// [`eval_as`](crate::eval_as): a bare `40` stays unitless, and a feature
/// field that reads it later applies the default unit then.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParameterTable {
    params: Vec<Parameter>,
}

/// The result of evaluating every parameter.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Evaluated {
    results: BTreeMap<String, Result<Quantity, ExprError>>,
}

impl Evaluated {
    /// The result for `name`.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Result<Quantity, ExprError>> {
        self.results.get(name)
    }

    /// The value of `name`, if it exists and evaluated successfully.
    #[must_use]
    pub fn value(&self, name: &str) -> Option<Quantity> {
        self.results
            .get(name)
            .and_then(|r| r.as_ref().ok().copied())
    }

    /// All results, sorted by name.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &Result<Quantity, ExprError>)> {
        self.results.iter()
    }
}

struct Graph {
    parsed: BTreeMap<String, Result<Expr, ExprError>>,
    /// name -> parameters it reads.
    deps: BTreeMap<String, BTreeSet<String>>,
    /// name -> parameters that read it.
    users: BTreeMap<String, BTreeSet<String>>,
}

impl Graph {
    /// Processing order of every name not in `resolved` (dependencies first),
    /// and the names that could never be processed.
    fn topo(&self, resolved: &BTreeSet<String>) -> (Vec<String>, BTreeSet<String>) {
        let mut indeg: BTreeMap<&String, usize> = BTreeMap::new();
        for (n, ds) in &self.deps {
            if !resolved.contains(n) {
                indeg.insert(n, ds.iter().filter(|d| !resolved.contains(*d)).count());
            }
        }
        let mut queue: VecDeque<&String> = indeg
            .iter()
            .filter(|(_, c)| **c == 0)
            .map(|(n, _)| *n)
            .collect();
        let mut order = Vec::new();
        while let Some(n) = queue.pop_front() {
            order.push(n.clone());
            for m in self.users.get(n).into_iter().flatten() {
                if let Some(c) = indeg.get_mut(m) {
                    *c -= 1;
                    if *c == 0 {
                        queue.push_back(m);
                    }
                }
            }
        }
        let done: BTreeSet<&String> = order.iter().collect();
        let left = indeg
            .keys()
            .filter(|n| !done.contains(**n))
            .map(|n| (*n).clone())
            .collect();
        (order, left)
    }

    /// A cycle through `start` that stays inside `within`, starting at the
    /// smallest name on it.
    fn find_cycle(&self, start: &str, within: &BTreeSet<String>) -> Option<Vec<String>> {
        let mut prev: BTreeMap<&str, &str> = BTreeMap::new();
        let mut queue: VecDeque<&str> = VecDeque::from([start]);
        while let Some(u) = queue.pop_front() {
            for v in self.deps.get(u).into_iter().flatten() {
                if !within.contains(v) {
                    continue;
                }
                if v == start {
                    let mut path = vec![u];
                    let mut cur = u;
                    while cur != start {
                        cur = prev.get(cur)?;
                        path.push(cur);
                    }
                    path.reverse();
                    let mut path: Vec<String> = path.into_iter().map(str::to_owned).collect();
                    let min = path
                        .iter()
                        .enumerate()
                        .min_by(|a, b| a.1.cmp(b.1))
                        .map_or(0, |(i, _)| i);
                    path.rotate_left(min);
                    return Some(path);
                }
                if !prev.contains_key(v.as_str()) {
                    prev.insert(v, u);
                    queue.push_back(v);
                }
            }
        }
        None
    }
}

impl ParameterTable {
    /// An empty table.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The parameters in insertion order.
    #[must_use]
    pub fn params(&self) -> &[Parameter] {
        &self.params
    }

    /// Looks a parameter up by name.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Parameter> {
        self.params.iter().find(|p| p.name == name)
    }

    fn get_mut(&mut self, name: &str) -> Result<&mut Parameter, ParamError> {
        self.params
            .iter_mut()
            .find(|p| p.name == name)
            .ok_or_else(|| ParamError::NotFound(name.to_owned()))
    }

    /// Adds a parameter without evaluating it.
    ///
    /// # Errors
    /// [`ParamError::InvalidName`] or [`ParamError::Duplicate`].
    pub fn add(&mut self, p: Parameter) -> Result<(), ParamError> {
        if !is_valid_name(&p.name) {
            return Err(ParamError::InvalidName(p.name));
        }
        if self.get(&p.name).is_some() {
            return Err(ParamError::Duplicate(p.name));
        }
        self.params.push(p);
        Ok(())
    }

    /// Replaces a parameter's expression. Unparsable text is stored; evaluation
    /// reports it.
    ///
    /// # Errors
    /// [`ParamError::NotFound`].
    pub fn set_expression(&mut self, name: &str, expression: String) -> Result<(), ParamError> {
        self.get_mut(name)?.expression = expression;
        Ok(())
    }

    /// Replaces a parameter's comment.
    ///
    /// # Errors
    /// [`ParamError::NotFound`].
    pub fn set_comment(&mut self, name: &str, comment: String) -> Result<(), ParamError> {
        self.get_mut(name)?.comment = comment;
        Ok(())
    }

    /// Renames a parameter and rewrites every expression that reads it. Only
    /// the identifier text changes; unparsable expressions are left alone.
    ///
    /// # Errors
    /// [`ParamError::NotFound`], [`ParamError::InvalidName`] or
    /// [`ParamError::Duplicate`].
    pub fn rename(&mut self, old: &str, new: &str) -> Result<(), ParamError> {
        if self.get(old).is_none() {
            return Err(ParamError::NotFound(old.to_owned()));
        }
        if !is_valid_name(new) {
            return Err(ParamError::InvalidName(new.to_owned()));
        }
        if old == new {
            return Ok(());
        }
        if self.get(new).is_some() {
            return Err(ParamError::Duplicate(new.to_owned()));
        }
        for p in &mut self.params {
            if let Ok(expr) = Expr::parse(&p.expression) {
                let mut text = p.expression.clone();
                for (n, span) in expr.identifier_spans().into_iter().rev() {
                    if n == old {
                        text.replace_range(span, new);
                    }
                }
                p.expression = text;
            }
            if p.name == old {
                new.clone_into(&mut p.name);
            }
        }
        Ok(())
    }

    /// Removes a parameter.
    ///
    /// # Errors
    /// [`ParamError::NotFound`], or [`ParamError::InUse`] if other parameters
    /// read it.
    pub fn remove(&mut self, name: &str) -> Result<Parameter, ParamError> {
        let idx = self
            .params
            .iter()
            .position(|p| p.name == name)
            .ok_or_else(|| ParamError::NotFound(name.to_owned()))?;
        let graph = self.graph();
        let users: Vec<String> = graph
            .users
            .get(name)
            .into_iter()
            .flatten()
            .filter(|u| u.as_str() != name)
            .cloned()
            .collect();
        if !users.is_empty() {
            return Err(ParamError::InUse {
                name: name.to_owned(),
                users,
            });
        }
        Ok(self.params.remove(idx))
    }

    /// The next free model name: `d1`, `d2`, ... (smallest unused).
    #[must_use]
    pub fn next_model_name(&self) -> String {
        (1u32..=u32::MAX)
            .map(|n| format!("d{n}"))
            .find(|n| self.get(n).is_none())
            .unwrap_or_default()
    }

    fn graph(&self) -> Graph {
        let names: BTreeSet<&str> = self.params.iter().map(|p| p.name.as_str()).collect();
        let mut parsed = BTreeMap::new();
        let mut deps = BTreeMap::new();
        let mut users: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for p in &self.params {
            let r = Expr::parse(&p.expression);
            let d: BTreeSet<String> = r
                .as_ref()
                .map(|e| {
                    e.identifiers()
                        .into_iter()
                        .filter(|i| names.contains(i.as_str()))
                        .collect()
                })
                .unwrap_or_default();
            for x in &d {
                users.entry(x.clone()).or_default().insert(p.name.clone());
            }
            deps.insert(p.name.clone(), d);
            parsed.insert(p.name.clone(), r);
        }
        Graph {
            parsed,
            deps,
            users,
        }
    }

    /// Evaluates every parameter in dependency order. Never fails as a whole:
    /// each parameter gets its own result. Parameters on a cycle get
    /// [`ExprErrorKind::Cycle`]; parameters reading a failed one get
    /// [`ExprErrorKind::DependsOnError`].
    #[must_use]
    pub fn evaluate(&self, units: Units) -> Evaluated {
        let g = self.graph();
        let mut results: BTreeMap<String, Result<Quantity, ExprError>> = BTreeMap::new();

        let (_, stuck) = g.topo(&BTreeSet::new());
        let mut cyclic = BTreeSet::new();
        for n in &stuck {
            if let Some(path) = g.find_cycle(n, &stuck) {
                let len = self.get(n).map_or(0, |p| p.expression.len());
                results.insert(
                    n.clone(),
                    Err(ExprError::new(0..len, ExprErrorKind::Cycle(path))),
                );
                cyclic.insert(n.clone());
            }
        }

        let (order, _) = g.topo(&cyclic);
        for n in order {
            let r = match g.parsed.get(&n) {
                None => continue,
                Some(Err(e)) => Err(e.clone()),
                Some(Ok(expr)) => {
                    let failed = g
                        .deps
                        .get(&n)
                        .into_iter()
                        .flatten()
                        .find(|d| matches!(results.get(*d), Some(Err(_))));
                    if let Some(d) = failed {
                        let span = expr
                            .identifier_spans()
                            .into_iter()
                            .find(|(i, _)| i == d)
                            .map_or(0..expr.source().len(), |(_, s)| s);
                        Err(ExprError::new(
                            span,
                            ExprErrorKind::DependsOnError(d.clone()),
                        ))
                    } else {
                        let lookup =
                            |name: &str| results.get(name).and_then(|r| r.as_ref().ok().copied());
                        expr.eval(&EvalContext {
                            units,
                            lookup: &lookup,
                        })
                    }
                }
            };
            results.insert(n, r);
        }
        Evaluated { results }
    }

    /// Names of the parameters that (transitively) read `name`, sorted.
    #[must_use]
    pub fn dependents(&self, name: &str) -> BTreeSet<String> {
        let g = self.graph();
        let mut seen = BTreeSet::new();
        let mut queue = VecDeque::from([name.to_owned()]);
        while let Some(n) = queue.pop_front() {
            for u in g.users.get(&n).into_iter().flatten() {
                if seen.insert(u.clone()) {
                    queue.push_back(u.clone());
                }
            }
        }
        seen.remove(name);
        seen
    }
}
