// SPDX-License-Identifier: GPL-3.0-or-later

//! The document state.

use bevel_model::{BodyId, Feature, FeatureId, Part};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A body's display properties.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyProps {
    /// Name; `None` uses the default ("Body1", ...).
    #[serde(default)]
    pub name: Option<String>,
    /// sRGB colour; `None` uses the default grey.
    #[serde(default)]
    pub color: Option<[u8; 3]>,
    /// Whether the body is shown.
    pub visible: bool,
}

impl Default for BodyProps {
    fn default() -> Self {
        Self {
            name: None,
            color: None,
            visible: true,
        }
    }
}

/// An immutable snapshot of a document, cheap to clone: the part and the
/// body properties are shared until a command changes them.
#[derive(Clone, Debug)]
pub struct DocState {
    /// The part (one per document until assemblies arrive).
    pub part: Arc<Part>,
    /// Display properties of bodies that differ from the defaults.
    pub body_props: Arc<BTreeMap<BodyId, BodyProps>>,
    /// The id the next feature gets; ids are never reused.
    pub next_feature_id: u64,
}

impl Default for DocState {
    fn default() -> Self {
        Self {
            part: Arc::new(Part::default()),
            body_props: Arc::new(BTreeMap::new()),
            next_feature_id: 1,
        }
    }
}

impl DocState {
    /// An empty document.
    pub fn new() -> Self {
        Self::default()
    }

    /// Feature `id`.
    pub fn feature(&self, id: FeatureId) -> Option<&Feature> {
        self.part.features.get(id).map(|s| s.op().as_ref())
    }

    /// The part, for changing (copied first if shared).
    pub(crate) fn part_mut(&mut self) -> &mut Part {
        Arc::make_mut(&mut self.part)
    }

    /// The first unused "{kind}{n}" name, e.g. "Box3".
    pub fn next_feature_name(&self, kind: &str) -> String {
        let taken = |name: &str| {
            self.part
                .features
                .steps()
                .iter()
                .any(|s| s.op().name == name)
        };
        // Among n = 1..=len+1 at least one name is free.
        let count = self.part.features.len() as u64 + 1;
        (1..=count)
            .map(|n| format!("{kind}{n}"))
            .find(|name| !taken(name))
            .unwrap_or_else(|| kind.to_owned())
    }

    /// Body `id`'s name: its own if set, otherwise its feature's ("Box1"),
    /// numbered when the feature makes several ("Pattern1 (2)").
    pub fn body_name(&self, id: BodyId) -> String {
        if let Some(name) = self.body_props.get(&id).and_then(|p| p.name.clone()) {
            return name;
        }
        let feature = self
            .feature(id.feature)
            .map_or_else(|| id.feature.to_string(), |f| f.name.clone());
        if id.index == 0 {
            feature
        } else {
            format!("{feature} ({})", id.index + 1)
        }
    }

    /// Body `id`'s display properties.
    pub fn body_props(&self, id: BodyId) -> BodyProps {
        self.body_props.get(&id).cloned().unwrap_or_default()
    }
}
