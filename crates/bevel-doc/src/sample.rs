// SPDX-License-Identifier: GPL-3.0-or-later

//! Sample documents.

use crate::command::add_primitive;
use crate::state::DocState;
use bevel_model::{OriginPlane, PrimitiveKind};
use std::collections::BTreeMap;

/// The five primitives in a row along X, each its own timeline step.
pub fn primitives() -> DocState {
    let rows: [(PrimitiveKind, &[(&str, &str)]); 5] = [
        (
            PrimitiveKind::Box,
            &[
                ("length", "40 mm"),
                ("width", "40 mm"),
                ("height", "40 mm"),
                ("x", "-150 mm"),
                ("y", "-20 mm"),
            ],
        ),
        (
            PrimitiveKind::Cylinder,
            &[("radius", "20 mm"), ("height", "40 mm"), ("x", "-75 mm")],
        ),
        (
            PrimitiveKind::Sphere,
            &[("radius", "22 mm"), ("z", "22 mm")],
        ),
        (
            PrimitiveKind::Cone,
            &[
                ("bottom_radius", "22 mm"),
                ("top_radius", "10 mm"),
                ("height", "40 mm"),
                ("x", "75 mm"),
            ],
        ),
        (
            PrimitiveKind::Torus,
            &[
                ("major_radius", "21 mm"),
                ("minor_radius", "9 mm"),
                ("x", "150 mm"),
                ("z", "9 mm"),
            ],
        ),
    ];
    let mut state = DocState::new();
    for (kind, values) in rows {
        let values: BTreeMap<String, String> = values
            .iter()
            .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
            .collect();
        let cmd = add_primitive(&state, kind, OriginPlane::Xy, &values);
        // The sample's commands are valid by construction.
        if let Ok(next) = cmd.apply(&state) {
            state = next;
        }
    }
    state
}
