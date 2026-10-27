// SPDX-License-Identifier: GPL-3.0-or-later

//! The modeling tools shown in the toolbar, grouped the way parametric CAD
//! tools usually group them. Sketching and inspection are ordinary tools of
//! the one design workspace, not separate modes.

use crate::icons::Icon;
use bevel_model::PrimitiveKind;

/// A toolbar group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolGroup {
    /// Sketches, features that add material, primitives and patterns.
    Create,
    /// Features that change existing bodies.
    Modify,
    /// Reference planes, axes and points.
    Construct,
    /// Measurement and analysis.
    Inspect,
    /// Bringing in geometry from files.
    Insert,
    /// Scripts and macros.
    Utilities,
}

impl ToolGroup {
    /// All groups, in toolbar order.
    pub const ALL: [Self; 6] = [
        Self::Create,
        Self::Modify,
        Self::Construct,
        Self::Inspect,
        Self::Insert,
        Self::Utilities,
    ];

    /// Label shown under the group's icons.
    pub fn label(self) -> &'static str {
        match self {
            Self::Create => "CREATE",
            Self::Modify => "MODIFY",
            Self::Construct => "CONSTRUCT",
            Self::Inspect => "INSPECT",
            Self::Insert => "INSERT",
            Self::Utilities => "UTILITIES",
        }
    }
}

/// One tool in the toolbar.
#[derive(Clone, Copy, Debug)]
pub struct Tool {
    /// Display name.
    pub name: &'static str,
    /// Toolbar group.
    pub group: ToolGroup,
    /// Icon.
    pub icon: Icon,
    /// Keyboard shortcut, if any.
    pub shortcut: Option<&'static str>,
    /// One-line description for the tooltip.
    pub description: &'static str,
    /// Shown directly in the toolbar; other tools are in the group's menu.
    pub pinned: bool,
    /// Roadmap phase that implements it.
    pub phase: &'static str,
}

const fn tool(
    name: &'static str,
    group: ToolGroup,
    icon: Icon,
    shortcut: Option<&'static str>,
    pinned: bool,
    phase: &'static str,
    description: &'static str,
) -> Tool {
    Tool {
        name,
        group,
        icon,
        shortcut,
        description,
        pinned,
        phase,
    }
}

use ToolGroup::{Construct, Create, Insert, Inspect, Modify, Utilities};

const SKETCHER: &str = "Phase 5: Sketcher";
const EXTRUDE: &str = "Phase 6: Extrude, revolve & naming";
const PRIMITIVES: &str = "Phase 3: Document & timeline";
const SWEEPS: &str = "Phase 9: Sweeps & patterns";
const HOLES: &str = "Phase 10: Holes & threads";
const BLENDS: &str = "Phase 8: Blends & offsets";
const BOOLEANS: &str = "Phase 4: Booleans";
const INSPECT: &str = "Phase 7: Inspect";
const STEP: &str = "Phase 11: STEP";
const SCRIPTING: &str = "Phase 13: Scripting";

/// Every tool, in menu order within each group.
#[rustfmt::skip]
pub const TOOLS: &[Tool] = &[
    tool("Create Sketch", Create, Icon::Sketch, Some("S"), true, SKETCHER, "Start a 2D sketch on a plane or planar face"),
    tool("Extrude", Create, Icon::Extrude, Some("E"), true, EXTRUDE, "Add or remove material by pushing a sketch profile along a direction"),
    tool("Revolve", Create, Icon::Revolve, None, true, EXTRUDE, "Sweep a profile around an axis"),
    tool("Sweep", Create, Icon::Sweep, None, false, SWEEPS, "Move a profile along a path"),
    tool("Loft", Create, Icon::Loft, None, false, SWEEPS, "Blend between two or more profiles"),
    tool("Hole", Create, Icon::Hole, Some("H"), true, HOLES, "Drill simple, counterbored, countersunk or tapped holes"),
    tool("Thread", Create, Icon::Thread, None, false, HOLES, "Add cosmetic or modeled threads to a cylindrical face"),
    tool("Box", Create, Icon::Box, None, true, PRIMITIVES, "Create a box body"),
    tool("Cylinder", Create, Icon::Cylinder, None, false, PRIMITIVES, "Create a cylinder body"),
    tool("Sphere", Create, Icon::Sphere, None, false, PRIMITIVES, "Create a sphere body"),
    tool("Cone", Create, Icon::Cone, None, false, PRIMITIVES, "Create a cone body"),
    tool("Torus", Create, Icon::Torus, None, false, PRIMITIVES, "Create a torus body"),
    tool("Rectangular Pattern", Create, Icon::PatternRectangular, None, false, SWEEPS, "Repeat features or bodies along one or two directions"),
    tool("Circular Pattern", Create, Icon::PatternCircular, None, false, SWEEPS, "Repeat features or bodies around an axis"),
    tool("Mirror", Create, Icon::Mirror, None, false, SWEEPS, "Mirror features or bodies across a plane"),
    tool("Fillet", Modify, Icon::Fillet, Some("F"), true, BLENDS, "Round edges with a constant radius"),
    tool("Chamfer", Modify, Icon::Chamfer, None, true, BLENDS, "Bevel edges by distance or angle"),
    tool("Shell", Modify, Icon::Shell, None, true, BLENDS, "Hollow a body, removing the selected faces"),
    tool("Draft", Modify, Icon::Draft, None, false, BLENDS, "Tilt faces relative to a neutral plane"),
    tool("Change Parameters", Modify, Icon::Parameters, None, false, PRIMITIVES, "Edit the parameters that drive the model"),
    tool("Combine", Modify, Icon::Combine, None, true, BOOLEANS, "Join, cut or intersect bodies"),
    tool("Split Body", Modify, Icon::SplitBody, None, false, SWEEPS, "Split a body with a plane or face"),
    tool("Move/Copy", Modify, Icon::Move, None, false, SWEEPS, "Move or copy bodies"),
    tool("Offset Plane", Construct, Icon::Plane, None, true, EXTRUDE, "Create a reference plane offset from a plane or face"),
    tool("Axis", Construct, Icon::Axis, None, true, EXTRUDE, "Create a reference axis"),
    tool("Point", Construct, Icon::Point, None, false, EXTRUDE, "Create a reference point"),
    tool("Measure", Inspect, Icon::Measure, Some("M"), true, INSPECT, "Measure the area, length, volume and centre of mass of the selection"),
    tool("Section Analysis", Inspect, Icon::Section, None, true, INSPECT, "Cut the view with a plane to look inside"),
    tool("Mass Properties", Inspect, Icon::Mass, None, false, INSPECT, "Volume, area, centre of mass and inertia"),
    tool("Insert STEP", Insert, Icon::Insert, None, true, STEP, "Import bodies from a STEP file"),
    tool("Run Script", Utilities, Icon::ScriptRun, None, true, SCRIPTING, "Run a Rhai script or macro"),
    tool("Script Editor", Utilities, Icon::ScriptEditor, None, false, SCRIPTING, "Write and debug scripts"),
];

/// Name of the Measure tool, which works today (distances and angles
/// between selections arrive in Phase 7).
pub const MEASURE: &str = "Measure";

/// Name of the Change Parameters tool.
pub const PARAMETERS: &str = "Change Parameters";

/// The primitive a Create tool makes, if it is one.
pub fn primitive(tool: &Tool) -> Option<PrimitiveKind> {
    PrimitiveKind::ALL
        .into_iter()
        .find(|k| k.name() == tool.name)
}

/// Whether `tool` can be used now.
pub fn is_available(tool: &Tool) -> bool {
    tool.name == MEASURE || tool.name == PARAMETERS || primitive(tool).is_some()
}

/// The tools of one group, in menu order.
pub fn in_group(group: ToolGroup) -> impl Iterator<Item = &'static Tool> {
    TOOLS.iter().filter(move |t| t.group == group)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_group_has_a_pinned_tool() {
        for group in ToolGroup::ALL {
            assert!(
                in_group(group).any(|t| t.pinned),
                "{group:?} has nothing in the toolbar"
            );
        }
    }

    #[test]
    fn tool_names_and_shortcuts_are_unique() {
        let mut names: Vec<_> = TOOLS.iter().map(|t| t.name).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), TOOLS.len());
        let mut keys: Vec<_> = TOOLS.iter().filter_map(|t| t.shortcut).collect();
        let count = keys.len();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(keys.len(), count);
    }
}
