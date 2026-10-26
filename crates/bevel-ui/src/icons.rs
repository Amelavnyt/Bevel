// SPDX-License-Identifier: GPL-3.0-or-later

//! The icon set: SVGs in `assets/icons/`, embedded at compile time and
//! rasterized by egui's image loaders at the size they are shown.

macro_rules! icons {
    ($($variant:ident => $file:literal,)*) => {
        /// An icon from `assets/icons/`.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[allow(missing_docs)]
        pub enum Icon {
            $($variant,)*
        }

        impl Icon {
            /// Every icon, for tests and previews.
            pub const ALL: &[Self] = &[$(Self::$variant,)*];

            /// The embedded SVG.
            pub fn source(self) -> egui::ImageSource<'static> {
                match self {
                    $(Self::$variant => egui::include_image!(concat!("../../../assets/icons/", $file, ".svg")),)*
                }
            }
        }
    };
}

icons! {
    Axis => "axis",
    Body => "body",
    Box => "box",
    Branch => "branch",
    Chamfer => "chamfer",
    ChevronDown => "chevron_down",
    Combine => "combine",
    Cone => "cone",
    Cylinder => "cylinder",
    Document => "document",
    Draft => "draft",
    Extrude => "extrude",
    Eye => "eye",
    EyeOff => "eye_off",
    Fillet => "fillet",
    First => "first",
    Fit => "fit",
    Folder => "folder",
    Grid => "grid",
    Hole => "hole",
    Home => "home",
    Insert => "insert",
    Last => "last",
    Loft => "loft",
    Log => "log",
    Logo => "logo",
    Mass => "mass",
    Measure => "measure",
    Mirror => "mirror",
    Move => "move",
    New => "new",
    Next => "next",
    Open => "open",
    Origin => "origin",
    Orthographic => "orthographic",
    Parameters => "parameters",
    PatternCircular => "pattern_circ",
    PatternRectangular => "pattern_rect",
    Perspective => "perspective",
    Plane => "plane",
    Point => "point",
    Prev => "prev",
    Redo => "redo",
    Revolve => "revolve",
    Save => "save",
    ScriptEditor => "script_editor",
    ScriptRun => "script_run",
    Section => "section",
    Shell => "shell",
    Sketch => "sketch",
    Sphere => "sphere",
    SplitBody => "split_body",
    Sweep => "sweep",
    Thread => "thread",
    Torus => "torus",
    Undo => "undo",
}

impl Icon {
    /// The icon as an image of `size` points square.
    pub fn image(self, size: f32) -> egui::Image<'static> {
        egui::Image::new(self.source()).fit_to_exact_size(egui::vec2(size, size))
    }
}
