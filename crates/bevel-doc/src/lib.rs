// SPDX-License-Identifier: GPL-3.0-or-later

//! Documents, commands and history.
//!
//! [`DocState`] is an immutable, cheaply cloned snapshot of a document.
//! Every change is a serializable [`Command`] that produces a new state; a
//! [`DocSession`] keeps previous states for undo and redo. Documents are
//! saved as `.bevel` files ([`file`]): a ZIP container around a
//! content-addressed [`ObjectStore`] that versions and branches will build
//! on.

mod command;
pub mod file;
pub mod sample;
mod session;
mod state;

pub use command::{Command, CommandError, ParamEdit, add_primitive};
pub use file::{FileError, ObjectStore};
pub use session::{DocSession, UNDO_LIMIT};
pub use state::{BodyProps, DocState};
