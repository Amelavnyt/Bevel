// SPDX-License-Identifier: GPL-3.0-or-later

//! Documents, commands and history.
//!
//! `DocState` is an immutable, cheaply cloned snapshot of a document. Every
//! change is a serializable `Command` that produces a new state; undo and
//! redo keep previous states. Versions and branches live in a
//! content-addressed object store that also backs the `.bevel` file format.
//!
//! Status: placeholder; implemented in the "Document & timeline" phase.
