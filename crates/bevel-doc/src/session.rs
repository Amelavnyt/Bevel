// SPDX-License-Identifier: GPL-3.0-or-later

//! A document being edited: the current state and its undo history.

use crate::command::{Command, CommandError};
use crate::state::DocState;

/// Undo steps kept; older ones are dropped. States share almost
/// everything, so each costs little.
pub const UNDO_LIMIT: usize = 200;

/// A document being edited.
#[derive(Clone, Debug, Default)]
pub struct DocSession {
    state: DocState,
    undo: Vec<(DocState, String)>,
    redo: Vec<(DocState, String)>,
    /// Whether there are changes since the last save.
    modified: bool,
}

impl DocSession {
    /// A session on `state`, with no history and nothing modified.
    pub fn new(state: DocState) -> Self {
        Self {
            state,
            ..Self::default()
        }
    }

    /// The current state.
    pub fn state(&self) -> &DocState {
        &self.state
    }

    /// Runs `cmd`. Marker moves change the state without an undo step.
    ///
    /// # Errors
    ///
    /// Why the command was refused; nothing changes.
    pub fn execute(&mut self, cmd: &Command) -> Result<(), CommandError> {
        let next = cmd.apply(&self.state)?;
        if cmd.is_marker_move() {
            self.state = next;
        } else {
            let label = cmd.label(&self.state);
            let previous = std::mem::replace(&mut self.state, next);
            self.undo.push((previous, label));
            if self.undo.len() > UNDO_LIMIT {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
        self.modified = true;
        Ok(())
    }

    /// The label of the step [`DocSession::undo`] would undo.
    pub fn undo_label(&self) -> Option<&str> {
        self.undo.last().map(|(_, l)| l.as_str())
    }

    /// The label of the step [`DocSession::redo`] would redo.
    pub fn redo_label(&self) -> Option<&str> {
        self.redo.last().map(|(_, l)| l.as_str())
    }

    /// Undoes the last step and returns its label.
    pub fn undo(&mut self) -> Option<String> {
        let (previous, label) = self.undo.pop()?;
        let current = std::mem::replace(&mut self.state, previous);
        self.redo.push((current, label.clone()));
        self.modified = true;
        Some(label)
    }

    /// Redoes the last undone step and returns its label.
    pub fn redo(&mut self) -> Option<String> {
        let (next, label) = self.redo.pop()?;
        let current = std::mem::replace(&mut self.state, next);
        self.undo.push((current, label.clone()));
        self.modified = true;
        Some(label)
    }

    /// Whether there are changes since the session started or was saved.
    pub fn is_modified(&self) -> bool {
        self.modified
    }

    /// Records that the document was saved.
    pub fn mark_saved(&mut self) {
        self.modified = false;
    }
}
