// SPDX-License-Identifier: GPL-3.0-or-later

//! The error type shared by every reader and writer in this crate.

/// Everything that can go wrong while reading or writing a mesh file.
///
/// Messages are written for users; the wrapped lower-level errors carry the
/// technical detail.
#[derive(Debug, thiserror::Error)]
pub enum IoError {
    /// The underlying reader or writer failed.
    #[error("could not read or write the file: {0}")]
    Io(#[from] std::io::Error),
    /// The ZIP container of a 3MF package is damaged or could not be written.
    #[error("the 3MF package (a ZIP archive) is invalid: {0}")]
    Zip(#[from] zip::result::ZipError),
    /// An XML part of a 3MF package is malformed.
    #[error("the XML inside the file is malformed: {0}")]
    Xml(#[from] quick_xml::Error),
    /// A mesh cannot be exported because of its contents.
    #[error("mesh \"{mesh}\" cannot be exported: {reason}")]
    InvalidMesh {
        /// Name of the offending mesh.
        mesh: String,
        /// What is wrong with it.
        reason: String,
    },
    /// A file being read does not follow its format.
    #[error("this is not a valid {format} file: {reason}")]
    InvalidFile {
        /// The format that was expected, such as "STL" or "3MF".
        format: &'static str,
        /// What is wrong with the file.
        reason: String,
    },
}

impl IoError {
    pub(crate) fn mesh(mesh: &str, reason: impl Into<String>) -> Self {
        Self::InvalidMesh {
            mesh: mesh.to_owned(),
            reason: reason.into(),
        }
    }

    pub(crate) fn file(format: &'static str, reason: impl Into<String>) -> Self {
        Self::InvalidFile {
            format,
            reason: reason.into(),
        }
    }
}
