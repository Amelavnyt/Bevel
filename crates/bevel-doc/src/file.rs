// SPDX-License-Identifier: GPL-3.0-or-later

//! The `.bevel` file format (see `docs/FILE_FORMAT.md`).
//!
//! A ZIP container:
//!
//! - `manifest.json`: format name, `format_version`, app version, `head`.
//! - `objects/<hash>`: content-addressed JSON objects (features, the
//!   parameter table), named by the BLAKE3 hash of their bytes.
//! - `working.json`: the working state, pointing at objects.
//!
//! Commits, branches and tags (`refs/...`) arrive with versions; the
//! object store is already the storage they will use.

use crate::state::{BodyProps, DocState};
use bevel_expr::{ParameterTable, Units};
use bevel_model::{BodyId, Feature, Part};
use bevel_timeline::{Digest, Step, Timeline};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::io::{Cursor, Read, Write};
use std::sync::Arc;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipArchive, ZipWriter};

/// The format version this build writes.
pub const FORMAT_VERSION: u32 = 1;
const FORMAT_NAME: &str = "bevel";
/// Largest entry the reader will inflate, against decompression bombs.
const MAX_ENTRY_BYTES: u64 = 1 << 28;

/// Why a file could not be read or written.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FileError {
    /// The container is not a readable ZIP archive.
    #[error("not a readable .bevel file: {0}")]
    Container(String),
    /// An entry is not valid JSON of the expected shape.
    #[error("{entry} is malformed: {message}")]
    Malformed {
        /// The entry.
        entry: String,
        /// What is wrong.
        message: String,
    },
    /// The file was written by a newer version of Bevel.
    #[error(
        "this file uses format version {found}, which needs a newer Bevel (this one reads up to {supported})"
    )]
    NewerVersion {
        /// The file's version.
        found: u32,
        /// The newest version this build reads.
        supported: u32,
    },
    /// An object is missing or does not match its hash.
    #[error("the file is damaged: {0}")]
    Corrupt(String),
}

fn malformed(entry: &str, e: impl std::fmt::Display) -> FileError {
    FileError::Malformed {
        entry: entry.to_owned(),
        message: e.to_string(),
    }
}

fn container(e: impl std::fmt::Display) -> FileError {
    FileError::Container(e.to_string())
}

/// Content-addressed storage: each object is stored under the hash of its
/// bytes, so identical objects are stored once.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ObjectStore {
    objects: BTreeMap<Digest, Arc<Vec<u8>>>,
}

impl ObjectStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Stores `bytes` and returns their hash.
    pub fn put(&mut self, bytes: Vec<u8>) -> Digest {
        let hash = Digest::of(&bytes);
        self.objects.entry(hash).or_insert_with(|| Arc::new(bytes));
        hash
    }

    /// The object with hash `hash`.
    pub fn get(&self, hash: &Digest) -> Option<&[u8]> {
        self.objects.get(hash).map(|b| b.as_slice())
    }

    /// The number of objects.
    pub fn len(&self) -> usize {
        self.objects.len()
    }

    /// Whether the store is empty.
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    /// Every object, in hash order.
    pub fn iter(&self) -> impl Iterator<Item = (&Digest, &[u8])> {
        self.objects.iter().map(|(h, b)| (h, b.as_slice()))
    }
}

#[derive(Serialize, Deserialize)]
struct Manifest {
    format: String,
    format_version: u32,
    app_version: String,
    head: String,
}

#[derive(Serialize, Deserialize)]
struct WorkingStep {
    object: String,
    suppressed: bool,
}

#[derive(Serialize, Deserialize)]
struct BodyEntry {
    body: BodyId,
    props: BodyProps,
}

#[derive(Serialize, Deserialize)]
struct Working {
    features: Vec<WorkingStep>,
    marker: usize,
    parameters: String,
    units: Units,
    body_props: Vec<BodyEntry>,
    next_feature_id: u64,
}

fn to_json<T: Serialize>(entry: &str, value: &T) -> Result<Vec<u8>, FileError> {
    serde_json::to_vec(value).map_err(|e| malformed(entry, e))
}

fn to_json_pretty<T: Serialize>(entry: &str, value: &T) -> Result<Vec<u8>, FileError> {
    serde_json::to_vec_pretty(value).map_err(|e| malformed(entry, e))
}

/// Writes `state` as a `.bevel` file. The output is deterministic: the
/// same document always gives the same bytes.
///
/// # Errors
///
/// A [`FileError`] if serializing or compressing fails.
pub fn write(state: &DocState) -> Result<Vec<u8>, FileError> {
    let mut store = ObjectStore::new();
    let part = &state.part;
    let mut features = Vec::new();
    for step in part.features.steps() {
        let hash = store.put(to_json("feature", step.op().as_ref())?);
        features.push(WorkingStep {
            object: hash.to_hex(),
            suppressed: step.suppressed(),
        });
    }
    let parameters = store.put(to_json("parameters", &part.parameters)?);
    let working = Working {
        features,
        marker: part.features.marker(),
        parameters: parameters.to_hex(),
        units: part.units,
        body_props: state
            .body_props
            .iter()
            .map(|(body, props)| BodyEntry {
                body: *body,
                props: props.clone(),
            })
            .collect(),
        next_feature_id: state.next_feature_id,
    };
    let manifest = Manifest {
        format: FORMAT_NAME.to_owned(),
        format_version: FORMAT_VERSION,
        app_version: env!("CARGO_PKG_VERSION").to_owned(),
        head: "refs/heads/main".to_owned(),
    };

    let options = SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .last_modified_time(DateTime::default());
    let mut zip = ZipWriter::new(Cursor::new(Vec::new()));
    let entry = |zip: &mut ZipWriter<Cursor<Vec<u8>>>, name: &str, bytes: &[u8]| {
        zip.start_file(name, options).map_err(container)?;
        zip.write_all(bytes).map_err(container)
    };
    entry(
        &mut zip,
        "manifest.json",
        &to_json_pretty("manifest.json", &manifest)?,
    )?;
    entry(
        &mut zip,
        "working.json",
        &to_json_pretty("working.json", &working)?,
    )?;
    for (hash, bytes) in store.iter() {
        entry(&mut zip, &format!("objects/{}", hash.to_hex()), bytes)?;
    }
    Ok(zip.finish().map_err(container)?.into_inner())
}

fn read_entry(archive: &mut ZipArchive<Cursor<&[u8]>>, name: &str) -> Result<Vec<u8>, FileError> {
    let file = archive.by_name(name).map_err(|e| match e {
        zip::result::ZipError::FileNotFound => FileError::Corrupt(format!("{name} is missing")),
        other => container(other),
    })?;
    let mut data = Vec::new();
    file.take(MAX_ENTRY_BYTES + 1)
        .read_to_end(&mut data)
        .map_err(container)?;
    if data.len() as u64 > MAX_ENTRY_BYTES {
        return Err(FileError::Corrupt(format!("{name} is too large")));
    }
    Ok(data)
}

/// Brings `working.json` from format version `from` to the current one.
/// Every format change adds a step here and a fixture file in
/// `fixtures/bevel/`.
///
/// # Errors
///
/// [`FileError::NewerVersion`] for versions this build does not know.
pub fn migrate(working: Value, from: u32) -> Result<Value, FileError> {
    match from {
        // Version 1 is current; later versions add steps like
        // `1 => migrate(v1_to_v2(working)?, 2)`.
        FORMAT_VERSION => Ok(working),
        found => Err(FileError::NewerVersion {
            found,
            supported: FORMAT_VERSION,
        }),
    }
}

/// Reads a `.bevel` file.
///
/// # Errors
///
/// A [`FileError`] if the file is not a `.bevel` file, is damaged or needs
/// a newer Bevel.
pub fn read(bytes: &[u8]) -> Result<DocState, FileError> {
    let mut archive = ZipArchive::new(Cursor::new(bytes)).map_err(container)?;
    let manifest: Manifest = serde_json::from_slice(&read_entry(&mut archive, "manifest.json")?)
        .map_err(|e| malformed("manifest.json", e))?;
    if manifest.format != FORMAT_NAME {
        return Err(malformed("manifest.json", "not a Bevel document"));
    }
    let working: Value = serde_json::from_slice(&read_entry(&mut archive, "working.json")?)
        .map_err(|e| malformed("working.json", e))?;
    let working: Working = serde_json::from_value(migrate(working, manifest.format_version)?)
        .map_err(|e| malformed("working.json", e))?;

    let mut object = |hex: &str| -> Result<Vec<u8>, FileError> {
        let hash = Digest::from_hex(hex)
            .ok_or_else(|| FileError::Corrupt(format!("bad object name {hex}")))?;
        let data = read_entry(&mut archive, &format!("objects/{hex}"))?;
        if Digest::of(&data) != hash {
            return Err(FileError::Corrupt(format!(
                "object {hex} does not match its hash"
            )));
        }
        Ok(data)
    };

    let mut steps = Vec::new();
    for ws in &working.features {
        let feature: Feature = serde_json::from_slice(&object(&ws.object)?)
            .map_err(|e| malformed(&format!("objects/{}", ws.object), e))?;
        steps.push(Step::new(Arc::new(feature), ws.suppressed));
    }
    let parameters: ParameterTable = serde_json::from_slice(&object(&working.parameters)?)
        .map_err(|e| malformed("parameters", e))?;
    let features = Timeline::from_steps(steps, working.marker)
        .map_err(|e| FileError::Corrupt(e.to_string()))?;
    let max_id = features
        .steps()
        .iter()
        .map(|s| s.op().id.0)
        .max()
        .unwrap_or(0);
    Ok(DocState {
        part: Arc::new(Part {
            features,
            parameters,
            units: working.units,
        }),
        body_props: Arc::new(
            working
                .body_props
                .into_iter()
                .map(|e| (e.body, e.props))
                .collect(),
        ),
        // Never hand out an id a feature already has.
        next_feature_id: working.next_feature_id.max(max_id + 1),
    })
}
