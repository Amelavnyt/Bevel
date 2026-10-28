// SPDX-License-Identifier: GPL-3.0-or-later

//! Debug dumps: the inputs of a failed operation, as JSON that
//! `bevel-cli kernel-repro` can replay.
//!
//! The kernel never writes files itself (it also builds for the web); the
//! caller decides where a dump goes and may use [`BooleanDump::file_name`]
//! for the name.

use serde::{Deserialize, Serialize};

use crate::boolean::{BooleanOp, boolean};
use crate::error::KernelError;
use crate::naming::{OpContext, OpOutput, OpTag};
use crate::topology::Body;
use crate::validate::validate;

/// The version of the dump format that [`BooleanDump`] writes and reads.
pub const DUMP_FORMAT: u32 = 1;

/// The inputs of a failed boolean operation, enough to replay it.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BooleanDump {
    /// The format version, [`DUMP_FORMAT`].
    pub format: u32,
    /// Which operation was run.
    pub op: BooleanOp,
    /// The number of the [`OpTag`] the operation ran with.
    pub tag: u64,
    /// The [`OpContext::tolerance`] the operation ran with.
    pub tolerance: f64,
    /// The body the operation modified.
    pub target: Body,
    /// The body that acted on the target.
    pub tool: Body,
    /// The text of the error the operation returned.
    pub error: String,
}

impl BooleanDump {
    /// Records the inputs of `boolean(target, tool, op, tag, cx)` and the
    /// `error` it returned.
    pub fn new(
        target: &Body,
        tool: &Body,
        op: BooleanOp,
        tag: OpTag,
        cx: &OpContext,
        error: &KernelError,
    ) -> Self {
        Self {
            format: DUMP_FORMAT,
            op,
            tag: tag.0,
            tolerance: cx.tolerance,
            target: target.clone(),
            tool: tool.clone(),
            error: error.to_string(),
        }
    }

    /// The dump as pretty-printed JSON.
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidInput`] if serialization fails.
    pub fn to_json(&self) -> Result<String, KernelError> {
        serde_json::to_string_pretty(self).map_err(|_| KernelError::InvalidInput {
            what: "boolean dump",
            reason: "it cannot be written as JSON",
        })
    }

    /// Reads a dump written by [`BooleanDump::to_json`].
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidInput`] if the text is not valid JSON of the
    /// right shape, has an unknown format version, or holds a body that
    /// fails validation (bodies are validated while they are read, so a bad
    /// file never reaches an operation).
    pub fn from_json(text: &str) -> Result<Self, KernelError> {
        let dump: Self = serde_json::from_str(text).map_err(|e| KernelError::InvalidInput {
            what: "boolean dump",
            reason: if e.is_syntax() || e.is_eof() {
                "the text is not valid JSON"
            } else {
                "the data is malformed, or a body in it is invalid"
            },
        })?;
        if dump.format != DUMP_FORMAT {
            return Err(KernelError::InvalidInput {
                what: "boolean dump",
                reason: "the format version is not supported",
            });
        }
        if !validate(&dump.target).is_empty() || !validate(&dump.tool).is_empty() {
            return Err(KernelError::InvalidInput {
                what: "boolean dump",
                reason: "a body in it is invalid",
            });
        }
        Ok(dump)
    }

    /// A file name for the dump: `bevel-debug-boolean-` and a 64-bit
    /// FNV-1a hash of its JSON in hex, so equal dumps share a name.
    pub fn file_name(&self) -> String {
        let hash = self.to_json().map_or(0, |json| fnv1a(json.as_bytes()));
        format!("bevel-debug-boolean-{hash:016x}.json")
    }

    /// Runs the recorded operation again, with the recorded tag and
    /// tolerance.
    ///
    /// # Errors
    ///
    /// Whatever [`boolean`] returns.
    pub fn replay(&self) -> Result<OpOutput, KernelError> {
        let mut cx = OpContext {
            tolerance: self.tolerance,
            ..OpContext::default()
        };
        boolean(&self.target, &self.tool, self.op, OpTag(self.tag), &mut cx)
    }
}

/// The 64-bit FNV-1a hash of `bytes`.
fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

    use super::*;
    use crate::{BoxParams, box_, mass_properties};
    use bevel_math::{Frame3, Point3, Vector3};

    fn block(origin: [f64; 3], tag: u64) -> Body {
        let params = BoxParams {
            frame: Frame3 {
                origin: Point3::from_array(origin),
                ..Frame3::WORLD
            },
            size: Vector3::from_array([10.0; 3]),
        };
        let out = box_(&params, OpTag(tag), &mut OpContext::new()).unwrap();
        (*out.bodies[0]).clone()
    }

    fn dump() -> BooleanDump {
        let a = block([0.0; 3], 1);
        let b = block([5.0; 3], 2);
        let err = KernelError::Cancelled;
        BooleanDump::new(&a, &b, BooleanOp::Union, OpTag(7), &OpContext::new(), &err)
    }

    fn volume(out: &OpOutput) -> f64 {
        out.bodies
            .iter()
            .map(|b| mass_properties(b, &OpContext::new()).unwrap().volume)
            .sum()
    }

    #[test]
    fn round_trips_and_replays() {
        let d = dump();
        let json = d.to_json().unwrap();
        let back = BooleanDump::from_json(&json).unwrap();
        assert_eq!(back.to_json().unwrap(), json);
        assert!(validate(&back.target).is_empty());
        assert!(validate(&back.tool).is_empty());
        assert_eq!(back.error, "the operation was cancelled");
        assert_eq!(back.file_name(), d.file_name());
        assert!(d.file_name().starts_with("bevel-debug-boolean-"));
        assert_eq!(d.file_name().len(), "bevel-debug-boolean-.json".len() + 16);

        let direct = boolean(
            &d.target,
            &d.tool,
            BooleanOp::Union,
            OpTag(7),
            &mut OpContext::new(),
        )
        .unwrap();
        let replayed = back.replay().unwrap();
        assert!((volume(&direct) - volume(&replayed)).abs() < 1e-9);
        assert!((volume(&replayed) - 1875.0).abs() < 1e-6);
    }

    #[test]
    fn rejects_bad_files() {
        let json = dump().to_json().unwrap();
        let v2 = json.replacen("\"format\": 1", "\"format\": 2", 1);
        assert_ne!(v2, json);
        assert!(BooleanDump::from_json(&v2).is_err());
        assert!(BooleanDump::from_json("not json").is_err());
        assert!(BooleanDump::from_json("{}").is_err());
        assert!(BooleanDump::from_json(&json[..json.len() / 2]).is_err());
    }
}
