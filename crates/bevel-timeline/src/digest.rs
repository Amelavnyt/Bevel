// SPDX-License-Identifier: GPL-3.0-or-later

//! Content hashes for steps and cache keys.

use std::fmt;

/// A 256-bit BLAKE3 hash.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Digest(pub [u8; 32]);

impl Digest {
    /// The hash of `bytes`.
    pub fn of(bytes: &[u8]) -> Self {
        Self(*blake3::hash(bytes).as_bytes())
    }

    /// Lower-case hexadecimal, 64 characters.
    pub fn to_hex(&self) -> String {
        use fmt::Write as _;
        let mut out = String::with_capacity(64);
        for b in self.0 {
            // Writing to a String cannot fail.
            let _ = write!(out, "{b:02x}");
        }
        out
    }

    /// Parses 64 hexadecimal characters.
    pub fn from_hex(text: &str) -> Option<Self> {
        let bytes = text.as_bytes();
        if bytes.len() != 64 {
            return None;
        }
        let mut out = [0u8; 32];
        for (o, pair) in out.iter_mut().zip(bytes.chunks_exact(2)) {
            let s = std::str::from_utf8(pair).ok()?;
            *o = u8::from_str_radix(s, 16).ok()?;
        }
        Some(Self(out))
    }
}

impl fmt::Debug for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Digest({})", &self.to_hex()[..12])
    }
}

impl fmt::Display for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

/// Builds a [`Digest`] from several parts. Each part is length-prefixed, so
/// `("ab", "c")` and `("a", "bc")` hash differently.
pub struct DigestBuilder(blake3::Hasher);

impl DigestBuilder {
    /// A builder for hashes in `domain`, so equal bytes hashed for different
    /// purposes never collide.
    pub fn new(domain: &str) -> Self {
        let mut b = Self(blake3::Hasher::new());
        b.bytes(domain.as_bytes());
        b
    }

    /// Adds a byte string.
    pub fn bytes(&mut self, bytes: &[u8]) -> &mut Self {
        self.0.update(&(bytes.len() as u64).to_le_bytes());
        self.0.update(bytes);
        self
    }

    /// Adds a digest.
    pub fn digest(&mut self, d: &Digest) -> &mut Self {
        self.bytes(&d.0)
    }

    /// Adds an integer.
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.bytes(&v.to_le_bytes())
    }

    /// Adds a float by its bits, so equal values hash equally and `-0.0`
    /// differs from `0.0`.
    pub fn f64(&mut self, v: f64) -> &mut Self {
        self.bytes(&v.to_bits().to_le_bytes())
    }

    /// The finished hash.
    pub fn finish(&self) -> Digest {
        Digest(*self.0.finalize().as_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_round_trips() {
        let d = Digest::of(b"bevel");
        assert_eq!(Digest::from_hex(&d.to_hex()), Some(d));
        assert_eq!(Digest::from_hex("zz"), None);
    }

    #[test]
    fn parts_are_length_prefixed() {
        let a = DigestBuilder::new("t").bytes(b"ab").bytes(b"c").finish();
        let b = DigestBuilder::new("t").bytes(b"a").bytes(b"bc").finish();
        assert_ne!(a, b);
        let c = DigestBuilder::new("u").bytes(b"ab").bytes(b"c").finish();
        assert_ne!(a, c);
    }
}
