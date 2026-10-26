// SPDX-License-Identifier: GPL-3.0-or-later

//! Errors returned by the numerical routines in this crate.

/// Why a numerical routine could not produce a result.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum MathError {
    /// A vector that must have a direction was zero or too short to normalize.
    #[error("the vector is zero or too short to have a direction")]
    DegenerateVector,
    /// An iterative method used up its iteration budget without converging.
    #[error("the iteration did not converge within {iterations} iterations")]
    NotConverged {
        /// Number of iterations that were performed.
        iterations: usize,
    },
    /// A derivative or Jacobian was zero or too small to solve against.
    #[error("the derivative or Jacobian is singular")]
    SingularJacobian,
    /// An argument was outside the range the routine accepts.
    #[error("invalid input: {0}")]
    InvalidInput(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_are_plain_english() {
        assert_eq!(
            MathError::NotConverged { iterations: 7 }.to_string(),
            "the iteration did not converge within 7 iterations"
        );
        assert_eq!(
            MathError::InvalidInput("n must be positive").to_string(),
            "invalid input: n must be positive"
        );
        assert!(!MathError::DegenerateVector.to_string().is_empty());
        assert!(!MathError::SingularJacobian.to_string().is_empty());
    }
}
