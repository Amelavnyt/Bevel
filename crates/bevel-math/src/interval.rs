// SPDX-License-Identifier: GPL-3.0-or-later

//! Closed intervals on the real line.

/// A closed interval `[min, max]` with `min <= max`.
///
/// Used for curve and surface parameter ranges and as search bounds in
/// root finding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interval {
    /// Lower end.
    pub min: f64,
    /// Upper end.
    pub max: f64,
}

impl Interval {
    /// The interval between `a` and `b`, in either order.
    pub fn new(a: f64, b: f64) -> Self {
        Self {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// `max - min`.
    pub fn length(&self) -> f64 {
        self.max - self.min
    }

    /// The midpoint.
    pub fn mid(&self) -> f64 {
        0.5 * (self.min + self.max)
    }

    /// Returns `true` if `min <= t <= max`, with no slack at the ends.
    pub fn contains(&self, t: f64) -> bool {
        t >= self.min && t <= self.max
    }

    /// Returns `true` if `t` lies within `tol` of the interval.
    pub fn contains_with(&self, t: f64, tol: f64) -> bool {
        t >= self.min - tol && t <= self.max + tol
    }

    /// The point of the interval closest to `t`.
    pub fn clamp(&self, t: f64) -> f64 {
        t.max(self.min).min(self.max)
    }

    /// Maps `s` in `[0, 1]` linearly onto the interval (`0` to `min`, `1` to `max`).
    pub fn lerp(&self, s: f64) -> f64 {
        self.min + s * (self.max - self.min)
    }

    /// Inverse of [`Interval::lerp`]: `(t - min) / length`.
    ///
    /// Returns `0.0` for an interval of zero length.
    pub fn normalize(&self, t: f64) -> f64 {
        let len = self.length();
        if len > 0.0 { (t - self.min) / len } else { 0.0 }
    }

    /// The overlap of the two intervals, or `None` if they are disjoint.
    /// Intervals that touch at one point intersect in a zero-length interval.
    pub fn intersect(&self, other: &Self) -> Option<Self> {
        let min = self.min.max(other.min);
        let max = self.max.min(other.max);
        (min <= max).then_some(Self { min, max })
    }

    /// The smallest interval containing both.
    #[must_use]
    pub fn hull(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Splits at `t` into `[min, t]` and `[t, max]`. `t` is clamped into the
    /// interval first, so one half may have zero length.
    pub fn split(&self, t: f64) -> (Self, Self) {
        let t = self.clamp(t);
        (
            Self {
                min: self.min,
                max: t,
            },
            Self {
                min: t,
                max: self.max,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_sorts_its_ends() {
        let i = Interval::new(3.0, -1.0);
        assert!((i.min + 1.0).abs() < 1e-15 && (i.max - 3.0).abs() < 1e-15);
        assert!((i.length() - 4.0).abs() < 1e-15);
        assert!((i.mid() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn contains_is_exactly_inclusive() {
        let i = Interval::new(0.0, 1.0);
        assert!(i.contains(0.0) && i.contains(1.0) && i.contains(0.5));
        assert!(!i.contains(1.0 + 1e-12));
        assert!(i.contains_with(1.0 + 1e-12, 1e-9));
        assert!(!i.contains_with(-1.0, 0.5));
    }

    #[test]
    fn clamp_lerp_normalize() {
        let i = Interval::new(2.0, 6.0);
        assert!((i.clamp(10.0) - 6.0).abs() < 1e-15);
        assert!((i.clamp(-10.0) - 2.0).abs() < 1e-15);
        assert!((i.lerp(0.25) - 3.0).abs() < 1e-15);
        assert!((i.normalize(5.0) - 0.75).abs() < 1e-15);
        assert!(Interval::new(1.0, 1.0).normalize(5.0).abs() < 1e-15);
    }

    #[test]
    fn intersect_and_hull() {
        let a = Interval::new(0.0, 2.0);
        let b = Interval::new(1.0, 3.0);
        assert_eq!(a.intersect(&b), Some(Interval::new(1.0, 2.0)));
        assert_eq!(
            a.intersect(&Interval::new(2.0, 5.0)),
            Some(Interval::new(2.0, 2.0))
        );
        assert_eq!(a.intersect(&Interval::new(2.5, 5.0)), None);
        assert_eq!(a.hull(&Interval::new(5.0, 6.0)), Interval::new(0.0, 6.0));
    }

    #[test]
    fn split_covers_the_interval() {
        let (l, r) = Interval::new(0.0, 4.0).split(1.0);
        assert_eq!(l, Interval::new(0.0, 1.0));
        assert_eq!(r, Interval::new(1.0, 4.0));
        let (l, r) = Interval::new(0.0, 4.0).split(9.0);
        assert_eq!(l, Interval::new(0.0, 4.0));
        assert!(r.length().abs() < 1e-15);
    }
}
