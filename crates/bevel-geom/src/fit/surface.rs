// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface fitting: interpolation and least-squares approximation of a
//! rectangular grid of points, and skinning of curves (The NURBS Book,
//! sections 9.2.5, 9.4.3 and 10.3).
//!
//! The data is a grid `Q_{k,l}` with `k = 0, ..., n` along `u` (the rows)
//! and `l = 0, ..., m` along `v` (the columns), stored row-major as a
//! `&[Point3]` with explicit row and column counts, the layout of the
//! control net of a [`NurbsSurface`]. The surface `S(u, v)` is non-rational
//! and defined on `[0, 1]^2`.
//!
//! # Parameters
//!
//! [`surface_parameters`] assigns `u_k` and `v_l` to the grid lines
//! (section 9.2.5). For every column `l` the curve fit's parametrization of
//! the points `Q_{0,l}, ..., Q_{n,l}` gives a parameter per row; the `u_k`
//! are the average of these over all columns, and the `v_l` are obtained in
//! the same way from the rows. A column (or row) whose points all coincide
//! carries no information on the spacing and is left out of the average, as
//! the book prescribes. Unlike [`parameters`](super::parameters) this does
//! not reject coincident neighbours inside a column: only the average has to
//! be strictly increasing.
//!
//! # Interpolation
//!
//! [`interpolate_surface`] is global surface interpolation (A9.4). The knot
//! vectors are the averaged knots (eq. 9.8, [`averaging_knots`]) of `u_k`
//! and `v_l`. The control points `P_{i,j}` solve
//! `sum_i sum_j N_{i,p}(u_k) N_{j,q}(v_l) P_{i,j} = Q_{k,l}`, which
//! separates into two families of curve interpolations
//! (section 9.2.5): first every column `l` is interpolated along `u`, giving
//! intermediate control points `R_{i,l}` with
//! `sum_i N_{i,p}(u_k) R_{i,l} = Q_{k,l}`; then every row `i` of the
//! `R_{i,l}` is interpolated along `v`, giving
//! `sum_j N_{j,q}(v_l) P_{i,j} = R_{i,l}`. All columns share one collocation
//! matrix, so it is factored once per direction (dense LU with partial
//! pivoting, as for curves) and solved for all right-hand sides at once. A
//! numerically singular matrix is reported as [`GeomError::NotConverged`].
//!
//! # Approximation
//!
//! [`approximate_surface`] applies the least-squares curve fit of
//! section 9.4.1 in two passes, the approach of section 9.4.3: every column
//! of the grid is fitted along `u` with `count_u` control points, and every
//! row of the resulting control points along `v` with `count_v` control
//! points. As for curves, the knots follow the data parameters
//! (section 9.4.1) and the first and last control point of every fit are the
//! first and last data point, so the four corners of the grid are
//! interpolated. The design matrix of each direction is factored once
//! (Householder QR) and shared by all lines.
//!
//! The two-pass fit is *not* the global least-squares optimum. The best
//! surface in the tensor-product space minimises the squared distance over
//! all `(k, l)` at once, which is one large system; the two passes minimise
//! one direction and then the other, and the result depends on which
//! direction goes first. The first pass is optimal for the columns, and
//! the second pass is optimal for the first pass's control points, not for
//! the original data. The deviation of the result can therefore exceed the
//! global optimum's, and, as for curves, is not guaranteed to decrease when
//! control points are added (the knot sets of different counts are not
//! nested). With `count_u == rows` and `count_v == cols` the fit is the
//! interpolation of A9.4 and the deviation is zero up to rounding.
//!
//! # Lofting
//!
//! [`loft`] skins a list of curves into a surface (section 10.3). The curves
//! are first made compatible ([`make_compatible`]): same degree, domain and
//! knot vector. A control point `P_{i}^k` of curve `k` then pairs with the
//! control point `i` of every other curve, and each such family
//! `P_0^i, ..., P_K^i` is interpolated along `v` as a curve. The `v`
//! parameters are the averages, over the control point families, of the
//! parametrization of the control polygons (section 10.3). Rational curves
//! are interpolated in homogeneous coordinates `(w P, w)`, which is what
//! makes the surface contain each curve exactly; a positive weight is not
//! guaranteed in the interpolated net, and a surface with a weight that is
//! not positive is rejected.
//!
//! All results are deterministic.

use bevel_math::{DVec3, DVec4, Point3};
use nalgebra::DMatrix;

use super::solve::{set, solve_least_squares, solve_square};
use super::{COINCIDENT_TOLERANCE, FitPoint, Parametrization, averaging_knots};
use crate::basis::basis_funs_raw;
use crate::knots::check_degree;
use crate::{GeomError, KnotVector, NurbsCurve3, NurbsSurface, make_compatible};

/// The two directions of a grid: along the rows (`u`) or the columns (`v`).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Direction {
    /// Parameters of the rows, `u_k`: lines run down a column.
    U,
    /// Parameters of the columns, `v_l`: lines run along a row.
    V,
}

/// Rejects a grid whose size does not match `rows x cols`, non-finite
/// coordinates and fewer than two rows or columns.
fn check_grid(grid: &[Point3], rows: usize, cols: usize) -> Result<(), GeomError> {
    if rows.checked_mul(cols) != Some(grid.len()) {
        return Err(GeomError::InvalidArgument(
            "the grid size does not match rows x cols",
        ));
    }
    if grid.iter().any(|&p| !p.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    if rows < 2 || cols < 2 {
        return Err(GeomError::InvalidArgument(
            "a grid needs at least two rows and two columns",
        ));
    }
    Ok(())
}

/// The lines of the grid that carry the parameters of `direction`: the
/// columns for [`Direction::U`] and the rows for [`Direction::V`]. The grid
/// has been validated by [`check_grid`].
fn grid_lines(grid: &[Point3], cols: usize, direction: Direction) -> Vec<Vec<Point3>> {
    match direction {
        Direction::U => (0..cols)
            .map(|l| grid.iter().skip(l).step_by(cols).copied().collect())
            .collect(),
        Direction::V => grid.chunks(cols).map(<[Point3]>::to_vec).collect(),
    }
}

/// The parameters of a grid direction, averaged over the non-degenerate
/// lines (section 9.2.5); `grid` has been validated by [`check_grid`].
fn direction_parameters(
    grid: &[Point3],
    rows: usize,
    cols: usize,
    direction: Direction,
    method: Parametrization,
) -> Result<Vec<f64>, GeomError> {
    let Some(&origin) = grid.first() else {
        return Err(GeomError::InvalidArgument("the grid is empty"));
    };
    let extent = grid
        .iter()
        .map(|&p| (p - origin).length())
        .fold(0.0, f64::max);
    let weight = |d: f64| match method {
        Parametrization::Uniform => 1.0,
        Parametrization::ChordLength => d,
        Parametrization::Centripetal => d.sqrt(),
    };
    let count = if direction == Direction::U {
        rows
    } else {
        cols
    };
    let mut sum = vec![0.0; count];
    let mut used = 0_usize;
    for line in grid_lines(grid, cols, direction) {
        let Some(&first) = line.first() else {
            continue;
        };
        let spread = line
            .iter()
            .map(|&p| (p - first).length())
            .fold(0.0, f64::max);
        if spread <= COINCIDENT_TOLERANCE * extent {
            continue;
        }
        let chords: Vec<f64> = line.windows(2).map(|w| (w[1] - w[0]).length()).collect();
        let total: f64 = chords.iter().map(|&d| weight(d)).sum();
        let mut acc = 0.0;
        for (s, &d) in sum.iter_mut().skip(1).zip(&chords) {
            acc += weight(d);
            *s += acc / total;
        }
        used += 1;
    }
    if used == 0 {
        return Err(GeomError::InvalidArgument(
            "every row or column of the grid is a single point",
        ));
    }
    let mut params: Vec<f64> = sum.iter().map(|&s| s / used as f64).collect();
    if let Some(first) = params.first_mut() {
        *first = 0.0;
    }
    // The last parameter is exactly 1 whatever the rounding.
    if let Some(last) = params.last_mut() {
        *last = 1.0;
    }
    if params.windows(2).any(|w| w[1] <= w[0]) {
        return Err(GeomError::InvalidArgument(
            "the grid points are too close together to give distinct parameters",
        ));
    }
    Ok(params)
}

/// The parameters `(u_k, v_l)` of the rows and columns of a grid of points
/// (The NURBS Book, section 9.2.5).
///
/// `grid` holds `rows x cols` points in row-major order (`grid[k * cols + l]`
/// is `Q_{k,l}`, `k` along `u`). The `u` parameters are the average, over
/// all columns, of the curve [`parameters`](super::parameters) of that
/// column, and the `v` parameters the same for the rows; both run from 0 to
/// 1 and are strictly increasing. Columns (rows) whose points all coincide,
/// to within 1e-12 of the extent of the grid, are left out of the average.
/// Coincident neighbours inside an otherwise non-degenerate line are
/// allowed, since only the average has to be strictly increasing.
///
/// # Errors
///
/// [`GeomError::InvalidArgument`] if `grid.len() != rows * cols`, there are
/// fewer than two rows or columns, every column or every row degenerates to
/// a single point, or the averaged parameters are not strictly increasing;
/// [`GeomError::NonFinite`] for NaN or infinite coordinates.
pub fn surface_parameters(
    grid: &[Point3],
    rows: usize,
    cols: usize,
    method: Parametrization,
) -> Result<(Vec<f64>, Vec<f64>), GeomError> {
    check_grid(grid, rows, cols)?;
    let u = direction_parameters(grid, rows, cols, Direction::U, method)?;
    let v = direction_parameters(grid, rows, cols, Direction::V, method)?;
    Ok((u, v))
}

/// The entry `(row, col)` of `m`, or 0 outside the matrix.
fn at(m: &DMatrix<f64>, row: usize, col: usize) -> f64 {
    m.get((row, col)).copied().unwrap_or(0.0)
}

/// The collocation matrix `[N_{j,p}(u_k)]` of `params` for `knots`.
fn collocation(params: &[f64], knots: &KnotVector, degree: usize) -> DMatrix<f64> {
    let count = params.len();
    let mut a = DMatrix::zeros(count, count);
    for (row, &u) in params.iter().enumerate() {
        let span = knots.span(degree, u);
        let basis = basis_funs_raw(knots.as_slice(), span, u, degree);
        for (j, &value) in basis.iter().take(degree + 1).enumerate() {
            set(&mut a, row, span - degree + j, value);
        }
    }
    a
}

/// The knot vector of section 9.4.1 for `count` control points, which must
/// be fewer than `params.len()`.
///
/// The same rule as the curve fit's, which keeps it private: the interior
/// knots follow the density of the data parameters, computed in integer
/// arithmetic.
fn approximation_knots(
    params: &[f64],
    degree: usize,
    count: usize,
) -> Result<KnotVector, GeomError> {
    let spans = count - degree;
    let points = params.len();
    let interior = (1..spans)
        .map(|j| {
            // jd = j (m + 1) / (n - p + 1), as quotient and remainder.
            let scaled = j * points;
            let (i, remainder) = (scaled / spans, scaled % spans);
            let alpha = remainder as f64 / spans as f64;
            match (i.checked_sub(1).and_then(|i| params.get(i)), params.get(i)) {
                (Some(&a), Some(&b)) => Ok((1.0 - alpha) * a + alpha * b),
                _ => Err(GeomError::InvalidArgument(
                    "too few data points for the control point count",
                )),
            }
        })
        .collect::<Result<Vec<f64>, GeomError>>()?;
    KnotVector::clamped_from_interior(degree, 0.0, 1.0, &interior)
}

/// Fits all columns of `data` (one data point per row, one independent
/// curve per column) with `count` control points of degree `degree` at the
/// parameters `params`: the interpolation of A9.1 for `count == params.len()`
/// and the least-squares fit of section 9.4.1 with fixed end points
/// otherwise. Returns the knots and the control points, one row each, with
/// the same columns as `data`.
///
/// `data` has `params.len()` rows, `params` is valid (strictly increasing
/// from 0 to 1) and `degree < count <= params.len()`.
fn fit_lines(
    data: &DMatrix<f64>,
    params: &[f64],
    degree: usize,
    count: usize,
) -> Result<(KnotVector, DMatrix<f64>), GeomError> {
    let total = params.len();
    let width = data.ncols();
    if count == total {
        let knots = averaging_knots(params, degree)?;
        let a = collocation(params, &knots, degree);
        let control = solve_square(a, data)?;
        return Ok((knots, control));
    }
    let knots = approximation_knots(params, degree, count)?;
    let end = count - 1;
    let free = count - 2;
    let solution = if free > 0 {
        let rows = total - 2;
        let mut a = DMatrix::zeros(rows, free);
        let mut rhs = DMatrix::from_fn(rows, width, |r, c| at(data, r + 1, c));
        for (row, &u) in params.iter().enumerate().skip(1).take(rows) {
            let span = knots.span(degree, u);
            let basis = basis_funs_raw(knots.as_slice(), span, u, degree);
            for (j, &value) in basis.iter().take(degree + 1).enumerate() {
                let index = span - degree + j;
                if index == 0 || index == end {
                    // A fixed end control point moves to the right side.
                    let fixed = if index == 0 { 0 } else { total - 1 };
                    for c in 0..width {
                        let r = at(&rhs, row - 1, c) - value * at(data, fixed, c);
                        set(&mut rhs, row - 1, c, r);
                    }
                } else {
                    set(&mut a, row - 1, index - 1, value);
                }
            }
        }
        solve_least_squares(a, &rhs)?
    } else {
        DMatrix::zeros(0, width)
    };
    let control = DMatrix::from_fn(count, width, |r, c| {
        if r == 0 {
            at(data, 0, c)
        } else if r == end {
            at(data, total - 1, c)
        } else {
            at(&solution, r - 1, c)
        }
    });
    Ok((knots, control))
}

/// The grid as a matrix with a row per grid row and the three coordinates
/// of the points of column `l` in matrix columns `3 l ..= 3 l + 2`.
fn grid_matrix(grid: &[Point3], rows: usize, cols: usize) -> DMatrix<f64> {
    DMatrix::from_fn(rows, 3 * cols, |k, c| {
        grid.get(k * cols + c / 3).map_or(0.0, |p| p.coord(c % 3))
    })
}

/// The second pass's data: the transpose of the first pass's control points
/// regrouped so that each of `rows` lines (a row of the control net) is one
/// matrix column triple and each matrix row is a position along `v`.
fn transpose_lines(m: &DMatrix<f64>, rows: usize, cols: usize) -> DMatrix<f64> {
    DMatrix::from_fn(cols, 3 * rows, |l, c| at(m, c / 3, 3 * l + c % 3))
}

/// The control net of two passes as points in row-major order (`i * count_v + j`).
fn control_net(m: &DMatrix<f64>, count_u: usize, count_v: usize) -> Vec<Point3> {
    (0..count_u)
        .flat_map(|i| {
            (0..count_v)
                .map(move |j| DVec3::new(at(m, j, 3 * i), at(m, j, 3 * i + 1), at(m, j, 3 * i + 2)))
        })
        .collect()
}

/// Validates a surface fit request up to the parameters: degrees, grid and
/// the number of rows and columns for the degrees.
fn check_surface_request(
    grid: &[Point3],
    rows: usize,
    cols: usize,
    degree_u: usize,
    degree_v: usize,
) -> Result<(), GeomError> {
    check_degree(degree_u)?;
    check_degree(degree_v)?;
    check_grid(grid, rows, cols)?;
    if rows < degree_u + 1 || cols < degree_v + 1 {
        return Err(GeomError::InvalidArgument(
            "too few rows or columns for the requested degree",
        ));
    }
    Ok(())
}

/// The shared two-pass fit of [`interpolate_surface`] and
/// [`approximate_surface`]; the arguments have been validated.
fn fit_surface(
    grid: &[Point3],
    (rows, cols): (usize, usize),
    (degree_u, degree_v): (usize, usize),
    (count_u, count_v): (usize, usize),
    method: Parametrization,
) -> Result<NurbsSurface, GeomError> {
    let (params_u, params_v) = surface_parameters(grid, rows, cols, method)?;
    // First pass: every column of the grid along u.
    let data = grid_matrix(grid, rows, cols);
    let (knots_u, along_u) = fit_lines(&data, &params_u, degree_u, count_u)?;
    // Second pass: every row of the intermediate net along v.
    let data = transpose_lines(&along_u, count_u, cols);
    let (knots_v, along_v) = fit_lines(&data, &params_v, degree_v, count_v)?;
    let net = control_net(&along_v, count_u, count_v);
    NurbsSurface::from_points(degree_u, degree_v, knots_u, knots_v, &net, None)
}

/// A non-rational surface of degrees `degree_u` and `degree_v` through the
/// grid `Q_{k,l}` (The NURBS Book, A9.4).
///
/// `grid` holds `rows x cols` points in row-major order (`grid[k * cols + l]`
/// with `k` along `u`). The surface is defined on `[0, 1]^2`, has one control
/// point per grid point and passes through `Q_{k,l}` at the parameters
/// `(u_k, v_l)` that [`surface_parameters`] gives for `method`. The knot
/// vectors are the averaged knots of those parameters
/// ([`averaging_knots`]). See the module documentation for the method.
///
/// Needs at least `degree_u + 1` rows and `degree_v + 1` columns. With
/// exactly that many in a direction the surface is a single Bezier segment
/// in it, and with degrees (1, 1) it is the piecewise bilinear surface
/// through the grid.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`;
/// [`GeomError::InvalidArgument`] if `grid.len() != rows * cols`, there are
/// fewer rows than `degree_u + 1` or fewer columns than `degree_v + 1`, or
/// the parameters cannot be assigned (see [`surface_parameters`]);
/// [`GeomError::NonFinite`] for NaN or infinite coordinates;
/// [`GeomError::NotConverged`] if a linear system is singular to working
/// precision (not expected for valid data).
pub fn interpolate_surface(
    grid: &[Point3],
    rows: usize,
    cols: usize,
    degree_u: usize,
    degree_v: usize,
    method: Parametrization,
) -> Result<NurbsSurface, GeomError> {
    check_surface_request(grid, rows, cols, degree_u, degree_v)?;
    fit_surface(
        grid,
        (rows, cols),
        (degree_u, degree_v),
        (rows, cols),
        method,
    )
}

/// A non-rational surface with `count_u x count_v` control points that
/// approximates the grid `Q_{k,l}` in the least-squares sense (The NURBS
/// Book, section 9.4.3), interpolating its four corners.
///
/// `grid` is as for [`interpolate_surface`]. The surface is defined on
/// `[0, 1]^2`, with the parameters of [`surface_parameters`]. The curve
/// least-squares fit (section 9.4.1) is applied to the columns along `u` and
/// then to the rows of the resulting control points along `v`; in each pass
/// the end points are fixed, so `Q_{0,0}`, `Q_{0,m}`, `Q_{n,0}` and
/// `Q_{n,m}` are exact. With `count_u == rows` and `count_v == cols` the
/// result is the interpolating surface of [`interpolate_surface`].
///
/// **The two-pass method is not the global least-squares optimum**: it
/// minimises one direction and then the other, so the sum of squared
/// deviations can exceed that of the best surface with the same knots, and
/// it is not guaranteed to decrease with the control point counts (the
/// knot sets of different counts are not nested). The module documentation
/// gives the details.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`;
/// [`GeomError::ControlPointCount`] if `count_u <= degree_u` or
/// `count_v <= degree_v`; [`GeomError::InvalidArgument`] if a count exceeds
/// the number of rows or columns, `grid.len() != rows * cols`, there are
/// fewer rows than `degree_u + 1` or fewer columns than `degree_v + 1`, or
/// the parameters cannot be assigned (see [`surface_parameters`]);
/// [`GeomError::NonFinite`] for NaN or infinite coordinates;
/// [`GeomError::NotConverged`] if a least-squares system is rank deficient
/// to working precision.
// The signature is the API's: one count and degree per direction.
#[allow(clippy::too_many_arguments)]
pub fn approximate_surface(
    grid: &[Point3],
    rows: usize,
    cols: usize,
    degree_u: usize,
    degree_v: usize,
    count_u: usize,
    count_v: usize,
    method: Parametrization,
) -> Result<NurbsSurface, GeomError> {
    check_surface_request(grid, rows, cols, degree_u, degree_v)?;
    for (count, degree) in [(count_u, degree_u), (count_v, degree_v)] {
        if count <= degree {
            return Err(GeomError::ControlPointCount {
                expected: degree + 1,
                found: count,
            });
        }
    }
    if count_u > rows || count_v > cols {
        return Err(GeomError::InvalidArgument(
            "more control points than grid rows or columns",
        ));
    }
    fit_surface(
        grid,
        (rows, cols),
        (degree_u, degree_v),
        (count_u, count_v),
        method,
    )
}

/// The control points of curves made compatible, as a grid with one row per
/// control point index and one column per curve (`grid[i * K + k]`), and the
/// compatible curves.
fn loft_grid(curves: &[NurbsCurve3]) -> Result<(Vec<NurbsCurve3>, Vec<Point3>), GeomError> {
    if curves.len() < 2 {
        return Err(GeomError::InvalidArgument(
            "a loft needs at least two curves",
        ));
    }
    let compatible = make_compatible(curves)?;
    let points: Vec<Vec<Point3>> = compatible.iter().map(NurbsCurve3::control_points).collect();
    let count = points.first().map_or(0, Vec::len);
    let grid = (0..count)
        .flat_map(|i| points.iter().map(move |p| p.get(i).copied()))
        .collect::<Option<Vec<Point3>>>()
        .ok_or(GeomError::InvalidArgument(
            "the compatible curves differ in size",
        ))?;
    Ok((compatible, grid))
}

/// The `v` parameters of a loft: one per curve, from 0 to 1 and strictly
/// increasing, averaged over the control point families of the compatible
/// curves (The NURBS Book, section 10.3).
///
/// These are the values at which [`loft`] places the curves: curve `k` of
/// `curves` is the isocurve of the surface at `v = parameters[k]`. The
/// parametrization of each family of corresponding control points follows
/// `method`; families whose points all coincide are left out of the average.
///
/// # Errors
///
/// [`GeomError::InvalidArgument`] for fewer than two curves, or if every
/// family of control points is a single point or the averaged parameters
/// are not strictly increasing (for example two identical curves in a
/// row); the errors of [`make_compatible`].
pub fn loft_parameters(
    curves: &[NurbsCurve3],
    method: Parametrization,
) -> Result<Vec<f64>, GeomError> {
    let (compatible, grid) = loft_grid(curves)?;
    direction_parameters(
        &grid,
        grid.len() / compatible.len().max(1),
        compatible.len(),
        Direction::V,
        method,
    )
}

/// A surface that contains the curves as isocurves in `v`, skinned through
/// them with degree `degree_v` across (The NURBS Book, section 10.3).
///
/// The curves are made compatible with [`make_compatible`] (the common
/// degree, domain of the first curve and knot vector are those of the
/// result in `u`), and the corresponding control points of all curves are
/// interpolated along `v` at the parameters [`loft_parameters`] gives, with
/// the averaged knots of those parameters. The surface equals curve `k` at
/// `v = loft_parameters(curves, method)[k]`, and is defined on the domain
/// of the first curve in `u` and `[0, 1]` in `v`. If any curve is rational,
/// the control points are interpolated in homogeneous coordinates and the
/// surface is rational; otherwise it is not.
///
/// Needs at least `degree_v + 1` curves.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`;
/// [`GeomError::InvalidArgument`] for too few curves or a `v` parametrization
/// that cannot be assigned (see [`loft_parameters`]);
/// [`GeomError::InvalidWeight`] if the interpolated net of a rational loft
/// has a weight that is not positive; [`GeomError::NotConverged`] if the
/// linear system is singular to working precision; the errors of
/// [`make_compatible`].
pub fn loft(
    curves: &[NurbsCurve3],
    degree_v: usize,
    method: Parametrization,
) -> Result<NurbsSurface, GeomError> {
    check_degree(degree_v)?;
    let (compatible, grid) = loft_grid(curves)?;
    let count_v = compatible.len();
    if count_v < degree_v + 1 {
        return Err(GeomError::InvalidArgument(
            "too few curves for the requested degree",
        ));
    }
    let Some(first) = compatible.first() else {
        return Err(GeomError::InvalidArgument(
            "a loft needs at least two curves",
        ));
    };
    let count_u = first.control_points().len();
    let params_v = direction_parameters(&grid, count_u, count_v, Direction::V, method)?;
    let rational = compatible.iter().any(NurbsCurve3::is_rational);
    let width = if rational { 4 } else { 3 };
    // One row per curve, `width` columns per control point index.
    let flat: Vec<Vec<f64>> = compatible
        .iter()
        .map(|curve| {
            if rational {
                curve
                    .control_weighted()
                    .iter()
                    .flat_map(|h| [h.x, h.y, h.z, h.w])
                    .collect()
            } else {
                curve
                    .control_points()
                    .iter()
                    .flat_map(|p| [p.x, p.y, p.z])
                    .collect()
            }
        })
        .collect();
    let data = DMatrix::from_fn(count_v, width * count_u, |k, c| {
        flat.get(k)
            .and_then(|row| row.get(c))
            .copied()
            .unwrap_or(0.0)
    });
    let (knots_v, along_v) = fit_lines(&data, &params_v, degree_v, count_v)?;
    let net = |i: usize, j: usize, a: usize| at(&along_v, j, width * i + a);
    if rational {
        let control: Vec<DVec4> = (0..count_u)
            .flat_map(|i| {
                (0..count_v).map(move |j| {
                    DVec4::new(net(i, j, 0), net(i, j, 1), net(i, j, 2), net(i, j, 3))
                })
            })
            .collect();
        NurbsSurface::new(
            first.degree(),
            degree_v,
            first.knots().clone(),
            knots_v,
            control,
        )
    } else {
        let control: Vec<Point3> = (0..count_u)
            .flat_map(|i| {
                (0..count_v).map(move |j| DVec3::new(net(i, j, 0), net(i, j, 1), net(i, j, 2)))
            })
            .collect();
        NurbsSurface::from_points(
            first.degree(),
            degree_v,
            first.knots().clone(),
            knots_v,
            &control,
            None,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::parameters;

    const METHODS: [Parametrization; 3] = [
        Parametrization::Uniform,
        Parametrization::ChordLength,
        Parametrization::Centripetal,
    ];

    /// A wavy height field over an irregular grid.
    fn bumps(rows: usize, cols: usize) -> Vec<Point3> {
        let mut grid = Vec::new();
        for k in 0..rows {
            for l in 0..cols {
                let (s, t) = (k as f64 / (rows - 1) as f64, l as f64 / (cols - 1) as f64);
                let x = 10.0 * s + 0.4 * (3.0 * t).sin();
                let y = 8.0 * t + 0.3 * (2.0 * s).cos();
                grid.push(DVec3::new(
                    x,
                    y,
                    (2.0 * x).sin() * (0.7 * y).cos() + 0.1 * x * y,
                ));
            }
        }
        grid
    }

    #[test]
    fn a_planar_grid_gives_a_planar_surface() {
        // z = 0.5 x - 0.25 y + 3 over an irregular grid.
        let plane = |x: f64, y: f64| DVec3::new(x, y, 0.5 * x - 0.25 * y + 3.0);
        let xs = [0.0, 1.0, 2.5, 3.0, 5.0, 8.0];
        let ys = [-1.0, 0.0, 0.5, 2.0, 4.0];
        let grid: Vec<Point3> = xs
            .iter()
            .flat_map(|&x| ys.iter().map(move |&y| plane(x, y)))
            .collect();
        for method in METHODS {
            let s = interpolate_surface(&grid, xs.len(), ys.len(), 3, 2, method).unwrap();
            for p in s.control_points() {
                assert!((p.z - (0.5 * p.x - 0.25 * p.y + 3.0)).abs() < 1e-11);
            }
            for i in 0..=10 {
                for j in 0..=10 {
                    let p = s.point(f64::from(i) / 10.0, f64::from(j) / 10.0);
                    assert!((p.z - (0.5 * p.x - 0.25 * p.y + 3.0)).abs() < 1e-11);
                }
            }
        }
    }

    #[test]
    fn bilinear_data_is_reproduced_by_degree_one() {
        let patch = NurbsSurface::bilinear(
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(4.0, 1.0, 1.0),
            DVec3::new(-1.0, 3.0, 2.0),
            DVec3::new(5.0, 5.0, -2.0),
        )
        .unwrap();
        let (rows, cols) = (4, 6);
        let grid: Vec<Point3> = (0..rows)
            .flat_map(|k| {
                let patch = &patch;
                (0..cols).map(move |l| {
                    patch.point(k as f64 / (rows - 1) as f64, l as f64 / (cols - 1) as f64)
                })
            })
            .collect();
        let s = interpolate_surface(&grid, rows, cols, 1, 1, Parametrization::Uniform).unwrap();
        assert_eq!(s.control_points().len(), rows * cols);
        for i in 0..=17 {
            for j in 0..=17 {
                let (u, v) = (f64::from(i) / 17.0, f64::from(j) / 17.0);
                assert!((s.point(u, v) - patch.point(u, v)).length() < 1e-12);
            }
        }
        // Other parametrizations still pass through the grid points.
        for method in METHODS {
            let s = interpolate_surface(&grid, rows, cols, 1, 1, method).unwrap();
            let (us, vs) = surface_parameters(&grid, rows, cols, method).unwrap();
            for (k, &u) in us.iter().enumerate() {
                for (l, &v) in vs.iter().enumerate() {
                    assert!((s.point(u, v) - grid[k * cols + l]).length() < 1e-12);
                }
            }
        }
    }

    #[test]
    fn parameters_average_the_lines() {
        let grid = bumps(5, 7);
        let (rows, cols) = (5, 7);
        for method in METHODS {
            let (us, vs) = surface_parameters(&grid, rows, cols, method).unwrap();
            assert_eq!((us.len(), vs.len()), (rows, cols));
            assert!(us[0].abs() < 1e-15 && (us[rows - 1] - 1.0).abs() < 1e-15);
            assert!(vs[0].abs() < 1e-15 && (vs[cols - 1] - 1.0).abs() < 1e-15);
            // Average of the per-column curve parameters.
            let mut expected = vec![0.0; rows];
            for l in 0..cols {
                let column: Vec<Point3> = (0..rows).map(|k| grid[k * cols + l]).collect();
                for (e, u) in expected
                    .iter_mut()
                    .zip(parameters(&column, method).unwrap())
                {
                    *e += u / cols as f64;
                }
            }
            for (a, b) in us.iter().zip(&expected) {
                assert!((a - b).abs() < 1e-14, "{method:?}");
            }
        }
    }

    #[test]
    fn degenerate_lines_are_skipped() {
        // Column 2 collapses to a point; rows still differ.
        let (rows, cols) = (5, 6);
        let mut grid = bumps(rows, cols);
        let pole = grid[2];
        for k in 0..rows {
            grid[k * cols + 2] = pole;
        }
        let method = Parametrization::ChordLength;
        let (us, _) = surface_parameters(&grid, rows, cols, method).unwrap();
        let mut expected = vec![0.0; rows];
        let live: Vec<usize> = (0..cols).filter(|&l| l != 2).collect();
        for &l in &live {
            let column: Vec<Point3> = (0..rows).map(|k| grid[k * cols + l]).collect();
            for (e, u) in expected
                .iter_mut()
                .zip(parameters(&column, method).unwrap())
            {
                *e += u / live.len() as f64;
            }
        }
        for (a, b) in us.iter().zip(&expected) {
            assert!((a - b).abs() < 1e-14);
        }
        // The surface still interpolates the grid.
        let s = interpolate_surface(&grid, rows, cols, 3, 3, method).unwrap();
        let (us, vs) = surface_parameters(&grid, rows, cols, method).unwrap();
        for (k, &u) in us.iter().enumerate() {
            for (l, &v) in vs.iter().enumerate() {
                assert!((s.point(u, v) - grid[k * cols + l]).length() < 1e-9);
            }
        }
    }

    #[test]
    fn a_collapsed_edge_row_is_allowed() {
        // The first row is a single point, like the apex of a cone.
        let (rows, cols) = (4, 8);
        let mut grid = bumps(rows, cols);
        let apex = DVec3::new(0.0, 0.0, 5.0);
        grid.iter_mut().take(cols).for_each(|p| *p = apex);
        for method in METHODS {
            let (us, vs) = surface_parameters(&grid, rows, cols, method).unwrap();
            let s = interpolate_surface(&grid, rows, cols, 2, 3, method).unwrap();
            for (k, &u) in us.iter().enumerate() {
                for (l, &v) in vs.iter().enumerate() {
                    assert!((s.point(u, v) - grid[k * cols + l]).length() < 1e-9);
                }
            }
        }
    }

    #[test]
    fn a_grid_of_degenerate_lines_is_rejected() {
        // Every column is a single point (the grid is constant along u).
        let (rows, cols) = (4, 5);
        let grid: Vec<Point3> = (0..rows)
            .flat_map(|_| (0..cols).map(|l| DVec3::new(l as f64, 1.0, 2.0)))
            .collect();
        assert!(matches!(
            surface_parameters(&grid, rows, cols, Parametrization::ChordLength),
            Err(GeomError::InvalidArgument(_))
        ));
        let same = vec![DVec3::splat(1.5); 12];
        assert!(matches!(
            surface_parameters(&same, 3, 4, Parametrization::Uniform),
            Err(GeomError::InvalidArgument(_))
        ));
    }

    #[test]
    fn approximation_keeps_the_corners_and_interpolates_at_full_count() {
        let (rows, cols) = (9, 11);
        let grid = bumps(rows, cols);
        for method in METHODS {
            for (count_u, count_v) in [(4, 4), (5, 8), (8, 6), (9, 3), (4, 11)] {
                let s =
                    approximate_surface(&grid, rows, cols, 3, 2, count_u, count_v, method).unwrap();
                assert_eq!((s.count_u(), s.count_v()), (count_u, count_v));
                assert_eq!((s.degree_u(), s.degree_v()), (3, 2));
                let corners = [
                    (0.0, 0.0, grid[0]),
                    (0.0, 1.0, grid[cols - 1]),
                    (1.0, 0.0, grid[(rows - 1) * cols]),
                    (1.0, 1.0, grid[rows * cols - 1]),
                ];
                for (u, v, q) in corners {
                    assert!((s.point(u, v) - q).length() < 1e-12, "{count_u} {count_v}");
                }
            }
            let a = approximate_surface(&grid, rows, cols, 3, 2, rows, cols, method).unwrap();
            let i = interpolate_surface(&grid, rows, cols, 3, 2, method).unwrap();
            assert_eq!(a, i);
        }
    }

    #[test]
    fn a_bicubic_patch_is_approximated_exactly_by_a_bezier_patch() {
        let f = |s: f64, t: f64| {
            DVec3::new(
                s + 0.5 * t,
                t - 0.25 * s * s,
                s * s * s - 2.0 * s * t * t + t * t * t,
            )
        };
        let (rows, cols) = (12, 9);
        let grid: Vec<Point3> = (0..rows)
            .flat_map(|k| {
                (0..cols)
                    .map(move |l| f(k as f64 / (rows - 1) as f64, l as f64 / (cols - 1) as f64))
            })
            .collect();
        let s =
            approximate_surface(&grid, rows, cols, 3, 3, 4, 4, Parametrization::Uniform).unwrap();
        for i in 0..=10 {
            for j in 0..=10 {
                let (u, v) = (f64::from(i) / 10.0, f64::from(j) / 10.0);
                assert!((s.point(u, v) - f(u, v)).length() < 1e-9);
            }
        }
    }

    #[test]
    fn invalid_inputs_are_reported() {
        let grid = bumps(5, 6);
        let m = Parametrization::ChordLength;
        // Size mismatch.
        assert!(matches!(
            interpolate_surface(&grid, 5, 5, 3, 3, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            interpolate_surface(&grid, usize::MAX, 2, 3, 3, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            surface_parameters(&grid[..1], 1, 1, m),
            Err(GeomError::InvalidArgument(_))
        ));
        // Degrees.
        assert!(matches!(
            interpolate_surface(&grid, 5, 6, 0, 3, m),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        assert!(matches!(
            interpolate_surface(&grid, 5, 6, 3, 26, m),
            Err(GeomError::InvalidDegree { degree: 26 })
        ));
        // Too few rows or columns.
        assert!(matches!(
            interpolate_surface(&grid, 5, 6, 5, 3, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            interpolate_surface(&grid, 5, 6, 3, 6, m),
            Err(GeomError::InvalidArgument(_))
        ));
        // Non-finite.
        let mut nan = grid.clone();
        nan[7].y = f64::NAN;
        assert_eq!(
            interpolate_surface(&nan, 5, 6, 3, 3, m),
            Err(GeomError::NonFinite)
        );
        assert_eq!(surface_parameters(&nan, 5, 6, m), Err(GeomError::NonFinite));
        assert_eq!(
            approximate_surface(&nan, 5, 6, 3, 3, 4, 4, m),
            Err(GeomError::NonFinite)
        );
        // Control point counts.
        assert!(matches!(
            approximate_surface(&grid, 5, 6, 3, 3, 3, 4, m),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 3
            })
        ));
        assert!(matches!(
            approximate_surface(&grid, 5, 6, 3, 3, 4, 3, m),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 3
            })
        ));
        assert!(matches!(
            approximate_surface(&grid, 5, 6, 3, 3, 6, 4, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            approximate_surface(&grid, 5, 6, 3, 3, 4, 7, m),
            Err(GeomError::InvalidArgument(_))
        ));
    }

    #[test]
    fn loft_contains_its_curves() {
        let curve = |z: f64, bend: f64| {
            let kv = KnotVector::clamped_uniform(2, 4).unwrap();
            let pts = [
                DVec3::new(0.0, 0.0, z),
                DVec3::new(1.0, bend, z + 0.2),
                DVec3::new(2.0, -bend, z),
                DVec3::new(3.0, 0.5, z - 0.1),
            ];
            NurbsCurve3::from_points(2, kv, &pts, None).unwrap()
        };
        let curves = [
            curve(0.0, 1.0),
            curve(1.0, 0.5),
            curve(2.5, -1.0),
            curve(3.0, 2.0),
        ];
        for method in METHODS {
            let vs = loft_parameters(&curves, method).unwrap();
            assert_eq!(vs.len(), curves.len());
            for degree_v in 1..=3 {
                let s = loft(&curves, degree_v, method).unwrap();
                assert_eq!(s.degree_v(), degree_v);
                assert!(!s.is_rational());
                for (c, &v) in curves.iter().zip(&vs) {
                    for i in 0..=20 {
                        let (a, b) = c.domain();
                        let t = a + (b - a) * f64::from(i) / 20.0;
                        assert!((s.point(t, v) - c.point(t)).length() < 1e-10);
                    }
                }
            }
        }
    }

    #[test]
    fn loft_rejects_bad_input() {
        let kv = KnotVector::clamped_uniform(1, 2).unwrap();
        let c = NurbsCurve3::from_points(1, kv, &[DVec3::ZERO, DVec3::X], None).unwrap();
        let m = Parametrization::ChordLength;
        assert!(matches!(
            loft(&[], 1, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            loft(std::slice::from_ref(&c), 1, m),
            Err(GeomError::InvalidArgument(_))
        ));
        let d = c.clone();
        assert!(matches!(
            loft(&[c.clone(), d.clone()], 2, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            loft(&[c.clone(), d.clone()], 0, m),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        // Two identical curves cannot be told apart.
        assert!(matches!(
            loft(&[c, d], 1, m),
            Err(GeomError::InvalidArgument(_))
        ));
    }
}
