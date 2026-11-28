mod astar;

use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::prelude::*;

#[pyfunction]
#[pyo3(signature = (dem, meters_per_x, meters_per_y, start, end, heuristic_weight=1.1))]
fn rs_pathfinder(
    dem: PyReadonlyArray2<'_, f32>,
    meters_per_x: PyReadonlyArray1<'_, f64>,
    meters_per_y: PyReadonlyArray1<'_, f64>,
    start: (usize, usize),
    end: (usize, usize),
    heuristic_weight: f32,
) -> PyResult<Vec<(usize, usize)>> {

    let view = dem.as_array();
    let (rows, cols) = view.dim();
    if start.0 >= rows || start.1 >= cols {
        return Err(pyo3::exceptions::PyValueError::new_err("start out of bounds"));
    }

    // turn dem, meters_per_x and meters_per_y into slices
    let dem_slice: &[f32] = dem
        .as_slice()
        .map_err(|_| pyo3::exceptions::PyValueError::new_err("dem is not contiguous"))?;

    let mx_slice: &[f64] = meters_per_x
        .as_slice()
        .map_err(|_| pyo3::exceptions::PyValueError::new_err("meters_per_x is not contiguous"))?;

    let my_slice: &[f64] = meters_per_y
        .as_slice()
        .map_err(|_| pyo3::exceptions::PyValueError::new_err("meters_per_y is not contiguous"))?;

    Ok(astar::search(dem_slice, rows, cols, mx_slice, my_slice, start, end, heuristic_weight))
}

#[pymodule]
fn pathfinder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(rs_pathfinder, m)?)?;
    Ok(())
}