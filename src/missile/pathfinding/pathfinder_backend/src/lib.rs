mod astar;

use numpy::PyReadonlyArray2;
use pyo3::prelude::*;

#[pyfunction]
fn rs_pathfinder(dem: PyReadonlyArray2<f32>, start: (usize, usize)) -> PyResult<f32> {
    let view = dem.as_array();
    let (rows, cols) = view.dim();
    if start.0 >= rows || start.1 >= cols {
        return Err(pyo3::exceptions::PyValueError::new_err("start out of bounds"));
    }
    Ok(view[[start.0, start.1]])
}

#[pymodule]
fn pathfinder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(rs_pathfinder, m)?)?;
    Ok(())
}