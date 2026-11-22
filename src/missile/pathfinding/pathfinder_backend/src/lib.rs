mod astar;

use pyo3::prelude::*;

#[pyfunction]
fn rs_pathfinder() -> PyResult<f64> {
    Ok(astar::search())
}

#[pymodule]
fn pathfinder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(rs_pathfinder, m)?)?;
    Ok(())
}