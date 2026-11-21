mod astar;

use pyo3::prelude::*;

#[pyfunction]
fn rs_pathfinder() -> PyResult<f64> {
    Ok(astar::search())
}


/// A Python module implemented in Rust.
#[pymodule]
mod pathfinder {
    use pyo3::prelude::*;

    /// Formats the sum of two numbers as string.
    #[pyfunction]
    fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
        Ok((a + b).to_string())
    }
}
