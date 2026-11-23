use pyo3::prelude::*;
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::cmp::Ordering;
use std::io::Read;
use std::fs::File;
use std::collections::HashMap;
use memmap2::Mmap;


struct Node {
    pub f_score: f32,
    pub index: usize
}

struct NodeState {
    pub g_score: f32,
    pub dir: u8 // direction which the previous node are at
    // i (= dir):    0    1    2    3    4    5    6    7
    // direct_r[i]: -1    1    0    0   -1   -1    1    1
    // direct_c[i]:  0    0   -1    1   -1    1   -1    1
    // move:         ↑    ↓    ←    →    ↖    ↗    ↙    ↘
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.total_cmp(&self.f_score)
 }
}
impl PartialOrd for Node { fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) } }
impl PartialEq for Node { fn eq(&self, other: &Self) -> bool { self.cmp(other) == Ordering::Equal } }
impl Eq for Node {}

// Main algorithm function
pub fn search(
    dem_slice: &[f32],
    rows: usize, // num of rows
    cols: usize, // num of cols
    mx_slice: &[f64],
    my_slice: &[f64],
    start: (usize, usize),
    end: (usize, usize)
) -> f64 {

    // constructing data structure to store unexplored and explored nodes
    let mut open = BinaryHeap::<Node>::new();
    let mut close: HashMap<usize, NodeState> = HashMap::new();

    let start_idx = start.0 * cols + start.1;
    let end_idx = end.0 * cols + end.1;

    if (start_idx < 0 || start_idx >= dem_slice.len()
        || end_idx < 0 || end_idx >= dem_slice.len()) {
        return 0.0;
    }
    return get_movement_cost();
}

fn get_movement_cost() -> f64 {
    return 1.0;
}



