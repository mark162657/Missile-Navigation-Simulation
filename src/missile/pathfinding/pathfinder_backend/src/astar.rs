use pyo3::prelude::*;
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::cmp::Ordering;


use std::io::Read;
use std::fs::File;

use memmap2::Mmap;


struct Node {
    pub f_score: f32,
    pub index: usize
}

struct NodeState {
    pub g_score: f32,
    pub dir: u8
}


impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.total_cmp(&self.f_score)
 }
}
impl PartialOrd for Node {
 fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl PartialEq for Node {
 fn eq(&self, other: &Self) -> bool { self.cmp(other) == Ordering::Equal }
}
impl Eq for Node {}

pub fn search(
    dem_slice: &[f32],
    rows: usize,
    cols: usize,
    mx_slice: &[f64],
    my_slice: &[f64],
    start: (usize, usize),
    end: (usize, usize)
) -> f64 {
    let mut open = BinaryHeap::<Node>::new();
    let mut close = BinaryHeap::<Node>::new();
    get_movement_cost()
}

fn get_movement_cost() -> f64 {
    return 1.0;
}



