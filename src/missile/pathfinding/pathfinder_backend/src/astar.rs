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
) -> Vec<(usize, usize)> {

    let mut node_explored: usize = 0;
    // constructing data structure to store unexplored and explored nodes
    let mut open_set = BinaryHeap::<Node>::new();
    let mut visited: HashMap<usize, NodeState> = HashMap::new();

    let start_idx = start.0 * cols + start.1;
    let end_idx = end.0 * cols + end.1;

    // insert start node into open set and visited
    open_set.push(Node{ f_score: 0.0, index: start_idx });
    visited.insert(start_idx, NodeState { g_score: 0.0, dir: 8 }); // invalid dir

    node_explored += 1;

    if (start_idx < 0 || start_idx >= dem_slice.len()
        || end_idx < 0 || end_idx >= dem_slice.len()) {
        return Vec::new();
    }

    // main pathfinding loop
    while let Some(current) = open_set.pop() {
        node_explored += 1;

        let curr_idx= current.index;
        if (curr_idx == end_idx) {
            println!("Target acquired... Total of {} nodes explored", node_explored);
            return reconstruct_path(&visited, curr_idx, rows, cols);
        }

        let row = curr_idx / cols;
        let col = curr_idx % cols;
    }
    Vec::new()
}

fn get_movement_cost() -> f64 {
    return 1.0;
}

// reconstructing path from end to start, <by GPT-6.2 Sol>
fn reconstruct_path(
    visited: &HashMap<usize, NodeState>,
    mut current_idx: usize,
    rows: usize,
    cols: usize,
) -> Vec<(usize, usize)> {
    const DIRECT_R: [isize; 8] = [-1, 1, 0, 0, -1, -1, 1, 1];
    const DIRECT_C: [isize; 8] = [0, 0, -1, 1, -1, 1, -1, 1];

    let mut path: Vec<(usize, usize)> = Vec::new();

    // A valid parent chain cannot contain more nodes than visited.
    for _ in 0..visited.len() {
        let Some(state) = visited.get(&current_idx) else {
            return Vec::new();
        };

        let row = current_idx / cols;
        let col = current_idx % cols;
        path.push((row, col));

        let dir = state.dir;

        if dir == 8 {
            path.reverse();
            return path;
        }

        if dir > 7 {
            return Vec::new();
        }

        let parent_row = row as isize + DIRECT_R[dir as usize];
        let parent_col = col as isize + DIRECT_C[dir as usize];

        if parent_row < 0
            || parent_row >= rows as isize
            || parent_col < 0
            || parent_col >= cols as isize
        {
            return Vec::new();
        }

        current_idx = (parent_row as usize) * cols + (parent_col as usize);
    }

    Vec::new()
}



