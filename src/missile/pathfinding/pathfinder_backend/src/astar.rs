use pyo3::prelude::*;
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::cmp::Ordering;
use std::io::Read;
use std::fs::File;
use std::collections::HashMap;
use std::thread::current;
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

pub struct CostWeight {
    pub climb_slope: f32,
    pub descent_slope: f32,
    pub max_grade: f32,
    pub turn_45: f32,
    pub turn_90: f32,
    pub too_sharp: f32
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.total_cmp(&self.f_score)
 }
}
impl PartialOrd for Node { fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) } }
impl PartialEq for Node { fn eq(&self, other: &Self) -> bool { self.cmp(other) == Ordering::Equal } }
impl Eq for Node {}

// (dr, dc): ↑   ↓   ←   →   ↖   ↗   ↙   ↘
const DIRS: [(isize, isize); 8] = [
    (-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1),
];
const REVERSE_DIR: [u8; 8] = [1, 0, 3, 2, 7, 6, 5, 4];

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

        let mx = mx_slice[row] as f32;
        let my = my_slice[row] as f32;
        let diag = mx.hypot(my);

        for (i, (dr, dc)) in DIRS.iter().enumerate() {
            let new_row = row as isize + dr;
            let new_col = col as isize + dc;

            // check boundaries
            if new_row >= 0
                && new_row < rows as isize
                && new_col >= 0
                && new_col < cols as isize
            {
                let dist_m = match i {
                    0 | 1 => my,
                    2 | 3 => mx,
                    _ => diag,
                };
                let w = CostWeight {
                    climb_slope: 40.0,
                    descent_slope: 20.0,
                    max_grade: 0.35,
                    turn_45: 0.03,
                    turn_90: 0.35,
                    too_sharp: f32::INFINITY,
                };
                let neigh_idx = (new_row as usize) * cols + (new_col as usize);
                let move_cost = get_movement_cost(dem_slice, dist_m, mx_slice, my_slice,
                                                  cols, curr_idx, neigh_idx, w);
            }
        }

    }
    Vec::new()
}

fn get_movement_cost(dem_slice: &[f32],
                     dist_m: f32,
                     mx_slice: &[f64],
                     my_slice: &[f64],
                     cols: usize,
                     curr_idx: usize,
                     neigh_idx: usize,
                     w: &CostWeight,
    ) -> f32 {
    let current_elev: f32 = dem_slice[curr_idx];
    let neighbor_elev: f32 = dem_slice[neigh_idx];

    if current_elev <= -100.0 || neighbor_elev <= -100.0 {
        return f32::INFINITY;
    }

    let row1 = curr_idx / cols;
    let col1 = curr_idx % cols;
    let row2 = neigh_idx / cols;
    let col2 = neigh_idx % cols;

    return 1.0;
}

// reconstructing path from end to start, <by GPT-6.2 Sol>
fn reconstruct_path(
    visited: &HashMap<usize, NodeState>,
    mut current_idx: usize,
    rows: usize,
    cols: usize,
) -> Vec<(usize, usize)> {
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

        // If `state.dir` stores the move direction `i` (parent -> child),
        // use `REVERSE_DIR[dir]` to step back to the parent:
        let (dr, dc) = DIRS[REVERSE_DIR[dir as usize] as usize];
        let parent_row = row as isize + dr;
        let parent_col = col as isize + dc;

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


