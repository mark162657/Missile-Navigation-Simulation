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

// --- Lookup Tables ---

// (dr, dc): ↑   ↓   ←   →   ↖   ↗   ↙   ↘
const DIRS: [(isize, isize); 8] = [
    (-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1),
];
const REVERSE_DIR: [u8; 8] = [1, 0, 3, 2, 7, 6, 5, 4];

/*
Discrete turn steps in 45° increments (0=0°, 1=45°, 2=90°, 3=135°, 4=180°)
Matches DIRS order: ↑   ↓   ←   →   ↖   ↗   ↙   ↘
The greater the degrees, the greater the cost
*/

const TURN_STEPS: [[u8; 8]; 8] = [
    [0, 4, 2, 2, 1, 1, 3, 3], // 0: N
    [4, 0, 2, 2, 3, 3, 1, 1], // 1: S
    [2, 2, 0, 4, 1, 3, 1, 3], // 2: W
    [2, 2, 4, 0, 3, 1, 3, 1], // 3: E
    [1, 3, 1, 3, 0, 2, 2, 4], // 4: NW
    [1, 3, 3, 1, 2, 0, 4, 2], // 5: NE
    [3, 1, 1, 3, 2, 4, 0, 2], // 6: SW
    [3, 1, 3, 1, 4, 2, 2, 0], // 7: SE
];


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

                let row = curr_idx / cols;
                let col = curr_idx % cols;
                let curr_elev = dem_slice[curr_idx];
                let incoming_dir = visited[&curr_idx].dir;
                let mx = mx_slice[row] as f32;
                let my = my_slice[row] as f32;
                let diag = mx.hypot(my);

                let neigh_idx = new_row as usize * cols + new_col as usize;
                let neigh_elev = dem_slice[neigh_idx];

                let w = &CostWeight {
                    climb_slope: 40.0,
                    descent_slope: 20.0,
                    max_grade: 0.35,
                    turn_45: 0.03,
                    turn_90: 0.35,
                    too_sharp: f32::INFINITY,
                };
                let neigh_idx = (new_row as usize) * cols + (new_col as usize);
                let move_cost = get_movement_cost(dem_slice, dist_m,
                                                  curr_elev, neigh_elev, incoming_dir, i as u8, w);
                if !move_cost.is_finite() {
                    continue;
                }
                let temp_g_score = visited[&curr_idx].g_score + move_cost;

                let is_better = match visited.get(&neigh_idx) {
                    None => true,
                    Some(state) => temp_g_score < state.g_score,
                };

                if is_better {
                    // Store `i as u8` (parent -> child)
                    visited.insert(
                        neigh_idx,
                        NodeState {
                            g_score: temp_g_score,
                            dir: i as u8,
                        },
                    );
                    let h_score = heuristic(new_row as usize, new_col as usize, end.0, end.1, mx_slice, my_slice);
                    let f_score = temp_g_score + (h_score * heuristic_weight);
                    open_set.push(Node {
                        f_score,
                        index: neigh_idx,
                    });
                }
            }
        }
    }
    println!("No path found...");
    Vec::new()
}

#[inline]
fn heuristic(
    r1: usize,
    c1: usize,
    r2: usize,
    c2: usize,
    mx_slice: &[f64],
    my_slice: &[f64],
) -> f32 {
    let mid_row = (r1 + r2) / 2;
    let dy = (r2.abs_diff(r1) as f64) * my_slice[mid_row];
    let dx = (c2.abs_diff(c1) as f64) * mx_slice[mid_row];
    dx.hypot(dy) as f32
}

fn get_movement_cost(dem_slice: &[f32],
                     dist_m: f32, // dist from current node to next node based on current direction
                     curr_elev: f32,
                     neigh_elev: f32,
                     incoming_dir: u8,
                     outgoing_dir: u8,
                     w: &CostWeight,
    ) -> f32 {

    if curr_elev <= -100.0 || neigh_elev <= -100.0 {
        return f32::INFINITY;
    }

    // 1. turn cost
    let turn_penalty = if incoming_dir >= 8 {
        0.0
    } else {
        match TURN_STEPS[incoming_dir as usize][outgoing_dir as usize] {
            0 => 0.0,
            1 => w.turn_45,
            2 => w.turn_90,
            3 => w.too_sharp,
            _ => return f32::INFINITY, // No 180deg reversal
        }
    };
    // allow for early exit (3 and _ hit infinity)
    if !turn_penalty.is_finite() {
        return f32::INFINITY;
    }

    // 2. change of elevation cost (penalize slope not altitude)
    let dz = neigh_elev - curr_elev;
    let grade = dz.abs() / dist_m.max(1e-3);
    if grade > w.max_grade {
        return f32::INFINITY;
    }

    let slope_w = if dz > 0.0 {
        w.climb_slope
    } else {
        w.descent_slope
    };

    // grade^2 so one sudden steep climb costs much more than spreading the same climb smoothly
    let slope_penalty = slope_w * grade.powi(2);

    // return total cost function
    return dist_m * (1.0 + slope_penalty + turn_penalty);
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


