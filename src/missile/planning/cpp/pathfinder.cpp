#include <algorithm>
#include <iostream>
#include <limits>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <queue>
#include <vector>

namespace py = pybind11;

// Setting up priority queue Node (heap)
struct Node {
  float f_score;
  int index;

  // Min-heap comparator, lowest on top
  // Sort node based on f_score
  bool operator>(const Node &other) const { return f_score > other.f_score; }
};

class PathfinderCPP {

  // Constructors and main A* Pathfinding Algorithm
public:
  // Constructor: accepting numpy array and var from Python
  PathfinderCPP(py::array_t<float> dem_array,         // float: float32
                py::array_t<double> lat_lookup_table, // double: float64
                double meter_per_y,
                int rows,
                int cols) // order matters!!
      : m_rows(rows), m_cols(cols), m_meter_per_y(meter_per_y) {
    auto buffer_dem = dem_array.request();
    auto buffer_lat = lat_lookup_table.request();

    // Pointer for raw access
    ptr_dem = static_cast<float *>(buffer_dem.ptr);
    ptr_lat_lookup = static_cast<double *>(buffer_lat.ptr);

    m_total_pixels = m_rows * m_cols;
    std::cout << "C++ Engine Online. Map Size: " << m_rows << "x" << m_cols
              << " (" << m_total_pixels << " pixels)" << std::endl;
  }

  // --- Main A* Pathfinding Algorithm ---
  // std::pair in std::vector == storing tuple() in py list
  std::vector<std::pair<int, int>> find_path(int start_idx, int end_idx,
                                             float heuristic_weight = 1) {
    // Bound check
    if (start_idx < 0 || start_idx >= m_total_pixels || end_idx < 0 ||
        end_idx >= m_total_pixels) {
      std::cerr << "[C++] Error: Start or End index out of bounds!"
                << std::endl;
      return {};
    }

    if (heuristic_weight < 0) {
      std::cout << "heuristic_weight cannot be less than 1" << std::endl;
    }

    // Allocating dense memory for our arrays
    std::vector<float> g_score(m_total_pixels,
                               std::numeric_limits<float>::infinity());
    std::vector<int> came_from(
        m_total_pixels, -1); // fill the array with -1 (None, Null...) first

    // Initiate g_score
    g_score[start_idx] = 0.0f;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    open_set.push({0.0f, start_idx});

    long node_explored = 0;

    // Main pathfinding loop
    while (!open_set.empty()) {
      node_explored += 1;
      Node current = open_set.top();
      open_set.pop();

      int curr_idx = current.index; // access the index from current Node

      // Check if target reached by pathfinder
      if (curr_idx == end_idx) {
        std::cout << "Target acquired... Total of " << node_explored
                  << " nodes explored." << std::endl;
        return reconstruct_path(came_from, end_idx);
      }

      // Getting row, col from index for the thousand time
      int row = curr_idx / m_cols;
      int col = curr_idx % m_cols;

      // Search neighbouring pixels (directions)
      const int direct_r[] = {-1, 1, 0, 0, -1, -1, 1, 1};
      const int direct_c[] = {0, 0, -1, 1, -1, 1, -1, 1};

      for (int i = 0; i < 8; ++i) {
        // update row, col pixel value for each direction
        int new_row = row + direct_r[i];
        int new_col = col + direct_c[i];
        // Check boundaries
        if (new_row >= 0 && new_row < m_rows && new_col >= 0 &&
            new_col < m_cols) {
          int neigh_idx = new_row * m_cols + new_col;

          // obtain movement cost
          float move_cost = get_movement_cost(curr_idx, neigh_idx);
          if (move_cost == std::numeric_limits<float>::infinity())
            continue;

          // establish g_score
          float temp_g_score = g_score[curr_idx] + move_cost;

          // Discover better path and update to open_set for each neighbor
          if (temp_g_score < g_score[neigh_idx]) {
            came_from[neigh_idx] = curr_idx;
            g_score[neigh_idx] = temp_g_score;

            // get f_score (find h_score first)
            float h_score = heuristic(neigh_idx, end_idx);
            float f_score = temp_g_score + (h_score * heuristic_weight);

            open_set.push({f_score, neigh_idx});
          }
        }
      }
    }
    std::cout << "[C++] Queue exhausted, no path found..." << std::endl;
    return {}; // return None
  }

  py::array_t<float> get_terrain_profile(py::array_t<float> row_path,
                                         py::array_t<float> col_path) {
    // Getting access to input array
    auto buffer_row = row_path.request();
    auto buffer_col = col_path.request();

    float *ptr_row = static_cast<float *>(buffer_row.ptr);
    float *ptr_col = static_cast<float *>(buffer_col.ptr);

    // get the size of the numpy array_t, we assume col and row have same size
    size_t num_points = buffer_row.size;

    // Prepare the result array
    auto result = py::array_t<float>(num_points);
    auto buffer_result = result.request();
    float *ptr_result = static_cast<float *>(buffer_result.ptr);

    // Main loop
    for (size_t i = 0; i < num_points; ++i) {

      float row_float = ptr_row[i];
      float col_float = ptr_col[i];

      // round to nearest pixel, use static_cast to convert to integer
      // as DEM pixel coordinate are integer pair
      int row = static_cast<int>(std::round(row_float));
      int col = static_cast<int>(std::round(col_float));

      // basic safety checks
      if (row >= 0 && row < m_rows && col >= 0 && col < m_cols) {
        // get height by ptr_dem[index]
        ptr_result[i] = ptr_dem[row * m_cols + col];
      } else {
        ptr_result[i] = -9999.0f;
      }
    }
    return result;
  }

  // Helper Functions
private:
  float *ptr_dem;
  double *ptr_lat_lookup;

  double m_meter_per_y; // Was m_meter_per_z. Represents meters per row.
  int m_rows;
  int m_cols;
  int m_total_pixels;

  // Helper functions (using inline to speed up)
  // --- _heuristic ---
  inline float heuristic(int idx1, int idx2) {
    int row1 = idx1 / m_cols;
    int col1 = idx1 % m_cols;
    int row2 = idx2 / m_cols;
    int col2 = idx2 % m_cols;

    float dist_y = std::abs(row2 - row1) * m_meter_per_y;
    int midrow = (row1 + row2) / 2;
    double dist_x = std::abs(col2 - col1) * ptr_lat_lookup[midrow];

    return (float)std::sqrt(dist_x * dist_x + dist_y * dist_y);
  }

  // --- _get_movement_cost ---
  inline float get_movement_cost(int current_idx, int neighbor_idx) {

    float current_elev = ptr_dem[current_idx];
    float neighbor_elev = ptr_dem[neighbor_idx];

    // New: slope penalty for trying to force missile to stay in valley instead
    // of taking lazy shortcut, risking exposure
    float climb = neighbor_elev - current_elev;
    float height_diff = std::abs(climb); // yeah its just like climb but with abs
    float slope_penalty = height_diff * 5.0f;

    // No-data check
    if (current_elev <= -100.0f || neighbor_elev <= -100.0f)
      return std::numeric_limits<float>::infinity();

    // Base distances: recalculate heuristic here to avoid function call cost
    // yes, so extract every penny as possible lol, for handling at least 800
    // million pixels
    int row1 = current_idx / m_cols;
    int col1 = current_idx % m_cols;
    int row2 = neighbor_idx / m_cols;
    int col2 = neighbor_idx % m_cols;

    double dist_y = std::abs(row2 - row1) * m_meter_per_y;
    double dist_x = std::abs(col2 - col1) * ptr_lat_lookup[row1];
    float dist_cost = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));

    // !! Crucial !! Apply extra serious penalty on altitude change, otherwise
    // the algorithm will still choose short distance vs low-altitude terrain
    // Note: This is also the part which results in the significant increase in
    // compile time (at least in Python)
    double height_penalty = neighbor_elev * 0.8;

    // --- Handle: decision penalties ---

    // downhill
    if (climb < 0)
      return dist_cost + (climb * 0.5) + height_penalty + slope_penalty;

    // uphill
    float gradient = climb / (dist_cost + 1e-6f);
    float penalty = 0.0f;

    // Level 1: gentle slope (<3°)
    if (gradient <= 0.05f) {
      // ignore, penalty weight: low (scale of 2)
      penalty = climb * 2.0f;
    }
    // Level 2: moderate (3-8.5°)
    else if (gradient <= 0.15f) {
      // penalty weight: medium (10)
      penalty = climb * 10.0f;
    }
    // Level 3: steep (>8.5°)
    else {
      // penalty weight: high (100), the program will choose other path unless
      // still the closest even with penalty
      penalty = climb * 100.0f; // apply a lot of penalty to very steep
    }
    return dist_cost + penalty + height_penalty + slope_penalty;
  }

  // --- _reconstruct_path ---
  std::vector<std::pair<int, int>>
  reconstruct_path(const std::vector<int> &came_from, int current_idx = 0) {
    std::vector<std::pair<int, int>> path;

    while (current_idx != -1) {
      int row = current_idx / m_cols;
      int col = current_idx % m_cols;

      path.push_back({row, col});

      current_idx = came_from[current_idx];
    }

    // Reverse: end -> start to start -> end nodes
    std::reverse(path.begin(), path.end());
    return path;
  }
};

// Module definition
PYBIND11_MODULE(missile_backend, m) { // (module name, handle)
  py::class_<PathfinderCPP>(m, "PathfinderCPP")
      .def(py::init<py::array_t<float>, py::array_t<double>, double, int,
                    int>()) // exposing constructors
      .def("find_path",
           &PathfinderCPP::find_path) // exposes the find_path method to python
      .def("get_terrain_profile", &PathfinderCPP::get_terrain_profile);
}