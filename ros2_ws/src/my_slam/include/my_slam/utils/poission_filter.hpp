#ifndef POISSION_FILTER_HPP
#define POISSION_FILTER_HPP
#include <common.hpp>
#include <unordered_map>

namespace utils {
template <std::size_t dim> class PoissionFilter {
public:
  typedef vector<dim> point_type;
  typedef std::array<int, dim> grid_idx_type;
  explicit PoissionFilter(double r)
      : radius_(r), radius_sq_(r * r), grid_size_(r / sqrt(double(dim))),
        grid_span_(int(ceil(sqrt(double(dim))))) {
    // https://www.geeksforgeeks.org/how-to-use-unordered_map-efficiently-in-c/
    s_grid.max_load_factor(0.25); // reduce the probability of collision
  };

  void clear() {
    points.clear();
    s_grid.clear();
  }

  void preset_point(const point_type &point) {
    auto idx = to_index(point);
    s_grid[idx] = points.size();
    points.emplace_back(point);
  }

  void preset_points(const std::vector<point_type> &points) {
    for (const auto &p : points)
      preset_point(p);
  }

  bool allow_point(const point_type &point) const {
    grid_idx_type idx;
    return point_check(point, idx);
  }

  bool insert_point(const point_type &point) {
    grid_idx_type idx;
    if (point_check(point, idx)) {
      s_grid[idx] = points.size();
      points.emplace_back(point);
      return true;
    }

    return false;
  }

  void insert_points(std::vector<point_type> &cp) {

    size_t pt_before = points.size();
    for (const auto &p : cp) {
      insert_point(p);
    }

    std::vector<point_type>(points.begin() + pt_before, points.end()).swap(cp);
  }

  const std::vector<point_type> &get_points() const { return points; }

private:
  struct IndexHash {
    // to reduce collisions that arises for hashing map keys -> good when
    // expecting large data
    std::size_t operator()(const grid_idx_type &idx) const {
      size_t seed = 0;
      for (size_t i = 0; i < dim; i++) {
        // 0x9e3779b9 -> irrational number hierachy... We stan an irrational
        // king (seed << 6) + (seed >> 2) -> for shuffling bits | ^-XOR function
        seed ^=
            std::hash<int>{}(idx[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

  grid_idx_type to_index(const point_type &p) {
    grid_idx_type idx;
    for (size_t i = 0; i < dim; ++i) {
      idx[i] = int(floor(p[i] / grid_size_));
    }
    return idx;
  }

  bool point_check(const point_type &p, grid_idx_type &index) {
    index = to_index(p);
    grid_idx_type start = index, stop = index;
    for (size_t idx = 0; idx < dim; ++idx) {
      start[idx] -= grid_span_;
      stop[idx] += grid_span_;
    }

    grid_idx_type curr = start;
    while (curr[dim - 1] <= stop[dim - 1]) {
      curr[0]++;
      for (size_t i = 0; i < dim - 1 && curr[i] > stop[i]; ++i) {
        curr[i] = start[i];
        curr[i + 1]++;
      }

      auto it = s_grid.find(curr);
      if (it != s_grid.end()) {
        if ((p - points[it->second]).norm() < radius_sq_)
          return false;
      }
    }
    return true;
  }

  /*-------------------------------*/
  double radius_ = 0.0;
  double radius_sq_ = 0.0;
  double grid_size_ = 0.0;
  int grid_span_;
  std::vector<point_type> points;
  std::unordered_map<grid_idx_type, size_t, IndexHash> s_grid;
};
} // namespace utils

#endif