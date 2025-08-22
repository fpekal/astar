#include <iostream>

#include "include/astar.hpp"

struct Point2D {
  int x, y;
  bool operator==(const Point2D &o) const { return x == o.x && y == o.y; }
};

template <> struct std::hash<Point2D> {
  size_t operator()(const Point2D &p) const {
    return std::hash<int>()(p.x * 73856093 ^ p.y * 19349663);
  }
};

int main() {
  // Create the map of the maze
  std::vector<std::string> map{
		"#######",
		"#.#.#.#",
		"#.#...#",
		"#.###.#",
		"#.....#",
		"#######"
	};

  Point2D start{1, 1};
  Point2D goal{3, 1};

  // How hard is it to get from one point to another
  auto get_weight = [](const Point2D &, const Point2D &) { return 1.0; };

  // Approximation of the distance between the `a` point and `b` point (`b` will
  // always be the goal of the graph)
  auto heuristic = [](const Point2D &a, const Point2D &b) {
    return static_cast<double>(abs(a.x - b.x) + abs(a.y - b.y));
  };

  // Convert the map to a list of edges of the current point.
  // Because this is a callback you are allowed to create the graph dynamically.
  auto get_edges = [&map](const Point2D &point) {
    std::vector<Point2D> ret;

    for (int x = -1; x <= 1; ++x) {
      if (x == 0)
        continue;

      int new_x = point.x + x;
      if (map[point.y][new_x] == '.')
        ret.emplace_back(new_x, point.y);
    }

    for (int y = -1; y <= 1; ++y) {
      if (y == 0)
        continue;

      int new_y = point.y + y;
      if (map[new_y][point.x] == '.')
        ret.emplace_back(point.x, new_y);
    }

    return ret;
  };

  // Run the A* algorithm
  auto path = astar<Point2D>(start, goal, get_edges, get_weight, heuristic);

  if (path) {
    std::cout << "Path found:\n";
    for (auto &p : *path) {
      std::cout << "(" << p.x << "," << p.y << ") ";
    }
    std::cout << "\n";
  } else {
    std::cout << "No path!\n";
  }
}
