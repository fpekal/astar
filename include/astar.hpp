#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

template <typename Node> struct ASTAR_PQItem {
  double priority;
  Node node;

  bool operator>(const ASTAR_PQItem &other) const {
    return priority > other.priority;
  }
};

template <typename Node>
std::optional<std::vector<Node>>
astar(const Node &start, const Node &goal,
      std::function<std::vector<Node>(const Node &)> get_edges,
      std::function<double(const Node &, const Node &)> get_weight,
      std::function<double(const Node &, const Node &)> heuristic) {
  std::priority_queue<ASTAR_PQItem<Node>, std::vector<ASTAR_PQItem<Node>>,
                      std::greater<>>
      open_set;
  open_set.push({0.0, start});

  std::unordered_map<Node, Node> came_from;
  std::unordered_map<Node, double> g_score;
  g_score[start] = 0.0;

  while (!open_set.empty()) {
    Node current = open_set.top().node;
    open_set.pop();

    if (current == goal) {
      // Path reconstruction
      std::vector<Node> path;
      for (Node n = goal;; n = came_from[n]) {
        path.push_back(n);
        if (n == start)
          break;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const Node &neighbor : get_edges(current)) {
      double tentative_g = g_score[current] + get_weight(current, neighbor);
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push({f, neighbor});
      }
    }
  }

  return std::nullopt; // No path
}
