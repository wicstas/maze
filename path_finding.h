#pragma once
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// `Vertex` should be cheap, otherwise user should use a
// wrapper
template <std::equality_comparable Vertex, typename Hash = std::hash<Vertex>>
std::vector<Vertex> path_finding_astar(Vertex start, Vertex goal, auto&& for_each_neighbors,
                                       auto&& heuristic) {
  using WeightType = std::decay_t<decltype(heuristic(start, goal))>;
  struct Link {
    Vertex from, to;
    WeightType w;
  };
  auto visited = std::unordered_set<Vertex, Hash>{start};
  auto best_predecessor = std::unordered_map<Vertex, Vertex, Hash>();

  auto cmp = [&](Link a, Link b) {
    return a.w + heuristic(a.to, goal) > b.w + heuristic(b.to, goal);
  };
  auto queue = std::priority_queue<Link, std::vector<Link>, decltype(cmp)>(cmp);

  for_each_neighbors(start,
                     [&](Vertex neighbor, WeightType w) { queue.push({start, neighbor, 0 + w}); });

  while (queue.size() != 0) {
    auto [current, next, w] = queue.top();
    if (!visited.insert(next).second) {
      queue.pop();
      continue;
    }
    best_predecessor[next] = current;
    if (next == goal) break;
    queue.pop();

    for_each_neighbors(next, [&](Vertex neighbor, WeightType lnik_w) {
      queue.push({next, neighbor, w + lnik_w});
    });
  }

  if (!best_predecessor.contains(goal)) return {};

  auto path = std::vector<Vertex>{goal};
  while (goal != start) {
    goal = best_predecessor.at(goal);
    path.push_back(goal);
  }

  std::ranges::reverse(path);
  return path;
}