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
  auto weight_so_far = std::unordered_map<Vertex, WeightType, Hash>();

  auto cmp = [&](Link a, Link b) {
    return weight_so_far.at(a.from) + a.w + heuristic(a.to, goal) >
           weight_so_far.at(b.from) + b.w + heuristic(b.to, goal);
  };
  auto queue = std::priority_queue<Link, std::vector<Link>, decltype(cmp)>(cmp);

  weight_so_far[start] = WeightType(0);
  for_each_neighbors(start,
                     [&](Vertex neighbor, WeightType w) { queue.push({start, neighbor, w}); });

  while (queue.size() != 0) {
    auto [current, next, w] = queue.top();
    if (visited.contains(next)) {
      queue.pop();
      continue;
    }
    weight_so_far[next] = weight_so_far[current] + w;
    best_predecessor[next] = current;
    visited.insert(next);
    if (next == goal) break;
    queue.pop();

    for_each_neighbors(next,
                       [&](Vertex neighbor, WeightType w) { queue.push({next, neighbor, w}); });
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