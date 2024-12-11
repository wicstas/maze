#include <path_finding.h>
#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <vector>

void assert_impl(const char* expr, auto... args) {
  std::cout << expr << " failed\t";
  ((std::cout << args << ' '), ..., (std::cout << '\n'));
  std::abort();
}

#define assert(x, ...) \
  if (!(x)) assert_impl(#x, __VA_ARGS__);

template <typename T>
using vector = std::vector<T>;

struct Vec2 {
  bool operator==(Vec2 rhs) const { return x == rhs.x && y == rhs.y; }

  float x = 0;
  float y = 0;
};
template <>
struct std::hash<Vec2> {
  size_t operator()(Vec2 v) const { return std::hash<float>()(v.x) ^ std::hash<float>()(v.y); }
};

class Maze {
 public:
  enum Direction { Right, Up, Left, Down };

  Maze(int width, int height) : w(width), h(height) {
    fill_n(back_inserter(cells), w, vector<uint8_t>(h, 0b11111));
    generate();
  }

  uint8_t operator[](int x, int y) const { return cells[x][y]; }
  bool operator[](int x, int y, Direction dir, bool infinite = false) const {
    if (infinite)
      if (x < 0 || y < 0 || x >= w || y >= h) return false;

    return (cells[x][y] >> int(dir)) & 1;
  }
  bool has_vertical_wall(int x, int y) const {
    auto& maze = *this;
    return maze[x - 1, y, Right, true] || maze[x, y, Left, true];
  }
  bool has_horizontal_wall(int x, int y) const {
    auto& maze = *this;
    return maze[x, y - 1, Up, true] || maze[x, y, Down, true];
  }

  int width() const { return w; }
  int height() const { return h; }

 private:
  bool generate(int x = 0, int y = 0) {
    cells[x][y] ^= 0b10000;
    while (true) {
      auto dir = random_new_direction(x, y);
      if (dir == -1) return false;
      auto next_x = x, next_y = y;
      if (dir == 0) {
        cells[x][y] ^= 0b0001;
        cells[x + 1][y] ^= 0b0100;
        next_x++;
      } else if (dir == 1) {
        cells[x][y] ^= 0b0010;
        cells[x][y + 1] ^= 0b1000;
        next_y++;
      } else if (dir == 2) {
        cells[x][y] ^= 0b0100;
        cells[x - 1][y] ^= 0b0001;
        next_x--;
      } else if (dir == 3) {
        cells[x][y] ^= 0b1000;
        cells[x][y - 1] ^= 0b0010;
        next_y--;
      }
      if (generate(next_x, next_y)) return true;
    }
  }
  int random_new_direction(int x, int y) {
    int list[4];
    int n = 0;
    if (x != w - 1 && cells[x + 1][y] & 0b10000) list[n++] = 0;
    if (y != h - 1 && cells[x][y + 1] & 0b10000) list[n++] = 1;
    if (x != 0 && cells[x - 1][y] & 0b10000) list[n++] = 2;
    if (y != 0 && cells[x][y - 1] & 0b10000) list[n++] = 3;
    if (n == 0)
      return -1;
    else
      return list[rand() % n];
  }

  vector<vector<uint8_t>> cells;
  int w, h;
};

float heuristic(Vec2 p0, Vec2 p1) {
  return std::sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
}

struct Link {
  Vec2 x;
  float w;
};
using VisibilityGraph = std::unordered_multimap<Vec2, Link>;

bool is_wall_edge(const Maze& maze, int x, int y) {
  if (maze[x - 1, y - 1, Maze::Up, true] && maze[x, y - 1, Maze::Up, true]) return true;
  if (maze[x - 1, y, Maze::Down, true] && maze[x, y, Maze::Down, true]) return true;
  if (maze[x - 1, y, Maze::Right, true] && maze[x - 1, y - 1, Maze::Right, true]) return true;
  if (maze[x, y, Maze::Left, true] && maze[x, y - 1, Maze::Left, true]) return true;

  return false;
}

bool is_visible(const Maze& maze, Vec2 p0, Vec2 p1) {
  if (p0.x < 0 || p1.x < 0 || p0.x >= maze.width() || p1.x >= maze.width()) return false;
  if (p0.y < 0 || p1.y < 0 || p0.y >= maze.height() || p1.y >= maze.height()) return false;

  if (p0.x > p1.x) std::swap(p0, p1);
  for (float x = std::floor(p0.x + 1); x < p1.x; x += 1) {
    auto y = lerp(p0.y, p1.y, (x - p0.x) / (p1.x - p0.x));

    if (y == std::floor(y)) {
      if (maze.has_vertical_wall(x, y) && maze.has_vertical_wall(x, y - 1)) return false;
      if (p1.y > p0.y && maze.has_vertical_wall(x, y) && maze.has_horizontal_wall(x, y))
        return false;
      if (p1.y > p0.y && maze.has_vertical_wall(x, y - 1) && maze.has_horizontal_wall(x - 1, y))
        return false;
      if (p1.y < p0.y && maze.has_vertical_wall(x, y) && maze.has_horizontal_wall(x - 1, y))
        return false;
      if (p1.y < p0.y && maze.has_vertical_wall(x, y - 1) && maze.has_horizontal_wall(x, y))
        return false;
    } else if (maze.has_vertical_wall(x, y))
      return false;
  }

  if (p0.y > p1.y) std::swap(p0, p1);
  for (float y = std::floor(p0.y + 1); y < p1.y; y += 1) {
    auto x = lerp(p0.x, p1.x, (y - p0.y) / (p1.y - p0.y));

    if (x == std::floor(x)) {
      if (maze.has_horizontal_wall(x, y) && maze.has_horizontal_wall(x - 1, y)) return false;
      if (p1.x > p0.x && maze.has_vertical_wall(x, y) && maze.has_horizontal_wall(x, y))
        return false;
      if (p1.x > p0.x && maze.has_vertical_wall(x, y - 1) && maze.has_horizontal_wall(x - 1, y))
        return false;
      if (p1.x < p0.x && maze.has_vertical_wall(x, y) && maze.has_horizontal_wall(x - 1, y))
        return false;
      if (p1.x < p0.x && maze.has_vertical_wall(x, y - 1) && maze.has_horizontal_wall(x, y))
        return false;
    } else if (maze.has_horizontal_wall(x, y))
      return false;
  }

  return true;
}

VisibilityGraph build_visibility_graph(const Maze& maze, Vec2 start, Vec2 goal) {
  auto graph = VisibilityGraph();
  const auto pad = 0.1f;

  if (is_visible(maze, start, goal)) {
    graph.emplace(start, Link(goal, heuristic(start, goal)));
    return graph;
  }

  for (int y = 0; y <= maze.height(); y++) {
    for (int x = 0; x <= maze.width(); x++) {
      if (is_wall_edge(maze, x, y)) continue;

      for (int yy = 0; yy <= maze.height(); yy++) {
        for (int xx = 0; xx <= maze.width(); xx++) {
          if (is_wall_edge(maze, xx, yy)) continue;

          for (float a = -pad; a <= pad; a += pad * 2) {
            for (float b = -pad; b <= pad; b += pad * 2) {
              for (float c = -pad; c <= pad; c += pad * 2) {
                for (float d = -pad; d <= pad; d += pad * 2) {
                  auto p0 = Vec2(x + a, y + b);
                  auto p1 = Vec2(xx + c, yy + d);
                  if (p0 == p1) continue;

                  if (is_visible(maze, p0, p1)) graph.emplace(p0, Link(p1, heuristic(p0, p1)));
                }
              }
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    auto [x, y] = i == 0 ? start : goal;
    for (int yy = 0; yy <= maze.height(); yy++) {
      for (int xx = 0; xx <= maze.width(); xx++) {
        if (is_wall_edge(maze, xx, yy)) continue;

        for (float a = -pad; a <= pad; a += pad * 2) {
          for (float b = -pad; b <= pad; b += pad * 2) {
            auto p0 = Vec2(x, y);
            auto p1 = Vec2(xx + a, yy + b);

            if (p0 == p1) continue;
            if (is_visible(maze, p0, p1)) {
              graph.emplace(p0, Link(p1, heuristic(p0, p1)));
              graph.emplace(p1, Link(p0, heuristic(p0, p1)));
            }
          }
        }
      }
    }
  }

  return graph;
}

template <typename It>
auto as_range(const std::pair<It, It>& p) {
  struct range {
    It begin_, end_;
    It begin() const { return begin_; }
    It end() const { return end_; }
  };
  return range{p.first, p.second};
}

int main() {
  const auto n = 20;
  auto maze = Maze(n, n);

  const auto S = 50.0f;
  const auto screen_width = n * S;
  const auto screen_height = n * S;

  InitWindow(screen_width, screen_height, "Test");

  SetTargetFPS(60);

  Vec2 p0, p1;
  auto i = 0;

  auto path = vector<Vec2>();
  auto graph = VisibilityGraph();
  graph = build_visibility_graph(maze, p0, Vec2(maze.width(), maze.height()));

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    for (int y = 0; y < n; y++) {
      for (int x = 0; x < n; x++) {
        auto cell = maze[x, y];
        if (cell & 0b0001)
          DrawLineEx({(x + 1) * S, (y + 0) * S}, {(x + 1) * S, (y + 1) * S}, 2, BLACK);
        if (cell & 0b0010)
          DrawLineEx({(x + 0) * S, (y + 1) * S}, {(x + 1) * S, (y + 1) * S}, 2, BLACK);
        if (cell & 0b0100)
          DrawLineEx({(x + 0) * S, (y + 0) * S}, {(x + 0) * S, (y + 1) * S}, 2, BLACK);
        if (cell & 0b1000)
          DrawLineEx({(x + 0) * S, (y + 0) * S}, {(x + 1) * S, (y + 0) * S}, 2, BLACK);
      }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      auto [x, y] = Vector2Scale(GetMousePosition(), 1 / S);

      i %= 2;
      if (i == 0) {
        path.clear();
        p0 = {x, y};
      } else {
        p1 = {x, y};
        graph = build_visibility_graph(maze, p0, p1);
        path = path_finding_astar(
            p0, p1,
            [&](Vec2 p, auto f) {
              for (auto [neighbor, link] : as_range(graph.equal_range(p))) f(link.x, link.w);
            },
            heuristic);
      }
      i++;
    }

    if (i > 0) DrawCircle(p0.x * S, p0.y * S, 5, BLUE);
    if (i > 1) DrawCircle(p1.x * S, p1.y * S, 5, RED);
    if (i == 2) DrawText(("Path length: " + std::to_string(path.size() - 1)).c_str(), 0, 0, 50, GREEN);
    if (i == 2 && path.size() == 0) DrawText("No path found", 0, 50, 50, GREEN);

    for (int i = 0; i + 1 < path.size(); i++) {
      auto p0 = path[i], p1 = path[i + 1];
      DrawLineEx({p0.x * S, p0.y * S}, {p1.x * S, p1.y * S}, 3, PURPLE);
    }

    EndDrawing();
  }

  CloseWindow();

  return 0;
}
