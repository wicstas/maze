#include <parallel.h>
#include <path_finding.h>
#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <vector>

template <typename T>
using vector = std::vector<T>;

void assert_impl(const char* expr, auto... args) {
  std::cout << expr << " failed\t";
  ((std::cout << args << ' '), ..., (std::cout << '\n'));
  std::abort();
}

#define assert(x, ...) \
  if (!(x)) assert_impl(#x, __VA_ARGS__);

struct Timer {
  float operator()() const { return (clock::now() - t0).count() * 1e-9f; }
  float reset() {
    auto elapsed = (*this)();
    t0 = clock::now();
    return elapsed;
  }

 private:
  using clock = std::chrono::high_resolution_clock;
  clock::time_point t0 = clock::now();
};

struct Vec2 {
  bool operator==(Vec2 rhs) const { return x == rhs.x && y == rhs.y; }
  Vec2& operator+=(Vec2 rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  Vec2 operator+(Vec2 rhs) const { return Vec2(*this) += rhs; }
  Vec2& operator*=(Vec2 rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
  }
  Vec2 operator*(Vec2 rhs) const { return Vec2(*this) *= rhs; }

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
  bool operator[](int x, int y, Direction dir, bool open_space = false) const {
    if (open_space)
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
  // return std::max(std::abs(p1.x - p0.x), std::abs(p1.y - p0.y));
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

  {
    if (p0.x > p1.x) std::swap(p0, p1);
    auto dydx = (p1.y - p0.y) / (p1.x - p0.x);
    auto x0 = p0.x;
    auto x = std::floor(p0.x + 1);
    auto y = p0.y + (x - x0) * dydx;
    for (; x < p1.x; x += 1, y += dydx) {
      if (y == std::floor(y)) {
        auto v0 = maze.has_vertical_wall(x, y), v1 = maze.has_vertical_wall(x, y - 1);
        if (v0 && v1) return false;
        if (p1.y > p0.y) {
          if (v0 && maze.has_horizontal_wall(x, y)) return false;
          if (v1 && maze.has_horizontal_wall(x - 1, y)) return false;
        } else if (p1.y < p0.y) {
          if (v0 && maze.has_horizontal_wall(x - 1, y)) return false;
          if (v1 && maze.has_horizontal_wall(x, y)) return false;
        }
      } else if (maze.has_vertical_wall(x, y))
        return false;
    }
  }

  {
    if (p0.y > p1.y) std::swap(p0, p1);
    auto dxdy = (p1.x - p0.x) / (p1.y - p0.y);
    auto y0 = p0.y;
    auto y = std::floor(p0.y + 1);
    auto x = p0.x + (y - y0) * dxdy;
    for (; y < p1.y; y += 1, x += dxdy) {
      if (x == std::floor(x)) {
        auto h0 = maze.has_horizontal_wall(x, y), h1 = maze.has_horizontal_wall(x - 1, y);
        if (h0 && h1) return false;
        if (p1.x > p0.x) {
          if (maze.has_vertical_wall(x, y) && h0) return false;
          if (maze.has_vertical_wall(x, y - 1) && h1) return false;
        } else if (p1.x < p0.x) {
          if (maze.has_vertical_wall(x, y) && h1) return false;
          if (maze.has_vertical_wall(x, y - 1) && h0) return false;
        }
      } else if (maze.has_horizontal_wall(x, y))
        return false;
    }
  }

  return true;
}

VisibilityGraph build_visibility_graph(const Maze& maze, Vec2 start, Vec2 goal) {
  if (is_visible(maze, start, goal)) return {std::pair{start, Link(goal, heuristic(start, goal))}};

  const auto pad0 = 0.1f;
  const auto pad1 = 0.11f;
  const auto pad2 = 0.12f;
  const auto pad3 = 0.13f;
  const Vec2 offsets[]{Vec2(pad0, pad0), Vec2(-pad1, pad1), Vec2(-pad2, -pad2), Vec2(pad3, -pad3)};

  auto graphs = std::vector<VisibilityGraph>(n_threads());
  parallel_for(maze.width(), maze.height(), [&](int x, int y) {
    if (is_wall_edge(maze, x, y)) return;

    for (int yy = y; yy <= maze.height(); yy++) {
      for (int xx = yy == y ? x : 0; xx <= maze.width(); xx++) {
        if (is_wall_edge(maze, xx, yy)) continue;

        for (Vec2 o : offsets) {
          for (Vec2 oo : offsets) {
            auto p0 = Vec2(x, y) + o;
            auto p1 = Vec2(xx, yy) + oo;
            if (p0 == p1) continue;
            // if (x == xx && y == yy && o.x != oo.x && o.y != oo.y) continue;

            if (is_visible(maze, p0, p1)) {
              graphs[thread_idx].emplace(p0, Link(p1, heuristic(p0, p1)));
              graphs[thread_idx].emplace(p1, Link(p0, heuristic(p0, p1)));
            }
          }
        }
      }
    }
  });

  auto& graph = graphs[0];
  for (size_t i = 1; i < graphs.size(); i++) graph.merge(graphs[i]);

  for (int i = 0; i < 2; i++) {
    auto [x, y] = i == 0 ? start : goal;
    for (int yy = 0; yy <= maze.height(); yy++) {
      for (int xx = 0; xx <= maze.width(); xx++) {
        if (is_wall_edge(maze, xx, yy)) continue;

        for (Vec2 o : offsets) {
          auto p0 = Vec2(x, y);
          auto p1 = Vec2(xx, yy) + o;

          if (p0 == p1) continue;
          if (is_visible(maze, p0, p1)) {
            graph.emplace(p0, Link(p1, heuristic(p0, p1)));
            graph.emplace(p1, Link(p0, heuristic(p0, p1)));
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
  const auto n = 30;
  auto maze = Maze(n, n);

  const auto S = 30.0f;
  const auto screen_width = n * S;
  const auto screen_height = n * S;

  SetTraceLogLevel(TraceLogLevel::LOG_ERROR);
  InitWindow(screen_width, screen_height, "Test");
  // ToggleFullscreen();

  SetTargetFPS(60);

  Vec2 p0, p1;
  auto i = 0;

  auto path = vector<Vec2>();

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

      i %= 1;
      if (i == 0) {
        path.clear();
        p0 = {x, y};
      }
      i++;
    }

    if (i == 1) {
      auto [x, y] = Vector2Scale(GetMousePosition(), 1 / S);
      p1 = {x, y};
      // auto timer = Timer();
      auto graph = build_visibility_graph(maze, p0, p1);
      // std::cout << timer.reset() << '\t';
      path = path_finding_astar(
          p0, p1,
          [&](Vec2 p, auto f) {
            for (auto [neighbor, link] : as_range(graph.equal_range(p))) f(link.x, link.w);
          },
          heuristic);
      // std::cout << timer() << '\n';
    }

    if (i > 0) DrawCircle(p0.x * S, p0.y * S, 5, BLUE);
    if (i > 1) DrawCircle(p1.x * S, p1.y * S, 5, RED);
    if (i == 2)
      DrawText(("Path length: " + std::to_string(path.size() - 1)).c_str(), 0, 0, 40, GREEN);
    if (i == 2 && path.size() == 0) DrawText("No path found", 0, 50, 50, GREEN);

    if (path.size()) {
      path.insert(path.begin(), path.front());
      path.push_back(path.back());
    }
    
    for (auto& p : path) p *= Vec2(S, S);
    DrawSplineCatmullRom((Vector2*)&path[0], path.size(), 3, PURPLE);

    EndDrawing();
  }

  CloseWindow();

  return 0;
}
