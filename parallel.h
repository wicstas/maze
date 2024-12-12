#pragma once
#include <atomic>
#include <shared_mutex>
#include <thread>
#include <vector>

inline int n_threads() { return std::thread::hardware_concurrency(); }

thread_local inline int thread_idx;

template <typename F, typename... Args>
void parallel_for_impl(int n_items, F&& f) {
  if (n_threads() <= 1) {
    for (int i = 0; i < n_items; i++) f(i);
    return;
  }

  auto threads = std::vector<std::thread>(n_threads());
  auto global_index = std::atomic<int>(0);
  auto tid = 0;
  auto mutex = std::shared_mutex();
  auto eptr = std::exception_ptr(nullptr);
  auto batch_count = std::max<int>(n_threads(), n_items / 64);
  auto batch_size = std::max<int>(n_items / batch_count, 1);

  for (auto& thread : threads)
    thread = std::thread([&, tid = tid++]() {
      thread_idx = tid;
      try {
        while (true) {
          auto index = (global_index += batch_size) - batch_size;
          auto end_index = std::min(index + batch_size, n_items);
          for (int i = index; i < end_index; i++) f(i);
          if (end_index >= n_items) break;
        }
      } catch (...) {
        mutex.lock();
        eptr = std::current_exception();
        mutex.unlock();
      }
    });

  for (auto& thread : threads) thread.join();
  if (eptr) std::rethrow_exception(eptr);
}

template <typename F, typename... Args>
void parallel_for(int64_t size, F&& f) {
  parallel_for_impl(size, [&f](int idx) { f(idx); });
}
template <typename F, typename... Args>
void parallel_for(int x, int y, F&& f) {
  parallel_for_impl(x * y, [&f, x](int idx) { f(idx % x, idx / x); });
}