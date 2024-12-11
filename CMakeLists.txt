cmake_minimum_required(VERSION 3.14)

project(game)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(game main.cpp)

target_link_libraries(game PRIVATE raylib GL m pthread dl rt X11)
target_include_directories(game PRIVATE ./)