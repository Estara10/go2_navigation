cmake_minimum_required(VERSION 3.16)
project(go2_nav_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Standard-library-only offline module. It must not link ROS, DDS, or the
# Unitree SDK.
add_library(go2_nav_core STATIC body_state.cpp)
target_include_directories(go2_nav_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(go2_nav_core PRIVATE -Wall -Wextra)

add_executable(go2_nav_core_selftest selftest.cpp)
target_link_libraries(go2_nav_core_selftest PRIVATE go2_nav_core)
target_compile_options(go2_nav_core_selftest PRIVATE -Wall -Wextra)
