cmake_minimum_required(VERSION 3.16)
project(go2_nav_plan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Offline module: standard library only. It links go2_nav_core for the frozen
# BodyState/VelocityCommand interface and the camera-to-base_link math.
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../go2_nav_core"
                 "${CMAKE_CURRENT_BINARY_DIR}/go2_nav_core")

add_library(go2_nav_plan STATIC
  path.cpp
  pure_pursuit.cpp
  vins_replay.cpp
  extrinsic_config.cpp
)
target_include_directories(go2_nav_plan
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(go2_nav_plan PUBLIC go2_nav_core)
target_compile_options(go2_nav_plan PRIVATE -Wall -Wextra)

add_executable(go2_nav_plan_selftest selftest.cpp)
target_link_libraries(go2_nav_plan_selftest PRIVATE go2_nav_plan)
target_compile_options(go2_nav_plan_selftest PRIVATE -Wall -Wextra)

option(GO2_NAV_PLAN_BUILD_DEMOS
  "Build go2_nav_plan demo and VINS replay executables" ON)
if(GO2_NAV_PLAN_BUILD_DEMOS)
  add_executable(go2_nav_plan_demo demo_main.cpp)
  target_link_libraries(go2_nav_plan_demo PRIVATE go2_nav_plan)
  target_compile_options(go2_nav_plan_demo PRIVATE -Wall -Wextra)

  add_executable(go2_vins_replay replay_main.cpp)
  target_link_libraries(go2_vins_replay PRIVATE go2_nav_plan)
  target_compile_options(go2_vins_replay PRIVATE -Wall -Wextra)
endif()
