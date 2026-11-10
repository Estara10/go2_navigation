cmake_minimum_required(VERSION 3.16)
project(go2_vins_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

# Standard-library core/plan build. The ROS node is added only when rclcpp and
# nav_msgs are available; the core and selftest build without ROS.
set(GO2_NAV_PLAN_BUILD_DEMOS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../go2_nav_plan"
                 "${CMAKE_CURRENT_BINARY_DIR}/go2_nav_plan")

add_library(go2_vins_bridge_core STATIC
  bridge_core.cpp
  udp_bridge_server.cpp
)
target_include_directories(go2_vins_bridge_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(go2_vins_bridge_core PUBLIC go2_nav_plan Threads::Threads)
target_compile_options(go2_vins_bridge_core PRIVATE -Wall -Wextra)

add_executable(go2_vins_bridge_selftest selftest.cpp)
target_link_libraries(go2_vins_bridge_selftest
  PRIVATE go2_vins_bridge_core Threads::Threads)
target_compile_options(go2_vins_bridge_selftest PRIVATE -Wall -Wextra)

# Full synthetic chain: VINS samples -> BridgeCore -> UDP -> native motion core
# (with a fake Go2 backend). It does not link ROS or the SDK.
add_executable(go2_vins_bridge_integration_test integration_test.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../go2_native_motion/native_motion_core.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../go2_native_motion/vins_udp_client.cpp"
)
target_include_directories(go2_vins_bridge_integration_test
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../go2_native_motion")
target_link_libraries(go2_vins_bridge_integration_test
  PRIVATE go2_vins_bridge_core Threads::Threads)
target_compile_options(go2_vins_bridge_integration_test PRIVATE -Wall -Wextra)

find_package(rclcpp QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
if(rclcpp_FOUND AND nav_msgs_FOUND AND sensor_msgs_FOUND)
  add_executable(go2_vins_bridge main.cpp)
  target_link_libraries(go2_vins_bridge
    PRIVATE go2_vins_bridge_core rclcpp::rclcpp ${nav_msgs_TARGETS}
            ${sensor_msgs_TARGETS})
  target_compile_options(go2_vins_bridge PRIVATE -Wall -Wextra)
  message(STATUS "Building ROS 2 bridge node: go2_vins_bridge")
else()
  message(STATUS
    "rclcpp/nav_msgs/sensor_msgs not found; ROS bridge node not built; "
    "core and selftest are still available")
endif()
