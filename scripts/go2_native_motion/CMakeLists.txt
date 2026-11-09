cmake_minimum_required(VERSION 3.16)
project(go2_native_motion LANGUAGES CXX)

# UNITREE_SDK2_ROOT is also the conventional <PACKAGE>_ROOT name for
# find_package(unitree_sdk2); make newer CMake use it consistently.
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 离线核心与自检不依赖 Unitree SDK2，也不初始化 DDS。
add_library(go2_native_motion_core STATIC native_motion_core.cpp)
target_include_directories(go2_native_motion_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(go2_native_motion_core PRIVATE -Wall -Wextra)

# Path/Pure Pursuit is a standard-library-only offline module. Its own demos
# are built only in the standalone plan tree, not in the native tool tree.
set(GO2_NAV_PLAN_BUILD_DEMOS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../go2_nav_plan"
                 "${CMAKE_CURRENT_BINARY_DIR}/go2_nav_plan")
target_link_libraries(go2_native_motion_core PUBLIC go2_nav_plan)

add_executable(go2_native_motion_selftest selftest.cpp)
target_link_libraries(go2_native_motion_selftest PRIVATE go2_native_motion_core)
target_compile_options(go2_native_motion_selftest PRIVATE -Wall -Wextra)

# 实机入口需要外部 Unitree SDK2。AUTO：找到就构建；ON：找不到直接报错；
# OFF：即使找到也不构建。默认 AUTO 保留“本机有 SDK 即可构建”的便利，
# 但不会在缺少 SDK 时静默宣称实机入口已构建。
set(GO2_REAL_ROBOT "AUTO" CACHE STRING
  "Build SDK-linked real-robot executables: AUTO, ON, or OFF")
set_property(CACHE GO2_REAL_ROBOT PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${GO2_REAL_ROBOT}" _go2_real_robot_mode)
if(NOT _go2_real_robot_mode MATCHES "^(AUTO|ON|OFF)$")
  message(FATAL_ERROR
    "GO2_REAL_ROBOT must be AUTO, ON, or OFF (got '${GO2_REAL_ROBOT}')")
endif()

# SDK 路径不写死。优先使用 -DUNITREE_SDK2_ROOT=...，其次使用同名环境变量。
set(UNITREE_SDK2_ROOT "" CACHE PATH
  "Unitree SDK2 installation prefix containing lib/cmake/unitree_sdk2")
if(NOT UNITREE_SDK2_ROOT AND DEFINED ENV{UNITREE_SDK2_ROOT})
  set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
endif()
if(UNITREE_SDK2_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${UNITREE_SDK2_ROOT}")
endif()

set(_go2_build_real_robot OFF)
if(NOT _go2_real_robot_mode STREQUAL "OFF")
  find_package(unitree_sdk2 CONFIG QUIET)
  if(unitree_sdk2_FOUND)
    set(_go2_build_real_robot ON)
  elseif(_go2_real_robot_mode STREQUAL "ON")
    message(FATAL_ERROR
      "GO2_REAL_ROBOT=ON but unitree_sdk2 was not found. Set "
      "-DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2 (the prefix containing "
      "lib/cmake/unitree_sdk2) or export UNITREE_SDK2_ROOT. "
      "For an offline-only build use -DGO2_REAL_ROBOT=OFF.")
  else()
    message(STATUS
      "unitree_sdk2 not found: only the offline core, plan and selftest targets "
      "will be built. Real-robot executables were NOT built. Use "
      "-DUNITREE_SDK2_ROOT=... and -DGO2_REAL_ROBOT=ON to require them.")
  endif()
endif()

if(_go2_build_real_robot)
  add_executable(go2_native_motion main.cpp)
  target_link_libraries(go2_native_motion
    PRIVATE go2_native_motion_core unitree_sdk2)
  target_compile_options(go2_native_motion PRIVATE -Wall -Wextra)

  add_executable(go2_native_readonly_query readonly_query.cpp)
  target_link_libraries(go2_native_readonly_query PRIVATE unitree_sdk2)
  target_compile_options(go2_native_readonly_query PRIVATE -Wall -Wextra)

  message(STATUS
    "Building real-robot entries: go2_native_motion, go2_native_readonly_query")
else()
  message(STATUS
    "Real-robot entries not built; offline core, plan and selftest targets only.")
endif()
