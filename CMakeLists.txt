###############################################################################
# zed_ros2_wrapper_mlx — Top-level CMakeLists.txt
#
# This is a REPLACEMENT build system for the upstream zed-ros2-wrapper.
# It removes all proprietary dependencies and builds against zed-sdk-mlx
# (the open-source ZED Open Capture library ported to macOS/MLX).
#
# ============================================================================
# WHAT WAS REMOVED AND WHY
# ============================================================================
#
# 1. find_package(ZED REQUIRED)
#    - The proprietary Stereolabs ZED SDK is not available on macOS ARM64.
#    - Replaced by zed_open_capture from the zed-sdk-mlx project, which
#      provides USB video capture and sensor access without the SDK.
#
# 2. find_package(CUDA REQUIRED)
#    - CUDA is NVIDIA-only and does not exist on Apple Silicon.
#    - Depth computation will use OpenCV CPU stereo or Metal/MLX in the future.
#
# 3. ament_cmake / colcon build system
#    - ROS2 is not installed on this development machine.
#    - All ament_* calls are wrapped in if(ROS2_AVAILABLE) guards so the
#      project can be built standalone with plain cmake.
#    - When ROS2 is available, the original ament integration still works.
#
# 4. ISAAC ROS Nitros integration
#    - NVIDIA-specific, removed entirely.
#
# 5. Object detection, body tracking, GNSS fusion, cost traversability
#    - These features depend on the proprietary ZED SDK neural networks.
#    - Conditionally excluded via ZED_MLX_SKIP_PROPRIETARY_FEATURES=ON.
#
# ============================================================================
# DEPENDENCIES
# ============================================================================
#
#   REQUIRED:
#     - OpenCV        (brew install opencv)
#     - zed-sdk-mlx   (built locally, provides libzed_open_capture.dylib)
#
#   OPTIONAL:
#     - ROS2 Humble/Jazzy (enables full ROS2 node components)
#
###############################################################################

cmake_minimum_required(VERSION 3.16)
project(zed_ros2_wrapper_mlx LANGUAGES CXX)

# ---- C++ Standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Export compile_commands.json for IDEs ----
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- zed-open-capture module guards ----
# These macros are required by videocapture.hpp and sensorcapture.hpp to expose
# the class definitions. Without them, everything is #ifdef'd out.
add_definitions(-DVIDEO_MOD_AVAILABLE)
add_definitions(-DSENSORS_MOD_AVAILABLE)

# ---- hidapi (required by sensorcapture.hpp) ----
find_path(HIDAPI_INCLUDE_DIR hidapi.h
    PATHS /opt/homebrew/include/hidapi /usr/include/hidapi /usr/local/include/hidapi
)
if(HIDAPI_INCLUDE_DIR)
    include_directories(${HIDAPI_INCLUDE_DIR})
    message(STATUS "hidapi include: ${HIDAPI_INCLUDE_DIR}")
endif()

# ---- Build type default ----
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

###############################################################################
# Options
###############################################################################

option(ZED_MLX_SKIP_PROPRIETARY_FEATURES
    "Skip features that require the proprietary ZED SDK (obj det, body trk, GNSS, traversability)"
    ON)

# Path hints for zed-sdk-mlx (override with -DZED_SDK_MLX_DIR=...)
set(ZED_SDK_MLX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../zed-sdk-mlx"
    CACHE PATH "Root of the zed-sdk-mlx source tree")

set(ZED_SDK_MLX_BUILD_DIR "${ZED_SDK_MLX_DIR}/build/zed-open-capture"
    CACHE PATH "Build directory containing libzed_open_capture")

###############################################################################
# Platform checks
###############################################################################

if(APPLE)
    message(STATUS "Building on macOS — CUDA/ZED SDK disabled, using zed_open_capture")
    add_definitions(-DZED_MLX_MACOS)
endif()

###############################################################################
# Find OpenCV (REQUIRED)
###############################################################################

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION} found: ${OpenCV_INCLUDE_DIRS}")

###############################################################################
# Find zed_open_capture library
###############################################################################

# Headers
find_path(ZED_OC_INCLUDE_DIR
    NAMES videocapture.hpp
    HINTS
        ${ZED_SDK_MLX_DIR}/include
        /usr/local/include/zed-open-capture
    DOC "zed_open_capture include directory"
)

# Library
find_library(ZED_OC_LIBRARY
    NAMES zed_open_capture
    HINTS
        ${ZED_SDK_MLX_BUILD_DIR}
        ${ZED_SDK_MLX_DIR}/build
        /usr/local/lib
    DOC "zed_open_capture shared library"
)

if(ZED_OC_INCLUDE_DIR AND ZED_OC_LIBRARY)
    message(STATUS "zed_open_capture found:")
    message(STATUS "  Include: ${ZED_OC_INCLUDE_DIR}")
    message(STATUS "  Library: ${ZED_OC_LIBRARY}")

    # Create an imported target for clean downstream usage
    if(NOT TARGET zed_open_capture::zed_open_capture)
        add_library(zed_open_capture::zed_open_capture SHARED IMPORTED)
        set_target_properties(zed_open_capture::zed_open_capture PROPERTIES
            IMPORTED_LOCATION "${ZED_OC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ZED_OC_INCLUDE_DIR}"
        )
    endif()
else()
    message(WARNING
        "zed_open_capture NOT found. Build will create stub targets only.\n"
        "  Looked for headers in: ${ZED_SDK_MLX_DIR}/include\n"
        "  Looked for library in: ${ZED_SDK_MLX_BUILD_DIR}\n"
        "  Set -DZED_SDK_MLX_DIR=<path> to fix."
    )
endif()

###############################################################################
# Optional: Check for ROS2 / ament
###############################################################################

set(ROS2_AVAILABLE FALSE)

# Try to find ament_cmake — if present, ROS2 is available
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    set(ROS2_AVAILABLE TRUE)
    message(STATUS "ROS2 ament_cmake found — full ROS2 integration enabled")
else()
    message(STATUS "ROS2 not found — building standalone (no ROS2 nodes)")
endif()

###############################################################################
# Subdirectories
###############################################################################

# The main component library
add_subdirectory(zed_components)

# Debug executable (only if ROS2 is available, since it uses rclcpp)
if(ROS2_AVAILABLE)
    add_subdirectory(zed_debug)
endif()

###############################################################################
# Install
###############################################################################

# Install the top-level CMake config so downstream can find_package() us
install(DIRECTORY zed_wrapper/config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY zed_wrapper/launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

message(STATUS "")
message(STATUS "=== zed_ros2_wrapper_mlx configuration ===")
message(STATUS "  Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV:           ${OpenCV_VERSION}")
message(STATUS "  zed_open_capture: ${ZED_OC_LIBRARY}")
message(STATUS "  ROS2 available:   ${ROS2_AVAILABLE}")
message(STATUS "  Skip proprietary: ${ZED_MLX_SKIP_PROPRIETARY_FEATURES}")
message(STATUS "  Platform:         ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "==========================================")
message(STATUS "")
