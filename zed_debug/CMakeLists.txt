###############################################################################
# zed_debug/CMakeLists.txt
#
# Rewritten for zed-sdk-mlx: removed ZED SDK + CUDA dependencies.
# This package is only built when ROS2 is available (it depends on rclcpp).
#
# MLX-CHANGE: Removed find_package(ZED REQUIRED), find_package(CUDA REQUIRED),
#             ${ZED_LIBRARIES}, ${CUDA_LIBRARIES}, ${ZED_INCLUDE_DIRS},
#             ${CUDA_INCLUDE_DIRS}, link_directories for ZED/CUDA.
###############################################################################

cmake_minimum_required(VERSION 3.16)
project(zed_debug LANGUAGES CXX)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    add_compile_options(-Wno-deprecated-declarations)
endif()
if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS " * Debug Mode")
endif()

###############################################################################
# MLX-CHANGE: Removed proprietary dependencies (ZED SDK, CUDA)
# Replaced with zed_open_capture from parent scope
###############################################################################

# OpenCV
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif()

# zed_open_capture — reuse from parent or find
if(NOT TARGET zed_open_capture::zed_open_capture)
    set(ZED_SDK_MLX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../zed-sdk-mlx"
        CACHE PATH "Root of zed-sdk-mlx source tree")
    set(ZED_SDK_MLX_BUILD_DIR "${ZED_SDK_MLX_DIR}/build/zed-open-capture"
        CACHE PATH "Build directory for zed_open_capture")

    find_path(ZED_OC_INCLUDE_DIR NAMES videocapture.hpp
        HINTS ${ZED_SDK_MLX_DIR}/include /usr/local/include/zed-open-capture)
    find_library(ZED_OC_LIBRARY NAMES zed_open_capture
        HINTS ${ZED_SDK_MLX_BUILD_DIR} /usr/local/lib)

    if(ZED_OC_INCLUDE_DIR AND ZED_OC_LIBRARY)
        add_library(zed_open_capture::zed_open_capture SHARED IMPORTED)
        set_target_properties(zed_open_capture::zed_open_capture PROPERTIES
            IMPORTED_LOCATION "${ZED_OC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ZED_OC_INCLUDE_DIR}")
    endif()
endif()

# MLX replacement for ${ZED_LIBS}
set(ZED_MLX_LIBS "")
if(TARGET zed_open_capture::zed_open_capture)
    list(APPEND ZED_MLX_LIBS zed_open_capture::zed_open_capture)
endif()
list(APPEND ZED_MLX_LIBS ${OpenCV_LIBS})

set(ZED_MLX_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
if(ZED_OC_INCLUDE_DIR)
    list(APPEND ZED_MLX_INCLUDE_DIRS ${ZED_OC_INCLUDE_DIR})
endif()

###############################################################################
# ROS2 dependencies (this package requires ROS2)
###############################################################################

################################################
# Check the ROS2 version
set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;bouncy;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;kilted;rolling")
    foreach(distro ${ROS2_DISTROS})
        if(NOT ROS2_FOUND)
            find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
            if(RCLCPP_H)
                set(FOUND_ROS2_DISTRO ${distro})
                set(ROS2_FOUND TRUE)
            endif()
        endif()
    endforeach()
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        add_definitions(-DFOUND_FOXY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        add_definitions(-DFOUND_HUMBLE)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "iron")
        add_definitions(-DFOUND_IRON)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
        add_definitions(-DFOUND_JAZZY)
    else()
        message("*** WARNING *** Unsupported ROS2 ${FOUND_ROS2_DISTRO}.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    zed_components
    backward_ros
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(zed_components REQUIRED)

# Debugging with backward-cpp
find_package(backward_ros REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# IDE indexing
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# SOURCES
set(ZED_DEBUG_PROC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_debug_proc.cpp
)

###############################################################################
# Build

# ZED DEBUG PROC executable
add_executable(zed_debug_proc
    ${ZED_DEBUG_PROC}
)
target_include_directories(zed_debug_proc PUBLIC
    # MLX-CHANGE: replaced ${CUDA_INCLUDE_DIRS} and ${ZED_INCLUDE_DIRS}
    ${ZED_MLX_INCLUDE_DIRS}
)
target_link_libraries(zed_debug_proc
    # MLX-CHANGE: replaced ${CUDA_LIBRARIES}
    ${ZED_MLX_LIBS}
)
ament_target_dependencies(zed_debug_proc
    ${DEPENDENCIES}
)

# Install executable
install(TARGETS zed_debug_proc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install LAUNCH and CONFIG files
install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
