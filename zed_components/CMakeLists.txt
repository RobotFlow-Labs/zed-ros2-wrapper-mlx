###############################################################################
# zed_components/CMakeLists.txt
#
# Rewritten for zed-sdk-mlx: replaces proprietary ZED SDK + CUDA with
# zed_open_capture (open-source USB capture) + OpenCV.
#
# Upstream compatibility notes:
#   - Original file structure and variable names preserved where possible
#     to make future upstream merges easier.
#   - Sections removed/changed are marked with "# MLX-CHANGE:" comments.
#   - Object detection, body tracking, GNSS replay, and cost traversability
#     sources are conditionally excluded when ZED_MLX_SKIP_PROPRIETARY_FEATURES=ON.
#
###############################################################################

# When built as a subdirectory from the top-level CMakeLists.txt, the project()
# call here is still useful for IDE integration and ament compatibility.
cmake_minimum_required(VERSION 3.16)
project(zed_components LANGUAGES CXX)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# C++ standard (inherit from parent, or set if standalone)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Default to C99 for any C sources
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    add_compile_options(-Wno-deprecated-declarations)
endif()
if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    add_compile_options(-Wno-deprecated-declarations)
endif()
if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS " * Debug Mode")
endif()

###############################################################################
# MLX-CHANGE: Removed proprietary dependencies
#
# REMOVED:
#   find_package(ZED REQUIRED)            — proprietary, not available on macOS
#   find_package(CUDA REQUIRED)           — NVIDIA-only
#   exec_program(uname ...) for aarch64   — Jetson-specific CUDA workaround
#   link_directories(${ZED_LIBRARY_DIR})
#   link_directories(${CUDA_LIBRARY_DIRS})
#   ${ZED_LIBRARIES}, ${CUDA_LIBRARIES}
#   ISAAC ROS Nitros integration          — NVIDIA-specific
#
# REPLACED WITH:
#   zed_open_capture (from parent scope or found here)
#   OpenCV
###############################################################################

#############################################
# Dependencies — zed_open_capture + OpenCV

# OpenCV (should be found by parent, but find again for standalone builds)
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif()

# zed_open_capture — use imported target from parent if available
if(NOT TARGET zed_open_capture::zed_open_capture)
    # Standalone build: try to find it ourselves
    set(ZED_SDK_MLX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../zed-sdk-mlx"
        CACHE PATH "Root of zed-sdk-mlx source tree")
    set(ZED_SDK_MLX_BUILD_DIR "${ZED_SDK_MLX_DIR}/build/zed-open-capture"
        CACHE PATH "Build directory for zed_open_capture")

    find_path(ZED_OC_INCLUDE_DIR
        NAMES videocapture.hpp
        HINTS ${ZED_SDK_MLX_DIR}/include /usr/local/include/zed-open-capture
    )
    find_library(ZED_OC_LIBRARY
        NAMES zed_open_capture
        HINTS ${ZED_SDK_MLX_BUILD_DIR} ${ZED_SDK_MLX_DIR}/build /usr/local/lib
    )

    if(ZED_OC_INCLUDE_DIR AND ZED_OC_LIBRARY)
        add_library(zed_open_capture::zed_open_capture SHARED IMPORTED)
        set_target_properties(zed_open_capture::zed_open_capture PROPERTIES
            IMPORTED_LOCATION "${ZED_OC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ZED_OC_INCLUDE_DIR}"
        )
    else()
        message(WARNING "zed_open_capture not found — targets will not link properly")
    endif()
endif()

# MLX-CHANGE: Define the replacement for ${ZED_LIBS}
# Instead of ${ZED_LIBRARIES} + ${CUDA_LIBRARIES}, we use zed_open_capture + OpenCV
set(ZED_MLX_LIBS "")
if(TARGET zed_open_capture::zed_open_capture)
    list(APPEND ZED_MLX_LIBS zed_open_capture::zed_open_capture)
endif()
list(APPEND ZED_MLX_LIBS ${OpenCV_LIBS})

# MLX-CHANGE: Include directories replacing ${ZED_INCLUDE_DIRS} and ${CUDA_INCLUDE_DIRS}
set(ZED_MLX_INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
)
if(ZED_OC_INCLUDE_DIR)
    list(APPEND ZED_MLX_INCLUDE_DIRS ${ZED_OC_INCLUDE_DIR})
endif()

###############################################################################
# Optional ROS2 dependencies
# When ROS2 is not present, we skip ament calls and ROS2 message types.
###############################################################################

set(ROS2_AVAILABLE FALSE)
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    set(ROS2_AVAILABLE TRUE)
endif()

if(ROS2_AVAILABLE)
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
    endif()

    set(DEPENDENCIES
        rclcpp
        rclcpp_components
        image_transport
        builtin_interfaces
        std_msgs
        rosgraph_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        nav_msgs
        nmea_msgs
        geographic_msgs
        sensor_msgs
        stereo_msgs
        zed_msgs
        std_srvs
        diagnostic_msgs
        diagnostic_updater
        visualization_msgs
        shape_msgs
        robot_localization
        backward_ros
    )

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    find_package(rcutils REQUIRED)
    find_package(zed_msgs QUIET)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_components REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosgraph_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(nmea_msgs REQUIRED)
    find_package(geographic_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(stereo_msgs REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(diagnostic_updater REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(shape_msgs REQUIRED)
    find_package(robot_localization REQUIRED)
    find_package(backward_ros REQUIRED)

    # MLX-CHANGE: ISAAC ROS Nitros removed entirely (NVIDIA-specific)

    # Check Point Cloud Transport availability
    find_package(point_cloud_transport QUIET)
    if(point_cloud_transport_FOUND)
        list(APPEND DEPENDENCIES point_cloud_transport)
        message(STATUS " * Point Cloud Transport is available")
        add_definitions(-DFOUND_POINT_CLOUD_TRANSPORT)
    endif()

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
endif() # ROS2_AVAILABLE

###############################################################################
# Add all files in subdirectories for IDE indexing
file(GLOB_RECURSE all_files ${CMAKE_CURRENT_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# SOURCES
###############################################################################

set(SL_TOOLS_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_tools.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_win_avg.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_logging.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/json.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_types.hpp
)

set(SL_TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_win_avg.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_types.cpp
)

# MLX-CHANGE: GNSS replay depends on proprietary ZED SDK GNSS fusion.
# Include only if not skipping proprietary features.
if(NOT ZED_MLX_SKIP_PROPRIETARY_FEATURES)
    list(APPEND SL_TOOLS_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/gnss_replay.hpp
    )
    list(APPEND SL_TOOLS_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/gnss_replay.cpp
    )
endif()

set(ZED_CAMERA_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/sl_version.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/zed_camera_component.hpp
)

# MLX-CHANGE: Conditionally exclude object detection and body tracking sources.
# These depend on ZED SDK neural network APIs not available in open-source capture.
set(ZED_CAMERA_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component_video_depth.cpp
)

if(NOT ZED_MLX_SKIP_PROPRIETARY_FEATURES)
    list(APPEND ZED_CAMERA_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component_objdet.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component_bodytrk.cpp
    )
    # Cost traversability also depends on ZED SDK depth AI
    list(APPEND ZED_CAMERA_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/cost_traversability.hpp
    )
    list(APPEND ZED_CAMERA_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/cost_traversability.cpp
    )
endif()

set(ZED_CAMERA_ONE_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/sl_version.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/include/zed_camera_one_component.hpp
)

set(ZED_CAMERA_ONE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/src/zed_camera_one_component_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/src/zed_camera_one_component_sensors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/src/zed_camera_one_component_video.cpp
)

###############################################################################
# Definitions for MLX build
###############################################################################

# Tell source code we are building without the proprietary ZED SDK
add_definitions(-DZED_MLX_BUILD)

if(ZED_MLX_SKIP_PROPRIETARY_FEATURES)
    add_definitions(-DZED_MLX_NO_OBJDET)
    add_definitions(-DZED_MLX_NO_BODYTRK)
    add_definitions(-DZED_MLX_NO_GNSS)
    add_definitions(-DZED_MLX_NO_TRAVERSABILITY)
endif()

###############################################################################
# Build targets
###############################################################################

if(ROS2_AVAILABLE)
    # ---- Full ROS2 component build ----

    # create ament index resource
    set(node_plugins "")

    # ZED Camera Component
    add_library(zed_camera_component SHARED
        ${SL_TOOLS_INC}
        ${SL_TOOLS_SRC}
        ${ZED_CAMERA_INC}
        ${ZED_CAMERA_SRC}
    )
    target_include_directories(zed_camera_component PUBLIC
        # MLX-CHANGE: replaced ${CUDA_INCLUDE_DIRS} and ${ZED_INCLUDE_DIRS}
        ${ZED_MLX_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    )
    target_compile_definitions(zed_camera_component
        PRIVATE "COMPOSITION_BUILDING_DLL"
    )
    target_link_libraries(zed_camera_component
        # MLX-CHANGE: replaced ${ZED_LIBS} (was ZED SDK + CUDA)
        ${ZED_MLX_LIBS}
    )
    ament_target_dependencies(zed_camera_component
        ${DEPENDENCIES}
    )

    rclcpp_components_register_nodes(zed_camera_component "stereolabs::ZedCamera")
    set(node_plugins "${node_plugins}stereolabs::ZedCamera;$<TARGET_FILE:zed_camera_component>\n")

    # ZED Camera One Component
    add_library(zed_camera_one_component SHARED
        ${SL_TOOLS_INC}
        ${SL_TOOLS_SRC}
        ${ZED_CAMERA_ONE_INC}
        ${ZED_CAMERA_ONE_SRC}
    )
    target_include_directories(zed_camera_one_component PUBLIC
        # MLX-CHANGE: replaced ${CUDA_INCLUDE_DIRS} and ${ZED_INCLUDE_DIRS}
        ${ZED_MLX_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    )
    target_compile_definitions(zed_camera_one_component
        PRIVATE "COMPOSITION_BUILDING_DLL"
    )
    target_link_libraries(zed_camera_one_component
        # MLX-CHANGE: replaced ${ZED_LIBS}
        ${ZED_MLX_LIBS}
    )
    ament_target_dependencies(zed_camera_one_component
        ${DEPENDENCIES}
    )

    rclcpp_components_register_nodes(zed_camera_one_component "stereolabs::ZedCameraOne")
    set(node_plugins "${node_plugins}stereolabs::ZedCameraOne;$<TARGET_FILE:zed_camera_one_component>\n")

    # Install components
    install(TARGETS zed_camera_component zed_camera_one_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    # Install header files
    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera_one/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
        DESTINATION include/${PROJECT_NAME}/
    )

    ament_export_include_directories(include)
    ament_export_libraries(
        zed_camera_component
        zed_camera_one_component
    )
    ament_export_dependencies(${DEPENDENCIES})
    ament_package()

else()
    # ---- Standalone build (no ROS2) ----
    # Build a bridge library with the tools and utility code that does not
    # depend on rclcpp. This is useful for testing the camera capture layer.

    message(STATUS "Building standalone sl_oc_bridge library (no ROS2)")

    add_library(sl_oc_bridge SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_win_avg.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_oc_bridge.cpp
    )
    # MLX-CHANGE: sl_types.cpp excluded from standalone build because
    # sl_types.hpp includes sl/Camera.hpp (proprietary ZED SDK).
    # sl_oc_bridge.cpp provides the macOS-compatible replacement types.
    target_include_directories(sl_oc_bridge PUBLIC
        ${ZED_MLX_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    )
    target_link_libraries(sl_oc_bridge
        ${ZED_MLX_LIBS}
    )

    # macOS-specific: set rpath for finding dylibs
    if(APPLE)
        set_target_properties(sl_oc_bridge PROPERTIES
            INSTALL_RPATH "@loader_path;@loader_path/../lib"
            BUILD_WITH_INSTALL_RPATH TRUE
        )
    endif()

    install(TARGETS sl_oc_bridge
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
        DESTINATION include/${PROJECT_NAME}/
    )

endif() # ROS2_AVAILABLE
