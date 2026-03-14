# ZED ROS2 Wrapper MLX -- Port TODO

Work this file top to bottom. The upstream wrapper depends on the **proprietary ZED SDK** (`sl/Camera.hpp`) and **CUDA**. Neither exists on macOS. Our strategy: replace the ZED SDK layer with `zed-sdk-mlx` (open capture + MLX depth) while preserving the ROS2 node/topic/service API shape so downstream ROS2 code keeps working.

## Architecture Overview

```
UPSTREAM (Linux):
  ZED SDK (proprietary, CUDA) --> zed_components (ROS2 nodes) --> topics

PORT (macOS/MLX):
  zed-sdk-mlx (open capture + MLX) --> zed_components_mlx (ROS2 nodes) --> same topics
```

## Scope Decisions

### In scope (port these)
- [x] Clone upstream and set remotes (origin=RobotFlow, upstream=stereolabs)
- [ ] Video capture node (stereo images via AVFoundation through zed-sdk-mlx)
- [ ] Sensor node (IMU, magnetometer, barometer, temperature via hidapi)
- [ ] Depth/disparity node (MLX stereo pipeline)
- [ ] Point cloud publisher (from MLX depth)
- [ ] Camera info / calibration publisher
- [ ] Stereo rectification
- [ ] Timestamp-based video/sensor sync
- [ ] TF publishing (camera frame transforms)
- [ ] Launch files for macOS
- [ ] Config YAML for ZED 2i on macOS
- [ ] Dynamic parameters for tuning (pyramid_factor, window_size, etc.)

### Out of scope (not ported, capability-gated)
- [ ] Object detection (requires ZED SDK neural engine + CUDA)
- [ ] Body tracking (requires ZED SDK neural engine + CUDA)
- [ ] Visual odometry / positional tracking (requires ZED SDK SLAM)
- [ ] Spatial mapping (requires ZED SDK)
- [ ] GNSS fusion (requires ZED SDK Fusion API)
- [ ] SVO recording/playback (ZED SDK format)
- [ ] Neural depth modes (NEURAL_LIGHT, NEURAL, NEURAL_PLUS)
- [ ] ZED X / ZED X One / GMSL2 camera support
- [ ] Camera controls beyond auto/locked toggle (macOS kernel limitation)
- [ ] Area memory / loop closure
- [ ] Streaming server

### Deferred
- [ ] Docker image for macOS (after base port works)
- [ ] CI workflow for macOS
- [ ] Virtual stereo (paired cameras)

---

## Build Environment

ROS2 Jazzy is available inside the existing Docker container `rlf-gazebo-sim`:
- **Container**: `rlf-gazebo-sim` (image: `rlf-gazebo:latest`, 8.37GB)
- **ROS2**: Jazzy (full desktop) on ARM64/Ubuntu Noble
- **Gazebo**: Sim 8.10.0
- **Key packages**: cv_bridge, image_transport, sensor_msgs, tf2, colcon
- **Already running topics**: /camera/rgb, /depth/image, /imu, /odom, /lidar/scan
- **Build command**: `docker exec rlf-gazebo-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build"`

The host machine (macOS Apple Silicon) does NOT have ROS2 installed. All ROS2 builds and tests happen inside Docker. The zed-sdk-mlx capture library runs natively on macOS and bridges to the container via shared memory or network transport.

## Phase 0: Repo Setup and Build Scaffold

- [x] Clone upstream into zed-ros2-wrapper-mlx
- [x] Set origin to RobotFlow-Labs/zed-ros2-wrapper-mlx
- [x] Set upstream to stereolabs/zed-ros2-wrapper
- [x] Add .claude/settings.json for permissions
- [x] Add TODO.md (this file)
- [x] Verify ROS2 is available (Docker container rlf-gazebo-sim has Jazzy)
- [ ] Create porting_plan.md with architecture decisions
- [ ] Create a top-level Makefile for macOS build workflow
- [ ] Mount wrapper repo into Docker container for builds
- [ ] Verify colcon build of unmodified upstream fails (expected — no ZED SDK in container)

Validation:
- `git remote -v` shows both remotes
- TODO.md exists
- `docker exec rlf-gazebo-sim ros2 pkg list` works

## Phase 1: Abstraction Layer -- Replace ZED SDK with zed-sdk-mlx

This is the core of the port. The upstream wrapper imports `sl/Camera.hpp` everywhere. We need a compatibility shim.

### 1.1 Create the SDK abstraction header
- [ ] Create `zed_components/src/include/sl_oc_bridge.hpp`
- [ ] Define a `SlOcCamera` class that wraps `sl_oc::video::VideoCapture` and `sl_oc::sensors::SensorCapture`
- [ ] Map the minimum required `sl::Camera` methods:
  - `open()` / `close()`
  - `grab()` -- returns stereo frame
  - `retrieveImage()` -- left, right, left_gray, right_gray
  - `retrieveMeasure()` -- depth, disparity, point_cloud (delegate to MLX via Python or C++ bridge)
  - `getCameraInformation()` -- resolution, serial, calibration, FPS
  - `getSensorsData()` -- IMU, magnetometer, barometer, temperature
  - `getCameraSettings()` / `setCameraSettings()` -- capability-gated
- [ ] Map `sl::InitParameters` to our open capture params
- [ ] Map `sl::RuntimeParameters` to our MLX config
- [ ] Map `sl::Resolution`, `sl::Mat`, `sl::Timestamp` types

### 1.2 Create the MLX depth bridge
- [ ] Design the C++ <-> Python bridge for MLX depth
  - Option A: Subprocess with shared memory (like zed_capture_stream + live_mlx_disparity)
  - Option B: pybind11 embedded Python interpreter in the ROS2 node
  - Option C: Pure C++ depth using OpenCV SGBM as fallback, MLX as optional
- [ ] Implement the chosen bridge
- [ ] Return depth as float32 array compatible with `sensor_msgs/Image`
- [ ] Return point cloud compatible with `sensor_msgs/PointCloud2`

### 1.3 Map sensor data types
- [ ] Map `sl_oc::sensors::data::Imu` to `sensor_msgs/Imu`
- [ ] Map `sl_oc::sensors::data::Magnetometer` to `sensor_msgs/MagneticField`
- [ ] Map `sl_oc::sensors::data::Environment` to barometer message
- [ ] Map `sl_oc::sensors::data::Temperature` to `sensor_msgs/Temperature`
- [ ] Handle coordinate system differences (RAW sensor frame -> camera frame)

Validation:
- `sl_oc_bridge.hpp` compiles standalone
- Unit test for type mappings

## Phase 2: CMake Build System

### 2.1 Replace ZED SDK / CUDA dependencies
- [ ] Fork `zed_components/CMakeLists.txt`
- [ ] Remove `find_package(ZED REQUIRED)` and `find_package(CUDA REQUIRED)`
- [ ] Add `find_package(zed_open_capture REQUIRED)` or direct path to zed-sdk-mlx build
- [ ] Link against `libzed_open_capture.dylib` instead of `${ZED_LIBRARIES}`
- [ ] Remove CUDA include dirs and libraries
- [ ] Add MLX/Python dependencies if using embedded interpreter
- [ ] Add OpenCV dependency (already used for rectification)

### 2.2 Handle platform-specific compilation
- [ ] Add `if(APPLE)` guards for macOS-specific code
- [ ] Skip object detection, body tracking, GNSS components on macOS
- [ ] Conditionally compile only the stereo camera component (not ZedCameraOne)

### 2.3 Package.xml updates
- [ ] Update `zed_components/package.xml` -- remove ZED SDK dependency, add zed-sdk-mlx
- [ ] Update `zed_wrapper/package.xml` if needed
- [ ] Keep `zed_ros2` metapackage working

Validation:
- `colcon build` succeeds (or our Makefile equivalent)
- No references to sl/Camera.hpp or CUDA remain in active build paths

## Phase 3: Stereo Camera Node (Core Port)

### 3.1 Strip the main component to essentials
- [ ] Fork `zed_camera_component_main.cpp` (9,944 lines) as `zed_camera_mlx_component.cpp`
- [ ] Remove all `sl::Camera` / `sl::InitParameters` / `sl::RuntimeParameters` usage
- [ ] Remove object detection thread and callbacks (~1,273 lines)
- [ ] Remove body tracking thread and callbacks (~463 lines)
- [ ] Remove GNSS fusion code
- [ ] Remove SVO recording/playback
- [ ] Remove positional tracking / visual odometry / area memory
- [ ] Remove spatial mapping
- [ ] Remove streaming server
- [ ] Keep: parameter declarations, timer callbacks, diagnostic publisher, TF publisher

### 3.2 Wire video capture
- [ ] Initialize `sl_oc::video::VideoCapture` in node startup
- [ ] Grab stereo frames in the grab thread
- [ ] Split side-by-side YUYV into left/right
- [ ] Convert to RGB for publishing
- [ ] Publish `left/image_rect_color`, `right/image_rect_color`
- [ ] Publish `left/camera_info`, `right/camera_info` from calibration

### 3.3 Wire sensor capture
- [ ] Initialize `sl_oc::sensors::SensorCapture` in node startup
- [ ] Poll IMU at configured rate
- [ ] Publish `imu/data` (sensor_msgs/Imu)
- [ ] Publish `mag/magnetic_field` (sensor_msgs/MagneticField) if available
- [ ] Publish temperature
- [ ] Handle timestamp sync (use our timestamp-fallback)

### 3.4 Wire depth pipeline
- [ ] Start MLX depth subprocess (shared memory bridge)
- [ ] Feed stereo frames to MLX pipeline
- [ ] Receive disparity/depth results
- [ ] Publish `depth/depth_registered` (sensor_msgs/Image, 32FC1)
- [ ] Publish `point_cloud/cloud_registered` (sensor_msgs/PointCloud2)
- [ ] Publish `disparity/disparity_image`

### 3.5 Wire camera controls (capability-gated)
- [ ] Expose `video.auto_exposure_gain` parameter (maps to AVFoundation mode toggle)
- [ ] Expose `video.auto_whitebalance` parameter (maps to AVFoundation mode toggle)
- [ ] Gate all numeric controls (exposure, gain, white_balance_temperature) with warnings
- [ ] Dynamic parameter callback for live tuning of MLX params

Validation:
- Node starts without crash
- `ros2 topic list` shows expected topics
- `ros2 topic echo` shows live data on image, IMU, depth topics

## Phase 4: Video/Depth Publishing (Port of video_depth component)

- [ ] Fork `zed_camera_component_video_depth.cpp` (3,265 lines)
- [ ] Replace `sl::Mat` image handling with OpenCV Mat from our capture
- [ ] Replace `sl::Camera::retrieveImage()` with direct frame buffer access
- [ ] Replace `sl::Camera::retrieveMeasure(DEPTH)` with MLX depth result
- [ ] Keep the ROS2 image publishing logic (it's standard)
- [ ] Keep point cloud conversion logic but adapt from `sl::Mat` to our format
- [ ] Handle resolution scaling (pub_resolution vs grab_resolution)
- [ ] Handle frame rate decimation (pub_frame_rate < grab_frame_rate)

Validation:
- Image topics contain valid data viewable in RViz2
- Depth topic contains valid float32 depth in meters
- Point cloud viewable in RViz2

## Phase 5: Configuration and Launch

### 5.1 Create macOS config
- [ ] Create `zed_wrapper/config/zed2i_macos.yaml` with MLX-specific params:
  - `general.grab_resolution: 'HD720'`
  - `depth.depth_mode: 'MLX'` (new mode)
  - `depth.mlx_pyramid_factor: 2`
  - `depth.mlx_window_size: 5`
  - `depth.mlx_refinement_radius: 0`
  - `video.auto_exposure_gain: true` (only supported mode)
- [ ] Create `zed_wrapper/config/common_stereo_macos.yaml`

### 5.2 Create macOS launch file
- [ ] Create `zed_wrapper/launch/zed_camera_macos.launch.py`
- [ ] Load macOS-specific configs
- [ ] Skip unsupported components (object detection, body tracking)
- [ ] Set default camera_model to 'zed2i'

### 5.3 URDF / TF
- [ ] Keep existing URDF models (they're camera-model-specific, not OS-specific)
- [ ] Verify TF tree publishes correctly: `map -> odom -> base_link -> camera_link`

Validation:
- `ros2 launch zed_wrapper zed_camera_macos.launch.py` starts cleanly
- All parameters load from YAML
- TF tree visible in RViz2

## Phase 6: Tools and Utilities

- [ ] Port `sl_tools.cpp` -- remove ZED SDK dependencies, keep ROS2 helpers
- [ ] Port `sl_types.cpp` -- replace sl:: types with our bridge types
- [ ] Keep `sl_win_avg.cpp` as-is (pure math, no SDK dependency)
- [ ] Keep `json.hpp` as-is (header-only, no changes needed)
- [ ] Remove or stub `gnss_replay.cpp` (requires ZED Fusion API)
- [ ] Remove `cost_traversability.cpp` (requires ZED SDK depth)

Validation:
- All tool files compile
- No dangling sl:: references

## Phase 7: Unsupported Feature Gating

- [ ] Create `zed_components/src/include/macos_unsupported.hpp` with clear error macros
- [ ] Gate object detection node registration on macOS
- [ ] Gate body tracking node registration on macOS
- [ ] Gate positional tracking services on macOS
- [ ] Gate SVO services on macOS
- [ ] Gate streaming services on macOS
- [ ] Gate GNSS services on macOS
- [ ] All gated features log a clear warning at startup: "Feature X requires ZED SDK + CUDA, not available on macOS"

Validation:
- Calling gated services returns descriptive error
- No silent failures

## Phase 8: Documentation and Packaging

- [ ] Update README.md with macOS port status
- [ ] Add docs/macos.md with setup instructions
- [ ] Document which topics/services are available vs gated
- [ ] Add port status table matching zed-sdk-mlx format
- [ ] Update CONTRIBUTING.md with macOS porting guidelines
- [ ] First push to origin (RobotFlow-Labs/zed-ros2-wrapper-mlx)

Validation:
- Fresh clone + build works
- README accurately describes what works

## Phase 9: Testing and Validation

- [ ] Verify all published topics with connected ZED 2i
- [ ] Verify image quality matches zed-sdk-mlx standalone
- [ ] Verify IMU data rate and quality
- [ ] Verify depth/point cloud quality
- [ ] Verify TF tree correctness
- [ ] Verify dynamic parameter changes work
- [ ] Profile node CPU/memory usage
- [ ] Test graceful shutdown

---

## Key Files Reference

| Upstream file | Lines | Port action |
|--------------|-------|-------------|
| `zed_camera_component_main.cpp` | 9,944 | Heavy refactor -- strip to essentials |
| `zed_camera_component_video_depth.cpp` | 3,265 | Moderate refactor -- replace sl::Mat with OpenCV |
| `zed_camera_component_objdet.cpp` | 1,273 | Skip -- gate as unsupported |
| `zed_camera_component_bodytrk.cpp` | 463 | Skip -- gate as unsupported |
| `zed_camera_one_component_*.cpp` | 4,311 | Skip -- mono cameras not in scope |
| `sl_tools.cpp` | 731 | Light refactor -- remove sl:: helpers |
| `sl_types.cpp` | 48 | Rewrite -- replace sl:: type mappings |
| `cost_traversability.cpp` | 194 | Skip -- requires ZED SDK depth |
| `gnss_replay.cpp` | 328 | Skip -- requires ZED Fusion API |
| **CMakeLists.txt** | ~220 | Rewrite -- remove ZED/CUDA, add zed-sdk-mlx |
| **Config YAMLs** | ~30 files | Add macOS variants, keep Linux ones |
| **Launch files** | ~3 files | Add macOS launch variant |

## Non-Portable Features (from zed-sdk-mlx investigation)

These are confirmed not portable and must be gated:

| Feature | Root cause |
|---------|-----------|
| Manual exposure/gain/WB | macOS kernel blocks USB control transfers |
| Neural depth modes | Requires ZED SDK + CUDA |
| Object detection | Requires ZED SDK neural engine |
| Body tracking | Requires ZED SDK neural engine |
| Visual odometry/SLAM | Requires ZED SDK |
| Spatial mapping | Requires ZED SDK |
| SVO recording | Proprietary ZED format |
| GNSS fusion | Requires ZED Fusion API |
| Streaming server | Requires ZED SDK |
| Hardware sync | macOS AVFoundation limitation |

## Build Commands (planned)

```bash
# Option A: If ROS2 is installed
cd ~/ros2_ws/src
ln -s /path/to/zed-ros2-wrapper-mlx .
cd ~/ros2_ws
colcon build --packages-select zed_components zed_wrapper zed_ros2

# Option B: Standalone (no ROS2 install)
cd zed-ros2-wrapper-mlx
make build  # custom Makefile using cmake directly

# Test
ros2 launch zed_wrapper zed_camera_macos.launch.py camera_model:=zed2i
```
