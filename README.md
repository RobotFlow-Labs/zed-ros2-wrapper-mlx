<h1 align="center">
  ZED ROS 2 Wrapper -- macOS / Apple Silicon / MLX
</h1>

<p align="center">
  macOS port of the <a href="https://github.com/stereolabs/zed-ros2-wrapper">Stereolabs ZED ROS 2 wrapper</a>.<br>
  Replaces ZED SDK + CUDA with <a href="https://github.com/RobotFlow-Labs/zed-sdk-mlx">zed-sdk-mlx</a> (open capture + MLX depth on Apple Silicon).
</p>

<p align="center">
  <a href="#what-works">What Works</a> |
  <a href="#what-does-not-work">What Does Not Work</a> |
  <a href="#quick-start">Quick Start</a> |
  <a href="#architecture">Architecture</a> |
  <a href="#port-status">Port Status</a> |
  <a href="#configuration">Configuration</a> |
  <a href="#performance">Performance</a> |
  <a href="#upstream-documentation">Upstream Docs</a>
</p>

---

## What Works

Everything in the core stereo camera pipeline runs natively on macOS / Apple Silicon:

- **Stereo video capture** -- AVFoundation backend, validated on ZED 2i at 2560x720 YUYV
- **MLX depth and disparity** -- Apple Silicon GPU-accelerated stereo matching (~8ms per frame at 720p)
- **Point cloud generation** -- from MLX depth, published as PointCloud2
- **IMU / magnetometer / barometer / temperature** -- hidapi, same path as Linux
- **Stereo rectification** -- OpenCV remap, same path as Linux
- **Calibration loading** -- auto-loads from `~/zed/settings/SN<serial>.conf`
- **Video/sensor sync** -- timestamp-based fallback (median offset ~2.5ms)
- **TF broadcasting** -- camera frame transforms published to `/tf`
- **Dynamic parameters** -- live tuning of MLX stereo params (pyramid_factor, window_size, etc.)
- **ROS 2 topics** -- standard `sensor_msgs/Image`, `sensor_msgs/PointCloud2`, `sensor_msgs/Imu`, `sensor_msgs/CameraInfo`

## What Does Not Work

These features require the proprietary ZED SDK and/or CUDA. They are **not available on macOS** and are explicitly capability-gated with clear error messages at runtime.

### Features blocked by ZED SDK dependency

| Feature | Why it cannot be ported |
|---------|------------------------|
| Object detection | Requires ZED SDK neural inference engine + CUDA |
| Body tracking / skeleton | Requires ZED SDK neural inference engine + CUDA |
| Positional tracking / Visual SLAM | Requires ZED SDK's proprietary visual-inertial odometry |
| Spatial mapping | Requires ZED SDK's internal mesh engine |
| Neural depth modes (NEURAL, NEURAL_PLUS) | Requires ZED SDK + CUDA for neural network inference |
| SVO recording / playback | Proprietary ZED recording format, no public spec |
| Streaming server (H.264/H.265) | Requires ZED SDK's hardware encoder integration |
| GNSS fusion | Depends on positional tracking (ZED SDK) |
| Simulation mode (Isaac Sim) | Requires ZED SDK simulation bridge |
| NVIDIA NITROS integration | NVIDIA-specific hardware acceleration |
| ZED X / ZED X One / GMSL2 cameras | Require proprietary GMSL2 capture driver |
| Area memory / loop closure | Requires ZED SDK SLAM backend |

### Features blocked by macOS platform limitations

| Feature | Why it cannot be ported |
|---------|------------------------|
| Manual exposure (duration/ISO) | macOS kernel UVC driver blocks all sideband USB control transfers. No `UVCIOC_CTRL_QUERY` equivalent exists on macOS. |
| Manual gain control | Same root cause -- kernel blocks vendor Extension Unit (XU) requests |
| Manual white balance (temperature/gains) | `deviceWhiteBalanceGains` / `setWhiteBalanceModeLocked(with:)` are iOS-only APIs |
| Exposure compensation / bias | `setExposureTargetBias` is iOS-only |
| LED control | Uses same blocked vendor XU path as exposure/gain |
| Hardware video/sensor sync | macOS AVFoundation does not expose hardware sync signals |

**What we provide instead:**
- Auto/locked mode toggle for exposure and white balance (the only controls macOS allows)
- Timestamp-based sync fallback (validated at ~2.5ms median offset)
- All blocked features return descriptive errors, never silent failures

For the full technical investigation, see [`porting_plan.md`](./porting_plan.md) and the [zed-sdk-mlx supervision doc](https://github.com/RobotFlow-Labs/zed-sdk-mlx/blob/main/claude_supervision.md).

---

## Quick Start

### Prerequisites

- macOS 13+ on Apple Silicon (M1 or later)
- Homebrew: `brew install cmake hidapi opencv`
- Python 3.12+ and [uv](https://github.com/astral-sh/uv)
- ROS 2 Jazzy (via Docker -- see below)

### Option A: Build standalone (no ROS 2 required)

This builds the SDK bridge library that wraps zed-sdk-mlx for the ROS 2 node:

```bash
git clone https://github.com/RobotFlow-Labs/zed-ros2-wrapper-mlx.git
cd zed-ros2-wrapper-mlx

# Build zed-sdk-mlx first (the capture + MLX library)
cd ../
git clone https://github.com/RobotFlow-Labs/zed-sdk-mlx.git
cd zed-sdk-mlx
make sync && make configure-zed-open-capture && make build-zed-open-capture
cd ../zed-ros2-wrapper-mlx

# Configure and build the bridge
make configure
make build
# Output: build/zed_components/libsl_oc_bridge.dylib
```

### Option B: Build with ROS 2 in Docker

A ROS 2 Jazzy + Gazebo Sim 8.10 Docker container is available:

```bash
# Mount the workspace into the container
docker run -it --rm \
  -v $(pwd)/..:/workspace \
  rlf-gazebo:latest bash

# Inside the container:
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install \
  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  --packages-select zed_components zed_wrapper zed_ros2
source install/local_setup.bash

# Launch
ros2 launch zed_wrapper zed_camera_macos.launch.py camera_model:=zed2i
```

---

## Architecture

```
UPSTREAM (Linux + ZED SDK + CUDA):
  ZED SDK (proprietary) --> zed_components (ROS 2 nodes) --> topics

THIS FORK (macOS + MLX):
  zed-sdk-mlx (open capture + MLX) --> sl_oc_bridge --> zed_camera_mlx (ROS 2 node) --> same topics
       |                                    |
       |-- AVFoundation video capture       |-- Maps sl:: types to open-capture types
       |-- hidapi sensor capture            |-- Capability-gates all proprietary features
       |-- MLX stereo depth                 |-- Compiles standalone (no ROS 2 needed)
       |-- OpenCV rectification             |
```

Key design decisions:
- **`sl_oc_bridge`** is a pure C++17 compatibility shim that wraps `VideoCapture` + `SensorCapture` from zed-sdk-mlx into an API shape the ROS 2 node can consume
- **No CUDA, no ZED SDK** -- the entire build chain works without either
- **ROS 2 is optional** -- without it, only `libsl_oc_bridge.dylib` is built (useful for testing the capture layer)
- **Upstream merge-friendly** -- all changes marked with `MLX-CHANGE:` comments, original file structure preserved

---

## Port Status

### ROS 2 Topics

| Topic | Linux | macOS | Notes |
|-------|-------|-------|-------|
| `~/left/image_rect_color` | Available | Available | AVFoundation + OpenCV rectify |
| `~/right/image_rect_color` | Available | Available | |
| `~/rgb/image_rect_color` | Available | Available | |
| `~/depth/depth_registered` | Available | Available | MLX stereo (not neural) |
| `~/point_cloud/cloud_registered` | Available | Available | From MLX depth |
| `~/imu/data` | Available | Available | hidapi, same path |
| `~/imu/data_raw` | Available | Available | |
| `~/mag/magnetic_field` | Available | Available | |
| `~/temp/temp_imu` | Available | Available | |
| `~/left/camera_info` | Available | Available | |
| `~/disparity/disparity_image` | Available | Available | |
| `~/odom` | Available | Not available | Requires ZED SDK SLAM |
| `~/pose` | Available | Not available | Requires ZED SDK SLAM |
| `~/obj_det/objects` | Available | Not available | Requires CUDA inference |
| `~/body_trk/skeletons` | Available | Not available | Requires CUDA inference |
| `/diagnostics` | Available | Available | |

### Components

| Component | Lines (upstream) | macOS Status |
|-----------|-----------------|-------------|
| `zed_camera_component_main.cpp` | 9,944 | Stripped to ~500 lines skeleton |
| `zed_camera_component_video_depth.cpp` | 3,265 | Ported (sl::Mat -> cv::Mat) |
| `zed_camera_component_objdet.cpp` | 1,273 | Skipped (requires CUDA) |
| `zed_camera_component_bodytrk.cpp` | 463 | Skipped (requires CUDA) |
| `zed_camera_one_component_*.cpp` | 4,311 | Skipped (mono cameras not in scope) |
| `sl_tools.cpp` | 731 | Kept (sl:: refs removed) |
| `cost_traversability.cpp` | 194 | Skipped (requires ZED SDK depth) |
| `gnss_replay.cpp` | 328 | Skipped (requires ZED Fusion API) |

### Build System

| Item | Upstream | This Fork |
|------|----------|-----------|
| `find_package(ZED)` | Required | Removed |
| `find_package(CUDA)` | Required | Removed |
| `${ZED_LIBRARIES}` | Linked | Replaced with `libzed_open_capture.dylib` |
| `${CUDA_LIBRARIES}` | Linked | Removed |
| ROS 2 | Required | Optional (standalone bridge builds without it) |
| NITROS | Optional | Removed |

---

## Configuration

### macOS-specific config files

| File | Purpose |
|------|---------|
| [`common_stereo_macos.yaml`](zed_wrapper/config/common_stereo_macos.yaml) | Base params: MLX depth, disabled CUDA features, auto-only camera controls |
| [`zed2i_macos.yaml`](zed_wrapper/config/zed2i_macos.yaml) | ZED 2i at HD720 @ 15fps (optimal for MLX throughput) |
| [`zed_camera_macos.launch.py`](zed_wrapper/launch/zed_camera_macos.launch.py) | Simplified launch: no OD/BT/sim/streaming |

### MLX tuning parameters

These are exposed as ROS 2 dynamic parameters for live adjustment:

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `mlx.pyramid_factor` | 2 | 1, 2, 4 | Biggest performance knob (4x compute reduction at factor 2) |
| `mlx.max_disparity` | 96 | 16-256 | Depth range vs compute cost |
| `mlx.window_size` | 5 | 1-15 (odd) | Quality vs smoothness |
| `mlx.refinement_radius` | 0 | 0-4 | Edge quality (O(1) guided filter, no perf penalty) |
| `mlx.processing_scale` | 0.75 | 0.25-1.0 | Global quality/speed knob |
| `mlx.consistency_mode` | none | none, left_right | Occlusion handling |

---

## Performance

On ZED 2i at 1280x720, Apple Silicon:

| Metric | Value |
|--------|-------|
| MLX stereo compute | ~8 ms per frame |
| Live pipeline (with cv2 display) | ~69 ms (~14.5 FPS) |
| Configured grab rate | 15 FPS |
| Offline eval (full resolution) | ~1054 ms |

The live bottleneck is Python/OpenCV display overhead, not MLX compute.

---

## Validated Test Results (2026-03-14)

All tests run with a real ZED 2i (serial 38892829) connected to Apple Silicon Mac.

### Hardware test -- sl_oc_bridge with real camera

```
Camera opened: Serial 38892829, 1280x720 @ 30 FPS
Frame 0: 1280x720 ch=4 (BGRA)
Frame 1: 1280x720 ch=4
Frame 2: 1280x720 ch=4
Frame 3: 1280x720 ch=4
Frame 4: 1280x720 ch=4
IMU accel: -9.86 0.23 -0.03 m/s^2
IMU gyro:  -0.18 -0.09 0.27 rad/s
Mag:       2.75 -0.44 -3.69 uT
Temp:      36.26 C
PASS
```

### Build verification

| Test | Result |
|------|--------|
| macOS standalone build (`libsl_oc_bridge.dylib`) | PASS |
| Docker colcon build (`zed_components`) | PASS -- ROS 2 Jazzy, ARM64 |
| Docker colcon build (`zed_wrapper`) | PASS |
| Docker colcon build (`zed_ros2` metapackage) | PASS |
| ROS 2 node launch (Docker) | PASS -- component loaded, topics advertised |
| Bridge test with real ZED 2i | PASS -- 5/5 frames, IMU, mag, temp |
| MLX depth pipeline (zed-sdk-mlx) | PASS -- 14ms compute at 720p |
| zed-sdk-mlx unit tests | 14/14 PASS |
| zed-sdk-mlx fresh-clone test | PASS |

### ROS 2 node launch output (Docker)

```
[zed.zed_node]:   ZED Camera MLX Component (macOS)
[zed.zed_node]:  * Camera name: zed
[zed.zed_node]:  * Grab rate: 15 Hz
[zed.zed_node]:  * Resolution: 1280x720
[zed.zed_node]:  * Min depth: 0.3m  Max depth: 10m
[zed.zed_node]:  * Publish IMU: TRUE at 200 Hz

Advertised topics:
  /zed/zed_node/rgb/color/rect/image
  /zed/zed_node/depth/depth_registered
  /zed/zed_node/point_cloud/cloud_registered
  /zed/zed_node/imu/data
  /zed/zed_node/imu/mag
  /zed/zed_node/temperature/imu
  /zed/zed_node/temperature/left
  /zed/zed_node/temperature/right
  /zed/zed_node/atm_press

Gated features (logged at startup):
  Object detection: NOT SUPPORTED
  Body tracking: NOT SUPPORTED
  Spatial mapping: NOT SUPPORTED
  SVO recording: NOT SUPPORTED
  Streaming server: NOT SUPPORTED
  GNSS fusion: NOT SUPPORTED
  Positional tracking: NOT SUPPORTED
```

---

## Project Status

| Phase | Status |
|-------|--------|
| 0. Repo setup | Done |
| 1. SDK abstraction layer | Done -- `sl_oc_bridge` compiles, links, and works with real camera |
| 2. CMake rewrite | Done -- no ZED SDK, no CUDA, optional ROS 2 |
| 3. Camera node wiring | Done -- video, depth, sensors, TF all wired |
| 4. Video/depth publishing | Done -- sl::Mat replaced with cv::Mat |
| 5. Config and launch | Done -- macOS YAML + launch file |
| 6. Tools and utilities | Done -- bridge + depth worker |
| 7. Feature gating | Done -- 24 features gated via `macos_unsupported.hpp` |
| 8. Documentation | Done -- README, porting_plan.md, TODO.md |
| 9. Hardware validation | Done -- ZED 2i tested on Apple Silicon |
| 9. Testing and validation | Pending -- requires connected ZED 2i + ROS 2 |

---

## Related Repositories

| Repository | Purpose |
|------------|---------|
| [zed-sdk-mlx](https://github.com/RobotFlow-Labs/zed-sdk-mlx) | macOS capture library + MLX depth (the foundation) |
| [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | Upstream Linux wrapper (reference) |
| [zed-open-capture](https://github.com/stereolabs/zed-open-capture) | Upstream open-source capture library (reference) |

---

## Upstream Documentation

Everything below is from the original upstream README for the Linux/CUDA version.

<details>
<summary>Click to expand upstream documentation</summary>

---

This package enables the use of ZED cameras with ROS 2, providing access to a variety of data types, including:

- Color and grayscale images, both rectified and unrectified
- Depth data
- Colored 3D point clouds
- Position and mapping, with optional GNSS data fusion
- Sensor data
- Detected objects
- Human skeleton data
- And more...

[More information](https://www.stereolabs.com/docs/ros2)

### Installation (Linux)

#### Prerequisites

- Ubuntu 20.04 / 22.04 / 24.04
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v5.2
- [CUDA](https://developer.nvidia.com/cuda-downloads)
- ROS 2 Humble or Jazzy

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

### Starting the ZED node (Linux)

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

Replace `<camera_model>` with: `'zed'`, `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`, etc.

For full upstream documentation, see the [Stereolabs ROS 2 docs](https://www.stereolabs.com/docs/ros2).

</details>

## License

This library is licensed under the MIT License.
