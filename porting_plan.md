# ZED ROS 2 Wrapper -- macOS / Apple Silicon / MLX Porting Plan

> This document describes the architecture, rationale, and status of the macOS port of the
> [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) for Apple Silicon
> with MLX-accelerated stereo depth.

---

## Architecture Overview

```
 +====================================================================+
 |                     macOS / Apple Silicon Host                      |
 +====================================================================+
 |                                                                     |
 |  +-------------------------------+   +---------------------------+  |
 |  |    ZED 2i Camera (USB 3.0)    |   |   Apple Silicon GPU (MLX) |  |
 |  |  (AVFoundation video capture) |   |   (Stereo depth compute)  |  |
 |  +-------------------------------+   +---------------------------+  |
 |        |                                       |                    |
 |        v                                       v                    |
 |  +------------------------------------------------------------+    |
 |  |               zed-sdk-mlx (capture + depth layer)          |    |
 |  |                                                            |    |
 |  |  +------------------+  +-------------------+  +---------+  |    |
 |  |  | AVFoundation     |  | MLX Stereo Match  |  | hidapi  |  |    |
 |  |  | Video Backend    |  | (~8ms @ 720p)     |  | Sensors |  |    |
 |  |  +------------------+  +-------------------+  +---------+  |    |
 |  |                                                            |    |
 |  +------------------------------------------------------------+    |
 |        |                    |                    |                   |
 |        v                    v                    v                   |
 |  +------------------------------------------------------------+    |
 |  |          zed-ros2-wrapper-mlx (this repository)            |    |
 |  |                                                            |    |
 |  |  +-------------------+  +------------------+               |    |
 |  |  | zed_components    |  | zed_wrapper      |               |    |
 |  |  | (C++ ROS node)    |  | (config + launch)|               |    |
 |  |  +-------------------+  +------------------+               |    |
 |  |        |                        |                          |    |
 |  |        v                        v                          |    |
 |  |  +-------------------------------------------------+      |    |
 |  |  |          ROS 2 Topic Interface                   |      |    |
 |  |  |                                                  |      |    |
 |  |  |  /zed/zed_node/rgb/image_rect_color     [OK]     |      |    |
 |  |  |  /zed/zed_node/depth/depth_registered   [OK]     |      |    |
 |  |  |  /zed/zed_node/point_cloud/cloud_registered [OK] |      |    |
 |  |  |  /zed/zed_node/imu/data                 [OK]     |      |    |
 |  |  |  /zed/zed_node/left/image_rect_color    [OK]     |      |    |
 |  |  |  /zed/zed_node/right/image_rect_color   [OK]     |      |    |
 |  |  |  /zed/zed_node/disparity/disparity_image [OK]    |      |    |
 |  |  |  /zed/zed_node/odom                     [GATED]  |      |    |
 |  |  |  /zed/zed_node/pose                     [GATED]  |      |    |
 |  |  |  /zed/zed_node/obj_det/*                [OFF]    |      |    |
 |  |  |  /zed/zed_node/body_trk/*              [OFF]    |      |    |
 |  |  |  /zed/zed_node/mapping/*                [OFF]    |      |    |
 |  |  +-------------------------------------------------+      |    |
 |  +------------------------------------------------------------+    |
 +=====================================================================+

 Legend:
   [OK]    = Available on macOS with MLX depth
   [GATED] = Disabled by default, future work (requires visual odometry port)
   [OFF]   = Not portable to macOS (requires CUDA / ZED SDK internals)
```

---

## What Was Replaced and Why

### Depth Engine: ZED SDK Neural -> MLX Stereo Matching

| Aspect | Upstream (Linux) | macOS Port |
|--------|-----------------|------------|
| Depth compute | ZED SDK NEURAL_LIGHT (CUDA) | MLX GPU stereo matching |
| Hardware | NVIDIA GPU (CUDA cores) | Apple Silicon (unified GPU) |
| Latency | ~10-15ms (varies by GPU) | ~8ms at 720p |
| Resolution | Up to HD2K | Optimal at HD720 |
| Depth modes | NEURAL, NEURAL_LIGHT, NEURAL_PLUS | MLX (single mode, tunable) |

**Why:** The ZED SDK depth modes are closed-source CUDA binaries. They cannot run on macOS.
The MLX stereo matching pipeline from zed-sdk-mlx provides equivalent depth quality at 720p
using Apple Silicon's Metal/MLX compute.

### Video Capture: V4L2 -> AVFoundation

| Aspect | Upstream (Linux) | macOS Port |
|--------|-----------------|------------|
| Capture API | V4L2 ioctls | AVFoundation |
| Format | YUV 4:2:2 | YUYV/yuvs |
| Camera controls | Full (via vendor XU) | Auto mode only |

**Why:** V4L2 is Linux-only. AVFoundation is the macOS native camera API. The ZED SDK's
internal capture uses V4L2 on Linux; we replace it with the zed-sdk-mlx AVFoundation backend.

### Sensor Capture: No Change

Sensor capture uses hidapi on both platforms. No replacement needed.

### Configuration Files: macOS Variants

| File | Purpose |
|------|---------|
| `common_stereo_macos.yaml` | Disables CUDA-dependent features, adds MLX params |
| `zed2i_macos.yaml` | HD720 @ 15fps (optimal for MLX pipeline) |
| `zed_camera_macos.launch.py` | Simplified launch (no OD/BT/sim/stream) |

---

## Non-Portable Features: Complete List

These features from the upstream ZED ROS 2 Wrapper cannot be ported to macOS.
Root causes fall into three categories:

### Category 1: Requires CUDA / NVIDIA GPU

| Feature | Upstream Module | Root Cause |
|---------|----------------|------------|
| Neural depth (NEURAL, NEURAL_LIGHT, NEURAL_PLUS) | `depth` | CUDA-only closed-source inference |
| Object detection (all models) | `object_detection` | CUDA-only DNN inference engine |
| Custom YOLO object detection | `object_detection` | CUDA-only ONNX runtime |
| Body tracking (all models) | `body_tracking` | CUDA-only pose estimation |
| NITROS integration | `advanced` | NVIDIA Isaac ROS-specific |

### Category 2: Requires ZED SDK Internals

| Feature | Upstream Module | Root Cause |
|---------|----------------|------------|
| Positional tracking (GEN_1/2/3) | `pos_tracking` | Closed-source visual-inertial SLAM |
| Spatial mapping (fused point cloud) | `mapping` | Closed-source volumetric fusion |
| GNSS fusion | `gnss_fusion` | Depends on positional tracking |
| Area memory (loop closure) | `pos_tracking` | ZED SDK relocalization engine |
| Floor alignment | `pos_tracking` | ZED SDK floor detection |
| Depth stabilization | `depth` | Requires positional tracking |
| Streaming server (H.264/H.265) | `stream_server` | ZED SDK hardware encoder |
| SVO recording | service | ZED SDK proprietary format |
| Simulation mode | `simulation` | ZED SDK simulation bridge |
| Region of interest (automatic) | `region_of_interest` | ZED SDK ROI generator |

### Category 3: macOS Kernel Limitations (Camera Controls)

The macOS kernel UVC driver has no public API for forwarding USB class-specific control
requests. We built the full vendor Extension Unit transport layer and confirmed every
request STALLs. This is a confirmed macOS platform limitation.

| Feature | Linux Path | macOS Status |
|---------|-----------|-------------|
| Manual exposure (duration/ISO) | V4L2 + vendor XU register I/O | Not portable |
| Manual gain control | Vendor XU sensor register write | Not portable |
| Manual white balance (temp/gains) | Vendor XU + V4L2 | Not portable |
| Exposure compensation/bias | V4L2 | Not portable |
| ROI-based exposure metering | V4L2 | Not portable |
| ISP register read/write | Vendor XU (unit 0x04, selector 0x02) | Not portable |
| LED control | Vendor XU | Not portable |
| Hardware video/sensor sync gate | Low-level camera signaling | Not portable |

**What we ship instead:**
- Auto/locked mode toggle for exposure and white balance
- Timestamp-based sync fallback (~2.5ms median offset on ZED 2i)

---

## ROS 2 Topic Comparison: Upstream vs macOS

### Available Topics (full functionality)

| Topic Pattern | Type | Notes |
|--------------|------|-------|
| `~/rgb/image_rect_color` | sensor_msgs/Image | Rectified color image |
| `~/left/image_rect_color` | sensor_msgs/Image | Left rectified |
| `~/right/image_rect_color` | sensor_msgs/Image | Right rectified |
| `~/left/image_rect_gray` | sensor_msgs/Image | Left rectified grayscale |
| `~/right/image_rect_gray` | sensor_msgs/Image | Right rectified grayscale |
| `~/stereo/image_rect_color` | sensor_msgs/Image | Side-by-side stereo |
| `~/left_raw/*` | sensor_msgs/Image | Unrectified images |
| `~/right_raw/*` | sensor_msgs/Image | Unrectified images |
| `~/depth/depth_registered` | sensor_msgs/Image | MLX depth map |
| `~/depth/camera_info` | sensor_msgs/CameraInfo | Depth camera info |
| `~/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | MLX point cloud |
| `~/disparity/disparity_image` | stereo_msgs/DisparityImage | MLX disparity |
| `~/imu/data` | sensor_msgs/Imu | IMU data (hidapi) |
| `~/imu/data_raw` | sensor_msgs/Imu | Raw IMU |
| `~/imu/mag` | sensor_msgs/MagneticField | Magnetometer |
| `~/atm_press` | sensor_msgs/FluidPressure | Barometer |
| `~/temperature/*` | sensor_msgs/Temperature | Temperature sensors |
| `~/left/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `~/right/camera_info` | sensor_msgs/CameraInfo | Camera calibration |

### Gated Topics (disabled by default, future work)

| Topic Pattern | Type | Blocker |
|--------------|------|---------|
| `~/odom` | nav_msgs/Odometry | Needs visual odometry port |
| `~/pose` | geometry_msgs/PoseStamped | Needs visual odometry port |
| `~/pose_with_covariance` | geometry_msgs/PoseWithCovarianceStamped | Needs visual odometry port |
| `~/path_odom` | nav_msgs/Path | Needs visual odometry port |
| `~/path_pose` | nav_msgs/Path | Needs visual odometry port |

### Unavailable Topics (not portable)

| Topic Pattern | Type | Root Cause |
|--------------|------|------------|
| `~/obj_det/*` | zed_msgs/ObjectsStamped | Requires CUDA OD |
| `~/body_trk/*` | zed_msgs/SkeletonsStamped | Requires CUDA BT |
| `~/mapping/fused_cloud` | sensor_msgs/PointCloud2 | Requires ZED SDK mapping |
| `~/plane/*` | zed_msgs/PlaneStamped | Requires ZED SDK mapping |

---

## Build Instructions for macOS

### Prerequisites

1. **macOS 13+ on Apple Silicon** (M1, M2, M3, M4 or later)
2. **Homebrew**: `brew install cmake hidapi opencv`
3. **Python 3.12+** and **uv** (for MLX depth pipeline)
4. **ROS 2** (Humble or Jazzy) -- can be installed via:
   - [robostack-humble](https://robostack.github.io/) (conda-based, recommended for macOS)
   - Source build of ROS 2 on macOS

### Build Steps

```bash
# 1. Set up ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/RobotFlow-Labs/zed-ros2-wrapper-mlx.git

# 3. Clone zed-sdk-mlx (the capture + MLX depth layer)
git clone https://github.com/RobotFlow-Labs/zed-sdk-mlx.git

# 4. Install ROS 2 dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install \
  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug

# 6. Source the workspace
source install/local_setup.bash
```

### Launch

```bash
# Default: ZED 2i at 720p with MLX depth
ros2 launch zed_wrapper zed_camera_macos.launch.py

# Specify camera model
ros2 launch zed_wrapper zed_camera_macos.launch.py camera_model:=zed2

# Override parameters
ros2 launch zed_wrapper zed_camera_macos.launch.py \
  ros_params_override_path:=/path/to/my_overrides.yaml
```

---

## MLX Depth Tuning

The MLX stereo matching parameters can be tuned in `common_stereo_macos.yaml` under the
`mlx:` section:

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `pyramid_factor` | 2 | 1-4 | Coarse-to-fine downscale. Higher = faster, less detail |
| `window_size` | 5 | 3-15 | Matching window. Higher = smoother, slower |
| `refinement_radius` | 0 | 0-3 | Sub-pixel refinement. 0 = disabled for speed |
| `max_disparity` | 96 | 48-256 | Max search range. Higher = closer min depth |
| `processing_scale` | 0.75 | 0.25-1.0 | Input downscale. Lower = faster, less detail |
| `consistency_mode` | none | none/lr | Left-right check. `lr` = higher quality, 2x compute |

### Performance Presets

| Preset | pyramid | window | refine | disp | scale | FPS (est.) |
|--------|---------|--------|--------|------|-------|------------|
| **Live (default)** | 2 | 5 | 0 | 96 | 0.75 | ~14.5 |
| **Quality** | 1 | 7 | 1 | 128 | 1.0 | ~6 |
| **Fast** | 3 | 3 | 0 | 64 | 0.5 | ~25 |

---

## Future Work

1. **Visual Odometry**: Port ORB-SLAM3 or similar to provide pose/odom topics
2. **CoreML Object Detection**: Replace CUDA OD with CoreML/Vision framework inference
3. **Metal Compute Shaders**: Potential alternative to MLX for stereo matching
4. **Multi-camera**: Validate multi-ZED setups on macOS
5. **SVO Playback**: Implement SVO file reader without ZED SDK dependency

---

## Related Repositories

- [zed-sdk-mlx](https://github.com/RobotFlow-Labs/zed-sdk-mlx) -- macOS capture + MLX depth layer
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) -- upstream Linux wrapper
- [zed-open-capture](https://github.com/stereolabs/zed-open-capture) -- upstream open capture library
