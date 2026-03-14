// Copyright 2025 Stereolabs / AIFLOW LABS LIMITED
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ===========================================================================
// zed_camera_mlx_component.cpp
//
// macOS MLX port of the ZED Camera ROS2 node.
// This file contains the component skeleton with the same startup sequence,
// grab loop, and sensor loop as the upstream ZED wrapper, but with all
// sl::Camera / sl::Fusion / sl::Mat dependencies replaced by:
//   - open-capture bridge  (USB camera access)
//   - cv::Mat              (frame buffers)
//   - MLX depth bridge     (neural depth inference on Apple Silicon)
//
// Every section that requires actual implementation is marked with:
//   // TODO(mlx-port): <description>
// ===========================================================================

#include "zed_camera_mlx_component.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <chrono>
#include <filesystem>
#include <sstream>

#ifdef ZED_ROS2_AVAILABLE
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>
#endif

using namespace std::chrono_literals;

namespace stereolabs
{

// ===========================================================================
// Constructor / Destructor
// ===========================================================================

ZedCameraMlx::ZedCameraMlx(
#ifdef ZED_ROS2_AVAILABLE
  const rclcpp::NodeOptions & options
)
: Node("zed_node", options),
  mQos(10),
  mDiagUpdater(this)
#else
)
#endif
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "================================");
  RCLCPP_INFO(get_logger(), "  ZED Camera MLX Component (macOS)");
  RCLCPP_INFO(get_logger(), "================================");

  // One-shot initialisation timer (same pattern as upstream so
  // shared_from_this() is available).
  mInitTimer = create_wall_timer(
    50ms,
    std::bind(&ZedCameraMlx::initNode, this));
#else
  std::cout << "[ZedCameraMlx] Constructed (ROS2 not available -- skeleton mode)"
            << std::endl;
#endif
}

ZedCameraMlx::~ZedCameraMlx()
{
  deInitNode();
}

// ===========================================================================
// initNode  --  called once from the one-shot timer
// ===========================================================================

void ZedCameraMlx::initNode()
{
#ifdef ZED_ROS2_AVAILABLE
  mInitTimer->cancel();

  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "Starting node initialisation...");
#endif

  // 1. Parameters
  initParameters();

  // 2. TF frame names
  setTFCoordFrameNames();

  // 3. Services
  initServices();

  // 4. Publishers
  initPublishers();

  // 5. Open camera via open-capture bridge
  if (!openCamera()) {
#ifdef ZED_ROS2_AVAILABLE
    RCLCPP_FATAL(get_logger(), "Failed to open camera. Exiting.");
#else
    std::cerr << "[ZedCameraMlx] Failed to open camera." << std::endl;
#endif
    return;
  }

  // 6. Load factory calibration and build rectification maps
  if (!loadCalibration()) {
#ifdef ZED_ROS2_AVAILABLE
    RCLCPP_WARN(get_logger(),
      "Calibration load failed -- images will NOT be rectified.");
#endif
  }

  // 7. Fill CameraInfo messages from calibration
  fillCamInfo(false);  // rectified
  fillCamInfo(true);   // raw

  // 8. Kick off processing threads
  initThreads();

#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== ZED MLX node initialisation complete ===");
#endif
}

// ===========================================================================
// deInitNode
// ===========================================================================

void ZedCameraMlx::deInitNode()
{
  if (mThreadStop) {
    return;
  }
  mThreadStop = true;

  // Join threads in reverse start order
  if (mSensThread.joinable()) { mSensThread.join(); }
  if (mPcThread.joinable()) { mPcThread.join(); }
  if (mVdThread.joinable()) { mVdThread.join(); }
  if (mGrabThread.joinable()) { mGrabThread.join(); }

  closeCamera();
}

// ===========================================================================
// initParameters
// ===========================================================================

void ZedCameraMlx::initParameters()
{
  getDebugParams();
  getGeneralParams();
  getVideoParams();
  getDepthParams();
  getSensorsParams();
  getMlxParams();
}

void ZedCameraMlx::getDebugParams()
{
#ifdef ZED_ROS2_AVAILABLE
  // TODO(mlx-port): Declare and load debug.* parameters via
  //   declare_parameter / get_parameter, same style as upstream.
  //   For now, defaults are fine.
  RCLCPP_INFO(get_logger(), "=== DEBUG parameters ===");
#endif
}

void ZedCameraMlx::getGeneralParams()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== GENERAL parameters ===");

  // TODO(mlx-port): Load general.camera_name, general.camera_model,
  //   general.serial_number, general.grab_frame_rate, general.gpu_id
  //   from ROS2 parameters.
  //
  // Example (when sl_tools equivalent is ready):
  //   declare_parameter("general.camera_name", mCameraName);
  //   get_parameter("general.camera_name", mCameraName);

  RCLCPP_INFO_STREAM(get_logger(), " * Camera name: " << mCameraName);
  RCLCPP_INFO_STREAM(get_logger(), " * Grab rate: " << mCamGrabFrameRate << " Hz");
#endif
}

void ZedCameraMlx::getVideoParams()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== VIDEO parameters ===");

  // TODO(mlx-port): Load video.brightness, video.contrast,
  //   video.auto_exposure_gain, video.exposure, video.gain
  //   from ROS2 parameters.  These will be forwarded to UVC controls
  //   via open-capture.

  RCLCPP_INFO_STREAM(get_logger(), " * Resolution: " << mCamWidth << "x" << mCamHeight);
#endif
}

void ZedCameraMlx::getDepthParams()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== DEPTH parameters ===");

  // TODO(mlx-port): Load depth.min_depth, depth.max_depth,
  //   depth.openni_depth_mode, depth.point_cloud_freq,
  //   depth.publish_depth_map, depth.publish_point_cloud
  //   from ROS2 parameters.

  if (!mDepthEnabled) {
    RCLCPP_INFO(get_logger(), " * Depth: DISABLED");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * Min depth [m]: " << mCamMinDepth);
    RCLCPP_INFO_STREAM(get_logger(), " * Max depth [m]: " << mCamMaxDepth);
    RCLCPP_INFO_STREAM(get_logger(), " * Point cloud rate [Hz]: " << mPcPubRate);
  }
#endif
}

void ZedCameraMlx::getSensorsParams()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== SENSORS parameters ===");

  // TODO(mlx-port): Load sensors.publish_imu, sensors.publish_imu_raw,
  //   sensors.publish_temp from ROS2 parameters.

  RCLCPP_INFO_STREAM(get_logger(), " * Publish IMU: " << (mPublishImu ? "TRUE" : "FALSE"));
  RCLCPP_INFO_STREAM(get_logger(), " * IMU rate [Hz]: " << mSensPubRate);
#endif
}

void ZedCameraMlx::getMlxParams()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== MLX DEPTH parameters ===");

  // TODO(mlx-port): Declare and load MLX-specific parameters:
  //   mlx.model_path        -- path to .mlx depth model weights
  //   mlx.input_width       -- model input width  (default 896)
  //   mlx.input_height      -- model input height (default 512)
  //   mlx.confidence_thresh  -- minimum confidence (default 0.5)
  //   mlx.use_fp16           -- FP16 inference flag (default true)
  //
  // These have no upstream equivalent -- they are new for the macOS port.

  RCLCPP_INFO_STREAM(get_logger(), " * Model path: " << mMlxModelPath);
  RCLCPP_INFO_STREAM(get_logger(), " * Input size: " << mMlxInputWidth << "x" << mMlxInputHeight);
  RCLCPP_INFO_STREAM(get_logger(), " * FP16: " << (mMlxUseFp16 ? "TRUE" : "FALSE"));
#endif
}

// ===========================================================================
// setTFCoordFrameNames  --  identical logic to upstream
// ===========================================================================

void ZedCameraMlx::setTFCoordFrameNames()
{
#ifdef ZED_ROS2_AVAILABLE
  mBaseFrameId = mCameraName + "_base_link";
  mCenterFrameId = mCameraName + "_camera_center";
  mLeftCamFrameId = mCameraName + "_left_camera_frame";
  mLeftCamOptFrameId = mCameraName + "_left_camera_frame_optical";
  mRightCamFrameId = mCameraName + "_right_camera_frame";
  mRightCamOptFrameId = mCameraName + "_right_camera_frame_optical";
  mImuFrameId = mCameraName + "_imu_link";
  mDepthFrameId = mLeftCamFrameId;
  mDepthOptFrameId = mLeftCamOptFrameId;
  mPointCloudFrameId = mDepthFrameId;

  RCLCPP_INFO(get_logger(), "=== TF FRAMES ===");
  RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t-> " << mBaseFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Camera\t-> " << mCenterFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t-> " << mLeftCamFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t-> " << mRightCamFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t-> " << mDepthFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t-> " << mImuFrameId);
#endif
}

// ===========================================================================
// initServices  --  kept subset (enable_depth only)
// ===========================================================================

void ZedCameraMlx::initServices()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== SERVICES ===");

  if (mDepthEnabled) {
    std::string srv_name = "~/enable_depth";
    mEnableDepthSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCameraMlx::callback_enableDepth, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on service: '" << mEnableDepthSrv->get_service_name() << "'");
  }

  // ---- Unsupported upstream services ----
  RCLCPP_INFO(get_logger(), " [mlx-port] Object detection service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Body tracking service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Spatial mapping service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] SVO recording service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Streaming server service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] GNSS fusion service: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Positional tracking service: NOT SUPPORTED");
#endif
}

// ===========================================================================
// initPublishers
// ===========================================================================

void ZedCameraMlx::initPublishers()
{
#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(), "=== PUBLISHED TOPICS ===");
#endif

  initVideoDepthPublishers();

  // ---- IMU publishers ----
#ifdef ZED_ROS2_AVAILABLE
  if (mPublishImu) {
    std::string imu_topic = mTopicRoot + "imu/data";
    mPubImu = create_publisher<sensor_msgs::msg::Imu>(imu_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubImu->get_topic_name());

    std::string temp_topic = mTopicRoot + "temperature/imu";
    mPubImuTemp = create_publisher<sensor_msgs::msg::Temperature>(temp_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubImuTemp->get_topic_name());
  }

  // ---- Unsupported upstream publishers ----
  RCLCPP_INFO(get_logger(), " [mlx-port] Odometry/Pose publishers: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Fused point cloud publisher: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Object detection publisher: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Body tracking publisher: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] GNSS publishers: NOT SUPPORTED");
#endif
}

// ===========================================================================
// initVideoDepthPublishers  --  same topic names as upstream
// ===========================================================================

void ZedCameraMlx::initVideoDepthPublishers()
{
#ifdef ZED_ROS2_AVAILABLE
  // ---- Build topic names (same structure as upstream) ----
  const std::string sens_rgb = "rgb/";
  const std::string sens_left = "left/";
  const std::string sens_right = "right/";
  const std::string sens_stereo = "stereo/";
  const std::string rectified = "rect/";
  const std::string raw = "raw/";
  const std::string color = "color/";
  const std::string type_image = "image";

  auto make_topic = [&](const std::string & sensor,
      const std::string & color_mode,
      const std::string & rect_raw,
      const std::string & type) -> std::string {
    return mTopicRoot + sensor + color_mode + rect_raw + type;
  };

  mLeftTopic = make_topic(sens_left, color, rectified, type_image);
  mLeftRawTopic = make_topic(sens_left, color, raw, type_image);
  mRightTopic = make_topic(sens_right, color, rectified, type_image);
  mRightRawTopic = make_topic(sens_right, color, raw, type_image);
  mRgbTopic = make_topic(sens_rgb, color, rectified, type_image);
  mRgbRawTopic = make_topic(sens_rgb, color, raw, type_image);
  mStereoTopic = make_topic(sens_stereo, color, rectified, type_image);
  mStereoRawTopic = make_topic(sens_stereo, color, raw, type_image);

  mDepthTopic = mTopicRoot + "depth/depth_registered";
  mDepthInfoTopic = mTopicRoot + "depth/depth_info";
  mPointcloudTopic = mTopicRoot + "point_cloud/cloud_registered";

  auto qos = mQos.get_rmw_qos_profile();

  // ---- Image publishers (image_transport, no NITROS on macOS) ----
  if (mPublishImgRgb) {
    mPubRgb = image_transport::create_publisher(this, mRgbTopic, qos);
    RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRgb.getTopic());

    if (mPublishImgRaw) {
      mPubRawRgb = image_transport::create_publisher(this, mRgbRawTopic, qos);
      RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRawRgb.getTopic());
    }
  }

  if (mPublishImgLeftRight) {
    mPubLeft = image_transport::create_publisher(this, mLeftTopic, qos);
    RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubLeft.getTopic());
    mPubRight = image_transport::create_publisher(this, mRightTopic, qos);
    RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRight.getTopic());

    if (mPublishImgRaw) {
      mPubRawLeft = image_transport::create_publisher(this, mLeftRawTopic, qos);
      RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRawLeft.getTopic());
      mPubRawRight = image_transport::create_publisher(this, mRightRawTopic, qos);
      RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRawRight.getTopic());
    }
  }

  if (mPublishImgStereo) {
    mPubStereo = image_transport::create_publisher(this, mStereoTopic, qos);
    RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubStereo.getTopic());
    if (mPublishImgRaw) {
      mPubRawStereo = image_transport::create_publisher(this, mStereoRawTopic, qos);
      RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubRawStereo.getTopic());
    }
  }

  // ---- Depth publisher ----
  if (mDepthEnabled && mPublishDepthMap) {
    mPubDepth = image_transport::create_publisher(this, mDepthTopic, qos);
    RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << mPubDepth.getTopic());
  }

  // ---- Camera Info publishers ----
  // TODO(mlx-port): Create camera info publishers for each image topic
  //   (mirroring the upstream make_cam_info_pub / make_cam_info_trans_pub
  //    lambda pattern).  For brevity only RGB/Left/Right/Depth shown here.
  if (mPublishImgRgb) {
    mPubRgbCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
      mRgbTopic + "/camera_info", mQos);
  }
  if (mPublishImgLeftRight) {
    mPubLeftCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
      mLeftTopic + "/camera_info", mQos);
    mPubRightCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
      mRightTopic + "/camera_info", mQos);
  }
  if (mDepthEnabled && mPublishDepthMap) {
    mPubDepthCamInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
      mDepthTopic + "/camera_info", mQos);
  }

  // ---- Point cloud publisher ----
  if (mDepthEnabled && mPublishPointcloud) {
    mPubCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
      mPointcloudTopic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubCloud->get_topic_name());
  }

  // ---- Unsupported depth-related publishers ----
  RCLCPP_INFO(get_logger(), " [mlx-port] Disparity publisher: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] Confidence map publisher: NOT SUPPORTED");
  RCLCPP_INFO(get_logger(), " [mlx-port] ROI mask publisher: NOT SUPPORTED");
#endif  // ZED_ROS2_AVAILABLE
}

// ===========================================================================
// initThreads  --  same structure as upstream
// ===========================================================================

void ZedCameraMlx::initThreads()
{
  // Sensor thread (IMU)
  if (mPublishImu) {
    mSensThread = std::thread(&ZedCameraMlx::threadFunc_sensorData, this);
  }

  // Video/Depth processing thread
  mVdDataReady = false;
  mVdThread = std::thread(&ZedCameraMlx::threadFunc_videoDepthElab, this);

  // Point cloud thread
  if (mDepthEnabled && mPublishPointcloud) {
    mPcDataReady = false;
    mPcThread = std::thread(&ZedCameraMlx::threadFunc_pointcloudElab, this);
  }

  // Main grab thread (producer)
  mGrabThread = std::thread(&ZedCameraMlx::threadFunc_grab, this);
}

// ===========================================================================
// openCamera  --  open via open-capture USB bridge
// ===========================================================================

bool ZedCameraMlx::openCamera()
{
  // TODO(mlx-port): Implement camera open using the open-capture bridge.
  //
  // Pseudocode:
  //   #include <open_capture_bridge.hpp>
  //   mCaptureHandle = open_capture::open(mCamSerialNumber, mCamWidth,
  //                                        mCamHeight, mCamGrabFrameRate);
  //   if (!mCaptureHandle) return false;
  //   mCameraOpen = true;
  //   return true;
  //
  // The open-capture bridge wraps libusb / UVC to open the ZED as a
  // standard stereo USB camera on macOS.

#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_WARN(get_logger(),
    "TODO(mlx-port): openCamera() -- open-capture bridge not yet implemented");
#endif
  mCameraOpen = false;  // Will become true once bridge is implemented
  return true;           // Return true for skeleton testing
}

void ZedCameraMlx::closeCamera()
{
  // TODO(mlx-port): Close the open-capture handle.
  //   if (mCaptureHandle) { open_capture::close(mCaptureHandle); }
  mCameraOpen = false;
}

// ===========================================================================
// loadCalibration  --  parse ZED factory .conf file
// ===========================================================================

bool ZedCameraMlx::loadCalibration()
{
  // TODO(mlx-port): Implement calibration loading.
  //
  // The ZED factory calibration is stored in:
  //   /usr/local/zed/settings/<serial_number>.conf   (Linux)
  //   ~/Library/Application Support/Stereolabs/settings/<serial>.conf (macOS)
  //
  // Parse the .conf INI file to extract:
  //   [LEFT_CAM_<resolution>]  fx, fy, cx, cy, k1..k6
  //   [RIGHT_CAM_<resolution>] fx, fy, cx, cy, k1..k6
  //   [STEREO]                 baseline, rx, rz
  //
  // Then build cv::initUndistortRectifyMap() for left and right:
  //   cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left,
  //     cv::Size(mCamWidth, mCamHeight), CV_32FC1,
  //     mCalib.left_map1, mCalib.left_map2);
  //
  // Store results in mCalib.

#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_WARN(get_logger(),
    "TODO(mlx-port): loadCalibration() -- factory .conf parser not yet implemented");
#endif
  return false;
}

// ===========================================================================
// fillCamInfo  --  populate CameraInfo messages from mCalib
// ===========================================================================

void ZedCameraMlx::fillCamInfo(bool rawParam)
{
  // TODO(mlx-port): Populate mLeftCamInfoMsg / mRightCamInfoMsg from mCalib.
  //
  // This mirrors the upstream fillCamInfo() but reads from our ZedCalibration
  // struct instead of sl::CalibrationParameters.
  //
  // Key fields to fill:
  //   msg->distortion_model = "rational_polynomial" or "plumb_bob"
  //   msg->d = { k1, k2, p1, p2, k3 [, k4, k5, k6] }
  //   msg->k = { fx, 0, cx, 0, fy, cy, 0, 0, 1 }
  //   msg->r = 3x3 identity (rectified) or rotation (raw)
  //   msg->p = 3x4 projection matrix
  //   msg->width  = mCamWidth
  //   msg->height = mCamHeight
  //   msg->header.frame_id = left/right optical frame

#ifdef ZED_ROS2_AVAILABLE
  if (rawParam) {
    mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    // TODO(mlx-port): Fill raw camera info from mCalib (unrectified intrinsics)
  } else {
    mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    // TODO(mlx-port): Fill rectified camera info from mCalib
  }
#endif
}

// ===========================================================================
// threadFunc_grab  --  main acquisition loop
// ===========================================================================

void ZedCameraMlx::threadFunc_grab()
{
  // This replaces upstream's threadFunc_zedGrab().
  // Sequence per iteration:
  //   1. Grab side-by-side frame from open-capture
  //   2. Split into left / right
  //   3. Rectify using calibration maps
  //   4. Run MLX depth inference on the rectified left image
  //   5. Signal the video/depth and point-cloud threads

  mFrameCount = 0;

  while (!mThreadStop) {
    // ---- Interruption check ----
#ifdef ZED_ROS2_AVAILABLE
    if (!rclcpp::ok()) {
      mThreadStop = true;
      break;
    }
#endif

    auto grab_start = std::chrono::steady_clock::now();

    // ------------------------------------------------------------------
    // STEP 1: Grab raw side-by-side frame
    // ------------------------------------------------------------------
    // TODO(mlx-port): Grab a frame from the open-capture bridge.
    //
    //   cv::Mat sbs_frame;  // side-by-side BGRA or BGR
    //   bool ok = open_capture::grab(mCaptureHandle, sbs_frame);
    //   if (!ok) { rclcpp::sleep_for(1ms); continue; }
    //
    // For now, create dummy frames for skeleton testing:
    cv::Mat sbs_frame;  // empty -- will be populated by bridge

    // ------------------------------------------------------------------
    // STEP 2: Split stereo
    // ------------------------------------------------------------------
    // TODO(mlx-port): Split the side-by-side frame into left and right.
    //
    //   int half_w = sbs_frame.cols / 2;
    //   mMatLeftRaw  = sbs_frame(cv::Rect(0,      0, half_w, sbs_frame.rows)).clone();
    //   mMatRightRaw = sbs_frame(cv::Rect(half_w, 0, half_w, sbs_frame.rows)).clone();

    // ------------------------------------------------------------------
    // STEP 3: Rectify
    // ------------------------------------------------------------------
    // TODO(mlx-port): Apply rectification maps from loadCalibration().
    //
    //   if (!mCalib.left_map1.empty()) {
    //     cv::remap(mMatLeftRaw,  mMatLeft,  mCalib.left_map1,  mCalib.left_map2,  cv::INTER_LINEAR);
    //     cv::remap(mMatRightRaw, mMatRight, mCalib.right_map1, mCalib.right_map2, cv::INTER_LINEAR);
    //   } else {
    //     mMatLeft  = mMatLeftRaw;
    //     mMatRight = mMatRightRaw;
    //   }

    // ------------------------------------------------------------------
    // STEP 4: MLX depth inference
    // ------------------------------------------------------------------
    if (mDepthEnabled) {
      // TODO(mlx-port): Run MLX depth inference.
      //
      //   #include <mlx_depth_bridge.hpp>
      //   MlxDepthResult result = mlx_depth::infer(
      //       mMatLeft, mMlxModelPath,
      //       mMlxInputWidth, mMlxInputHeight, mMlxUseFp16);
      //   mMatDepth      = result.depth_f32;
      //   mMatConfidence = result.confidence_u8;
      //
      //   // Clamp depth to [min, max]
      //   cv::threshold(mMatDepth, mMatDepth, mCamMaxDepth, 0, cv::THRESH_TRUNC);
      //   // Set values below min to 0 (invalid)
      //   cv::Mat mask = mMatDepth < mCamMinDepth;
      //   mMatDepth.setTo(0.0f, mask);
    }

    // ------------------------------------------------------------------
    // STEP 5: Update timestamp and signal consumers
    // ------------------------------------------------------------------
    mLastGrabTs_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
#ifdef ZED_ROS2_AVAILABLE
    mFrameTimestamp = now();
#endif
    mFrameCount++;

    // Signal video/depth thread
    {
      std::lock_guard<std::mutex> lock(mVdMutex);
      mVdDataReady = true;
    }
    mVdDataReadyCondVar.notify_one();

    // Signal point cloud thread
    if (mDepthEnabled && mPublishPointcloud) {
      std::lock_guard<std::mutex> lock(mPcMutex);
      mPcDataReady = true;
      mPcDataReadyCondVar.notify_one();
    }

    // ------------------------------------------------------------------
    // Frame rate limiting
    // ------------------------------------------------------------------
    auto grab_end = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
      grab_end - grab_start).count();
    int64_t target_us = 1000000 / std::max(mCamGrabFrameRate, 1);
    if (elapsed_us < target_us) {
      std::this_thread::sleep_for(std::chrono::microseconds(target_us - elapsed_us));
    }
  }
}

// ===========================================================================
// threadFunc_videoDepthElab  --  video/depth publish thread
// ===========================================================================

void ZedCameraMlx::threadFunc_videoDepthElab()
{
  // This mirrors upstream threadFunc_videoDepthElab().
  // It waits for new frame data from the grab thread, then publishes
  // images and depth.

  while (!mThreadStop) {
#ifdef ZED_ROS2_AVAILABLE
    if (!rclcpp::ok()) { break; }
#endif

    // Wait for data
    {
      std::unique_lock<std::mutex> lock(mVdMutex);
      mVdDataReadyCondVar.wait_for(lock, 500ms, [this] { return mVdDataReady.load(); });
      if (!mVdDataReady) { continue; }
      mVdDataReady = false;
    }

    // ---- Publish images ----
    // See zed_camera_mlx_video_depth.cpp for the actual publishing logic.
    // TODO(mlx-port): Call publishing functions:
    //
    //   publishLeftAndRgbImages();
    //   publishRightImages();
    //   publishDepthImage();
    //   publishCameraInfos();
  }
}

// ===========================================================================
// threadFunc_pointcloudElab  --  point cloud publish thread
// ===========================================================================

void ZedCameraMlx::threadFunc_pointcloudElab()
{
  while (!mThreadStop) {
#ifdef ZED_ROS2_AVAILABLE
    if (!rclcpp::ok()) { break; }
#endif

    // Wait for depth data
    {
      std::unique_lock<std::mutex> lock(mPcMutex);
      mPcDataReadyCondVar.wait_for(lock, 500ms, [this] { return mPcDataReady.load(); });
      if (!mPcDataReady) { continue; }
      mPcDataReady = false;
    }

    // TODO(mlx-port): Build and publish point cloud.
    //   publishPointCloud(mMatDepth, mMatLeft);
  }
}

// ===========================================================================
// threadFunc_sensorData  --  IMU publish loop
// ===========================================================================

void ZedCameraMlx::threadFunc_sensorData()
{
  // This replaces upstream's threadFunc_pubSensorsData().
  // On macOS, we read IMU data from the ZED's built-in IMU via:
  //   - open-capture's sensor polling, or
  //   - direct hidapi access to the ZED IMU USB endpoint.

  while (!mThreadStop) {
#ifdef ZED_ROS2_AVAILABLE
    if (!rclcpp::ok()) { break; }
#endif

    // TODO(mlx-port): Poll IMU data from the open-capture bridge.
    //
    //   ImuSample sample;
    //   bool ok = open_capture::getImuData(mCaptureHandle, sample);
    //   if (ok) {
    //     publishImuData(sample);
    //   }

    // Rate-limit to mSensPubRate
    auto sleep_us = static_cast<int>(1000000.0 / std::max(mSensPubRate, 1.0));
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
  }
}

// ===========================================================================
// Dynamic parameter callback
// ===========================================================================

#ifdef ZED_ROS2_AVAILABLE
rcl_interfaces::msg::SetParametersResult
ZedCameraMlx::callback_dynamicParamChange(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    // TODO(mlx-port): Handle dynamic parameter changes for:
    //   - video.brightness, video.contrast, video.exposure, video.gain
    //     -> forward to UVC controls via open-capture
    //   - depth.min_depth, depth.max_depth
    //     -> update clamping range
    //   - mlx.confidence_thresh, mlx.use_fp16
    //     -> update MLX inference settings

    RCLCPP_INFO_STREAM(get_logger(),
      "Dynamic param changed: " << param.get_name() << " = "
        << param.value_to_string());
  }

  return result;
}
#endif

// ===========================================================================
// Diagnostic callback
// ===========================================================================

#ifdef ZED_ROS2_AVAILABLE
void ZedCameraMlx::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // TODO(mlx-port): Populate diagnostic status with:
  //   - Camera open state
  //   - Grab FPS
  //   - MLX inference time
  //   - IMU status

  if (mCameraOpen) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera running");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera not open");
  }

  stat.add("Frame count", std::to_string(mFrameCount));
  stat.add("MLX model", mMlxModelPath);
}
#endif

// ===========================================================================
// Service callbacks
// ===========================================================================

#ifdef ZED_ROS2_AVAILABLE
void ZedCameraMlx::callback_enableDepth(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  mDepthEnabled = req->data;
  res->success = true;
  res->message = mDepthEnabled ? "Depth enabled" : "Depth disabled";
  RCLCPP_INFO_STREAM(get_logger(), "Depth processing: " << res->message);
}
#endif

}  // namespace stereolabs
