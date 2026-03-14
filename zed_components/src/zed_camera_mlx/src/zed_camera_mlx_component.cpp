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
#include <geometry_msgs/msg/transform_stamped.hpp>
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

  // 4b. TF broadcasters
#ifdef ZED_ROS2_AVAILABLE
  mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  mTfListener = std::make_unique<tf2_ros::TransformListener>(*mTfBuffer);
  mStaticTfBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  RCLCPP_INFO(get_logger(), "=== TF BROADCASTERS initialised ===");
#endif

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
  // Fixed world frames (no SLAM, so these are global)
  mMapFrameId = "map";
  mOdomFrameId = "odom";

  // Camera-prefixed frames
  mBaseFrameId = mCameraName + "_base_link";
  mCenterFrameId = mCameraName + "_camera_center";
  mLeftCamFrameId = mCameraName + "_left_camera_frame";
  mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
  mRightCamFrameId = mCameraName + "_right_camera_frame";
  mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";
  mImuFrameId = mCameraName + "_imu_link";
  mDepthFrameId = mLeftCamFrameId;
  mDepthOptFrameId = mLeftCamOptFrameId;
  mPointCloudFrameId = mLeftCamOptFrameId;

  RCLCPP_INFO(get_logger(), "=== TF FRAMES ===");
  RCLCPP_INFO_STREAM(get_logger(), " * Map\t\t-> " << mMapFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Odom\t\t-> " << mOdomFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t-> " << mBaseFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Camera\t-> " << mCenterFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t-> " << mLeftCamFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left Opt\t-> " << mLeftCamOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t-> " << mRightCamFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right Opt\t-> " << mRightCamOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t-> " << mDepthFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t-> " << mImuFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t-> " << mPointCloudFrameId);
#endif
}

// ===========================================================================
// publishStaticTFs  --  one-shot static transforms
// ===========================================================================

void ZedCameraMlx::publishStaticTFs()
{
#ifdef ZED_ROS2_AVAILABLE
  if (!mStaticTfBroadcaster) {
    return;
  }

  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
  auto stamp = now();

  // --- Helper: create an identity TransformStamped ---
  auto makeIdentityTF = [&](const std::string & parent,
                            const std::string & child) -> geometry_msgs::msg::TransformStamped {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    return tf;
  };

  // --- Static TF: camera_center -> left_camera_optical_frame ---
  // Standard ROS2 optical frame convention:
  //   Camera frame:  X-right, Y-down, Z-forward
  //   Optical frame: X-right, Y-down, Z-forward (after rotation)
  // The rotation from camera_link to optical frame is:
  //   R = Ry(-pi/2) * Rz(-pi/2)
  // Quaternion: (x=-0.5, y=0.5, z=-0.5, w=0.5)
  {
    auto tf = makeIdentityTF(mCenterFrameId, mLeftCamOptFrameId);
    tf.transform.rotation.x = -0.5;
    tf.transform.rotation.y =  0.5;
    tf.transform.rotation.z = -0.5;
    tf.transform.rotation.w =  0.5;
    static_transforms.push_back(tf);
  }

  // --- Static TF: camera_center -> right_camera_optical_frame ---
  // Same rotation as left, but offset by baseline along the X axis
  {
    auto tf = makeIdentityTF(mCenterFrameId, mRightCamOptFrameId);
    tf.transform.translation.x = -mCalib.baseline_m;  // right cam offset in negative X
    tf.transform.rotation.x = -0.5;
    tf.transform.rotation.y =  0.5;
    tf.transform.rotation.z = -0.5;
    tf.transform.rotation.w =  0.5;
    static_transforms.push_back(tf);
  }

  // --- Static TF: camera_center -> left_camera_frame (physical frame, no rotation) ---
  {
    auto tf = makeIdentityTF(mCenterFrameId, mLeftCamFrameId);
    static_transforms.push_back(tf);
  }

  // --- Static TF: camera_center -> right_camera_frame (offset by baseline) ---
  {
    auto tf = makeIdentityTF(mCenterFrameId, mRightCamFrameId);
    tf.transform.translation.x = -mCalib.baseline_m;
    static_transforms.push_back(tf);
  }

  // --- Static TF: camera_center -> imu_link ---
  // Small offset between camera center and IMU; use identity for now.
  // TODO(mlx-port): Use actual camera-to-IMU extrinsic from ZED factory calibration.
  {
    auto tf = makeIdentityTF(mCenterFrameId, mImuFrameId);
    static_transforms.push_back(tf);
  }

  mStaticTfBroadcaster->sendTransform(static_transforms);
  RCLCPP_INFO(get_logger(),
    "Published %zu static TF transforms", static_transforms.size());
#endif
}

// ===========================================================================
// publishTFs  --  per-frame dynamic transforms (identity without SLAM)
// ===========================================================================

void ZedCameraMlx::publishTFs()
{
#ifdef ZED_ROS2_AVAILABLE
  if (!mTfBroadcaster) {
    return;
  }

  auto stamp = now();

  auto makeIdentityTF = [&](const std::string & parent,
                            const std::string & child) -> geometry_msgs::msg::TransformStamped {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    return tf;
  };

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // map -> odom: identity (no SLAM drift correction)
  transforms.push_back(makeIdentityTF(mMapFrameId, mOdomFrameId));

  // odom -> base_link: identity (no odometry / wheel encoders)
  transforms.push_back(makeIdentityTF(mOdomFrameId, mBaseFrameId));

  // base_link -> camera_center: identity (URDF typically handles this,
  // but we publish it here as fallback since robot_state_publisher may not
  // be running in a standalone camera setup)
  transforms.push_back(makeIdentityTF(mBaseFrameId, mCenterFrameId));

  mTfBroadcaster->sendTransform(transforms);
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

    // ---- Magnetometer publisher ----
    std::string mag_topic = mTopicRoot + "imu/mag";
    mPubMag = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubMag->get_topic_name());

    // ---- Barometer publisher ----
    std::string baro_topic = mTopicRoot + "atm_press";
    mPubBaro = create_publisher<sensor_msgs::msg::FluidPressure>(baro_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubBaro->get_topic_name());

    // ---- Camera CMOS temperature publishers ----
    std::string temp_left_topic = mTopicRoot + "temperature/left";
    mPubTempLeft = create_publisher<sensor_msgs::msg::Temperature>(temp_left_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubTempLeft->get_topic_name());

    std::string temp_right_topic = mTopicRoot + "temperature/right";
    mPubTempRight = create_publisher<sensor_msgs::msg::Temperature>(temp_right_topic, mQos);
    RCLCPP_INFO_STREAM(get_logger(),
      " * Advertised on topic: " << mPubTempRight->get_topic_name());
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
  // Create the SlOcCamera instance and configure InitParameters
  mSlCamera = std::make_unique<sl_oc_bridge::SlOcCamera>();

  sl_oc_bridge::InitParameters init_params;
  init_params.camera_resolution = sl_oc_bridge::RESOLUTION_PRESET::HD720;
  init_params.camera_fps = mCamGrabFrameRate;
  init_params.camera_device_id = -1;  // auto-detect
  init_params.depth_minimum_distance = static_cast<float>(mCamMinDepth);
  init_params.depth_maximum_distance = static_cast<float>(mCamMaxDepth);

  // If a serial number filter was configured, apply it
  if (mCamSerialNumber > 0) {
    init_params.serial_number_filter = mCamSerialNumber;
  }

  // If an OpenCV calibration file was specified, pass it through
  if (!mOpencvCalibFile.empty()) {
    init_params.opencv_calibration_file = mOpencvCalibFile;
  }

  sl_oc_bridge::ERROR_CODE err = mSlCamera->open(init_params);

  if (err != sl_oc_bridge::ERROR_CODE::SUCCESS) {
#ifdef ZED_ROS2_AVAILABLE
    RCLCPP_ERROR(get_logger(),
      "SlOcCamera::open() failed: %s", sl_oc_bridge::toString(err));
#else
    std::cerr << "[ZedCameraMlx] SlOcCamera::open() failed: "
              << sl_oc_bridge::toString(err) << std::endl;
#endif
    mSlCamera.reset();
    mCameraOpen = false;
    return false;
  }

  // Read back actual resolution from the camera
  sl_oc_bridge::CameraInformation cam_info = mSlCamera->getCameraInformation();
  mCamWidth = static_cast<int>(cam_info.camera_resolution.width);
  mCamHeight = static_cast<int>(cam_info.camera_resolution.height);
  mCamSerialNumber = cam_info.serial_number;

  // Populate calibration struct from camera info
  const auto& cal = cam_info.camera_configuration;
  mCalib.left_fx = cal.left_cam.fx;
  mCalib.left_fy = cal.left_cam.fy;
  mCalib.left_cx = cal.left_cam.cx;
  mCalib.left_cy = cal.left_cam.cy;
  mCalib.right_fx = cal.right_cam.fx;
  mCalib.right_fy = cal.right_cam.fy;
  mCalib.right_cx = cal.right_cam.cx;
  mCalib.right_cy = cal.right_cam.cy;
  mCalib.baseline_m = cal.baseline;

  // Store distortion coefficients (up to 12 from CameraParameters::disto)
  mCalib.left_distortion.assign(cal.left_cam.disto, cal.left_cam.disto + 12);
  mCalib.right_distortion.assign(cal.right_cam.disto, cal.right_cam.disto + 12);

  // Apply initial camera settings
  if (mCamAutoExpGain) {
    mSlCamera->setAutoExposureGain(true);
  } else {
    mSlCamera->setAutoExposureGain(false);
    mSlCamera->setExposure(mCamExposure);
    mSlCamera->setGain(mCamGain);
  }
  mSlCamera->setBrightness(mCamBrightness);
  mSlCamera->setContrast(mCamContrast);

  mCameraOpen = true;

#ifdef ZED_ROS2_AVAILABLE
  RCLCPP_INFO(get_logger(),
    "Camera opened: serial=%d, resolution=%dx%d, fps=%d",
    mCamSerialNumber, mCamWidth, mCamHeight, mCamGrabFrameRate);
#else
  std::cout << "[ZedCameraMlx] Camera opened: serial=" << mCamSerialNumber
            << " resolution=" << mCamWidth << "x" << mCamHeight
            << " fps=" << mCamGrabFrameRate << std::endl;
#endif

  return true;
}

void ZedCameraMlx::closeCamera()
{
  if (mSlCamera) {
    mSlCamera->close();
    mSlCamera.reset();
  }
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
#ifdef ZED_ROS2_AVAILABLE
  auto fillMsg = [&](
    sensor_msgs::msg::CameraInfo::SharedPtr & msg,
    double fx, double fy, double cx, double cy,
    const std::vector<double> & distortion,
    const std::string & frameId,
    double baseline_fx)  // Tx component of projection matrix: -fx * baseline (for right cam)
  {
    msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

    msg->width = static_cast<uint32_t>(mCamWidth);
    msg->height = static_cast<uint32_t>(mCamHeight);
    msg->header.frame_id = frameId;

    // Distortion model
    if (rawParam && distortion.size() > 5) {
      msg->distortion_model = "rational_polynomial";
    } else {
      msg->distortion_model = "plumb_bob";
    }

    // Distortion coefficients: k1, k2, p1, p2, k3 [, k4, k5, k6]
    if (rawParam) {
      // For raw, include actual distortion
      size_t n = std::min(distortion.size(), static_cast<size_t>(8));
      msg->d.resize(n);
      for (size_t i = 0; i < n; ++i) {
        msg->d[i] = distortion[i];
      }
    } else {
      // For rectified, distortion is zero
      msg->d.assign(5, 0.0);
    }

    // Intrinsic camera matrix K (3x3 row-major)
    msg->k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };

    // Rectification matrix R (3x3 identity for rectified; identity placeholder for raw)
    msg->r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // Projection matrix P (3x4)
    // For left camera: P = [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]
    // For right camera: P = [fx 0 cx Tx; 0 fy cy 0; 0 0 1 0]  where Tx = -fx * baseline
    msg->p = {
      fx, 0.0, cx, baseline_fx,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };
  };

  if (rawParam) {
    fillMsg(mLeftCamInfoRawMsg,
      mCalib.left_fx, mCalib.left_fy, mCalib.left_cx, mCalib.left_cy,
      mCalib.left_distortion,
      mLeftCamOptFrameId,
      0.0);

    fillMsg(mRightCamInfoRawMsg,
      mCalib.right_fx, mCalib.right_fy, mCalib.right_cx, mCalib.right_cy,
      mCalib.right_distortion,
      mRightCamOptFrameId,
      -mCalib.right_fx * mCalib.baseline_m);
  } else {
    fillMsg(mLeftCamInfoMsg,
      mCalib.left_fx, mCalib.left_fy, mCalib.left_cx, mCalib.left_cy,
      mCalib.left_distortion,
      mLeftCamOptFrameId,
      0.0);

    fillMsg(mRightCamInfoMsg,
      mCalib.right_fx, mCalib.right_fy, mCalib.right_cx, mCalib.right_cy,
      mCalib.right_distortion,
      mRightCamOptFrameId,
      -mCalib.right_fx * mCalib.baseline_m);
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
    // STEP 1: Grab frame via SlOcCamera bridge
    // ------------------------------------------------------------------
    if (!mSlCamera || !mSlCamera->isOpened()) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    sl_oc_bridge::ERROR_CODE grab_err = mSlCamera->grab();
    if (grab_err != sl_oc_bridge::ERROR_CODE::SUCCESS) {
      // No new frame or corrupted -- retry next iteration
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // ------------------------------------------------------------------
    // STEP 2: Retrieve left and right BGRA images from the bridge
    // ------------------------------------------------------------------
    sl_oc_bridge::Mat bridge_left, bridge_right;

    // view: 0=LEFT, 1=RIGHT (bridge handles YUYV->BGRA internally)
    sl_oc_bridge::ERROR_CODE err_l = mSlCamera->retrieveImage(bridge_left, 0);
    sl_oc_bridge::ERROR_CODE err_r = mSlCamera->retrieveImage(bridge_right, 1);

    if (err_l != sl_oc_bridge::ERROR_CODE::SUCCESS ||
        err_r != sl_oc_bridge::ERROR_CODE::SUCCESS) {
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // Wrap bridge Mat data as cv::Mat (BGRA, 4 channels, 8-bit)
    // The bridge Mat owns the data; we clone into our cv::Mat members.
    mMatLeftRaw = cv::Mat(
      static_cast<int>(bridge_left.getHeight()),
      static_cast<int>(bridge_left.getWidth()),
      CV_8UC4,
      bridge_left.getPtr<uint8_t>(),
      bridge_left.getStepBytes()).clone();

    mMatRightRaw = cv::Mat(
      static_cast<int>(bridge_right.getHeight()),
      static_cast<int>(bridge_right.getWidth()),
      CV_8UC4,
      bridge_right.getPtr<uint8_t>(),
      bridge_right.getStepBytes()).clone();

    // ------------------------------------------------------------------
    // STEP 3: Rectify using calibration maps (if available)
    // ------------------------------------------------------------------
    if (!mCalib.left_map1.empty()) {
      cv::remap(mMatLeftRaw,  mMatLeft,  mCalib.left_map1,  mCalib.left_map2,  cv::INTER_LINEAR);
      cv::remap(mMatRightRaw, mMatRight, mCalib.right_map1, mCalib.right_map2, cv::INTER_LINEAR);
    } else {
      mMatLeft  = mMatLeftRaw;
      mMatRight = mMatRightRaw;
    }

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
    // STEP 6: Publish TF frames
    // ------------------------------------------------------------------
#ifdef ZED_ROS2_AVAILABLE
    // Publish static TFs once (camera_link -> optical frames, imu_link)
    if (!mStaticTfPublished) {
      publishStaticTFs();
      mStaticTfPublished = true;
    }
    // Publish dynamic TFs each frame (identity: map->odom, odom->base_link)
    publishTFs();
#endif

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
    // Publish left/RGB image (left is used as RGB for the ZED)
#ifdef ZED_ROS2_AVAILABLE
    if (mPublishImgRgb && !mMatLeft.empty()) {
      publishImageWithInfo(mMatLeft, "bgra8", mPubRgb, mLeftCamOptFrameId);
      if (mPubRgbCamInfo && mLeftCamInfoMsg) {
        mLeftCamInfoMsg->header.stamp = mFrameTimestamp;
        mPubRgbCamInfo->publish(*mLeftCamInfoMsg);
      }
    }

    if (mPublishImgRgb && mPublishImgRaw && !mMatLeftRaw.empty()) {
      publishImageWithInfo(mMatLeftRaw, "bgra8", mPubRawRgb, mLeftCamOptFrameId);
    }

    // Publish left/right stereo pair
    if (mPublishImgLeftRight && !mMatLeft.empty()) {
      publishImageWithInfo(mMatLeft, "bgra8", mPubLeft, mLeftCamOptFrameId);
      if (mPubLeftCamInfo && mLeftCamInfoMsg) {
        mLeftCamInfoMsg->header.stamp = mFrameTimestamp;
        mPubLeftCamInfo->publish(*mLeftCamInfoMsg);
      }
    }

    if (mPublishImgLeftRight && !mMatRight.empty()) {
      publishImageWithInfo(mMatRight, "bgra8", mPubRight, mRightCamOptFrameId);
      if (mPubRightCamInfo && mRightCamInfoMsg) {
        mRightCamInfoMsg->header.stamp = mFrameTimestamp;
        mPubRightCamInfo->publish(*mRightCamInfoMsg);
      }
    }

    if (mPublishImgLeftRight && mPublishImgRaw) {
      if (!mMatLeftRaw.empty()) {
        publishImageWithInfo(mMatLeftRaw, "bgra8", mPubRawLeft, mLeftCamOptFrameId);
      }
      if (!mMatRightRaw.empty()) {
        publishImageWithInfo(mMatRightRaw, "bgra8", mPubRawRight, mRightCamOptFrameId);
      }
    }

    // Publish depth image
    if (mDepthEnabled && mPublishDepthMap && !mMatDepth.empty()) {
      publishDepthMapWithInfo(mMatDepth);
      if (mPubDepthCamInfo && mLeftCamInfoMsg) {
        mLeftCamInfoMsg->header.stamp = mFrameTimestamp;
        mPubDepthCamInfo->publish(*mLeftCamInfoMsg);
      }
    }
#endif
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

    // Build and publish point cloud from depth + left RGB
    if (!mMatDepth.empty() && !mMatLeft.empty()) {
      publishPointCloud(mMatDepth, mMatLeft);
    }
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

  // Target rates: IMU at mSensPubRate (default 200Hz),
  // magnetometer/temperature/barometer at 10Hz
  constexpr double kSlowSensorRate = 10.0;
  const auto slow_period_ns = static_cast<uint64_t>(1e9 / kSlowSensorRate);
  uint64_t last_mag_publish_ns = 0;
  uint64_t last_temp_publish_ns = 0;
  uint64_t last_baro_publish_ns = 0;

  while (!mThreadStop) {
#ifdef ZED_ROS2_AVAILABLE
    if (!rclcpp::ok()) { break; }
#endif

    auto loop_start = std::chrono::steady_clock::now();

    // Poll all sensor data from the bridge
    if (mSlCamera && mSlCamera->isOpened()) {
      auto err = mSlCamera->getSensorsData(mSensorsData);
      if (err == sl_oc_bridge::ERROR_CODE::SUCCESS) {
        auto now_ns = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

        // ---- IMU: publish at full rate when new data arrives ----
        if (mSensorsData.imu.is_available &&
            mSensorsData.imu.timestamp.getNanoseconds() != mLastImuTs_ns)
        {
          mLastImuTs_ns = mSensorsData.imu.timestamp.getNanoseconds();

          ImuSample sample;
          sample.timestamp_ns = static_cast<double>(mLastImuTs_ns);
          sample.ax = mSensorsData.imu.linear_acceleration.x;
          sample.ay = mSensorsData.imu.linear_acceleration.y;
          sample.az = mSensorsData.imu.linear_acceleration.z;
          // Convert angular velocity from deg/s to rad/s
          constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
          sample.gx = mSensorsData.imu.angular_velocity.x * kDegToRad;
          sample.gy = mSensorsData.imu.angular_velocity.y * kDegToRad;
          sample.gz = mSensorsData.imu.angular_velocity.z * kDegToRad;
          sample.temperature_c = mSensorsData.imu.temperature;

          publishImuData(sample);
        }

        // ---- Magnetometer: publish at slow rate ----
        if (mSensorsData.magnetometer.is_available &&
            mSensorsData.magnetometer.timestamp.getNanoseconds() != mLastMagTs_ns &&
            (now_ns - last_mag_publish_ns) >= slow_period_ns)
        {
          mLastMagTs_ns = mSensorsData.magnetometer.timestamp.getNanoseconds();
          last_mag_publish_ns = now_ns;
          publishMagnetometerData(mSensorsData.magnetometer);
        }

        // ---- Temperature (camera CMOS sensors): publish at slow rate ----
        if (mSensorsData.temperature.is_available &&
            mSensorsData.temperature.timestamp.getNanoseconds() != mLastTempTs_ns &&
            (now_ns - last_temp_publish_ns) >= slow_period_ns)
        {
          mLastTempTs_ns = mSensorsData.temperature.timestamp.getNanoseconds();
          last_temp_publish_ns = now_ns;
          publishTemperatureData(mSensorsData.temperature);
        }

        // ---- Barometer: publish at slow rate ----
        if (mSensorsData.barometer.is_available &&
            mSensorsData.barometer.timestamp.getNanoseconds() != mLastBaroTs_ns &&
            (now_ns - last_baro_publish_ns) >= slow_period_ns)
        {
          mLastBaroTs_ns = mSensorsData.barometer.timestamp.getNanoseconds();
          last_baro_publish_ns = now_ns;
          publishBarometerData(mSensorsData.barometer);
        }
      }
    }

    // Rate-limit to mSensPubRate
    auto loop_end = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
      loop_end - loop_start).count();
    int64_t target_us = static_cast<int64_t>(1000000.0 / std::max(mSensPubRate, 1.0));
    if (elapsed_us < target_us) {
      std::this_thread::sleep_for(std::chrono::microseconds(target_us - elapsed_us));
    }
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

// Register the component with ROS2 component infrastructure
#ifdef ZED_ROS2_AVAILABLE
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraMlx)
#endif
