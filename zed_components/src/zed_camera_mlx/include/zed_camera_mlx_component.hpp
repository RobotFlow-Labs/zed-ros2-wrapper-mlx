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

#ifndef ZED_CAMERA_MLX_COMPONENT_HPP_
#define ZED_CAMERA_MLX_COMPONENT_HPP_

// ===========================================================================
// macOS MLX port of the ZED Camera ROS2 component.
//
// All sl::Camera / sl::Fusion / sl::Mat / sl::Pose / sl::ERROR_CODE
// dependencies have been removed.  The node keeps the same ROS2 topic
// namespace so downstream consumers (rviz2, nav2, etc.) do not need to
// change.
//
// Build-gate: the file compiles (as a read-only skeleton) even when ROS2
// headers are not present, guarded by ZED_ROS2_AVAILABLE.
// ===========================================================================

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------- OpenCV (always available on macOS via Homebrew / conda) ----------
#include <opencv2/core.hpp>

// ---------- Open-capture bridge (camera + sensor access) ----------
#include "sl_oc_bridge.hpp"

// ---------- ROS2 headers -- gated so we can review without ROS2 installed ---
#ifdef ZED_ROS2_AVAILABLE

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#else  // !ZED_ROS2_AVAILABLE -- forward-declare enough for the header to parse

namespace rclcpp {
class Node {};
class TimerBase { public: using SharedPtr = std::shared_ptr<TimerBase>; };
class QoS { public: QoS(int) {} };
namespace contexts { inline void* get_global_default_context() { return nullptr; } }
}  // namespace rclcpp

namespace sensor_msgs::msg {
struct CameraInfo {};
struct Image {};
struct Imu {};
struct PointCloud2 {};
struct Temperature {};
struct MagneticField {};
struct FluidPressure {};
}  // namespace sensor_msgs::msg

namespace image_transport {
class Publisher {};
}  // namespace image_transport

namespace diagnostic_updater {
class Updater { public: template<typename... A> Updater(A&&...) {} };
class DiagnosticStatusWrapper {};
}  // namespace diagnostic_updater

namespace tf2_ros {
class Buffer {};
class TransformListener {};
class TransformBroadcaster {};
class StaticTransformBroadcaster {};
}  // namespace tf2_ros

namespace tf2 {
class Transform {};
}  // namespace tf2

#endif  // ZED_ROS2_AVAILABLE

namespace stereolabs
{

// ---------------------------------------------------------------------------
// Bridge type forward declarations (implemented in the zed-sdk-mlx repo).
// These will be provided by <open_capture_bridge.hpp> and <mlx_depth_bridge.hpp>.
// ---------------------------------------------------------------------------

/// Handle returned by the open-capture USB bridge after opening the camera.
struct OpenCaptureHandle;

/// Per-frame result from the MLX depth inference bridge.
struct MlxDepthResult
{
  cv::Mat depth_f32;       ///< CV_32FC1 depth in metres
  cv::Mat confidence_u8;   ///< CV_8UC1 confidence map (0-255)
  double inference_ms;     ///< wall-clock inference time
};

/// Calibration data loaded from the ZED factory calibration file.
struct ZedCalibration
{
  // Left camera intrinsics
  double left_fx = 0, left_fy = 0, left_cx = 0, left_cy = 0;
  std::vector<double> left_distortion;

  // Right camera intrinsics
  double right_fx = 0, right_fy = 0, right_cx = 0, right_cy = 0;
  std::vector<double> right_distortion;

  // Extrinsics
  double baseline_m = 0.12;           ///< stereo baseline in metres
  cv::Mat R_rect;                     ///< 3x3 rectification rotation
  cv::Mat left_map1, left_map2;       ///< rectification look-up tables
  cv::Mat right_map1, right_map2;
};

/// IMU sample from the ZED built-in IMU (via open-capture or hidapi).
struct ImuSample
{
  double timestamp_ns = 0;
  double ax = 0, ay = 0, az = 0;       ///< linear acceleration m/s^2
  double gx = 0, gy = 0, gz = 0;       ///< angular velocity rad/s
  double temperature_c = 0;
};

// ---------------------------------------------------------------------------
// ZedCameraMlx -- macOS-native ROS2 component
// ---------------------------------------------------------------------------

class ZedCameraMlx
#ifdef ZED_ROS2_AVAILABLE
  : public rclcpp::Node
#endif
{
public:
  explicit ZedCameraMlx(
#ifdef ZED_ROS2_AVAILABLE
    const rclcpp::NodeOptions & options
#endif
  );

  virtual ~ZedCameraMlx();

protected:
  // ---- Initialisation chain (mirrors upstream order) ----
  void initNode();
  void deInitNode();
  void initParameters();
  void initServices();
  void initPublishers();
  void initVideoDepthPublishers();
  void initThreads();

  // ---- Parameter loading (kept subset) ----
  void getDebugParams();
  void getGeneralParams();
  void getVideoParams();
  void getDepthParams();
  void getSensorsParams();
  void getMlxParams();            ///< NEW: MLX-specific tuning knobs

  // ---- TF frame names and publishing ----
  void setTFCoordFrameNames();
  void publishStaticTFs();       ///< Publish static TFs (camera_link -> optical frames, imu_link)
  void publishTFs();             ///< Publish dynamic TFs (map->odom->base_link, identity without SLAM)

  // ---- Camera lifecycle ----
  bool openCamera();              ///< Open via open-capture bridge
  void closeCamera();
  bool loadCalibration();         ///< Parse factory .conf calibration file

  // ---- Camera info ----
  void fillCamInfo(
    bool rawParam = false);

  // ---- Thread functions ----
  void threadFunc_grab();         ///< Main grab loop (replaces threadFunc_zedGrab)
  void threadFunc_videoDepthElab();
  void threadFunc_pointcloudElab();
  void threadFunc_sensorData();   ///< IMU publish loop

  // ---- Publishing helpers (implemented in video_depth source) ----
  void publishImageWithInfo(
    const cv::Mat & img,
    const std::string & encoding,
#ifdef ZED_ROS2_AVAILABLE
    image_transport::Publisher & pubImg,
#endif
    const std::string & frameId);

  void publishDepthMapWithInfo(const cv::Mat & depth);
  void publishPointCloud(const cv::Mat & depth, const cv::Mat & rgb);
  void publishImuData(const ImuSample & sample);
  void publishMagnetometerData(const sl_oc_bridge::MagnetometerData & mag_data);
  void publishTemperatureData(const sl_oc_bridge::TemperatureData & temp_data);
  void publishBarometerData(const sl_oc_bridge::BarometerData & baro_data);

  // ---- Dynamic parameter callback ----
#ifdef ZED_ROS2_AVAILABLE
  rcl_interfaces::msg::SetParametersResult
  callback_dynamicParamChange(std::vector<rclcpp::Parameter> parameters);
#endif

  // ---- Diagnostic ----
#ifdef ZED_ROS2_AVAILABLE
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
#endif

  // ---- Service callbacks (retained subset) ----
#ifdef ZED_ROS2_AVAILABLE
  void callback_enableDepth(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
#endif

private:
  // ===========================================================
  //  Member variables -- NO sl:: types anywhere
  // ===========================================================

  // ---- Bridge handles ----
  std::unique_ptr<sl_oc_bridge::SlOcCamera> mSlCamera;
  ZedCalibration mCalib;

  uint64_t mFrameCount = 0;

  // ---- Topic name roots (same as upstream) ----
  std::string mTopicRoot = "~/";

  // Image topics
  std::string mLeftTopic;
  std::string mLeftRawTopic;
  std::string mRightTopic;
  std::string mRightRawTopic;
  std::string mRgbTopic;
  std::string mRgbRawTopic;
  std::string mStereoTopic;
  std::string mStereoRawTopic;

  // Depth topics
  std::string mDepthTopic;
  std::string mDepthInfoTopic;
  std::string mPointcloudTopic;

  // ---- Parameter variables (kept subset) ----
  // Debug
  bool mDebugMode = false;
  bool mDebugGrab = false;
  bool mDebugVideoDepth = false;
  bool mDebugSensors = false;

  // General
  std::string mCameraName = "zed";
  int mCamSerialNumber = 0;
  int mCamGrabFrameRate = 15;
  std::string mOpencvCalibFile;

  // Resolution
  int mCamWidth = 1280;
  int mCamHeight = 720;
  int mPubWidth = 0;              ///< 0 = same as grab
  int mPubHeight = 0;

  // Depth
  bool mDepthEnabled = true;
  double mCamMinDepth = 0.3;
  double mCamMaxDepth = 10.0;
  bool mOpenniDepthMode = false;

  // Point cloud
  bool mPublishPointcloud = true;
  double mPcPubRate = 10.0;
  int mPcStride = 2;             ///< Point cloud decimation stride (1 = full, 2 = skip every other pixel)

  // Sensor
  bool mPublishImu = true;
  double mSensPubRate = 200.0;

  // Image publishing flags (same semantics as upstream)
  bool mPublishImgLeftRight = false;
  bool mPublishImgRaw = false;
  bool mPublishImgRgb = true;
  bool mPublishImgStereo = false;
  bool mPublishDepthMap = true;

  // MLX depth inference parameters
  std::string mMlxModelPath;         ///< Path to .mlx depth model weights
  int mMlxInputWidth = 896;          ///< Model input width
  int mMlxInputHeight = 512;         ///< Model input height
  double mMlxConfidenceThreshold = 0.5;
  bool mMlxUseFp16 = true;           ///< Use FP16 for inference

  // ---- Dynamic params ----
  double mVdPubRate = 15.0;
  int mCamBrightness = 4;
  int mCamContrast = 4;
  int mCamExposure = 80;
  int mCamGain = 80;
  bool mCamAutoExpGain = true;

#ifdef ZED_ROS2_AVAILABLE
  // ---- QoS ----
  rclcpp::QoS mQos{10};

  // ---- Frame IDs ----
  std::string mBaseFrameId;
  std::string mCenterFrameId;
  std::string mLeftCamFrameId;
  std::string mLeftCamOptFrameId;
  std::string mRightCamFrameId;
  std::string mRightCamOptFrameId;
  std::string mImuFrameId;
  std::string mDepthFrameId;
  std::string mDepthOptFrameId;
  std::string mPointCloudFrameId;
  std::string mMapFrameId = "map";
  std::string mOdomFrameId = "odom";

  // ---- Camera info messages ----
  std::shared_ptr<sensor_msgs::msg::CameraInfo> mLeftCamInfoMsg;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> mRightCamInfoMsg;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> mLeftCamInfoRawMsg;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> mRightCamInfoRawMsg;

  // ---- Image publishers ----
  image_transport::Publisher mPubRgb;
  image_transport::Publisher mPubRawRgb;
  image_transport::Publisher mPubLeft;
  image_transport::Publisher mPubRawLeft;
  image_transport::Publisher mPubRight;
  image_transport::Publisher mPubRawRight;
  image_transport::Publisher mPubStereo;
  image_transport::Publisher mPubRawStereo;
  image_transport::Publisher mPubDepth;

  // ---- Camera info publishers ----
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubRgbCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubRawRgbCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubLeftCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubRawLeftCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubRightCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubRawRightCamInfo;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPubDepthCamInfo;

  // ---- Depth / Point cloud publishers ----
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPubCloud;

  // ---- IMU publisher ----
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mPubImu;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mPubImuTemp;

  // ---- Magnetometer publisher ----
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mPubMag;

  // ---- Barometer publisher ----
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr mPubBaro;

  // ---- Camera temperature publishers (left/right CMOS) ----
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mPubTempLeft;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mPubTempRight;

  // ---- TF ----
  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> mTfListener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mStaticTfBroadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

  // ---- Diagnostic ----
  diagnostic_updater::Updater mDiagUpdater{this};

  // ---- Services (kept subset) ----
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mEnableDepthSrv;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr mInitTimer;
#endif  // ZED_ROS2_AVAILABLE

  // ---- Frame buffers (cv::Mat, NOT sl::Mat) ----
  cv::Mat mMatLeft, mMatLeftRaw;
  cv::Mat mMatRight, mMatRightRaw;
  cv::Mat mMatDepth;                  ///< CV_32FC1 depth in metres
  cv::Mat mMatConfidence;             ///< CV_8UC1 confidence

  // ---- Threads & sync ----
  std::thread mGrabThread;
  std::thread mVdThread;
  std::thread mPcThread;
  std::thread mSensThread;
  std::atomic<bool> mThreadStop{false};

  std::mutex mVdMutex;
  std::condition_variable mVdDataReadyCondVar;
  std::atomic_bool mVdDataReady{false};

  std::mutex mPcMutex;
  std::condition_variable mPcDataReadyCondVar;
  std::atomic_bool mPcDataReady{false};

  // ---- Timestamps ----
#ifdef ZED_ROS2_AVAILABLE
  rclcpp::Time mFrameTimestamp;
#endif
  uint64_t mLastGrabTs_ns = 0;

  // ---- Sensor data & timestamps for rate limiting ----
  sl_oc_bridge::SensorsData mSensorsData;
  uint64_t mLastImuTs_ns = 0;
  uint64_t mLastMagTs_ns = 0;
  uint64_t mLastTempTs_ns = 0;
  uint64_t mLastBaroTs_ns = 0;

  // ---- Status flags ----
  bool mCameraOpen = false;
  bool mStaticTfPublished = false;  ///< True after static TFs have been broadcast
};

}  // namespace stereolabs

#endif  // ZED_CAMERA_MLX_COMPONENT_HPP_
