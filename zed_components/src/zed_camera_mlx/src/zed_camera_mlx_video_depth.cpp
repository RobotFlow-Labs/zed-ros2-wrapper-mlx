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
// zed_camera_mlx_video_depth.cpp
//
// Video/Depth publishing routines for the macOS MLX port.
//
// This file is the stripped counterpart of
//   zed_camera_component_video_depth.cpp
// with all sl::Mat replaced by cv::Mat, sl::Camera::retrieveImage replaced
// by direct frame buffer access, and sl::Camera::retrieveMeasure(DEPTH)
// replaced by the MLX depth inference result.
//
// Every section that needs implementation is marked with:
//   // TODO(mlx-port): <description>
// ===========================================================================

#include "zed_camera_mlx_component.hpp"

#include <opencv2/imgproc.hpp>

#ifdef ZED_ROS2_AVAILABLE
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_transport/camera_common.hpp>
#endif

namespace stereolabs
{

// ===========================================================================
// publishImageWithInfo  --  publish a cv::Mat as a ROS Image + CameraInfo
// ===========================================================================

void ZedCameraMlx::publishImageWithInfo(
  const cv::Mat & img,
  const std::string & encoding,
#ifdef ZED_ROS2_AVAILABLE
  image_transport::Publisher & pubImg,
#endif
  const std::string & frameId)
{
#ifdef ZED_ROS2_AVAILABLE
  if (img.empty()) {
    return;
  }

  // TODO(mlx-port): Check subscriber count before constructing message.
  //   if (pubImg.getNumSubscribers() == 0) return;

  // ---- Build sensor_msgs::msg::Image from cv::Mat ----
  auto imgMsg = std::make_unique<sensor_msgs::msg::Image>();

  imgMsg->header.stamp = mFrameTimestamp;
  imgMsg->header.frame_id = frameId;

  imgMsg->height = static_cast<uint32_t>(img.rows);
  imgMsg->width = static_cast<uint32_t>(img.cols);
  imgMsg->encoding = encoding;
  imgMsg->is_bigendian = false;

  // Step = bytes per row
  imgMsg->step = static_cast<uint32_t>(img.step[0]);

  // Copy pixel data
  size_t data_size = imgMsg->step * imgMsg->height;
  imgMsg->data.resize(data_size);
  std::memcpy(imgMsg->data.data(), img.data, data_size);

  // Publish
  pubImg.publish(std::move(imgMsg));

#else
  (void)img;
  (void)encoding;
  (void)frameId;
#endif
}

// ===========================================================================
// publishDepthMapWithInfo  --  publish depth as a ROS Image
// ===========================================================================

void ZedCameraMlx::publishDepthMapWithInfo(const cv::Mat & depth)
{
#ifdef ZED_ROS2_AVAILABLE
  if (depth.empty()) {
    return;
  }

  auto depthMsg = std::make_unique<sensor_msgs::msg::Image>();

  depthMsg->header.stamp = mFrameTimestamp;
  depthMsg->header.frame_id = mDepthOptFrameId;

  depthMsg->height = static_cast<uint32_t>(depth.rows);
  depthMsg->width = static_cast<uint32_t>(depth.cols);

  if (mOpenniDepthMode) {
    // OpenNI mode: 16-bit unsigned, millimetres
    // Convert from float metres to uint16 millimetres
    cv::Mat depth_mm;
    depth.convertTo(depth_mm, CV_16UC1, 1000.0);
    depthMsg->encoding = sensor_msgs::image_encodings::MONO16;
    depthMsg->step = static_cast<uint32_t>(depth_mm.step[0]);
    size_t data_size = depthMsg->step * depthMsg->height;
    depthMsg->data.resize(data_size);
    std::memcpy(depthMsg->data.data(), depth_mm.data, data_size);
  } else {
    // Standard mode: 32-bit float, metres
    depthMsg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depthMsg->step = static_cast<uint32_t>(depth.step[0]);
    size_t data_size = depthMsg->step * depthMsg->height;
    depthMsg->data.resize(data_size);
    std::memcpy(depthMsg->data.data(), depth.data, data_size);
  }

  depthMsg->is_bigendian = false;

  mPubDepth.publish(std::move(depthMsg));

  // Depth CameraInfo is published from threadFunc_videoDepthElab() alongside
  // the depth image publish call, using mPubDepthCamInfo.

#else
  (void)depth;
#endif
}

// ===========================================================================
// publishPointCloud  --  build PointCloud2 from depth + RGB
// ===========================================================================

void ZedCameraMlx::publishPointCloud(
  const cv::Mat & depth,
  const cv::Mat & rgb)
{
#ifdef ZED_ROS2_AVAILABLE
  if (depth.empty() || rgb.empty()) {
    return;
  }

  if (!mPubCloud) {
    return;
  }

  // TODO(mlx-port): Check subscriber count.
  //   if (count_subscribers(mPubCloud->get_topic_name()) == 0) return;

  // ---- Build PointCloud2 message with stride-based decimation ----
  const int stride = std::max(mPcStride, 1);
  const int pc_rows = (depth.rows + stride - 1) / stride;
  const int pc_cols = (depth.cols + stride - 1) / stride;

  auto pcMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  pcMsg->header.stamp = mFrameTimestamp;
  pcMsg->header.frame_id = mPointCloudFrameId;

  pcMsg->height = static_cast<uint32_t>(pc_rows);
  pcMsg->width = static_cast<uint32_t>(pc_cols);
  pcMsg->is_dense = false;
  pcMsg->is_bigendian = false;

  // Define fields: x, y, z, rgb
  sensor_msgs::PointCloud2Modifier modifier(*pcMsg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(static_cast<size_t>(pc_rows) * pc_cols);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pcMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pcMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pcMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*pcMsg, "rgb");

  // Use calibration intrinsics for reprojection; fall back to defaults
  // if calibration was not loaded (fx == 0).
  const double fx = (mCalib.left_fx > 0.0) ? mCalib.left_fx : 700.0;
  const double fy = (mCalib.left_fy > 0.0) ? mCalib.left_fy : 700.0;
  const double cx = (mCalib.left_cx > 0.0) ? mCalib.left_cx
                    : static_cast<double>(depth.cols) / 2.0;
  const double cy = (mCalib.left_cy > 0.0) ? mCalib.left_cy
                    : static_cast<double>(depth.rows) / 2.0;

  // Ensure RGB is BGR8 for iteration
  cv::Mat rgb_bgr;
  if (rgb.channels() == 4) {
    cv::cvtColor(rgb, rgb_bgr, cv::COLOR_BGRA2BGR);
  } else {
    rgb_bgr = rgb;
  }

  for (int v = 0; v < depth.rows; v += stride) {
    const float * depth_row = depth.ptr<float>(v);
    const uint8_t * rgb_row = rgb_bgr.ptr<uint8_t>(v);

    for (int u = 0; u < depth.cols; u += stride,
      ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      float d = depth_row[u];

      if (d <= 0.0f || std::isnan(d) || std::isinf(d)) {
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
      } else {
        // Reproject to 3D (optical frame: X-right, Y-down, Z-forward)
        *iter_x = static_cast<float>((u - cx) * d / fx);
        *iter_y = static_cast<float>((v - cy) * d / fy);
        *iter_z = d;
      }

      // Pack BGR into RGB bytes (same as PCL XYZRGB convention)
      int idx = u * 3;
      iter_rgb[0] = rgb_row[idx + 2];  // R
      iter_rgb[1] = rgb_row[idx + 1];  // G
      iter_rgb[2] = rgb_row[idx + 0];  // B
    }
  }

  mPubCloud->publish(std::move(pcMsg));

#else
  (void)depth;
  (void)rgb;
#endif
}

// ===========================================================================
// publishImuData  --  publish IMU sample as sensor_msgs/Imu
// ===========================================================================

void ZedCameraMlx::publishImuData(const ImuSample & sample)
{
#ifdef ZED_ROS2_AVAILABLE
  if (!mPubImu) {
    return;
  }

  auto imuMsg = std::make_unique<sensor_msgs::msg::Imu>();

  // Convert sensor timestamp (nanoseconds) to ROS time
  uint64_t ts_ns = static_cast<uint64_t>(sample.timestamp_ns);
  imuMsg->header.stamp = rclcpp::Time(
    static_cast<int32_t>(ts_ns / 1000000000ULL),
    static_cast<uint32_t>(ts_ns % 1000000000ULL));
  imuMsg->header.frame_id = mImuFrameId;

  // Linear acceleration (m/s^2)
  imuMsg->linear_acceleration.x = sample.ax;
  imuMsg->linear_acceleration.y = sample.ay;
  imuMsg->linear_acceleration.z = sample.az;

  // Angular velocity (rad/s)
  imuMsg->angular_velocity.x = sample.gx;
  imuMsg->angular_velocity.y = sample.gy;
  imuMsg->angular_velocity.z = sample.gz;

  // TODO(mlx-port): Fill orientation quaternion.
  //   The ZED SDK provides a fused orientation; without it, we set
  //   the orientation to identity and mark covariance as unknown (-1).
  imuMsg->orientation.w = 1.0;
  imuMsg->orientation.x = 0.0;
  imuMsg->orientation.y = 0.0;
  imuMsg->orientation.z = 0.0;
  imuMsg->orientation_covariance[0] = -1.0;  // unknown

  // TODO(mlx-port): Set proper covariance matrices from sensor specs.
  //   For now, use -1 to indicate "unknown".
  imuMsg->linear_acceleration_covariance[0] = -1.0;
  imuMsg->angular_velocity_covariance[0] = -1.0;

  mPubImu->publish(std::move(imuMsg));

  // ---- IMU Temperature ----
  if (mPubImuTemp) {
    auto tempMsg = std::make_unique<sensor_msgs::msg::Temperature>();
    tempMsg->header.stamp = rclcpp::Time(
      static_cast<int32_t>(ts_ns / 1000000000ULL),
      static_cast<uint32_t>(ts_ns % 1000000000ULL));
    tempMsg->header.frame_id = mImuFrameId;
    tempMsg->temperature = sample.temperature_c;
    tempMsg->variance = 0.0;
    mPubImuTemp->publish(std::move(tempMsg));
  }

#else
  (void)sample;
#endif
}

// ===========================================================================
// publishMagnetometerData  --  publish magnetometer as sensor_msgs/MagneticField
// ===========================================================================

void ZedCameraMlx::publishMagnetometerData(
  const sl_oc_bridge::MagnetometerData & mag_data)
{
#ifdef ZED_ROS2_AVAILABLE
  if (!mPubMag) {
    return;
  }

  auto magMsg = std::make_unique<sensor_msgs::msg::MagneticField>();

  // Convert sensor timestamp (nanoseconds) to ROS time
  uint64_t ts_ns = mag_data.timestamp.getNanoseconds();
  magMsg->header.stamp = rclcpp::Time(
    static_cast<int32_t>(ts_ns / 1000000000ULL),
    static_cast<uint32_t>(ts_ns % 1000000000ULL));
  magMsg->header.frame_id = mImuFrameId;

  // Magnetic field in Tesla (bridge provides microtesla, convert: 1 uT = 1e-6 T)
  magMsg->magnetic_field.x = mag_data.magnetic_field_calibrated.x * 1e-6;
  magMsg->magnetic_field.y = mag_data.magnetic_field_calibrated.y * 1e-6;
  magMsg->magnetic_field.z = mag_data.magnetic_field_calibrated.z * 1e-6;

  // Covariance unknown
  magMsg->magnetic_field_covariance[0] = 0.0;

  mPubMag->publish(std::move(magMsg));

#else
  (void)mag_data;
#endif
}

// ===========================================================================
// publishTemperatureData  --  publish camera CMOS temperatures
// ===========================================================================

void ZedCameraMlx::publishTemperatureData(
  const sl_oc_bridge::TemperatureData & temp_data)
{
#ifdef ZED_ROS2_AVAILABLE
  uint64_t ts_ns = temp_data.timestamp.getNanoseconds();
  auto stamp = rclcpp::Time(
    static_cast<int32_t>(ts_ns / 1000000000ULL),
    static_cast<uint32_t>(ts_ns % 1000000000ULL));

  // Left CMOS temperature
  if (mPubTempLeft) {
    auto tempMsg = std::make_unique<sensor_msgs::msg::Temperature>();
    tempMsg->header.stamp = stamp;
    tempMsg->header.frame_id = mLeftCamFrameId;
    tempMsg->temperature = temp_data.temperature_left;
    tempMsg->variance = 0.0;
    mPubTempLeft->publish(std::move(tempMsg));
  }

  // Right CMOS temperature
  if (mPubTempRight) {
    auto tempMsg = std::make_unique<sensor_msgs::msg::Temperature>();
    tempMsg->header.stamp = stamp;
    tempMsg->header.frame_id = mRightCamFrameId;
    tempMsg->temperature = temp_data.temperature_right;
    tempMsg->variance = 0.0;
    mPubTempRight->publish(std::move(tempMsg));
  }

#else
  (void)temp_data;
#endif
}

// ===========================================================================
// publishBarometerData  --  publish barometer as sensor_msgs/FluidPressure
// ===========================================================================

void ZedCameraMlx::publishBarometerData(
  const sl_oc_bridge::BarometerData & baro_data)
{
#ifdef ZED_ROS2_AVAILABLE
  if (!mPubBaro) {
    return;
  }

  auto baroMsg = std::make_unique<sensor_msgs::msg::FluidPressure>();

  // Convert sensor timestamp (nanoseconds) to ROS time
  uint64_t ts_ns = baro_data.timestamp.getNanoseconds();
  baroMsg->header.stamp = rclcpp::Time(
    static_cast<int32_t>(ts_ns / 1000000000ULL),
    static_cast<uint32_t>(ts_ns % 1000000000ULL));
  baroMsg->header.frame_id = mImuFrameId;

  // Pressure in Pascals (bridge provides hPa, convert: 1 hPa = 100 Pa)
  baroMsg->fluid_pressure = baro_data.pressure * 100.0;
  baroMsg->variance = 0.0;

  mPubBaro->publish(std::move(baroMsg));

#else
  (void)baro_data;
#endif
}

}  // namespace stereolabs
