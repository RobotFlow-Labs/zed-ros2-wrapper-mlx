// Copyright 2025 AIFLOW LABS LIMITED
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

// ============================================================================
// sl_oc_bridge.cpp
//
// Implementation of the SlOcCamera class and Mat utilities.
// Wraps zed-sdk-mlx (zed-open-capture) VideoCapture and SensorCapture into
// an API that mirrors the sl::Camera interface used by the ZED ROS2 wrapper.
//
// Standalone C++17, no ROS2 dependencies.
// ============================================================================

// Include zed-open-capture headers BEFORE the bridge header so that the
// forward declarations in sl_oc_bridge.hpp are immediately completed by
// the full class definitions.
#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include "videocapture_def.hpp"
#include "sensorcapture_def.hpp"
#include "defines.hpp"

#include "sl_oc_bridge.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>

namespace sl_oc_bridge {

// ============================================================================
// Mat implementation
// ============================================================================

static size_t channelsFor(MAT_TYPE t) {
  switch (t) {
    case MAT_TYPE::F32_C1: return 1;
    case MAT_TYPE::F32_C2: return 2;
    case MAT_TYPE::F32_C3: return 3;
    case MAT_TYPE::F32_C4: return 4;
    case MAT_TYPE::U8_C1:  return 1;
    case MAT_TYPE::U8_C2:  return 2;
    case MAT_TYPE::U8_C3:  return 3;
    case MAT_TYPE::U8_C4:  return 4;
    case MAT_TYPE::U16_C1: return 1;
    default:               return 0;
  }
}

static size_t bytesPerChannel(MAT_TYPE t) {
  switch (t) {
    case MAT_TYPE::F32_C1:
    case MAT_TYPE::F32_C2:
    case MAT_TYPE::F32_C3:
    case MAT_TYPE::F32_C4:
      return 4;
    case MAT_TYPE::U8_C1:
    case MAT_TYPE::U8_C2:
    case MAT_TYPE::U8_C3:
    case MAT_TYPE::U8_C4:
      return 1;
    case MAT_TYPE::U16_C1:
      return 2;
    default:
      return 0;
  }
}

Mat::Mat() = default;

Mat::Mat(size_t width, size_t height, MAT_TYPE type, MEM /*mem*/) {
  alloc(width, height, type);
}

Mat::Mat(const Mat& other)
  : width_(other.width_), height_(other.height_), type_(other.type_),
    timestamp(other.timestamp)
{
  if (other.data_ && other.owns_data_) {
    size_t bytes = getStepBytes() * height_;
    data_ = new uint8_t[bytes];
    std::memcpy(data_, other.data_, bytes);
    owns_data_ = true;
  }
}

Mat& Mat::operator=(const Mat& other) {
  if (this != &other) {
    free();
    width_ = other.width_;
    height_ = other.height_;
    type_ = other.type_;
    timestamp = other.timestamp;
    if (other.data_ && other.owns_data_) {
      size_t bytes = getStepBytes() * height_;
      data_ = new uint8_t[bytes];
      std::memcpy(data_, other.data_, bytes);
      owns_data_ = true;
    }
  }
  return *this;
}

Mat::Mat(Mat&& other) noexcept
  : data_(other.data_), width_(other.width_), height_(other.height_),
    type_(other.type_), owns_data_(other.owns_data_), timestamp(other.timestamp)
{
  other.data_ = nullptr;
  other.owns_data_ = false;
  other.width_ = 0;
  other.height_ = 0;
}

Mat& Mat::operator=(Mat&& other) noexcept {
  if (this != &other) {
    free();
    data_ = other.data_;
    width_ = other.width_;
    height_ = other.height_;
    type_ = other.type_;
    owns_data_ = other.owns_data_;
    timestamp = other.timestamp;
    other.data_ = nullptr;
    other.owns_data_ = false;
    other.width_ = 0;
    other.height_ = 0;
  }
  return *this;
}

Mat::~Mat() {
  free();
}

void Mat::alloc(size_t width, size_t height, MAT_TYPE type, MEM /*mem*/) {
  free();
  width_ = width;
  height_ = height;
  type_ = type;
  size_t bytes = getStepBytes() * height_;
  if (bytes > 0) {
    data_ = new uint8_t[bytes];
    std::memset(data_, 0, bytes);
    owns_data_ = true;
  }
}

void Mat::free() {
  if (data_ && owns_data_) {
    delete[] data_;
  }
  data_ = nullptr;
  owns_data_ = false;
}

size_t Mat::getChannels() const { return channelsFor(type_); }
size_t Mat::getPixelBytes() const { return channelsFor(type_) * bytesPerChannel(type_); }
size_t Mat::getStepBytes() const { return width_ * getPixelBytes(); }

ERROR_CODE Mat::getValue(size_t x, size_t y, float* value, MEM /*mem*/) const {
  if (!data_ || x >= width_ || y >= height_) return ERROR_CODE::FAILURE;
  if (type_ != MAT_TYPE::F32_C1) return ERROR_CODE::FAILURE;
  const float* row = reinterpret_cast<const float*>(data_ + y * getStepBytes());
  *value = row[x];
  return ERROR_CODE::SUCCESS;
}

ERROR_CODE Mat::getValue(size_t x, size_t y, float4* value, MEM /*mem*/) const {
  if (!data_ || x >= width_ || y >= height_) return ERROR_CODE::FAILURE;
  if (type_ != MAT_TYPE::F32_C4) return ERROR_CODE::FAILURE;
  const float* row = reinterpret_cast<const float*>(data_ + y * getStepBytes());
  const float* px = row + x * 4;
  *value = float4(px[0], px[1], px[2], px[3]);
  return ERROR_CODE::SUCCESS;
}

ERROR_CODE Mat::setValue(size_t x, size_t y, float value, MEM /*mem*/) {
  if (!data_ || x >= width_ || y >= height_) return ERROR_CODE::FAILURE;
  if (type_ != MAT_TYPE::F32_C1) return ERROR_CODE::FAILURE;
  float* row = reinterpret_cast<float*>(data_ + y * getStepBytes());
  row[x] = value;
  return ERROR_CODE::SUCCESS;
}

ERROR_CODE Mat::setValue(size_t x, size_t y, const float4& value, MEM /*mem*/) {
  if (!data_ || x >= width_ || y >= height_) return ERROR_CODE::FAILURE;
  if (type_ != MAT_TYPE::F32_C4) return ERROR_CODE::FAILURE;
  float* row = reinterpret_cast<float*>(data_ + y * getStepBytes());
  float* px = row + x * 4;
  px[0] = value.x; px[1] = value.y; px[2] = value.z; px[3] = value.w;
  return ERROR_CODE::SUCCESS;
}

ERROR_CODE Mat::setFrom(const uint8_t* src, size_t step, size_t w, size_t h, MAT_TYPE type) {
  alloc(w, h, type);
  if (!data_) return ERROR_CODE::FAILURE;
  size_t row_bytes = std::min(step, getStepBytes());
  for (size_t r = 0; r < h; ++r) {
    std::memcpy(data_ + r * getStepBytes(), src + r * step, row_bytes);
  }
  return ERROR_CODE::SUCCESS;
}

ERROR_CODE Mat::setTo(int value, MEM /*mem*/) {
  if (!data_) return ERROR_CODE::FAILURE;
  std::memset(data_, value, getStepBytes() * height_);
  return ERROR_CODE::SUCCESS;
}

// ============================================================================
// SlOcCamera::Impl -- pImpl hiding platform details
// ============================================================================

struct SlOcCamera::Impl {
  // zed-open-capture objects
  std::unique_ptr<sl_oc::video::VideoCapture> video;
#ifdef SENSORS_MOD_AVAILABLE
  std::unique_ptr<sl_oc::sensors::SensorCapture> sensors;
#endif

  // State
  bool opened = false;
  InitParameters init_params;
  CameraInformation cam_info;
  Timestamp last_grab_ts;
  float measured_fps = 0.0f;

  // Last frame data (side-by-side YUV from VideoCapture)
  std::mutex frame_mutex;
  uint64_t last_frame_id = 0;

  // Pre-allocated Mat for left/right images
  Mat mat_left_bgra;
  Mat mat_right_bgra;
  Mat mat_left_raw;
  Mat mat_right_raw;

  // FPS measurement
  uint64_t fps_frame_count = 0;
  uint64_t fps_start_time_ns = 0;

  // ----> Helper: map our RESOLUTION_PRESET to sl_oc values
  sl_oc::video::RESOLUTION mapResolution(RESOLUTION_PRESET r) const {
    switch (r) {
      case RESOLUTION_PRESET::HD2K:   return sl_oc::video::RESOLUTION::HD2K;
      case RESOLUTION_PRESET::HD1080: return sl_oc::video::RESOLUTION::HD1080;
      case RESOLUTION_PRESET::HD720:  return sl_oc::video::RESOLUTION::HD720;
      case RESOLUTION_PRESET::VGA:    return sl_oc::video::RESOLUTION::VGA;
      case RESOLUTION_PRESET::AUTO:   return sl_oc::video::RESOLUTION::HD720;
      default:                        return sl_oc::video::RESOLUTION::HD720;
    }
  }

  sl_oc::video::FPS mapFPS(int fps) const {
    if (fps <= 15) return sl_oc::video::FPS::FPS_15;
    if (fps <= 30) return sl_oc::video::FPS::FPS_30;
    if (fps <= 60) return sl_oc::video::FPS::FPS_60;
    return sl_oc::video::FPS::FPS_100;
  }

  Resolution getResolutionSize(RESOLUTION_PRESET r) const {
    switch (r) {
      case RESOLUTION_PRESET::HD2K:   return Resolution(2208, 1242);
      case RESOLUTION_PRESET::HD1080: return Resolution(1920, 1080);
      case RESOLUTION_PRESET::HD720:  return Resolution(1280, 720);
      case RESOLUTION_PRESET::VGA:    return Resolution(672, 376);
      case RESOLUTION_PRESET::AUTO:   return Resolution(1280, 720);
      default:                        return Resolution(1280, 720);
    }
  }

  MODEL mapDeviceToModel(sl_oc::video::SL_DEVICE dev) const {
    switch (dev) {
      case sl_oc::video::SL_DEVICE::ZED:
      case sl_oc::video::SL_DEVICE::ZED_CBS:
        return MODEL::ZED;
      case sl_oc::video::SL_DEVICE::ZED_M:
      case sl_oc::video::SL_DEVICE::ZED_M_CBS:
        return MODEL::ZED_M;
      case sl_oc::video::SL_DEVICE::ZED_2:
        return MODEL::ZED2;
      case sl_oc::video::SL_DEVICE::ZED_2i:
        return MODEL::ZED2i;
      default:
        return MODEL::ZED2i;
    }
  }

  // Simple YUV 4:2:2 (YUYV) to BGRA conversion for a single half-frame
  // src points to YUYV data, dst is BGRA, w/h is single-eye dimensions
  void yuyv_to_bgra(const uint8_t* src, uint8_t* dst,
                     size_t w, size_t h, size_t src_stride) {
    for (size_t row = 0; row < h; ++row) {
      const uint8_t* s = src + row * src_stride;
      uint8_t* d = dst + row * w * 4;
      for (size_t col = 0; col < w; col += 2) {
        int y0 = s[0];
        int u  = s[1];
        int y1 = s[2];
        int v  = s[3];
        s += 4;

        int c0 = y0 - 16;
        int c1 = y1 - 16;
        int d_u = u - 128;
        int d_v = v - 128;

        auto clamp = [](int val) -> uint8_t {
          return static_cast<uint8_t>(std::max(0, std::min(255, val)));
        };

        // Pixel 0 (BGRA)
        d[0] = clamp((298 * c0 + 516 * d_u + 128) >> 8);           // B
        d[1] = clamp((298 * c0 - 100 * d_u - 208 * d_v + 128) >> 8); // G
        d[2] = clamp((298 * c0 + 409 * d_v + 128) >> 8);           // R
        d[3] = 255;                                                  // A
        d += 4;

        // Pixel 1 (BGRA)
        d[0] = clamp((298 * c1 + 516 * d_u + 128) >> 8);
        d[1] = clamp((298 * c1 - 100 * d_u - 208 * d_v + 128) >> 8);
        d[2] = clamp((298 * c1 + 409 * d_v + 128) >> 8);
        d[3] = 255;
        d += 4;
      }
    }
  }
};

// ============================================================================
// SlOcCamera lifecycle
// ============================================================================

SlOcCamera::SlOcCamera() : pImpl_(std::make_unique<Impl>()) {}

SlOcCamera::~SlOcCamera() {
  close();
}

ERROR_CODE SlOcCamera::open(const InitParameters& params) {
  if (pImpl_->opened) {
    return ERROR_CODE::CAMERA_ALREADY_IN_USE;
  }

  pImpl_->init_params = params;

  // Validate SVO (not supported)
  if (!params.svo_input_filename.empty()) {
    std::cerr << "[sl_oc_bridge] SVO input is not supported on macOS." << std::endl;
    return ERROR_CODE::NOT_SUPPORTED;
  }

  // Create VideoParams from InitParameters
  sl_oc::video::VideoParams vparams;
  vparams.res = pImpl_->mapResolution(params.camera_resolution);
  vparams.fps = pImpl_->mapFPS(params.camera_fps);
  vparams.verbose = params.sdk_verbose;

  // Create and initialize VideoCapture
  pImpl_->video = std::make_unique<sl_oc::video::VideoCapture>(vparams);
  if (!pImpl_->video->initializeVideo(params.camera_device_id)) {
    std::cerr << "[sl_oc_bridge] Failed to initialize video capture." << std::endl;
    pImpl_->video.reset();
    return ERROR_CODE::CAMERA_NOT_DETECTED;
  }

  // Get actual frame size
  int fw = 0, fh = 0;
  pImpl_->video->getFrameSize(fw, fh);
  // The frame is side-by-side, so single-eye width = fw/2
  size_t eye_w = static_cast<size_t>(fw) / 2;
  size_t eye_h = static_cast<size_t>(fh);

  // Populate camera information
  pImpl_->cam_info.serial_number = pImpl_->video->getSerialNumber();
  pImpl_->cam_info.camera_resolution = Resolution(eye_w, eye_h);
  pImpl_->cam_info.camera_fps = static_cast<float>(params.camera_fps);
  // Camera model detection would need USB PID checking; default to ZED2i
  pImpl_->cam_info.camera_model = MODEL::ZED2i;

  // Set up calibration placeholder (actual calibration should be loaded
  // from Stereolabs servers or local .conf file -- future work)
  auto& cal = pImpl_->cam_info.camera_configuration;
  cal.left_cam.image_size = Resolution(eye_w, eye_h);
  cal.right_cam.image_size = Resolution(eye_w, eye_h);
  // Default focal length estimate for HD720 (placeholder)
  float default_fx = static_cast<float>(eye_w) * 0.5f / std::tan(55.0f * 3.14159f / 180.0f / 2.0f);
  cal.left_cam.fx = cal.left_cam.fy = default_fx;
  cal.left_cam.cx = static_cast<float>(eye_w) / 2.0f;
  cal.left_cam.cy = static_cast<float>(eye_h) / 2.0f;
  cal.right_cam.fx = cal.right_cam.fy = default_fx;
  cal.right_cam.cx = static_cast<float>(eye_w) / 2.0f;
  cal.right_cam.cy = static_cast<float>(eye_h) / 2.0f;
  cal.baseline = 0.12f;  // 120mm default baseline (ZED2)

  // Copy to rectified and raw params
  cal.left_cam_rectified = cal.left_cam;
  cal.right_cam_rectified = cal.right_cam;
  pImpl_->cam_info.calibration_parameters = cal;
  pImpl_->cam_info.calibration_parameters_raw = cal;

#ifdef SENSORS_MOD_AVAILABLE
  // Initialize sensor capture
  sl_oc::VERBOSITY sens_verb = static_cast<sl_oc::VERBOSITY>(
    std::min(params.sdk_verbose, static_cast<int>(sl_oc::VERBOSITY::INFO)));
  pImpl_->sensors = std::make_unique<sl_oc::sensors::SensorCapture>(sens_verb);

  int sn = pImpl_->cam_info.serial_number;
  if (!pImpl_->sensors->initializeSensors(sn)) {
    std::cerr << "[sl_oc_bridge] WARNING: Failed to initialize sensor capture "
              << "(IMU/mag/temp will not be available)." << std::endl;
    pImpl_->sensors.reset();
  } else {
    // Get firmware version
    uint16_t fw_major = 0, fw_minor = 0;
    pImpl_->sensors->getFirmwareVersion(fw_major, fw_minor);
    pImpl_->cam_info.sensors_firmware_version =
      static_cast<unsigned int>(fw_major) * 256 + fw_minor;

    // Enable sync between video and sensors
    pImpl_->video->enableSensorSync(pImpl_->sensors.get());
  }
#endif

  // Pre-allocate image mats
  pImpl_->mat_left_bgra.alloc(eye_w, eye_h, MAT_TYPE::U8_C4);
  pImpl_->mat_right_bgra.alloc(eye_w, eye_h, MAT_TYPE::U8_C4);

  pImpl_->opened = true;
  pImpl_->fps_start_time_ns = getSteadyTimestamp();
  pImpl_->fps_frame_count = 0;

  std::cout << "[sl_oc_bridge] Camera opened successfully." << std::endl;
  std::cout << "[sl_oc_bridge]   Serial: " << pImpl_->cam_info.serial_number << std::endl;
  std::cout << "[sl_oc_bridge]   Resolution: " << eye_w << "x" << eye_h << std::endl;
  std::cout << "[sl_oc_bridge]   FPS: " << params.camera_fps << std::endl;

  return ERROR_CODE::SUCCESS;
}

void SlOcCamera::close() {
  if (!pImpl_->opened) return;

#ifdef SENSORS_MOD_AVAILABLE
  pImpl_->sensors.reset();
#endif
  pImpl_->video.reset();
  pImpl_->opened = false;

  std::cout << "[sl_oc_bridge] Camera closed." << std::endl;
}

bool SlOcCamera::isOpened() const {
  return pImpl_->opened;
}

// ============================================================================
// Frame acquisition
// ============================================================================

ERROR_CODE SlOcCamera::grab(const RuntimeParameters& /*params*/) {
  if (!pImpl_->opened || !pImpl_->video) {
    return ERROR_CODE::CAMERA_NOT_INITIALIZED;
  }

  // Get frame from VideoCapture (blocking with timeout)
  const sl_oc::video::Frame& frame = pImpl_->video->getLastFrame(100);

  if (frame.data == nullptr) {
    return ERROR_CODE::CORRUPTED_FRAME;
  }

  // Check if this is actually a new frame
  if (frame.frame_id == pImpl_->last_frame_id && pImpl_->last_frame_id != 0) {
    // No new frame yet; not an error, just try again
    return ERROR_CODE::CORRUPTED_FRAME;
  }

  pImpl_->last_frame_id = frame.frame_id;
  pImpl_->last_grab_ts = Timestamp(frame.timestamp);

  // Convert side-by-side YUYV frame to two BGRA images
  size_t eye_w = pImpl_->cam_info.camera_resolution.width;
  size_t eye_h = pImpl_->cam_info.camera_resolution.height;
  // Side-by-side frame: total width = eye_w * 2, YUYV = 2 bytes/pixel
  size_t sbs_stride = eye_w * 2 * 2;  // full width * 2 bytes per YUYV pixel

  {
    std::lock_guard<std::mutex> lock(pImpl_->frame_mutex);

    // Left eye: first eye_w pixels of each row
    pImpl_->yuyv_to_bgra(
      frame.data, pImpl_->mat_left_bgra.getPtr(),
      eye_w, eye_h, sbs_stride);

    // Right eye: second eye_w pixels (offset by eye_w*2 bytes per row)
    pImpl_->yuyv_to_bgra(
      frame.data + eye_w * 2,  // offset to right half in YUYV
      pImpl_->mat_right_bgra.getPtr(),
      eye_w, eye_h, sbs_stride);

    pImpl_->mat_left_bgra.timestamp = pImpl_->last_grab_ts;
    pImpl_->mat_right_bgra.timestamp = pImpl_->last_grab_ts;
  }

  // Update FPS measurement
  pImpl_->fps_frame_count++;
  uint64_t now = getSteadyTimestamp();
  double elapsed = static_cast<double>(now - pImpl_->fps_start_time_ns) / 1e9;
  if (elapsed >= 1.0) {
    pImpl_->measured_fps = static_cast<float>(pImpl_->fps_frame_count) / static_cast<float>(elapsed);
    pImpl_->fps_frame_count = 0;
    pImpl_->fps_start_time_ns = now;
  }

  return ERROR_CODE::SUCCESS;
}

ERROR_CODE SlOcCamera::retrieveImage(Mat& mat, int view, MEM /*mem*/, Resolution resolution) {
  if (!pImpl_->opened) {
    return ERROR_CODE::CAMERA_NOT_INITIALIZED;
  }

  std::lock_guard<std::mutex> lock(pImpl_->frame_mutex);

  const Mat* src = nullptr;

  // view: 0=LEFT, 1=RIGHT, 2=LEFT_UNRECTIFIED, 3=RIGHT_UNRECTIFIED
  switch (view) {
    case 0:  // LEFT (same as unrectified for now -- rectification is future work)
    case 2:  // LEFT_UNRECTIFIED
      src = &pImpl_->mat_left_bgra;
      break;
    case 1:  // RIGHT
    case 3:  // RIGHT_UNRECTIFIED
      src = &pImpl_->mat_right_bgra;
      break;
    default:
      return ERROR_CODE::FAILURE;
  }

  if (!src->isInit()) {
    return ERROR_CODE::CORRUPTED_FRAME;
  }

  // If resolution requested is different, we would need to resize.
  // For now, just copy at native resolution.
  if (resolution.width > 0 && resolution.height > 0 &&
      (resolution.width != src->getWidth() || resolution.height != src->getHeight())) {
    // TODO: implement resize (OpenCV or manual bilinear)
    std::cerr << "[sl_oc_bridge] Image resize not yet implemented; returning native resolution."
              << std::endl;
  }

  mat = *src;  // copy
  return ERROR_CODE::SUCCESS;
}

// ============================================================================
// Camera information
// ============================================================================

CameraInformation SlOcCamera::getCameraInformation(Resolution /*image_size*/) const {
  return pImpl_->cam_info;
}

MODEL SlOcCamera::getCameraModel() const {
  return pImpl_->cam_info.camera_model;
}

int SlOcCamera::getSerialNumber() const {
  return pImpl_->cam_info.serial_number;
}

Resolution SlOcCamera::getCameraResolution() const {
  return pImpl_->cam_info.camera_resolution;
}

float SlOcCamera::getCurrentFPS() const {
  return pImpl_->measured_fps;
}

Timestamp SlOcCamera::getTimestamp() const {
  return pImpl_->last_grab_ts;
}

// ============================================================================
// Sensor data
// ============================================================================

ERROR_CODE SlOcCamera::getSensorsData(SensorsData& data, int /*time_reference*/) {
  if (!pImpl_->opened) {
    return ERROR_CODE::CAMERA_NOT_INITIALIZED;
  }

#ifdef SENSORS_MOD_AVAILABLE
  if (!pImpl_->sensors) {
    // Sensors not available but camera is open -- return empty data
    data = SensorsData{};
    return ERROR_CODE::SUCCESS;
  }

  // --- IMU ---
  const auto& imu_raw = pImpl_->sensors->getLastIMUData(1500);
  if (imu_raw.valid != sl_oc::sensors::data::Imu::NOT_PRESENT) {
    data.imu.is_available = true;
    data.imu.timestamp = Timestamp(imu_raw.timestamp);
    data.imu.linear_acceleration = float3(imu_raw.aX, imu_raw.aY, imu_raw.aZ);
    data.imu.angular_velocity = float3(imu_raw.gX, imu_raw.gY, imu_raw.gZ);
    data.imu.temperature = imu_raw.temp;
  } else {
    data.imu.is_available = false;
  }

  // --- Magnetometer ---
  const auto& mag_raw = pImpl_->sensors->getLastMagnetometerData(100);
  if (mag_raw.valid != sl_oc::sensors::data::Magnetometer::NOT_PRESENT) {
    data.magnetometer.is_available = true;
    data.magnetometer.timestamp = Timestamp(mag_raw.timestamp);
    data.magnetometer.magnetic_field_calibrated = float3(mag_raw.mX, mag_raw.mY, mag_raw.mZ);
  } else {
    data.magnetometer.is_available = false;
  }

  // --- Temperature ---
  const auto& temp_raw = pImpl_->sensors->getLastCameraTemperatureData(100);
  if (temp_raw.valid != sl_oc::sensors::data::Temperature::NOT_PRESENT) {
    data.temperature.is_available = true;
    data.temperature.timestamp = Timestamp(temp_raw.timestamp);
    data.temperature.temperature_left = temp_raw.temp_left;
    data.temperature.temperature_right = temp_raw.temp_right;
  } else {
    data.temperature.is_available = false;
  }

  // IMU temperature is separate from camera sensor temps
  data.temperature.temperature_imu = data.imu.temperature;

  // --- Environment (barometer) ---
  const auto& env_raw = pImpl_->sensors->getLastEnvironmentData(100);
  if (env_raw.valid != sl_oc::sensors::data::Environment::NOT_PRESENT) {
    data.barometer.is_available = true;
    data.barometer.timestamp = Timestamp(env_raw.timestamp);
    data.barometer.pressure = env_raw.press;
  } else {
    data.barometer.is_available = false;
  }

  return ERROR_CODE::SUCCESS;
#else
  // No sensor module compiled
  data = SensorsData{};
  return ERROR_CODE::SUCCESS;
#endif
}

// ============================================================================
// Camera settings (delegated to VideoCapture)
// ============================================================================

int SlOcCamera::setBrightness(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setBrightness(value);
  return 0;
}

int SlOcCamera::getBrightness() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getBrightness();
}

int SlOcCamera::setContrast(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setContrast(value);
  return 0;
}

int SlOcCamera::getContrast() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getContrast();
}

int SlOcCamera::setHue(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setHue(value);
  return 0;
}

int SlOcCamera::getHue() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getHue();
}

int SlOcCamera::setSaturation(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setSaturation(value);
  return 0;
}

int SlOcCamera::getSaturation() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getSaturation();
}

int SlOcCamera::setSharpness(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setSharpness(value);
  return 0;
}

int SlOcCamera::getSharpness() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getSharpness();
}

int SlOcCamera::setGamma(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setGamma(value);
  return 0;
}

int SlOcCamera::getGamma() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getGamma();
}

int SlOcCamera::setGain(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setGain(sl_oc::video::CAM_SENS_POS::LEFT, value);
  pImpl_->video->setGain(sl_oc::video::CAM_SENS_POS::RIGHT, value);
  return 0;
}

int SlOcCamera::getGain() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getGain(sl_oc::video::CAM_SENS_POS::LEFT);
}

int SlOcCamera::setExposure(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setExposure(sl_oc::video::CAM_SENS_POS::LEFT, value);
  pImpl_->video->setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, value);
  return 0;
}

int SlOcCamera::getExposure() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getExposure(sl_oc::video::CAM_SENS_POS::LEFT);
}

int SlOcCamera::setWhiteBalance(int value) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setWhiteBalance(value);
  return 0;
}

int SlOcCamera::getWhiteBalance() const {
  if (!pImpl_->video) return -1;
  return pImpl_->video->getWhiteBalance();
}

int SlOcCamera::setAutoExposureGain(bool enable) {
  if (!pImpl_->video) return -1;
  return pImpl_->video->setAECAGC(enable);
}

bool SlOcCamera::getAutoExposureGain() const {
  if (!pImpl_->video) return false;
  return pImpl_->video->getAECAGC();
}

int SlOcCamera::setAutoWhiteBalance(bool enable) {
  if (!pImpl_->video) return -1;
  pImpl_->video->setAutoWhiteBalance(enable);
  return 0;
}

bool SlOcCamera::getAutoWhiteBalance() const {
  if (!pImpl_->video) return false;
  return pImpl_->video->getAutoWhiteBalance();
}

int SlOcCamera::setLED(bool enable) {
  if (!pImpl_->video) return -1;
  return pImpl_->video->setLEDstatus(enable);
}

bool SlOcCamera::getLED() const {
  if (!pImpl_->video) return false;
  bool status = false;
  pImpl_->video->getLEDstatus(&status);
  return status;
}

}  // namespace sl_oc_bridge
