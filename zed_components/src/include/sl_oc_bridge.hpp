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

#ifndef SL_OC_BRIDGE_HPP_
#define SL_OC_BRIDGE_HPP_

// ============================================================================
// sl_oc_bridge.hpp
//
// Compatibility shim that maps the minimum set of sl:: types used by the
// ZED ROS2 wrapper to the open-source zed-sdk-mlx (zed-open-capture) library.
//
// This file provides:
//   - sl_oc_bridge::ERROR_CODE, MODEL, RESOLUTION (mirroring sl:: enums)
//   - sl_oc_bridge::Timestamp, Resolution, Mat (lightweight replacements)
//   - sl_oc_bridge::InitParameters (maps to sl_oc::video::VideoParams)
//   - sl_oc_bridge::CameraInformation, CalibrationParameters
//   - sl_oc_bridge::SensorData (IMU, magnetometer, temperature)
//   - SlOcCamera class wrapping VideoCapture + SensorCapture
//
// The bridge is pure C++17 with NO ROS2 dependencies.
// ============================================================================

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <atomic>
#include <mutex>

#include "macos_unsupported.hpp"

// Include zed-open-capture headers directly. The forward-declaration approach
// caused incomplete-type errors because enum class forward declarations require
// matching underlying types, and the compiler treated them as shadowing the
// real definitions.
#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include "videocapture_def.hpp"
#include "sensorcapture_def.hpp"

namespace sl_oc_bridge {

// ============================================================================
// Error codes (subset of sl::ERROR_CODE used by the wrapper)
// ============================================================================
enum class ERROR_CODE : int32_t {
  SUCCESS = 0,
  FAILURE = -1,
  NO_GPU_COMPATIBLE = -2,          // mapped to "no CUDA" on macOS
  NOT_SUPPORTED = -3,              // feature requires proprietary SDK
  CAMERA_NOT_DETECTED = -4,
  CAMERA_NOT_INITIALIZED = -5,
  INVALID_RESOLUTION = -6,
  CORRUPTED_FRAME = -7,
  CAMERA_ALREADY_IN_USE = -8,
  END_OF_SVOFILE_REACHED = -9,    // SVO not supported
  LAST
};

inline const char* toString(ERROR_CODE ec) {
  switch (ec) {
    case ERROR_CODE::SUCCESS:                return "SUCCESS";
    case ERROR_CODE::FAILURE:                return "FAILURE";
    case ERROR_CODE::NO_GPU_COMPATIBLE:      return "NO_GPU_COMPATIBLE";
    case ERROR_CODE::NOT_SUPPORTED:          return "NOT_SUPPORTED";
    case ERROR_CODE::CAMERA_NOT_DETECTED:    return "CAMERA_NOT_DETECTED";
    case ERROR_CODE::CAMERA_NOT_INITIALIZED: return "CAMERA_NOT_INITIALIZED";
    case ERROR_CODE::INVALID_RESOLUTION:     return "INVALID_RESOLUTION";
    case ERROR_CODE::CORRUPTED_FRAME:        return "CORRUPTED_FRAME";
    case ERROR_CODE::CAMERA_ALREADY_IN_USE:  return "CAMERA_ALREADY_IN_USE";
    case ERROR_CODE::END_OF_SVOFILE_REACHED: return "END_OF_SVOFILE_REACHED";
    default:                                 return "UNKNOWN";
  }
}

// ============================================================================
// Camera model enum (subset of sl::MODEL)
// ============================================================================
enum class MODEL : int32_t {
  ZED = 0,
  ZED_M = 1,
  ZED2 = 2,
  ZED2i = 3,
  ZED_X = 4,       // not supported via open-capture
  ZED_X_MINI = 5,  // not supported via open-capture
  VIRTUAL = 6,     // simulation
  LAST
};

inline const char* toString(MODEL m) {
  switch (m) {
    case MODEL::ZED:        return "ZED";
    case MODEL::ZED_M:      return "ZED Mini";
    case MODEL::ZED2:       return "ZED 2";
    case MODEL::ZED2i:      return "ZED 2i";
    case MODEL::ZED_X:      return "ZED X";
    case MODEL::ZED_X_MINI: return "ZED X Mini";
    case MODEL::VIRTUAL:    return "Virtual";
    default:                return "Unknown";
  }
}

// ============================================================================
// Resolution enum (subset of sl::RESOLUTION)
// ============================================================================
enum class RESOLUTION_PRESET : int32_t {
  HD2K = 0,    // 2208x1242
  HD1080 = 1,  // 1920x1080
  HD720 = 2,   // 1280x720
  VGA = 3,     // 672x376
  AUTO = 4,    // default to HD720
  LAST
};

inline const char* toString(RESOLUTION_PRESET r) {
  switch (r) {
    case RESOLUTION_PRESET::HD2K:   return "HD2K";
    case RESOLUTION_PRESET::HD1080: return "HD1080";
    case RESOLUTION_PRESET::HD720:  return "HD720";
    case RESOLUTION_PRESET::VGA:    return "VGA";
    case RESOLUTION_PRESET::AUTO:   return "AUTO";
    default:                        return "Unknown";
  }
}

// ============================================================================
// Depth mode (only NONE and basic stereo BM/SGBM available; neural modes gated)
// ============================================================================
enum class DEPTH_MODE : int32_t {
  NONE = 0,
  PERFORMANCE = 1,   // maps to OpenCV StereoBM
  QUALITY = 2,       // maps to OpenCV StereoSGBM
  ULTRA = 3,         // gated (requires SDK)
  NEURAL = 4,        // gated (requires SDK) -- future: MLX stereo
  NEURAL_PLUS = 5,   // gated (requires SDK)
  LAST
};

inline const char* toString(DEPTH_MODE d) {
  switch (d) {
    case DEPTH_MODE::NONE:        return "NONE";
    case DEPTH_MODE::PERFORMANCE: return "PERFORMANCE";
    case DEPTH_MODE::QUALITY:     return "QUALITY";
    case DEPTH_MODE::ULTRA:       return "ULTRA";
    case DEPTH_MODE::NEURAL:      return "NEURAL";
    case DEPTH_MODE::NEURAL_PLUS: return "NEURAL_PLUS";
    default:                      return "Unknown";
  }
}

// ============================================================================
// Coordinate system & units (for compatibility; only ROS convention used)
// ============================================================================
enum class COORDINATE_SYSTEM : int32_t {
  IMAGE = 0,
  LEFT_HANDED_Y_UP = 1,
  RIGHT_HANDED_Y_UP = 2,
  RIGHT_HANDED_Z_UP = 3,
  LEFT_HANDED_Z_UP = 4,
  RIGHT_HANDED_Z_UP_X_FWD = 5,
  LAST
};

enum class UNIT : int32_t {
  MILLIMETER = 0,
  CENTIMETER = 1,
  METER = 2,
  INCH = 3,
  FOOT = 4,
  LAST
};

// ============================================================================
// Timestamp -- nanoseconds since epoch, compatible with sl::Timestamp
// ============================================================================
struct Timestamp {
  uint64_t data_ns = 0;

  Timestamp() = default;
  explicit Timestamp(uint64_t ns) : data_ns(ns) {}
  Timestamp(int v) : data_ns(static_cast<uint64_t>(v)) {}  // for `= 0` compat

  uint64_t getNanoseconds() const { return data_ns; }
  uint64_t getMicroseconds() const { return data_ns / 1000ULL; }
  uint64_t getMilliseconds() const { return data_ns / 1000000ULL; }
  double getSeconds() const { return static_cast<double>(data_ns) / 1e9; }

  void setNanoseconds(uint64_t ns) { data_ns = ns; }

  bool operator==(const Timestamp& o) const { return data_ns == o.data_ns; }
  bool operator!=(const Timestamp& o) const { return data_ns != o.data_ns; }
  bool operator<(const Timestamp& o) const { return data_ns < o.data_ns; }
  bool operator>(const Timestamp& o) const { return data_ns > o.data_ns; }

  // Support `Timestamp ts = 0;` pattern used throughout wrapper
  bool operator==(int v) const { return data_ns == static_cast<uint64_t>(v); }
};

// ============================================================================
// Resolution struct (compatible with sl::Resolution)
// ============================================================================
struct Resolution {
  size_t width = 0;
  size_t height = 0;

  Resolution() = default;
  Resolution(size_t w, size_t h) : width(w), height(h) {}

  size_t area() const { return width * height; }
};

// ============================================================================
// float2 / float3 / float4 (compatible with sl::float2/3/4)
// ============================================================================
struct float2 {
  float x = 0.0f, y = 0.0f;
  float2() = default;
  float2(float x_, float y_) : x(x_), y(y_) {}
  float& operator[](int i) { return (&x)[i]; }
  const float& operator[](int i) const { return (&x)[i]; }
};

struct float3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float3() = default;
  float3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  float& operator[](int i) { return (&x)[i]; }
  const float& operator[](int i) const { return (&x)[i]; }
};

struct float4 {
  float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
  float4() = default;
  float4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
  float& operator[](int i) { return (&x)[i]; }
  const float& operator[](int i) const { return (&x)[i]; }
};

// ============================================================================
// Mat -- lightweight CPU-only image container (replaces sl::Mat)
// ============================================================================
enum class MAT_TYPE : int32_t {
  F32_C1 = 0,   // 32-bit float, 1 channel (depth)
  F32_C2 = 1,
  F32_C3 = 2,
  F32_C4 = 3,   // XYZRGBA point cloud
  U8_C1 = 4,    // 8-bit unsigned, 1 channel (gray)
  U8_C2 = 5,    // YUV 4:2:2
  U8_C3 = 6,    // BGR
  U8_C4 = 7,    // BGRA
  U16_C1 = 8,   // 16-bit depth (OpenNI mode)
  LAST
};

enum class MEM : int32_t {
  CPU = 0,
  GPU = 1  // not available on macOS; always falls back to CPU
};

class Mat {
public:
  Mat();
  Mat(size_t width, size_t height, MAT_TYPE type, MEM mem = MEM::CPU);
  Mat(const Mat& other);
  Mat& operator=(const Mat& other);
  Mat(Mat&& other) noexcept;
  Mat& operator=(Mat&& other) noexcept;
  ~Mat();

  // Allocation
  void alloc(size_t width, size_t height, MAT_TYPE type, MEM mem = MEM::CPU);
  void free();

  // Accessors
  size_t getWidth() const { return width_; }
  size_t getHeight() const { return height_; }
  Resolution getResolution() const { return {width_, height_}; }
  size_t getChannels() const;
  size_t getPixelBytes() const;
  size_t getStepBytes() const;
  MAT_TYPE getDataType() const { return type_; }

  // Raw data pointer (CPU only)
  template<typename T = uint8_t>
  T* getPtr(MEM /*mem*/ = MEM::CPU) {
    return reinterpret_cast<T*>(data_);
  }

  template<typename T = uint8_t>
  const T* getPtr(MEM /*mem*/ = MEM::CPU) const {
    return reinterpret_cast<const T*>(data_);
  }

  // Get value at pixel
  ERROR_CODE getValue(size_t x, size_t y, float* value, MEM mem = MEM::CPU) const;
  ERROR_CODE getValue(size_t x, size_t y, float4* value, MEM mem = MEM::CPU) const;

  // Set value at pixel
  ERROR_CODE setValue(size_t x, size_t y, float value, MEM mem = MEM::CPU);
  ERROR_CODE setValue(size_t x, size_t y, const float4& value, MEM mem = MEM::CPU);

  // Copy from external buffer (e.g., from sl_oc::video::Frame)
  ERROR_CODE setFrom(const uint8_t* src, size_t step, size_t w, size_t h, MAT_TYPE type);

  // Set all bytes to a value
  ERROR_CODE setTo(int value, MEM mem = MEM::CPU);

  bool isInit() const { return data_ != nullptr; }

  // Timestamp associated with this Mat (mirrors sl::Mat.timestamp)
  Timestamp timestamp;

private:
  uint8_t* data_ = nullptr;
  size_t width_ = 0;
  size_t height_ = 0;
  MAT_TYPE type_ = MAT_TYPE::U8_C4;
  bool owns_data_ = false;
};

// ============================================================================
// Transform (4x4 matrix, compatible with sl::Transform)
// ============================================================================
struct Transform {
  float m[4][4];

  Transform() { setIdentity(); }

  void setIdentity() {
    std::memset(m, 0, sizeof(m));
    m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0f;
  }

  float3 getTranslation() const {
    return float3(m[0][3], m[1][3], m[2][3]);
  }

  void setTranslation(const float3& t) {
    m[0][3] = t.x;
    m[1][3] = t.y;
    m[2][3] = t.z;
  }
};

// ============================================================================
// Pose (compatibility stub -- positional tracking is gated)
// ============================================================================
struct Pose {
  Transform pose_data;
  Timestamp timestamp;
  int pose_confidence = -1;  // -1 = not available
  float3 getTranslation() const { return pose_data.getTranslation(); }
};

// ============================================================================
// Calibration parameters (loaded from factory calibration file or server)
// ============================================================================
struct CameraParameters {
  float fx = 0.0f;      // focal length x (pixels)
  float fy = 0.0f;      // focal length y (pixels)
  float cx = 0.0f;      // principal point x
  float cy = 0.0f;      // principal point y
  double disto[12] = {}; // distortion coefficients (k1,k2,p1,p2,k3,...)
  Resolution image_size;
  float v_fov = 0.0f;   // vertical field of view (degrees)
  float h_fov = 0.0f;   // horizontal field of view (degrees)
  float d_fov = 0.0f;   // diagonal field of view (degrees)
};

struct CalibrationParameters {
  CameraParameters left_cam;
  CameraParameters right_cam;
  Transform stereo_transform;  // right-to-left extrinsic
  float baseline = 0.0f;       // in meters (getCameraBaseline)

  // Rectified parameters (after stereo rectification)
  CameraParameters left_cam_rectified;
  CameraParameters right_cam_rectified;
};

// ============================================================================
// Camera information
// ============================================================================
struct CameraInformation {
  int serial_number = 0;
  unsigned int camera_firmware_version = 0;
  unsigned int sensors_firmware_version = 0;
  MODEL camera_model = MODEL::ZED2i;
  Resolution camera_resolution;
  CalibrationParameters camera_configuration;     // at current resolution
  CalibrationParameters calibration_parameters;   // at native resolution
  CalibrationParameters calibration_parameters_raw;
  Transform camera_imu_transform;
  float camera_fps = 0.0f;
};

// ============================================================================
// Sensor data structures (mirrors sl::SensorsData)
// ============================================================================
struct ImuData {
  bool is_available = false;
  Timestamp timestamp;
  float3 angular_velocity;       // deg/s
  float3 linear_acceleration;    // m/s^2
  float temperature = -273.15f;  // IMU temp in Celsius
};

struct MagnetometerData {
  bool is_available = false;
  Timestamp timestamp;
  float3 magnetic_field_calibrated;  // uT
};

struct TemperatureData {
  bool is_available = false;
  Timestamp timestamp;
  float temperature_left = -273.15f;   // CMOS temp left
  float temperature_right = -273.15f;  // CMOS temp right
  float temperature_imu = -273.15f;
};

struct BarometerData {
  bool is_available = false;
  Timestamp timestamp;
  float pressure = 0.0f;   // hPa
  float relative_altitude = 0.0f;
};

struct SensorsData {
  ImuData imu;
  MagnetometerData magnetometer;
  TemperatureData temperature;
  BarometerData barometer;
};

// ============================================================================
// InitParameters (maps to VideoParams + sensor init)
// ============================================================================
struct InitParameters {
  RESOLUTION_PRESET camera_resolution = RESOLUTION_PRESET::HD720;
  int camera_fps = 30;
  DEPTH_MODE depth_mode = DEPTH_MODE::NONE;
  COORDINATE_SYSTEM coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  UNIT coordinate_units = UNIT::METER;
  float depth_minimum_distance = 0.3f;   // meters
  float depth_maximum_distance = 20.0f;  // meters
  int camera_device_id = -1;             // -1 = auto
  int camera_image_flip = 0;             // 0 = auto
  bool camera_disable_self_calib = false;
  bool enable_right_side_measure = false;
  std::string svo_input_filename;        // empty = live camera
  bool svo_real_time_mode = false;
  int sdk_verbose = 0;
  std::string sdk_verbose_log_file;
  int sdk_gpu_id = -1;
  std::string opencv_calibration_file;
  int input_type = 0;  // 0 = USB, 1 = SVO (not supported), 2 = stream (not supported)

  // Serial number filter (-1 = any)
  int serial_number_filter = -1;
};

// ============================================================================
// RuntimeParameters (per-frame grab settings)
// ============================================================================
struct RuntimeParameters {
  bool enable_depth = true;
  bool enable_fill_mode = false;
  int confidence_threshold = 95;
  int texture_confidence_threshold = 100;
  bool remove_saturated_areas = true;
};

// ============================================================================
// RecordingStatus (stub)
// ============================================================================
struct RecordingStatus {
  bool is_recording = false;
  bool is_paused = false;
  bool status = false;
  double current_compression_time = 0.0;
  double current_compression_ratio = 0.0;
  double average_compression_time = 0.0;
  double average_compression_ratio = 0.0;
};

// ============================================================================
// SlOcCamera -- the main camera class wrapping VideoCapture + SensorCapture
// ============================================================================
class SlOcCamera {
public:
  SlOcCamera();
  ~SlOcCamera();

  // Non-copyable
  SlOcCamera(const SlOcCamera&) = delete;
  SlOcCamera& operator=(const SlOcCamera&) = delete;

  // ----> Lifecycle
  ERROR_CODE open(const InitParameters& params = InitParameters());
  void close();
  bool isOpened() const;
  // <---- Lifecycle

  // ----> Frame acquisition
  // Grabs a new frame. Returns SUCCESS if a new frame is available.
  ERROR_CODE grab(const RuntimeParameters& params = RuntimeParameters());

  // Retrieve images into Mat containers
  // view: 0=LEFT, 1=RIGHT, 2=LEFT_UNRECTIFIED, 3=RIGHT_UNRECTIFIED
  ERROR_CODE retrieveImage(Mat& mat, int view = 0, MEM mem = MEM::CPU,
                           Resolution resolution = Resolution());
  // <---- Frame acquisition

  // ----> Camera information
  CameraInformation getCameraInformation(Resolution image_size = Resolution()) const;
  MODEL getCameraModel() const;
  int getSerialNumber() const;
  Resolution getCameraResolution() const;
  float getCurrentFPS() const;
  Timestamp getTimestamp() const;
  // <---- Camera information

  // ----> Sensor data
  ERROR_CODE getSensorsData(SensorsData& data, int time_reference = 0);
  // <---- Sensor data

  // ----> Camera settings (delegated to VideoCapture)
  int setBrightness(int value);
  int getBrightness() const;
  int setContrast(int value);
  int getContrast() const;
  int setHue(int value);
  int getHue() const;
  int setSaturation(int value);
  int getSaturation() const;
  int setSharpness(int value);
  int getSharpness() const;
  int setGamma(int value);
  int getGamma() const;
  int setGain(int value);
  int getGain() const;
  int setExposure(int value);
  int getExposure() const;
  int setWhiteBalance(int value);
  int getWhiteBalance() const;
  int setAutoExposureGain(bool enable);
  bool getAutoExposureGain() const;
  int setAutoWhiteBalance(bool enable);
  bool getAutoWhiteBalance() const;
  int setLED(bool enable);
  bool getLED() const;
  // <---- Camera settings

  // ----> Unsupported features (return NOT_SUPPORTED)
  ERROR_CODE enablePositionalTracking(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::POSITIONAL_TRACKING);
  }
  ERROR_CODE enableSpatialMapping(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::SPATIAL_MAPPING);
  }
  ERROR_CODE enableObjectDetection(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::OBJECT_DETECTION);
  }
  ERROR_CODE enableBodyTracking(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::BODY_TRACKING);
  }
  ERROR_CODE enableStreaming(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::STREAMING_SERVER);
  }
  ERROR_CODE enableRecording(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::SVO_RECORDING);
  }
  ERROR_CODE getPosition(Pose& /*pose*/, int /*reference_frame*/ = 0) {
    SL_OC_UNSUPPORTED(unsupported::Feature::POSITIONAL_TRACKING);
  }
  ERROR_CODE retrieveMeasure(Mat& /*mat*/, int /*measure*/ = 0, MEM /*mem*/ = MEM::CPU,
                             Resolution /*res*/ = Resolution()) {
    // Depth measure could be supported via MLX in the future;
    // for now, gate it unless depth_mode is PERFORMANCE or QUALITY
    SL_OC_UNSUPPORTED(unsupported::Feature::NEURAL_DEPTH);
  }
  ERROR_CODE findPlaneAtHit(/* params */) {
    SL_OC_UNSUPPORTED(unsupported::Feature::PLANE_DETECTION);
  }
  // <---- Unsupported features

  // ----> SDK version compatibility
  static std::string getSDKVersion() { return "0.6.0-oc-bridge"; }
  static void getSDKVersion(int& major, int& minor, int& patch) {
    major = 0; minor = 6; patch = 0;
  }
  // <---- SDK version compatibility

private:
  struct Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace sl_oc_bridge

#endif  // SL_OC_BRIDGE_HPP_
