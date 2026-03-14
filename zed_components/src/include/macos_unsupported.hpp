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

#ifndef MACOS_UNSUPPORTED_HPP_
#define MACOS_UNSUPPORTED_HPP_

// ============================================================================
// macos_unsupported.hpp
//
// Capability-gating layer for features that require the proprietary ZED SDK
// (CUDA, Neural depth, SLAM, Object Detection, etc.) and cannot run on macOS
// with the open-source zed-sdk-mlx library.
//
// Usage:
//   SL_OC_UNSUPPORTED(Feature::OBJECT_DETECTION);
//   // expands to a log warning + returns sl_oc_bridge::ERROR_CODE::NOT_SUPPORTED
//
//   if (!sl_oc_bridge::unsupported::isFeatureSupported(Feature::BODY_TRACKING)) { ... }
// ============================================================================

#include <cstdint>
#include <string>
#include <iostream>

namespace sl_oc_bridge {
namespace unsupported {

// -----------------------------------------------------------------------
// Enumeration of all features that are gated on macOS / open-source path
// -----------------------------------------------------------------------
enum class Feature : uint32_t {
  // Depth (Neural / CUDA-based)
  NEURAL_DEPTH = 0,
  ULTRA_DEPTH,
  NEURAL_PLUS_DEPTH,

  // Positional Tracking / SLAM
  POSITIONAL_TRACKING,
  SPATIAL_MAPPING,
  AREA_MEMORY,

  // Object Detection / Body Tracking
  OBJECT_DETECTION,
  CUSTOM_OBJECT_DETECTION,
  BODY_TRACKING,

  // Recording / Streaming
  SVO_RECORDING,
  SVO_PLAYBACK,
  STREAMING_SERVER,
  STREAMING_CLIENT,

  // Fusion module
  FUSION,
  GNSS_FUSION,

  // Plane detection
  PLANE_DETECTION,

  // Region of Interest (auto)
  AUTO_ROI,

  // CUDA / GPU
  CUDA_ACCELERATION,
  GPU_MEMORY_ACCESS,

  // Camera models not supported via open-capture
  ZEDX_CAMERA,
  ZEDX_MINI_CAMERA,

  // Confidence map (requires SDK depth engine)
  CONFIDENCE_MAP,

  // Point cloud (fused)
  FUSED_POINT_CLOUD,

  // Health / diagnostics from SDK
  SDK_HEALTH_STATUS,

  FEATURE_COUNT  // sentinel
};

// -----------------------------------------------------------------------
// Human-readable feature names
// -----------------------------------------------------------------------
inline const char* featureName(Feature f) {
  switch (f) {
    case Feature::NEURAL_DEPTH:            return "Neural Depth";
    case Feature::ULTRA_DEPTH:             return "Ultra Depth";
    case Feature::NEURAL_PLUS_DEPTH:       return "Neural+ Depth";
    case Feature::POSITIONAL_TRACKING:     return "Positional Tracking";
    case Feature::SPATIAL_MAPPING:         return "Spatial Mapping";
    case Feature::AREA_MEMORY:             return "Area Memory";
    case Feature::OBJECT_DETECTION:        return "Object Detection";
    case Feature::CUSTOM_OBJECT_DETECTION: return "Custom Object Detection";
    case Feature::BODY_TRACKING:           return "Body Tracking";
    case Feature::SVO_RECORDING:           return "SVO Recording";
    case Feature::SVO_PLAYBACK:            return "SVO Playback";
    case Feature::STREAMING_SERVER:        return "Streaming Server";
    case Feature::STREAMING_CLIENT:        return "Streaming Client";
    case Feature::FUSION:                  return "Fusion Module";
    case Feature::GNSS_FUSION:             return "GNSS Fusion";
    case Feature::PLANE_DETECTION:         return "Plane Detection";
    case Feature::AUTO_ROI:                return "Automatic ROI";
    case Feature::CUDA_ACCELERATION:       return "CUDA Acceleration";
    case Feature::GPU_MEMORY_ACCESS:       return "GPU Memory Access";
    case Feature::ZEDX_CAMERA:             return "ZED-X Camera";
    case Feature::ZEDX_MINI_CAMERA:        return "ZED-X Mini Camera";
    case Feature::CONFIDENCE_MAP:          return "Confidence Map";
    case Feature::FUSED_POINT_CLOUD:       return "Fused Point Cloud";
    case Feature::SDK_HEALTH_STATUS:       return "SDK Health Status";
    default:                               return "Unknown Feature";
  }
}

// -----------------------------------------------------------------------
// Feature support query -- returns false for everything that needs ZED SDK
// -----------------------------------------------------------------------
inline bool isFeatureSupported(Feature /*f*/) {
  // On the macOS / zed-sdk-mlx path, NONE of the gated features are supported.
  // When/if individual features get open-source implementations, add cases here.
  return false;
}

}  // namespace unsupported
}  // namespace sl_oc_bridge

// -----------------------------------------------------------------------
// Convenience macro: logs a warning and returns NOT_SUPPORTED error code.
// Use inside functions that return sl_oc_bridge::ERROR_CODE.
// -----------------------------------------------------------------------
#define SL_OC_UNSUPPORTED(feature)                                              \
  do {                                                                          \
    std::cerr << "[sl_oc_bridge] WARNING: "                                     \
              << sl_oc_bridge::unsupported::featureName(feature)                \
              << " is not supported on macOS (requires proprietary ZED SDK)."   \
              << std::endl;                                                     \
    return sl_oc_bridge::ERROR_CODE::NOT_SUPPORTED;                             \
  } while (0)

// Variant that returns void (for void functions)
#define SL_OC_UNSUPPORTED_VOID(feature)                                         \
  do {                                                                          \
    std::cerr << "[sl_oc_bridge] WARNING: "                                     \
              << sl_oc_bridge::unsupported::featureName(feature)                \
              << " is not supported on macOS (requires proprietary ZED SDK)."   \
              << std::endl;                                                     \
    return;                                                                     \
  } while (0)

// Variant that returns false (for bool functions)
#define SL_OC_UNSUPPORTED_BOOL(feature)                                         \
  do {                                                                          \
    std::cerr << "[sl_oc_bridge] WARNING: "                                     \
              << sl_oc_bridge::unsupported::featureName(feature)                \
              << " is not supported on macOS (requires proprietary ZED SDK)."   \
              << std::endl;                                                     \
    return false;                                                               \
  } while (0)

#endif  // MACOS_UNSUPPORTED_HPP_
