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

#ifndef MLX_DEPTH_BRIDGE_HPP_
#define MLX_DEPTH_BRIDGE_HPP_

// ============================================================================
// mlx_depth_bridge.hpp
//
// Bidirectional POSIX shared-memory bridge between the C++ ROS2 camera node
// and the Python MLX stereo depth worker (mlx_depth_worker.py).
//
// Data flow:
//   C++ (ROS2 node) --[stereo frames]--> SHM input  --> Python (MLX worker)
//   C++ (ROS2 node) <--[depth result]--- SHM output <-- Python (MLX worker)
//
// Protocol:
//   Both SHM segments use a seqlock double-buffer scheme identical to the
//   one used by zed-sdk-mlx's zed_capture_stream / shm_frame pair:
//     - 64-byte header with magic, version, slot_count, slot_bytes, etc.
//     - 2 data slots; writer alternates between them
//     - sequence counter: odd while writing, even when stable
//     - reader validates header before AND after copy; retries on mismatch
//
// The bridge is pure C++17 with NO ROS2 dependencies.
// ============================================================================

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <atomic>

// Platform POSIX headers (macOS + Linux)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mlx_depth_bridge {

// ============================================================================
// Shared memory header -- binary-compatible with zed-sdk-mlx shm_frame.py
// ============================================================================
constexpr uint32_t kShmVersion = 1;
constexpr uint32_t kSlotCount = 2;
constexpr uint32_t kDefaultStereoSlotBytes = 8 * 1024 * 1024;  // 8 MiB per slot (grayscale pair)
constexpr uint32_t kDefaultDepthSlotBytes  = 8 * 1024 * 1024;  // 8 MiB per slot (float32 depth + disparity)

// Magic bytes for the two SHM segments (distinguishes them from capture SHM)
constexpr char kStereoInMagic[8]  = {'Z', 'R', 'S', 'T', 'I', 'N', '1', '\0'};
constexpr char kDepthOutMagic[8]  = {'Z', 'R', 'D', 'O', 'U', 'T', '1', '\0'};

#pragma pack(push, 1)
struct ShmHeader {
    char     magic[8];       //  0: segment identifier
    uint32_t version;        //  8: protocol version (1)
    uint32_t slot_count;     // 12: number of data slots (2)
    uint32_t slot_bytes;     // 16: max bytes per slot
    uint32_t payload_bytes;  // 20: actual bytes written in active slot
    uint32_t width;          // 24: image width
    uint32_t height;         // 28: image height
    uint32_t channels;       // 32: channels per pixel (1=gray, 2=stereo pair, etc.)
    int32_t  reserved;       // 36: padding / future use
    uint64_t frame_id;       // 40: monotonic frame counter
    uint64_t timestamp;      // 48: nanosecond timestamp
    uint32_t active_slot;    // 56: index of the slot that was last fully written
    uint32_t sequence;       // 60: seqlock sequence number (odd=writing, even=stable)
};
#pragma pack(pop)

static_assert(sizeof(ShmHeader) == 64, "ShmHeader must be exactly 64 bytes");

// ============================================================================
// ShmWriter -- creates and writes to a POSIX SHM segment (seqlock protocol)
// ============================================================================
class ShmWriter {
public:
    ShmWriter() = default;
    ~ShmWriter();

    // Non-copyable
    ShmWriter(const ShmWriter&) = delete;
    ShmWriter& operator=(const ShmWriter&) = delete;

    /// Initialize: creates (or re-creates) the named POSIX SHM segment.
    /// @param name      POSIX SHM name (e.g. "zed_ros2_stereo_in")
    /// @param magic     8-byte magic identifier
    /// @param slotBytes Maximum payload bytes per slot
    /// @return true on success
    bool initialize(const std::string& name, const char magic[8], uint32_t slotBytes);

    /// Write a frame payload into the next slot (seqlock protocol).
    /// @param data          pointer to payload bytes
    /// @param dataBytes     number of bytes to write
    /// @param width         frame width
    /// @param height        frame height
    /// @param channels      channels (e.g. 2 for stereo pair packed side-by-side)
    /// @param frameId       monotonic frame identifier
    /// @param timestampNs   capture timestamp in nanoseconds
    /// @return true on success
    bool writeFrame(const void* data, uint32_t dataBytes,
                    uint32_t width, uint32_t height, uint32_t channels,
                    uint64_t frameId, uint64_t timestampNs);

    void shutdown();

private:
    std::string name_;
    int fd_ = -1;
    void* mapping_ = nullptr;
    size_t mappingBytes_ = 0;
    ShmHeader* header_ = nullptr;
    uint8_t* slots_ = nullptr;
};

// ============================================================================
// ShmReader -- attaches to an existing POSIX SHM segment (seqlock protocol)
// ============================================================================
class ShmReader {
public:
    ShmReader() = default;
    ~ShmReader();

    // Non-copyable
    ShmReader(const ShmReader&) = delete;
    ShmReader& operator=(const ShmReader&) = delete;

    /// Attach to an existing named SHM segment (created by the other process).
    /// @param name      POSIX SHM name
    /// @param magic     expected 8-byte magic
    /// @return true on success (segment exists and magic matches)
    bool attach(const std::string& name, const char magic[8]);

    /// Attempt a single non-blocking read.
    /// Returns true if a new, valid frame was read (frame_id > lastFrameId).
    /// The caller must provide a buffer of at least maxBytes.
    /// On success, outBytes/outWidth/outHeight/outChannels/outFrameId/outTimestamp are filled.
    bool tryRead(uint64_t lastFrameId,
                 void* outBuffer, uint32_t maxBytes,
                 uint32_t& outBytes, uint32_t& outWidth, uint32_t& outHeight,
                 uint32_t& outChannels, uint64_t& outFrameId, uint64_t& outTimestamp);

    bool isAttached() const { return header_ != nullptr; }

    void detach();

private:
    std::string name_;
    int fd_ = -1;
    void* mapping_ = nullptr;
    size_t mappingBytes_ = 0;
    ShmHeader* header_ = nullptr;
    uint8_t* slots_ = nullptr;
    char expectedMagic_[8] = {};
};

// ============================================================================
// MlxDepthBridge -- high-level bridge managing both SHM segments
// ============================================================================
class MlxDepthBridge {
public:
    /// Construct with configurable SHM names.
    /// @param shmInName   name for the stereo input segment (C++ writes, Python reads)
    /// @param shmOutName  name for the depth output segment (Python writes, C++ reads)
    explicit MlxDepthBridge(
        const std::string& shmInName  = "zed_ros2_stereo_in",
        const std::string& shmOutName = "zed_ros2_depth_out");

    ~MlxDepthBridge();

    // Non-copyable
    MlxDepthBridge(const MlxDepthBridge&) = delete;
    MlxDepthBridge& operator=(const MlxDepthBridge&) = delete;

    /// Initialize both SHM segments.
    /// @param stereoSlotBytes  max bytes per slot for the stereo input segment
    /// @param depthSlotBytes   max bytes per slot for the depth output segment
    /// @return true if the stereo-in writer was created successfully
    bool initialize(uint32_t stereoSlotBytes = kDefaultStereoSlotBytes,
                    uint32_t depthSlotBytes  = kDefaultDepthSlotBytes);

    /// Submit a stereo frame (left + right grayscale) to the MLX worker.
    /// The two grayscale images are packed contiguously: [left_gray | right_gray].
    /// Both must be the same size (width x height, uint8_t, 1 channel each).
    /// @param leftGray    pointer to left grayscale image (row-major, uint8_t)
    /// @param rightGray   pointer to right grayscale image (row-major, uint8_t)
    /// @param width       image width
    /// @param height      image height
    /// @param frameId     monotonic frame counter
    /// @param timestampNs capture timestamp in nanoseconds
    /// @return true on success
    bool submitStereoFrame(const uint8_t* leftGray, const uint8_t* rightGray,
                           uint32_t width, uint32_t height,
                           uint64_t frameId, uint64_t timestampNs);

    /// Try to read the latest depth result from the MLX worker.
    /// The output contains two float32 images packed contiguously:
    ///   [depth_mm (W*H*float32) | disparity (W*H*float32)]
    /// @param outDepthMm    output buffer for depth in millimeters (float32, W*H)
    /// @param outDisparity  output buffer for raw disparity map (float32, W*H)
    /// @param outWidth      image width of the result
    /// @param outHeight     image height of the result
    /// @param outFrameId    frame_id that this depth corresponds to
    /// @return true if a new result was available (frame_id > last read frame_id)
    bool getDepthResult(float* outDepthMm, float* outDisparity,
                        uint32_t& outWidth, uint32_t& outHeight,
                        uint64_t& outFrameId);

    /// Shut down both SHM segments.
    void shutdown();

    bool isInitialized() const { return initialized_; }

    const std::string& shmInName()  const { return shmInName_; }
    const std::string& shmOutName() const { return shmOutName_; }

private:
    std::string shmInName_;
    std::string shmOutName_;
    bool initialized_ = false;

    ShmWriter stereoWriter_;   // C++ writes stereo frames here
    ShmReader depthReader_;    // C++ reads depth results here

    uint64_t lastDepthFrameId_ = 0;

    // Temporary buffer for packing stereo pair / unpacking depth result
    std::vector<uint8_t> packBuffer_;
    std::vector<uint8_t> readBuffer_;
};

}  // namespace mlx_depth_bridge

#endif  // MLX_DEPTH_BRIDGE_HPP_
