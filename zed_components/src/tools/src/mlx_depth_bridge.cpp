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
// mlx_depth_bridge.cpp
//
// Implementation of the bidirectional POSIX shared-memory bridge between the
// C++ ROS2 camera node and the Python MLX stereo depth worker.
//
// Standalone C++17, no ROS2 dependencies.
// ============================================================================

#include "mlx_depth_bridge.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace mlx_depth_bridge {

// ============================================================================
// ShmWriter
// ============================================================================

ShmWriter::~ShmWriter() {
    shutdown();
}

bool ShmWriter::initialize(const std::string& name, const char magic[8], uint32_t slotBytes) {
    shutdown();

    if (name.empty() || slotBytes < 1024) {
        std::cerr << "[MlxDepthBridge] ShmWriter: invalid name or slotBytes" << std::endl;
        return false;
    }

    name_ = name;
    if (name_[0] != '/') {
        name_.insert(name_.begin(), '/');
    }

    mappingBytes_ = sizeof(ShmHeader) + static_cast<size_t>(slotBytes) * kSlotCount;

    // Remove stale segment, then create fresh
    shm_unlink(name_.c_str());
    fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd_ < 0) {
        std::perror("[MlxDepthBridge] shm_open (writer)");
        return false;
    }

    if (ftruncate(fd_, static_cast<off_t>(mappingBytes_)) != 0) {
        std::perror("[MlxDepthBridge] ftruncate (writer)");
        shutdown();
        return false;
    }

    mapping_ = mmap(nullptr, mappingBytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
        std::perror("[MlxDepthBridge] mmap (writer)");
        mapping_ = nullptr;
        shutdown();
        return false;
    }

    std::memset(mapping_, 0, mappingBytes_);
    header_ = static_cast<ShmHeader*>(mapping_);
    slots_ = reinterpret_cast<uint8_t*>(header_ + 1);

    std::memcpy(header_->magic, magic, 8);
    header_->version = kShmVersion;
    header_->slot_count = kSlotCount;
    header_->slot_bytes = slotBytes;
    header_->reserved = 0;

    std::cout << "[MlxDepthBridge] ShmWriter initialized: " << name_
              << " (" << mappingBytes_ << " bytes)" << std::endl;
    return true;
}

bool ShmWriter::writeFrame(const void* data, uint32_t dataBytes,
                           uint32_t width, uint32_t height, uint32_t channels,
                           uint64_t frameId, uint64_t timestampNs) {
    if (header_ == nullptr || slots_ == nullptr) {
        return false;
    }
    if (dataBytes > header_->slot_bytes) {
        std::cerr << "[MlxDepthBridge] ShmWriter: payload too large ("
                  << dataBytes << " > " << header_->slot_bytes << ")" << std::endl;
        return false;
    }

    // Seqlock: increment to odd (signals "writing in progress")
    uint32_t startSeq = header_->sequence + 1;
    if ((startSeq % 2) == 0) {
        ++startSeq;
    }

    const uint32_t nextSlot = (header_->active_slot + 1) % header_->slot_count;
    header_->sequence = startSeq;
    std::atomic_thread_fence(std::memory_order_release);

    // Copy payload into the inactive slot
    uint8_t* dest = slots_ + static_cast<size_t>(nextSlot) * header_->slot_bytes;
    std::memcpy(dest, data, dataBytes);
    std::atomic_thread_fence(std::memory_order_release);

    // Update header metadata
    header_->payload_bytes = dataBytes;
    header_->width = width;
    header_->height = height;
    header_->channels = channels;
    header_->frame_id = frameId;
    header_->timestamp = timestampNs;
    header_->active_slot = nextSlot;
    std::atomic_thread_fence(std::memory_order_release);

    // Seqlock: increment to even (signals "write complete")
    header_->sequence = startSeq + 1;
    return true;
}

void ShmWriter::shutdown() {
    if (mapping_ != nullptr) {
        munmap(mapping_, mappingBytes_);
        mapping_ = nullptr;
    }
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    if (!name_.empty()) {
        shm_unlink(name_.c_str());
        name_.clear();
    }
    mappingBytes_ = 0;
    header_ = nullptr;
    slots_ = nullptr;
}

// ============================================================================
// ShmReader
// ============================================================================

ShmReader::~ShmReader() {
    detach();
}

bool ShmReader::attach(const std::string& name, const char magic[8]) {
    detach();

    std::string shmName = name;
    if (!shmName.empty() && shmName[0] != '/') {
        shmName.insert(shmName.begin(), '/');
    }

    fd_ = shm_open(shmName.c_str(), O_RDONLY, 0);
    if (fd_ < 0) {
        // Segment doesn't exist yet -- this is normal during startup
        return false;
    }

    // Stat to get size
    struct stat st;
    if (fstat(fd_, &st) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    mappingBytes_ = static_cast<size_t>(st.st_size);
    if (mappingBytes_ < sizeof(ShmHeader)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    mapping_ = mmap(nullptr, mappingBytes_, PROT_READ, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
        mapping_ = nullptr;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Validate magic
    auto* hdr = static_cast<const ShmHeader*>(mapping_);
    if (std::memcmp(hdr->magic, magic, 8) != 0) {
        std::cerr << "[MlxDepthBridge] ShmReader: magic mismatch for " << name << std::endl;
        munmap(mapping_, mappingBytes_);
        mapping_ = nullptr;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    header_ = const_cast<ShmHeader*>(hdr);
    slots_ = reinterpret_cast<uint8_t*>(
        static_cast<uint8_t*>(mapping_) + sizeof(ShmHeader));
    name_ = shmName;
    std::memcpy(expectedMagic_, magic, 8);

    std::cout << "[MlxDepthBridge] ShmReader attached: " << name_
              << " (" << mappingBytes_ << " bytes)" << std::endl;
    return true;
}

bool ShmReader::tryRead(uint64_t lastFrameId,
                        void* outBuffer, uint32_t maxBytes,
                        uint32_t& outBytes, uint32_t& outWidth, uint32_t& outHeight,
                        uint32_t& outChannels, uint64_t& outFrameId, uint64_t& outTimestamp) {
    if (header_ == nullptr || slots_ == nullptr) {
        return false;
    }

    // Read header (first snapshot)
    std::atomic_thread_fence(std::memory_order_acquire);
    ShmHeader h1;
    std::memcpy(&h1, header_, sizeof(ShmHeader));

    // Seqlock: sequence must be even (write not in progress)
    if (h1.sequence % 2 != 0) {
        return false;
    }
    if (h1.frame_id == 0 || h1.frame_id <= lastFrameId) {
        return false;
    }
    if (h1.payload_bytes == 0 || h1.payload_bytes > maxBytes) {
        return false;
    }
    if (h1.active_slot >= h1.slot_count) {
        return false;
    }
    if (h1.payload_bytes > h1.slot_bytes) {
        return false;
    }

    // Copy payload from active slot
    size_t slotOffset = sizeof(ShmHeader) + static_cast<size_t>(h1.active_slot) * h1.slot_bytes;
    if (slotOffset + h1.payload_bytes > mappingBytes_) {
        return false;
    }

    const uint8_t* src = static_cast<const uint8_t*>(mapping_) + slotOffset;
    std::memcpy(outBuffer, src, h1.payload_bytes);

    // Seqlock: validate header hasn't changed
    std::atomic_thread_fence(std::memory_order_acquire);
    ShmHeader h2;
    std::memcpy(&h2, header_, sizeof(ShmHeader));
    if (std::memcmp(&h1, &h2, sizeof(ShmHeader)) != 0) {
        return false;  // Writer was active during our read; discard
    }

    outBytes = h1.payload_bytes;
    outWidth = h1.width;
    outHeight = h1.height;
    outChannels = h1.channels;
    outFrameId = h1.frame_id;
    outTimestamp = h1.timestamp;
    return true;
}

void ShmReader::detach() {
    if (mapping_ != nullptr) {
        munmap(mapping_, mappingBytes_);
        mapping_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    // Reader does NOT unlink -- the writer owns the segment
    name_.clear();
    mappingBytes_ = 0;
    header_ = nullptr;
    slots_ = nullptr;
}

// ============================================================================
// MlxDepthBridge
// ============================================================================

MlxDepthBridge::MlxDepthBridge(const std::string& shmInName, const std::string& shmOutName)
    : shmInName_(shmInName), shmOutName_(shmOutName)
{}

MlxDepthBridge::~MlxDepthBridge() {
    shutdown();
}

bool MlxDepthBridge::initialize(uint32_t stereoSlotBytes, uint32_t depthSlotBytes) {
    if (initialized_) {
        shutdown();
    }

    // Create the stereo input segment (C++ is the writer)
    if (!stereoWriter_.initialize(shmInName_, kStereoInMagic, stereoSlotBytes)) {
        std::cerr << "[MlxDepthBridge] Failed to create stereo input SHM: "
                  << shmInName_ << std::endl;
        return false;
    }

    // The depth output segment is created by the Python worker.
    // We will try to attach lazily in getDepthResult().
    // (Do NOT fail initialization if it doesn't exist yet.)

    initialized_ = true;
    lastDepthFrameId_ = 0;

    std::cout << "[MlxDepthBridge] Initialized."
              << " stereo_in=" << shmInName_
              << " depth_out=" << shmOutName_ << std::endl;
    return true;
}

bool MlxDepthBridge::submitStereoFrame(const uint8_t* leftGray, const uint8_t* rightGray,
                                       uint32_t width, uint32_t height,
                                       uint64_t frameId, uint64_t timestampNs) {
    if (!initialized_) {
        return false;
    }

    // Pack left + right grayscale contiguously: [left (W*H) | right (W*H)]
    const uint32_t singleSize = width * height;
    const uint32_t totalSize = singleSize * 2;

    if (packBuffer_.size() < totalSize) {
        packBuffer_.resize(totalSize);
    }

    std::memcpy(packBuffer_.data(), leftGray, singleSize);
    std::memcpy(packBuffer_.data() + singleSize, rightGray, singleSize);

    // channels=2 signals "stereo pair" (each channel is one grayscale eye)
    return stereoWriter_.writeFrame(
        packBuffer_.data(), totalSize,
        width, height, /*channels=*/2,
        frameId, timestampNs);
}

bool MlxDepthBridge::getDepthResult(float* outDepthMm, float* outDisparity,
                                    uint32_t& outWidth, uint32_t& outHeight,
                                    uint64_t& outFrameId) {
    if (!initialized_) {
        return false;
    }

    // Lazy attach to the depth output segment
    if (!depthReader_.isAttached()) {
        if (!depthReader_.attach(shmOutName_, kDepthOutMagic)) {
            return false;  // Python worker hasn't created it yet
        }
    }

    // The depth output packs two float32 maps: [depth_mm | disparity]
    // Total payload = width * height * sizeof(float) * 2
    // We need a read buffer large enough for the max slot size.
    // Use a generous upper bound; the actual size is validated by tryRead.
    constexpr uint32_t kMaxReadBytes = 32 * 1024 * 1024;  // 32 MiB
    if (readBuffer_.size() < kMaxReadBytes) {
        readBuffer_.resize(kMaxReadBytes);
    }

    uint32_t bytesRead = 0;
    uint32_t w = 0, h = 0, ch = 0;
    uint64_t fid = 0, ts = 0;

    if (!depthReader_.tryRead(lastDepthFrameId_,
                              readBuffer_.data(), kMaxReadBytes,
                              bytesRead, w, h, ch, fid, ts)) {
        return false;
    }

    // Validate: channels should be 2 (depth_mm + disparity), each float32
    // payload_bytes should be w * h * sizeof(float) * 2
    const uint32_t expectedBytes = w * h * static_cast<uint32_t>(sizeof(float)) * 2;
    if (bytesRead != expectedBytes || ch != 2) {
        std::cerr << "[MlxDepthBridge] Unexpected depth result format: "
                  << bytesRead << " bytes, " << ch << " channels "
                  << "(expected " << expectedBytes << " bytes, 2 channels)" << std::endl;
        return false;
    }

    const uint32_t singleMapBytes = w * h * static_cast<uint32_t>(sizeof(float));
    std::memcpy(outDepthMm,   readBuffer_.data(),                  singleMapBytes);
    std::memcpy(outDisparity,  readBuffer_.data() + singleMapBytes, singleMapBytes);

    outWidth = w;
    outHeight = h;
    outFrameId = fid;
    lastDepthFrameId_ = fid;
    return true;
}

void MlxDepthBridge::shutdown() {
    stereoWriter_.shutdown();
    depthReader_.detach();
    initialized_ = false;
    lastDepthFrameId_ = 0;
    std::cout << "[MlxDepthBridge] Shutdown." << std::endl;
}

}  // namespace mlx_depth_bridge
