#!/usr/bin/env python3
# Copyright 2025 AIFLOW LABS LIMITED
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
mlx_depth_worker.py

Standalone Python process that bridges POSIX shared memory to the MLX stereo
depth pipeline. It reads stereo grayscale frames from an input SHM segment
(written by the C++ ROS2 node via MlxDepthBridge), runs the MLX disparity +
depth computation, and writes the results back to an output SHM segment.

Data flow:
    C++ node --> [zed_ros2_stereo_in SHM] --> this worker
    this worker --> [zed_ros2_depth_out SHM] --> C++ node

SHM protocol:
    Both segments use the same 64-byte header + double-buffer seqlock scheme
    as zed-sdk-mlx's shm_frame / zed_capture_stream.

Usage:
    python mlx_depth_worker.py \
        --shm-in zed_ros2_stereo_in \
        --shm-out zed_ros2_depth_out \
        --focal-length 350.0 \
        --baseline-mm 120.0 \
        [--pyramid-factor 2] \
        [--window-size 7] \
        [--max-disparity 96] \
        [--poll-ms 1] \
        [--verbose]
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time
from multiprocessing import shared_memory

import numpy as np

# Import the MLX stereo pipeline from the zed_mlx package
from zed_mlx.mlx_stereo import compute_disparity_refined, disparity_to_depth_mm

# ---------------------------------------------------------------------------
# SHM header definition -- must match mlx_depth_bridge.hpp ShmHeader exactly
# ---------------------------------------------------------------------------
HEADER_STRUCT = struct.Struct("<8sIIIIIIIiQQII")
HEADER_SIZE = HEADER_STRUCT.size
assert HEADER_SIZE == 64, f"Header size mismatch: {HEADER_SIZE} != 64"

SLOT_COUNT = 2

# Magic bytes -- must match the C++ constants
STEREO_IN_MAGIC = b"ZRSTIN1\x00"
DEPTH_OUT_MAGIC = b"ZRDOUT1\x00"

SHM_VERSION = 1

# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------
_shutdown_requested = False


def _signal_handler(signum, frame):
    global _shutdown_requested
    _shutdown_requested = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


# ---------------------------------------------------------------------------
# SHM reader (reads stereo input from C++ node)
# ---------------------------------------------------------------------------
class ShmStereoReader:
    """Reads stereo frame pairs from the input SHM segment."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._shm: shared_memory.SharedMemory | None = None

    def _ensure_open(self) -> bool:
        if self._shm is not None:
            return True
        try:
            self._shm = shared_memory.SharedMemory(name=self.name, create=False)
        except FileNotFoundError:
            return False
        return True

    def close(self) -> None:
        if self._shm is not None:
            self._shm.close()
            self._shm = None

    def try_read(self, last_frame_id: int) -> tuple[np.ndarray, np.ndarray, int, int, int, int] | None:
        """
        Attempt a single non-blocking read of a stereo pair.

        Returns (left_gray, right_gray, width, height, frame_id, timestamp)
        or None if no new frame is available.
        """
        if not self._ensure_open():
            return None

        assert self._shm is not None
        buf = self._shm.buf

        # First header snapshot
        h1 = HEADER_STRUCT.unpack_from(buf, 0)
        magic, version, slot_count, slot_bytes, payload_bytes, \
            width, height, channels, reserved, frame_id, timestamp, \
            active_slot, sequence = h1

        # Validate seqlock (even = stable)
        if sequence % 2 != 0:
            return None
        if magic != STEREO_IN_MAGIC:
            return None
        if version != SHM_VERSION:
            return None
        if frame_id == 0 or frame_id <= last_frame_id:
            return None
        if payload_bytes == 0:
            return None
        if active_slot >= slot_count:
            return None

        # Validate geometry: channels=2 means [left | right] packed
        if channels != 2:
            return None
        expected_bytes = width * height * 2  # two grayscale images
        if payload_bytes != expected_bytes:
            return None
        if payload_bytes > slot_bytes:
            return None

        # Read payload
        slot_offset = HEADER_SIZE + active_slot * slot_bytes
        raw = bytes(buf[slot_offset : slot_offset + payload_bytes])

        # Second header snapshot (seqlock validation)
        h2 = HEADER_STRUCT.unpack_from(buf, 0)
        if h1 != h2:
            return None  # Writer was active during read

        # Unpack into two grayscale images
        single_size = width * height
        left_gray = np.frombuffer(raw, dtype=np.uint8, count=single_size).reshape(height, width)
        right_gray = np.frombuffer(raw, dtype=np.uint8, offset=single_size, count=single_size).reshape(height, width)

        return left_gray, right_gray, width, height, frame_id, timestamp


# ---------------------------------------------------------------------------
# SHM writer (writes depth output for C++ node to read)
# ---------------------------------------------------------------------------
class ShmDepthWriter:
    """Creates and writes to the depth output SHM segment."""

    def __init__(self, name: str, slot_bytes: int) -> None:
        self.name = name
        self.slot_bytes = slot_bytes
        self._shm: shared_memory.SharedMemory | None = None
        self._sequence = 0

    def initialize(self) -> bool:
        total_size = HEADER_SIZE + self.slot_bytes * SLOT_COUNT

        # Clean up stale segment
        try:
            old = shared_memory.SharedMemory(name=self.name, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass

        try:
            self._shm = shared_memory.SharedMemory(
                name=self.name, create=True, size=total_size
            )
        except Exception as exc:
            print(f"[mlx_depth_worker] Failed to create depth SHM '{self.name}': {exc}",
                  file=sys.stderr)
            return False

        # Zero-fill and write initial header
        buf = self._shm.buf
        buf[:total_size] = b"\x00" * total_size
        HEADER_STRUCT.pack_into(
            buf, 0,
            DEPTH_OUT_MAGIC,  # magic
            SHM_VERSION,      # version
            SLOT_COUNT,       # slot_count
            self.slot_bytes,  # slot_bytes
            0,                # payload_bytes
            0,                # width
            0,                # height
            0,                # channels
            0,                # reserved
            0,                # frame_id
            0,                # timestamp
            0,                # active_slot
            0,                # sequence
        )

        print(f"[mlx_depth_worker] Depth SHM created: {self.name} ({total_size} bytes)")
        return True

    def write_result(
        self,
        depth_mm: np.ndarray,
        disparity: np.ndarray,
        width: int,
        height: int,
        frame_id: int,
        timestamp: int,
    ) -> bool:
        if self._shm is None:
            return False

        # Pack [depth_mm (float32) | disparity (float32)]
        depth_bytes = depth_mm.astype(np.float32).tobytes()
        disp_bytes = disparity.astype(np.float32).tobytes()
        payload = depth_bytes + disp_bytes
        payload_len = len(payload)

        if payload_len > self.slot_bytes:
            print(f"[mlx_depth_worker] Payload too large: {payload_len} > {self.slot_bytes}",
                  file=sys.stderr)
            return False

        buf = self._shm.buf

        # Read current active_slot from header
        current_active = struct.unpack_from("<I", buf, 56)[0]

        # Seqlock: set sequence to odd (writing)
        self._sequence += 1
        if self._sequence % 2 == 0:
            self._sequence += 1
        struct.pack_into("<I", buf, 60, self._sequence)

        # Write to the next slot
        next_slot = (current_active + 1) % SLOT_COUNT
        slot_offset = HEADER_SIZE + next_slot * self.slot_bytes
        buf[slot_offset : slot_offset + payload_len] = payload

        # Update header metadata
        # channels=2 means two float32 maps (depth_mm + disparity)
        HEADER_STRUCT.pack_into(
            buf, 0,
            DEPTH_OUT_MAGIC,
            SHM_VERSION,
            SLOT_COUNT,
            self.slot_bytes,
            payload_len,
            width,
            height,
            2,          # channels (depth_mm + disparity)
            0,          # reserved
            frame_id,
            timestamp,
            next_slot,
            self._sequence + 1,  # even = write complete
        )
        self._sequence += 1

        return True

    def close(self) -> None:
        if self._shm is not None:
            self._shm.close()
            try:
                self._shm.unlink()
            except FileNotFoundError:
                pass
            self._shm = None


# ---------------------------------------------------------------------------
# Main worker loop
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="MLX stereo depth worker: reads stereo frames from SHM, "
                    "computes depth via MLX, writes results back to SHM."
    )
    parser.add_argument("--shm-in", default="zed_ros2_stereo_in",
                        help="Name of the input SHM segment (stereo frames)")
    parser.add_argument("--shm-out", default="zed_ros2_depth_out",
                        help="Name of the output SHM segment (depth results)")
    parser.add_argument("--slot-bytes", type=int, default=8 * 1024 * 1024,
                        help="Max bytes per slot for depth output SHM (default: 8 MiB)")

    # MLX stereo parameters
    parser.add_argument("--pyramid-factor", type=int, default=2, choices=[1, 2, 4],
                        help="Pyramid downscale factor (1=full, 2=half, 4=quarter)")
    parser.add_argument("--window-size", type=int, default=7,
                        help="Cost aggregation window size (odd integer)")
    parser.add_argument("--max-disparity", type=int, default=96,
                        help="Maximum disparity search range (pixels)")
    parser.add_argument("--consistency-mode", default="none",
                        choices=["none", "left_right"],
                        help="Left-right consistency check mode")
    parser.add_argument("--refinement-radius", type=int, default=2,
                        help="Edge-aware refinement radius")

    # Depth conversion parameters (camera calibration)
    parser.add_argument("--focal-length", type=float, required=True,
                        help="Focal length in pixels (from camera calibration)")
    parser.add_argument("--baseline-mm", type=float, required=True,
                        help="Stereo baseline in millimeters")

    # Runtime
    parser.add_argument("--poll-ms", type=int, default=1,
                        help="Poll interval in milliseconds when no new frame")
    parser.add_argument("--verbose", action="store_true",
                        help="Print per-frame timing information")

    args = parser.parse_args()

    print(f"[mlx_depth_worker] Starting...")
    print(f"[mlx_depth_worker]   SHM in:  {args.shm_in}")
    print(f"[mlx_depth_worker]   SHM out: {args.shm_out}")
    print(f"[mlx_depth_worker]   pyramid_factor={args.pyramid_factor} "
          f"window_size={args.window_size} max_disparity={args.max_disparity}")
    print(f"[mlx_depth_worker]   focal_length={args.focal_length}px "
          f"baseline={args.baseline_mm}mm")

    # Create the depth output SHM
    depth_writer = ShmDepthWriter(args.shm_out, args.slot_bytes)
    if not depth_writer.initialize():
        return 1

    # Open the stereo input SHM (created by C++ node)
    stereo_reader = ShmStereoReader(args.shm_in)

    last_frame_id = 0
    frame_count = 0
    poll_interval = args.poll_ms / 1000.0

    print(f"[mlx_depth_worker] Waiting for stereo frames on SHM '{args.shm_in}'...")

    try:
        while not _shutdown_requested:
            result = stereo_reader.try_read(last_frame_id)
            if result is None:
                time.sleep(poll_interval)
                continue

            left_gray, right_gray, width, height, frame_id, timestamp = result
            last_frame_id = frame_id

            t_start = time.monotonic()

            # Run MLX stereo matching
            disparity = compute_disparity_refined(
                left_gray,
                right_gray,
                max_disparity=args.max_disparity,
                window_size=args.window_size,
                pyramid_factor=args.pyramid_factor,
                consistency_mode=args.consistency_mode,
                refinement_radius=args.refinement_radius,
            )

            # Convert disparity to depth in millimeters
            depth_mm = disparity_to_depth_mm(
                disparity,
                focal_length_px=args.focal_length,
                baseline_mm=args.baseline_mm,
            )

            t_compute = time.monotonic()

            # Write result to output SHM
            ok = depth_writer.write_result(
                depth_mm, disparity, width, height, frame_id, timestamp
            )

            t_end = time.monotonic()
            frame_count += 1

            if args.verbose or (frame_count % 30 == 0):
                compute_ms = (t_compute - t_start) * 1000
                total_ms = (t_end - t_start) * 1000
                print(f"[mlx_depth_worker] frame={frame_id} "
                      f"{width}x{height} "
                      f"compute={compute_ms:.1f}ms "
                      f"total={total_ms:.1f}ms "
                      f"ok={ok} "
                      f"count={frame_count}")

    except KeyboardInterrupt:
        pass
    finally:
        print(f"[mlx_depth_worker] Shutting down (processed {frame_count} frames)...")
        stereo_reader.close()
        depth_writer.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
