# Native camera data plane

The field camera path keeps device capture and high-bandwidth transport in C++:

```text
Orbbec SDK capture process
  -> binary capture records over a private pipe
  -> lingtu_camera_dds (C++)
       -> POSIX SHM color/depth/info rings
       -> optional typed DDS color/depth compatibility stream
       -> typed DDS CameraInfo at low rate
  -> DdsCameraModule / ShmCameraModule (Python mmap reader)
  -> color_image / depth_image / camera_info Module ports
```

`DdsCameraModule` is registered under the canonical `camera` role with the
`dds` backend. It does not import or start a Python DDS reader. Full image
payloads use POSIX SHM by default.

## SHM objects

| Stream | POSIX name | Linux path |
| --- | --- | --- |
| Color | `/lingtu_camera_color` | `/dev/shm/lingtu_camera_color` |
| Depth | `/lingtu_camera_depth` | `/dev/shm/lingtu_camera_depth` |
| Intrinsics | `/lingtu_camera_info` | `/dev/shm/lingtu_camera_info` |

Each object is a two-slot ring by default. Each slot reserves 8 MiB, enough for
raw 1920x1080 RGB8 or depth frames. The values are configurable with
`--shm-slot-count` and `--shm-slot-capacity-bytes`.

## Binary contract

Schema: `lingtu.camera.shm_frame.v1`

The 64-byte superblock contains:

- 8-byte magic `LTCSHM01`
- schema version and encoded header sizes
- slot count and per-slot payload capacity
- active slot and published sequence
- creation timestamp and writer heartbeat

Every slot has a fixed 256-byte header followed by its fixed-capacity payload.
The slot header contains:

- begin/end seqlock guards
- sequence and nanosecond Unix timestamp
- stream kind
- width, height, stride, and payload size/capacity
- fixed encoding and frame-id fields
- CRC32 of the committed payload
- camera intrinsics, depth scale, and five distortion coefficients

The C++ writer marks the target slot with an odd guard, writes metadata and
payload, computes CRC32, publishes matching even guards, then atomically updates
the active slot and published sequence. The Python reader accepts a frame only
when both header reads and both superblock reads are identical, guards match and
are even, sequence values agree, payload length equals `stride * height`, CRC32
matches, and the timestamp is inside the configured freshness window.

The Python Module copies the validated visible pixels into its own NumPy array.
This one post-validation copy is intentional: the C++ writer may reuse a ring
slot immediately after the read completes.

## Runtime controls

Native service options:

```text
--color-shm NAME
--depth-shm NAME
--info-shm NAME
--shm-slot-count N
--shm-slot-capacity-bytes N
--publish-image-dds
```

`--publish-image-dds` is off by default and exists only for compatibility and
diagnostics. CameraInfo remains available over typed DDS. The Python Module can
override paths with `LINGTU_CAMERA_COLOR_SHM`, `LINGTU_CAMERA_DEPTH_SHM`, and
`LINGTU_CAMERA_INFO_SHM`.

## Verification

```bash
python -m pytest tests/drivers/test_camera_shm.py \
  tests/drivers/test_camera_dds_module.py \
  tests/drivers/test_orbbec_camera_module.py -q

g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 \
  src/drivers/real/camera/native/tests/test_shm_frame_ring.cpp \
  -o /tmp/test_camera_shm
/tmp/test_camera_shm
```

`native/tests/fake_capture.cpp` and `verify_shm_cross_language.py` exercise the
actual C++ camera service and Python mmap reader against the same binary layout.

## Integration boundary

Deployment readiness should use the camera status file plus the three SHM
objects and their advancing sequences. A catalog that still requires live
color/depth DDS samples describes the compatibility path, not the default data
plane, and must be migrated by the runtime/service owner.
