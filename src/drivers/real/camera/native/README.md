# Camera adapter architecture

The camera extension boundary is a **capture executable and a versioned record
stream**, not a vendor SDK plugin loaded into the video process. Existing
Orbbec and RealSense executables now share [RecordWriter](record_writer.hpp).
No additional SDK, camera model, or transport is supported by this extraction.

```text
physical camera / existing camera owner
  -> vendor adapter (device selection, decoding, alignment, calibration)
  -> RecordWriter -> camera_record v2 on a private pipe
  -> lingtu_camera_dds (record validation, source generation, SHM publication)
       -> color / depth / intrinsics SHM -> perception
       -> color SHM -> video ColorReader -> target encoder -> WHEP
                                                Gateway -> App receiver
```

## Ownership and contracts

| Boundary | Contract and owner |
| --- | --- |
| Hardware to adapter | The adapter alone opens the device, or subscribes to an already owned source. It owns its vendor SDK, source timestamps, calibration, alignment and source frame deduplication. |
| Adapter to publisher | [camera_record.hpp](camera_record.hpp): little-endian v2, fixed 116-byte header, followed by exactly `payload_size` bytes. Intrinsics must precede images. Only RGB8/BGR8 packed color and depth-u16 in millimeters are accepted. |
| Shared writer | [record_writer.hpp](record_writer.hpp): validates each header and payload pointer before output, writes bytes synchronously, and reports output/flush failures. The caller retains valid payload storage through `write()` and chooses the flush boundary. It does not infer calibration, resample, decode compressed data or synchronize streams. |
| Publisher to consumers | [shm_frame_ring.hpp](shm_frame_ring.hpp): same-Linux-host latest-frame rings. Publisher owns one generation and per-stream sequences; readers validate and observe freshness. The publisher exits on an invalid/ended/stalled capture stream. |
| Video and network | [Video delivery guide](video/DELIVERY.md): video reads color SHM, selects a target-supported encoder, and owns WHEP sessions. Gateway owns signalling/access policy; App owns reception, decoded-frame freshness and teardown. These layers never acquire the physical camera. |

The `Service` in [sdk.hpp](sdk.hpp) remains an optional in-process interface used
by Orbbec. RealSense uses its own SDK pipeline. Both publish through the same
record writer and publisher; neither must implement an artificial common device
object to preserve the process boundary.

The record protocol does not carry separate intrinsics for every stream or a
general extrinsic graph. Adapters must preserve the existing aligned RGB-D
coordinate convention and depth scale expected by their selected consumers.
Do not claim arbitrary independent color/depth calibration is representable.
The current Product camera readiness requires color, depth and intrinsics;
color-only hardware requires an explicit consumer/readiness change before it
can qualify as that Product camera.

## Existing adapters

| Adapter | SDK boundary | Record output |
| --- | --- | --- |
| Orbbec | `../impl/orbbec/` and `capture_process.cpp` | Shared `RecordWriter`; current per-record flushing retained. |
| RealSense | `../impl/realsense/capture.cpp` | Shared `RecordWriter`; current per-frameset flushing retained. Depth aligned to color; repeated source frames are not republished. |
| USB/CSI/RTSP/GS130W | No general adapter implemented | Must produce the same accepted records before the existing SHM/video consumers can be reused. |

## Adding a source

1. Keep the source-specific code and build dependencies under `../impl/<source>/`.
   Reuse an existing device owner where one already exists. Do not open a device
   twice to provide perception and video.
2. Resolve the actual source mode and calibration. Decode compressed inputs and
   remove row padding in this adapter. Use fresh source frames and Unix seconds;
   never label device uptime or a replayed frame as a fresh UTC observation.
3. Include `record_writer.hpp`, construct headers with `makeRecordHeader()`,
   emit valid intrinsics before images, call `RecordWriter::write()`, then flush
   at the chosen frame/frameset boundary. Normalize depth with the existing
   depth-unit helpers. Keep logs on stderr; stdout contains records only.
4. Exercise the adapter through the existing `lingtu_camera_dds --capture-bin
   /absolute/path/to/capture -- CAPTURE_ARGS...` boundary. For an already managed
   producer, `--record-source stdin` accepts only a record stream and does not
   spawn or own the upstream producer. Neither option defines Product lifecycle
   or selects a new production driver automatically.
5. Add the device selection, binary packaging and readiness mapping through the
   existing RobotConfig/RunPlan and ProductControl owners. The current managed
   launcher selects only `orbbec_native` or `realsense_native`; a third name
   needs its explicit mapping and deployment validation.
6. Run the record writer/reader tests, then qualify the new hardware: actual
   calibration and alignment, source-frame freshness, disconnect/reconnect,
   concurrent perception/video load, target encoder and App decoded frames.
   Follow [DELIVERY.md](video/DELIVERY.md) and record target-specific results.

Cross-host camera sources need an explicit receiver before the local record
boundary; POSIX SHM is not a network transport. This architecture does not
choose that transport or grant video viewing permission.

## Local verification

The SDK-independent contract can be tested without a camera or CycloneDDS:

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -DNDEBUG \
  -I src/drivers/real/camera/native \
  tests/drivers/cpp/camera/native/test_record_writer.cpp \
  -o /tmp/test_camera_record_writer
/tmp/test_camera_record_writer
```

The native CMake test target `lingtu_camera_record_writer` runs the same test.
It checks unchanged wire bytes, rejected records producing no output, and
write/flush error propagation. SDK compilation and physical capture remain
separate checks.
