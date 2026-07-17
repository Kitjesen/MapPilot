# LingTu Point Cloud Codec

This is the native C++ encoder for Gateway live point-cloud frames.

It owns only the hot-path compute work:

```text
float32 XYZ point cloud
  -> min-origin quantization
  -> int16 PCLD websocket frame
```

Python still owns Gateway control flow, WebSocket fan-out, and fallback loading.
Those are not the per-point hot path.

Build:

```bash
bash scripts/build/build_pointcloud_codec.sh
```

On Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build/build_pointcloud_codec.ps1
```

At runtime, `runtime.utils.binary_codec` loads the shared library from this
directory's `build/` folder, or from `LINGTU_POINTCLOUD_CODEC_LIB` when set.
If the library is absent, it falls back to the NumPy implementation.
