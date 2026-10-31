"""Image processing must not erase the camera source used by Follow."""

import numpy as np
import pytest

from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


@pytest.mark.parametrize("fmt", [ImageFormat.BGR, ImageFormat.RGB, ImageFormat.RGBA, ImageFormat.GRAY])
@pytest.mark.parametrize("operation", ["to_rgb", "to_bgr", "to_grayscale", "resize", "crop"])
def test_pixel_operations_keep_source_identity(fmt, operation):
    channels = {ImageFormat.BGR: 3, ImageFormat.RGB: 3, ImageFormat.RGBA: 4}
    shape = (8, 8, channels[fmt]) if fmt in channels else (8, 8)
    image = Image(
        data=np.ones(shape, dtype=np.uint8), format=fmt, ts=123.5,
        frame_id="camera_optical", source_generation=42, source_sequence=7,
    )
    args = (4, 4) if operation == "resize" else (0, 0, 4, 4) if operation == "crop" else ()
    converted = getattr(image, operation)(*args)
    assert (converted.source_generation, converted.source_sequence, converted.ts) == (42, 7, 123.5)
    assert converted.frame_id == "camera_optical"


def test_camera_info_dictionary_preserves_source():
    info = CameraIntrinsics(fx=10, fy=10, width=8, height=8, source_generation=42, source_sequence=3)
    recovered = CameraIntrinsics.from_dict(info.to_dict())
    assert (recovered.source_generation, recovered.source_sequence, recovered.ts) == (42, 3, info.ts)
