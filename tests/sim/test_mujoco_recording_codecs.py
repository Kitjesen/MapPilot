"""Exercise the recording dependencies installed by the sim-recording extra."""

import cv2
import imageio.v2 as imageio
import numpy as np
import pytest

pytestmark = [pytest.mark.sim]


def test_recording_can_write_and_read_png_mp4_and_gif(tmp_path):
    rgb = np.zeros((64, 96, 3), dtype=np.uint8)
    rgb[:, :, 0] = 220
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    png = tmp_path / "frame.png"
    assert cv2.imwrite(str(png), bgr)
    np.testing.assert_array_equal(cv2.imread(str(png)), bgr)

    video = tmp_path / "recording.mp4"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), 10, (96, 64))
    try:
        assert writer.isOpened(), "MP4 encoder unavailable"
        for _ in range(8):
            writer.write(bgr)
    finally:
        writer.release()

    capture = cv2.VideoCapture(str(video))
    count = 0
    try:
        assert capture.isOpened(), "MP4 decoder unavailable"
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            assert frame.shape == bgr.shape
            assert np.abs(frame.astype(float) - bgr).mean() < 10
            count += 1
    finally:
        capture.release()
    assert count == 8

    gif = tmp_path / "preview.gif"
    imageio.mimsave(gif, [rgb, 255 - rgb], duration=100)
    frames = imageio.mimread(gif)
    assert len(frames) == 2
    np.testing.assert_array_equal(frames[0], rgb)
    np.testing.assert_array_equal(frames[1], 255 - rgb)
