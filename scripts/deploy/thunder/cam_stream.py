#!/usr/bin/env python3
"""One color encoder/remux pair for go2rtc."""

import ctypes
import signal
import subprocess
import sys


def child_setup():
    # Kill children when go2rtc kills this producer.
    if ctypes.CDLL(None).prctl(1, signal.SIGTERM) != 0:
        raise OSError("cannot set parent-death signal")


video_cmd = [
    "/opt/lingtu/current/bin/lingtu_camera_video", "--color-shm", "/lingtu_camera_color",
    "--output", "-", "--codec", "libx264", "--fps", "30", "--bitrate-kbps", "1500",
    "--idle-timeout-ms", "3000",
]
mux_cmd = [
    "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "h264", "-framerate", "30",
    "-i", "pipe:0", "-c:v", "copy", "-f", "mpegts", "pipe:1",
]
video = mux = None


def stop(_signum, _frame):
    for process in (mux, video):
        if process is not None and process.poll() is None:
            process.terminate()
    raise SystemExit(0)


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
try:
    video = subprocess.Popen(video_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, preexec_fn=child_setup)
    mux = subprocess.Popen(mux_cmd, stdin=video.stdout, stdout=sys.stdout.buffer, stderr=subprocess.DEVNULL, preexec_fn=child_setup)
    video.stdout.close()
    raise SystemExit(mux.wait())
finally:
    for process in (mux, video):
        if process is not None and process.poll() is None:
            process.terminate()
    for process in (mux, video):
        if process is not None:
            process.wait(timeout=2)
