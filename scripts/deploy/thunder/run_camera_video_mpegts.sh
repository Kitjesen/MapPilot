#!/usr/bin/env bash
set -euo pipefail

# Go2RTC's exec reader needs a timestamped container to pace a raw Annex-B
# H.264 pipe. The camera encoder remains the only SHM consumer; this wrapper
# only repackages its output and never opens a camera device.
exec bash -c '
  set -o pipefail
  /opt/lingtu/current/bin/lingtu_camera_video \
    --color-shm /lingtu_camera_color \
    --output - \
    --codec libx264 \
    --fps 30 \
    --bitrate-kbps 1500 \
    --idle-timeout-ms 3000 \
  | ffmpeg -hide_banner -loglevel error \
      -f h264 -framerate 30 -i pipe:0 \
      -c:v copy -f mpegts pipe:1
'
