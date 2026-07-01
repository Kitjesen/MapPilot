# Perception Script Entrypoints

This directory currently contains legacy ROS2 live camera demos:

- `live_detect.py`
- `live_track.py`

They subscribe to ROS camera topics and are kept as compatibility/debug tools.
Product runtime should use LingTu modules through `python lingtu.py ...` and
Gateway camera endpoints instead of wiring new default behavior through these
scripts.
