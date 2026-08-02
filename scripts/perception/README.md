# Perception Script Entrypoints

The legacy ROS2 live camera demos were moved into the explicit compatibility
namespace:

- `scripts/compat/ros2/perception/live_detect.py`
- `scripts/compat/ros2/perception/live_track.py`

They subscribe to ROS camera topics and remain compatibility/debug tools only.
Do not add new ROS2 scripts to this directory. Product runtime should use
LingTu modules through `python lingtu.py ...` and Gateway camera endpoints.
