# OTA scripts

This directory contains legacy colcon-based OTA helpers kept for compatibility
with older robot installs.

For new Thunder field releases, prefer:

```bash
bash scripts/deploy/deploy_thunder.sh
bash scripts/deploy/cut_release.sh vX.Y.Z
```

The default product runtime is built from native Python/C++ artifacts such as
`nav_kernel` and OctoPlanner3D. Use these OTA scripts only when intentionally
deploying a ROS compatibility workspace or an existing install tree.
