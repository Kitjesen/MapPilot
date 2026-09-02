# Gazebo Adapter Assets

External scenes are optional assets used for server-side validation. Keep large
or license-restricted assets out of git unless their redistribution terms are
confirmed.

Unity exports are not simulator runtime assets. Import them directly through
the offline `lingtu-maps-import-unity` tool; do not place Unity executables,
ROS bridge state, or runtime evidence under this directory.

Keep imported server-only Gazebo assets beside this adapter:

```text
sim/adapters/gazebo/assets/external/
  <scene_name>/
    worlds/
    models/
    maps/
```

Use a Product-backed scene for an acceptance claim only after its current gate
proves sensor topics, frame alignment, map growth, non-zero simulated motion,
obstacle clearance, and no hardware command output.
