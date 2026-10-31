# Dataset tools

Choose a tool by the input and evidence you need. Run commands from the
repository root in the existing project environment. Tool output belongs under
`build/` or an explicitly selected output directory.

| Task | Entry points |
| --- | --- |
| Discover public SLAM recordings and prepare replay manifests | [`public_slam_dataset.py`](public_slam_dataset.py) |
| Capture SLAM status and evaluate inspection readiness | [`capture_slam_status_jsonl.py`](capture_slam_status_jsonl.py), [`inspection_localization_readiness.py`](inspection_localization_readiness.py) |
| Convert normalized LiDAR/IMU streams or official Elevator-LIO bags to LTU1 | [`normalized_lidar_imu_to_ltu1.py`](normalized_lidar_imu_to_ltu1.py), [`elevator_bag_to_ltu1.py`](elevator_bag_to_ltu1.py) |
| Inventory local semantic-map evidence | [`semantic_dataset_inventory.py`](semantic_dataset_inventory.py) |
| Inspect Habitat scenes, episodes, objects, and visual evidence | [`habitat_probe.py`](habitat_probe.py), [`habitat_scene.py`](habitat_scene.py), [`habitat_episode.py`](habitat_episode.py), [`habitat_objects.py`](habitat_objects.py), [`habitat_vision.py`](habitat_vision.py) |
| Inspect ReplicaCAD and 3DSSG data | [`replicacad_probe.py`](replicacad_probe.py), [`replicacad_persistence_check.py`](replicacad_persistence_check.py), [`3dssg_probe.py`](3dssg_probe.py), [`3dssg_goal_query.py`](3dssg_goal_query.py) |
| Inspect TUM RGB-D or evaluate person tracking | [`tum_rgbd_probe.py`](tum_rgbd_probe.py), [`person_follow.py`](person_follow.py), [`dino_follow.py`](dino_follow.py), [`person_eval.py`](person_eval.py) |

```bash
python tools/datasets/public_slam_dataset.py list
python tools/datasets/semantic_dataset_inventory.py --write build/dataset-inventory.json
python tools/datasets/elevator_bag_to_ltu1.py --help
python tools/datasets/person_eval.py --help
```

Dataset availability, calibration, timestamps, and ground-truth coverage belong
to each recording's manifest. A conversion or replay process finishing does not
establish localization accuracy. Replay usage and evidence limits are described
in the [tools index](../README.md#specialized-development-tools) and
[SLAM evaluation guide](../../sim/evaluation/slam/README.md).

Habitat helper modules and [`qwen_chat.py`](qwen_chat.py) also support other
tools. Preserve their import paths when changing dataset entrypoints.
