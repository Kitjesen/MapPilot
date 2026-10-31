# Semantic-map dataset plan

**Status:** Dataset-first execution plan, 2026-10-03  
**Scope:** Establish evidence for geometry, multimodal fusion, persistent
semantic mapping, and semantic navigation without mixing their acceptance
levels.

森哥，当前的“语义地图”讨论混合了四种不同结果。先把它们分开：

| Level | Required evidence | What it proves | Current state |
| --- | --- | --- | --- |
| D0 inventory | file/catalog report | We know exactly what data exists | **Done** |
| D1 geometry map | PCD or LiDAR replay | surfaces, structure, collision candidates, planner geometry | **Done for offline PCD** |
| D2 multimodal projection | synchronized RGB/RGB-D + LiDAR + poses + calibration | image detections can become map-frame 3D observations | **Missing as a fixed dataset** |
| D3 persistent semantic map | multi-frame association + map identity + object history | desk/box/room nodes persist across observations | **Design exists; end-to-end evidence missing** |
| D4 task grounding | semantic query + native preview plan | “go to the box near the desk” becomes a checked candidate | **Offline symbolic flow exists; visual map input missing** |
| D5 field navigation | native map activation + localization + motion evidence | the robot executes and terminates safely | **Separate product gate; not proven by semantic data alone** |

## What exists now

The reproducible inventory command is:

```powershell
.venv/Scripts/python.exe tools/datasets/semantic_dataset_inventory.py `
  --write artifacts/semantic-dataset-inventory-20261003.json
```

The current repository evidence is:

- `sim/evaluation/data/octoplanner3d/building2_9.pcd`: 853,227 points,
  fields `x y z _`.
- `sim/evaluation/data/octoplanner3d/plaza3_10.pcd`: 1,675,994 points,
  fields `x y z _`.
- `sim/evaluation/data/` contains LiDAR/IMU bags and ROS 2 databases, but no
  RGB, depth, or segmentation image files.
- The simulation catalog already defines front RGB, front depth, MID-360,
  IMU, and calibration transforms in
  `sim/packages/sensor_rigs/doso/thunder_v4/navigation/`.
- The factory-park simulation has semantic truth in
  `sim/packages/worlds/factory_park_hf/generated/semantic-entities.json`.
  This is world truth, not a detector result.

Therefore the exact answer to “does semantic mapping require vision?” is:

- LiDAR alone is enough for **geometry semantics**: support, structure,
  elevation, collision candidates, and unknown-space evidence when the sensor
  model provides rays.
- Vision is not mathematically mandatory for every object classifier; a
  point-cloud-only model can propose open-vocabulary objects.
- For the product query “会议室里的桌子旁边的箱子”, RGB or RGB-D is the
  practical first requirement because it supplies appearance and object
  category evidence. It still needs synchronized timestamps, camera intrinsics,
  LiDAR-camera extrinsics, and a map-frame pose for every observation.

## Dataset ladder

### 1. Keep the existing geometry and SLAM evidence

Use the two PCDs for conversion and global-planner verification. Use the
existing bags/databases for LiDAR/IMU replay and localization diagnostics. Do
not call either group an object-semantic dataset.

Acceptance evidence:

- PCD header and point count are recorded in the inventory JSON.
- PCD conversion produces the native geometric artifact.
- The official global-planner benchmark reports route verdicts separately from
  continuous collision diagnostics.
- LiDAR/IMU replay reports timestamps, point-time availability, and estimator
  status.

### 2. Build one small deterministic multimodal simulation dataset

This is the first missing dataset and the highest-value next step. Use the
existing Thunder V4 navigation sensor rig and factory-park world to record a
short fixed route, not an open-ended data collection job.

Each accepted frame must carry:

```text
session_id
sequence / simulation_time_ns
T_map_base
T_map_lidar
T_map_camera
RGB frame
metric depth frame
LiDAR scan or point packet
IMU sample window
camera intrinsics and depth scale
LiDAR-camera extrinsic
semantic entity truth from the world package
```

The repository already has a camera SHM recording path in
`sim/runtime/recording/camera_payload.py` and a deterministic simulation
recording format in `sim/runtime/recording/timeline.py`. The dataset task is to
make one short recording using those contracts and to export a small manifest
that points to the recorded streams. We should not invent a second recording
format.

Acceptance evidence for D2:

1. The manifest opens and lists every stream.
2. RGB and depth timestamps are paired within the declared tolerance.
3. Every camera frame has intrinsics and frame identity.
4. Every LiDAR frame has a map-frame pose and the declared extrinsic.
5. A known simulated entity projects from an RGB/depth observation into a
   bounded map-frame 3D cluster.
6. The projected cluster overlaps the simulator truth entity within a declared
   position and extent tolerance.

### 3. Add the smallest object-semantic benchmark

Start with three static classes that directly match the product task:

```text
room / meeting_room
table
box
```

Add one distractor object and one relation:

```text
box near table
```

This is enough to prove the data path. More classes before this path works will
make progress harder to see.

Acceptance evidence for D3:

- one object record per truth entity;
- stable `object_id` across at least three views;
- map-bound `map_id`, `content_epoch`, and `geometry_ref`;
- `near` and `inside` relations retain source observation and confidence;
- a reload produces the same semantic query result.

### 4. Connect the semantic query to native planning

Run the fixed instruction:

```text
先去会议室，然后找到桌子旁边的箱子
```

The LLM or rule parser may create symbolic steps. It must not create motion
coordinates. The semantic resolver selects an object candidate; native
`preview_plan` decides whether a valid approach pose and route exist.

Acceptance evidence for D4:

- the query returns the truth-backed `box` candidate;
- the candidate carries the active map identity and frame;
- native preview accepts or rejects it with a concrete reason;
- no preview pass is reported as field motion success.

## Recommended external data for later comparison

The first external dataset has now been fetched outside the repository:

```text
D:\inovxio\datasets\lingtu\tum_rgbd\freiburg1_desk
```

The probe produced `586` one-to-one RGB/depth pairs, with a maximum pairing
error of `19.94 ms`, `2,335` ground-truth poses, and `16,008` valid projected depth
points in the sample frame. Its evidence is
`artifacts/tum-rgbd-freiburg1-desk-20261003/report.json` and
`frame-0000-projection.png`. This is a **D2 geometry-projection partial pass**;
it does not contain LiDAR or object labels. The reproducible probe is
`tools/datasets/tum_rgbd_probe.py`.

The online dataset order is:

| Dataset | Useful evidence | Limitation for LingTu | Decision |
| --- | --- | --- | --- |
| [TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) | RGB, depth, camera trajectory, accelerometer; CC BY 4.0 | no LiDAR and no object labels | **Fetched and probed first** |
| [Replica / ReplicaCAD](https://aihabitat.org/datasets/replica_cad/index.html) | indoor semantic scenes; ReplicaCAD adds object placement and interaction metadata; CC BY 4.0 for ReplicaCAD | not a robot LiDAR recording; render sensors through Habitat | Best accessible indoor semantic source after TUM |
| [3RScan](https://vmnavab26.in.tum.de/3RScan/documentation.php) | aligned RGB-D sequences and reconstructed meshes for indoor object tasks | download requires the dataset terms form | Use for D3 after access is granted |
| [ScanNet](https://www.scan-net.org/ScanNet/) | RGB-D stream, poses, reconstructed mesh, 2D/3D instance semantics | download requires an agreement; no robot LiDAR/IMU | Use for indoor semantic association comparison |
| [KITTI-360](https://www.cvlibs.net/datasets/kitti-360/) | cameras, laser scans, calibration, poses, 2D/3D semantic and instance labels | vehicle domain and CC BY-NC-SA 3.0 | Use only for multimodal projection research |
| [SemanticKITTI](https://www.semantic-kitti.org/) | sequential 10 Hz LiDAR semantic and instance labels | automotive LiDAR, no RGB object evidence | Use for LiDAR-only temporal baseline |
| [nuScenes](https://www.nuscenes.org/) | cameras, LiDAR, IMU/GPS, calibration, 3D boxes and lidar segmentation | non-commercial terms and vehicle domain | Optional sensor-schema comparison |

For a public RGB-D semantic reference, [ScanNet](https://www.scan-net.org/ScanNet/)
provides RGB-D video, camera poses, reconstructions, and instance semantic
segmentations. [TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
is useful for RGB-D pose and synchronization checks. [ScanNet++](https://scannetpp.mlsg.cit.tum.de/scannetpp/)
offers higher-quality indoor scans with registered DSLR and RGB-D streams.
These datasets are comparison or model-development inputs; they do not replace
our robot-specific LiDAR/IMU/extrinsic capture.

## Progress reporting rule

Every future semantic-map update must report exactly four items:

1. dataset ID and modalities;
2. input and output file paths;
3. one visual or JSON evidence artifact;
4. the level it proves (`D1`, `D2`, `D3`, `D4`, or `D5`).

This prevents a successful parser test, a pretty point-cloud picture, and a
running robot from being described as the same kind of progress.
