# Saved-map evidence comparison

This directory contains offline-only helpers for the fixed three-way map comparison:

1. old prune + old saved-ray replay;
2. new prune + old saved-ray replay;
3. new prune + new saved-ray replay.

The runner copies one complete saved-map snapshot into three candidate directories. It never
edits the source snapshot or an active map. `map.pcd.preclean` is preferred when present; use
`--input-pcd` to select another original PCD from the same snapshot.

Build the evidence tool separately from the Maps runtime:

```powershell
cmake -S tests/maps/tools -B build/maps-evidence-tool `
  -DCMAKE_PREFIX_PATH=C:/opt/octomap-1.10.0-msvc-x64
cmake --build build/maps-evidence-tool --config Release
ctest --test-dir build/maps-evidence-tool -C Release --output-on-failure
```

The evaluation config is JSON. `rois` must keep body clearance and ground support in separate
boxes. `labels` identify physical truth: retained walls/posts use `expected: "occupied"` and a
known residue uses `expected: "not_occupied"`. `plans` hold the unchanged historical blocked
and known-good start/goal requests, including the exact planner options shared by all groups.

```json
{
  "planner_options": {"robot_radius": 0.2},
  "rois": [
    {"name": "start_body", "role": "body_clearance", "min": [0, 0, 0.2], "max": [1, 1, 0.7]},
    {"name": "start_ground", "role": "ground_support", "min": [0, 0, -0.1], "max": [1, 1, 0.15]}
  ],
  "labels": [
    {"name": "measured_wall", "kind": "wall", "expected": "occupied", "point": [1, 2, 0.5]},
    {"name": "measured_post", "kind": "post", "expected": "occupied", "point": [2, 2, 0.5]},
    {"name": "known_ghost", "kind": "residue", "expected": "not_occupied", "point": [3, 2, 0.5]}
  ],
  "plans": [
    {"name": "historical_false_block", "start": [0, 0, 0.3], "goal": [2, 0, 0.3], "expected_ok": true}
  ]
}
```

The coordinates above only document the schema. Do not use them for field acceptance; record
coordinates from the actual `903room_v4_5cm_rays` evidence package. A report remains explicitly
`acceptance_ready: false` when the complete source bundle, truth labels, or fixed plan requests
are missing.

`acceptance_ready` means the offline fixture is complete and its final-group expectations pass.
The report always records `validation_level: "offline"` and
`field_navigation_accepted: false`; it cannot stand in for NX no-motion or supervised field-motion
evidence.
