# Field Mapping Acceptance

Status: current field acceptance gate

This page defines the operator loop for proving that a saved map is usable for
navigation. It is stricter than checking whether files exist.

## What The Bad Diagnostic Means

The diagnostic plot is not a product map preview. It compares two data streams:

- `trajectory.txt`: the full SLAM trajectory recorded during the run.
- `poses.txt`: the keyframe poses passed to PGO/HBA/map optimization.

If the full trajectory covers several meters but `poses.txt` covers only a few
centimeters, the saved map is not acceptable. It means the optimizer saw
stationary or near-duplicate keyframes even though the robot moved.

Example failure from `codex_trace_20260708_103219`:

- Full trajectory: `8.355 m`
- Optimizer keyframe path: `0.030 m`
- Keyframe coverage ratio: `0.0036`
- Result: reject the map, even though `map.pcd`, `metadata.json`,
  `octomap.ot`, and `tomogram.pickle` exist and pass checksum validation.

## Acceptance Command

Run this on the robot after saving a map:

```bash
cd /opt/lingtu/current
PYTHONPATH=src python3 scripts/gates/saved_map_field_acceptance.py \
  /home/sunrise/data/nova/maps/<map_name> \
  --plot-out /tmp/<map_name>_field_acceptance.png \
  --json
```

The gate requires, by default:

- `map.pcd`
- `metadata.json`
- `octomap.ot`
- `tomogram.pickle`
- `map_optimization.json`
- `trajectory.txt`
- `poses.txt`

It rejects a map when:

- trajectory length is less than `1.0 m`
- fewer than `10` patch poses were saved
- patch pose path length is less than `20%` of the full SLAM trajectory length
- artifact metadata or checksums do not match
- optimization reports failure or degraded input

The optional `--plot-out` image uses one shared coordinate frame. It overlays
the full SLAM trajectory, optimizer input poses, and optimized poses in the same
axes so a collapsed optimizer input cannot be hidden by a zoomed subplot.

## Joint Mapping Procedure

1. Start from a clean localization/mapping session.
2. Confirm services are online:

   ```bash
   systemctl is-active lingtu-livox-dds.service lingtu-slam-dds.service lingtu-nav-dds.service lingtu.service
   curl -fsS http://127.0.0.1:5050/health
   ```

3. Confirm SLAM is tracking and receiving LiDAR:

   ```bash
   python3 - <<'PY'
   import json
   from pathlib import Path
   d = json.loads(Path('/tmp/lingtu_slam_status.json').read_text())
   print(json.dumps({
       'state': d.get('state'),
       'mode': d.get('mode'),
       'lidar_input_hz': d.get('lidar_input_hz'),
       'processed_scan_hz': d.get('processed_scan_hz'),
       'map_loaded': d.get('map_loaded'),
   }, indent=2))
   PY
   ```

4. Move the robot through a short but real path:

   - at least `6-10 m` total motion
   - include at least one turn
   - avoid saving after a long stationary wait

5. Save the map from the UI with a unique name.
6. Run `saved_map_field_acceptance.py` on the saved map.
7. Run the read-only loop shadow audit with a unique report outside the map:

   ```bash
   lt_loop_verify \
     --map /home/sunrise/data/nova/maps/<map_name> \
     --report /tmp/<map_name>-loop-$(date +%s).json
   ```

   A successful `shadow_no_verified_loops` is valid for a route with no return
   visit. `shadow_verified_loops` is diagnostic evidence only; it does not
   enable or apply PGO. In the v3 report, inspect `planar_inliers`,
   `point_to_plane_eigenvalues`, `point_to_plane_weak_mode`, and
   `point_to_plane_condition`; single-wall/corridor sliding must be rejected by
   a point-to-plane rank or condition gate. `information_diagonal` remains zero
   until the full graph tangent conversion is implemented.
8. Accept the map only if the field gate returns exit code `0` and the loop
   audit did not report invalid inputs, provenance mismatch, or a changing map.
9. Then test relocalization against the accepted map and send one short
   navigation goal.

## Fallback Tomogram Requirement

The robot may not have `open3d` or `numba`. In that case the tomogram builder
must still load ASCII/binary PCD through the local PCD reader and use the NumPy
fallback path. A valid fallback run reports `tomogram.pickle` exists and has a
non-empty `data` shape.
