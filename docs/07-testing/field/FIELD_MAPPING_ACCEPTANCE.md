# Field Mapping Acceptance

Status: current field acceptance gate

This page defines the operator loop for proving that a saved map is usable for
navigation. It is stricter than checking whether files exist.

## What The Bad Diagnostic Means

The diagnostic plot is not a product map preview. It compares two data streams:

- `trajectory.txt`: the full SLAM trajectory recorded during the run.
- `poses.txt`: the poses associated with the saved scan patches.

If the full trajectory covers several meters but `poses.txt` covers only a few
centimeters, the saved map is not acceptable. It means the saved patches are
stationary or near-duplicates even though the robot moved.

Example failure from `codex_trace_20260708_103219`:

- Full trajectory: `8.355 m`
- Saved patch path: `0.030 m`
- Keyframe coverage ratio: `0.0036`
- Result: reject the map, even though `map.pcd`, `metadata.json`, and
  `octomap.ot` exist.

## Acceptance Command

Run this on the robot after saving a map:

```bash
cd /opt/lingtu/current
PYTHONPATH=src python3 -m diagnostics.field.map_artifacts \
  <map_name> \
  --require-octomap \
  --json

PYTHONPATH=src python3 -m diagnostics.field.map_acceptance \
  /var/lib/lingtu/maps/<map_name> \
  --plot-out /tmp/<map_name>_field_acceptance.png \
  --json
```

The first command asks native mapd to validate the active artifact semantics.
The second command checks the saved directory's trajectory and patch coverage;
it does not open a second mapd connection or treat a filesystem path as map identity.

The gate requires, by default:

- `map.pcd`
- `metadata.json`
- `octomap.ot`
- `trajectory.txt`
- `poses.txt`
- matching `patches/*.pcd`

It rejects a map when:

- trajectory length is less than `1.0 m`
- fewer than `10` patch poses were saved
- patch pose path length is less than `20%` of the full SLAM trajectory length
- the number of patch files and saved poses differs
- a required file in the inspected saved directory is missing

The optional `--plot-out` image uses one shared coordinate frame. It overlays
the full SLAM trajectory and saved patch poses in the same axes so collapsed
patch coverage cannot be hidden by a zoomed subplot.

## Joint Mapping Procedure

1. Start from a clean localization/mapping session.
2. Confirm services are online:

   ```bash
   systemctl is-active lt-lidar.service lt-slam.service lt-nav.service lt-host.service
   curl -fsS http://localhost:5050/health
   ```

3. Confirm SLAM is tracking and receiving LiDAR:

   ```bash
   python3 - <<'PY'
   import json
   from pathlib import Path
   d = json.loads(Path('/tmp/messages_status.json').read_text())
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
6. Run `python -m diagnostics.field.map_acceptance` on the saved map.
7. Accept the map only if the field gate returns exit code `0`.
8. Then test relocalization against the accepted map and send one short
   navigation goal.
