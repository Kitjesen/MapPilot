# Dynamic Obstacle Removal

> Strip moving people / objects out of the cumulative map so the saved
> baseline contains only static geometry.

## Problem

The mapping pipeline `Fast-LIO2 鈫?SlamBridge 鈫?Gateway._on_map_cloud` only
applies a 0.15 m voxel dedup. Fast-LIO2 itself is purely geometric and does
not classify static vs dynamic points; the localizer's ICP refinement only
converges, it does not delete points. A person walking down a corridor leaves
a worm-shaped trail in the saved PCD.

Baseline behaviour (v1.7.5):

```python
# gateway_module.py 鈥?mapping branch
combined = np.concatenate([self._map_points, pts])
combined = _voxel_downsample(combined, 0.15)   # dedup, no dynamic removal
self._map_points = combined
```

## Method Survey

| Method | Source | Mode | Per-frame | Indoor | Integration cost |
|--------|--------|------|-----------|--------|------------------|
| Dynablox | ETH-ASL RA-L'23 | online | 58 ms | strong | high (ROS1 only) |
| **DUFOMap** | **KTH RA-L'24** | **online + offline** | **0.06 ms** | medium (sparse LiDAR) | medium (build from source on aarch64) |
| BeautyMap | HKUST RA-L'24 | offline only | medium | strong | medium |
| ERASOR | KAIST ICRA'21 | offline | medium | medium (outdoor bias) | medium |
| Hit-count voting | ours | online | < 1 ms | medium | trivial |

**Picked combination:** hit-count voting for the live view + DUFOMap as a save-time
refinement. Dynablox is ROS1-only; porting to ROS2 Humble on S100P is not
worth the engineering cost.

## Phase 1 鈥?Hit-count voting (live)

In `gateway_module.py:_on_map_cloud` (mapping branch) we keep an observation
counter per voxel:

```python
voxel_hits: Dict[Tuple[int, int, int], int]

# per incoming frame
for voxel_key in voxelize(pts):
    voxel_hits[key] += 1

# at query / publish
stable = {k for k, c in voxel_hits.items() if c >= MIN_HITS}
```

Tunables:

- `MIN_HITS = 3` (env `LINGTU_MAP_MIN_HITS`). Walls get hit 30-100 times in a
  typical building loop; a person walking past leaves 1-3 hits per voxel.
- `voxel_size = 0.15 m` (matches the existing dedup grid).

Limitations:

- Needs the robot to move 鈥?standing-still mapping accumulates no votes.
- A person standing still for a long time becomes "static" by this measure.
- Slow movers (e.g. a meandering dog) leak through.

Live SSE point-cloud stream uses this filter; saved PCD goes through Phase 2
on top.

## Phase 2 鈥?DUFOMap save-time refinement (shipped)

We do not depend on the Python `dufomap` package 鈥?there is no aarch64 wheel
on PyPI and building the binding in-process is fragile. Instead we ship a
subprocess wrapper.

### Inputs

DUFOMap needs **per-frame poses + per-frame scans** for ray-casting; a fused
`map.pcd` is not enough. PGO already dumps both:

- `<map>/patches/*.pcd` 鈥?body-frame keyframe scans (~1500 pts each)
- `<map>/poses.txt` 鈥?`patch.pcd tx ty tz qw qx qy qz` per line

`src/nav/services/dynamic_filter.py:refilter_map` does:

1. Parse `poses.txt`.
2. For each patch: read points, rewrite the PCD `VIEWPOINT` header to the
   patch pose.
3. Stage in a temp dir as `<tmp>/pcd/*.pcd`.
4. `subprocess.run(dufomap_run, tmp, config.toml, timeout=300s)`.
5. Backup `map.pcd 鈫?map.pcd.predufo`, then atomic `os.replace` into
   `map.pcd`. If the backup itself fails, the function aborts and leaves the
   original PCD intact.

### Integration points

Both save paths run Phase 2 unconditionally:

| Caller | File |
|--------|------|
| Web button `POST /api/v1/map/save` | `src/gateway/gateway_module.py` |
| MCP / programmatic save | `src/nav/services/maps.py:_map_save` |

### Build script

`scripts/build_dufomap.sh` is idempotent on a bare aarch64 Ubuntu 22.04:

- `apt install libtbb-dev liblz4-dev` (with a wget fallback for `liblzf-dev`
  when the campus mirror flakes).
- `git clone --recursive`.
- Patches three UFOMap headers to gate `#include <immintrin.h>` behind
  `__BMI2__` (the upstream headers assume x86).
- `cmake -DCMAKE_BUILD_TYPE=Release && make`.
- Output binary: `~/src/dufomap/build/dufomap_run`.

### LingTu DUFOMap config

Default DUFOMap config is tuned for KITTI-class 64-line LiDAR. Livox Mid-360
is sparse and non-repetitive, so we ship `config/dufomap.toml` with looser
thresholds:

| Parameter | Upstream | LingTu |
|-----------|----------|--------|
| `inflate_unknown` | 1 | **2** |
| `inflate_hits_dist` | 0.2 | **0.15** |
| `min_range` | 0.2 | **0.3** (clip the dog's own legs) |
| `max_range` | -1 | **30.0** (Livox far-field is noisy) |

### Measured cost (S100P aarch64)

105 patches 脳 ~1600 pts (`corrected_20260406_224020`):

- DUFOMap (OpenMP) total: ~0.6 s.
- Python wrapper overhead (repack + IPC): ~0.1 s.
- Static-only scene drop rate: 0.01 % (expected 鈥?nothing to remove).
- Dynamic scene drop rate: pending real-data benchmark.

## Phase 3 鈥?Live DUFOMap (parked)

Calling DUFOMap per frame inside `_on_map_cloud` would push the live SSE map
through the same filter. Risk: Mid-360 single frames are ~2k points which is
near DUFOMap's 74 % availability lower bound for the published benchmarks.
Won't pursue unless Phase 1 + 2 prove insufficient.

## Acceptance Criteria

- [x] Phase 1 deployed.
- [x] Phase 2 deployed; DUFOMap binary builds clean on aarch64.
- [x] Offline test passes (105 patches, 0.6 s, 鈥?3 pts on a static map).
- [x] LingTu config in effect at save time.
- [ ] Real-world dynamic-scene test 鈥?residual ghost rate 鈮?60 % reduction.
- [ ] Localizer ICP fitness on the new PCD 鈮?baseline.
- [ ] Save-time gateway CPU peak < baseline + 30 %.

## Key Files

| File | Role |
|------|------|
| `src/gateway/gateway_module.py` (`_on_map_cloud`, `/api/v1/map/save`) | Phase 1 voxel voting + Phase 2 entry on the web path |
| `src/nav/services/maps.py:_map_save` | Phase 2 entry on the MCP path |
| `src/nav/services/dynamic_filter.py` | DUFOMap subprocess wrapper, atomic overwrite, backup |
| `scripts/build_dufomap.sh` | Idempotent aarch64 build |
| `scripts/dufomap_offline_test.py` | Standalone offline validator |
| `config/dufomap.toml` | LingTu-tuned DUFOMap config |
| `~/src/dufomap/build/dufomap_run` (S100P) | C++ binary executed via subprocess |

## Environment Variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `LINGTU_MAP_MIN_HITS` | 3 | Phase 1 voxel vote threshold |
| `LINGTU_SAVE_DYNAMIC_FILTER` | 1 | Phase 2 master switch (0 disables) |
| `LINGTU_DUFOMAP_BIN` | `~/src/dufomap/build/dufomap_run` | DUFOMap binary path |
| `LINGTU_DUFOMAP_CONFIG` | `<repo>/config/dufomap.toml` | DUFOMap config path |

## References

- [Dynablox 鈥?ETH-ASL, RA-L 2023](https://github.com/ethz-asl/dynablox)
- [DUFOMap 鈥?KTH-RPL, RA-L 2024](https://github.com/KTH-RPL/dufomap)
- [BeautyMap 鈥?HKUST, RA-L 2024](https://github.com/MKJia/BeautyMap)
- [DynamicMap_Benchmark 鈥?KTH-RPL, ITSC 2023](https://github.com/KTH-RPL/DynamicMap_Benchmark)
