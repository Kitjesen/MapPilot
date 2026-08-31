# ERASOR2 Backend

This folder is the optional GPLv3 backend for running the upstream ERASOR2
algorithm on LingTu saved maps.

It is intentionally separate from `core/` and `prune`:

- `core/` and `prune` are the LingTu-owned default product path.
- `refs/erasor2/` is a GPL-3.0-only backend that may call or link upstream ERASOR2.
- The backend is disabled by default. Enable it only when the distributed
  component can carry GPLv3 obligations.

Upstream source:

```text
third_party/research_nav/ERASOR2
```

Important upstream files:

```text
third_party/research_nav/ERASOR2/src/erasor2/main.cpp
third_party/research_nav/ERASOR2/include/dataloader/dataloader.h
third_party/research_nav/ERASOR2/src/dataloader/
third_party/research_nav/ERASOR2/src/erasor2/erasor2.cpp
```

License:

```text
GPL-3.0-only, inherited from third_party/research_nav/ERASOR2/Licence
```

## Direct Backend Flow

The direct backend runs the ERASOR2 core sequence through the upstream
`run_erasor2` executable:

```text
LingTu map dir
  -> erasor2_stage
  -> ERASOR2-readable HeLiPR/SemanticKITTI-style dataset
  -> run_erasor2 <stage>/erasor2.yaml
  -> <stage>/output/*_estimated.pcd
```

The upstream core functions used by `run_erasor2` are:

```text
ERASOR2::setSubmap()
ERASOR2::updateSteppableRegion()
ERASOR2::detectMovingObjects()
ERASOR2::filterDynamicObjects()
ERASOR2::saveStaticMap()
```

This maps to the LingTu saved-map cleaning stages:

| LingTu stage | ERASOR2 function |
| --- | --- |
| `load` | `DataLoader::getScanAndPose()` |
| `label` | staged `patchwork/*.label` and `hdbscan/*.label` |
| `submap` | `ERASOR2::setSubmap()` |
| `evidence` | `ERASOR2::updateSteppableRegion()` |
| `protect` | steppable-region protection inside update/filter |
| `score` | `ERASOR2::detectMovingObjects()` and `setMovingInstanceScore()` |
| `split` | `ERASOR2::filterDynamicObjects()` |
| `save` | `ERASOR2::saveStaticMap()` |

## Build

Default build does not include this backend:

```bash
bash scripts/build/build_prune.sh
```

Enable the GPL backend explicitly:

```bash
LINGTU_PRUNE_ERASOR2=ON bash scripts/build/build_prune.sh
```

This requires the upstream ERASOR2 dependencies on the build host: Eigen3, PCL,
Boost filesystem/system, OpenCV, OpenMP, and yaml-cpp. Robot builds use
`refs/erasor2/rerun_stub` by default so S100P does not need to download the upstream
rerun SDK from GitHub. Set `-DLINGTU_ERASOR2_USE_RERUN_STUB=OFF` only when
building an interactive visualization binary with network access.

## Run

```bash
build/prune/erasor2_clean \
  --map-dir /var/lib/lingtu/maps/<map> \
  --stage-dir /tmp/erasor2_stage/<map> \
  --overwrite
```

Use `--skip-run` to stage data only and print the exact command that would run
upstream ERASOR2.
