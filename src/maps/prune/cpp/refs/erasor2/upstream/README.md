# ERASOR2 Upstream Snapshot

This directory is a local GPL-3.0-only snapshot of the ERASOR2 core files that
matter for LingTu `prune` migration work.

It is not part of the default LingTu `prune` product binary. The default product
path is still `src/maps/prune/cpp/core` plus `prune.cpp`.

## License

These copied files keep the upstream ERASOR2 license:

```text
GPL-3.0-only
```

See `Licence` in this directory. Do not move code from this folder into
`src/maps/prune/cpp/core` unless it is rewritten independently.

## Copied Core Files

```text
src/erasor2/main.cpp              run_erasor2 entrypoint and workflow order
src/erasor2/erasor2.cpp           core submap, evidence, scoring, split, save logic
src/erasor2/Config.cpp            YAML config reader
src/erasor2/grid_map.cpp          grid map helper implementation
src/erasor2/erasor_utils.cpp      utility functions used by the core
src/erasor2/RerunLogger.cpp       visualization adapter
src/dataloader/dataloader.cpp     SemanticKITTI/HeLiPR scan, pose, label loader
include/erasor2/*.hpp|*.h         ERASOR2 core interfaces
include/dataloader/dataloader.h   loader interface
include/rosparam_server.hpp       parameter container
include/tools/*.hpp               utility interfaces
include/dataprocessor/TrajectoryClustering.hpp
```

## LingTu Mapping

Use this folder to read the upstream algorithm shape:

```text
DataLoader::getScanAndPose()
  -> ERASOR2::setScanAndPose()
  -> ERASOR2::setSubmap()
  -> ERASOR2::updateSteppableRegion()
  -> ERASOR2::detectMovingObjects()
  -> ERASOR2::filterDynamicObjects()
  -> ERASOR2::saveStaticMap()
```

The product implementation should translate the ideas into LingTu-owned code in
`src/maps/prune/cpp/core`, not include this snapshot directly.
