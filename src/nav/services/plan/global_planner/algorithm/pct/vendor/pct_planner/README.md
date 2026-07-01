# PCT Planner Compatibility Source

This directory keeps the PCT native planner source and tomogram utilities for
compatibility tests and baseline comparisons. It is no longer a ROS package and
must not be used as a runtime orchestration layer.

Current product navigation uses `octoplanner3d` through `GlobalPlannerService`.
PCT is no longer seeded as a product `planner_backend`; use it only through
explicit benchmark or legacy parity tools:

```text
manual/bench command
  -> nav.services.plan.global_planner.algorithm.pct.runtime.api
  -> planner/scripts/planner_wrapper.py
```

Kept here:

- `planner/scripts/planner_wrapper.py`: Python wrapper for native PCT planning
- `planner/lib/`: native planner source/build inputs
- `tomography/`: tomogram generation utilities
- `rsc/`: sample map resources used by compatibility tests

Removed from here:

- ROS `package.xml`
- ROS `launch/`
- ROS node entrypoints such as `global_planner.py`, `pct_planner_astar.py`, and
  `fake_localization.py`

Build the portable Rust optimizer artifacts with:

```bash
bash src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/build_all_platforms.sh
```

The legacy Linux/GTSAM native module build is gated:

```bash
LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 \
  bash src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build.sh
```
