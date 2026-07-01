# TARE Planner (vendored)

Hierarchical exploration planner for ground vehicles — CMU Robotics
Institute (Chao Cao, Hongbiao Zhu, Ji Zhang, Howie Choset). Originally
published as [`caochao39/tare_planner`](https://github.com/caochao39/tare_planner)
branch `humble-jazzy`.

## Why vendored, not submodule

Commit-pinned vendoring avoids submodule churn on S100P, gives us a single
`colcon build` entry for the LingTu tree, and lets us patch upstream
directly (lever-arm fixes, aarch64 tweaks, etc.) without maintaining a
fork.

## License

Redistributed under the BSD license declared in `package.xml`
(`<license>BSD</license>`). See `LICENSE` alongside this file.
Upstream has no standalone `LICENSE` file, but the ROS package metadata
carries the license declaration — this is the canonical BSD source for the
package.

Author contact: Chao Cao <ccao1@andrew.cmu.edu>.

## OR-Tools

Linked against Google OR-Tools 9.x (Apache 2.0). **Not committed to the
repository** — 130+ MB x86-64 binary, plus we need arm64 on S100P. Run

    scripts/build/fetch_ortools.sh

once before the first `scripts/build/build_tare.sh` and the arch-correct
archive will be downloaded into `or-tools/` here.

## Integration into LingTu

  - C++ node runs as a `NativeModule` subprocess, started by
    ``exploration.native_factories.tare_explorer``.
  - Topic contract bridged by
    ``exploration.tare_explorer_module.TAREExplorerModule`` (cyclonedds /
    stub fallback chain). Use ``compat.ros2.tare_bridge`` for rclpy topics.
  - Stack factory ``core.blueprints.stacks.exploration.exploration(backend="tare")``
    wires both together.

## Local modifications

See `PATCHES.md` for a log of every change we make on top of the pinned
upstream snapshot. Keep that list short — prefer upstream-safe wrappers
over in-tree patches.
