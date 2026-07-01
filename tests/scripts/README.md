# Script Tests And Manual Smokes

This directory separates automated script tests from manual robot/ROS smoke checks.

| Path | How to run | Purpose |
| --- | --- | --- |
| `test_*.py` | `python -m pytest tests/scripts -q` | Pytest-collected checks for script utilities. |
| `*.sh` | Manual or CI-specific shell invocation | Legacy integration helpers; read each script before running on hardware. |
| `smoke/*.py` | Manual, from repository root | Thunder compatibility smoke checks migrated from the old `scripts/test_*.py` paths. |

The smoke scripts are intentionally not pytest tests: they may require ROS 2,
Gateway, an active map, MCP, or real Thunder access. Keep their paths stable and
document runtime prerequisites in `smoke/README.md`.
