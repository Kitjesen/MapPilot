# DimOS Comparison Notes

Status: design reference, not LingTu acceptance evidence

DimOS is useful to LingTu as an organization and developer-experience
reference. It is not the source of LingTu's field runtime or navigation
algorithms.

## Upstream references

- <https://github.com/dimensionalOS/dimos>
- <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/native/index.md>
- <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/nav_stack.md>
- <https://github.com/dimensionalOS/dimos/blob/main/docs/development/testing.md>

## What to copy

| DimOS strength | LingTu application |
| --- | --- |
| Vendor/model locality | Keep robot configuration under `config/robots/<vendor>/<model>/`. |
| Small model-facing connection surface | Keep Host commands and native adapters under `src/nav/commands/`, `services/`, `skills/`, and `adapters/native/`. |
| One place to find model assets and setup | Keep each model's README, model declaration, sensor profiles, and calibration references together. |
| Comparable real/sim concepts | Keep Product capability names stable while `env=real` and `env=sim` select different process/device implementations. |
| Replay and recording as normal tools | Use LingTu's native MCAP recorder/player and model-specific recording presets. |
| Visible integration tests | Keep explicit native component, MuJoCo Product, no-motion field, and supervised field evidence separate. |

Current robot-locality examples:

```text
config/robots/doso/thunder_v4/
config/robots/unitree/go2/
```

## What not to copy

- Do not introduce a second Python planning or motion-control chain beside
  native `navd`.
- Do not reproduce DimOS package names when LingTu already has an owning
  Product, Host Module, or native service.
- Do not add another benchmark framework around existing native tests, MuJoCo
  acceptance, MCAP, and field gates.
- Do not make ROS a default Product dependency. LingTu's field boundary is
  native typed CycloneDDS.

## Current LingTu mapping

| Concern | Current LingTu owner |
| --- | --- |
| Product and environment choice | `config/runtime_graph/` and `src/lingtu/` |
| Robot/vendor/model configuration | `config/robots/` |
| Host navigation surface | `src/nav/commands/`, `services/`, `skills/`, `inspection/`, `building/` |
| Native navigation | `src/nav/cpp/` |
| Map lifecycle | `src/maps/` and native `mapd` |
| SLAM/localization | `src/localization/` and native `slamd` |
| Recording/replay | `src/native/recording/` |
| Simulation acceptance | `docs/07-testing/simulation/` and `sim/scripts/mujoco/` |

## Comparison rule

Compare user outcomes and evidence levels, not file counts or framework names.
For every claimed capability, identify:

1. the Product and robot model;
2. the current command/data owner;
3. the real or sim environment;
4. the narrowest acceptance report that proves the claim.

The useful DimOS lesson is discoverability. LingTu should keep its stronger
native field ownership while making each robot model and capability just as
easy to locate.
