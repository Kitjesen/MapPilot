# LingTu Runtime Graph

Runtime Graph is the readable contract layer for product and endpoint wiring.
It is not the runtime data plane.

## Boundaries

- Real and real-equivalent simulation data planes must use native DDS / C++
  runtime contracts.
- Python Module wiring remains for mock runs, CI harnesses, Gateway/status,
  semantic modules, visualization, and compatibility adapters.
- Runtime Graph YAML declares what a product or endpoint must expose before a
  report can claim real-equivalent behavior.

## Files

- `topics.yaml`: canonical topic, frame, schema, producer, and consumer roles.
- `products/*.yaml`: product modes such as `map`, `nav`, `explore`, and
  `teleop_avoid`.
- `endpoints/*.yaml`: endpoint contracts such as `thunder_field`,
  `mujoco_native_dds`, `sim_mujoco_live`, and `replay`.

## Real-Equivalent Rule

`thunder_field` and `mujoco_native_dds` must share the native contract topics:

```text
/lidar/raw_frame
/imu/raw
/slam/odometry
/slam/map_cloud
/slam/localization_health
/nav/cmd_vel
```

`sim_mujoco_live` is intentionally marked as `module_sim_harness` and
`real_equivalent: false`. It can validate downstream Python Module graph
behavior, but it must not be used as evidence for native field-runtime closure.

## Validation

Use the Python helper in `src/runtime/graph`:

```python
from runtime.graph import assert_runtime_graph_valid

assert_runtime_graph_valid()
```

Focused tests live in `src/runtime/tests/test_runtime_graph_contract.py`.
