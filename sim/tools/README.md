# Simulation Tools

`sim/tools/` contains offline authoring, asset conditioning, generation,
inspection, and toolchain helpers.

## Contents

- `assets/`: robot and world asset conditioning, conversion, projection, and
  Unreal recipe builders.
- `worlds/`: deterministic FactoryPark, Forest, and OpenField generators plus
  Blender authoring helpers.
- `planning/`: OctoPlanner3D route visualization.
- `toolchains/`: pinned Windows and Unreal build discovery and execution tools.
- [`game_asset_review_catalog.py`](game_asset_review_catalog.py): compile asset
  review catalogs consumed by the UE asset browser.
- [`game_selection_catalog.py`](game_selection_catalog.py) and
  [`game_selection_launcher.py`](game_selection_launcher.py): compile the UE
  selection catalog and hand the selected session to its runtime owner.
- [`sensor_selection.py`](sensor_selection.py): build sensor selections and write
  SessionSpec inputs for ProductControl.
- [`livox_stream_sim.py`](livox_stream_sim.py): provide synthetic Livox LTU1 data
  for driver process integration tests.
- [`matrix_reference.py`](matrix_reference.py): offline staging of external
  MATRiX releases for [reference research](../../research/matrix/README.md).
  It is not a simulation runtime dependency.

## Entry points

```powershell
python -m sim.tools.worlds.factory_park_hf.generate --help
python -m sim.tools.worlds.open_field_hf.generate --help
python -m sim.tools.planning.octoplanner3d_route_viz --help
python -m sim.tools.game_selection_launcher --help
```

## Boundary

Tools may create or transform checked-in or staged assets, but they are not
simulation runtime dependencies. Runtime code must not import authoring or DCC
helpers, and tool output becomes authoritative only through the owning package
and its normal qualification path.
