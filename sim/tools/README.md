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
- `game_selection_*.py` and `livox_stream_sim.py`: local selection, launch, and
  synthetic stream utilities.

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
