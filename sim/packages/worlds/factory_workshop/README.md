# DOSO factory campus

`factory_workshop@2.0.0` is the installed native MuJoCo campus, promoted from
the reviewed `v8-r4-detail-study` Blender export. It is selected by the Thunder
`default.yaml` and `nav.yaml` Session presets. The dedicated `teleop_avoid`
field and tracking's pedestrian scenario remain separate.

Actual integration results and remaining motion failures:
[2026-09-08 validation](INTEGRATION_V2.md). The truth-mode Product started
successfully, but idle drift and goal-navigation failures remain; it was stopped
after evidence capture, not left running unattended.

## Runtime assets

- Site: 120 × 90 m, including exterior roads, greenery and service areas.
- Factory: 80 × 48 m, a complete second floor at 6 m, roof at 12 m, two stairs.
- Retained machining equipment, workstations, storage, DOSO paint and signs.
- MJCF: `2.0.0/physics/campus.xml`; 50 mesh/lightmap pairs under
  `2.0.0/visual/native/`. Use Git LFS when checking out these assets.
- Contact/LiDAR proxies use hidden geometry group 4; baked visual geometry
  uses group 2. The formal LiDAR mask includes group 4, not visual meshes.
- Spawn: `(61, 16.5, 0.02)`, facing +Y toward the south entrance. The robot's
  Controller and initial standing keyframe are resolved from its existing
  packages, not overridden by the world.

## Formal entry point

Run from the repository root with `src` on `PYTHONPATH`:

```powershell
python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --backend mujoco --viewer
python -m lingtu.control switch nav --robot doso/thunder_v4 --env sim --backend mujoco --local-planner scan --map factory_workshop_v2 --viewer
python -m lingtu.control status --robot doso/thunder_v4 --env sim
python -m lingtu.control stop --robot doso/thunder_v4 --env sim
```

The `nav` command requires a built saved map named `factory_workshop_v2` in
the configured map root. It must not reuse the old industrial-park map. The
current CLI uses the repository's Fast-LIO2 default; changing the world does
not change localization. The campus startup check uses the existing explicit
truth-localization option through the public ProductControl API instead:

```python
from lingtu.control import ProductControl

control = ProductControl(
    robot="doso/thunder_v4", env="sim",
    env_config={"backend": "mujoco", "viewer": True, "localization": "truth"},
)
control.switch("nav", map_name="factory_workshop_v2", local_planner="scan")
```

This is the diagnostic truth path, not Fast-LIO2 qualification. Mapd and Navd
must be built from the same current message contract; an older Mapd binary
can publish collision messages that the newer Navd cannot deserialize.

## Same-source navigation map

```powershell
python -m sim.tools.worlds.factory_workshop.export_map --world-package sim/packages/worlds/factory_workshop/2.0.0/world.package.yaml --output-dir artifacts/factory_workshop/v2-map-source --spacing 0.12
```

This exports `map.pcd` and `source.json` from contact geometry, including the
upper floor and stairs. Put these in a new `factory_workshop_v2` directory
under the saved-map root, then use `lingtu-mapctl build` to generate the
OctoMap and identity metadata before ProductControl activates it. Do not
mark the package ready by hand. The export is **synthetic geometry**, not
SLAM evidence; the diagnostic navigation check uses MuJoCo ground-truth localization.

## Qualification boundary

Installing a world is not a claim of navigation success. Two-storey geometry
does not qualify the current locomotion policy for stairs. Native startup,
flat-ground navigation, stair traversal and Unreal rendering are distinct
checks. This package supplies native MuJoCo assets, not a cooked Unreal level.

Landscape texture attribution: [sources](materials/landscape/SOURCES.md).
The promotion recipe is `sim.tools.worlds.factory_workshop.install_campus`;
it installs a new exact-version package and does not replace an existing one.
