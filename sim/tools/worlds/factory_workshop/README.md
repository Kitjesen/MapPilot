# Workshop navigation map

The exporter samples the world's contact geometry; its output is an offline
synthetic map, not a LiDAR or SLAM recording. The live local collision grid still
comes from MuJoCo LiDAR rays through Mapd.

For a 10 cm OctoMap, use the default 5 cm sampling and **zero support dilation**:

```powershell
python sim/tools/worlds/factory_workshop/export_map.py --output-dir <map-root>/<map-name>
build/maps-windows/Release/lingtu-mapctl.exe build <map-name> --map-root <map-root> --build-mode native_octomap --resolution 0.1 --support-dilation-cells 0 --free-layers-above 3 --free-dilation-cells 1 --frame map --data-source mujoco --slam-source none --mapping-source synthetic_mujoco_collision_geometry
```

The previous 12 cm sampling needed gap filling at 10 cm resolution. Applying
one-cell support dilation to every occupied point also enlarged stair fronts
and railings horizontally. A body-origin route checked against that expanded
map can disagree with the live LiDAR map. Denser source sampling covers the
surface without this extra expansion. Navigation itself still applies the
robot envelope and requires ground support.

The exporter writes bounded point batches and patches the PCD count after the
payload is complete, then replaces `map.pcd`. This avoids holding the full
campus point cloud in memory. Within each batch it retains one original point
per OctoMap voxel (`--resolution`, default `0.1 m`), preserving occupied-cell
coverage while removing duplicate hits before the native map build/load.
Build and qualify a separate map before selecting
it through ProductControl; rebuilding a file does not change a running RunPlan.
