# XGRIDS 3DGS minimum loop for RobotSimUE

This local proof uses UE 5.8, the XGRIDS LCC4Unreal 3.4.0 plugin, and the official `lcc2/LCC2.lcc2` sample. The sample has splats for the image and `data/mesh/*.ply` for a separate UE collision proxy. The generated level is `/Game/RobotSim/Maps/GSPOC_XGrids_LCC2`.

1. Download the Win64 UE 5.8 plugin from the [XGRIDS SDK download page](https://developer.xgrids.com/#/download?page=LCC_UNREAL_SDK_UE58), and the sample `Content.zip` from the [official example release](https://github.com/xgrids/3dgs-unreal-example/releases/tag/v1.0). Extract both outside this repository.
2. Mount the extracted plugin directory and the sample's `Content/3DGSData` directory:

   ```powershell
   .\Scripts\GSPOC\stage_xgrids_poc.ps1 `
     -PluginDirectory 'D:\path\to\LCC4Unreal-v3.4.0-win-UE5_8' `
     -SampleDataDirectory 'D:\path\to\example\Content\3DGSData'
   ```

3. Export the mesh chunks to an editable Blender file and FBX. Pass the directory `Content/3DGSData/lcc2/data/mesh` to `--mesh-dir`:

   ```powershell
   & 'D:\path\to\blender.exe' --background --factory-startup --python '.\Scripts\GSPOC\export_lcc2_collision.py' -- --mesh-dir 'D:\path\to\example\Content\3DGSData\lcc2\data\mesh' --output-dir '.\Saved\GSPOC'
   ```

4. Launch the editor with `-ExecCmds="py D:/inovxio/brain/lingtu/sim/runtime/visual/RobotSimUE/Scripts/GSPOC/build_xgrids_poc.py"`. Use the absolute path to the script on another machine. The builder creates the map, camera, two UE depth markers, and a light; it imports the FBX as an invisible collision proxy, takes a screenshot, and samples UE line traces.
5. Inspect `Saved/GSPOC/xgrids_lcc2.png` and `Saved/GSPOC/evidence.json`. Success requires `valid_splat_data` and `line_trace_hit` true, with proxy bounds matching LCC2 centimeters. Failures are recorded in `Saved/GSPOC/error.txt` and the UE log.
6. Reopen the saved map with `-ExecCmds="py <absolute path>/Scripts/GSPOC/verify_xgrids_poc.py"`. `Saved/GSPOC/persisted_evidence.json` must show that the same ray hits with the proxy and misses when that proxy is ignored.

The plugin and sample data are mounted by junctions and ignored by Git. They are not bundled with RobotSimUE. The map is generated under the project's ignored `Content/RobotSim/` tree; rerun the builder after a clean checkout. The baked illumination in the 3DGS scene is capture data. A UE directional light illuminates the marker meshes; this proof does not establish physically correct relighting of the splats. UE collision here is for visual interactions and checks; Thunder's training contacts still belong to MuJoCo.

The official v3.4.0 documentation says LCC2 can use `data/mesh/*.ply` as collision, but this Win64 build repeatedly looked only for `collision.lci` on both official sample maps and this scene. `HaveValidCollisionData()` stayed false. The explicit FBX proxy is a measured workaround; keep `lcc2_internal_collision_data` in the evidence to distinguish the plugin's own collision from UE proxy collision.

For our real site, replace the sample LCC2 and its collision mesh with a captured/exported scene, then verify scale, coordinate frame, traversability, trace hits, and render depth again before connecting robot navigation. See [XGRIDS collision requirements](https://docs.xgrids.com/zh-cn/07-plugin-sdk/01-unreal/v3.4.0/15-collision.html).
