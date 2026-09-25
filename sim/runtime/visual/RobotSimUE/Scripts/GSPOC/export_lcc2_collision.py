"""Blender background script: turn the official LCC2 mesh chunks into one FBX proxy."""

import argparse
import pathlib
import sys

import bpy


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)

    sources = sorted(args.mesh_dir.glob("*.ply"))
    if not sources:
        raise FileNotFoundError(f"No LCC2 collision PLY files in {args.mesh_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    objects = []
    for source in sources:
        before = set(bpy.data.objects)
        bpy.ops.wm.ply_import(filepath=str(source))
        imported = [obj for obj in bpy.data.objects if obj not in before and obj.type == "MESH"]
        if len(imported) != 1:
            raise RuntimeError(f"Expected one mesh from {source}, got {len(imported)}")
        objects.extend(imported)

    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    proxy = bpy.context.view_layer.objects.active
    proxy.name = "GSPOC_LCC2_CollisionProxy"
    proxy.data.name = proxy.name
    if len(proxy.data.polygons) < 1000:
        raise RuntimeError("The collision proxy unexpectedly has fewer than 1000 faces")

    blend_path = args.output_dir / "GSPOC_LCC2_CollisionProxy.blend"
    fbx_path = args.output_dir / "GSPOC_LCC2_CollisionProxy.fbx"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
    bpy.ops.export_scene.fbx(
        filepath=str(fbx_path),
        use_selection=True,
        object_types={"MESH"},
        global_scale=1.0,
        apply_unit_scale=False,
        axis_forward="X",
        axis_up="Z",
        bake_space_transform=False,
        add_leaf_bones=False,
    )
    print(
        f"GSPOC_PROXY_EXPORTED vertices={len(proxy.data.vertices)} "
        f"faces={len(proxy.data.polygons)} blend={blend_path} fbx={fbx_path}"
    )


if __name__ == "__main__":
    main()
