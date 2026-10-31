"""Blender task body for Agent 3D Workbench; retains supplied PBR resources."""

import json
from pathlib import Path

import bpy
from mathutils import Vector


def prepare(workbench):
    source = Path(workbench["inputs"][0])
    output = Path(workbench["output"])
    source_info = json.loads((source.parent / "source.json").read_text(encoding="utf-8"))
    bpy.ops.import_scene.gltf(filepath=str(source))
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    points = [obj.matrix_world @ Vector(corner) for obj in meshes for corner in obj.bound_box]
    lower = [min(p[i] for p in points) for i in range(3)]
    upper = [max(p[i] for p in points) for i in range(3)]
    report = {
        "provider": "Poly Haven",
        "source": source_info,
        "axes": "Blender Z-up metres; GLB Y-up metres",
        "bounds_m": [lower, upper],
        "mesh_objects": [obj.name for obj in meshes],
        "materials": [mat.name for mat in bpy.data.materials],
        "images": [
            {"name": im.name, "width": im.size[0], "height": im.size[1]} for im in bpy.data.images if im.type == "IMAGE"
        ],
        "processing": "glTF import, embedded-texture GLB and FBX export; no decimation or geometry edits",
        "ue_imported": False,
        "physics_qualified": False,
    }
    bpy.ops.file.pack_all()
    bpy.ops.export_scene.fbx(
        filepath=str(output / "model.fbx"),
        object_types={"MESH", "EMPTY"},
        path_mode="COPY",
        embed_textures=True,
        bake_anim=False,
    )
    (output / "intake-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (output / "workbench-result.json").write_text(
        json.dumps(
            {
                "schema": "workbench-result/v1",
                "work_id": "polyhaven-" + source_info["id"],
                "work_title": workbench["params"]["title"],
                "variant": "CC0 / PBR",
                "version": "1.0.0",
                "note": "带贴图源资产；未通过 UE / MuJoCo 交互验收。",
                "primary": "scene.glb",
                "thumbnail": "preview.png",
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    prepare(globals()["workbench"])
