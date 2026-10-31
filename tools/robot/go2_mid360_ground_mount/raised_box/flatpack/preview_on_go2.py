"""Render the actual flatpack assembly on the reference Go2 visual meshes."""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from assembly_geometry import read_collada_batches
from generate import ASSETS, ry
from preview import link_transforms, origin_transform, simplify_for_preview


def main():
    font_manager.fontManager.addfont("C:/Windows/Fonts/msyh.ttc")
    plt.rcParams["font.family"] = "Microsoft YaHei"
    plt.rcParams["axes.unicode_minus"] = False
    batches, colors = [], []

    def add(triangles, color):
        normals = np.cross(triangles[:, 1] - triangles[:, 0],
                           triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
        illumination = .62 + .38 * np.abs(normals @ np.array([.3, -.4, .866]))
        batches.append(triangles)
        colors.append(np.clip(np.asarray(color)[None, :] * illumination[:, None], 0, 1))

    robot = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
    transforms = link_transforms(robot, standing=True)
    cache = {}
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename").removeprefix("package://go2_description/")
            if filename not in cache:
                cache[filename] = read_collada_batches(ASSETS / filename)
            transform = transforms[link.get("name")] @ origin_transform(visual.find("origin"))
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            for triangles, color in cache[filename]:
                local = simplify_for_preview(triangles * scale, cell=.004)
                add(local @ transform[:3, :3].T + transform[:3, 3], color)

    parameters = json.loads((HERE.parent / "parameters.json").read_text())
    mount = np.array(parameters["mount_xyz_m"])
    for name in ("left_side", "right_side", "base", "lid", "left_foot", "right_foot"):
        data = np.load(HERE / f"{name}.npz")
        triangles = simplify_for_preview(data["vertices"][data["faces"]], cell=1.0)
        color = "#238be1" if name != "lid" else "#57a9eb"
        add(triangles / 1000 + mount, to_rgb(color))

    sensor = np.load(HERE.parents[1] / "reference/mid360_official.npz")
    local = simplify_for_preview(sensor["vertices"][sensor["faces"]], cell=1.0) / 1000
    world = local @ ry(parameters["pitch_deg"]).T + parameters["optical_xyz_m"]
    window = local[:, :, 2].mean(axis=1) > -.0074
    add(world[~window], to_rgb("#aab3bc"))
    add(world[window], to_rgb("#202b32"))
    triangles, facecolors = np.concatenate(batches), np.concatenate(colors)

    fig = plt.figure(figsize=(16, 9), facecolor="#f3f5f8")
    fig.text(.04, .94, "Go2 EDU + MID-360  /  蓝色 PLA 拼装支架", fontsize=23,
             weight="bold", color="#18334d")
    fig.text(.04, .885, "按实际零件网格装配 · 雷达前倾 35° · 光学中心较原配置提高 80 mm",
             fontsize=12, color="#52687b")

    def panel(rect, view, limits, title):
        ax = fig.add_axes(rect, projection="3d", facecolor="#f3f5f8")
        inside = np.ones(len(triangles), dtype=bool)
        for axis, (lo, hi) in enumerate(limits):
            inside &= (triangles[:, :, axis].max(axis=1) >= lo)
            inside &= (triangles[:, :, axis].min(axis=1) <= hi)
        ax.add_collection3d(Poly3DCollection(triangles[inside], facecolors=facecolors[inside],
                                            edgecolor="none", linewidth=0, rasterized=True))
        ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1]); ax.set_zlim(*limits[2])
        ax.set_box_aspect([hi-lo for lo, hi in limits])
        ax.set_proj_type("ortho")
        ax.view_init(*view)
        ax.set_axis_off()
        ax.set_title(title, fontsize=12, color="#405d73", pad=0)

    full = [(-.40, .43), (-.25, .25), (-.36, .29)]
    panel([.015, .10, .61, .73], (20, -55), full, "01 / 整机比例")
    panel([.625, .46, .36, .37], (0, -90), full, "02 / 侧面：头部上方前倾安装")
    panel([.625, .075, .36, .35], (20, -55),
          [(.10, .33), (-.075, .075), (.045, .26)], "03 / 安装特写")
    fig.text(.04, .055, "设计装配预览，非实物照片。机身接口尚待试装；螺钉和线缆未显示。",
             fontsize=11, color="#52687b")
    fig.savefig(HERE / "on_go2.png", dpi=150)
    plt.close(fig)
    print(f"Rendered {len(triangles)} display triangles to {HERE / 'on_go2.png'}")


if __name__ == "__main__":
    main()
