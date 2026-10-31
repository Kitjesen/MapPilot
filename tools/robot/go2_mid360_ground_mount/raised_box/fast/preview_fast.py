"""Render the actual quick-fit meshes without changing printable geometry."""

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from generate import ry
from preview import simplify_for_preview

font_manager.fontManager.addfont("C:/Windows/Fonts/msyh.ttc")
plt.rcParams["font.family"] = "Microsoft YaHei"
plt.rcParams["axes.unicode_minus"] = False
p = json.loads((HERE / "parameters.json").read_text())
rotation = ry(p["pitch_deg"])
optic = (np.array(p["optical_xyz_m"]) - p["mount_xyz_m"]) * 1000
fig = plt.figure(figsize=(13, 8), facecolor="#f3f5f8")
fig.text(0.04, 0.935, "GO2 / MID-360   开放框架快打版", fontsize=23, weight="bold", color="#16304b")
fig.text(0.04, 0.885, "蓝色 PLA · 四件同盘 · 保留原安装孔和雷达名义外参", fontsize=13, color="#506379")
for index in range(2):
    ax = fig.add_axes([0.02 + index * 0.49, 0.19, 0.47, 0.64], projection="3d")
    batches, colors = [], []
    for name in ("box", "lid", "left_pad", "right_pad"):
        if index and name == "lid":
            continue
        data = np.load(HERE / f"{name}.npz")
        triangles = simplify_for_preview(data["vertices"][data["faces"]], cell=1.5)
        batches.append(triangles)
        color = "#217bcc" if name == "box" else "#57a4e3" if name == "lid" else "#365d82"
        colors.extend([to_rgba(color)] * len(triangles))
    if not index:
        sensor = np.load(HERE.parents[1] / "reference/mid360_official.npz")
        triangles = simplify_for_preview(sensor["vertices"][sensor["faces"]], cell=1.5) @ rotation.T + optic
        batches.append(triangles)
        colors.extend([to_rgba("#c0c7cd")] * len(triangles))
    triangles = np.concatenate(batches)
    colors = np.array(colors)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    light = np.array([0.3, -0.5, 1.0])
    light /= np.linalg.norm(light)
    colors[:, :3] *= (0.6 + 0.4 * np.abs(normals @ light))[:, None]
    ax.add_collection3d(Poly3DCollection(triangles, facecolor=colors, edgecolor="none"))
    ax.set(xlim=(-70, 90), ylim=(-70, 70), zlim=(-48, 145))
    ax.set_box_aspect([160, 140, 193])
    ax.view_init(elev=25, azim=-55)
    ax.set_xlabel("前方 X / mm")
    ax.set_ylabel("Y / mm")
    ax.set_zlabel("安装基准 Z / mm")
    ax.set_facecolor("#f3f5f8")
    ax.set_title("装配形态" if not index else "拆掉顶盖：斜撑与底部安装孔", fontsize=12)
fig.text(0.04, 0.115, "3 mm 余留壁 · 5 mm 顶盖/底板 · 孔座与交叉斜撑保留 · 几何体积减少约 39%", fontsize=12)
fig.text(0.04, 0.065, "试装原型；孔距、承载与温升未实机验证。打印时间以新版切片为准。", fontsize=11, color="#506379")
fig.savefig(HERE / "preview.png", dpi=130)
plt.close(fig)
