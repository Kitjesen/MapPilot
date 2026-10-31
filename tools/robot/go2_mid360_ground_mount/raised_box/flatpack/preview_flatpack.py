"""Render the actual generated assembled and flat-print meshes."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import generate_flatpack as geometry

HERE = Path(__file__).resolve().parent
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
names = ["left_side", "right_side", "base", "lid", "left_foot", "right_foot"]
colors = ["#1774ba", "#2892ce", "#f1b24a", "#6399c4", "#5c6675", "#5c6675"]
geometry.PITCH = 1.0
display = {}
display["left_side"] = geometry.mesh_field(geometry.side_field, [(-56,67),(33,51),(-8,93)])
display["right_side"] = display["left_side"].copy()
display["right_side"].apply_transform(np.diag([1,-1,1,1]))
display["base"] = geometry.mesh_field(geometry.base_field, [(-56,56),(-51.5,51.5),(-10,-4)])
for name in names[3:]:
    data = np.load(HERE / f"{name}.npz")
    display[name] = trimesh.Trimesh(data["vertices"], data["faces"], process=False)
fig = plt.figure(figsize=(14, 7), facecolor="#f5f7fa")
ax = fig.add_subplot(121, projection="3d")
for name, color in zip(names, colors):
    v, f = display[name].vertices, display[name].faces
    ax.add_collection3d(Poly3DCollection(v[f], facecolor=color, linewidth=0, rasterized=True))
ax.set(xlim=(-65, 80), ylim=(-65, 65), zlim=(-40, 110), title="装配形态：侧板＋底板＋顶板＋贴合垫座")
ax.set_box_aspect((145, 130, 150))
ax.view_init(elev=24, azim=-53)
ax.set_xlabel("X / mm"); ax.set_ylabel("Y / mm"); ax.set_zlabel("Z / mm")
ax = fig.add_subplot(122, projection="3d")
positions = [(10, 140), (155, 140), (155, 10), (10, 10), (15, 265), (165, 265)]
for name, color, (x, y) in zip(names, colors, positions):
    m = display[name].copy()
    rotation = {"left_side": np.array([[1,0,0],[0,0,1],[0,-1,0]]),
                "right_side": np.array([[1,0,0],[0,0,-1],[0,1,0]]),
                "lid": geometry.ROT.T,
                "left_foot": np.diag([1,-1,-1]), "right_foot": np.diag([1,-1,-1])}.get(name, np.eye(3))
    transform = np.eye(4); transform[:3,:3] = rotation
    m.apply_transform(transform); m.apply_translation(-m.bounds[0])
    v = m.vertices + [x, y, 0]
    ax.add_collection3d(Poly3DCollection(v[m.faces], facecolor=color, linewidth=0, rasterized=True))
    ax.text(x + 30, y + 10, 20, name, fontsize=8)
ax.set(xlim=(0, 300), ylim=(0, 310), zlim=(0, 35), title="六件平放：局部接头加厚，最高约16 mm")
ax.set_box_aspect((300, 310, 65))
ax.view_init(elev=65, azim=-90)
fig.suptitle("GO2 / MID-360  螺钉拼装快打版", fontsize=22, fontweight="bold", color="#17324d")
fig.text(.5, .035, "保留雷达名义位置与35°角度 · B3蓝色PLA · 试装原型，承载与实际机身接口未验证", ha="center", fontsize=11)
fig.savefig(HERE / "preview.png", dpi=140, bbox_inches="tight")
