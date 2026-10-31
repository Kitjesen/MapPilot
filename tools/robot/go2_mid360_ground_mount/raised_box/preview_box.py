"""Render the generated geometry and a geometric ground-FOV comparison."""

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from assembly_geometry import read_collada
from generate import ASSETS, ry
from preview import simplify_for_preview

font = Path("C:/Windows/Fonts/msyh.ttc")
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams["axes.unicode_minus"] = False
p = json.loads((HERE / "parameters.json").read_text())
report = json.loads((HERE / "geometry_report.json").read_text())
r = ry(p["pitch_deg"])
mount = np.array(p["mount_xyz_m"]) * 1000
optic = np.array(p["optical_xyz_m"]) * 1000
fig = plt.figure(figsize=(16, 10), facecolor="#f4f6f8")
fig.text(0.04, 0.956, "GO2 / MID-360     增高箱式支架 V7", fontsize=24, weight="bold", color="#172a3a")
fig.text(
    0.04,
    0.918,
    "实际生成网格 · 光学中心 +80 mm / 前移 78.6 mm / 前倾 35° · PETG 试装原型",
    fontsize=12,
    color="#566879",
)
colors = {"box": "#216778", "lid": "#ea9f39", "left_pad": "#596979", "right_pad": "#596979"}
for index, exploded in enumerate([False, True]):
    ax = fig.add_axes([0.025 + 0.49 * index, 0.39, 0.46, 0.48], projection="3d")
    mesh_batches, mesh_colors = [], []
    for name in ["box", "lid", "left_pad", "right_pad"]:
        data = np.load(HERE / f"{name}.npz")
        triangles = data["vertices"][data["faces"]].copy()
        if exploded and name == "lid":
            triangles += r @ [0, 0, 28]
        triangles = simplify_for_preview(triangles, cell=1.3)
        mesh_batches.append(triangles)
        mesh_colors.extend([to_rgba(colors[name])] * len(triangles))
    sensor = np.load(HERE.parent / "reference/mid360_official.npz")
    triangles = sensor["vertices"][sensor["faces"]] @ r.T + optic - mount
    if exploded:
        triangles += r @ [0, 0, 48]
    triangles = simplify_for_preview(triangles, cell=1.0)
    mesh_batches.append(triangles)
    mesh_colors.extend([to_rgba("#b7bec3")] * len(triangles))
    triangles = np.concatenate(mesh_batches)
    face_colors = np.array(mesh_colors)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    light = np.array([0.5, -0.8, 1.0])
    light /= np.linalg.norm(light)
    face_colors[:, :3] *= (0.65 + 0.35 * np.abs(normals @ light))[:, None]
    ax.add_collection3d(Poly3DCollection(triangles, facecolor=face_colors, edgecolor="none", rasterized=True))
    if not exploded:
        body = read_collada(ASSETS / "dae/base.dae") * 1000 - mount
        body = simplify_for_preview(body, cell=4)
        ax.add_collection3d(Poly3DCollection(body, facecolor="#cdd4d8", edgecolor="none", alpha=0.32, rasterized=True))
    ax.set_xlim(-85, 105)
    ax.set_ylim(-85, 85)
    ax.set_zlim(-48, 190 if exploded else 145)
    ax.set_box_aspect([190, 170, 238 if exploded else 193])
    ax.view_init(elev=22, azim=-55)
    ax.set_xlabel("前方 X / mm")
    ax.set_ylabel("Y / mm")
    ax.set_zlabel("安装基准 Z / mm")
    ax.set_title("装配 / 半透明机身仅用于比例参考" if not exploded else "顶盖拆分 / 螺钉与线缆未显示", fontsize=12)
    ax.set_facecolor("#f4f6f8")
ax = fig.add_axes([0.075, 0.10, 0.53, 0.22])
old_h = 0.3 + 0.12262
new_h = 0.3 + p["optical_xyz_m"][2]
for label, height, angle, color in [
    ("原装 13°", old_h, 13, "#7a8490"),
    ("只增高 80 mm，仍 13°", new_h, 13, "#c66558"),
    ("增高 80 mm + 前倾 35°", new_h, 35, "#216778"),
]:
    distance = height / math.tan(math.radians(angle + 7))
    ax.plot([0, distance], [height, 0], color=color, lw=2.5, label=f"{label}：{distance:.2f} m")
ax.axhline(0, color="#172a3a")
ax.set_xlim(0, 1.6)
ax.set_ylim(-0.02, 0.58)
ax.set_xlabel("距雷达垂足的前方距离 / m")
ax.set_ylabel("高度 / m")
ax.grid(alpha=0.2)
ax.legend(loc="upper right", fontsize=9)
fig.text(0.65, 0.292, "只抬高不能解决脚下盲区", fontsize=17, weight="bold", color="#172a3a")
fig.text(
    0.65,
    0.12,
    "4 mm 箱壁 / 8 mm 顶盖与底板\n四角 M4 连接 / 内侧斜筋 / 后部出线\n58.5 mm 机身孔距须试片确认\n35°会牺牲后方近地覆盖\n未验证承载、温升与实机贴合",
    fontsize=12,
    linespacing=1.65,
    color="#45596b",
)
fig.text(
    0.04,
    0.035,
    "视场计算：假定 base 离地 0.30 m、平地、无机身/线缆遮挡；表示几何边界，不保证点云密度。",
    fontsize=10,
    color="#657787",
)
fig.savefig(HERE / "preview.png", dpi=150)
plt.close(fig)
with Image.open(HERE / "preview.png") as page:
    page.convert("RGB").save(HERE / "design-sheet.pdf", resolution=150)
