"""Small side-view explanation; dimensional checks remain three-dimensional."""
import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from generate import HERE, ry
from matplotlib import font_manager
from matplotlib.patches import Polygon

p = json.loads((HERE / "parameters.json").read_text())
font_manager.fontManager.addfont("C:/Windows/Fonts/msyh.ttc")
plt.rcParams["font.family"] = font_manager.FontProperties(fname="C:/Windows/Fonts/msyh.ttc").get_name()
plt.rcParams["axes.unicode_minus"] = False
fig, ax = plt.subplots(figsize=(11, 6), facecolor="#f3f5f6")
ax.set_facecolor("#f3f5f6")
origin = np.array(p["target_lidar_xyz_m"]) * 1000
rear = np.array(p["camera_rear_xyz_m"]) * 1000
cam = np.array([[0, 0, -12.5], [25.05, 0, -12.5], [25.05, 0, 12.5], [0, 0, 12.5]]) @ ry(10).T + rear
lidar = np.array([[-32.5, 0, -47], [32.5, 0, -47], [32.5, 0, 13], [-32.5, 0, 13]]) @ ry(35).T + origin
ax.add_patch(Polygon(lidar[:, [0, 2]], fc="#92a5af", ec="#394e58", lw=1.5))
ax.add_patch(Polygon(cam[:, [0, 2]], fc="#354b59", ec="#182d39", lw=1.5))
xs = np.linspace(origin[0], 390, 100)
lower = origin[2] - np.tan(np.radians(42)) * (xs - origin[0])
upper = origin[2] + np.tan(np.radians(17)) * (xs - origin[0])
ax.fill_between(xs, lower, upper, color="#2ba6bb", alpha=.13)
ax.plot(xs, lower, color="#db8037", lw=2, label="前向最低扫描边界 −42°")
ax.plot(origin[0], origin[2], "o", color="#137f91", ms=6)
ax.annotate("MID-360 光学原点", origin[[0, 2]], xytext=(171, 225), arrowprops={"arrowstyle":"-", "color":"#526b78"}, fontsize=12)
ax.annotate("D435i 下置 · 10°\n整个外形位于扫描锥下方", cam[:, [0, 2]].mean(0), xytext=(317, 105), arrowprops={"arrowstyle":"-", "color":"#526b78"}, fontsize=12)
ax.text(165, 66, "前方 →", color="#526b78", fontsize=12)
ax.set(xlim=(155, 395), ylim=(60, 245), xlabel="base X / mm", ylabel="base Z / mm")
ax.set_aspect("equal")
ax.spines[["top", "right"]].set_visible(False)
ax.grid(alpha=.15)
ax.legend(loc="upper right", frameon=False)
fig.suptitle("V6B · 相机与雷达扫描边界", fontsize=20, x=.1, ha="left")
fig.text(.1, .025, "侧视包络示意；遮挡检查覆盖完整三维网格及 USB 插头预留体积。线缆需固定在预留范围内。", fontsize=10, color="#526b78")
fig.tight_layout(rect=(0, .04, 1, .92))
fig.savefig(HERE / "scan_clearance.png", dpi=140)
plt.close(fig)
