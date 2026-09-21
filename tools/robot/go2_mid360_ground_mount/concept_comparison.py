"""Compare the selected concept with an offline render of the actual assembly."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from generate import HERE
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from preview import scene_geometry


def main():
    font_path = "C:/Windows/Fonts/msyh.ttc"
    font_manager.fontManager.addfont(font_path)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=font_path).get_name()
    triangles, colors = scene_geometry()
    # Crop geometry outside the head region before sending it to the renderer.
    keep = triangles[:, :, 0].max(axis=1) > .015
    triangles, colors = triangles[keep], colors[keep]
    fig = plt.figure(figsize=(14, 8), facecolor="#f3f5f6")
    left = fig.add_axes([.025, .14, .44, .74])
    concept = plt.imread(HERE / "concept_v6.png")
    left.imshow(concept[80:975, 15:815])
    left.set_axis_off()
    ax = fig.add_axes([.47, .14, .51, .74], projection="3d", facecolor="#f3f5f6")
    ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolor="none",
                                        antialiased=False, linewidths=0))
    ax.set_xlim(.015, .355)
    ax.set_ylim(-.17, .17)
    ax.set_zlim(-.13, .265)
    ax.set_box_aspect([.34, .34, .395])
    ax.set_proj_type("ortho")
    ax.view_init(16, -40)
    ax.set_axis_off()
    fig.text(.035, .94, "V6B · 按已选概念重建结构", fontsize=24, color="#1c303b", weight="bold")
    fig.text(.04, .89, "已确认概念图", fontsize=13, color="#526c79")
    fig.text(.52, .89, "实际 CAD / 官方传感器模型", fontsize=13, color="#526c79")
    fig.text(.04, .085, "对应：180 mm 长鞍座 / 开放中心 / 圆角双侧肋 / 一体承托 / 前下置 D435i", fontsize=13, color="#334855")
    fig.text(.04, .035, "工程差异：保留 35° 地面观测倾角与真实传感器比例；机身接口、PLA 强度及温升仍待实测。", fontsize=11, color="#946021")
    fig.savefig(HERE / "concept_comparison.png", dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
