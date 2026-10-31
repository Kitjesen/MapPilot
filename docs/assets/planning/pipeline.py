"""Draw the offline PCD planning pipeline used by the README examples."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams.update({"font.family": "Arial", "font.size": 10,
                     "svg.fonttype": "none", "savefig.facecolor": "white"})
fig, ax = plt.subplots(figsize=(8.4, 2.5))
fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
ax.set(xlim=(0, 1), ylim=(0, 1))
ax.axis("off")
nodes = [
    (.015, "Source PCD", "Building / Plaza\nXYZ points"),
    (.268, "3D occupancy map", "Occupied cells\nModeled free layers"),
    (.521, "Native A*", "26-neighbor search\nSupport + edge sweep"),
    (.774, "Path + diagnostics", "XYZ route\nGeometry / motion checks"),
]
width, bottom, height = .211, .49, .44
for x, title, detail in nodes:
    ax.add_patch(FancyBboxPatch((x, bottom), width, height,
        boxstyle="round,pad=0.008,rounding_size=0.02", linewidth=.8,
        edgecolor="#a8b8bc", facecolor="#f5f8f8"))
    ax.text(x+width/2, .80, title, ha="center", va="center", weight="bold", color="#182c32")
    ax.text(x+width/2, .63, detail, ha="center", va="center", linespacing=1.5, color="#42565e")
for i in range(3):
    ax.add_patch(FancyArrowPatch((nodes[i][0]+width+.009, .71),
        (nodes[i+1][0]-.012, .71), arrowstyle="-|>", mutation_scale=10,
        linewidth=1, color="#087f73"))
ax.add_patch(FancyBboxPatch((.29, .07), .66, .20,
    boxstyle="round,pad=0.008,rounding_size=0.02", linewidth=.8,
    edgecolor="#a8b8bc", facecolor="white"))
ax.text(.62, .17, "Start / goal  +  robot body, support and clearance settings",
        ha="center", va="center", color="#42565e")
ax.add_patch(FancyArrowPatch((.6265, .28), (.6265, .48),
    arrowstyle="-|>", mutation_scale=10, linewidth=1, color="#087f73"))
out = Path(__file__).resolve().parent
fig.savefig(out / "pipeline.svg")
fig.savefig(out / "pipeline.png", dpi=180)
plt.close(fig)
