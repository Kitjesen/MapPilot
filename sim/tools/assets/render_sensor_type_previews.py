"""Author original sensor schematics using shared SVG/Pillow drawing commands.

Run from any directory with an existing Pillow installation; no engine, network,
input images, external fonts, or generated photographs are used. These previews
explain modalities, not particular hardware or simulated measurement results.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Resources/Sensors"
WIDTH, HEIGHT = 480, 240
SCALE = 3
GENERATOR = "sim/tools/assets/render_sensor_type_previews.py"
PALETTE = {
    "background": "#edf0f3",
    "paper": "#f8fafc",
    "ink": "#202933",
    "muted": "#616e7c",
    "grid": "#d4dce4",
    "accent": "#1675ed",
    "soft": "#d9e8fc",
}
SPECS = {
    "rgb": ("RGB CAMERA", "Visible light / R G B", "Camera lens and a visible-light scene with RGB channels."),
    "depth": ("DEPTH", "Range / m", "Near-to-far distance bands and a range ruler in metres."),
    "lidar": ("LIDAR", "Ray returns / m", "Radial ranging rays ending at discrete return points."),
    "imu": ("IMU", "Acceleration + rotation", "Three body axes, acceleration vector and angular-motion arc."),
    "thermal": ("THERMAL", "Temperature / K", "Illustrative temperature grid and a Kelvin scale."),
    "acoustic": ("ACOUSTIC", "Pressure / Pa", "Sound propagation and a pressure waveform over time."),
    "semantic": ("SEMANTIC", "Class + instance IDs", "Region masks with illustrative class and instance identifiers."),
    "gnss": ("GNSS", "Satellite positioning", "Satellite signals intersecting at an illustrative position fix."),
    "truth_odom": (
        "TRUTH ODOM",
        "Simulator reference pose",
        "Reference trajectory and pose axes, not an estimated path.",
    ),
}


class Diagram:
    """Send the same authored primitives to a vector source and a fresh raster."""

    def __init__(self, title: str, subtitle: str, description: str):
        from PIL import Image, ImageDraw

        self.image = Image.new("RGB", (WIDTH * SCALE, HEIGHT * SCALE), PALETTE["background"])
        self.draw = ImageDraw.Draw(self.image)
        self.svg = ET.Element(
            "svg",
            {
                "xmlns": "http://www.w3.org/2000/svg",
                "width": str(WIDTH),
                "height": str(HEIGHT),
                "viewBox": f"0 0 {WIDTH} {HEIGHT}",
                "role": "img",
                "aria-labelledby": "title desc",
            },
        )
        ET.SubElement(self.svg, "title", {"id": "title"}).text = title
        ET.SubElement(self.svg, "desc", {"id": "desc"}).text = description + " Schematic, not measurement data."
        self.rect(0, 0, WIDTH, HEIGHT, PALETTE["background"])
        self.text(22, 16, title, 18, PALETTE["ink"])
        self.text(22, 42, subtitle, 11)
        self.line([(22, 64), (458, 64)], PALETTE["grid"], 1)
        self.text(22, 217, "FUNCTIONAL DIAGRAM", 9)
        self.text(361, 217, "SCHEMATIC", 10, PALETTE["accent"])

    def rect(self, x, y, width, height, fill, stroke=None, weight=1):
        ET.SubElement(
            self.svg,
            "rect",
            {
                "x": str(x),
                "y": str(y),
                "width": str(width),
                "height": str(height),
                "fill": fill,
                "stroke": stroke or "none",
                "stroke-width": str(weight),
            },
        )
        self.draw.rectangle(
            (x * SCALE, y * SCALE, (x + width) * SCALE, (y + height) * SCALE),
            fill=fill,
            outline=stroke,
            width=weight * SCALE,
        )

    def circle(self, x, y, radius, fill, stroke=None, weight=1):
        ET.SubElement(
            self.svg,
            "circle",
            {
                "cx": str(x),
                "cy": str(y),
                "r": str(radius),
                "fill": fill or "none",
                "stroke": stroke or "none",
                "stroke-width": str(weight),
            },
        )
        self.draw.ellipse(
            ((x - radius) * SCALE, (y - radius) * SCALE, (x + radius) * SCALE, (y + radius) * SCALE),
            fill=fill,
            outline=stroke,
            width=weight * SCALE,
        )

    def line(self, points, color=None, weight=2):
        color = color or PALETTE["accent"]
        ET.SubElement(
            self.svg,
            "polyline",
            {
                "points": " ".join(f"{x:.2f},{y:.2f}" for x, y in points),
                "fill": "none",
                "stroke": color,
                "stroke-width": str(weight),
                "stroke-linejoin": "round",
            },
        )
        self.draw.line([(x * SCALE, y * SCALE) for x, y in points], fill=color, width=weight * SCALE)

    def polygon(self, points, fill):
        ET.SubElement(
            self.svg,
            "polygon",
            {
                "points": " ".join(f"{x:.2f},{y:.2f}" for x, y in points),
                "fill": fill,
            },
        )
        self.draw.polygon([(x * SCALE, y * SCALE) for x, y in points], fill=fill)

    def text(self, x, y, value, size=12, color=None):
        from PIL import ImageFont

        color = color or PALETTE["muted"]
        # Pillow's bundled font keeps raster generation independent of OS fonts.
        font = ImageFont.load_default(size=size * SCALE)
        ET.SubElement(
            self.svg,
            "text",
            {
                "x": str(x),
                "y": str(y + size),
                "font-family": "sans-serif",
                "font-size": str(size),
                "fill": color,
            },
        ).text = value
        self.draw.text((x * SCALE, y * SCALE), value, font=font, fill=color, anchor="lt")

    def arrow(self, start, end, color=None, weight=2):
        color = color or PALETTE["accent"]
        self.line([start, end], color, weight)
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        head = [end] + [
            (end[0] - 8 * math.cos(angle + offset), end[1] - 8 * math.sin(angle + offset)) for offset in (-0.45, 0.45)
        ]
        self.polygon(head, color)

    def arc(self, x, y, radius, begin, end):
        points = [
            (
                x + radius * math.cos(math.radians(begin + (end - begin) * i / 32)),
                y + radius * math.sin(math.radians(begin + (end - begin) * i / 32)),
            )
            for i in range(33)
        ]
        self.line(points)
        self.arrow(points[-2], points[-1])

    def export(self):
        from PIL import Image

        output = io.BytesIO()
        self.image.resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS).save(output, format="PNG")
        return ET.tostring(self.svg, encoding="utf-8", xml_declaration=True), output.getvalue()


def draw_modality(modality: str) -> Diagram:
    """Compose an original diagram of what the modality means, not its brand."""
    d = Diagram(*SPECS[modality])
    blue, soft, grid = (PALETTE[key] for key in ("accent", "soft", "grid"))
    paper, ink = PALETTE["paper"], PALETTE["ink"]
    if modality == "rgb":
        d.polygon([(166, 137), (285, 91), (285, 189)], soft)
        d.rect(44, 106, 110, 65, paper, ink, 2)
        d.rect(61, 94, 37, 12, grid)
        d.circle(141, 138, 29, paper, ink, 2)
        d.circle(141, 138, 19, soft, blue, 3)
        d.circle(141, 138, 7, blue)
        d.rect(286, 85, 164, 108, paper, grid)
        d.circle(414, 104, 8, "#efbd63")
        d.polygon([(295, 176), (336, 124), (367, 157), (391, 132), (443, 176)], "#a9bed8")
        for x, color, label in [(304, "#ce6c70", "R"), (348, "#659a83", "G"), (392, blue, "B")]:
            d.rect(x, 182, 30, 20, color)
            d.text(x + 10, 185, label, 11, "#ffffff")
    elif modality == "depth":
        colors = ["#cfdef2", "#a5c8f1", "#6aa4ed", "#1675ed"]
        for index, color in enumerate(colors):
            d.rect(40 + index * 78, 82, 76, 108, color)
        d.text(52, 93, "FAR", 11, ink)
        d.text(281, 93, "NEAR", 11, "#ffffff")
        d.line([(61, 177), (129, 126), (201, 148), (272, 111), (334, 131)], paper, 3)
        d.arrow((404, 95), (404, 185), ink)
        for y, label in [(94, "8 m"), (134, "4 m"), (178, "0 m")]:
            d.line([(396, y), (412, y)], ink, 1)
            d.text(422, y - 6, label, 11)
    elif modality == "lidar":
        for radius in (35, 65, 97):
            # Upper scan-sector range guides, not a hardware silhouette.
            points = [
                (162 + radius * math.cos(math.radians(a)), 181 + radius * math.sin(math.radians(a)))
                for a in range(195, 346, 3)
            ]
            d.line(points, grid, 1)
        for angle, radius in zip(range(198, 345, 18), (99, 83, 105, 91, 108, 87, 101, 111, 95)):
            end = (162 + radius * math.cos(math.radians(angle)), 181 + radius * math.sin(math.radians(angle)))
            d.line([(162, 181), end], blue, 1)
            d.circle(*end, 4, blue)
        d.circle(162, 181, 9, paper, ink, 2)
        d.text(310, 100, "EMIT / RETURN", 12, ink)
        for i in range(4):
            d.circle(314 + i * 32, 138 - (i % 2) * 15, 4, blue)
        d.text(310, 168, "Discrete ranges", 11)
        d.text(310, 186, "Point cloud / m", 11)
    elif modality == "imu":
        center = (145, 158)
        d.polygon([(145, 126), (175, 143), (145, 162), (115, 143)], soft)
        d.polygon([(115, 143), (145, 162), (145, 188), (115, 169)], "#becfe4")
        d.polygon([(145, 162), (175, 143), (175, 169), (145, 188)], "#91b6e9")
        for end, label, pos in [
            ((230, 158), "X", (235, 151)),
            ((92, 197), "Y", (77, 188)),
            ((145, 83), "Z", (153, 80)),
        ]:
            d.arrow(center, end)
            d.text(*pos, label, 12, blue)
        d.arc(145, 154, 57, 205, 310)
        d.arrow((309, 151), (429, 151))
        d.text(310, 93, "BODY FRAME", 12, ink)
        d.text(310, 119, "a / m/s^2", 13)
        d.text(310, 171, "Angular rate / rad/s", 11)
    elif modality == "thermal":
        colors = ["#dae7f5", "#a7c9ed", "#6d9fe0", "#dba584", "#c87558"]
        for row in range(5):
            for col in range(8):
                index = max(0, 4 - int(math.hypot(col - 4, row - 2)))
                d.rect(39 + col * 29, 81 + row * 23, 26, 20, colors[index])
        for index, color in enumerate(colors):
            d.rect(320, 181 - index * 20, 20, 20, color)
        d.text(351, 96, "330 K", 12, ink)
        d.text(351, 178, "280 K", 12, ink)
        d.text(320, 77, "TEMPERATURE", 11)
        d.text(43, 198, "Illustrative temperature field", 10)
    elif modality == "acoustic":
        d.circle(64, 142, 9, blue)
        for radius in (27, 44, 61):
            points = [
                (64 + radius * math.cos(math.radians(a)), 142 + radius * math.sin(math.radians(a)))
                for a in range(-65, 66, 4)
            ]
            d.line(points, blue, 2)
        d.arrow((173, 148), (451, 148), grid, 1)
        d.arrow((181, 194), (181, 88), ink, 1)
        points = [
            (191 + i, 148 - 33 * math.sin(i * math.pi / 30) * (0.65 + 0.35 * math.sin(i / 74))) for i in range(249)
        ]
        d.line(points, blue, 2)
        d.text(193, 81, "Pressure / Pa", 11, ink)
        d.text(398, 181, "Time / s", 11)
        d.text(39, 193, "Sound wave", 10)
        d.text(166, 145, "0", 9)
    elif modality == "semantic":
        d.rect(40, 83, 235, 118, paper, grid)
        d.rect(41, 160, 233, 40, "#d0d9e4")
        d.rect(59, 119, 68, 58, "#a7c9ed")
        d.polygon([(175, 104), (204, 138), (197, 182), (154, 182), (147, 138)], "#779bcb")
        d.text(69, 136, "C2 / I7", 11, ink)
        d.text(151, 149, "C5 / I3", 11, "#ffffff")
        d.text(48, 185, "C1 / I0", 10, ink)
        d.text(305, 95, "REGION MASKS", 12, ink)
        for y, color, label in [
            (128, "#d0d9e4", "1  Ground"),
            (153, "#a7c9ed", "2  Object"),
            (178, "#779bcb", "5  Person"),
        ]:
            d.rect(306, y, 13, 13, color)
            d.text(329, y, label, 11)
    elif modality == "gnss":
        d.line([(66, 186), (441, 186)], grid, 1)
        for x, y in [(99, 101), (239, 86), (379, 101)]:
            d.line([(x, y + 13), (239, 173)], "#a7c9ed", 1)
            d.rect(x - 37, y - 9, 23, 18, soft, blue)
            d.rect(x + 14, y - 9, 23, 18, soft, blue)
            d.rect(x - 10, y - 13, 20, 26, paper, ink)
            d.line([(x - 25, y - 8), (x - 25, y + 8)], blue, 1)
            d.line([(x + 25, y - 8), (x + 25, y + 8)], blue, 1)
        d.circle(239, 173, 16, None, blue, 2)
        d.circle(239, 173, 5, blue)
        d.line([(216, 173), (262, 173)], blue, 1)
        d.line([(239, 150), (239, 196)], blue, 1)
        d.text(294, 168, "POSITION FIX", 11, ink)
        d.text(42, 196, "Satellite signals / position", 10)
    elif modality == "truth_odom":
        for x in range(48, 430, 32):
            d.line([(x, 81), (x, 201)], grid, 1)
        for y in range(89, 202, 28):
            d.line([(48, y), (430, y)], grid, 1)
        path = [(67, 183), (114, 173), (160, 183), (209, 155), (247, 118), (296, 111), (345, 132)]
        d.line(path, blue, 3)
        for point in path[:-1]:
            d.circle(*point, 4, paper, blue, 2)
        d.polygon([(360, 136), (333, 117), (336, 139)], blue)
        d.arrow((345, 132), (395, 132), ink, 1)
        d.arrow((345, 132), (345, 86), ink, 1)
        d.text(399, 126, "X", 11, ink)
        d.text(351, 83, "Y", 11, ink)
        d.text(263, 180, "REFERENCE / NOT ESTIMATE", 10, ink)
    return d


def generate(output_dir: Path = DEFAULT_OUTPUT_DIR) -> list[Path]:
    """Render the full set in memory before publishing any output files."""
    files = {}
    images = {}
    for modality, (_, _, description) in SPECS.items():
        svg, png = draw_modality(modality).export()
        files[f"{modality}.svg"] = svg
        files[f"{modality}.png"] = png
        images[modality] = {
            "path": f"{modality}.png",
            "source": f"{modality}.svg",
            "width": WIDTH,
            "height": HEIGHT,
            "classification": "schematic",
            "measurement": False,
            "depicts": description,
        }
    manifest = {
        "meta": {
            "version": 1,
            "root": ".",
            "generator": GENERATOR,
            "classification": "schematic",
            "measurement": False,
            "method": "original_code_authored_svg_and_pillow_primitives",
            "palette": PALETTE,
        },
        "images": images,
    }
    files["manifest.json"] = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    rows = "\n".join(f"| `{key}.png` | `{key}.svg` | {value[2]} |" for key, value in SPECS.items())
    sources = f"""# Sensor type preview sources

Classification: **schematic**, not measurement data. All nine diagrams are
original code-authored functional illustrations; they are not photographs,
hardware/product renders, AI-generated imagery, or edits of existing images.
They do not demonstrate sensor availability, accuracy, calibration or runtime
readiness. Grids, traces, ranges, temperatures and IDs are illustrative only;
semantic IDs are not a project label taxonomy. Truth odometry depicts a simulator
reference pose, not a physical sensor or localization estimate.

## Authorship and reproduction

Source of truth: `{GENERATOR}`. No third-party image assets or external image
sources were used. The bundled SVGs retain editable vector geometry and text.
The PNGs are drawn from the same primitives on new Pillow canvases at 3x size,
then downsampled to 480 x 240. Pillow's bundled default font is used for raster
labels; SVG text uses the viewer's sans-serif font. No CairoSVG is required.
No UE, Blender, Tripo, network access, paid services or dependency installation
is involved. Existing Pillow is required to regenerate; pytest checks the pack.

Run from the repository root using an existing Python with Pillow:

```console
python {GENERATOR}
python {GENERATOR} --output-dir <directory>
python -m pytest tests/sim/test_sensor_type_previews.py -q
```

The light-gray background, dark ink and blue accent follow the selector's
visual direction in `SLingTuSimRobotMapPicker.cpp`; modality-specific colors
encode RGB channels, temperature bands and semantic regions only.
`manifest.json` classifies every 480 x 240 PNG/SVG pair as schematic and
explicitly sets `measurement` to false. UI integration belongs to the caller.

## Generated pairs

| PNG | Bundled SVG source | Meaning |
| --- | --- | --- |
{rows}
"""
    files["SOURCES.md"] = sources.encode("utf-8")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (output_dir / name).write_bytes(data)
    return [output_dir / name for name in files]


def main() -> None:
    """Write the nine preview pairs and provenance documents."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    for path in generate(args.output_dir):
        print(path)


if __name__ == "__main__":
    main()
