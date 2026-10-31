"""Check nominal M3 bores and lid-off tool access against the generated mesh."""

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def vertical_hits(triangles, x, y):
    selected = triangles[
        (triangles[:, :, 0].min(axis=1) <= x)
        & (triangles[:, :, 0].max(axis=1) >= x)
        & (triangles[:, :, 1].min(axis=1) <= y)
        & (triangles[:, :, 1].max(axis=1) >= y)
    ]
    a, b, c = selected[:, 0], selected[:, 1], selected[:, 2]
    e, f = b - a, c - a
    det = e[:, 0] * f[:, 1] - e[:, 1] * f[:, 0]
    valid = np.abs(det) > 1e-10
    a, e, f, det = a[valid], e[valid], f[valid], det[valid]
    u = ((x - a[:, 0]) * f[:, 1] - (y - a[:, 1]) * f[:, 0]) / det
    v = (e[:, 0] * (y - a[:, 1]) - e[:, 1] * (x - a[:, 0])) / det
    valid = (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
    return np.unique(np.round((a[:, 2] + u * e[:, 2] + v * f[:, 2])[valid], 5))


def check(directory=HERE):
    p = json.loads((directory / "parameters.json").read_text())
    mesh = np.load(directory / "box.npz")
    triangles = mesh["vertices"][mesh["faces"]]
    output = []
    for sign, pad_name in ((1, "left_pad"), (-1, "right_pad")):
        center_y = sign * p["head_pitch_mm"] / 2
        pad = np.load(directory / f"{pad_name}.npz")
        pad_triangles = pad["vertices"][pad["faces"]]
        # Check center and the M3 shank boundary through both printed parts.
        for radius in (0, 1.5):
            for angle in np.linspace(0.031, 2 * np.pi + 0.031, 16, endpoint=False):
                x, y = radius * np.cos(angle), center_y + radius * np.sin(angle)
                if len(vertical_hits(triangles, x, y)) or len(vertical_hits(pad_triangles, x, y)):
                    raise ValueError(f"{pad_name}: M3 shank intersects printed material")
        seats = []
        for angle in np.linspace(0.031, 2 * np.pi + 0.031, 16, endpoint=False):
            hits = vertical_hits(triangles, 3.5 * np.cos(angle), center_y + 3.5 * np.sin(angle))
            if len(hits) != 2:
                raise ValueError(f"{pad_name}: washer support or top access is interrupted: {hits}")
            seats.append(float(hits[-1]))
        if np.ptp(seats) > p["mesh_pitch_mm"]:
            raise ValueError(f"{pad_name}: washer seat is not level")
        # Sample a 12 mm diameter straight tool corridor with the lid removed.
        for radius in (2, 4, 6):
            for angle in np.linspace(0.031, 2 * np.pi + 0.031, 24, endpoint=False):
                hits = vertical_hits(triangles, radius * np.cos(angle), center_y + radius * np.sin(angle))
                if len(hits) and hits[-1] > max(seats) + p["mesh_pitch_mm"]:
                    raise ValueError(f"{pad_name}: lid-off tool corridor blocked")
        pad_hits = vertical_hits(pad_triangles, 3.5, center_y)
        output.append(
            {
                "part": pad_name,
                "nominal_center_xy_mm": [0, center_y],
                "m3_shank_sampled_clear": True,
                "washer_seat_z_mm": round(float(np.mean(seats)), 3),
                "lid_off_tool_corridor_sampled_diameter_mm": 12,
                "seat_to_body_surface_nominal_mm": round(float(np.mean(seats)) - float(pad_hits.min()), 3),
            }
        )
    report = {
        "status": "nominal_mesh_access_checked_not_physical_interface_verification",
        "mounting": output,
        "physical_interface_pending": "Go2 hole spacing, thread size, usable depth and load-bearing surface",
    }
    (directory / "fastener_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    check()
