"""Generate one CMU collision-correspondence library per robot profile.

The candidate paths are shared.  Only ``correspondences.txt`` changes with
the robot collision radius, so runtime never needs MATLAB or SciPy.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


PROFILES = {
    "go2": 0.55,
    "thunder": 0.75,
}
PATH_FILES = ("startPaths.ply", "pathList.ply", "paths.ply")

VOXEL_SIZE = 0.02
OFFSET_X = 3.2
OFFSET_Y = 4.5
VOXEL_COUNT_X = 161
VOXEL_COUNT_Y = 451


def read_paths(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("r", encoding="ascii") as stream:
        header_lines = 0
        for header_lines, line in enumerate(stream, start=1):
            if line.strip() == "end_header":
                break
        else:
            raise ValueError(f"PLY header is incomplete: {path}")

    values = np.loadtxt(path, skiprows=header_lines, usecols=(0, 1, 3))
    return values[:, :2], values[:, 2].astype(np.int32)


def voxel_points(radius: float) -> np.ndarray:
    index_x = np.repeat(np.arange(VOXEL_COUNT_X), VOXEL_COUNT_Y)
    index_y = np.tile(np.arange(VOXEL_COUNT_Y), VOXEL_COUNT_X)
    x = OFFSET_X - VOXEL_SIZE * index_x
    scale_y = x / OFFSET_X + radius / OFFSET_Y * (OFFSET_X - x) / OFFSET_X
    y = scale_y * (OFFSET_Y - VOXEL_SIZE * index_y)
    return np.column_stack((x, y))


def write_correspondences(
    destination: Path,
    points: np.ndarray,
    path_ids: np.ndarray,
    radius: float,
) -> None:
    tree = cKDTree(points)
    voxels = voxel_points(radius)
    temporary = destination.with_suffix(".tmp")

    with temporary.open("w", encoding="ascii", newline="\n") as stream:
        for begin in range(0, len(voxels), 256):
            matches = tree.query_ball_point(
                voxels[begin : begin + 256],
                radius,
                return_sorted=True,
                workers=-1,
            )
            for offset, point_indices in enumerate(matches):
                ids = path_ids[point_indices]
                if len(ids) > 1:
                    ids = ids[np.concatenate(([True], ids[1:] != ids[:-1]))]
                fields = [str(begin + offset), *(str(value) for value in ids), "-1"]
                stream.write(" ".join(fields) + "\n")

    temporary.replace(destination)


def generate(profile: str, source: Path) -> None:
    radius = PROFILES[profile]
    destination = source / profile
    destination.mkdir(exist_ok=True)
    for filename in PATH_FILES:
        shutil.copyfile(source / filename, destination / filename)

    points, path_ids = read_paths(destination / "paths.ply")
    write_correspondences(
        destination / "correspondences.txt",
        points,
        path_ids,
        radius,
    )
    (destination / "search_radius.txt").write_text(
        f"{radius:.3f}\n", encoding="ascii"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "profiles",
        nargs="*",
        metavar="PROFILE",
    )
    args = parser.parse_args()
    profiles = args.profiles or list(PROFILES)
    unknown = [profile for profile in profiles if profile not in PROFILES]
    if unknown:
        parser.error(f"unknown profile: {', '.join(unknown)}")
    source = Path(__file__).resolve().parent
    for profile in profiles:
        print(f"Generating {profile}: search radius {PROFILES[profile]:.2f} m")
        generate(profile, source)


if __name__ == "__main__":
    main()
