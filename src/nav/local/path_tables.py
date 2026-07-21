"""CMU local planner path table loading."""

from __future__ import annotations

import logging
import os
from pathlib import Path

from nav.local.models import GROUP_NUM, PATH_NUM
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


def local_planner_paths_dir() -> str:
    """Return the shared CMU local_planner paths directory."""
    return os.path.normpath(os.path.join(os.path.dirname(__file__), "paths"))


def _ply_payload_text(path: Path) -> str | None:
    with open(path, "rb") as f:
        raw = f.read()
    pos = raw.find(b"end_header\r\n")
    if pos == -1:
        pos = raw.find(b"end_header\n")
        if pos == -1:
            logger.error("LocalPlanner [cmu_py]: bad %s header", path.name)
            return None
        pos += len(b"end_header\n")
    else:
        pos += len(b"end_header\r\n")
    return raw[pos:].decode("ascii")


def load_cmu_py_paths(paths_dir: str | None = None) -> dict | None:
    """Load pre-computed CMU local planner paths from PLY/text files."""
    pd = Path(paths_dir or local_planner_paths_dir())

    paths_ply = pd / "paths.ply"
    if not paths_ply.exists():
        logger.error("LocalPlanner [cmu_py]: paths.ply not found at %s", paths_ply)
        return None

    text = _ply_payload_text(paths_ply)
    if text is None:
        return None
    paths_data: list[list] = [[] for _ in range(PATH_NUM)]
    for line in text.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        path_id = int(parts[3])
        if 0 <= path_id < PATH_NUM:
            paths_data[path_id].append((x, y, z))
    paths = [
        np.array(pts, dtype=np.float32)
        if pts
        else np.zeros((0, 3), dtype=np.float32)
        for pts in paths_data
    ]

    path_list_text = _ply_payload_text(pd / "pathList.ply")
    if path_list_text is None:
        return None
    group_of_path = np.zeros(PATH_NUM, dtype=np.int32)
    for line in path_list_text.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        path_id = int(parts[3])
        group_id = int(parts[4])
        if 0 <= path_id < PATH_NUM and 0 <= group_id < GROUP_NUM:
            group_of_path[path_id] = group_id

    start_paths_text = _ply_payload_text(pd / "startPaths.ply")
    if start_paths_text is None:
        return None
    start_paths_data: list[list] = [[] for _ in range(GROUP_NUM)]
    for line in start_paths_text.strip().splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        group_id = int(parts[3])
        if 0 <= group_id < GROUP_NUM:
            start_paths_data[group_id].append((x, y, z))
    start_paths = [
        np.array(pts, dtype=np.float32)
        if pts
        else np.zeros((0, 3), dtype=np.float32)
        for pts in start_paths_data
    ]

    correspondences: dict[int, list[int]] = {}
    max_voxel_id = 0
    with open(pd / "correspondences.txt", encoding="utf-8") as f:
        for line in f:
            tokens = line.split()
            if not tokens:
                continue
            voxel_id = int(tokens[0])
            path_ids_for_voxel = [int(t) for t in tokens[1:] if t != "-1"]
            if path_ids_for_voxel:
                correspondences[voxel_id] = path_ids_for_voxel
            max_voxel_id = max(max_voxel_id, voxel_id)

    grid_voxel_num_x = 161
    grid_voxel_num_y = 531
    expected_max = grid_voxel_num_x * grid_voxel_num_y - 1
    if max_voxel_id > expected_max:
        grid_voxel_num_y = (max_voxel_id // grid_voxel_num_x) + 2
        logger.warning(
            "LocalPlanner [cmu_py]: adjusted gridVoxelNumY to %d",
            grid_voxel_num_y,
        )

    logger.info(
        "LocalPlanner [cmu_py]: loaded %d paths, %d groups, %d non-empty correspondences",
        PATH_NUM,
        GROUP_NUM,
        len(correspondences),
    )
    return {
        "paths": paths,
        "group_of_path": group_of_path,
        "start_paths": start_paths,
        "correspondences": correspondences,
        "grid_voxel_num_x": grid_voxel_num_x,
        "grid_voxel_num_y": grid_voxel_num_y,
    }
