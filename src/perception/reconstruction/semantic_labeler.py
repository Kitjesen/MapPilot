"""
semantic_labeler.py — 为点云注入语义标签

从场景图 JSON 解析物体 {label, position, extent}，
用最近邻匹配为点云每个点分配语义标签。
"""

import json
import threading

import numpy as np

LABEL_BACKGROUND = "background"
LABEL_RADIUS_FACTOR = 1.5   # extent * factor = 最大匹配半径


class SemanticLabeler:
    """解析场景图，按距离为点云分配语义标签。"""

    def __init__(self):
        self._objects: list[dict] = []   # [{label, position, radius}, ...]
        self._lock = threading.Lock()

    def update_from_scene_graph(self, scene_graph_json: str) -> int:
        """解析场景图 JSON，返回解析到的物体数量。"""
        try:
            sg = json.loads(scene_graph_json)
        except (json.JSONDecodeError, TypeError):
            return 0

        objects = sg.get("objects", [])
        parsed = []
        for obj in objects:
            pos = obj.get("position")
            label = obj.get("label", "unknown")
            extent = obj.get("extent", [0.5, 0.5, 0.5])
            if pos is None or len(pos) != 3:
                continue
            # 匹配半径 = 最大 extent 维度 * factor
            radius = max(extent) * LABEL_RADIUS_FACTOR if extent else 0.75
            parsed.append({
                "label": label,
                "position": np.array(pos, dtype=np.float32),
                "radius": float(radius),
            })

        with self._lock:
            self._objects = parsed

        return len(parsed)

    def label_cloud(self, xyzrgb: np.ndarray) -> list[str]:
        """
        为 (N, 6) 点云中每个点分配语义标签。

        Returns:
            labels: list of str，长度 N
        """
        n = len(xyzrgb)
        labels = [LABEL_BACKGROUND] * n
        if n == 0:
            return labels

        with self._lock:
            objects = list(self._objects)

        if not objects:
            return labels

        points = xyzrgb[:, :3]  # Nx3

        # 对每个物体，找 radius 内的点打标签（后处理，更近的物体覆盖）
        obj_positions = np.array([o["position"] for o in objects])  # Mx3
        obj_radii = np.array([o["radius"] for o in objects])        # M

        for i, obj in enumerate(objects):
            diffs = points - obj_positions[i]
            dists = np.linalg.norm(diffs, axis=1)
            mask = dists < obj_radii[i]
            for j in np.where(mask)[0]:
                # 若已有标签，只有距离更近时才覆盖
                if labels[j] == LABEL_BACKGROUND:
                    labels[j] = obj["label"]
                else:
                    # 找当前标签对应物体的距离
                    prev_obj = next(
                        (o for o in objects if o["label"] == labels[j]), None
                    )
                    if prev_obj is not None:
                        prev_dist = np.linalg.norm(points[j] - prev_obj["position"])
                        if dists[j] < prev_dist:
                            labels[j] = obj["label"]

        return labels

    @property
    def object_count(self) -> int:
        with self._lock:
            return len(self._objects)
