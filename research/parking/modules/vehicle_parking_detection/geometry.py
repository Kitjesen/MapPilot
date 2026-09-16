from __future__ import annotations

from math import hypot


def box_center_bottom(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, _y1, x2, y2 = box
    return ((x1 + x2) / 2.0, y2)


def box_center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[int, int]]) -> bool:
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        intersects = (current_y > y) != (previous_y > y)
        if intersects:
            denominator = previous_y - current_y
            if abs(denominator) < 1e-9:
                previous_x, previous_y = current_x, current_y
                continue
            x_at_y = (previous_x - current_x) * (y - current_y) / denominator + current_x
            if x <= x_at_y:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def box_iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def center_distance_ratio(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> float:
    ax, ay = box_center(a)
    bx, by = box_center(b)
    distance = hypot(ax - bx, ay - by)
    scale = max(1.0, box_diag(a), box_diag(b))
    return distance / scale


def box_diag(box: tuple[float, float, float, float]) -> float:
    return hypot(max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1]))
