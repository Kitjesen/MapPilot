"""Meter-scale terrain shared by the offline mountain preview and its trail."""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise


@dataclass(frozen=True)
class TerrainGrid:
    span_m: float
    heights: tuple[tuple[float, ...], ...]

    def mesh_data(self):
        count = len(self.heights)
        step = self.span_m / (count - 1)
        vertices = [
            (-self.span_m / 2 + x * step, -self.span_m / 2 + y * step, self.heights[y][x])
            for y in range(count)
            for x in range(count)
        ]
        faces = []
        for y in range(count - 1):
            for x in range(count - 1):
                index = y * count + x
                faces.extend(((index, index + 1, index + count + 1), (index, index + count + 1, index + count)))
        return vertices, faces

    def height_at(self, x: float, y: float) -> float:
        count = len(self.heights)
        gx = (x / self.span_m + 0.5) * (count - 1)
        gy = (y / self.span_m + 0.5) * (count - 1)
        if not (0 <= gx <= count - 1 and 0 <= gy <= count - 1):
            raise ValueError("Position is outside the mountain terrain")
        ix, iy = min(int(gx), count - 2), min(int(gy), count - 2)
        u, v = gx - ix, gy - iy
        z00, z10 = self.heights[iy][ix : ix + 2]
        z01, z11 = self.heights[iy + 1][ix : ix + 2]
        if u >= v:
            return z00 + (z10 - z00) * u + (z11 - z10) * v
        return z00 + (z11 - z01) * u + (z01 - z00) * v


def trail_weights(vertices, centerline, *, width_m=2.8, feather_m=0.8):
    """Paint the trail onto the terrain itself, with a meter-scale soft edge."""
    if len(centerline) < 2 or width_m <= 0 or feather_m <= 0:
        raise ValueError("A trail needs two points and positive width/feather")
    weights = []
    for x, y, _ in vertices:
        distance = math.inf
        for (ax, ay), (bx, by) in pairwise(centerline):
            dx, dy = bx - ax, by - ay
            length_squared = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_squared)) if length_squared else 0.0
            distance = min(distance, math.hypot(x - ax - t * dx, y - ay - t * dy))
        edge = max(0.0, min(1.0, (distance - width_m / 2) / feather_m))
        weights.append(1.0 - edge * edge * (3.0 - 2.0 * edge))
    return weights


def mountain_grid(*, samples=129, span_m=84.0):
    if samples < 2 or span_m <= 0:
        raise ValueError("Terrain needs at least two samples and a positive span")
    heights = []
    for row in range(samples):
        y = -span_m / 2 + row * span_m / (samples - 1)
        line = []
        for column in range(samples):
            x = -span_m / 2 + column * span_m / (samples - 1)
            broad = 0.8 * math.sin(x * 0.075 + 0.7) + 0.55 * math.cos(y * 0.09)
            ridge = 1.6 * math.exp(-((x + 8) ** 2) / 280.0) * math.sin(y * 0.11)
            valley = -0.9 * math.exp(-((x - 4) ** 2 + (y + 10) ** 2) / 220.0)
            line.append(broad + ridge + valley)
        heights.append(tuple(line))
    return TerrainGrid(span_m=span_m, heights=tuple(heights))


def background_mesh(surface):
    """Continue the ground into rounded distant landforms, leaving the site open."""
    half = surface.span_m / 2
    inner = [-half + i * surface.span_m / (len(surface.heights) - 1) for i in range(len(surface.heights))]
    outer = [float(value) for value in range(80, 641, 40) if value > half]
    axis = [-value for value in reversed(outer)] + inner + outer
    vertices = []
    for y in axis:
        for x in axis:
            base = surface.height_at(max(-half, min(half, x)), max(-half, min(half, y)))
            distance = math.hypot(max(0, abs(x) - half), max(0, abs(y) - half))
            blend = min(1.0, distance / 80.0)
            blend = blend * blend * (3 - 2 * blend)
            peaks = (
                130 * math.exp(-(((x + 190) / 180) ** 2) - ((y - 300) / 160) ** 2)
                + 185 * math.exp(-(((x - 170) / 170) ** 2) - ((y - 440) / 155) ** 2)
                + 90 * math.exp(-(((x - 400) / 140) ** 2) - ((y - 230) / 180) ** 2)
            )
            detail = 5 * math.sin(x * 0.031 + y * 0.017) * math.sin(y * 0.026)
            vertices.append((x, y, base + blend * (peaks + detail)))
    faces = []
    count = len(axis)
    for row in range(count - 1):
        for column in range(count - 1):
            if abs((axis[column] + axis[column + 1]) / 2) < half and abs((axis[row] + axis[row + 1]) / 2) < half:
                continue
            a = row * count + column
            faces.extend(((a, a + 1, a + count + 1), (a, a + count + 1, a + count)))
    return vertices, faces
