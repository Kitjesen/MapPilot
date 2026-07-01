from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PointFieldSpec:
    """ROS-free point-field descriptor for legacy tomogram utilities."""

    name: str
    offset: int
    datatype: int
    count: int = 1


FLOAT32 = 7


POINT_FIELDS_XYZI = [
    PointFieldSpec(name='x', offset=0, datatype=FLOAT32, count=1),
    PointFieldSpec(name='y', offset=4, datatype=FLOAT32, count=1),
    PointFieldSpec(name='z', offset=8, datatype=FLOAT32, count=1),
    PointFieldSpec(name='intensity', offset=12, datatype=FLOAT32, count=1)
]


def GRID_POINTS_XYZI(resolution, dim_x, dim_y):
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    point_proto[:, :2] *= resolution
    point_proto[:, 3] = 1.0

    return index_proto, point_proto
