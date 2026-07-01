from __future__ import annotations

import typing

import numpy

from . import a_star as _a_star_mod
from . import traj_opt as _traj_opt_mod

_Shape = tuple[int, ...]

m = typing.TypeVar("m")
n = typing.TypeVar("n")

# Re-export referenced types for type annotations
Astar = _a_star_mod.Astar
GPMPOptimizer = _traj_opt_mod.GPMPOptimizer
GPMPOptimizerWnoa = _traj_opt_mod.GPMPOptimizerWnoa

__all__ = [
    "OfflineElePlanner"
]


class DenseElevationMap:
    """Opaque C++ type returned by OfflineElePlanner.get_map()."""
    ...

class OfflineElePlanner:
    def __init__(self, max_heading_rate: float, use_quintic: bool = False) -> None: ...
    def debug(self) -> None: ...
    def get_debug_path(self) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def get_map(self) -> DenseElevationMap: ...
    def get_path_finder(self) -> Astar: ...
    def get_trajectory_optimizer(self) -> GPMPOptimizerWnoa: ...
    def get_trajectory_optimizer_wnoj(self) -> GPMPOptimizer: ...
    def init_map(self, arg0: float, arg1: float, arg2: float, arg3: int, arg4: numpy.ndarray[numpy.float64, _Shape[m, n]], arg5: numpy.ndarray[numpy.float64, _Shape[m, n]], arg6: numpy.ndarray[numpy.float64, _Shape[m, n]], arg7: numpy.ndarray[numpy.float64, _Shape[m, n]], arg8: numpy.ndarray[numpy.float64, _Shape[m, n]], arg9: numpy.ndarray[numpy.float64, _Shape[m, n]]) -> None: ...
    def plan(self, arg0: numpy.ndarray[numpy.int32, _Shape[3, 1]], arg1: numpy.ndarray[numpy.int32, _Shape[3, 1]], arg2: bool) -> bool: ...
    def set_max_iterations(self, arg0: int) -> None: ...
    def set_reference_height(self, arg0: float) -> None: ...
    pass
