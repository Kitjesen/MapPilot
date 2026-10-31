from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "tum_rgbd_probe.py"
SPEC = importlib.util.spec_from_file_location("tum_rgbd_probe", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_rgb_depth_association_does_not_reuse_a_depth_frame() -> None:
    rgb = [(0.000, Path("rgb-0")), (0.010, Path("rgb-1")), (0.020, Path("rgb-2"))]
    depth = [(0.004, Path("depth-0")), (0.024, Path("depth-1"))]

    pairs = MODULE._associate(rgb, depth, max_delta_s=0.015)

    assert len(pairs) == 2
    assert len({pair["depth"] for pair in pairs}) == 2
