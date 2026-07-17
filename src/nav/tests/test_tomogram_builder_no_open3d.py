from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from unittest.mock import patch


def _load_builder_module(*, block_optional_deps: bool = False):
    script = (
        Path(__file__).resolve().parents[1]
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "vendor"
        / "pct_planner"
        / "tomography"
        / "scripts"
        / "build_tomogram.py"
    )
    spec = importlib.util.spec_from_file_location("lingtu_test_build_tomogram", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    if not block_optional_deps:
        spec.loader.exec_module(module)
        return module

    def guarded_import(name, *args, **kwargs):
        if name == "open3d" or name.startswith("open3d."):
            raise ModuleNotFoundError(name)
        if name == "numba" or name.startswith("numba."):
            raise ModuleNotFoundError(name)
        return real_import(name, *args, **kwargs)

    for loaded in ("kernels", "tomogram", "lingtu_test_build_tomogram"):
        sys.modules.pop(loaded, None)
    real_import = __import__
    with patch("builtins.__import__", side_effect=guarded_import):
        spec.loader.exec_module(module)
    return module


def test_tomogram_builder_reads_ascii_pcd_without_open3d_or_numba(
    tmp_path: Path,
) -> None:
    module = _load_builder_module(block_optional_deps=True)
    pcd = tmp_path / "map.pcd"
    output = tmp_path / "tomogram.pickle"
    pcd.write_text(
        "\n".join(
            [
                "# .PCD v0.7",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 4",
                "HEIGHT 1",
                "POINTS 4",
                "DATA ascii",
                "0 0 0",
                "1 0 0.1",
                "0 1 0.2",
                "1 1 0.3",
            ]
        )
    )

    data = module.build_tomogram_from_pcd(
        str(pcd),
        str(output),
        resolution=0.5,
        slice_dh=0.2,
        ground_h=0.0,
    )

    assert output.is_file()
    assert data["data"].shape[0] == 5
    assert data["resolution"] == 0.5
