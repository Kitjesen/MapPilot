from __future__ import annotations

import subprocess
import sys
import os
from pathlib import Path

import pytest


def _module_import_is_safe(module_name: str) -> bool:
    try:
        probe = subprocess.run(
            [sys.executable, "-c", f"import {module_name}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except Exception:
        return False
    return probe.returncode == 0


_NUMPY_IMPORT_SAFE = _module_import_is_safe("numpy")

from drivers.sim.pointcloud import SimPointCloudProvider


def test_pointcloud_provider_import_does_not_load_numpy():
    repo_root = Path(__file__).resolve().parents[2]
    src_root = repo_root / "src"
    env = dict(os.environ)
    env["PYTHONPATH"] = (
        str(src_root)
        if not env.get("PYTHONPATH")
        else f"{src_root}{os.pathsep}{env['PYTHONPATH']}"
    )
    code = (
        "import sys; "
        "import drivers.sim.pointcloud; "
        "raise SystemExit(1 if 'numpy' in sys.modules else 0)"
    )
    probe = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(repo_root),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=10,
    )

    assert probe.returncode == 0


@pytest.mark.skipif(
    not _NUMPY_IMPORT_SAFE,
    reason="NumPy import is unsafe in this host Python",
)
def test_pointcloud_provider_applies_nested_body_transform_and_excludes_robot(
    tmp_path,
):
    import numpy as np

    scene = tmp_path / "nested_scene.xml"
    scene.write_text(
        """
<mujoco>
  <worldbody>
    <body name="robot_placeholder" pos="8 0 0.5">
      <geom name="chassis" type="box" size="1 1 0.4"/>
    </body>
    <body name="obstacle_body" pos="1 2 0.5" euler="0 0 1.57079632679">
      <geom name="rotated_crate" type="box" pos="1 0 0" size="1 0.25 0.4"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    provider = SimPointCloudProvider(sample_spacing=0.25)

    points = provider._parse_scene(scene)

    assert provider._geom_count == 1
    assert points.shape[1] == 3
    assert len(points) > 0
    center = points.mean(axis=0)
    assert center[:2] == pytest.approx(np.array([1.0, 3.0]), abs=0.15)
    assert points[:, 0].min() == pytest.approx(0.75, abs=0.05)
    assert points[:, 0].max() == pytest.approx(1.25, abs=0.05)
    assert points[:, 1].min() == pytest.approx(2.0, abs=0.05)
    assert points[:, 1].max() == pytest.approx(4.0, abs=0.05)
    assert not np.any(points[:, 0] > 6.0)
