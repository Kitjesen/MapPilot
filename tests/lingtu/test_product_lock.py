# ruff: noqa: S101, S603

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from lingtu.product_lock import ProductControlLock

ROOT = Path(__file__).resolve().parents[2]


def _probe_lock(state_dir: Path) -> subprocess.CompletedProcess[str]:
    code = """
from pathlib import Path
import sys

from lingtu.product_lock import ProductControlBusy, ProductControlLock

try:
    with ProductControlLock(Path(sys.argv[1]), timeout_s=0.15, poll_interval_s=0.01):
        print("acquired")
except ProductControlBusy:
    print("busy")
"""
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (str(ROOT / "src"), environment.get("PYTHONPATH", ""))
        if value
    )
    return subprocess.run(
        [sys.executable, "-B", "-c", code, str(state_dir)],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )


def test_product_control_lock_blocks_another_process(tmp_path: Path) -> None:
    with ProductControlLock(tmp_path, timeout_s=1.0):
        blocked = _probe_lock(tmp_path)

    acquired = _probe_lock(tmp_path)

    assert blocked.returncode == 0, blocked.stderr
    assert blocked.stdout.strip() == "busy"
    assert acquired.returncode == 0, acquired.stderr
    assert acquired.stdout.strip() == "acquired"


def test_product_control_lock_is_reentrant_in_one_thread(tmp_path: Path) -> None:
    with ProductControlLock(tmp_path, timeout_s=1.0):
        with ProductControlLock(tmp_path, timeout_s=1.0):
            pass
