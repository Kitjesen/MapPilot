# ruff: noqa: S101
"""Process identity behavior used by simulation child ownership."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from lingtu.sim.identity import ProcessIdentity


@pytest.mark.skipif(
    os.name != "posix" or not Path("/proc").is_dir(),
    reason="Linux procfs behavior",
)
def test_zombie_process_is_not_live() -> None:
    child = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(0.05)"],
    )
    identity = ProcessIdentity.current(child.pid)
    stat_path = Path("/proc") / str(child.pid) / "stat"
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            raw = stat_path.read_text(encoding="ascii")
            state = raw[raw.rfind(")") + 2 :].split()[0]
            if state == "Z":
                break
            time.sleep(0.01)
        else:
            pytest.fail("child did not become a zombie")

        assert identity.matches() is False
    finally:
        child.wait(timeout=5)
