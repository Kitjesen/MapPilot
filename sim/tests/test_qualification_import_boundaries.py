from __future__ import annotations

import subprocess
import sys


def test_coordinator_and_recording_packages_import_in_either_order() -> None:
    """Qualification writers must not pull replay visual/coordinator through packages."""

    orders = (
        ("sim.runtime.coordinator", "sim.runtime.recording"),
        ("sim.runtime.recording", "sim.runtime.coordinator"),
    )
    for order in orders:
        code = "\n".join(
            [
                "import importlib",
                *(f"importlib.import_module({name!r})" for name in order),
            ]
        )
        subprocess.run([sys.executable, "-c", code], check=True)  # noqa: S603
