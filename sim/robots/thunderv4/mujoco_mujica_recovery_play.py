"""ThunderV4 MUJICA recovery playback entrypoint for MuJoCo.

This file is intentionally separate from ``mujoco_him_keyboard.py`` because
HIM flat locomotion and MUJICA recovery use different policies, observations,
and action semantics.
"""

from __future__ import annotations

import runpy
from pathlib import Path

INOVXIO_ROOT = Path(__file__).resolve().parents[5]
IMPL = INOVXIO_ROOT / "tools" / "rl_eval" / "mujoco_v86_recovery_play_aligned.py"


def main() -> None:
    if not IMPL.exists():
        raise FileNotFoundError(f"Missing MUJICA recovery playback implementation: {IMPL}")
    runpy.run_path(str(IMPL), run_name="__main__")


if __name__ == "__main__":
    main()
