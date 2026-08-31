from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_root = root / ".manual_tmp_test" / f"motion-recording-run-{stamp}"
    runner_log = root / ".manual_tmp_test" / f"motion-recording-runner-{stamp}"
    stdout = runner_log / "stdout.json"
    stderr = runner_log / "stderr.log"
    run_root.mkdir(parents=True, exist_ok=True)
    runner_log.mkdir(parents=True, exist_ok=True)
    argv = [
        sys.executable,
        "-m",
        "sim.runtime.coordinator.motion_recording",
        ".manual_tmp_test/motion-recording-bundle-20260809-195131",
        "--repo-root",
        ".",
        "--run-root",
        str(run_root),
        "--run-id",
        "codex-thunderv4-motion-sixway",
        "--mujoco-host",
        "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
        "--unreal-editor",
        r"D:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe",
        "--truth-odom-publisher",
        "build/mujoco_native_dds/Release/lingtu_truth_odom_publisher.exe",
        "--snapshot-port",
        "25143",
        "--ready-timeout-s",
        "240",
        "--warmup-steps",
        "8",
        "--steps-per-frame",
        "8",
        "--refresh-steps",
        "10",
        "--transition-frames",
        "20",
        "--minimum-displacement-m",
        "0.05",
        "--minimum-rotation-rad",
        "0.15",
        "--maximum-turn-drift-m",
        "0.35",
        "--frame-capture-every",
        "8",
        "--frame-capture-max",
        "160",
        "--minimum-frames",
        "30",
        "--frame-flush-timeout-s",
        "30",
        "--motion-camera-stable-id",
        "thunder_01/base_link",
        "--maneuver",
        "forward:100:0.30:0.00:0.00",
        "--maneuver",
        "backward:100:-0.30:0.00:0.00",
        "--maneuver",
        "left:100:0.00:0.30:0.00",
        "--maneuver",
        "right:100:0.00:-0.30:0.00",
        "--maneuver",
        "turn_left:100:0.00:0.00:0.80",
        "--maneuver",
        "turn_right:100:0.00:0.00:-0.80",
    ]
    (runner_log / "command.json").write_text(
        json.dumps(
            {
                "argv": argv,
                "run_root": str(run_root),
                "stdout": str(stdout),
                "stderr": str(stderr),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    with stdout.open("w", encoding="utf-8") as out, stderr.open("w", encoding="utf-8") as err:
        return subprocess.run(argv, cwd=root, stdout=out, stderr=err).returncode


if __name__ == "__main__":
    raise SystemExit(main())
