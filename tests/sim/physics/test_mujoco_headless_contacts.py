import json
import os
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
HEADLESS_EXE = Path(
    os.environ.get(
        "LINGTU_MUJOCO_HEADLESS_EXE",
        REPO_ROOT
        / "build"
        / "mujoco-motion-contact-win"
        / "Release"
        / "lingtu_mujoco_headless.exe",
    )
)


def test_headless_snapshot_serializes_real_contacts_and_reset() -> None:
    if not HEADLESS_EXE.is_file():
        pytest.skip(f"focused headless executable is unavailable: {HEADLESS_EXE}")

    world = REPO_ROOT / "tests/sim/physics/fixtures/scene_world.xml"
    robot = REPO_ROOT / "tests/sim/physics/fixtures/free_body.xml"
    completed = subprocess.run(
        [
            str(HEADLESS_EXE),
            "--session",
            "contact-test",
            "1",
            str(world),
            "--global-policy",
            "0.002",
            "euler",
            "newton",
            "100",
            "0",
            "0",
            "-9.81",
            "--robot",
            "robot",
            str(robot),
            "test_body",
            "0",
            "0",
            "0",
            "1",
            "0",
            "0",
            "0",
        ],
        input=(
            "start\n"
            "advance-sampled-realtime 500 500\n"
            "snapshot\n"
            "reset\n"
            "stop\n"
        ),
        capture_output=True,
        check=True,
        text=True,
        timeout=20,
    )
    events = [json.loads(line) for line in completed.stdout.splitlines()]
    visual_batch = next(
        event for event in events if event["event"] == "snapshot_batch"
    )
    realtime_snapshot = visual_batch["snapshots"][0]
    assert realtime_snapshot["contact_count"] == len(
        realtime_snapshot["contacts"]
    )
    assert realtime_snapshot["contact_count"] > 0
    snapshots = [event for event in events if event["event"] == "snapshot"]

    resting, reset = snapshots
    assert resting["contact_count"] == len(resting["contacts"])
    assert resting["contact_count"] > 0
    floor_contact = next(
        contact
        for contact in resting["contacts"]
        if contact["geom1_name"].endswith("floor")
        or contact["geom2_name"].endswith("floor")
    )
    assert "robot/test_body" in {
        floor_contact["body1_stable_id"],
        floor_contact["body2_stable_id"],
    }
    assert floor_contact["distance_m"] <= 0.0
    assert floor_contact["normal_force_n"] > 0.0
    assert reset["contact_count"] == 0
    assert reset["contacts"] == []
