"""Check TaskSpeedLimit against navd's resolved follower speed budget.

The endpoint runs in an isolated DDS domain with motion publication disabled.
TaskSpeedLimit is a LingTu DDS extension to the pinned SCAN integration.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

from nav.adapters.native.abi import NativeCommandSession


def check_speed_budget(endpoint, client_path, nav_cap, follower_cap, requests):
    domain_id = 170 + os.getpid() % 40
    with tempfile.TemporaryDirectory(prefix="lingtu-task-speed-") as temporary, ExitStack() as resources:
        tmp_path = Path(temporary)
        status_file = tmp_path / "status.json"
        if os.name == "nt":
            resources.enter_context(os.add_dll_directory(str(client_path.parent)))
        log = resources.enter_context((tmp_path / "navd.log").open("wb"))
        process = subprocess.Popen(
            [
                str(endpoint), "--control-mode", "teleop", "--domain-id", str(domain_id),
                "--publish-cmd-vel", "false", "--check-obstacle", "false",
                "--max-speed-mps", str(nav_cap),
                "--path-follower-max-speed-mps", str(follower_cap),
                "--status-file", str(status_file),
                "--estop-latch-file", str(tmp_path / "estop.json"),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
        client = None
        try:
            deadline = time.monotonic() + 10.0
            while not status_file.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            assert status_file.exists(), (tmp_path / "navd.log").read_text(errors="replace")
            client = NativeCommandSession(client_path, domain_id=domain_id, timeout_ms=3000)
            for index, (speed, expected) in enumerate(requests):
                receipt = client.set_navigation_task_speed_limit(
                    "absent-task", f"speed-{index}", speed,
                )
                assert receipt["accepted"] is False
                assert receipt["reason"] == expected, (speed, expected, receipt)
        finally:
            try:
                if client is not None:
                    client.close()
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", type=Path, required=True)
    parser.add_argument("--client", type=Path, required=True)
    args = parser.parse_args()
    endpoint = args.endpoint.resolve()
    client_path = args.client.resolve()
    assert endpoint.is_file(), endpoint
    assert client_path.is_file(), client_path
    os.environ["LINGTU_HOST_BOOT_ID"] = f"speed-test-{os.getpid()}"
    for nav_cap, follower_cap, requests in (
        (1.4, 0.4, [(0.6, "task_speed_limit_invalid"), (0.3, "task_not_active")]),
        (0.4, 0.8, [(0.6, "task_not_active")]),
    ):
        check_speed_budget(endpoint, client_path, nav_cap, follower_cap, requests)
        print(f"PASS nav={nav_cap} follower={follower_cap}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
