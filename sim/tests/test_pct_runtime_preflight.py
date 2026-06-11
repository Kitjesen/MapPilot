from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts import pct_runtime_preflight

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_pct_runtime_preflight_reports_host_and_abi_blockers(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setattr(
        pct_runtime_preflight,
        "inspect_pct_runtime",
        lambda repo_root, machine=None: {
            "ok": False,
            "machine": "amd64",
            "canonical_arch": "x86_64",
            "python_tag": "py313",
            "known_good_python_tag": "py310",
            "python_abi_matches_known_good": False,
            "platform_system": "windows",
            "native_binary_format": "linux_elf",
            "host_platform_supported": False,
            "host_platform_blocker": "PCT native artifacts are Linux ELF extension modules",
            "lib_dir": "pct/lib/x86_64",
            "searched": ["pct/lib/x86_64"],
            "required": ["a_star.cpython-313-x86_64-linux-gnu.so"],
            "missing": ["a_star.cpython-313-x86_64-linux-gnu.so"],
            "shared_missing": ["libmetis-gtsam.so"],
            "recommended_build_command": (
                "bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh"
            ),
            "error": "No runnable PCT native modules",
        },
    )

    report = pct_runtime_preflight.build_report(machine="AMD64")

    assert report["schema_version"] == "lingtu.pct_runtime_preflight.v1"
    assert report["ok"] is False
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["current_host"]["python_tag"] == "py313"
    assert report["checks"]["host_platform"]["ok"] is False
    assert report["checks"]["python_abi"]["ok"] is False
    assert report["checks"]["extension_modules"]["ok"] is False
    assert report["checks"]["shared_libraries"]["ok"] is False
    assert report["runtime_fingerprint"]["reason_code"] == "unsupported_host_platform"
    assert report["runtime_fingerprint"]["python"]["executable"] == sys.executable
    assert report["runtime_fingerprint"]["python"]["abi_matches_known_good"] is False
    assert report["runtime_fingerprint"]["lib_discovery"]["selected_lib_dir"] == "pct/lib/x86_64"
    assert report["runtime_fingerprint"]["lib_discovery"]["selected_candidate_index"] == 0
    assert report["runtime_fingerprint"]["ros2_environment"] == {
        "ROS_DISTRO": "humble",
        "ROS_DOMAIN_ID": "44",
    }
    assert report["claim_boundary"] == "environment_blocked_no_algorithm_claim"
    assert any("build_host_x86_64.sh" in item for item in report["recommended_setup_commands"])


def test_pct_runtime_preflight_cli_writes_json_without_importing_numpy(tmp_path: Path):
    report_path = tmp_path / "pct_runtime_preflight.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/pct_runtime_preflight.py"),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema_version"] == "lingtu.pct_runtime_preflight.v1"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert "native_runtime" in report
    assert report["execution_mode"] == "pct_runtime_preflight"
    assert probe.returncode == (0 if report["ok"] else 1)
