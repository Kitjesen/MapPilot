#!/usr/bin/env python3
"""Run a matrix of MuJoCo continuous mapping gates on sunrise for tuning."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

try:
    import paramiko
except ImportError:
    print("paramiko required: pip install paramiko", file=sys.stderr)
    raise SystemExit(2)

from run_sunrise_continuous_mapping_gate import (  # noqa: E402
    MAX_CYCLONEDDS_DOMAIN_ID,
    REMOTE_ROOT,
    ROOT,
    _connect,
    _mkdir_p,
    _validate_domain_id,
)

DEFAULT_SLAM_CONFIG = (
    ROOT / "src" / "localization" / "fastlio2" / "config" / "sim_mid360_slam.yaml"
)
VEL_TIGHT_SLAM_CONFIG = (
    ROOT
    / "src"
    / "localization"
    / "fastlio2"
    / "config"
    / "sim_mid360_slam_tight.yaml"
)

SYNC_PATHS = [
    "sim/scripts/mujoco/continuous_mapping_quality_gate.py",
    "sim/scripts/mujoco/native_dds_sensors.py",

    "sim/scripts/run_sunrise_continuous_mapping_gate.py",
    "src/localization/fastlio2/config/sim_mid360_slam.yaml",
    "src/localization/fastlio2/config/sim_mid360_slam_tight.yaml",
]


@dataclass(frozen=True)
class SweepCase:
    name: str
    drive_profile: str
    duration_s: float
    slam_config: Path


DEFAULT_CASES: tuple[SweepCase, ...] = (
    SweepCase("box180_baseline", "box_explore", 180.0, DEFAULT_SLAM_CONFIG),
    SweepCase("arc180", "arc", 180.0, DEFAULT_SLAM_CONFIG),
    SweepCase("box120", "box_explore", 120.0, DEFAULT_SLAM_CONFIG),
    SweepCase("box180_vel_tight", "box_explore", 180.0, VEL_TIGHT_SLAM_CONFIG),
    SweepCase("box180_gentle", "box_explore_gentle", 180.0, DEFAULT_SLAM_CONFIG),
)


def _sync_sweep_files(client: paramiko.SSHClient, remote_root: str) -> None:
    sftp = client.open_sftp()
    try:
        for rel in SYNC_PATHS:
            local = ROOT / rel
            remote = f"{remote_root}/{rel.replace(chr(92), '/')}"
            remote_dir = str(Path(remote).parent).replace("\\", "/")
            try:
                sftp.stat(remote_dir)
            except OSError:
                _mkdir_p(sftp, remote_dir)
            sftp.put(str(local), remote)
            print(f"synced {rel}")
    finally:
        sftp.close()


def _remote_exec(client: paramiko.SSHClient, command: str, timeout_s: float) -> tuple[int, str, str]:
    _, stdout, stderr = client.exec_command(command, timeout=int(timeout_s))
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return stdout.channel.recv_exit_status(), out, err


def _kill_stale_slam(client: paramiko.SSHClient, domain_id: int) -> None:
    command = (
        f"pkill -f 'slamd.*--domain-id {domain_id}' >/dev/null 2>&1 || true; "
        f"sleep 1"
    )
    _remote_exec(client, command, timeout_s=15.0)


def _run_case(
    client: paramiko.SSHClient,
    remote_root: str,
    case: SweepCase,
    domain_id: int,
    *,
    kill_stale: bool,
) -> dict:
    if kill_stale:
        _kill_stale_slam(client, domain_id)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = f"{remote_root}/artifacts/sunrise_mujoco_continuous_sweep_{case.name}_{stamp}"
    remote_config = f"{remote_root}/{case.slam_config.relative_to(ROOT).as_posix()}"
    command = (
        f"set -euo pipefail; "
        f"cd {remote_root}; "
        f"export PYTHONPATH=\"$PWD/src:$PWD\"; "
        f"mkdir -p {run_dir}; "
        f"python3 sim/scripts/mujoco/continuous_mapping_quality_gate.py "
        f"--duration {case.duration_s} "
        f"--domain-id {domain_id} "
        f"--drive-profile {case.drive_profile} "
        f"--slam-config {remote_config} "
        f"--run-dir {run_dir} "
        f"--json-out {run_dir}/summary.json "
        f"2>&1 | tee {run_dir}/gate.log; "
        f"echo RUN_DIR={run_dir}"
    )
    print(f"\n=== {case.name} domain={domain_id} profile={case.drive_profile} duration={case.duration_s}s ===")
    print(f"slam_config={case.slam_config.name}")
    exit_code, out, err = _remote_exec(client, command, timeout_s=case.duration_s + 600.0)
    print(out)
    if err.strip():
        print(err, file=sys.stderr)

    summary_path = f"{run_dir}/summary.json"
    _, cat_out, _ = client.exec_command(f"cat {summary_path}", timeout=30)
    summary_text = cat_out.read().decode("utf-8", errors="replace")
    summary = json.loads(summary_text) if summary_text.strip() else {"ok": False, "error": "summary_missing"}
    summary["sweep_case"] = case.name
    summary["remote_exit_code"] = exit_code
    summary["remote_run_dir"] = run_dir
    summary["slam_config"] = str(case.slam_config)

    local_artifact = ROOT / "artifacts" / Path(run_dir).name
    local_artifact.mkdir(parents=True, exist_ok=True)
    (local_artifact / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (local_artifact / "gate.log").write_text(out, encoding="utf-8")
    sftp = client.open_sftp()
    try:
        for name in (
            "saved_map_quality.json",
            "bridge_report.json",
            "scale_convergence.png",
            "trajectory_overlay.png",
        ):
            remote = f"{run_dir}/{name}"
            local = local_artifact / name
            try:
                sftp.get(remote, str(local))
            except OSError:
                pass
    finally:
        sftp.close()
    return summary


def _compact_row(summary: dict) -> dict:
    continuity = summary.get("continuity") or {}
    convergence = summary.get("convergence") or {}
    map_quality = summary.get("map_quality") or {}
    rates = continuity.get("rates") or {}
    return {
        "case": summary.get("sweep_case"),
        "ok": summary.get("ok"),
        "drive_profile": summary.get("drive_profile"),
        "duration_s": summary.get("duration_s"),
        "continuity_ok": not bool(continuity.get("remaining_gaps")),
        "lidar_hz": (rates.get("lidar_input_hz") or {}).get("effective"),
        "imu_hz": (rates.get("imu_input_hz") or {}).get("effective"),
        "cumulative_ratio": convergence.get("cumulative_path_ratio"),
        "max_window_ratio": convergence.get("max_window_ratio"),
        "ate_rmse_m": (convergence.get("ate") or {}).get("rmse_m"),
        "map_quality_ok": map_quality.get("ok"),
        "near_ratio": map_quality.get("near_ratio"),
        "gaps": summary.get("remaining_gaps") or [],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.66.13")
    parser.add_argument("--user", default="sunrise")
    parser.add_argument("--password", default=os.environ.get("S100P_PASSWORD"))
    parser.add_argument("--start-domain-id", type=int, default=226)
    parser.add_argument(
        "--cases",
        default="all",
        help="Comma-separated case names or 'all'. Choices: "
        + ",".join(case.name for case in DEFAULT_CASES),
    )
    parser.add_argument("--no-sync", action="store_true")
    parser.add_argument("--no-kill-stale", action="store_true")
    parser.add_argument("--json-out", default="", help="Write sweep matrix summary JSON locally.")
    args = parser.parse_args()
    if not args.password:
        raise SystemExit("--password or S100P_PASSWORD is required")

    if str(args.cases).strip().lower() == "all":
        cases = list(DEFAULT_CASES)
    else:
        wanted = {name.strip() for name in str(args.cases).split(",") if name.strip()}
        cases = [case for case in DEFAULT_CASES if case.name in wanted]
        missing = wanted - {case.name for case in cases}
        if missing:
            raise SystemExit(f"unknown case names: {sorted(missing)}")

    start_domain = _validate_domain_id(int(args.start_domain_id))
    if start_domain + len(cases) - 1 > MAX_CYCLONEDDS_DOMAIN_ID:
        raise SystemExit(
            f"need {len(cases)} domains starting at {start_domain}; max is {MAX_CYCLONEDDS_DOMAIN_ID}"
        )

    client = _connect(args.host, args.user, args.password)
    rows: list[dict] = []
    try:
        if not args.no_sync:
            _sync_sweep_files(client, REMOTE_ROOT)
        for index, case in enumerate(cases):
            domain_id = start_domain + index
            summary = _run_case(
                client,
                REMOTE_ROOT,
                case,
                domain_id,
                kill_stale=not args.no_kill_stale,
            )
            rows.append(_compact_row(summary))
    finally:
        client.close()

    matrix = {
        "schema_version": "lingtu.mujoco_continuous_mapping_sweep.v1",
        "host": args.host,
        "started_domain_id": start_domain,
        "rows": rows,
    }
    text = json.dumps(matrix, indent=2, ensure_ascii=True)
    print("\n=== sweep matrix ===")
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if all(row.get("ok") for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
