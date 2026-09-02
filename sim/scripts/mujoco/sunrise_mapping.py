#!/usr/bin/env python3
"""Run the MuJoCo continuous mapping quality gate on sunrise via SSH."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import paramiko
except ImportError:
    paramiko = None  # type: ignore[assignment]

ROOT = Path(__file__).resolve().parents[3]
REMOTE_ROOT = "/home/sunrise/data/inovxio/lingtu"
MAX_CYCLONEDDS_DOMAIN_ID = 232
SYNC_PATHS = [
    "sim/scripts/mujoco/continuous_mapping_quality_gate.py",
    "sim/scripts/mujoco/native_dds_sensors.py",

    "src/localization/fastlio2/config/sim_mid360_slam.yaml",
    "src/localization/fastlio2/config/sim_mid360_slam_tight.yaml",
]
DEFAULT_SLAM_CONFIG = ROOT / "src" / "localization" / "fastlio2" / "config" / "sim_mid360_slam.yaml"
VEL_TIGHT_SLAM_CONFIG = ROOT / "src" / "localization" / "fastlio2" / "config" / "sim_mid360_slam_tight.yaml"


@dataclass(frozen=True)
class SweepCase:
    """One named continuous-mapping sweep case."""

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


def _validate_domain_id(domain_id: int) -> int:
    value = int(domain_id)
    if value < 0 or value > MAX_CYCLONEDDS_DOMAIN_ID:
        raise SystemExit(
            f"--domain-id must be in [0, {MAX_CYCLONEDDS_DOMAIN_ID}] for CycloneDDS; got {value}"
        )
    return value


def _connect(host: str, user: str, password: str) -> paramiko.SSHClient:
    if paramiko is None:
        raise SystemExit("paramiko required: pip install paramiko")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password, timeout=20)
    return client


def _sync_files(client: paramiko.SSHClient, remote_root: str) -> None:
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


def _mkdir_p(sftp: paramiko.SFTPClient, remote_dir: str) -> None:
    parts = remote_dir.strip("/").split("/")
    path = ""
    for part in parts:
        path += f"/{part}"
        try:
            sftp.stat(path)
        except OSError:
            sftp.mkdir(path)


def _run(
    client: paramiko.SSHClient,
    remote_root: str,
    duration: float,
    domain_id: int,
    drive_profile: str,
    slam_config: Path,
    *,
    kill_stale: bool,
) -> dict:
    if kill_stale:
        command = (
            f"pkill -f 'slamd.*--domain-id {domain_id}' >/dev/null 2>&1 || true; "
            f"sleep 1"
        )
        client.exec_command(command, timeout=15)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = f"{remote_root}/artifacts/sunrise_mujoco_continuous_mapping_gate_{stamp}"
    remote_config = f"{remote_root}/{slam_config.relative_to(ROOT).as_posix()}"
    command = (
        f"set -euo pipefail; "
        f"cd {remote_root}; "
        f"export PYTHONPATH=\"$PWD/src:$PWD\"; "
        f"mkdir -p {run_dir}; "
        f"python3 sim/scripts/mujoco/continuous_mapping_quality_gate.py "
        f"--duration {duration} "
        f"--domain-id {domain_id} "
        f"--drive-profile {drive_profile} "
        f"--slam-config {remote_config} "
        f"--run-dir {run_dir} "
        f"--json-out {run_dir}/summary.json "
        f"2>&1 | tee {run_dir}/gate.log; "
        f"echo RUN_DIR={run_dir}"
    )
    print(f"remote command: {command[:200]}...")
    _, stdout, stderr = client.exec_command(command, timeout=int(duration + 600))
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    exit_code = stdout.channel.recv_exit_status()
    print(out)
    if err.strip():
        print(err, file=sys.stderr)
    summary_path = f"{run_dir}/summary.json"
    _, cat_out, _ = client.exec_command(f"cat {summary_path}", timeout=30)
    summary_text = cat_out.read().decode("utf-8", errors="replace")
    summary = json.loads(summary_text) if summary_text.strip() else {"ok": False, "error": "summary_missing"}
    summary["remote_exit_code"] = exit_code
    summary["remote_run_dir"] = run_dir
    local_artifact = ROOT / "artifacts" / Path(run_dir).name
    local_artifact.mkdir(parents=True, exist_ok=True)
    (local_artifact / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (local_artifact / "gate.log").write_text(out, encoding="utf-8")
    sftp = client.open_sftp()
    try:
        for name in ("saved_map_quality.json", "bridge_report.json", "scale_convergence.png", "trajectory_overlay.png"):
            remote = f"{run_dir}/{name}"
            local = local_artifact / name
            try:
                sftp.get(remote, str(local))
            except OSError:
                pass
    finally:
        sftp.close()
    return summary


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default="192.168.66.13")
    parser.add_argument("--user", default="sunrise")
    parser.add_argument("--password", default=os.environ.get("S100P_PASSWORD"))


def _run_command(args: argparse.Namespace) -> int:
    if not args.password:
        raise SystemExit("--password or S100P_PASSWORD is required")
    domain_id = _validate_domain_id(int(args.domain_id))
    slam_config = Path(args.slam_config)
    if not slam_config.is_file():
        raise SystemExit(f"--slam-config missing: {slam_config}")

    client = _connect(args.host, args.user, args.password)
    try:
        if not args.no_sync:
            _sync_files(client, REMOTE_ROOT)
        summary = _run(
            client,
            REMOTE_ROOT,
            float(args.duration),
            domain_id,
            str(args.drive_profile),
            slam_config,
            kill_stale=not args.no_kill_stale,
        )
    finally:
        client.close()

    print(json.dumps(summary, indent=2, ensure_ascii=True))
    return 0 if summary.get("ok") else 1


def _remote_exec(client: paramiko.SSHClient, command: str, timeout_s: float) -> tuple[int, str, str]:
    _, stdout, stderr = client.exec_command(command, timeout=int(timeout_s))
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return stdout.channel.recv_exit_status(), out, err


def _run_case(
    client: paramiko.SSHClient,
    remote_root: str,
    case: SweepCase,
    domain_id: int,
    *,
    kill_stale: bool,
) -> dict:
    if kill_stale:
        _remote_exec(
            client,
            f"pkill -f 'slamd.*--domain-id {domain_id}' >/dev/null 2>&1 || true; sleep 1",
            timeout_s=15.0,
        )

    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = f"{remote_root}/artifacts/sunrise_mujoco_continuous_sweep_{case.name}_{stamp}"
    remote_config = f"{remote_root}/{case.slam_config.relative_to(ROOT).as_posix()}"
    command = (
        f"set -euo pipefail; cd {remote_root}; export PYTHONPATH=\"$PWD/src:$PWD\"; "
        f"mkdir -p {run_dir}; python3 sim/scripts/mujoco/continuous_mapping_quality_gate.py "
        f"--duration {case.duration_s} --domain-id {domain_id} "
        f"--drive-profile {case.drive_profile} --slam-config {remote_config} "
        f"--run-dir {run_dir} --json-out {run_dir}/summary.json "
        f"2>&1 | tee {run_dir}/gate.log; echo RUN_DIR={run_dir}"
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
    summary.update(
        sweep_case=case.name,
        remote_exit_code=exit_code,
        remote_run_dir=run_dir,
        slam_config=str(case.slam_config),
    )
    local_artifact = ROOT / "artifacts" / Path(run_dir).name
    local_artifact.mkdir(parents=True, exist_ok=True)
    (local_artifact / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (local_artifact / "gate.log").write_text(out, encoding="utf-8")
    sftp = client.open_sftp()
    try:
        for name in ("saved_map_quality.json", "bridge_report.json", "scale_convergence.png", "trajectory_overlay.png"):
            try:
                sftp.get(f"{run_dir}/{name}", str(local_artifact / name))
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


def _sweep_command(args: argparse.Namespace) -> int:
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
        raise SystemExit(f"need {len(cases)} domains starting at {start_domain}; max is {MAX_CYCLONEDDS_DOMAIN_ID}")

    client = _connect(args.host, args.user, args.password)
    rows: list[dict] = []
    try:
        if not args.no_sync:
            _sync_files(client, REMOTE_ROOT)
        for index, case in enumerate(cases):
            rows.append(
                _compact_row(
                    _run_case(
                        client,
                        REMOTE_ROOT,
                        case,
                        start_domain + index,
                        kill_stale=not args.no_kill_stale,
                    )
                )
            )
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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run_parser = commands.add_parser("run", help="Run one continuous mapping gate.")
    _add_connection_args(run_parser)
    run_parser.add_argument("--duration", type=float, default=180.0)
    run_parser.add_argument(
        "--domain-id",
        type=int,
        default=231,
        help=f"Isolated CycloneDDS domain in [0, {MAX_CYCLONEDDS_DOMAIN_ID}]. Avoid production domain 0.",
    )
    run_parser.add_argument("--drive-profile", choices=["arc", "box_explore", "box_explore_gentle"], default="box_explore")
    run_parser.add_argument(
        "--slam-config",
        default=str(DEFAULT_SLAM_CONFIG),
        help="Fast-LIO yaml on the dev machine; synced to sunrise before run.",
    )
    run_parser.add_argument("--no-sync", action="store_true")
    run_parser.add_argument("--no-kill-stale", action="store_true", help="Do not pkill prior gate SLAM on this domain.")

    sweep_parser = commands.add_parser("sweep", help="Run the standard tuning matrix.")
    _add_connection_args(sweep_parser)
    sweep_parser.add_argument("--start-domain-id", type=int, default=226)
    sweep_parser.add_argument(
        "--cases",
        default="all",
        help="Comma-separated case names or 'all'. Choices: " + ",".join(case.name for case in DEFAULT_CASES),
    )
    sweep_parser.add_argument("--no-sync", action="store_true")
    sweep_parser.add_argument("--no-kill-stale", action="store_true")
    sweep_parser.add_argument("--json-out", default="", help="Write sweep matrix summary JSON locally.")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run one gate or the standard sweep matrix."""

    args = _build_parser().parse_args(argv)
    if args.command == "run":
        return _run_command(args)
    return _sweep_command(args)


if __name__ == "__main__":
    raise SystemExit(main())
