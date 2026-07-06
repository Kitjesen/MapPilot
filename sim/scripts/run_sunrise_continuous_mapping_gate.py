#!/usr/bin/env python3
"""Run the MuJoCo continuous mapping quality gate on sunrise via SSH."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

try:
    import paramiko
except ImportError:
    print("paramiko required: pip install paramiko", file=sys.stderr)
    raise SystemExit(2)

ROOT = Path(__file__).resolve().parents[2]
REMOTE_ROOT = "/home/sunrise/data/inovxio/lingtu"
MAX_CYCLONEDDS_DOMAIN_ID = 232
SYNC_PATHS = [
    "sim/scripts/mujoco/continuous_mapping_quality_gate.py",
    "sim/scripts/mujoco/native_dds_sensors.py",
    "sim/scripts/mujoco_continuous_mapping_quality_gate.py",
    "src/localization/fastlio2/config/mid360_mujoco_native_dds.yaml",
    "src/localization/fastlio2/config/mid360_mujoco_native_dds_vel_tight.yaml",
]
DEFAULT_SLAM_CONFIG = ROOT / "src" / "localization" / "fastlio2" / "config" / "mid360_mujoco_native_dds.yaml"


def _validate_domain_id(domain_id: int) -> int:
    value = int(domain_id)
    if value < 0 or value > MAX_CYCLONEDDS_DOMAIN_ID:
        raise SystemExit(
            f"--domain-id must be in [0, {MAX_CYCLONEDDS_DOMAIN_ID}] for CycloneDDS; got {value}"
        )
    return value


def _connect(host: str, user: str, password: str) -> paramiko.SSHClient:
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
            f"pkill -f 'lingtu_slam_cyclone_runtime.*--domain-id {domain_id}' >/dev/null 2>&1 || true; "
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.66.13")
    parser.add_argument("--user", default="sunrise")
    parser.add_argument("--password", default="sunrise")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument(
        "--domain-id",
        type=int,
        default=231,
        help=f"Isolated CycloneDDS domain in [0, {MAX_CYCLONEDDS_DOMAIN_ID}]. Avoid production domain 0.",
    )
    parser.add_argument("--drive-profile", choices=["arc", "box_explore", "box_explore_gentle"], default="box_explore")
    parser.add_argument(
        "--slam-config",
        default=str(DEFAULT_SLAM_CONFIG),
        help="Fast-LIO yaml on the dev machine; synced to sunrise before run.",
    )
    parser.add_argument("--no-sync", action="store_true")
    parser.add_argument("--no-kill-stale", action="store_true", help="Do not pkill prior gate SLAM on this domain.")
    args = parser.parse_args()
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


if __name__ == "__main__":
    raise SystemExit(main())
