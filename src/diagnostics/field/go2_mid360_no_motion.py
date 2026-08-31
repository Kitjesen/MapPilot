"""Run the Go2 MID-360 sensor chain without starting any motion process."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from diagnostics.field.dds_readiness import collect_readiness
from lingtu.assembly.compiler import compile_run_plan
from runtime.config import load_config
from runtime.utils.livox_config import build_mid360_config_dict

SCHEMA_VERSION = "lingtu.go2_mid360.no_motion.v1"
_REQUIRED_SENSOR_PROCESSES = {"lidar", "slam"}
_EXPECTED_MOTION_PROCESSES = {"nav", "driver"}
_MOTION_UNITS = (
    "lt-nav.service",
    "lt-driver.service",
)
_MOTION_PROCESSES = ("navd", "lingtu_driver")
_TOPIC_REQUIREMENTS = {
    "rt/lidar/raw_frame": (5, 5.0),
    "rt/imu/raw": (20, 20.0),
    "rt/slam/odometry": (5, 5.0),
    "rt/slam/registered_cloud": (5, 5.0),
}
_CMD_VEL_TOPIC = "rt/nav/cmd_vel"


class GateFailed(RuntimeError):
    """Raised when the no-motion gate cannot prove its contract."""


def _command(command: list[str], *, timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(  # noqa: S603 - commands are fixed field tools or resolved release binaries.
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _resolved_environment(repo: Path) -> dict[str, str]:
    plan = compile_run_plan("teleop_avoid", "real", robot="unitree/go2")
    roles = {process.name for process in plan.processes}
    environment: dict[str, str] = plan.native_process_environment
    if plan.product != "teleop_avoid" or plan.env != "real":
        raise GateFailed("resolved RunPlan is not real teleop_avoid")
    if not _REQUIRED_SENSOR_PROCESSES <= roles or not _EXPECTED_MOTION_PROCESSES <= roles:
        raise GateFailed("teleop_avoid RunPlan does not contain the expected sensor and motion roles")
    if environment.get("LINGTU_DRIVER_BACKEND") != "go2":
        raise GateFailed("resolved robot backend is not go2")
    if environment.get("LINGTU_LIVOX_NET_IFACE") != environment.get("LINGTU_DRIVER_NETWORK_INTERFACE"):
        raise GateFailed("Go2 DDS and MID-360 must use the same field interface for this profile")

    for key in ("LINGTU_CONFIG_PATH", "LINGTU_SLAM_CONFIG"):
        path = Path(environment[key])
        environment[key] = str(path if path.is_absolute() else repo / path)
    return environment


def _assert_motion_inactive() -> None:
    active_units = [
        unit for unit in _MOTION_UNITS if _command(["systemctl", "is-active", "--quiet", unit]).returncode == 0
    ]
    active_processes = [process for process in _MOTION_PROCESSES if _command(["pgrep", "-x", process]).returncode == 0]
    if active_units or active_processes:
        detail = ", ".join((*active_units, *active_processes))
        raise GateFailed(f"motion runtime is already active: {detail}")


def _assert_network(environment: dict[str, str]) -> dict[str, str]:
    interface = environment["LINGTU_LIVOX_NET_IFACE"]
    host_ip = environment["LINGTU_LIVOX_HOST_IP"]
    lidar_ip = environment["LINGTU_LIVOX_LIDAR_IP"]
    address = _command(["ip", "-o", "-4", "addr", "show", "dev", interface])
    if address.returncode != 0 or f" {host_ip}/" not in f" {address.stdout}":
        raise GateFailed(f"{interface} does not own configured MID-360 host address {host_ip}")
    result = _command(["ping", "-c", "1", "-W", "2", lidar_ip], timeout=4.0)
    if result.returncode != 0:
        raise GateFailed(f"MID-360 is not reachable at {lidar_ip} through the field network")
    return {"interface": interface, "host_ip": host_ip, "lidar_ip": lidar_ip}


def _require_executable(path: Path, label: str) -> Path:
    if not path.is_file() or not os.access(path, os.X_OK):
        raise GateFailed(f"{label} is missing or not executable: {path}")
    return path


def evaluate_topics(payload: Any) -> tuple[bool, list[str], dict[str, dict[str, Any]]]:
    """Validate fresh sensor/SLAM samples and the absence of final velocity samples."""

    if not isinstance(payload, Mapping):
        raise GateFailed("DDS observations are not a topic mapping")
    rows = {str(topic): dict(row) for topic, row in payload.items() if isinstance(row, Mapping)}
    blockers: list[str] = []
    for topic, (minimum_samples, minimum_hz) in _TOPIC_REQUIREMENTS.items():
        row = rows.get(topic, {})
        samples = int(row.get("samples", 0) or 0)
        hz = float(row.get("rate_hz", 0.0) or 0.0)
        last_ts = float(row.get("last_ts", 0.0) or 0.0)
        if samples < minimum_samples or hz < minimum_hz or last_ts <= 0.0:
            blockers.append(f"{topic}: samples={samples}, hz={hz:.2f}, last_ts={last_ts:.3f}")
    cmd_vel = rows.get(_CMD_VEL_TOPIC)
    if cmd_vel is None:
        blockers.append(f"{_CMD_VEL_TOPIC}: probe result missing")
        return False, blockers, rows
    cmd_vel_samples = int(cmd_vel.get("samples", 0) or 0)
    if cmd_vel_samples != 0:
        blockers.append(f"{_CMD_VEL_TOPIC}: unexpected samples={cmd_vel_samples}")
    return not blockers, blockers, rows


def _start(command: list[str], log_path: Path, environment: dict[str, str]) -> tuple[subprocess.Popen[bytes], Any]:
    log = log_path.open("wb")
    process = subprocess.Popen(  # noqa: S603 - commands are resolved from the trusted RunPlan/release.
        command,
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, log


def _stop(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)  # type: ignore[attr-defined]
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)  # type: ignore[attr-defined]
        except ProcessLookupError:
            pass
        process.wait(timeout=2.0)


def _tail(path: Path, lines: int = 30) -> str:
    if not path.exists():
        return ""
    return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])


def run_gate(*, repo: Path, output_dir: Path, seconds: float, settle_seconds: float) -> dict[str, Any]:
    """Collect one bounded sensor-only field run and return its evidence report."""

    if output_dir.exists():
        if any(output_dir.iterdir()):
            raise FileExistsError(output_dir)
    else:
        output_dir.mkdir(parents=True)
    cloud_dir = output_dir / "clouds"
    cloud_dir.mkdir()
    lidar_log = output_dir / "livox.log"
    slam_log = output_dir / "slam.log"
    status_path = output_dir / "slam_status.json"
    report_path = output_dir / "report.json"
    lidar_process: subprocess.Popen[bytes] | None = None
    slam_process: subprocess.Popen[bytes] | None = None
    logs: list[Any] = []

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "ok": False,
    }
    try:
        plan_environment = _resolved_environment(repo)
        _assert_motion_inactive()
        network = _assert_network(plan_environment)

        environment = dict(os.environ)
        environment.update(plan_environment)
        environment.setdefault("LINGTU_DDS_DOMAIN_ID", "0")
        environment["LINGTU_CONFIG_PATH"] = plan_environment["LINGTU_CONFIG_PATH"]

        livox = _require_executable(
            Path(environment.get("LINGTU_LIVOX_BIN", repo / "build/livox_sdk2_stream/livox_sdk2_stream")),
            "Livox DDS publisher",
        )
        slam = _require_executable(
            Path(environment.get("LINGTU_SLAM_BIN", repo / "build/slam_core/slamd")),
            "SLAM runtime",
        )
        slam_config = Path(environment["LINGTU_SLAM_CONFIG"])
        if not slam_config.is_file():
            raise GateFailed(f"SLAM config is missing: {slam_config}")

        robot_config = load_config(environment["LINGTU_CONFIG_PATH"])
        generated_livox = output_dir / "MID360_config.json"
        generated_livox.write_text(
            json.dumps(
                build_mid360_config_dict(
                    robot_config,
                    host_ip=network["host_ip"],
                    lidar_ip=network["lidar_ip"],
                ),
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        lidar_process, lidar_handle = _start(
            [
                str(livox),
                "--dds",
                "--domain-id",
                environment["LINGTU_DDS_DOMAIN_ID"],
                "--publish-freq",
                "10",
                "--lidar-frame",
                "lidar_link",
                "--imu-frame",
                "imu_link",
                str(generated_livox),
            ],
            lidar_log,
            environment,
        )
        logs.append(lidar_handle)
        time.sleep(1.0)
        if lidar_process.poll() is not None:
            raise GateFailed(f"Livox publisher exited during startup\n{_tail(lidar_log)}")

        slam_process, slam_handle = _start(
            [
                str(slam),
                "--backend",
                "fastlio2",
                "--mode",
                "mapping",
                "--config",
                str(slam_config),
                "--domain-id",
                environment["LINGTU_DDS_DOMAIN_ID"],
                "--tick-hz",
                "50",
                "--log-status-s",
                "5",
                "--status-json",
                str(status_path),
                "--status-json-hz",
                "10",
                "--cloud-snapshot-dir",
                str(cloud_dir),
                "--cloud-snapshot-hz",
                "5",
                "--lidar-scan-snapshot-hz",
                "10",
            ],
            slam_log,
            environment,
        )
        logs.append(slam_handle)
        time.sleep(settle_seconds)
        for name, process, path in (
            ("Livox publisher", lidar_process, lidar_log),
            ("SLAM runtime", slam_process, slam_log),
        ):
            if process.poll() is not None:
                raise GateFailed(f"{name} exited before probing\n{_tail(path)}")

        readiness = collect_readiness(
            (*_TOPIC_REQUIREMENTS, _CMD_VEL_TOPIC),
            seconds=seconds,
            domain_id=int(environment["LINGTU_DDS_DOMAIN_ID"]),
        )
        _, blockers, rows = evaluate_topics(readiness["topics"])
        if not status_path.is_file():
            raise GateFailed("SLAM did not publish its status snapshot")
        slam_status = json.loads(status_path.read_text(encoding="utf-8"))
        slam_state = str(slam_status.get("state", ""))
        if slam_state != "TRACKING":
            blockers.append(f"SLAM state is {slam_state or 'missing'}, expected TRACKING")
        _assert_motion_inactive()

        report.update(
            {
                "ok": not blockers,
                "blockers": blockers,
                "network": network,
                "topics": rows,
                "slam_state": slam_state,
            }
        )
        return report
    except Exception as exc:
        report.setdefault("blockers", []).append(str(exc))
        return report
    finally:
        _stop(slam_process)
        _stop(lidar_process)
        for log in logs:
            log.close()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0, help="DDS sample window")
    parser.add_argument("--settle-seconds", type=float, default=8.0, help="SLAM initialization wait")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the no-motion gate CLI."""

    args = _parser().parse_args(argv)
    if args.seconds < 3.0 or args.settle_seconds < 1.0:
        print("ERROR: --seconds must be >= 3 and --settle-seconds must be >= 1", file=sys.stderr)
        return 2
    repo = Path(os.environ.get("LINGTU_REPO", Path(__file__).resolve().parents[3])).resolve()
    output_dir = args.output_dir or Path(
        tempfile.mkdtemp(prefix=f"lingtu-go2-mid360-no-motion-{time.strftime('%Y%m%d-%H%M%S')}-")
    )
    try:
        result = run_gate(
            repo=repo,
            output_dir=output_dir,
            seconds=args.seconds,
            settle_seconds=args.settle_seconds,
        )
    except FileExistsError:
        print(f"ERROR: output directory already exists: {output_dir}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        outcome = "PASS" if result.get("ok") else "FAIL"
        print(f"{outcome}: Go2 MID-360 no-motion gate")
        for blocker in result.get("blockers", []):
            print(f"  - {blocker}")
        print(f"Evidence: {output_dir / 'report.json'}")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
