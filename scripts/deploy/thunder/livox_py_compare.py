#!/usr/bin/env python3
"""Compare the native Livox DDS service with the Python DDS reader path.

This is a read-only field diagnostic. It does not start or stop services and it
does not open the Livox device. The native C++ service owns the device; this
script only checks service/config state and, when cyclonedds-python is present,
subscribes to the same DDS output topics as a Python-side comparison.
"""

from __future__ import annotations

import argparse
import importlib
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


DEFAULT_CONFIG = Path("/opt/lingtu/config/livox/MID360_config.json")
LIVOX_SERVICE = "lingtu-livox-dds.service"
SLAM_STATUS = Path("/tmp/lingtu_slam_status.json")


@dataclass
class TopicStats:
    samples: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    frame_id: str = ""
    points: int | None = None

    def observe(self, msg: Any) -> None:
        now = time.time()
        if self.samples == 0:
            self.first_ts = now
        self.samples += 1
        self.last_ts = now
        header = getattr(msg, "header", None)
        if header is not None:
            self.frame_id = str(getattr(header, "frame_id", "") or self.frame_id)
        self.points = _point_count(msg, self.points)

    def hz(self) -> float:
        if self.samples < 2:
            return 0.0
        return (self.samples - 1) / max(self.last_ts - self.first_ts, 1e-9)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)

    from runtime.runtime_interface import TOPICS

    print("== native C++ Livox service ==")
    print(f"service: {LIVOX_SERVICE} active={_service_active(LIVOX_SERVICE)}")
    _print_livox_process()
    _print_livox_config(args.config)
    _print_slam_lidar_rate()

    print()
    print("== Python DDS comparison ==")
    topics = (TOPICS.lidar_scan, TOPICS.imu)
    return _probe_python_dds(topics, seconds=args.seconds, domain_id=args.domain)


def _service_active(name: str) -> str:
    result = _run(["systemctl", "is-active", name])
    return result.stdout.strip() or result.stderr.strip() or f"exit={result.returncode}"


def _print_livox_process() -> None:
    result = _run(["pgrep", "-a", "-f", "livox_sdk2_stream"])
    lines = [line for line in result.stdout.splitlines() if "livox_sdk2_stream" in line]
    if not lines:
        print("process: not found")
        return
    for line in lines[:3]:
        print(f"process: {line}")


def _print_livox_config(path: Path) -> None:
    print(f"config: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"config_error: {type(exc).__name__}: {exc}")
        return
    host_info = (data.get("MID360") or {}).get("host_net_info") or []
    lidar_configs = data.get("lidar_configs") or []
    if host_info:
        host = host_info[0]
        print(
            f"host_ip: {host.get('host_ip')} point_port={host.get('point_data_port')} imu_port={host.get('imu_data_port')}"
        )
    if lidar_configs:
        lidar = lidar_configs[0]
        print(f"lidar_ip: {lidar.get('ip')} pcl_data_type={lidar.get('pcl_data_type')}")


def _print_slam_lidar_rate() -> None:
    if not SLAM_STATUS.exists():
        print(f"slam_status: missing {SLAM_STATUS}")
        return
    try:
        status = json.loads(SLAM_STATUS.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"slam_status_error: {type(exc).__name__}: {exc}")
        return
    keys = ("state", "lidar_input_hz", "imu_input_hz", "processed_scan_hz", "registered_points")
    parts = [f"{key}={status.get(key)}" for key in keys]
    print("slam_status: " + " ".join(parts))


def _probe_python_dds(topics: tuple[str, ...], *, seconds: float, domain_id: int) -> int:
    try:
        from message.dds import dds_type_for_topic

        reader_mod, DDSReader = _load_dds_reader()
    except Exception as exc:
        print(f"python_dds_import: unavailable ({type(exc).__name__}: {exc})")
        return 2

    if not getattr(reader_mod, "_HAS_CYCLONEDDS", False):
        print("python_dds_import: unavailable (cyclonedds-python is not installed or disabled)")
        print("result: C++ service may be healthy, but Python cannot subscribe for comparison on this host.")
        return 2

    reader = DDSReader(domain_id=domain_id)
    stats = {topic: TopicStats() for topic in topics}
    for topic in topics:
        dds_type = dds_type_for_topic(topic)
        if dds_type is None:
            print(f"{topic}: no typed DDS contract")
            return 2
        reader.subscribe(topic, dds_type, lambda msg, topic=topic: stats[topic].observe(msg))

    if not reader.spin_background():
        print("python_dds_reader: failed to start")
        return 2
    try:
        time.sleep(max(seconds, 0.1))
    finally:
        reader.stop()

    print(f"{'topic':24} {'samples':>7} {'hz':>7} {'frame':16} {'points':>8} {'age_s':>7}")
    now = time.time()
    ok = True
    for topic, item in stats.items():
        ok = ok and item.samples > 0
        points = "" if item.points is None else str(item.points)
        age = now - item.last_ts if item.last_ts else 0.0
        print(f"{topic:24} {item.samples:7d} {item.hz():7.2f} {item.frame_id[:16]:16} {points:>8} {age:7.2f}")
    return 0 if ok else 1


def _load_dds_reader() -> tuple[Any, Any]:
    mod = importlib.import_module("runtime.adapters.dds.reader")
    return mod, mod.DDSReader


def _point_count(msg: Any, default: int | None = None) -> int | None:
    if hasattr(msg, "point_num"):
        return int(msg.point_num)
    if hasattr(msg, "points"):
        try:
            return len(msg.points)
        except TypeError:
            pass
    if hasattr(msg, "width") and hasattr(msg, "height"):
        return int(msg.width) * int(msg.height)
    return default


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(args, text=True, capture_output=True, timeout=3.0, check=False)
    except Exception as exc:
        return subprocess.CompletedProcess(args, 99, "", f"{type(exc).__name__}: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
