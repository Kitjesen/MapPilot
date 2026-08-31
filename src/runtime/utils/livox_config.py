"""Livox MID-360 driver config generation.

Enterprise-grade rule: single source of truth.

We keep all network addresses in the selected RobotConfig (RobotConfig.lidar),
and generate one JSON file that both the ROS-free SDK2 source and the
livox_ros_driver2 compatibility path can consume.

This avoids manual edits to vendor JSON files and prevents config drift across
robots / deployments.
"""

from __future__ import annotations

import json
import os
from ipaddress import ip_address, ip_interface
from pathlib import Path
from typing import Any

from runtime.config import RobotConfig


def _env_bool(name: str) -> bool | None:
    raw = os.environ.get(name)
    if raw is None:
        return None
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_text(name: str) -> str | None:
    raw = os.environ.get(name)
    if raw is None:
        return None
    value = raw.strip()
    return value or None


def select_livox_host_ip(
    cfg: RobotConfig,
    interface_addresses: list[str] | tuple[str, ...],
    *,
    host_ip: str | None = None,
    lidar_ip: str | None = None,
) -> str:
    """Select the host IP used in the Livox SDK2 config.

    ``robot_config.yaml`` is the source of truth. Interface probing is only a
    fallback for incomplete field configs, and prefers an address on the same
    subnet as the configured LiDAR.
    """

    configured_host = str(host_ip or cfg.lidar.host_ip or "").strip()
    if configured_host:
        return configured_host

    resolved_lidar_ip = str(lidar_ip or cfg.lidar.lidar_ip or "").strip()
    lidar_addr = None
    if resolved_lidar_ip:
        try:
            lidar_addr = ip_address(resolved_lidar_ip)
        except ValueError:
            lidar_addr = None

    parsed_interfaces = [addr for raw in interface_addresses if (addr := _parse_interface_address(raw)) is not None]
    if lidar_addr is not None:
        for iface in parsed_interfaces:
            if iface.version == lidar_addr.version and lidar_addr in iface.network:
                return str(iface.ip)
    if parsed_interfaces:
        return str(parsed_interfaces[0].ip)
    return ""


def _parse_interface_address(raw: str):
    text = str(raw or "").strip()
    if not text:
        return None
    if "/" not in text:
        text = f"{text}/32"
    try:
        return ip_interface(text)
    except ValueError:
        return None


def build_mid360_config_dict(
    cfg: RobotConfig,
    *,
    host_ip: str | None = None,
    lidar_ip: str | None = None,
    multicast_ip: str | None = None,
    bind_lidar_ip: bool | None = None,
) -> dict[str, Any]:
    """Return a MID360_config.json dict for SDK2 and livox_ros_driver2."""
    resolved_lidar_ip = lidar_ip or cfg.lidar.lidar_ip
    resolved_host_ip = host_ip or cfg.lidar.host_ip

    # Ports follow Livox reference defaults; override via cfg.raw['lidar'] if needed.
    raw_lidar = (cfg.raw or {}).get("lidar", {})
    ports = raw_lidar.get("livox_ports", {}) if isinstance(raw_lidar, dict) else {}
    resolved_multicast_ip = (
        multicast_ip if multicast_ip is not None else str(raw_lidar.get("livox_multicast_ip", "224.1.1.5"))
    )
    resolved_bind_lidar_ip = (
        bool(raw_lidar.get("livox_bind_lidar_ip", False)) if bind_lidar_ip is None else bool(bind_lidar_ip)
    )

    def p(name: str, default: int) -> int:
        try:
            v = int(ports.get(name, default))
        except Exception:
            v = default
        return v

    host_info: dict[str, Any] = {
        "host_ip": resolved_host_ip,
        "multicast_ip": resolved_multicast_ip,
        "cmd_data_port": p("cmd_data_port_host", 56101),
        "push_msg_port": p("push_msg_port_host", 56201),
        "point_data_port": p("point_data_port_host", 56301),
        "imu_data_port": p("imu_data_port_host", 56401),
        "log_data_port": p("log_data_port_host", 56501),
    }
    if resolved_bind_lidar_ip and resolved_lidar_ip:
        host_info["lidar_ip"] = [resolved_lidar_ip]

    cfg_dict: dict[str, Any] = {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": p("cmd_data_port", 56100),
                "push_msg_port": p("push_msg_port", 56200),
                "point_data_port": p("point_data_port", 56300),
                "imu_data_port": p("imu_data_port", 56400),
                "log_data_port": p("log_data_port", 56500),
            },
            # Keep the official array form. By default we do not pin lidar_ip:
            # field deployments often keep the MID-360 on a custom subnet and
            # SDK2 discovery is more robust than stale config values.
            "host_net_info": [host_info],
        },
        "lidar_configs": [
            {
                "ip": resolved_lidar_ip,
                "pcl_data_type": int(raw_lidar.get("livox_pcl_data_type", 1)),
                "pattern_mode": int(raw_lidar.get("livox_pattern_mode", 0)),
                # Keep extrinsics at zero here. SLAM uses the selected RobotConfig
                # as the unified TF source of truth.
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }
    return cfg_dict


def ensure_mid360_config_file(cfg: RobotConfig, out_dir: str | None = None) -> str:
    """Write a generated MID360_config.json and return its absolute path."""
    base = Path(out_dir).expanduser() if out_dir else Path(os.path.expanduser("~/.lingtu/generated/livox"))
    base.mkdir(parents=True, exist_ok=True)

    path = base / "MID360_config.json"
    data = build_mid360_config_dict(
        cfg,
        host_ip=_env_text("LINGTU_LIVOX_HOST_IP"),
        lidar_ip=_env_text("LINGTU_LIVOX_LIDAR_IP"),
        multicast_ip=_env_text("LINGTU_LIVOX_MULTICAST_IP"),
        bind_lidar_ip=_env_bool("LINGTU_LIVOX_BIND_LIDAR_IP"),
    )
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return str(path)
