"""Livox MID-360 driver config generation.

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

import yaml

from runtime.config import RobotConfig


def lidar_settings() -> dict[str, Any]:
    path = Path(__file__).resolve().parents[3] / "config/system/settings/lidar.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["lidar"]


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
    settings: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a MID360_config.json dict for SDK2 and livox_ros_driver2."""
    resolved_lidar_ip = lidar_ip or cfg.lidar.lidar_ip
    resolved_host_ip = host_ip or cfg.lidar.host_ip

    sdk = (lidar_settings() if settings is None else settings)["sdk"]
    if set(sdk) != {"pcl_data_type", "pattern_mode", "multicast_ip", "bind_lidar_ip", "device_ports", "host_ports"}:
        raise ValueError("lidar.sdk has missing or unknown settings")
    if not isinstance(sdk["bind_lidar_ip"], bool):
        raise ValueError("lidar.sdk.bind_lidar_ip must be a boolean")
    raw_lidar = (cfg.raw or {}).get("lidar", {})
    ports = raw_lidar.get("livox_ports", {}) if isinstance(raw_lidar, dict) else {}
    resolved_multicast_ip = (
        multicast_ip if multicast_ip is not None else str(raw_lidar.get("livox_multicast_ip", sdk["multicast_ip"]))
    )
    resolved_bind_lidar_ip = (
        bool(raw_lidar.get("livox_bind_lidar_ip", sdk["bind_lidar_ip"])) if bind_lidar_ip is None else bool(bind_lidar_ip)
    )

    channel_names = {"command": "cmd_data_port", "status": "push_msg_port", "points": "point_data_port", "imu": "imu_data_port", "log": "log_data_port"}
    port_sets = {}
    for owner in ("device", "host"):
        values = sdk[f"{owner}_ports"]
        if not isinstance(values, dict) or set(values) != channel_names.keys():
            raise ValueError(f"lidar.sdk.{owner}_ports has missing or unknown channels")
        port_sets[owner] = {
            target: int(ports.get(target + ("_host" if owner == "host" else ""), values[channel]))
            for channel, target in channel_names.items()
        }

    host_info: dict[str, Any] = {
        "host_ip": resolved_host_ip,
        "multicast_ip": resolved_multicast_ip,
        **port_sets["host"],
    }
    if resolved_bind_lidar_ip and resolved_lidar_ip:
        host_info["lidar_ip"] = [resolved_lidar_ip]

    cfg_dict: dict[str, Any] = {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": port_sets["device"],
            # Keep the official array form. By default we do not pin lidar_ip:
            # field deployments often keep the MID-360 on a custom subnet and
            # SDK2 discovery is more robust than stale config values.
            "host_net_info": [host_info],
        },
        "lidar_configs": [
            {
                "ip": resolved_lidar_ip,
                "pcl_data_type": int(raw_lidar.get("livox_pcl_data_type", sdk["pcl_data_type"])),
                "pattern_mode": int(raw_lidar.get("livox_pattern_mode", sdk["pattern_mode"])),
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


def ensure_mid360_config_file(cfg: RobotConfig | None, out_dir: str | None = None) -> str:
    """Write a generated MID360_config.json and return its absolute path."""
    base = Path(out_dir).expanduser() if out_dir else Path(os.path.expanduser("~/.lingtu/generated/livox"))
    base.mkdir(parents=True, exist_ok=True)

    path = base / "MID360_config.json"
    snapshot = os.environ.get("LINGTU_LIVOX_CONFIG_JSON") if cfg is None else None
    if not snapshot and cfg is None:
        raise ValueError("Livox configuration requires RobotConfig or a compiled Product snapshot")
    data = json.loads(snapshot) if snapshot else build_mid360_config_dict(
        cfg,
        host_ip=_env_text("LINGTU_LIVOX_HOST_IP"),
        lidar_ip=_env_text("LINGTU_LIVOX_LIDAR_IP"),
        multicast_ip=_env_text("LINGTU_LIVOX_MULTICAST_IP"),
        bind_lidar_ip=_env_bool("LINGTU_LIVOX_BIND_LIDAR_IP"),
    )
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return str(path)
