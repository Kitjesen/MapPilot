#!/usr/bin/env python3
"""Validate saved-map relocalization product contracts without moving hardware."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from runtime.runtime_policy import (
    backend_capability_defaults,
    default_slam_profile_for_mode,
    session_transition_plan,
    slam_backend_contract,
    slam_switch_plan,
)

ROOT = Path(__file__).resolve().parents[2]


def _bool(value: Any) -> bool:
    return bool(value)


def _collect_bridge_status(profile: str) -> dict[str, Any]:
    from localization.bridge import SlamBridgeModule

    module = SlamBridgeModule(
        backend_profile=profile,
        odom_timeout=0.5,
        cloud_timeout=0.5,
        localizer_health_timeout=0.5,
        watchdog_hz=50,
    )
    seen: list[dict[str, Any]] = []
    module.localization_status._add_callback(seen.append)
    module._mark_odom_received()
    module._mark_cloud_received()
    if profile == "localizer":
        class HealthMsg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        module._on_rclpy_localization_health(HealthMsg())
    module.start()
    try:
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if any(item.get("state") == "TRACKING" for item in seen):
                break
            time.sleep(0.02)
    finally:
        module.stop()
    return next((item for item in seen if item.get("state") == "TRACKING"), seen[-1] if seen else {})


def run_gate() -> dict[str, Any]:
    launch_path = ROOT / "launch/profiles/localizer_icp.launch.py"
    launch_text = launch_path.read_text(encoding="utf-8")

    contracts = {
        name: slam_backend_contract(name)
        for name in ("native_dds", "localizer", "fastlio2", "super_lio", "super_lio_relocation")
    }
    capability_defaults = {
        name: backend_capability_defaults(name)
        for name in ("native_dds", "localizer", "fastlio2", "super_lio", "super_lio_relocation")
    }
    navigating_plan = session_transition_plan("navigating", "native_dds")
    localizer_switch = slam_switch_plan("localizer")

    localizer_status = _collect_bridge_status("localizer")
    super_lio_status = _collect_bridge_status("super_lio")

    launch_services = {
        "/slam/relocalize": "/slam/relocalize" in launch_text,
        "/slam/relocalize_check": "/slam/relocalize_check" in launch_text,
        "/slam/global_relocalize": "/slam/global_relocalize" in launch_text,
        "/slam/saved_map_cloud": "/slam/saved_map_cloud" in launch_text,
    }
    unsupported_backends_block_saved_map_relocalize = all(
        not _bool(contracts[name]["saved_map_relocalization_supported"])
        for name in ("fastlio2", "super_lio", "super_lio_relocation")
    )
    localizer_contract_ok = (
        contracts["localizer"]["health_source"] == "localizer_health_topic"
        and contracts["localizer"]["map_save_source"] == "active_map"
        and contracts["localizer"]["recovery_method"] == "relocalize_service"
        and contracts["localizer"]["recovery_action"] == "relocalize_service"
        and _bool(contracts["localizer"]["relocalization_supported"])
        and _bool(contracts["localizer"]["saved_map_relocalization_supported"])
    )
    status_ok = (
        localizer_status.get("backend") == "localizer"
        and localizer_status.get("health_source") == "localizer_health_topic"
        and localizer_status.get("localizer_health") == "LOCKED"
        and localizer_status.get("relocalization_state") == "idle"
        and _bool(localizer_status.get("pose_fresh"))
        and _bool(localizer_status.get("saved_map_relocalization_supported"))
        and super_lio_status.get("backend") == "super_lio"
        and not _bool(super_lio_status.get("saved_map_relocalization_supported"))
        and super_lio_status.get("relocalization_state") == "unsupported"
    )
    nav_mode_ok = (
        default_slam_profile_for_mode("navigating") == "native_dds"
        and navigating_plan.ensure == ("slam",)
        and "slam_pgo" in navigating_plan.stop
        and localizer_switch.ensure == ("slam",)
        and "super_lio" in localizer_switch.stop
        and "super_lio_relocation" in localizer_switch.stop
    )

    blockers: list[str] = []
    if not localizer_contract_ok:
        blockers.append("localizer saved-map relocalization contract is not conservative and service-backed")
    if not unsupported_backends_block_saved_map_relocalize:
        blockers.append("non-localizer backend exposes saved-map relocalization")
    if not all(launch_services.values()):
        missing = [name for name, present in launch_services.items() if not present]
        blockers.append(f"localizer launch remaps missing: {', '.join(missing)}")
    if not nav_mode_ok:
        blockers.append("navigation mode does not force native DDS service chain")
    if not status_ok:
        blockers.append("SlamBridge status does not expose localizer relocalize readiness correctly")

    return {
        "schema_version": "lingtu.saved_map_relocalize_contract.v1",
        "ok": not blockers,
        "validation_level": "contract_only_no_runtime_relocalization",
        "runtime_stage": "saved_map_relocalization",
        "map_dependency": "saved_map_required",
        "requires_saved_map": True,
        "requires_live_slam": False,
        "requires_tomogram": False,
        "runtime_relocalization_executed": False,
        "runtime_relocalization_validated": False,
        "validated_claims": (
            "saved_map_relocalization_service_contract",
            "localizer_bridge_status_contract",
        ),
        "forbidden_claims": (
            "runtime_relocalization_validated",
            "saved_map_navigation_validated",
            "pct_saved_map_navigation_validated",
        ),
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "blockers": blockers,
        "default_profiles": {
            "navigating": default_slam_profile_for_mode("navigating"),
            "mapping": default_slam_profile_for_mode("mapping"),
        },
        "contracts": contracts,
        "capability_defaults": capability_defaults,
        "plans": {
            "session_navigating_native_dds": {
                "stop": navigating_plan.stop,
                "ensure": navigating_plan.ensure,
                "wait_ready": navigating_plan.wait_ready,
                "clear_live_map": navigating_plan.clear_live_map,
            },
            "switch_localizer": {
                "stop": localizer_switch.stop,
                "ensure": localizer_switch.ensure,
                "wait_ready": localizer_switch.wait_ready,
            },
        },
        "launch_services": launch_services,
        "bridge_status": {
            "localizer": {
                "state": localizer_status.get("state"),
                "backend": localizer_status.get("backend"),
                "health_source": localizer_status.get("health_source"),
                "localizer_health": localizer_status.get("localizer_health"),
                "pose_fresh": localizer_status.get("pose_fresh"),
                "saved_map_relocalization_supported": localizer_status.get(
                    "saved_map_relocalization_supported"
                ),
                "relocalization_state": localizer_status.get("relocalization_state"),
                "recovery_method": localizer_status.get("recovery_method"),
            },
            "super_lio": {
                "state": super_lio_status.get("state"),
                "backend": super_lio_status.get("backend"),
                "health_source": super_lio_status.get("health_source"),
                "saved_map_relocalization_supported": super_lio_status.get(
                    "saved_map_relocalization_supported"
                ),
                "relocalization_state": super_lio_status.get("relocalization_state"),
                "recovery_method": super_lio_status.get("recovery_method"),
            },
        },
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/saved_map_relocalize/report.json",
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate()
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
