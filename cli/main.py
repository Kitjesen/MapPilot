"""Argument parsing and process lifecycle for the LingTu CLI."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import textwrap
import threading
from pathlib import Path

from . import term as T
from .lifecycle import (
    daemonize,
    health_check,
    kill_residual_ports,
    lite_runtime_lifecycle_blockers,
    needs_full_preflight,
)
from .logging_util import setup_logging
from .profiles_data import PROFILES
from .run_state import (
    _lingtu_version,
    clear_run_state,
    save_run_state,
    update_run_state,
)
from .runtime_bootstrap import install_runtime_plugin_catalog, seed_runtime_plugins
from .runtime_display import (
    format_frame_links,
    format_inspection_acceptance,
    format_product_field_check,
    format_real_runtime_evidence_summary,
    format_runtime_audit_payload,
    format_runtime_boundary,
    format_runtime_contract_manifest,
    format_runtime_flow,
    format_runtime_flow_stages,
    format_runtime_frames,
    format_runtime_sources,
    format_runtime_spec_payload,
    format_runtime_switch_plan,
    format_runtime_topic_frames,
)
from .term import IS_TTY
from .ui import (
    cmd_health_external,
    cmd_log_external,
    cmd_restart,
    cmd_show_config_external,
    cmd_status_external,
    cmd_stop,
    list_profiles,
    print_banner,
    select_interactive,
    wizard_interactive,
)

logger = logging.getLogger("lingtu")


_SPECIAL_COMMANDS = {
    "stop",
    "restart",
    "status",
    "show-config",
    "log",
    "health",
    "doctor",
    "rerun",
    "switch-plan",
    "runtime-contract",
    "runtime-spec",
    "runtime-audit",
    "dataflow",
    "gateway-runtime-acceptance",
    "field-check",
    "inspection-check",
    "saved-map-artifact-gate",
    "real-runtime-evidence",
    "mujoco",
    "portable-mujoco",
    "mujoco-portable",
}


def _runtime_endpoint(name: str):
    from runtime.profiles.endpoints import runtime_endpoint

    return runtime_endpoint(name)


def _module_transport_names() -> tuple[str, ...]:
    from runtime.transport.abc import TransportStrategy

    return tuple(strategy.value for strategy in TransportStrategy)


def _canonical_profile_name(profile_name: str) -> str:
    from runtime.profiles.resolver import canonical_profile_name

    return canonical_profile_name(profile_name)


def _is_runtime_endpoint_error(exc: Exception) -> bool:
    from runtime.profiles.endpoints import RuntimeEndpointError

    return isinstance(exc, RuntimeEndpointError)


def _apply_runtime_process_env(profile_name: str, cfg: dict, args: argparse.Namespace):
    import os

    from runtime.profiles.launcher import resolve_runtime_process_context
    from runtime.runtime_switch import validate_runtime_switch

    context = resolve_runtime_process_context(
        profile_name,
        cfg,
        record=bool(getattr(args, "record", False)),
        extra_args=tuple(args.extra or ()),
    )
    spec = context.spec
    validation = validate_runtime_switch(spec)
    if not validation.ok:
        print(f"  {T.red('Error')}: runtime boundary invalid")
        for blocker in validation.blockers:
            print(f"    - {blocker}")
        sys.exit(2)
    for key, value in context.env.items():
        os.environ[key] = value
    return spec


def preflight(profile_name: str, cfg: dict) -> None:
    blockers = lite_runtime_lifecycle_blockers(profile_name, cfg)
    if blockers:
        _exit_lite_lifecycle_error(profile_name, blockers)
    if not needs_full_preflight(cfg):
        return
    try:
        from .runtime_extra import preflight
    except ModuleNotFoundError:
        print(
            f"  {T.red('Error')}: profile {profile_name!r} requires the full "
            "runtime preflight module, but this package only contains the Lite runtime."
        )
        sys.exit(2)
    preflight(profile_name, cfg)


def _exit_lite_lifecycle_error(profile_name: str, blockers: tuple[str, ...]) -> None:
    print(
        f"  {T.red('Error')}: profile {profile_name!r} is a Lite runtime "
        "but requested full-stack lifecycle features."
    )
    for blocker in blockers:
        print(f"    - {blocker}")
    sys.exit(2)


def _preflight(profile_name: str, cfg: dict) -> None:
    preflight(profile_name, cfg)


def _run_external_profile_launcher(
    profile_name: str,
    cfg: dict,
    args: argparse.Namespace,
    repo_root: Path,
) -> None:
    """Run a first-class profile that delegates to a simulator launcher."""

    import os
    import subprocess

    from runtime.profiles.launcher import build_external_launch_context

    context = build_external_launch_context(
        profile_name,
        cfg,
        repo_root=repo_root,
        record=bool(getattr(args, "record", False)),
        extra_args=tuple(args.extra or ()),
        base_env=os.environ.copy(),
    )
    spec = context.spec
    if not context.launcher_path.exists():
        print(f"  {T.red('Error')}: launcher not found: {context.launcher_path}")
        sys.exit(1)
    if args.daemon:
        print(
            f"  {T.red('Error')}: --daemon is not supported for external simulation launchers; "
            "use the launcher's status/stop commands."
        )
        sys.exit(2)

    print(f"\n  Launching external simulation profile ({T.green(profile_name)})...", flush=True)
    print(f"  Launcher: {spec.launcher}", flush=True)
    print(f"  Runtime:  {format_runtime_boundary(spec)}", flush=True)
    print(f"  SLAM:     {format_runtime_sources(spec)}", flush=True)
    print(f"  Frame ids: {format_runtime_frames(spec)}", flush=True)
    print(f"  Frames:   {format_frame_links(spec)}", flush=True)
    print(f"  Topic frames: {format_runtime_topic_frames(spec)}", flush=True)
    print(f"  Flow:     {format_runtime_flow(spec)}", flush=True)
    print(f"  Flow stages: {format_runtime_flow_stages(spec)}", flush=True)
    command = list(context.command)
    print(f"  Command:  {' '.join(command)}", flush=True)
    try:
        proc = subprocess.run(
            command,
            cwd=str(repo_root),
            env=dict(context.env),
            check=False,
        )
    except FileNotFoundError:
        required = "python" if str(spec.launcher).endswith(".py") else "bash"
        print(f"  {T.red('Error')}: {required} is required to run {spec.launcher}")
        sys.exit(127)
    if proc.returncode:
        sys.exit(proc.returncode)


def _cmd_switch_plan(args: argparse.Namespace) -> None:
    import json

    from runtime.profiles.endpoints import resolve_runtime_run_spec
    from runtime.profiles.product_mode_contracts import (
        PRODUCT_MODE_CONTRACTS,
        product_mode_switch_plan,
    )
    from runtime.runtime_switch import compare_runtime_switch, validate_runtime_switch

    if len(args.extra) < 2:
        print(
            "  Usage: lingtu switch-plan <current-profile> <target-profile> "
            "[--current-endpoint ENDPOINT] [--endpoint ENDPOINT]"
        )
        sys.exit(1)
    current_profile = _canonical_profile_name(args.extra[0])
    target_profile = _canonical_profile_name(args.extra[1])

    current_args = argparse.Namespace(**vars(args))
    current_args.endpoint = args.current_endpoint
    target_args = argparse.Namespace(**vars(args))
    current_cfg = _resolve_config(current_profile, current_args, allow_wizard=False)
    target_cfg = _resolve_config(target_profile, target_args, allow_wizard=False)
    current_spec = resolve_runtime_run_spec(current_profile, current_cfg)
    target_spec = resolve_runtime_run_spec(target_profile, target_cfg)
    current_validation = validate_runtime_switch(current_spec)
    target_validation = validate_runtime_switch(target_spec)
    payload = compare_runtime_switch(current_spec, target_spec)
    payload["product_mode_switch"] = (
        product_mode_switch_plan(current_profile, target_profile)
        if current_profile in PRODUCT_MODE_CONTRACTS
        and target_profile in PRODUCT_MODE_CONTRACTS
        else None
    )
    payload["ok"] = current_validation.ok and target_validation.ok
    payload["current_validation"] = {
        "ok": current_validation.ok,
        "blockers": list(current_validation.blockers),
        "warnings": list(current_validation.warnings),
    }
    payload["target_validation"] = {
        "ok": target_validation.ok,
        "blockers": list(target_validation.blockers),
        "warnings": list(target_validation.warnings),
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_runtime_switch_plan(payload))
    if not current_validation.ok or not target_validation.ok:
        sys.exit(2)


def _cmd_runtime_spec(args: argparse.Namespace) -> None:
    import json

    from runtime.profiles.endpoints import resolve_runtime_run_spec
    from runtime.runtime_switch import runtime_spec_summary, validate_runtime_switch

    if len(args.extra) != 1:
        print("  Usage: lingtu runtime-spec <profile> [--endpoint ENDPOINT]")
        sys.exit(1)
    profile_name = _canonical_profile_name(args.extra[0])
    cfg = _resolve_config(profile_name, args, allow_wizard=False)
    spec = resolve_runtime_run_spec(profile_name, cfg)
    validation = validate_runtime_switch(spec)
    payload = {
        "ok": validation.ok,
        "validation": {
            "ok": validation.ok,
            "blockers": list(validation.blockers),
            "warnings": list(validation.warnings),
        },
        "spec": runtime_spec_summary(spec),
        "env": dict(spec.env),
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_runtime_spec_payload(payload))
    if not validation.ok:
        sys.exit(2)


def _cmd_runtime_contract(args: argparse.Namespace) -> None:
    import json

    from runtime.runtime_interface import runtime_contract_manifest

    if args.extra:
        print("  Usage: lingtu runtime-contract [--json] [--json-out PATH]")
        sys.exit(1)
    payload = runtime_contract_manifest()
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_runtime_contract_manifest(payload))


def _cmd_runtime_audit(args: argparse.Namespace) -> None:
    import json

    from cli.runtime_audit import build_runtime_contract_audit

    if args.extra:
        print("  Usage: lingtu runtime-audit [--json] [--json-out PATH]")
        sys.exit(1)
    payload = build_runtime_contract_audit()
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_runtime_audit_payload(payload))
    if not payload["ok"]:
        sys.exit(2)


def _cmd_gateway_runtime_acceptance(args: argparse.Namespace) -> None:
    import json

    from runtime.gateway_runtime_acceptance import (
        collect_gateway_runtime_acceptance,
        format_gateway_runtime_acceptance,
    )

    if args.extra:
        print(
            "  Usage: lingtu gateway-runtime-acceptance "
            "[--gateway-url URL] [--acceptance-mode non_motion|simulation|field] "
            "[--gateway-timeout-sec SEC] [--json] [--json-out PATH]"
        )
        sys.exit(1)

    payload = collect_gateway_runtime_acceptance(
        gateway_url=args.gateway_url,
        timeout_sec=args.gateway_timeout_sec,
        mode=args.acceptance_mode or "non_motion",
    )
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_gateway_runtime_acceptance(payload))
    if not payload["ok"]:
        sys.exit(2)


def _gateway_get_json(
    gateway_url: str,
    path: str,
    *,
    timeout_sec: float,
    query: dict[str, str] | None = None,
) -> dict:
    import json
    from urllib.error import URLError
    from urllib.parse import urlencode, urljoin
    from urllib.request import Request, urlopen

    base = gateway_url.rstrip("/") + "/"
    url = urljoin(base, path.lstrip("/"))
    if query:
        url = f"{url}?{urlencode(query)}"
    req = Request(url, headers={"Accept": "application/json"})
    try:
        with urlopen(req, timeout=timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))
    except (OSError, URLError, json.JSONDecodeError) as exc:
        return {
            "ok": False,
            "error": "gateway_request_failed",
            "message": str(exc),
            "url": url,
        }


def _format_dataflow_interfaces(items: object) -> str:
    if not isinstance(items, list):
        return "none"
    parts: list[str] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        transport = item.get("transport") or item.get("method") or "interface"
        path = item.get("path") or item.get("payload") or item.get("field") or "unknown"
        parts.append(f"{transport}:{path}")
    return ", ".join(parts) if parts else "none"


def _format_dataflow_topic(payload: dict) -> str:
    status = "PASS" if payload.get("ok") is True else "FAIL"
    topic = payload.get("topic") if isinstance(payload.get("topic"), dict) else {}
    inspection = (
        payload.get("inspection") if isinstance(payload.get("inspection"), dict) else {}
    )
    live = str(bool(inspection.get("live"))).lower()
    communicate = "gateway_commands" if inspection.get("communicate") else "read_only"
    lines = [
        f"Runtime dataflow topic: {status}",
        f"topic={topic.get('topic')} selector={payload.get('selector')}",
        f"live={live} observation={inspection.get('observation_level', 'unknown')}",
        f"payload={_format_dataflow_interfaces(inspection.get('payload_interfaces'))}",
        f"communication={communicate}",
    ]
    write_interfaces = _format_dataflow_interfaces(inspection.get("write_interfaces"))
    if write_interfaces != "none":
        lines.append(f"write_interfaces={write_interfaces}")
    endpoint_topic_required = inspection.get(
        "endpoint_topic_required",
        inspection.get("ros2_topic_required"),
    )
    lines.append(
        "primary=Gateway+ModulePorts "
        f"endpoint_topic_required={str(bool(endpoint_topic_required)).lower()} "
        "adapter=endpoint_only"
    )
    lines.append(
        "arbitrary_publish_supported="
        f"{str(bool(inspection.get('arbitrary_publish_supported'))).lower()}"
    )
    if payload.get("error"):
        lines.append(f"error={payload.get('error')}")
    return "\n".join(lines)


def _format_dataflow_summary(payload: dict) -> str:
    ok = payload.get("ok")
    status = "FAIL" if ok is False or payload.get("error") else "PASS"
    transport_layers = (
        payload.get("transport_layers")
        if isinstance(payload.get("transport_layers"), dict)
        else {}
    )
    module_bus = transport_layers.get("module_port_bus", {})
    endpoint_adapter = (
        transport_layers.get("endpoint_adapter")
        or transport_layers.get("ros2_adapter")
        or {}
    )
    topics = payload.get("topics") if isinstance(payload.get("topics"), list) else []
    live_count = 0
    commandable_topics = 0
    for topic in topics:
        if not isinstance(topic, dict):
            continue
        inspection = topic.get("inspection")
        if isinstance(inspection, dict) and inspection.get("live"):
            live_count += 1
        communication = topic.get("communication")
        if isinstance(communication, dict) and communication.get("allowed"):
            commandable_topics += 1
    stages = (
        payload.get("stage_evidence")
        if isinstance(payload.get("stage_evidence"), list)
        else []
    )
    live_stages = 0
    missing_stages = 0
    for stage in stages:
        if not isinstance(stage, dict):
            continue
        if stage.get("live"):
            live_stages += 1
        if stage.get("status") == "missing" or stage.get("observable") is False:
            missing_stages += 1
    control = (
        payload.get("control_boundary")
        if isinstance(payload.get("control_boundary"), dict)
        else {}
    )
    command_interfaces = (
        control.get("command_interfaces")
        if isinstance(control.get("command_interfaces"), list)
        else []
    )
    lines = [
        f"Runtime dataflow: {status}",
        f"runtime_contract={payload.get('runtime_contract')}",
        "primary=Gateway+ModulePorts "
        f"module_port_bus.primary={str(bool(module_bus.get('primary'))).lower()} "
        f"endpoint_adapter.primary={str(bool(endpoint_adapter.get('primary'))).lower()}",
        f"topics={len(topics)} live_topics={live_count}",
        f"stages={len(stages)} live_stages={live_stages} missing_stages={missing_stages}",
        (
            f"commandable_topics={commandable_topics} "
            f"command_interfaces={len(command_interfaces)}"
        ),
        "arbitrary_publish_supported="
        f"{str(bool(control.get('arbitrary_publish_supported'))).lower()}",
        "Use `python lingtu.py dataflow <topic>` for one stream.",
    ]
    if payload.get("error"):
        lines.append(f"error={payload.get('error')}")
    if payload.get("message"):
        lines.append(f"message={payload.get('message')}")
    if payload.get("url"):
        lines.append(f"url={payload.get('url')}")
    return "\n".join(lines)


def _cmd_dataflow(args: argparse.Namespace) -> None:
    import json

    if len(args.extra) > 1:
        print(
            "  Usage: lingtu dataflow [topic] "
            "[--gateway-url URL] [--gateway-timeout-sec SEC] [--json]"
        )
        sys.exit(1)
    topic = args.topic or (args.extra[0] if args.extra else None)
    if topic:
        payload = _gateway_get_json(
            args.gateway_url,
            "/api/v1/runtime/dataflow/topic",
            timeout_sec=args.gateway_timeout_sec,
            query={"topic": topic},
        )
    else:
        payload = _gateway_get_json(
            args.gateway_url,
            "/api/v1/runtime/dataflow",
            timeout_sec=args.gateway_timeout_sec,
        )
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(_format_dataflow_topic(payload) if topic else _format_dataflow_summary(payload))
    if payload.get("ok") is False:
        sys.exit(2)


def _cmd_field_check(args: argparse.Namespace) -> None:
    import json

    from runtime.product_field_check import collect_product_field_check

    if len(args.extra) > 1:
        print(
            "  Usage: lingtu field-check [map-dir] "
            "[--gateway-url URL] [--acceptance-mode non_motion|simulation|field] "
            "[--require-tomogram] [--require-occupancy] [--json] [--json-out PATH]"
        )
        sys.exit(1)

    payload = collect_product_field_check(
        gateway_url=args.gateway_url,
        timeout_sec=args.gateway_timeout_sec,
        mode=args.acceptance_mode or "simulation",
        map_dir=args.extra[0] if args.extra else None,
        require_tomogram=args.require_tomogram,
        require_occupancy=args.require_occupancy,
        expected_data_source=args.expected_data_source,
        expected_source_profile=args.expected_source_profile,
        expected_frame_id=args.expected_frame_id,
    )
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_product_field_check(payload))
    if not payload["ok"]:
        sys.exit(2)


def _parse_inspection_point(raw: str):
    return raw.strip()


def _cmd_inspection_check(args: argparse.Namespace) -> None:
    import json

    from runtime.inspection_acceptance import collect_inspection_acceptance

    if len(args.extra) > 1:
        print(
            "  Usage: lingtu inspection-check [map-dir] "
            "[--point NAME] [--tag TAG] "
            "[--gateway-url URL] [--acceptance-mode non_motion|simulation|field] "
            "[--require-tomogram] [--require-occupancy] [--json] [--json-out PATH]"
        )
        sys.exit(1)

    payload = collect_inspection_acceptance(
        gateway_url=args.gateway_url,
        timeout_sec=args.gateway_timeout_sec,
        mode=args.acceptance_mode or "simulation",
        map_dir=args.extra[0] if args.extra else None,
        points=[_parse_inspection_point(item) for item in args.point],
        tag=args.tag,
        require_tomogram=args.require_tomogram,
        require_occupancy=args.require_occupancy,
        expected_data_source=args.expected_data_source,
        expected_source_profile=args.expected_source_profile,
        expected_frame_id=args.expected_frame_id,
    )
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    elif not args.json_out:
        print(format_inspection_acceptance(payload))
    if not payload["ok"]:
        sys.exit(2)


def _cmd_saved_map_artifact_gate(args: argparse.Namespace) -> None:
    import subprocess

    if len(args.extra) != 1:
        print(
            "  Usage: lingtu saved-map-artifact-gate <map-dir> "
            "[--require-tomogram] [--require-occupancy] [--json-out PATH]"
        )
        sys.exit(1)

    repo_root = Path(__file__).resolve().parent.parent
    cmd = [
        sys.executable,
        str(repo_root / "scripts" / "gates" / "saved_map_artifact_gate.py"),
        args.extra[0],
    ]
    if args.require_tomogram:
        cmd.append("--require-tomogram")
    if args.require_occupancy:
        cmd.append("--require-occupancy")
    if args.expected_data_source:
        cmd.extend(["--expected-data-source", args.expected_data_source])
    if args.expected_source_profile:
        cmd.extend(["--expected-source-profile", args.expected_source_profile])
    if args.expected_frame_id:
        cmd.extend(["--expected-frame-id", args.expected_frame_id])
    if args.json_out:
        cmd.extend(["--json-out", str(args.json_out)])
    if args.json:
        cmd.append("--json")

    proc = subprocess.run(cmd, cwd=repo_root, check=False)
    if proc.returncode:
        sys.exit(proc.returncode)


def _cmd_real_runtime_evidence(args: argparse.Namespace) -> None:
    import json
    import subprocess

    if args.extra:
        print(
            "  Usage: lingtu real-runtime-evidence "
            "[--collector gateway|ros2] [--gateway-url URL] "
            "[--duration-sec SEC] [--json-out PATH] [--json] "
            "[--expected-command-subscriber NAME] [--no-validate]"
        )
        sys.exit(1)

    from runtime.runtime_evidence import REAL_RUNTIME_CONTRACT

    repo_root = Path(__file__).resolve().parent.parent
    report_path = args.json_out or Path("artifacts/thunder_field_runtime/report.json")
    cmd = [
        sys.executable,
        str(repo_root / "scripts" / "gates" / "real_runtime_evidence_collect.py"),
        "--collector",
        args.collector,
        "--gateway-url",
        args.gateway_url,
        "--gateway-timeout-sec",
        str(args.gateway_timeout_sec),
        "--duration-sec",
        str(args.duration_sec),
        "--min-motion-m",
        str(args.min_motion_m),
        "--min-cmd-vel-norm",
        str(args.min_cmd_vel_norm),
        "--expected-contract",
        REAL_RUNTIME_CONTRACT,
        "--json-out",
        str(report_path),
    ]
    for subscriber in args.expected_command_subscriber:
        cmd.extend(["--expected-command-subscriber", subscriber])
    if args.no_validate:
        cmd.append("--no-validate")
    if args.json:
        cmd.append("--json")

    proc = subprocess.run(cmd, cwd=repo_root, check=False)
    if not args.json:
        print(f"  Real runtime evidence report: {report_path}")
        if report_path.exists():
            report = json.loads(report_path.read_text(encoding="utf-8"))
            if isinstance(report, dict):
                print(format_real_runtime_evidence_summary(report))
    if proc.returncode:
        sys.exit(proc.returncode)


def _cmd_portable_mujoco(args: argparse.Namespace) -> None:
    """Run the no-ROS portable MuJoCo planning + sensor acceptance gate."""

    import json
    import subprocess

    if args.extra:
        print(
            "  Usage: lingtu mujoco "
            "[--mujoco-world WORLD] [--nav-duration SEC] "
            "[--nav-goal-distance M] [--min-nav-motion M] "
            "[--json] [--json-out PATH]"
        )
        sys.exit(1)

    repo_root = Path(__file__).resolve().parent.parent
    report_path = args.json_out or Path("artifacts/mujoco_full_system.json")
    if not report_path.is_absolute():
        report_path = repo_root / report_path
    worlds = args.mujoco_worlds or ["open_field"]

    cmd = [
        sys.executable,
        str(repo_root / "sim" / "validation" / "full_system.py"),
        "--run-mujoco",
        "--nav-duration",
        str(args.nav_duration),
        "--nav-goal-distance",
        str(args.nav_goal_distance),
        "--min-nav-motion",
        str(args.min_nav_motion),
        "--json-out",
        str(report_path),
    ]
    for world in worlds:
        cmd.extend(["--mujoco-world", world])
    if args.require_all:
        cmd.append("--require-all")

    proc = subprocess.run(
        cmd,
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode and proc.stdout:
        print(proc.stdout, end="")
    if proc.returncode and proc.stderr:
        print(proc.stderr, end="", file=sys.stderr)

    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            summary = report.get("summary", {})
            print("  MuJoCo planning + sensor gate")
            print(f"  Report: {report_path}")
            print(f"  Result: {'PASS' if report.get('passed') else 'FAIL'} {summary}")
            for check in report.get("checks", []):
                if check.get("name") in {
                    "sim_nav_planning_wiring",
                    "mujoco_sensor_runtime",
                    "mujoco_kinematic_nav_runtime",
                    "mujoco_lidar_open_field",
                }:
                    print(
                        f"  - {check.get('name')}: {check.get('status')} 鈥?"
                        f"{check.get('summary')}"
                    )
    elif not proc.returncode:
        print(f"  {T.yellow('WARN')}: portable MuJoCo gate succeeded but no report was written: {report_path}")

    if proc.returncode:
        sys.exit(proc.returncode)


def _resolve_profile_name(explicit_profile: str | None, args: argparse.Namespace) -> str:
    profile_name = explicit_profile
    if profile_name is None:
        has_custom = any(
            [
                args.dog_host,
                args.detector,
                args.encoder,
                args.llm,
                args.planner,
            ]
        )
        if has_custom:
            profile_name = "stub"
        elif not IS_TTY:
            profile_name = "stub"
        else:
            profile_name = select_interactive()
    return _canonical_profile_name(profile_name)


def _validate_backend_overrides(args: argparse.Namespace) -> None:
    from runtime.backend_status import require_backend
    from runtime.profiles.planner_backends import normalize_planner_name
    from runtime.registry import list_plugins
    from runtime.runtime_policy import normalize_slam_profile

    planner_backend = getattr(args, "planner", None)
    if planner_backend is not None:
        normalized_planner = normalize_planner_name(planner_backend)
        require_backend(
            "planner",
            normalized_planner,
            ("octoplanner3d", "pct", "direct"),
        )
        args.planner = normalized_planner

    slam_profile = getattr(args, "slam_profile", None)
    if slam_profile is not None:
        require_backend(
            "slam_profile",
            normalize_slam_profile(slam_profile),
            (
                "none",
                "fastlio2",
                "localizer",
                "bridge",
                "super_lio",
                "super_lio_relocation",
            ),
        )

    exploration_backend = getattr(args, "exploration_backend", None)
    if exploration_backend is not None:
        require_backend(
            "exploration",
            exploration_backend,
            ("none", "tare", "tare_external"),
        )

    if getattr(args, "local_planner_backend", None) is not None:
        seed_runtime_plugins(groups=("autonomy",), reload_loaded=True)
        require_backend(
            "local_planner",
            args.local_planner_backend,
            list_plugins("local_planner"),
        )

    if getattr(args, "path_follower_backend", None) is not None:
        seed_runtime_plugins(groups=("autonomy",), reload_loaded=True)
        require_backend(
            "path_follower",
            args.path_follower_backend,
            list_plugins("path_follower"),
        )

    if getattr(args, "terrain_backend", None) is not None:
        seed_runtime_plugins(groups=("autonomy",), reload_loaded=True)
        require_backend(
            "terrain",
            args.terrain_backend,
            list_plugins("terrain"),
        )


def _resolve_config(
    profile_name: str,
    args: argparse.Namespace,
    *,
    allow_wizard: bool = True,
) -> dict:
    profile_name = _canonical_profile_name(profile_name)
    if profile_name not in PROFILES:
        print(f"  {T.red('Error')}: Unknown profile '{profile_name}'")
        print(f"  Available: {', '.join(PROFILES.keys())}")
        sys.exit(1)

    try:
        _validate_backend_overrides(args)
    except ValueError as exc:
        print(f"  {T.red('Error')}: {exc}")
        sys.exit(2)

    endpoint_name = getattr(args, "endpoint", None)
    overrides = {
        "dog_host": getattr(args, "dog_host", None),
        "dog_port": getattr(args, "dog_port", None),
        "detector": getattr(args, "detector", None),
        "encoder": getattr(args, "encoder", None),
        "llm": getattr(args, "llm", None),
        "planner": getattr(args, "planner", None),
        "slam_profile": getattr(args, "slam_profile", None),
        "exploration_backend": getattr(args, "exploration_backend", None),
        "local_planner_backend": getattr(args, "local_planner_backend", None),
        "path_follower_backend": getattr(args, "path_follower_backend", None),
        "terrain_backend": getattr(args, "terrain_backend", None),
        "tomogram": getattr(args, "tomogram", None),
        "plan_safety_policy": getattr(args, "plan_safety_policy", None),
        "fallback_planner_name": getattr(args, "fallback_planner_name", None),
        "gateway_port": getattr(args, "gateway_port", None),
        "module_transport": getattr(args, "module_transport", None),
        "nav_plan_transport": getattr(args, "nav_plan_transport", None),
    }
    overrides = {key: value for key, value in overrides.items() if value is not None}
    if getattr(args, "local_planner_backend", None) is not None:
        overrides["python_autonomy_backend"] = args.local_planner_backend
    if getattr(args, "path_follower_backend", None) is not None:
        overrides["python_path_follower_backend"] = args.path_follower_backend
    if getattr(args, "no_semantic", False):
        overrides["enable_semantic"] = False
    if getattr(args, "no_gateway", False):
        overrides["enable_gateway"] = False
    if getattr(args, "native", False):
        overrides["enable_native"] = True
    if getattr(args, "no_native", False):
        overrides["enable_native"] = False
    if getattr(args, "rerun", False):
        overrides["enable_rerun"] = True

    lifecycle_check_overrides = dict(overrides)
    if endpoint_name is not None:
        lifecycle_check_overrides["runtime_endpoint"] = endpoint_name
    blockers = lite_runtime_lifecycle_blockers(profile_name, lifecycle_check_overrides)
    if blockers:
        _exit_lite_lifecycle_error(profile_name, blockers)

    from runtime.profiles.resolver import resolve_profile_config

    try:
        cfg = resolve_profile_config(
            profile_name,
            runtime_endpoint=endpoint_name,
            robot_preset=getattr(args, "robot", None),
            overrides=overrides,
            include_profile_metadata=True,
        )
    except Exception as exc:
        if not _is_runtime_endpoint_error(exc):
            raise
        print(f"  {T.red('Error')}: {exc}")
        sys.exit(2)

    if allow_wizard and args.target is None and IS_TTY:
        wizard_interactive(profile_name, cfg)

    return cfg


def main() -> None:
    install_runtime_plugin_catalog()

    parser = argparse.ArgumentParser(
        description=(
            "LingTu Navigation System. Product inspection uses Gateway + "
            "ModulePorts runtime dataflow; ROS 2 topics are endpoint adapter "
            "details, not the operator entry point."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            product profiles:
              lite      Lightweight local Thunder runtime without ROS/SLAM
              map       Build a new map
              nav       Navigation with pre-built map
              explore   Exploration, no pre-built map
              aliases   thunder-lite, thunder-map, thunder-nav, thunder-explore

            advanced profiles:
              tare_explore  LingTu-native traversable frontier exploration
              use --list --all for simulation, dev, and experimental profiles

            runtime endpoints:
              endpoints are connection/adaptation layers; they do not replace the product task graph
              --endpoint thunder-lite  Run lightweight local Thunder without ROS/SLAM
              --endpoint thunder-field Run a product task against the physical Thunder robot
              --endpoint mujoco_live  Run a product task against MuJoCo raw MID-360 + Fast-LIO
              --endpoint replay       Run a product task against no-actuation replay logs
              --endpoint gazebo       Run a product task against Gazebo/GZ industrial adapter
              --endpoint cmu_unity    Run TARE bridge task against CMU Unity external runtime

            product simulation examples:
              python lingtu.py explore --endpoint mujoco_live
              python lingtu.py nav --endpoint replay status
              python lingtu.py switch-plan explore explore --current-endpoint mujoco_live --endpoint thunder-field
              python lingtu.py tare_explore --endpoint mujoco_live
              python lingtu.py tare_explore --endpoint cmu_unity --record

            lifecycle commands:
              stop         Stop running daemon
              restart      Stop and relaunch with the same argv
              status       Show external run status (add --json for jq)
              show-config  Print resolved config (add --json for jq)
              runtime-contract Print canonical frames/topics/data-flow manifest
              runtime-spec Print resolved runtime data-flow/frame contract for one profile
              switch-plan  Print sim/replay/real runtime-boundary diff
              runtime-audit Check runtime manifest/YAML/profile/collector contracts
              dataflow    Inspect live Gateway/ModulePort flow and whitelist commands
              gateway-runtime-acceptance Check product runtime state through Gateway only
              field-check  One-screen field readiness from Gateway/evidence/map gates
              inspection-check Non-motion inspection-point acceptance pack
              saved-map-artifact-gate Validate saved map artifact provenance
              real-runtime-evidence Collect read-only Thunder field runtime evidence
              mujoco       Run MuJoCo planning + sensor gate on this computer
              log          Print the current run log (add -f to follow)
              health       Sensor and module health (add --json for jq)
              doctor       Run diagnostics
              rerun        Launch Rerun 3D viewer
        """
        ),
    )
    parser.add_argument("target", nargs="?", default=None, help="Profile name or command")
    parser.add_argument("extra", nargs="*", help=argparse.SUPPRESS)
    parser.add_argument("--list", action="store_true", help="List product profiles and exit")
    parser.add_argument("--all", action="store_true", help="Show advanced, simulation, and dev profiles with --list")
    parser.add_argument("--version", action="store_true", help="Print LingTu version and exit")
    parser.add_argument("--json", action="store_true",
                        help="Machine-readable JSON output (status / show-config / switch-plan / runtime-spec / runtime-contract / runtime-audit / dataflow / gateway-runtime-acceptance / field-check / inspection-check / real-runtime-evidence / mujoco)")
    parser.add_argument("--json-out", type=Path, default=None,
                        help="Write JSON output for commands such as switch-plan, runtime-spec, runtime-contract, runtime-audit, dataflow, gateway-runtime-acceptance, field-check, inspection-check, real-runtime-evidence, or mujoco")
    parser.add_argument("--daemon", "-d", action="store_true", help="Run as background daemon (Unix)")
    parser.add_argument("--follow", "-f", action="store_true", help="Follow output for `lingtu log`")
    parser.add_argument("--lines", type=int, default=80, help="Number of lines for `lingtu log` (default: 80)")
    parser.add_argument("--force", action="store_true", help="Force action for commands like `lingtu stop`")
    parser.add_argument(
        "--endpoint",
        default=None,
        metavar="ENDPOINT",
        help="Runtime endpoint/connection layer for map/nav/explore/tare_explore",
    )
    parser.add_argument(
        "--current-endpoint",
        default=None,
        metavar="ENDPOINT",
        help="Current runtime endpoint for switch-plan dry-run comparisons",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Use the endpoint's visible recording/demo action when available",
    )
    parser.add_argument("--robot", default=None)
    parser.add_argument("--dog-host", default=None, dest="dog_host")
    parser.add_argument("--dog-port", type=int, default=None, dest="dog_port")
    parser.add_argument("--detector", default=None)
    parser.add_argument("--encoder", default=None)
    parser.add_argument("--llm", default=None)
    parser.add_argument("--planner", default=None)
    parser.add_argument("--slam-profile", dest="slam_profile", default=None)
    parser.add_argument("--exploration-backend", default=None)
    parser.add_argument("--local-planner-backend", default=None)
    parser.add_argument("--path-follower-backend", default=None)
    parser.add_argument("--terrain-backend", default=None)
    parser.add_argument("--tomogram", default=None)
    parser.add_argument(
        "--plan-safety-policy",
        default=None,
        choices=["off", "observe", "reject"],
        dest="plan_safety_policy",
        help="Global plan safety handling: observe, reject, or disable safety checks",
    )
    parser.add_argument(
        "--fallback-planner",
        default=None,
        dest="fallback_planner_name",
        help="Legacy explicit fallback backend for non-product planner experiments",
    )
    parser.add_argument("--gateway-port", type=int, default=None, dest="gateway_port")
    parser.add_argument(
        "--module-transport",
        choices=_module_transport_names(),
        default=None,
        dest="module_transport",
        help="ModulePort transport strategy for this process (default: endpoint/local)",
    )
    parser.add_argument(
        "--nav-plan-transport",
        choices=_module_transport_names(),
        default=None,
        dest="nav_plan_transport",
        help="Transport only for Navigation <-> LocalPlanner execution wires",
    )
    parser.add_argument("--no-semantic", action="store_true")
    parser.add_argument("--no-gateway", action="store_true")
    parser.add_argument("--native", action="store_true", help="Force C++ autonomy stack (terrain+local_planner+pathFollower)")
    parser.add_argument("--no-native", action="store_true")
    parser.add_argument("--rerun", action="store_true", help="Enable Rerun 3D visualization on startup")
    parser.add_argument("--no-repl", action="store_true", help="Foreground daemon (no interactive REPL)")
    parser.add_argument("--log-level", default="INFO", dest="log_level")
    parser.add_argument("--log-format", default="text", choices=["text", "json"],
                        dest="log_format", help="Log file format: text (default) or json")
    parser.add_argument("--duration-sec", type=float, default=20.0,
                        help="Duration for `lingtu real-runtime-evidence`")
    parser.add_argument("--collector", choices=["gateway", "ros2"], default="gateway",
                        help="Read-only evidence source for `lingtu real-runtime-evidence`")
    parser.add_argument("--mujoco-world", action="append", default=[], dest="mujoco_worlds",
                        help="MuJoCo world for `lingtu mujoco`. Repeat for multiple worlds; default: open_field")
    parser.add_argument("--nav-duration", type=float, default=8.0,
                        help="Navigation duration in seconds for `lingtu mujoco`")
    parser.add_argument("--nav-goal-distance", type=float, default=1.0,
                        help="Goal distance in meters for `lingtu mujoco`")
    parser.add_argument("--min-nav-motion", type=float, default=0.15,
                        help="Minimum MuJoCo motion in meters for `lingtu mujoco`")
    parser.add_argument("--require-all", action="store_true",
                        help="For `lingtu mujoco`, fail on any blocked optional check")
    parser.add_argument("--gateway-url", default="http://127.0.0.1:5050",
                        help="Gateway base URL for dataflow, acceptance, field-check, inspection-check, or real-runtime-evidence")
    parser.add_argument("--gateway-timeout-sec", type=float, default=2.0,
                        help="Per-request timeout for Gateway dataflow/acceptance/field-check/inspection-check")
    parser.add_argument("--topic", default=None,
                        help="Runtime dataflow topic or short alias for `lingtu dataflow`")
    parser.add_argument("--acceptance-mode", default=None,
                        choices=["non_motion", "simulation", "field"],
                        help="Gateway acceptance strictness: non_motion checks product observability; simulation requires a live simulation endpoint; field requires Thunder field evidence. Defaults: gateway-runtime-acceptance=non_motion, field-check=simulation, inspection-check=simulation")
    parser.add_argument("--min-motion-m", type=float, default=0.05,
                        help="Minimum odometry motion for `lingtu real-runtime-evidence`")
    parser.add_argument("--min-cmd-vel-norm", type=float, default=0.01,
                        help="Minimum cmd_vel norm for `lingtu real-runtime-evidence`")
    parser.add_argument("--expected-command-subscriber", action="append", default=[],
                        help="Accepted hardware cmd_vel subscriber for `lingtu real-runtime-evidence`")
    parser.add_argument("--require-tomogram", action="store_true",
                        help="Require tomogram.pickle for `lingtu saved-map-artifact-gate`")
    parser.add_argument("--require-occupancy", action="store_true",
                        help="Require occupancy.npz for `lingtu saved-map-artifact-gate`")
    parser.add_argument("--expected-data-source", default=None,
                        help="Expected data_source for `lingtu saved-map-artifact-gate`")
    parser.add_argument("--expected-source-profile", default=None,
                        help="Expected source_profile for `lingtu saved-map-artifact-gate`")
    parser.add_argument("--expected-frame-id", default=None,
                        help="Expected frame_id for `lingtu saved-map-artifact-gate`")
    parser.add_argument("--point", action="append", default=[],
                        help="Inspection target for `lingtu inspection-check`: saved location name. Repeat for multiple points")
    parser.add_argument("--tag", default=None,
                        help="Use saved locations with this tag for `lingtu inspection-check`")
    parser.add_argument("--no-validate", action="store_true",
                        help="Collect real runtime evidence without running the validation gate")
    args, trailing_extra = parser.parse_known_args()
    if trailing_extra:
        args.extra.extend(trailing_extra)

    if args.version:
        print(f"lingtu {_lingtu_version()}")
        return

    if args.list:
        list_profiles(show_all=args.all)
        return

    if args.target == "stop":
        cmd_stop(force=args.force)
        return

    if args.target == "restart":
        cmd_restart()
        return

    if args.target == "status":
        cmd_status_external(as_json=args.json)
        return

    if args.target == "health":
        cmd_health_external(as_json=args.json)
        return

    _repo = Path(__file__).resolve().parent.parent

    if args.target == "doctor":
        import subprocess as _sp

        _sp.run([
            sys.executable,
            str(_repo / "scripts" / "diagnostics" / "doctor.py"),
            "--gateway-url",
            args.gateway_url,
            "--gateway-timeout-sec",
            str(args.gateway_timeout_sec),
            *args.extra,
        ])
        return

    if args.target == "rerun":
        import subprocess as _sp

        use_ros2_rerun = "--ros2" in args.extra
        rerun_extra = [item for item in args.extra if item != "--ros2"]
        script = (
            _repo / "scripts" / "visualization" / "rerun_live.py"
            if use_ros2_rerun
            else _repo / "scripts" / "visualization" / "rerun_gateway_live.py"
        )
        cmd = [sys.executable, str(script)]
        if args.native:
            rerun_extra.append("--native")
        if not use_ros2_rerun:
            cmd.extend([
                "--gateway-url",
                args.gateway_url,
                "--gateway-timeout-sec",
                str(args.gateway_timeout_sec),
            ])
        cmd.extend(rerun_extra)
        _sp.run(cmd)
        return

    if args.target == "log":
        cmd_log_external(follow=args.follow, lines=args.lines)
        return

    if args.target == "switch-plan":
        _cmd_switch_plan(args)
        return

    if args.target == "runtime-contract":
        _cmd_runtime_contract(args)
        return

    if args.target == "runtime-spec":
        _cmd_runtime_spec(args)
        return

    if args.target == "runtime-audit":
        _cmd_runtime_audit(args)
        return

    if args.target == "dataflow":
        _cmd_dataflow(args)
        return

    if args.target == "gateway-runtime-acceptance":
        _cmd_gateway_runtime_acceptance(args)
        return

    if args.target == "field-check":
        _cmd_field_check(args)
        return

    if args.target == "inspection-check":
        _cmd_inspection_check(args)
        return

    if args.target == "saved-map-artifact-gate":
        _cmd_saved_map_artifact_gate(args)
        return

    if args.target == "real-runtime-evidence":
        _cmd_real_runtime_evidence(args)
        return

    if args.target in {"mujoco", "portable-mujoco", "mujoco-portable"}:
        _cmd_portable_mujoco(args)
        return

    if args.target == "show-config":
        profile_name = args.extra[0] if args.extra else None
        if len(args.extra) > 1:
            print("  Usage: lingtu show-config [profile] [overrides]")
            sys.exit(1)
        profile_name = _resolve_profile_name(profile_name, args)
        cfg = _resolve_config(profile_name, args, allow_wizard=False)
        cmd_show_config_external(profile_name, cfg, as_json=args.json)
        return

    profile_name = _resolve_profile_name(args.target, args)
    endpoint_external = False
    if args.endpoint:
        endpoint_external = bool(_runtime_endpoint(args.endpoint).external_launcher)
    external_profile = bool(
        PROFILES.get(profile_name, {}).get("_external_launcher") or endpoint_external
    )

    if args.target not in _SPECIAL_COMMANDS and args.extra and not external_profile:
        print(f"  {T.red('Error')}: Unexpected extra positional arguments: {' '.join(args.extra)}")
        sys.exit(1)

    cfg = _resolve_config(profile_name, args, allow_wizard=True)

    if cfg.get("_external_launcher"):
        _run_external_profile_launcher(profile_name, cfg, args, _repo)
        return

    runtime_spec = _apply_runtime_process_env(profile_name, cfg, args)

    if args.daemon:
        args.no_repl = True
    if not IS_TTY:
        args.no_repl = True

    log_dir = setup_logging(args.log_level, profile_name, args.log_format)

    if args.daemon:
        log_file = str(Path(log_dir) / "lingtu.log")
        daemonize(log_file)

    desc = cfg.pop("_desc", "custom")
    blueprint_cfg = dict(cfg)
    cfg["_desc"] = desc

    _preflight(profile_name, cfg)

    print(f"  Runtime:  {format_runtime_boundary(runtime_spec)}")
    print(f"  SLAM:     {format_runtime_sources(runtime_spec)}")
    print(f"  Frame ids: {format_runtime_frames(runtime_spec)}")
    print(f"  Frames:   {format_frame_links(runtime_spec)}")
    print(f"  Topic frames: {format_runtime_topic_frames(runtime_spec)}")
    print(f"  Flow:     {format_runtime_flow(runtime_spec)}")
    print(f"  Flow stages: {format_runtime_flow_stages(runtime_spec)}")
    print(f"\n  Building system ({T.green(profile_name)})...")

    from runtime.blueprints.profile_builder import build_system_from_resolved_profile

    try:
        system = build_system_from_resolved_profile(profile_name, blueprint_cfg)
    except Exception as e:
        logger.error("Build failed: %s", e, exc_info=True)
        print(f"\n  {T.red('Build failed')}: {e}")
        sys.exit(1)

    if not health_check(system):
        print(f"  {T.red('Health check failed')} 鈥?some modules did not build correctly")
        sys.exit(1)
    logger.info("Health check passed: %d modules OK", len(system.modules))

    kill_residual_ports(blueprint_cfg)

    # In --no-repl mode, defer uvicorn startup so main thread can run it
    # (avoids GIL starvation from DDS callbacks in daemon threads).
    exit_code = 0
    if args.no_repl:
        try:
            gw = system.get_module("GatewayModule")
            gw._defer_server = True
        except (KeyError, AttributeError):
            pass

    try:
        system.start()
    except Exception as e:
        logger.error("Start failed: %s", e, exc_info=True)
        print(f"\n  {T.red('Start failed')}: {e}")
        sys.exit(1)

    # Inject SystemHandle so MCPServerModule can serve get_health / list_modules
    for _mod_name in ("MCPServerModule",):
        try:
            system.get_module(_mod_name).set_system_handle(system)
        except (KeyError, AttributeError):
            pass

    if cfg.get("enable_rerun"):
        rerun_mod = None
        try:
            rerun_mod = system.get_module("RerunBridgeModule")
        except KeyError:
            pass
        if rerun_mod:
            url = rerun_mod.start_rerun()
            logger.info("Rerun auto-started: %s", url)

    try:
        _mod_count = len(system.modules)
    except Exception:
        _mod_count = None
    try:
        _wire_count = len(getattr(system, "_connections", []) or [])
    except Exception:
        _wire_count = None
    from runtime.runtime_switch import runtime_spec_summary

    save_run_state(
        profile_name,
        cfg,
        log_dir,
        log_format=args.log_format,
        argv=sys.argv[1:],
        cwd=str(Path.cwd()),
        daemon=args.daemon,
        status="running",
        module_count=_mod_count,
        wire_count=_wire_count,
        runtime=runtime_spec_summary(runtime_spec),
    )

    print_banner(profile_name, cfg, system, log_dir)

    shutdown = threading.Event()

    def _on_signal(signum, frame):
        shutdown.set()

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)

    if args.no_repl:
        # Run uvicorn in the main thread so it gets reliable event-loop
        # scheduling.  Daemon threads (DDS/LiDAR callbacks) can starve a
        # threaded uvicorn of GIL time, making all HTTP endpoints hang.
        gw = None
        try:
            gw = system.get_module("GatewayModule")
        except (KeyError, AttributeError):
            pass
        if gw is not None and hasattr(gw, "_run_server"):
            if hasattr(gw, "_start_client_http_prewarm"):
                gw._start_client_http_prewarm(timeout_s=30.0)
            server_stopped_cleanly = bool(gw._run_server())  # blocks until shutdown
            if not server_stopped_cleanly and not shutdown.is_set():
                logger.error("Gateway server exited unexpectedly; returning failure for systemd")
                exit_code = 1
        else:
            shutdown.wait()
    else:
        signal.signal(signal.SIGINT, signal.default_int_handler)
        from .repl import LingTuREPL

        repl = LingTuREPL(system, cfg)
        try:
            repl.cmdloop()
        except KeyboardInterrupt:
            print()

    print("  Stopping modules...")
    try:
        update_run_state(status="stopping")
    except Exception:
        pass
    system.stop()
    try:
        from lingtu.ros2_shutdown import shutdown_ros2_runtime

        shutdown_ros2_runtime()
    except Exception:
        pass
    try:
        from runtime.service_manager import get_service_manager

        svc = get_service_manager()
        if svc._started:
            print("  Releasing services: %s" % ", ".join(svc._started))
            svc.stop_all_started()
    except Exception:
        pass
    clear_run_state()
    print(f"  {T.green('Done.')}\n")
    if exit_code:
        sys.exit(exit_code)
