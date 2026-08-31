#!/usr/bin/env python3
"""Attach-only native DDS recording acceptance for an already-running MuJoCo Product."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence, cast

PROBE_NAME = "mujoco_native_recording_acceptance"
REQUIRED_TOPICS = ("/imu/raw", "/lidar/raw_frame")
MAX_CAPTURE_SECONDS = 30.0
DEFAULT_CAPTURE_SECONDS = 5.0
DEFAULT_STOP_GRACE_MS = 5000
MINIMUM_FREE_BYTES = 5 * 1024**3
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RECORDER = ROOT / "build" / "native-recording" / "lingtu_recorder"
DEFAULT_VERIFIER = ROOT / "build" / "native-recording" / "lingtu_dds_player"


def _dds_domain(value: str) -> int:
    domain = int(value)
    if not 0 <= domain <= 232:
        raise argparse.ArgumentTypeError("DDS domain must be in range 0..232")
    return domain


def _capture_seconds(value: str) -> float:
    seconds = float(value)
    if not 0.0 < seconds <= MAX_CAPTURE_SECONDS:
        raise argparse.ArgumentTypeError(f"capture seconds must be greater than 0 and at most {MAX_CAPTURE_SECONDS:g}")
    return seconds


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture and verify native DDS sensors without launching or stopping the "
            "already-running simulation Product."
        ),
        epilog=(
            "Preconditions:\n"
            "  * A sim Product and its native DDS sensor publishers are already running.\n"
            "  * --dds-domain is that Product's allocated live domain (0..232).\n"
            "  * /imu/raw and /lidar/raw_frame remain live for the capture interval.\n"
            "  * The output directory does not exist; its parent has at least 5 GiB free.\n"
            "  * Run mode requires Linux lingtu_recorder and lingtu_dds_player binaries.\n"
            "Dry-run prints the complete attach-only plan and needs no native binaries or DDS."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--output-dir", required=True, help="New recording session directory")
    parser.add_argument("--dds-domain", required=True, type=_dds_domain)
    parser.add_argument("--product", required=True, help="Already-running Product name")
    parser.add_argument(
        "--product-session-id",
        required=True,
        help="Session id of the already-running Product",
    )
    parser.add_argument(
        "--seconds",
        type=_capture_seconds,
        default=DEFAULT_CAPTURE_SECONDS,
        help=f"Capture duration, >0 and <= {MAX_CAPTURE_SECONDS:g} (default: %(default)s)",
    )
    parser.add_argument("--recorder", default=str(DEFAULT_RECORDER))
    parser.add_argument("--verifier", default=str(DEFAULT_VERIFIER))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands and preconditions without checking binaries or DDS",
    )
    mode.add_argument(
        "--preflight-only",
        action="store_true",
        help="Check local prerequisites without starting the recorder or joining DDS",
    )
    return parser


def _commands(args: argparse.Namespace, session_dir: Path) -> dict[str, list[str]]:
    mcap_path = session_dir / "dds" / "sensors.mcap"
    return {
        "record": [
            str(args.recorder),
            "record",
            "--output-dir",
            str(session_dir),
            "--seconds",
            f"{args.seconds:g}",
            "--stop-grace-ms",
            str(DEFAULT_STOP_GRACE_MS),
            "--product",
            args.product,
            "--product-session-id",
            args.product_session_id,
            "--dds",
            "on",
            "--camera",
            "off",
            "--dds-domain",
            str(args.dds_domain),
            "--dds-preset",
            "generic-sensors-v1",
        ],
        "catalog": [str(args.recorder), "list", "--root", str(session_dir.parent)],
        "verify": [str(args.verifier), str(mcap_path), "--dry-run"],
        "inspect_topics": [str(args.verifier), "--info", str(mcap_path)],
    }


def _base_report(args: argparse.Namespace, session_dir: Path) -> dict[str, object]:
    mcap_path = session_dir / "dds" / "sensors.mcap"
    return {
        "schema_version": 1,
        "probe": PROBE_NAME,
        "ok": bool(args.dry_run),
        "mode": ("dry_run" if args.dry_run else "preflight" if args.preflight_only else "run"),
        "attach_only": True,
        "preconditions": {
            "product_runtime": "already_running",
            "product": args.product,
            "product_session_id": args.product_session_id,
            "dds_domain": args.dds_domain,
            "required_publishers": list(REQUIRED_TOPICS),
            "output_directory": "must_not_exist",
            "minimum_parent_free_gib": 5,
            "catalog_root": str(session_dir.parent),
            "linux_host_required": not args.dry_run,
            "native_binaries_required": not args.dry_run,
            "owned_lifecycle": "lingtu_recorder_only",
        },
        "capture": {
            "seconds": args.seconds,
            "required_topics": list(REQUIRED_TOPICS),
            "session_dir": str(session_dir),
            "declared_mcap": str(mcap_path),
        },
        "commands": _commands(args, session_dir),
        "checks": {"native_execution": "skipped" if args.dry_run else "pending"},
        "blockers": [],
    }


def _executable_check(command: str) -> dict[str, object]:
    resolved = shutil.which(command)
    if resolved is None:
        candidate = Path(command).expanduser()
        if candidate.is_file():
            resolved = str(candidate.resolve())
    if resolved is None:
        return {"ok": False, "path": command, "reason": "not_found"}
    if not os.access(resolved, os.X_OK):
        return {"ok": False, "path": str(resolved), "reason": "not_executable"}
    return {"ok": True, "path": str(Path(resolved).resolve())}


def _nearest_existing_directory(path: Path) -> Path:
    candidate = path.resolve()
    while not candidate.exists() and candidate.parent != candidate:
        candidate = candidate.parent
    return candidate


def _storage_check(session_dir: Path) -> dict[str, object]:
    storage_path = _nearest_existing_directory(session_dir.parent)
    try:
        usage = shutil.disk_usage(storage_path)
    except OSError as exc:
        return {
            "ok": False,
            "path": str(storage_path),
            "minimum_free_bytes": MINIMUM_FREE_BYTES,
            "available_bytes": None,
            "error": f"{type(exc).__name__}: {exc}",
        }
    return {
        "ok": usage.free >= MINIMUM_FREE_BYTES,
        "path": str(storage_path),
        "minimum_free_bytes": MINIMUM_FREE_BYTES,
        "available_bytes": usage.free,
    }


def _apply_preflight(
    report: dict[str, object],
    args: argparse.Namespace,
    session_dir: Path,
) -> None:
    recorder = _executable_check(str(args.recorder))
    host_platform = {
        "ok": sys.platform.startswith("linux"),
        "actual": sys.platform,
        "required": "linux",
    }
    verifier = _executable_check(str(args.verifier))
    output_absent = {
        "ok": not session_dir.exists(),
        "path": str(session_dir),
        "reason": None if not session_dir.exists() else "already_exists",
    }
    storage = _storage_check(session_dir)
    report["checks"] = {
        "host_platform": host_platform,
        "recorder": recorder,
        "verifier": verifier,
        "output_dir_absent": output_absent,
        "storage": storage,
        "native_execution": "not_started",
    }
    blockers: list[str] = []
    if not host_platform["ok"]:
        blockers.append("unsupported_platform")
    for name, check in (("recorder", recorder), ("verifier", verifier)):
        if not check["ok"]:
            blockers.append(f"{name}_{check['reason']}")
    if not output_absent["ok"]:
        blockers.append("output_dir_already_exists")
    if not storage["ok"]:
        blockers.append("storage_unavailable" if "error" in storage else "insufficient_free_space")
    report["blockers"] = blockers
    report["ok"] = not blockers


def _add_blocker(report: dict[str, object], blocker: str) -> None:
    blockers = cast(list[str], report["blockers"])
    if blocker not in blockers:
        blockers.append(blocker)


def _invoke(command: list[str], *, timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(  # noqa: S603 - explicit argv without a shell
        command,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout_s,
    )


def _process_evidence(result) -> dict[str, object]:
    return {
        "ok": result.returncode == 0,
        "returncode": result.returncode,
        "stdout_tail": (result.stdout or "")[-4096:],
        "stderr_tail": (result.stderr or "")[-4096:],
    }


def _load_completed_mcap(
    report: dict[str, object],
    session_dir: Path,
) -> Path | None:
    checks = cast(dict[str, object], report["checks"])
    manifest_path = session_dir / "session.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        checks["terminal_session"] = {
            "ok": False,
            "manifest": str(manifest_path),
            "error": f"{type(exc).__name__}: {exc}",
        }
        _add_blocker(report, "session_manifest_invalid")
        return None

    terminal_ok = (
        isinstance(manifest, dict)
        and manifest.get("state") == "completed"
        and manifest.get("ended_at_unix_ns") is not None
    )
    checks["terminal_session"] = {
        "ok": terminal_ok,
        "manifest": str(manifest_path),
        "state": manifest.get("state") if isinstance(manifest, dict) else None,
        "ended_at_unix_ns": (manifest.get("ended_at_unix_ns") if isinstance(manifest, dict) else None),
    }
    if not terminal_ok:
        _add_blocker(report, "session_not_completed")
        return None

    children = manifest.get("children")
    dds_children = (
        [child for child in children if isinstance(child, dict) and child.get("name") == "dds"]
        if isinstance(children, list)
        else []
    )
    dds_child = dds_children[0] if len(dds_children) == 1 else None
    child_ok = bool(
        dds_child
        and dds_child.get("required") is True
        and dds_child.get("state") == "exited"
        and dds_child.get("exit_code") == 0
    )
    checks["dds_child"] = {
        "ok": child_ok,
        "count": len(dds_children),
        "state": dds_child.get("state") if dds_child else None,
        "exit_code": dds_child.get("exit_code") if dds_child else None,
    }
    if not child_ok:
        _add_blocker(report, "dds_child_not_clean")
        return None

    declared_required_value = dds_child.get("required_topics")
    declarations_valid = isinstance(declared_required_value, list) and all(
        isinstance(topic, str) for topic in declared_required_value
    )
    declared_required = (
        declared_required_value if isinstance(declared_required_value, list) else []
    )
    missing_declarations = (
        sorted(set(REQUIRED_TOPICS) - set(declared_required))
        if declarations_valid
        else list(REQUIRED_TOPICS)
    )
    checks["manifest_required_topics"] = {
        "ok": declarations_valid and not missing_declarations,
        "declared": declared_required,
        "missing": missing_declarations,
    }
    if missing_declarations:
        _add_blocker(report, "manifest_required_topics_missing")
        return None

    artifacts = dds_child.get("artifacts")
    artifacts = artifacts if isinstance(artifacts, list) else []
    relative_mcap = "dds/sensors.mcap"
    if relative_mcap not in artifacts:
        checks["declared_mcap"] = {
            "ok": False,
            "declared": artifacts,
            "reason": "canonical_artifact_not_declared",
        }
        _add_blocker(report, "declared_mcap_missing")
        return None

    mcap_path = (session_dir / relative_mcap).resolve()
    try:
        mcap_path.relative_to(session_dir.resolve())
    except ValueError:
        checks["declared_mcap"] = {
            "ok": False,
            "path": str(mcap_path),
            "reason": "outside_session",
        }
        _add_blocker(report, "declared_mcap_unsafe")
        return None
    is_regular = mcap_path.is_file() and not mcap_path.is_symlink()
    size = mcap_path.stat().st_size if is_regular else 0
    checks["declared_mcap"] = {
        "ok": is_regular and size > 0,
        "path": str(mcap_path),
        "relative_path": relative_mcap,
        "bytes": size,
    }
    if not is_regular:
        _add_blocker(report, "declared_mcap_missing")
        return None
    if size == 0:
        _add_blocker(report, "declared_mcap_empty")
        return None
    return mcap_path


def _validated_messages(stdout: str) -> int:
    for line in stdout.splitlines():
        fields = dict(field.split("=", 1) for field in line.split() if "=" in field)
        if "validated" in fields:
            try:
                return int(fields["validated"])
            except ValueError:
                return 0
    return 0


def _topic_counts(stdout: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for line in stdout.splitlines():
        fields = dict(field.split("=", 1) for field in line.split() if "=" in field)
        if "topic" not in fields or "count" not in fields:
            continue
        try:
            counts[fields["topic"]] = int(fields["count"])
        except ValueError:
            continue
    return counts


def _catalog_evidence(result, session_dir: Path) -> dict[str, object]:
    session_id = session_dir.name
    evidence = _process_evidence(result)
    try:
        payload = json.loads(result.stdout or "")
    except json.JSONDecodeError as exc:
        return {
            **evidence,
            "ok": False,
            "session_id": session_id,
            "state": None,
            "error": f"{type(exc).__name__}: {exc}",
        }
    sessions = payload.get("sessions") if isinstance(payload, dict) else None
    matches = (
        [session for session in sessions if isinstance(session, dict) and session.get("session_id") == session_id]
        if isinstance(sessions, list)
        else []
    )
    session = matches[0] if len(matches) == 1 else None
    listed_directory = session.get("session_directory") if session else None
    directory_matches = bool(
        isinstance(listed_directory, str) and Path(listed_directory).resolve() == session_dir.resolve()
    )
    ok = bool(
        result.returncode == 0
        and isinstance(payload, dict)
        and payload.get("control_version") == 1
        and payload.get("ok") is True
        and session
        and session.get("state") == "completed"
        and directory_matches
    )
    return {
        **evidence,
        "ok": ok,
        "control_version": payload.get("control_version") if isinstance(payload, dict) else None,
        "session_id": session_id,
        "state": session.get("state") if session else None,
        "session_directory": listed_directory,
        "directory_matches": directory_matches,
        "matches": len(matches),
    }


def _execute(
    report: dict[str, object],
    args: argparse.Namespace,
    session_dir: Path,
) -> None:
    checks = cast(dict[str, object], report["checks"])
    commands = cast(dict[str, list[str]], report["commands"])
    record_timeout_s = args.seconds + DEFAULT_STOP_GRACE_MS / 1000.0 + 10.0
    try:
        recorder_result = _invoke(commands["record"], timeout_s=record_timeout_s)
    except subprocess.TimeoutExpired as exc:
        checks["recorder_process"] = {
            "ok": False,
            "timeout_s": record_timeout_s,
            "error": f"{type(exc).__name__}: {exc}",
        }
        _add_blocker(report, "recorder_timeout")
        report["ok"] = False
        return
    except OSError as exc:
        checks["recorder_process"] = {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        _add_blocker(report, "recorder_launch_failed")
        report["ok"] = False
        return

    checks["recorder_process"] = _process_evidence(recorder_result)
    checks["native_execution"] = "completed"
    if recorder_result.returncode != 0:
        _add_blocker(report, "recorder_failed")
        report["ok"] = False
        return

    mcap_path = _load_completed_mcap(report, session_dir)

    try:
        catalog_result = _invoke(commands["catalog"], timeout_s=10.0)
    except (OSError, subprocess.TimeoutExpired) as exc:
        checks["catalog_session"] = {
            "ok": False,
            "session_id": session_dir.name,
            "state": None,
            "error": f"{type(exc).__name__}: {exc}",
        }
        _add_blocker(report, "catalog_query_failed")
        report["ok"] = False
        return
    catalog_evidence = _catalog_evidence(catalog_result, session_dir)
    checks["catalog_session"] = catalog_evidence
    if not catalog_evidence["ok"]:
        _add_blocker(report, "catalog_session_not_completed")
        report["ok"] = False
        return
    if mcap_path is None:
        report["ok"] = False
        return

    try:
        verifier_result = _invoke(commands["verify"], timeout_s=30.0)
    except (OSError, subprocess.TimeoutExpired) as exc:
        checks["verifier"] = {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        _add_blocker(report, "verifier_failed")
        report["ok"] = False
        return
    validated = _validated_messages(verifier_result.stdout or "")
    checks["verifier"] = {
        **_process_evidence(verifier_result),
        "validated_messages": validated,
    }
    if verifier_result.returncode != 0 or validated <= 0:
        checks["verifier"]["ok"] = False
        _add_blocker(report, "verifier_failed")
        report["ok"] = False
        return

    try:
        info_result = _invoke(commands["inspect_topics"], timeout_s=30.0)
    except (OSError, subprocess.TimeoutExpired) as exc:
        checks["required_topics"] = {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "counts": {},
        }
        _add_blocker(report, "required_topics_unverified")
        report["ok"] = False
        return
    topic_counts = _topic_counts(info_result.stdout or "")
    required_counts = {topic: topic_counts.get(topic, 0) for topic in REQUIRED_TOPICS}
    topics_ok = info_result.returncode == 0 and all(count > 0 for count in required_counts.values())
    checks["required_topics"] = {
        **_process_evidence(info_result),
        "ok": topics_ok,
        "counts": required_counts,
    }
    if not topics_ok:
        _add_blocker(report, "required_topics_missing")
    report["ok"] = not report["blockers"]


def main(argv: Sequence[str] | None = None) -> int:
    """Run the attach-only recording acceptance probe."""
    args = _parser().parse_args(argv)
    session_dir = Path(args.output_dir).expanduser().resolve()
    report = _base_report(args, session_dir)
    if not args.dry_run:
        _apply_preflight(report, args, session_dir)
        if not args.preflight_only and report["ok"]:
            _execute(report, args, session_dir)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
