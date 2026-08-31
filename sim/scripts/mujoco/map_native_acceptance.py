"""Attach-only acceptance for an already-running MuJoCo map Product."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
for path in (ROOT, SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from sim.scripts.mujoco.product_acceptance import (  # noqa: E402
    AcceptanceTarget,
    classify_evidence,
    validate_product_contract,
)

from drivers.real.camera.shm import ShmFrameReader, StreamKind  # noqa: E402
from lingtu.product_lock import CURRENT_RUN_FILE_NAME  # noqa: E402
from lingtu.sdk.client import LingTuClient  # noqa: E402
from lingtu.sim.acceptance import validate_runner_plan  # noqa: E402
from lingtu.sim.identity import (  # noqa: E402
    ProcessIdentity,
    ProcessIdentityError,
    SimChildLedger,
    SimChildLedgerError,
    SimChildRecord,
)
from lingtu.sim.readiness import load_typed_readiness, readiness_expectation_for_process  # noqa: E402
from lingtu.sim.switch import _load_committed_plan  # noqa: E402

DEFAULT_MANIFEST = ROOT / "config/runtime_graph/acceptance/mujoco_map_native_acceptance.json"
REPORT_SCHEMA = "lingtu.mujoco.map_native_acceptance.report.v1"
_CAMERA_STALE_TIMEOUT_S = 1.0
_ATTACHED_PROCESSES = (
    "mujoco_feeder",
    "lidar_publisher",
    "imu_publisher",
    "camera_publisher",
    "slam_runtime",
    "map_runtime",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser


def _load(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid map acceptance JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"map acceptance JSON must be an object: {path}")
    return payload


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _attached_context(plan: Any, run_plan_path: Path) -> tuple[Path, str, bytes]:
    exact = run_plan_path.expanduser().resolve()
    state_root = exact.parent
    committed = _load_committed_plan(state_root, os.environ)
    if committed is None:
        raise ValueError("strict map acceptance requires a committed current RunPlan")
    if committed.path != exact:
        raise ValueError("strict map acceptance RunPlan is not the committed current RunPlan")
    if (committed.plan, committed.plan.product, committed.plan.env) != (plan, "map", "sim"):
        raise ValueError("strict map acceptance current identity does not match the RunPlan")
    try:
        current = (state_root / CURRENT_RUN_FILE_NAME).read_bytes()
    except OSError as exc:
        raise ValueError("strict map acceptance current identity is unavailable") from exc
    return state_root, committed.product_session_id, current


def _revalidate_children(children: Mapping[str, SimChildRecord]) -> dict[str, Mapping[str, Any]]:
    evidence: dict[str, Mapping[str, Any]] = {}
    for name, record in children.items():
        try:
            current = ProcessIdentity.current(record.process_identity.pid)
        except ProcessIdentityError as exc:
            raise RuntimeError(f"strict map acceptance {name} child is not live") from exc
        if current != record.process_identity:
            raise RuntimeError(f"strict map acceptance {name} child identity changed")
        evidence[name] = {
            "process_identity": record.process_identity.as_dict(),
            "started_wall_ns": record.started_wall_ns,
        }
    return evidence


def _attached_children(
    plan: Any,
    state_root: Path,
    session_id: str,
) -> dict[str, SimChildRecord]:
    try:
        snapshot = SimChildLedger(state_root).load()
    except SimChildLedgerError as exc:
        raise RuntimeError("strict map acceptance child ledger is not trusted") from exc
    if snapshot is None or snapshot.product_session_id != session_id:
        raise RuntimeError("strict map acceptance child ledger identity is invalid")
    children: dict[str, SimChildRecord] = {}
    for name in _ATTACHED_PROCESSES:
        process = next((item for item in plan.processes if item.name == name), None)
        if process is None:
            raise RuntimeError(f"map RunPlan does not declare {name}")
        records = [item for item in snapshot.children if item.target == process.target]
        if len(records) != 1:
            raise RuntimeError(f"strict map acceptance {name} child is missing")
        children[name] = records[0]
    _revalidate_children(children)
    return children


def _readiness(
    plan: Any,
    state_root: Path,
    children: Mapping[str, SimChildRecord],
    session_id: str,
) -> dict[str, Mapping[str, Any]]:
    evidence: dict[str, Mapping[str, Any]] = {}
    for name in (
        "lidar_publisher",
        "imu_publisher",
        "camera_publisher",
        "slam_runtime",
        "map_runtime",
    ):
        process = next((item for item in plan.processes if item.name == name), None)
        if process is None or process.command is None or process.command.readiness is None:
            raise RuntimeError(f"map RunPlan does not declare {name} readiness")
        readiness = process.command.readiness
        expectation = readiness_expectation_for_process(name, readiness.target)
        if readiness.kind != "file" or expectation is None:
            raise RuntimeError(f"map RunPlan {name} readiness is not canonical")
        extra = {"expected_slam_mode": "mapping"} if name == "slam_runtime" else {}
        evidence[name] = load_typed_readiness(
            state_root / readiness.target,
            expectation=expectation,
            product_session_id=session_id,
            product="map",
            process=name,
            started_wall_ns=children[name].started_wall_ns,
            **extra,
        )
    details = evidence["map_runtime"].get("details")
    if not isinstance(details, Mapping) or details.get("live") is not True:
        raise RuntimeError("typed mapd readiness is not live")
    for field in ("reset_epoch", "observation_sequence", "processed_observations"):
        value = details.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise RuntimeError(f"typed mapd readiness has no live {field}")
    return evidence


def _camera(state_root: Path, feeder_start_ns: int) -> dict[str, Any]:
    now_ns = time.time_ns()
    streams: dict[str, dict[str, Any]] = {}
    for name, filename, kind in (
        ("color", "camera_color.shm", StreamKind.COLOR),
        ("depth", "camera_depth.shm", StreamKind.DEPTH),
        ("info", "camera_info.shm", StreamKind.INFO),
    ):
        with ShmFrameReader(
            state_root / filename, max_age_s=None if name == "info" else _CAMERA_STALE_TIMEOUT_S
        ) as reader:
            frame = reader.read_latest(now_ns=now_ns)
        if frame is None:
            raise RuntimeError(f"camera {name} SHM has no committed frame")
        if frame.stream_kind is not kind:
            raise RuntimeError(f"camera {name} SHM carries the wrong stream kind")
        if frame.timestamp_ns < feeder_start_ns:
            raise RuntimeError(f"camera {name} SHM predates the exact feeder child")
        streams[name] = {"sequence": frame.sequence, "timestamp_ns": frame.timestamp_ns, "frame_id": frame.frame_id}
    return {"ready": True, "transport": "session_shm", "observed_wall_ns": now_ns, "streams": streams}


def _map_save(plan: Any, session_id: str, timeout_s: float) -> tuple[Mapping[str, Any], int]:
    port = plan.host_config.get("gateway_port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("map RunPlan Gateway port is invalid")
    name = f"mujoco-map-{session_id.lower()}"
    client = LingTuClient("127.0.0.1", port)
    started_ns = time.time_ns()
    operation = client.save_map_and_wait(name, timeout=timeout_s, poll_interval=0.25)
    payload = operation.get("operation")
    state = str(payload.get("state") or "").upper() if isinstance(payload, Mapping) else ""
    if state != "SUCCEEDED":
        raise RuntimeError(f"Gateway map save did not succeed: {state or 'UNKNOWN'}")
    saved = next((item for item in client.maps().maps if item.name == name), None)
    if saved is None or saved.has_pcd is not True:
        raise RuntimeError("Gateway map list does not expose the saved PCD")
    size_bytes = max(0, saved.size_bytes) if type(saved.size_bytes) is int else 0
    size_mb = saved.raw.get("size_mb") if isinstance(saved.raw, Mapping) else None
    size_mb_value = (
        float(size_mb)
        if isinstance(size_mb, (int, float))
        and not isinstance(size_mb, bool)
        and math.isfinite(float(size_mb))
        and float(size_mb) > 0
        else None
    )
    if size_bytes <= 0 and size_mb_value is None:
        raise RuntimeError("Gateway map list exposes an empty saved PCD")
    return {
        "name": name,
        "operation_state": state,
        "has_pcd": True,
        "size_bytes": size_bytes,
        "size_mb": size_mb_value,
    }, started_ns


def _require_camera_advanced(
    before: Mapping[str, Any],
    after: Mapping[str, Any],
    save_started_ns: int,
) -> None:
    for stream in ("color", "depth"):
        if (
            after["streams"][stream]["sequence"] <= before["streams"][stream]["sequence"]
            or after["streams"][stream]["timestamp_ns"] < save_started_ns
        ):
            raise RuntimeError(f"camera {stream} did not advance after Gateway save started")


def _run_attached_acceptance(
    plan: Any, run_plan_path: Path, manifest: Mapping[str, Any], *, timeout_s: float
) -> dict[str, Any]:
    state_root, session_id, current_before = _attached_context(plan, run_plan_path)
    children = _attached_children(plan, state_root, session_id)
    readiness = _readiness(plan, state_root, children, session_id)
    child_identity_before = _revalidate_children(children)
    before = _camera(state_root, children["mujoco_feeder"].started_wall_ns)
    saved, save_started_ns = _map_save(plan, session_id, timeout_s)
    child_identity_after = _revalidate_children(children)
    after = _camera(state_root, children["mujoco_feeder"].started_wall_ns)
    _require_camera_advanced(before, after, save_started_ns)
    if _attached_context(plan, run_plan_path) != (state_root, session_id, current_before):
        raise RuntimeError("strict map acceptance current identity changed during Gateway save")
    report: dict[str, Any] = {
        "schema_version": REPORT_SCHEMA,
        "ok": True,
        "strict": True,
        "preflight_only": False,
        "product_contract": manifest.get("product_contract"),
        "acceptance_scope": manifest.get("acceptance_scope"),
        "attached_identity": {
            "run_plan": str(run_plan_path.resolve()),
            "product_session_id": session_id,
        },
        "children": {
            "before_save": child_identity_before,
            "after_save": child_identity_after,
        },
        "readiness": {name: dict(value) for name, value in readiness.items()},
        "camera": {"before": before, "after": after},
        "map_save": dict(saved),
        "blockers": [],
    }
    report.update(
        classify_evidence(manifest.get("acceptance_scope"), run_plan_verified=True, acceptance_evaluated=True, ok=True)
    )
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report_path = args.json_out.expanduser().resolve() if args.json_out else None
    try:
        manifest_path = args.manifest.expanduser().resolve()
        plan = (
            validate_runner_plan(
                ROOT, args.run_plan, manifest_path, expected_products=("map",)
            )
            if args.run_plan
            else None
        )
        validate_product_contract(AcceptanceTarget("map", Path(__file__).resolve(), manifest_path), plan=plan)
        manifest = _load(manifest_path)
        if not args.strict:
            raise ValueError("map acceptance is attach-only and requires --strict")
        if plan is None or args.run_plan is None:
            raise ValueError("strict map acceptance requires --run-plan")
        timeout_s = float(args.duration if args.duration is not None else 300.0)
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("strict map acceptance timeout must be positive")
        report_path = report_path or args.run_plan.resolve().parent / "mujoco-map-acceptance.report.json"
        _attached_context(plan, args.run_plan)
        details = (
            {"preflight_only": True}
            if args.preflight_only
            else _run_attached_acceptance(plan, args.run_plan, manifest, timeout_s=timeout_s)
        )
        report = {
            "schema_version": REPORT_SCHEMA,
            "ok": True,
            "strict": True,
            "acceptance_scope": manifest.get("acceptance_scope"),
            "blockers": [],
            **details,
        }
        report.update(
            classify_evidence(
                manifest.get("acceptance_scope"),
                run_plan_verified=True,
                acceptance_evaluated=not args.preflight_only,
                ok=True,
            )
        )
        _write_json(report_path, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except (OSError, RuntimeError) as exc:
        report = {
            "schema_version": REPORT_SCHEMA,
            "ok": False,
            "strict": bool(args.strict),
            "product_acceptance_passed": False,
            "blockers": [str(exc)],
        }
        if report_path:
            _write_json(report_path, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
