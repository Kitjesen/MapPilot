"""Attach-only strict acceptance for an already-running inspection Product."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
for path in (ROOT, SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from sim.scripts.mujoco.product_acceptance import (
    AcceptanceTarget,
    classify_evidence,
    validate_product_contract,
)

from lingtu.product_lock import CURRENT_RUN_FILE_NAME
from lingtu.sdk.client import LingTuClient
from lingtu.sim.acceptance import load_manifest, validate_runner_plan
from lingtu.sim.switch import _load_committed_plan
from lingtu.switch_contracts import (
    MapIdentity,
    map_identity_as_record,
    map_identity_environment,
    map_identity_from_record,
)

DEFAULT_MANIFEST = ROOT / "config" / "acceptance" / "mujoco" / "inspection.json"
REPORT_SCHEMA = "lingtu.mujoco.inspection_native_acceptance.report.v1"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid inspection JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"inspection JSON must be an object: {path}")
    return payload


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _current_identity(plan: Any, run_plan_path: Path) -> tuple[Path, str, bytes]:
    exact = run_plan_path.expanduser().resolve()
    root = exact.parent
    committed = _load_committed_plan(root, os.environ)
    if committed is None:
        raise ValueError("strict inspection acceptance requires a committed current RunPlan")
    if committed.path != exact:
        raise ValueError("strict inspection acceptance RunPlan is not current")
    if (committed.plan, committed.plan.product, committed.plan.env) != (
        plan,
        "inspection",
        "sim",
    ):
        raise ValueError("strict inspection acceptance current identity does not match the RunPlan")
    try:
        snapshot = (root / CURRENT_RUN_FILE_NAME).read_bytes()
    except OSError as exc:
        raise ValueError("strict inspection acceptance current identity is unavailable") from exc
    return root, committed.product_session_id, snapshot


def _current_map_identity(
    plan: Any,
    current_snapshot: bytes,
) -> MapIdentity:
    try:
        current = json.loads(current_snapshot.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("inspection committed current identity is invalid") from exc
    if not isinstance(current, Mapping):
        raise RuntimeError("inspection committed current identity is not an object")
    identity = map_identity_from_record(
        current.get("map_identity"),
        field_name="inspection committed map identity",
    )
    if current.get("map_name") != identity.map_id:
        raise RuntimeError("inspection committed map identity does not match map_name")
    environment = plan.native_process_environment
    expected_environment = map_identity_environment(identity)
    if any(environment.get(key) != value for key, value in expected_environment.items()):
        raise RuntimeError("inspection RunPlan does not bind the committed map identity")
    return identity


def _client(plan: Any) -> LingTuClient:
    port = plan.host_config.get("gateway_port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("inspection RunPlan Gateway port is invalid")
    return LingTuClient("127.0.0.1", port)


def _gateway_identity(client: LingTuClient, plan: Any, product_session_id: str) -> dict[str, Any]:
    readiness = client.readiness()
    contract = readiness.get("product_contract")
    if not isinstance(contract, Mapping) or (
        contract.get("product"),
        contract.get("product_session_id"),
    ) != ("inspection", product_session_id):
        raise RuntimeError("Gateway readiness identity does not match the inspection RunPlan")
    dataflow = client._get("/api/v1/runtime/dataflow")
    run_plan = dataflow.get("run_plan")
    identity = run_plan.get("identity") if isinstance(run_plan, Mapping) else None
    if (
        dataflow.get("authoritative") is not True
        or dataflow.get("available") is not True
        or not isinstance(identity, Mapping)
        or (identity.get("product"), identity.get("env"))
        != ("inspection", "sim")
    ):
        raise RuntimeError("Gateway dataflow identity does not match the inspection RunPlan")
    return {"readiness": dict(contract), "dataflow_identity": dict(identity)}


def _position(client: LingTuClient) -> tuple[float, float, float]:
    state = client._get("/api/v1/state")
    localization = state.get("localization") if isinstance(state, Mapping) else None
    odometry = localization.get("odometry") if isinstance(localization, Mapping) else None
    if not isinstance(odometry, Mapping) or odometry.get("frame_id") != "map":
        raise RuntimeError("Gateway state has no native map-frame robot pose")
    raw_point = (odometry.get("x"), odometry.get("y"), odometry.get("z"))
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in raw_point):
        raise RuntimeError("Gateway native map-frame robot pose is invalid")
    point = tuple(float(item) for item in raw_point)
    if not all(math.isfinite(item) for item in point):
        raise RuntimeError("Gateway native map-frame robot pose is invalid")
    return point


def _poll_task(client: LingTuClient, task_id: str, timeout_s: float, interval_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        payload = client._get(f"/api/v1/inspection/tasks/{task_id}")
        if not isinstance(payload, Mapping):
            raise RuntimeError("inspection task status is not an object")
        status = dict(payload)
        if status.get("task_id") != task_id:
            raise RuntimeError("inspection task status identity mismatch")
        if status.get("terminal") is True:
            return status
        time.sleep(interval_s)
    raise RuntimeError("inspection task did not reach a native terminal state")


def _expected_task_id(request_id: str) -> str:
    return f"inspection-task-{uuid.uuid5(uuid.NAMESPACE_URL, f'lingtu-inspection:{request_id}').hex}"


def _require_identity(value: Any, expected: Mapping[str, Any], label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeError(f"inspection {label} identity is missing")
    for field, expected_value in expected.items():
        if value.get(field) != expected_value:
            raise RuntimeError(f"inspection {label} identity mismatch: {field}")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"inspection route point {label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"inspection route point {label} must be finite")
    return number


def _route_points(
    manifest: Mapping[str, Any],
    start_x: float,
    start_y: float,
    start_z: float,
    minimum_m: float,
) -> list[dict[str, Any]]:
    raw_points = manifest.get("route_points")
    if raw_points is None:
        raw_points = [
            {
                "id": "overview-1",
                "relative_dx": max(0.5, minimum_m * 3.0),
                "relative_dy": 0.0,
                "action": "capture:overview",
            }
        ]
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("inspection route_points must be a non-empty list")

    points: list[dict[str, Any]] = []
    point_ids: set[str] = set()
    for index, raw_point in enumerate(raw_points):
        if not isinstance(raw_point, Mapping):
            raise ValueError(f"inspection route point {index} must be an object")
        point_id = str(raw_point.get("id") or "").strip()
        action = str(raw_point.get("action") or "capture:overview").strip()
        if not point_id or point_id in point_ids:
            raise ValueError("inspection route point ids must be non-empty and unique")
        if not action:
            raise ValueError(f"inspection route point {point_id} action is empty")
        point_ids.add(point_id)

        has_relative = "relative_dx" in raw_point or "relative_dy" in raw_point
        has_world = "x" in raw_point or "y" in raw_point
        if has_relative == has_world:
            raise ValueError(
                f"inspection route point {point_id} must use exactly one coordinate mode: "
                "relative_dx/relative_dy or x/y"
            )
        if has_relative:
            if "relative_dx" not in raw_point or "relative_dy" not in raw_point:
                raise ValueError(
                    f"inspection route point {point_id} requires relative_dx and relative_dy"
                )
            x = start_x + _finite_number(raw_point["relative_dx"], f"{point_id}.relative_dx")
            y = start_y + _finite_number(raw_point["relative_dy"], f"{point_id}.relative_dy")
            if "z" in raw_point:
                raise ValueError(
                    f"inspection relative route point {point_id} must use relative_dz, not z"
                )
            z = start_z + _finite_number(
                raw_point.get("relative_dz", 0.0), f"{point_id}.relative_dz"
            )
        else:
            if "x" not in raw_point or "y" not in raw_point:
                raise ValueError(f"inspection route point {point_id} requires x and y")
            x = _finite_number(raw_point["x"], f"{point_id}.x")
            y = _finite_number(raw_point["y"], f"{point_id}.y")
            if "relative_dz" in raw_point:
                raise ValueError(
                    f"inspection world route point {point_id} must use z, not relative_dz"
                )
            z = _finite_number(raw_point.get("z", start_z), f"{point_id}.z")

        points.append(
            {
                "id": point_id,
                "x": x,
                "y": y,
                "z": z,
                "yaw": _finite_number(raw_point.get("yaw", 0.0), f"{point_id}.yaw"),
                "tolerance": _finite_number(
                    raw_point.get("tolerance", 0.15), f"{point_id}.tolerance"
                ),
                "dwell": _finite_number(raw_point.get("dwell", 0.0), f"{point_id}.dwell"),
                "action": action,
            }
        )
    return points


def _validate_success_status(
    status: Mapping[str, Any],
    *,
    task_id: str,
    request_id: str,
    route_id: str,
    revision: int,
    map_id: str,
    map_content_epoch: int,
    route_points: Sequence[Mapping[str, Any]],
    loop_count: int = 1,
) -> list[Mapping[str, Any]]:
    _require_identity(status, {"task_id": task_id}, "status")
    _require_identity(
        status.get("identity"),
        {
            "task_id": task_id,
            "route_id": route_id,
            "route_revision": revision,
            "map_id": map_id,
            "map_content_epoch": map_content_epoch,
        },
        "status",
    )
    _require_identity(
        status.get("last_submission"),
        {"request_id": request_id, "action": "start"},
        "status submission",
    )
    _require_identity(
        status.get("latest_event"),
        {
            "task_id": task_id,
            "request_id": request_id,
            "route_id": route_id,
            "route_revision": revision,
            "map_id": map_id,
            "map_content_epoch": map_content_epoch,
        },
        "terminal event",
    )
    delivery = status.get("delivery")
    if (
        status.get("terminal") is not True
        or status.get("current_state") != "SUCCESS"
        or status.get("execution_confirmed") is not True
        or status.get("terminal_source") != "native_task_event"
        or not isinstance(delivery, Mapping)
        or delivery.get("history_complete") is not True
    ):
        event = status.get("latest_event")
        reason = event.get("reason") if isinstance(event, Mapping) else None
        raise RuntimeError(
            f"inspection native terminal status is not successful: "
            f"state={status.get('current_state')}, reason={reason or 'unknown'}"
        )
    _require_identity(
        status.get("latest_event"),
        {"point_index": len(route_points), "action": ""},
        "terminal event",
    )
    timeline = status.get("timeline")
    if not isinstance(timeline, list):
        raise RuntimeError("inspection native terminal timeline is missing")
    evidence_events = [
        event
        for event in timeline
        if isinstance(event, Mapping) and event.get("kind") == 5
    ]
    visits = [(loop, index, point) for loop in range(loop_count)
              for index, point in enumerate(route_points)]
    if len(evidence_events) != len(visits):
        raise RuntimeError("inspection native evidence event coverage mismatch")
    for (loop_index, point_index, point), event in zip(visits, evidence_events, strict=True):
        _require_identity(
            event,
            {
                "task_id": task_id,
                "request_id": request_id,
                "route_id": route_id,
                "route_revision": revision,
                "map_id": map_id,
                "map_content_epoch": map_content_epoch,
                "point_index": point_index,
                "loop_index": loop_index,
                "action": point["action"],
            },
            f"native evidence event {point_index}",
        )
        if not str(event.get("evidence_id") or ""):
            raise RuntimeError(f"inspection native evidence event {point_index} has no evidence")
    completions = [event for event in timeline if isinstance(event, Mapping)
                   and event.get("kind") == 3 and event.get("reason") == "point_completed"]
    if len(completions) != len(visits):
        raise RuntimeError("inspection native point completion coverage mismatch")
    for (loop_index, point_index, _point), event in zip(visits, completions, strict=True):
        _require_identity(event, {"task_id": task_id, "route_id": route_id,
                                 "route_revision": revision, "map_id": map_id,
                                 "map_content_epoch": map_content_epoch,
                                 "loop_index": loop_index, "point_index": point_index},
                          f"native completion {loop_index}/{point_index}")
    return evidence_events


def _loop_waits(status: Mapping[str, Any], loop_count: int, loop_interval_s: float) -> list[dict[str, Any]]:
    if loop_interval_s == 0.0:
        return []
    timeline = status.get("timeline", [])
    waits = [event for event in timeline if isinstance(event, Mapping)
             and event.get("state") == 14 and event.get("reason") == "waiting_next_loop"]
    if [event.get("loop_index") for event in waits] != list(range(1, loop_count)):
        raise RuntimeError("inspection native loop wait coverage mismatch")
    return [dict(event) for event in waits]


def _repetition(manifest: Mapping[str, Any]) -> tuple[int, float]:
    loop_count = manifest.get("loop_count", 1)
    if isinstance(loop_count, bool) or not isinstance(loop_count, int) or loop_count < 1:
        raise ValueError("inspection loop_count must be a positive integer")
    interval_s = _finite_number(manifest.get("loop_interval_s", 0.0), "loop_interval_s")
    if interval_s < 0.0:
        raise ValueError("inspection loop_interval_s must be nonnegative")
    return loop_count, interval_s


def _completed_recording(
    status: Mapping[str, Any],
    task_id: str,
    product_session_id: str,
) -> Mapping[str, Any] | None:
    _require_identity(status, {"task_id": task_id, "terminal": True}, "recording status")
    recording = status.get("recording")
    if not isinstance(recording, Mapping):
        return None
    state = str(recording.get("state") or "")
    if state == "failed":
        raise RuntimeError("task-bound recording failed")
    if state != "completed":
        return None
    session_id = recording.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        raise RuntimeError("task-bound recording completed without a session identity")
    if recording.get("product_session_id") != product_session_id:
        raise RuntimeError("task-bound recording Product session mismatch")
    return recording


def _poll_completed_recording(
    client: LingTuClient,
    task_id: str,
    initial: Mapping[str, Any],
    product_session_id: str,
    timeout_s: float,
    interval_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    status = dict(initial)
    while time.monotonic() < deadline:
        if _completed_recording(status, task_id, product_session_id) is not None:
            return status
        time.sleep(interval_s)
        payload = client._get(f"/api/v1/inspection/tasks/{task_id}")
        if not isinstance(payload, Mapping):
            raise RuntimeError("inspection terminal task status is not an object")
        status = dict(payload)
        if status.get("task_id") != task_id or status.get("terminal") is not True:
            raise RuntimeError("inspection terminal task identity changed while recording stopped")
    raise RuntimeError("task-bound recording did not complete")


def _cancel_expected_task(
    client: LingTuClient,
    task_id: str,
    product_session_id: str,
    timeout_s: float,
    interval_s: float,
) -> dict[str, Any]:
    cancel_request_id = f"cancel-{uuid.uuid4().hex}"
    ack = client._request_json(
        f"/api/v1/inspection/tasks/{task_id}/cancel",
        {"reason": "mujoco_acceptance_cleanup", "request_id": cancel_request_id},
    )
    _require_identity(
        ack,
        {"task_id": task_id, "request_id": cancel_request_id, "action": "cancel"},
        "cleanup ACK",
    )
    terminal = _poll_task(client, task_id, min(timeout_s, 10.0), interval_s)
    final = _poll_completed_recording(
        client,
        task_id,
        terminal,
        product_session_id,
        min(timeout_s, 10.0),
        interval_s,
    )
    return {"cancel_ack": dict(ack), "terminal_status": final}


def _recording_mcap(recording: Mapping[str, Any], task_id: str, product_session_id: str) -> tuple[str, Path]:
    session_id = str(recording.get("session_id") or "").strip()
    if recording.get("state") != "completed" or not session_id:
        raise RuntimeError("task-bound recording is not completed")
    if str(recording.get("product_session_id") or "") != product_session_id:
        raise RuntimeError("task-bound recording Product session mismatch")
    root = Path(os.path.expanduser(os.environ.get("LINGTU_RECORDING_ROOT", "~/data/lingtu/recordings"))).resolve()
    session_dir = (root / session_id).resolve()
    if session_dir.parent != root:
        raise RuntimeError("task-bound recording session path is unsafe")
    try:
        manifest = _load_json(session_dir / "session.json")
    except ValueError as exc:
        raise RuntimeError("task-bound recording manifest is invalid") from exc
    context = manifest.get("context")
    if manifest.get("state") != "completed" or not isinstance(context, Mapping):
        raise RuntimeError("task-bound recording manifest is not completed")
    if (context.get("task_id"), context.get("product_session_id")) != (task_id, product_session_id):
        raise RuntimeError("task-bound recording manifest identity mismatch")
    candidates: list[Path] = []
    raw_children = manifest.get("children")
    children = raw_children if isinstance(raw_children, list) else []
    for child in children:
        if not isinstance(child, Mapping):
            continue
        raw_artifacts = child.get("artifacts")
        artifacts = raw_artifacts if isinstance(raw_artifacts, list) else []
        for artifact in artifacts:
            if not isinstance(artifact, str) or not artifact.endswith(".mcap"):
                continue
            candidate = (session_dir / artifact).resolve()
            if session_dir in candidate.parents and candidate.is_file() and candidate.stat().st_size > 0:
                candidates.append(candidate)
    if len(candidates) != 1:
        raise RuntimeError("task-bound recording has no unique non-empty MCAP")
    return session_id, candidates[0]


def _run_attached(plan: Any, run_plan_path: Path, manifest: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
    state_root, product_session_id, current_before = _current_identity(plan, run_plan_path)
    map_identity = _current_map_identity(
        plan,
        current_before,
    )
    raw_thresholds = manifest.get("thresholds")
    thresholds: Mapping[str, Any] = raw_thresholds if isinstance(raw_thresholds, Mapping) else {}
    interval_s = float(thresholds.get("poll_interval_s", 0.25))
    minimum_m = float(thresholds.get("minimum_displacement_m", 0.1))
    client = _client(plan)
    identity = _gateway_identity(client, plan, product_session_id)
    start_x, start_y, start_z = _position(client)
    route_id = f"mujoco-inspection-{product_session_id.lower()}"
    request_id = f"mujoco-inspection-{uuid.uuid4().hex}"
    task_id = _expected_task_id(request_id)
    revision = 1
    route_points = _route_points(manifest, start_x, start_y, start_z, minimum_m)
    loop_count, loop_interval_s = _repetition(manifest)
    visits = [(loop, index, point) for loop in range(loop_count)
              for index, point in enumerate(route_points)]
    map_id = map_identity.map_id

    def assert_current_unchanged() -> None:
        observed_root, observed_session_id, current_after = _current_identity(
            plan,
            run_plan_path,
        )
        if (observed_root, observed_session_id, current_after) != (
            state_root,
            product_session_id,
            current_before,
        ):
            raise RuntimeError("current RunPlan changed during inspection acceptance")

    route_body = {
        "id": route_id,
        "name": "MuJoCo attached inspection acceptance",
        "map_id": map_id,
        "map_content_epoch": map_identity.content_epoch,
        "revision": revision,
        "points": route_points,
        "loop_count": loop_count,
        "loop_interval_s": loop_interval_s,
        "failure_policy": "stop",
        "max_retries": 0,
    }
    try:
        route = client._request_json("/api/v1/inspection/routes", route_body)
    except Exception:
        assert_current_unchanged()
        raise
    if route.get("ok") is not True or not isinstance(route.get("route"), Mapping):
        assert_current_unchanged()
        error = str(route.get("error") or route.get("code") or "unknown_error")
        detail = route.get("detail")
        raise RuntimeError(
            "inspection route was not accepted: "
            f"error={error}, detail={json.dumps(detail, ensure_ascii=False, sort_keys=True)}"
        )
    _require_identity(
        route["route"],
        {
            "id": route_id,
            "map_id": map_id,
            "map_content_epoch": map_identity.content_epoch,
            "revision": revision,
        },
        "route",
    )
    submission_attempted = False
    native_failed_status: Mapping[str, Any] | None = None
    try:
        submission_attempted = True
        ack = client._request_json(
            "/api/v1/inspection/tasks",
            {"route_id": route_id, "map_id": map_id, "revision": revision, "request_id": request_id},
        )
        if ack.get("ok") is False and ack.get("accepted") is not True:
            submission_attempted = False
        if (
            ack.get("ok") is not True
            or ack.get("accepted") is not True
            or ack.get("action") != "start"
            or ack.get("lifecycle") != "submission_accepted"
            or ack.get("terminal") is not False
        ):
            error = str(ack.get("error") or ack.get("code") or "unknown_error")
            detail = ack.get("detail")
            raise RuntimeError(
                "inspection business ACK is invalid: "
                f"error={error}, detail={json.dumps(detail, ensure_ascii=False, sort_keys=True)}"
            )
        _require_identity(
            ack,
            {
                "task_id": task_id,
                "request_id": request_id,
                "route_id": route_id,
                "revision": revision,
                "map_id": map_id,
            },
            "business ACK",
        )
        status = _poll_task(client, task_id, timeout_s, interval_s)
        if (status.get("current_state") != "SUCCESS"
                and status.get("terminal_source") == "native_task_event"
                and status.get("execution_confirmed") is True):
            native_failed_status = status
        evidence_events = _validate_success_status(
            status,
            task_id=task_id,
            request_id=request_id,
            route_id=route_id,
            revision=revision,
            map_id=map_id,
            map_content_epoch=map_identity.content_epoch,
            route_points=route_points,
            loop_count=loop_count,
        )
        status = _poll_completed_recording(
            client,
            task_id,
            status,
            product_session_id,
            timeout_s,
            interval_s,
        )
        evidence_events = _validate_success_status(
            status,
            task_id=task_id,
            request_id=request_id,
            route_id=route_id,
            revision=revision,
            map_id=map_id,
            map_content_epoch=map_identity.content_epoch,
            route_points=route_points,
            loop_count=loop_count,
        )
        loop_waits = _loop_waits(status, loop_count, loop_interval_s)
        report = client._get(f"/api/v1/inspection/tasks/{task_id}/report")
        _require_identity(report, {"task_id": task_id}, "report")
        _require_identity(
            report.get("identity"),
            {
                "route_id": route_id,
                "route_revision": revision,
                "map_id": map_id,
                "map_content_epoch": map_identity.content_epoch,
            },
            "report",
        )
        execution = report.get("execution")
        coverage = report.get("coverage")
        if (
            report.get("report_status") != "COMPLETE"
            or report.get("terminal") is not True
            or not isinstance(execution, Mapping)
            or execution.get("terminal") is not True
            or execution.get("confirmed") is not True
            or execution.get("history_complete") is not True
            or report.get("issues") != []
            or not isinstance(coverage, Mapping)
            or any(
                coverage.get(field) != expected
                for field, expected in {
                    "required_points": len(visits),
                    "completed_points": len(visits),
                    "required_evidence": len(visits),
                    "verified_evidence": len(visits),
                    "missing_evidence": 0,
                    "invalid_evidence": 0,
                    "unavailable_evidence": 0,
                    "unknown_evidence": 0,
                }.items()
            )
        ):
            raise RuntimeError("inspection report has incomplete execution or capture coverage")
        points = report.get("points")
        if not isinstance(points, list) or len(points) != len(visits):
            raise RuntimeError("inspection report point coverage mismatch")
        evidence_records: list[dict[str, Any]] = []
        for (loop_index, point_index, expected_point), point, event in zip(
            visits, points, evidence_events, strict=True
        ):
            if not isinstance(point, Mapping):
                raise RuntimeError(f"inspection report point {point_index} is invalid")
            _require_identity(
                point,
                {
                    "loop_index": loop_index,
                    "point_index": point_index,
                    "point_id": expected_point["id"],
                    "action": expected_point["action"],
                    "status": "COMPLETED",
                    "evidence_status": "VERIFIED",
                },
                f"report point {point_index}",
            )
            evidence_id = str(point.get("evidence_id") or "")
            if not evidence_id or evidence_id != event.get("evidence_id"):
                raise RuntimeError(
                    f"inspection report point {point_index} does not match native evidence"
                )
            evidence = client._get(f"/api/v1/inspection/evidence/{evidence_id}")
            summary = evidence.get("evidence")
            if not isinstance(summary, Mapping):
                raise RuntimeError(f"inspection evidence summary {point_index} is missing")
            _require_identity(summary, {"evidence_id": evidence_id}, f"evidence {point_index}")
            _require_identity(
                summary.get("request"),
                {
                    "request_id": evidence_id,
                    "task_id": task_id,
                    "route_id": route_id,
                    "route_revision": revision,
                    "map_id": map_id,
                    "map_content_epoch": map_identity.content_epoch,
                    "point_index": point_index,
                    "action": expected_point["action"],
                },
                f"evidence request {point_index}",
            )
            if event.get("action_request_id") != evidence_id:
                raise RuntimeError(
                    f"inspection evidence request {point_index} does not match the native event"
                )
            artifacts = summary.get("artifacts")
            rgb = [
                item
                for item in artifacts or ()
                if isinstance(item, Mapping) and item.get("kind") == "rgb"
            ]
            rgb_bytes = rgb[0].get("bytes") if len(rgb) == 1 else None
            if (
                len(rgb) != 1
                or isinstance(rgb_bytes, bool)
                or not isinstance(rgb_bytes, int)
                or rgb_bytes <= 0
            ):
                raise RuntimeError(
                    f"verified inspection evidence {point_index} has no valid RGB artifact"
                )
            evidence_records.append(dict(evidence))
        recording = status.get("recording")
        if not isinstance(recording, Mapping):
            raise RuntimeError("inspection status has no task-bound recording")
        recording_session_id, mcap = _recording_mcap(recording, task_id, product_session_id)
        end_x, end_y, _ = _position(client)
        displacement_m = math.hypot(end_x - start_x, end_y - start_y)
        if displacement_m < minimum_m:
            raise RuntimeError("inspection displacement is below the minimum threshold")
        assert_current_unchanged()
        return {
            "identity": identity,
            "evidence_worker": {"admission": "gateway_task_start"},
            "task_ack": ack,
            "task_status": status,
            "task_report": report,
            "evidence": evidence_records,
            "repetition": {"loop_count": loop_count, "loop_interval_s": loop_interval_s,
                           "visits_verified": len(visits), "waits": loop_waits},
            "equipment_condition_verified": False,
            "inspection_findings": report.get("inspection_result"),
            "recording": {"session_id": recording_session_id, "mcap": str(mcap)},
            "motion": {"start": [start_x, start_y], "end": [end_x, end_y], "displacement_m": displacement_m},
            "attached_identity": {
                "run_plan": str(run_plan_path.resolve()),
                "product_session_id": product_session_id,
                "map_identity": map_identity_as_record(map_identity),
            },
        }
    except Exception as exc:
        blockers = [str(exc)]
        if native_failed_status is not None:
            try:
                _poll_completed_recording(client, task_id, native_failed_status,
                                          product_session_id, min(timeout_s, 10.0), interval_s)
            except Exception as recording_exc:
                blockers.append(f"inspection recording finalization failed: {recording_exc}")
        elif submission_attempted:
            try:
                _cancel_expected_task(
                    client,
                    task_id,
                    product_session_id,
                    timeout_s,
                    interval_s,
                )
            except Exception as cleanup_exc:
                blockers.append(f"inspection cleanup failed: {cleanup_exc}")
        try:
            assert_current_unchanged()
        except Exception as identity_exc:
            blockers.append(str(identity_exc))
        raise RuntimeError("; ".join(blockers)) from exc


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    manifest_path = args.manifest.expanduser().resolve()
    try:
        plan = (
            validate_runner_plan(
                ROOT,
                args.run_plan,
                manifest_path,
                expected_products=("inspection",),
            )
            if args.run_plan is not None
            else None
        )
        validate_product_contract(
            AcceptanceTarget("inspection", Path(__file__).resolve(), manifest_path),
            plan=plan,
        )
        manifest = load_manifest(manifest_path, root=ROOT)
        if not args.strict:
            raise ValueError("inspection acceptance is attach-only and requires --strict")
        if plan is None or args.run_plan is None:
            raise ValueError("strict inspection acceptance requires --run-plan")
        timeout_s = float(args.duration or (manifest.get("thresholds") or {}).get("timeout_s", 120.0))
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("inspection acceptance timeout must be positive")
        report_path = (
            args.json_out.resolve()
            if args.json_out
            else args.run_plan.resolve().parent / "mujoco-inspection-acceptance.report.json"
        )
        _current_identity(plan, args.run_plan)
        details = (
            {"preflight_only": True} if args.preflight_only else _run_attached(plan, args.run_plan, manifest, timeout_s)
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
        if args.json_out is not None:
            _write_json(args.json_out.resolve(), report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
