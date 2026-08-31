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

from sim.scripts.mujoco.product_acceptance import (  # noqa: E402
    AcceptanceTarget,
    classify_evidence,
    validate_product_contract,
)

from lingtu.product_lock import CURRENT_RUN_FILE_NAME  # noqa: E402
from lingtu.sdk.client import LingTuClient  # noqa: E402
from lingtu.sim.acceptance import validate_runner_plan  # noqa: E402
from lingtu.sim.switch import _load_committed_plan  # noqa: E402

DEFAULT_MANIFEST = ROOT / "config/runtime_graph/acceptance/mujoco_inspection_native_acceptance.json"
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


def _client(plan: Any) -> LingTuClient:
    port = plan.host_config.get("gateway_port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("inspection RunPlan Gateway port is invalid")
    return LingTuClient("127.0.0.1", port)


def _gateway_identity(client: LingTuClient, plan: Any, product_session_id: str) -> dict[str, Any]:
    readiness = client._get("/ready")
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


def _position(client: LingTuClient) -> tuple[float, float]:
    value = client.position()
    point = (float(value.x), float(value.y))
    if not all(math.isfinite(item) for item in point):
        raise RuntimeError("Gateway returned an invalid robot position")
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


def _validate_success_status(
    status: Mapping[str, Any],
    *,
    task_id: str,
    request_id: str,
    route_id: str,
    revision: int,
    map_id: str,
    point_id: str,
    action: str,
) -> Mapping[str, Any]:
    _require_identity(status, {"task_id": task_id}, "status")
    _require_identity(
        status.get("identity"),
        {
            "task_id": task_id,
            "route_id": route_id,
            "route_revision": revision,
            "map_id": map_id,
        },
        "status",
    )
    _require_identity(
        status.get("last_submission"),
        {"request_id": request_id, "action": "start"},
        "status submission",
    )
    latest = _require_identity(
        status.get("latest_event"),
        {
            "task_id": task_id,
            "request_id": request_id,
            "route_id": route_id,
            "route_revision": revision,
            "map_id": map_id,
            "point_id": point_id,
            "action": action,
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
        raise RuntimeError("inspection native terminal status is not successful")
    return latest


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
    ack = client._post(
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
    raw_thresholds = manifest.get("thresholds")
    thresholds: Mapping[str, Any] = raw_thresholds if isinstance(raw_thresholds, Mapping) else {}
    interval_s = float(thresholds.get("poll_interval_s", 0.25))
    minimum_m = float(thresholds.get("minimum_displacement_m", 0.1))
    client = _client(plan)
    identity = _gateway_identity(client, plan, product_session_id)
    start_x, start_y = _position(client)
    route_id = f"mujoco-inspection-{product_session_id.lower()}"
    request_id = f"mujoco-inspection-{uuid.uuid4().hex}"
    task_id = _expected_task_id(request_id)
    revision = 1
    point_id = "overview-1"
    action = "capture:overview"
    map_id = str(
        plan.native_process_environment.get("LINGTU_MAP_NAME") or plan.host_config.get("active_map") or ""
    ).strip()
    if not map_id:
        raise RuntimeError("inspection RunPlan has no exact saved-map identity")

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
        "revision": revision,
        "points": [
            {
                "id": point_id,
                "x": start_x + max(0.5, minimum_m * 3.0),
                "y": start_y,
                "yaw": 0.0,
                "tolerance": 0.15,
                "dwell": 0.0,
                "action": action,
            }
        ],
        "loop_count": 1,
        "failure_policy": "stop",
        "max_retries": 0,
    }
    try:
        route = client._post("/api/v1/inspection/routes", route_body)
    except Exception:
        assert_current_unchanged()
        raise
    if route.get("ok") is not True or not isinstance(route.get("route"), Mapping):
        assert_current_unchanged()
        raise RuntimeError("inspection route was not accepted")
    _require_identity(
        route["route"],
        {"id": route_id, "map_id": map_id, "revision": revision},
        "route",
    )
    submission_attempted = False
    try:
        submission_attempted = True
        ack = client._post(
            "/api/v1/inspection/tasks",
            {"route_id": route_id, "map_id": map_id, "revision": revision, "request_id": request_id},
        )
        if (
            ack.get("ok") is not True
            or ack.get("accepted") is not True
            or ack.get("action") != "start"
            or ack.get("lifecycle") != "submission_accepted"
            or ack.get("terminal") is not False
        ):
            raise RuntimeError("inspection business ACK is invalid")
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
        latest_event = _validate_success_status(
            status,
            task_id=task_id,
            request_id=request_id,
            route_id=route_id,
            revision=revision,
            map_id=map_id,
            point_id=point_id,
            action=action,
        )
        status = _poll_completed_recording(
            client,
            task_id,
            status,
            product_session_id,
            timeout_s,
            interval_s,
        )
        latest_event = _validate_success_status(
            status,
            task_id=task_id,
            request_id=request_id,
            route_id=route_id,
            revision=revision,
            map_id=map_id,
            point_id=point_id,
            action=action,
        )
        report = client._get(f"/api/v1/inspection/tasks/{task_id}/report")
        _require_identity(report, {"task_id": task_id}, "report")
        _require_identity(
            report.get("identity"),
            {"route_id": route_id, "route_revision": revision, "map_id": map_id},
            "report",
        )
        execution = report.get("execution")
        coverage = report.get("coverage")
        if (
            report.get("report_status") != "COMPLETE"
            or report.get("acceptance") != "ACCEPTABLE"
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
                    "required_points": 1,
                    "completed_points": 1,
                    "required_evidence": 1,
                    "verified_evidence": 1,
                    "missing_evidence": 0,
                    "invalid_evidence": 0,
                    "unavailable_evidence": 0,
                    "unknown_evidence": 0,
                }.items()
            )
        ):
            raise RuntimeError("inspection report is not COMPLETE and ACCEPTABLE")
        points = report.get("points")
        if not isinstance(points, list) or len(points) != 1 or not isinstance(points[0], Mapping):
            raise RuntimeError("inspection report has no unique verified evidence")
        point = points[0]
        _require_identity(
            point,
            {
                "loop_index": 0,
                "point_index": 0,
                "point_id": point_id,
                "action": action,
                "status": "COMPLETED",
                "evidence_status": "VERIFIED",
            },
            "report point",
        )
        evidence_id = str(point.get("evidence_id") or "")
        if not evidence_id:
            raise RuntimeError("inspection report has no unique verified evidence")
        evidence = client._get(f"/api/v1/inspection/evidence/{evidence_id}")
        summary = evidence.get("evidence")
        if not isinstance(summary, Mapping):
            raise RuntimeError("inspection evidence summary is missing")
        _require_identity(summary, {"evidence_id": evidence_id}, "evidence")
        _require_identity(
            summary.get("request"),
            {
                "request_id": evidence_id,
                "run_id": task_id,
                "route_id": route_id,
                "route_revision": revision,
                "map_id": map_id,
                "point_id": point_id,
                "point_index": 0,
                "action": action,
            },
            "evidence request",
        )
        if latest_event.get("action_request_id") != evidence_id:
            raise RuntimeError("inspection evidence request does not match the terminal event")
        artifacts = summary.get("artifacts") if isinstance(summary, Mapping) else None
        rgb = [item for item in artifacts or () if isinstance(item, Mapping) and item.get("kind") == "rgb"]
        rgb_bytes = rgb[0].get("bytes") if len(rgb) == 1 else None
        if (
            len(rgb) != 1
            or isinstance(rgb_bytes, bool)
            or not isinstance(rgb_bytes, int)
            or rgb_bytes <= 0
        ):
            raise RuntimeError("verified inspection evidence has no valid RGB artifact")
        recording = status.get("recording")
        if not isinstance(recording, Mapping):
            raise RuntimeError("inspection status has no task-bound recording")
        recording_session_id, mcap = _recording_mcap(recording, task_id, product_session_id)
        end_x, end_y = _position(client)
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
            "evidence": evidence,
            "recording": {"session_id": recording_session_id, "mcap": str(mcap)},
            "motion": {"start": [start_x, start_y], "end": [end_x, end_y], "displacement_m": displacement_m},
            "attached_identity": {
                "run_plan": str(run_plan_path.resolve()),
                "product_session_id": product_session_id,
            },
        }
    except Exception as exc:
        blockers = [str(exc)]
        if submission_attempted:
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
        manifest = _load_json(manifest_path)
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
