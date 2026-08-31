"""Attach-only visual-follow acceptance for an already-running tracking Product."""

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

from lingtu.sdk.client import LingTuClient  # noqa: E402
from lingtu.sim.acceptance import validate_runner_plan  # noqa: E402
from lingtu.sim.switch import _load_committed_plan  # noqa: E402

DEFAULT_MANIFEST = ROOT / "config/runtime_graph/acceptance/mujoco_tracking_native_acceptance.json"
REPORT_SCHEMA = "lingtu.mujoco.tracking_native_acceptance.report.v1"
PERSON_LABELS = frozenset({"person", "human", "pedestrian"})
ACTIVE_NAVIGATION_STATES = frozenset({"accepted", "planning", "path_active"})
STOPPED_NAVIGATION_STATES = frozenset({"cancelled", "reached", "idle"})


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
        raise ValueError(f"invalid tracking JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"tracking JSON must be an object: {path}")
    return payload


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _current_identity(plan: Any, run_plan_path: Path) -> tuple[Path, str]:
    exact = run_plan_path.expanduser().resolve()
    root = exact.parent
    committed = _load_committed_plan(root, os.environ)
    if committed is None:
        raise ValueError("strict tracking acceptance requires a committed current RunPlan")
    if committed.path != exact:
        raise ValueError("strict tracking acceptance RunPlan is not current")
    if (committed.plan, committed.plan.product, committed.plan.env) != (plan, "tracking", "sim"):
        raise ValueError("strict tracking acceptance current identity does not match the RunPlan")
    return root, committed.product_session_id


def _client(plan: Any) -> LingTuClient:
    port = plan.host_config.get("gateway_port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("tracking RunPlan Gateway port is invalid")
    return LingTuClient("127.0.0.1", port)


def _gateway_identity(client: LingTuClient, product_session_id: str) -> dict[str, Any]:
    readiness = client._get("/ready")
    contract = readiness.get("product_contract")
    if not isinstance(contract, Mapping) or (
        contract.get("product"),
        contract.get("product_session_id"),
    ) != ("tracking", product_session_id):
        raise RuntimeError("Gateway readiness identity does not match the tracking RunPlan")
    dataflow = client._get("/api/v1/runtime/dataflow")
    run_plan = dataflow.get("run_plan")
    identity = run_plan.get("identity") if isinstance(run_plan, Mapping) else None
    if (
        dataflow.get("authoritative") is not True
        or dataflow.get("available") is not True
        or not isinstance(identity, Mapping)
        or (identity.get("product"), identity.get("env")) != ("tracking", "sim")
    ):
        raise RuntimeError("Gateway dataflow identity does not match the tracking RunPlan")
    return {"readiness": dict(contract), "dataflow_identity": dict(identity)}


def _poll_person_id(client: LingTuClient, timeout_s: float, interval_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    previous = ""
    consecutive = 0
    while time.monotonic() < deadline:
        payload = client._get("/api/v1/scene_graph")
        objects = payload.get("objects")
        people = sorted(
            {
                str(item.get("id") or "").strip()
                for item in objects if isinstance(item, Mapping)
                if str(item.get("label") or "").strip().lower() in PERSON_LABELS
                and str(item.get("id") or "").strip()
            }
        ) if isinstance(objects, list) else []
        current = people[0] if people else ""
        consecutive = consecutive + 1 if current and current == previous else int(bool(current))
        previous = current
        if consecutive >= 2:
            return current
        time.sleep(interval_s)
    raise RuntimeError("scene graph did not expose one stable person identity")


def _visual_state(client: LingTuClient) -> Mapping[str, Any]:
    state = client._get("/api/v1/state")
    visual = state.get("visual_servo")
    if not isinstance(visual, Mapping):
        raise RuntimeError("Gateway visual servo state is unavailable")
    return visual


def _poll_following(
    client: LingTuClient,
    person_id: str,
    timeout_s: float,
    interval_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        visual = _visual_state(client)
        person = visual.get("person")
        if (
            visual.get("state") == "following"
            and visual.get("frame_id") == "map"
            and visual.get("target_id") == person_id
            and visual.get("target_visible") is True
            and visual.get("navigation_state") in ACTIVE_NAVIGATION_STATES
            and isinstance(person, Mapping)
            and person.get("id") == person_id
        ):
            return dict(visual)
        time.sleep(interval_s)
    raise RuntimeError("visual follow did not publish an exact target map goal")


def _poll_stopped(client: LingTuClient, timeout_s: float, interval_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        visual = _visual_state(client)
        if (
            visual.get("state") == "idle"
            and visual.get("navigation_state") in STOPPED_NAVIGATION_STATES
        ):
            return dict(visual)
        time.sleep(interval_s)
    raise RuntimeError("visual follow did not return to idle with its goal cleared")


def _person_position(visual: Mapping[str, Any], person_id: str) -> tuple[float, float, float]:
    person = visual.get("person")
    position = person.get("position") if isinstance(person, Mapping) else None
    if (
        not isinstance(person, Mapping)
        or person.get("id") != person_id
        or not isinstance(position, list)
        or len(position) < 3
    ):
        raise RuntimeError("visual follow person position is unavailable")
    values = tuple(float(value) for value in position[:3])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("visual follow person position is invalid")
    return values


def _robot_position(visual: Mapping[str, Any]) -> tuple[float, float, float]:
    position = visual.get("robot_position")
    if not isinstance(position, list) or len(position) < 3:
        raise RuntimeError("visual follow robot position is unavailable")
    values = tuple(float(value) for value in position[:3])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("visual follow robot position is invalid")
    return values


def _goal_position(visual: Mapping[str, Any]) -> tuple[float, float, float]:
    position = visual.get("goal_position")
    if not isinstance(position, list) or len(position) < 3:
        raise RuntimeError("visual follow goal position is unavailable")
    values = tuple(float(value) for value in position[:3])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("visual follow goal position is invalid")
    return values


def _poll_person_motion(
    client: LingTuClient,
    person_id: str,
    initial: Mapping[str, Any],
    timeout_s: float,
    interval_s: float,
    minimum_motion_m: float,
    minimum_robot_motion_m: float,
    minimum_goal_motion_m: float,
    maximum_distance_growth_m: float,
) -> dict[str, Any]:
    person_start = _person_position(initial, person_id)
    robot_start = _robot_position(initial)
    goal_start = _goal_position(initial)
    start_distance = math.dist(person_start[:2], robot_start[:2])
    task_ids = {
        task_id
        for task_id in (str(initial.get("navigation_task_id") or "").strip(),)
        if task_id
    }
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        visual = _visual_state(client)
        if (
            visual.get("state") == "following"
            and visual.get("target_visible") is True
            and visual.get("navigation_state") in ACTIVE_NAVIGATION_STATES
        ):
            person_current = _person_position(visual, person_id)
            robot_current = _robot_position(visual)
            goal_current = _goal_position(visual)
            person_motion = math.dist(person_start, person_current)
            robot_motion = math.dist(robot_start, robot_current)
            goal_motion = math.dist(goal_start, goal_current)
            current_distance = math.dist(person_current[:2], robot_current[:2])
            task_id = str(visual.get("navigation_task_id") or "").strip()
            if task_id:
                task_ids.add(task_id)
            if (
                person_motion >= minimum_motion_m
                and robot_motion >= minimum_robot_motion_m
                and goal_motion >= minimum_goal_motion_m
                and current_distance - start_distance <= maximum_distance_growth_m
            ):
                return {
                    "person_start_position": list(person_start),
                    "person_end_position": list(person_current),
                    "person_motion_m": person_motion,
                    "robot_start_position": list(robot_start),
                    "robot_end_position": list(robot_current),
                    "robot_motion_m": robot_motion,
                    "goal_start_position": list(goal_start),
                    "goal_end_position": list(goal_current),
                    "goal_motion_m": goal_motion,
                    "start_distance_m": start_distance,
                    "end_distance_m": current_distance,
                    "navigation_tasks_observed": len(task_ids),
                    "navigation_state": visual.get("navigation_state"),
                }
        time.sleep(interval_s)
    raise RuntimeError(
        "visual follow did not move the robot with updated native goals while the person moved"
    )


def _accepted(ack: Mapping[str, Any], action: str) -> None:
    command = ack.get("command")
    if (
        ack.get("ok") is not True
        or not isinstance(command, Mapping)
        or command.get("name") != "visual_servo"
        or command.get("accepted") is not True
    ):
        raise RuntimeError(f"visual servo {action} command was not accepted")


def _run_attached(
    plan: Any,
    run_plan_path: Path,
    manifest: Mapping[str, Any],
    timeout_s: float,
) -> dict[str, Any]:
    state_root, product_session_id = _current_identity(plan, run_plan_path)
    raw_thresholds = manifest.get("thresholds")
    thresholds = raw_thresholds if isinstance(raw_thresholds, Mapping) else {}
    interval_s = float(thresholds.get("poll_interval_s", 0.25))
    if not math.isfinite(interval_s) or interval_s <= 0:
        raise ValueError("tracking acceptance poll interval must be positive")
    minimum_motion_m = float(thresholds.get("min_person_motion_m", 0.2))
    if not math.isfinite(minimum_motion_m) or minimum_motion_m <= 0:
        raise ValueError("tracking acceptance person motion must be positive")
    minimum_robot_motion_m = float(thresholds.get("min_robot_motion_m", 0.2))
    if not math.isfinite(minimum_robot_motion_m) or minimum_robot_motion_m <= 0:
        raise ValueError("tracking acceptance robot motion must be positive")
    minimum_goal_motion_m = float(thresholds.get("min_goal_motion_m", 0.2))
    if not math.isfinite(minimum_goal_motion_m) or minimum_goal_motion_m <= 0:
        raise ValueError("tracking acceptance goal motion must be positive")
    maximum_distance_growth_m = float(
        thresholds.get("max_follow_distance_growth_m", 0.3)
    )
    if not math.isfinite(maximum_distance_growth_m) or maximum_distance_growth_m < 0:
        raise ValueError("tracking acceptance distance growth must be non-negative")
    client = _client(plan)
    identity = _gateway_identity(client, product_session_id)
    person_id = _poll_person_id(client, timeout_s, interval_s)

    def assert_current_unchanged() -> None:
        if _current_identity(plan, run_plan_path) != (state_root, product_session_id):
            raise RuntimeError("current RunPlan changed during tracking acceptance")

    request_id = f"mujoco-tracking-{uuid.uuid4().hex}"
    submission_attempted = False
    follow_status: dict[str, Any] | None = None
    motion: dict[str, Any] | None = None
    stop_ack: dict[str, Any] | None = None
    stop_status: dict[str, Any] | None = None
    failure: Exception | None = None
    try:
        submission_attempted = True
        follow_ack = client._post(
            "/api/v1/visual_servo",
            {
                "mode": "follow",
                "target_id": person_id,
                "request_id": request_id,
                "client_id": "mujoco-tracking-acceptance",
            },
        )
        _accepted(follow_ack, "follow")
        follow_status = _poll_following(client, person_id, timeout_s, interval_s)
        motion = _poll_person_motion(
            client,
            person_id,
            follow_status,
            timeout_s,
            interval_s,
            minimum_motion_m,
            minimum_robot_motion_m,
            minimum_goal_motion_m,
            maximum_distance_growth_m,
        )
    except Exception as exc:
        failure = exc
    finally:
        if submission_attempted:
            try:
                stop_ack = client._post(
                    "/api/v1/visual_servo",
                    {
                        "mode": "stop",
                        "request_id": f"stop-{request_id}",
                        "client_id": "mujoco-tracking-acceptance",
                    },
                )
                _accepted(stop_ack, "stop")
                stop_status = _poll_stopped(client, timeout_s, interval_s)
            except Exception as stop_exc:
                message = f"visual follow cleanup failed: {stop_exc}"
                failure = RuntimeError(f"{failure}; {message}" if failure else message)

    assert_current_unchanged()
    if failure is not None:
        raise failure
    if follow_status is None or motion is None or stop_ack is None or stop_status is None:
        raise RuntimeError("tracking acceptance did not complete")
    return {
        "product_session_id": product_session_id,
        "person_id": person_id,
        "identity": identity,
        "follow": follow_status,
        "motion": motion,
        "stop": stop_status,
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    manifest_path = args.manifest.expanduser().resolve()
    try:
        plan = (
            validate_runner_plan(
                ROOT,
                args.run_plan,
                manifest_path,
                expected_products=("tracking",),
            )
            if args.run_plan is not None
            else None
        )
        validate_product_contract(
            AcceptanceTarget("tracking", Path(__file__).resolve(), manifest_path),
            plan=plan,
        )
        manifest = _load_json(manifest_path)
        if not args.strict:
            raise ValueError("tracking acceptance is attach-only and requires --strict")
        if plan is None or args.run_plan is None:
            raise ValueError("strict tracking acceptance requires --run-plan")
        timeout_s = float(args.duration or (manifest.get("thresholds") or {}).get("timeout_s", 30.0))
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("tracking acceptance timeout must be positive")
        report_path = (
            args.json_out.resolve()
            if args.json_out
            else args.run_plan.resolve().parent / "mujoco-tracking-acceptance.report.json"
        )
        _current_identity(plan, args.run_plan)
        details = (
            {"preflight_only": True}
            if args.preflight_only
            else _run_attached(plan, args.run_plan, manifest, timeout_s)
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
