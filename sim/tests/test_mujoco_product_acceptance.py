# ruff: noqa: S101

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import pytest

import lingtu.sim.switch as sim_switch
from drivers.real.camera.shm import ShmFrameWriter, StreamKind
from lingtu.assembly.compiler import compile_run_plan
from lingtu.control import ProductControl
from lingtu.run_plan import RunPlan
from lingtu.sim.identity import (
    ProcessIdentity,
    ProcessIdentityError,
    SimChildLedger,
    SimChildRecord,
    SimChildSnapshot,
)
from lingtu.sim.stop import (
    PROCESS_LAUNCH_ID_ENV,
    process_launch_id,
)
from lingtu.switch_contracts import ProcessReport
from runtime.graph import RuntimeGraph
from runtime.graph.loader import load_runtime_graph
from sim.scripts.mujoco import (
    inspection_native_acceptance,
    map_native_acceptance,
    product_acceptance,
    tracking_native_acceptance,
)
from sim.scripts.mujoco.evidence import (
    FEEDER_STATUS_SCHEMA,
    SimFeederStatusError,
    publish_feeder_status,
)

EXPECTED_PRODUCTS = (
    "teleop",
    "teleop_avoid",
    "map",
    "tracking",
    "nav",
    "inspection",
    "explore",
)


ROOT = Path(__file__).resolve().parents[2]
PRODUCT_SESSION_ID = "a" * 32

DIAGNOSTIC_TARGETS = {
    "teleop": ("teleop_native_acceptance.py", "mujoco_teleop_native_acceptance.json"),
    "teleop_avoid": (
        "teleop_avoid_native_acceptance.py",
        "mujoco_teleop_avoid_native_acceptance.json",
    ),
    "map": ("map_native_acceptance.py", "mujoco_map_native_acceptance.json"),
    "tracking": (
        "tracking_native_acceptance.py",
        "mujoco_tracking_native_acceptance.json",
    ),
    "nav": (
        "native_navigation_acceptance.py",
        "mujoco_local_cmu.json",
    ),
    "inspection": (
        "inspection_native_acceptance.py",
        "mujoco_inspection_native_acceptance.json",
    ),
    "explore": (
        "explore_native_acceptance.py",
        "mujoco_explore_native_acceptance.json",
    ),
}


def _target(product: str) -> product_acceptance.AcceptanceTarget:
    runner, manifest = DIAGNOSTIC_TARGETS[product]
    return product_acceptance.AcceptanceTarget(
        product=product,
        runner=(ROOT / "sim" / "scripts" / "mujoco" / runner).resolve(),
        manifest=(ROOT / "config" / "runtime_graph" / "acceptance" / manifest).resolve(),
    )


def _dispatcher_args(product: str, run_plan: Path, *extra: str) -> list[str]:
    target = _target(product)
    return [
        "--run-plan",
        str(run_plan),
        "--runner",
        str(target.runner),
        "--manifest",
        str(target.manifest),
        *extra,
    ]


def test_teleop_manifest_uses_canonical_windows_runtime_closure() -> None:
    manifest = json.loads(
        (ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_teleop_native_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    candidates = {name: spec["candidates"][0] for name, spec in manifest["binaries"].items()}
    assert candidates == {
        "sensor_publisher": ("build/windows-native-dds-adapter/Release/lingtu_mujoco_sensor_publisher.exe"),
        "navigation": ("build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe"),
        "navigation_control": ("build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_nav_control.exe"),
        "driver_bridge": ("build/windows-native-dds-adapter/Release/lingtu_mujoco_driver_bridge.exe"),
    }


def test_all_acceptance_manifests_use_only_canonical_windows_artifacts() -> None:
    acceptance_root = ROOT / "config" / "runtime_graph" / "acceptance"
    allowed_prefixes = (
        "build/maps-windows/Release/",
        "build/nav-cpp/windows-x64-nav-endpoint/Release/",
        "build/slam-core-windows-x64/stage/bin/",
        "build/windows-native-dds-adapter/Release/",
    )
    windows_candidates: list[tuple[Path, str]] = []
    binary_specs: list[tuple[Path, str, list[str]]] = []
    for path in sorted(acceptance_root.glob("*.json")):
        manifest = json.loads(path.read_text(encoding="utf-8"))
        for name, spec in (manifest.get("binaries") or {}).items():
            candidates = list(spec.get("candidates") or [])
            binary_specs.append((path, name, candidates))
            for candidate in candidates:
                if Path(candidate).suffix.casefold() == ".exe":
                    windows_candidates.append((path, candidate))

    assert windows_candidates
    assert [
        (path.name, candidate) for path, candidate in windows_candidates if not candidate.startswith(allowed_prefixes)
    ] == []
    assert [
        (path.name, name)
        for path, name, candidates in binary_specs
        if not any(Path(candidate).suffix.casefold() == ".exe" for candidate in candidates)
    ] == []


def test_teleop_case_uses_run_plan_dds_domain(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts.mujoco import teleop_native_acceptance

    captured: dict[str, object] = {}
    monkeypatch.setattr(
        teleop_native_acceptance,
        "prepare_runtime",
        lambda _args: {"ok": True},
    )

    def run_attached(**kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {"ok": True}

    monkeypatch.setattr(
        teleop_native_acceptance,
        "run_attached",
        run_attached,
    )
    target = product_acceptance.AcceptanceTarget(
        product="teleop",
        runner=tmp_path / "runner.py",
        manifest=tmp_path / "manifest.json",
    )
    scenario = product_acceptance._teleop_case(target)
    plan = SimpleNamespace(
        processes=(
            SimpleNamespace(
                name="nav_runtime",
                command=SimpleNamespace(
                    argv=("navd", "--domain-id", "231"),
                    env=(("LINGTU_DDS_DOMAIN_ID", "231"),),
                ),
            ),
        ),
    )

    assert scenario(plan, tmp_path / "plan.json", "a" * 32) == {"ok": True}
    assert cast(object, captured["args"]).domain_id == 231  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    ("processes", "message"),
    (
        ((), "select exactly one nav_runtime"),
        (
            (
                SimpleNamespace(name="nav_runtime", command=SimpleNamespace(argv=(), env=())),
                SimpleNamespace(name="nav_runtime", command=SimpleNamespace(argv=(), env=())),
            ),
            "select exactly one nav_runtime",
        ),
        (
            (
                SimpleNamespace(
                    name="nav_runtime",
                    command=SimpleNamespace(argv=("navd",), env=()),
                ),
            ),
            "declare --domain-id exactly once",
        ),
        (
            (
                SimpleNamespace(
                    name="nav_runtime",
                    command=SimpleNamespace(
                        argv=("navd", "--domain-id", "231", "--domain-id", "230"),
                        env=(),
                    ),
                ),
            ),
            "declare --domain-id exactly once",
        ),
        (
            (
                SimpleNamespace(
                    name="nav_runtime",
                    command=SimpleNamespace(
                        argv=("navd", "--domain-id", "not-a-domain"),
                        env=(),
                    ),
                ),
            ),
            "DDS domain ID is invalid",
        ),
        (
            (
                SimpleNamespace(
                    name="nav_runtime",
                    command=SimpleNamespace(
                        argv=("navd", "--domain-id", "231"),
                        env=(("LINGTU_DDS_DOMAIN_ID", "230"),),
                    ),
                ),
            ),
            "conflicts with command env",
        ),
    ),
)
def test_exact_teleop_attach_only_rejects_ambiguous_run_plan_dds_domain(
    processes: tuple[object, ...],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        product_acceptance._selected_nav_dds_domain_id(cast(RunPlan, SimpleNamespace(processes=processes)))


def _materialized_linux_graph(tmp_path: Path) -> RuntimeGraph:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    processes = envs["sim"]["backends"]["mujoco"]["processes"]
    for process_name, process in processes.items():
        commands = process.get("platforms")
        if commands is None:
            commands = {"portable": process["command"]}
        for platform, command in commands.items():
            suffix = ".exe" if platform == "windows" else ""
            relative = f"artifacts/{platform}/{process_name}{suffix}"
            artifact = tmp_path / relative
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_bytes(f"{platform}:{process_name}".encode())
            command["artifact"]["path"] = relative
            entry_index = 1 if str(command["argv"][0]).startswith("python") else 0
            command["argv"][entry_index] = relative
            materialized_dependencies: dict[str, str] = {}
            for index, dependency in enumerate(command.get("dependencies", [])):
                original = str(dependency["path"])
                dependency_relative = f"artifacts/{platform}/{process_name}-dependency-{index}{suffix}"
                dependency_path = tmp_path / dependency_relative
                dependency_path.write_bytes(f"{platform}:{process_name}:dependency:{index}".encode())
                dependency["path"] = dependency_relative
                materialized_dependencies[original] = dependency_relative
            for env_name, env_value in command.get("env", {}).items():
                if env_value in materialized_dependencies:
                    command["env"][env_name] = materialized_dependencies[env_value]
    return RuntimeGraph(
        root=tmp_path,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )


@pytest.fixture
def linux_sim_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> Callable[..., RunPlan]:
    graph = _materialized_linux_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "linux",
    )
    plans: dict[tuple[str, str | None, str | None], RunPlan] = {}

    def resolve(
        product: str,
        *,
        product_variant: str | None = None,
        local_planner: str | None = None,
    ) -> RunPlan:
        key = (product, product_variant, local_planner)
        if key not in plans:
            plans[key] = compile_run_plan(
                product,
                "sim",
                robot="doso/thunder_v4",
                env_config={"backend": "mujoco"},
                product_variant=product_variant,
                local_planner=local_planner,
                graph=graph,
            )
        plan = plans[key]
        catalog = graph.envs["sim"]["backends"]["mujoco"]["processes"]
        for process in plan.processes:
            if "platforms" not in catalog[process.name]:
                continue
            assert process.command is not None
            assert process.command.artifact.path.startswith("artifacts/linux/")
        return plan

    return resolve


def _write_exact_feeder_ledger(
    state_root: Path,
    plan: object,
    *,
    product_session_id: str,
    started_wall_ns: int,
) -> None:
    identity = ProcessIdentity.current()
    targets = {
        process.name: process.target
        for process in plan.processes  # type: ignore[attr-defined]
        if process.name in map_native_acceptance._ATTACHED_PROCESSES
    }
    SimChildLedger(state_root).replace(
        SimChildSnapshot.create(
            product_session_id=product_session_id,
            children=tuple(
                SimChildRecord(
                    target=targets[name],
                    process_identity=identity,
                    process_group=identity.pid,
                    started_wall_ns=started_wall_ns,
                    launch_id=f"launch-{name}",
                )
                for name in map_native_acceptance._ATTACHED_PROCESSES
            ),
        )
    )


def _write_camera_shm(
    state_root: Path,
    *,
    timestamp_ns: int,
    info_timestamp_ns: int | None = None,
    include: tuple[str, ...] = ("color", "depth", "info"),
    repeats: int = 1,
) -> None:
    specs = {
        "color": (StreamKind.COLOR, "camera_color.shm", "rgb8", 6, b"\x01" * 6),
        "depth": (StreamKind.DEPTH, "camera_depth.shm", "16UC1", 4, b"\x01" * 4),
        "info": (StreamKind.INFO, "camera_info.shm", "camera_info", 0, b""),
    }
    for name in include:
        kind, filename, encoding, stride, payload = specs[name]
        with ShmFrameWriter(
            state_root / filename,
            stream_kind=kind,
            slot_capacity=max(1, len(payload)),
        ) as writer:
            for offset in range(repeats):
                writer.publish(
                    timestamp_ns=(
                        info_timestamp_ns if name == "info" and info_timestamp_ns is not None else timestamp_ns + offset
                    ),
                    width=2,
                    height=1,
                    stride=stride,
                    encoding=encoding,
                    frame_id="camera_link",
                    payload=payload,
                    fx=100.0,
                    fy=100.0,
                    cx=1.0,
                    cy=0.5,
                )


def _ready(
    process: Any,
    *,
    product: str,
    product_session_id: str,
) -> dict[str, object]:
    readiness = process.command.readiness
    if readiness.kind == "process":
        return {"kind": "process", "target": process.target, "active": True}
    expected = product_acceptance.readiness_expectation_for_process(
        process.name,
        readiness.target,
    )
    assert expected is not None
    return {
        "kind": "file",
        "target": process.target,
        "adapter": expected.adapter,
        "payload": {
            "adapter": expected.adapter,
            "product_session_id": product_session_id,
            "product": product,
            "process": process.name,
            "ready": True,
        },
    }


class _Runner:
    def __init__(
        self,
        state_root: Path,
        *,
        fail_apply_number: int | None = None,
        incomplete_ready: bool = False,
    ) -> None:
        self.state_root = state_root
        self.fail_apply_number = fail_apply_number
        self.incomplete_ready = incomplete_ready
        self.stopped = False
        self.calls: list[str] = []

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert timeout_s is None
        self.calls.append("apply")
        plan = RunPlan.load(run_plan_path)
        apply_number = self.calls.count("apply")
        if self.fail_apply_number == apply_number:
            return ProcessReport(
                product=plan.product,
                env=plan.env,
                action="apply",
                ok=False,
                status="failed",
                error="injected apply failure",
            )
        self.stopped = False
        identity = ProcessIdentity.current()
        SimChildLedger(self.state_root).replace(
            SimChildSnapshot.create(
                product_session_id=product_session_id,
                children=tuple(
                    SimChildRecord(
                        target=process.target,
                        process_identity=identity,
                        process_group=identity.pid,
                        started_wall_ns=time.time_ns(),
                        launch_id=_launch_id(process.target),
                    )
                    for process in plan.processes
                ),
            )
        )
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
            planned=[process.target for process in plan.processes],
            started=[process.target for process in plan.processes],
            ready={
                process.name: _ready(
                    process,
                    product=plan.product,
                    product_session_id=product_session_id,
                )
                for index, process in enumerate(plan.processes)
                if not self.incomplete_ready or index > 0
            },
        )

    def quiesce(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert product_session_id
        assert timeout_s is None
        self.calls.append("quiesce")
        plan = RunPlan.load(run_plan_path)
        self.stopped = True
        SimChildLedger(self.state_root).clear()
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            ok=True,
            status="stopped",
        )

    def stop(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert product_session_id
        assert timeout_s is None
        self.calls.append("stop")
        plan = RunPlan.load(run_plan_path)
        self.stopped = True
        SimChildLedger(self.state_root).clear()
        _write_feeder_status(
            self.state_root,
            plan,
            product_session_id=product_session_id,
        )
        stop_evidence = {
            process.name: (
                {
                    "terminal_ack": True,
                    "outcome": "zero_applied",
                }
                if process.command is not None
                and process.command.shutdown is not None
                and process.command.shutdown.kind == "file"
                else {
                    "process": process.name,
                    "target": process.target,
                    "inactive": True,
                    "forced": False,
                }
            )
            for process in plan.processes
        }
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="stop",
            ok=True,
            status="stopped",
            stopped=[process.target for process in plan.processes],
            stop_evidence=stop_evidence,
        )


def _launch_id_for(target: str) -> str:
    return f"launch-{target}"


def _launch_id(target: str) -> str:
    return process_launch_id({PROCESS_LAUNCH_ID_ENV: _launch_id_for(target)})


def _write_feeder_status(
    state_root: Path,
    plan: RunPlan,
    *,
    product_session_id: str,
) -> None:
    feeder = next(
        (process for process in plan.processes if process.name == "mujoco_feeder"),
        None,
    )
    if feeder is None:
        return
    payload = _feeder_status_payload(plan, product_session_id=product_session_id)
    publish_feeder_status(
        session_root=state_root,
        environment={PROCESS_LAUNCH_ID_ENV: _launch_id_for(feeder.target)},
        payload=payload,
    )


def _feeder_status_payload(
    plan: RunPlan,
    *,
    product_session_id: str = "a" * 32,
) -> dict[str, object]:
    streams = plan.simulation["sensor_plan"]["streams"]
    expected: dict[str, float] = {}
    if plan.has_process("imu") and streams["imu"]:
        expected["imu"] = float(streams["imu"][0]["rate_hz"])
    if plan.has_process("lidar") and streams["mid360"]:
        expected["lidar"] = float(streams["mid360"][0]["rate_hz"])
    if plan.has_process("camera") and streams["rgb"] and streams["depth"]:
        expected["camera_rgbd"] = float(streams["rgb"][0]["rate_hz"])
    window_s = 10.0
    return {
        "schema": FEEDER_STATUS_SCHEMA,
        "product": plan.product,
        "product_session_id": product_session_id,
        "process": "mujoco_feeder",
        "state": "stopped",
        "sequence": 2,
        "updated_wall_ns": time.time_ns(),
        "window_s": window_s,
        "streams": {
            name: {
                "expected_hz": rate,
                "scheduled_count": int(rate * window_s),
                "published_count": int(rate * window_s),
                "dropped_count": 0,
                "actual_hz": rate,
                "max_schedule_lateness_ms": 1.0,
            }
            for name, rate in expected.items()
        },
    }


def _control(
    plan: RunPlan,
    root: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    fail_apply_number: int | None = None,
    incomplete_ready: bool = False,
) -> tuple[ProductControl, _Runner]:
    runner = _Runner(
        root,
        fail_apply_number=fail_apply_number,
        incomplete_ready=incomplete_ready,
    )
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    return control, runner


def _rollback(
    plan: RunPlan,
    root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> tuple[ProductControl, _Runner]:
    root.mkdir()
    return _control(plan, root, monkeypatch, fail_apply_number=2)


def _live(main: _Runner, rollback: _Runner) -> bool:
    return not (rollback if rollback.calls else main).stopped


def test_mujoco_is_the_only_product_mujoco_backend() -> None:
    backends = load_runtime_graph().envs["sim"]["backends"]
    assert "mujoco" in backends
    assert "mujoco_native" not in backends
    assert "mujoco_host" not in backends


def test_mujoco_catalog_declares_persistent_products() -> None:
    backend = load_runtime_graph().envs["sim"]["backends"]["mujoco"]
    assert backend["process_control"] == "subprocess"
    assert backend["process_manager"] == "direct"
    assert tuple(backend["supported_products"]) == (
        "teleop",
        "teleop_avoid",
        "map",
        "nav",
        "tracking",
        "inspection",
        "explore",
    )
    assert "supported_product_variants" not in backend
    assert "acceptance" not in backend
    assert "acceptance_manifests" not in backend
    assert "acceptance_runners" not in backend


def test_nav_catalog_binds_non_degraded_60m_manifest() -> None:
    manifest_path = (
        ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_industrial_park_60m_navigation_acceptance.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    start = manifest["start"]
    goal = manifest["goal"]
    bounds = manifest["scenario_geometry"]["bounds_xy_m"]
    clearance = float(manifest["scenario_geometry"]["min_endpoint_boundary_clearance_m"])

    assert manifest["product_contract"] == {
        "product": "nav",
        "source": "config/runtime_graph/products/nav.yaml",
        "native_control_mode": "autonomy",
        "slam_mode": "localization",
        "requires_map": True,
    }
    assert manifest["slam_runtime"]["provider"] == "fastlio2"
    assert "fixture" not in json.dumps(manifest).lower()
    assert manifest["sensor_runtime"]["imu_hz"] == 200.0
    assert manifest["sensor_runtime"]["scan_time_profile"] == "physical_rolling"
    assert manifest["sensor_runtime"].get("publish_odom_prior") is not True
    assert "navigation_prior" not in manifest["paths"]["slam_config"]
    assert math.hypot(goal[0] - start[0], goal[1] - start[1]) >= 50.0
    for point in (start, goal):
        assert bounds["min"][0] + clearance <= point[0] <= bounds["max"][0] - clearance
        assert bounds["min"][1] + clearance <= point[1] <= bounds["max"][1] - clearance
    assert manifest["thresholds"]["require_goal_reached"] is True
    assert manifest["thresholds"]["min_motion_m"] >= 50.0
    assert manifest["thresholds"]["min_net_displacement_m"] >= 50.0
    assert manifest["thresholds"]["min_goal_distance_reduction_m"] >= 50.0
    assert manifest["acceptance_scope"]["coverage"] == "component"
    assert "exact ProductControl RunPlan process realization" in manifest["acceptance_scope"]["excluded_claims"]
    assert (ROOT / manifest["world"]).is_file()


def test_persistent_teleop_run_plan_binds_its_acceptance_target(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop")

    assert plan.process_control == "subprocess"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert {process.name for process in plan.processes} == {
        "driver_bridge",
        "mujoco_feeder",
        "nav_runtime",
        "host_runtime",
    }


def test_persistent_teleop_avoid_run_plan_has_the_native_assisted_chain(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")

    assert plan.process_control == "subprocess"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert plan.lifecycle["native_control_mode"] == "teleop_avoid"
    assert {process.name for process in plan.processes} == {
        "driver_bridge",
        "lidar_publisher",
        "imu_publisher",
        "slam_runtime",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "traversability_runtime",
        "host_runtime",
    }
    assert {role for process in plan.processes for role in process.provides} == {
        "driver",
        "host",
        "imu",
        "lidar",
        "maps",
        "nav",
        "slam",
        "traversability",
    }
    assert plan.host_config["enable_camera"] is False
    assert "camera_backend" not in plan.host_config


def test_mujoco_explore_supports_live_and_saved_map_routes(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    live = linux_sim_plan("explore", product_variant="live")
    saved_map = linux_sim_plan("explore", product_variant="map")

    assert live.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "live"
    assert saved_map.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "map"
    assert live.lifecycle["requires_map"] is False
    assert saved_map.lifecycle["requires_map"] is True
    assert "acceptance" not in live.as_dict()["launch"]
    assert "acceptance" not in saved_map.as_dict()["launch"]
    assert {process.name for process in saved_map.processes} == {
        "driver_bridge",
        "lidar_publisher",
        "imu_publisher",
        "slam_runtime",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "traversability_runtime",
        "explore_runtime",
        "host_runtime",
    }
    assert "--route" not in saved_map.process("explore").command.argv
    assert "LINGTU_EXPLORE_ROUTE" not in dict(saved_map.process("explore").command.env)
    assert live != saved_map


@pytest.mark.parametrize("product", ("nav", "tracking", "inspection"))
def test_mujoco_saved_map_navigation_products_resolve_exact_chain(
    product: str,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan(product)

    assert plan.lifecycle["requires_map"] is True
    expected = {
        "driver_bridge",
        "lidar_publisher",
        "imu_publisher",
        "slam_runtime",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "traversability_runtime",
        "host_runtime",
    }
    if product in {"inspection", "tracking"}:
        expected.add("camera_publisher")
    assert {process.name for process in plan.processes} == expected
    assert "acceptance" not in plan.as_dict()["launch"]
    assert plan.host_config["enable_camera"] is (product in {"inspection", "tracking"})


def test_scan_is_a_nav_local_planner_selection(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("nav", local_planner="scan")
    target = product_acceptance.resolve_target(
        plan,
        runner=ROOT / "sim/scripts/mujoco/native_navigation_acceptance.py",
        manifest=ROOT / "config/runtime_graph/acceptance/mujoco_local_scan.json",
    )
    restored = RunPlan.from_dict(plan.as_dict())

    assert plan.product == "nav"
    assert plan.product_variant is None
    assert plan.native_nav["local_planner"] == "scan"
    assert "LINGTU_LOCAL_PLANNER_PATHS" not in plan.native_process_environment
    assert target.product == "nav"
    assert target.manifest.name == "mujoco_local_scan.json"
    assert restored == plan
    assert restored.native_nav["local_planner"] == "scan"


def test_cmu_and_scan_share_mujoco_truth_localization(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plans = tuple(
        linux_sim_plan("nav", local_planner=backend)
        for backend in ("cmu", "scan")
    )

    assert {
        plan.native_process_environment["LINGTU_SLAM_CONFIG"].replace("\\", "/")
        for plan in plans
    } == {"src/localization/fastlio2/config/sim_mid360.yaml"}
    assert all(product_acceptance._uses_mujoco_truth_localization(plan) for plan in plans)
    assert {plan.process("slam").name for plan in plans} == {"slam_runtime"}


def test_mujoco_product_acceptance_scopes_do_not_overclaim_partial_gates() -> None:
    scopes = {
        product: product_acceptance.acceptance_scope(
            product_acceptance.AcceptanceTarget(
                product=product,
                runner=(ROOT / "sim" / "scripts" / "mujoco" / runner).resolve(),
                manifest=(ROOT / "config" / "runtime_graph" / "acceptance" / manifest).resolve(),
            )
        )
        for product, (runner, manifest) in DIAGNOSTIC_TARGETS.items()
    }
    assert {product for product, scope in scopes.items() if scope["coverage"] == "product"} == set()
    assert scopes["teleop"]["coverage"] == "component"
    assert "exact ProductControl RunPlan process realization" in scopes["teleop"]["excluded_claims"]
    assert scopes["teleop_avoid"]["coverage"] == "component"
    assert "exact ProductControl RunPlan process realization" in scopes["teleop_avoid"]["excluded_claims"]
    assert scopes["map"]["coverage"] == "component"
    assert "exact-session Gateway SaveMap success" in scopes["map"]["claims"]
    assert "saved-map metadata and format validation" in scopes["map"]["excluded_claims"]
    assert scopes["inspection"]["coverage"] == "component"
    assert "capture:overview route submission and business acknowledgement" in scopes["inspection"]["claims"]
    assert "stable scene-graph person identity selection" in scopes["tracking"]["claims"]
    assert "native navigation goal acceptance or path execution" in scopes["tracking"]["excluded_claims"]
    for product in ("nav", "tracking", "explore"):
        assert scopes[product]["coverage"] == "component"


@pytest.mark.parametrize(
    (
        "coverage",
        "verified",
        "evaluated",
        "ok",
        "lifecycle_verified",
        "stop_verified",
        "cleanup_verified",
        "scope",
        "passed",
    ),
    (
        ("product", True, False, True, True, True, True, "preflight", False),
        ("product", False, True, True, True, True, True, "diagnostic_component", False),
        ("component", True, True, True, False, False, False, "component_e2e", False),
        ("product", True, True, True, False, True, True, "product_lifecycle_incomplete", False),
        ("product", True, True, True, True, False, True, "product_lifecycle_incomplete", False),
        ("product", True, True, True, True, True, False, "product_lifecycle_incomplete", False),
        ("product", True, True, False, True, True, True, "product_e2e", False),
        ("product", True, True, True, True, True, True, "product_e2e", True),
    ),
)
def test_acceptance_evidence_classification_never_overclaims(
    coverage: str,
    verified: bool,
    evaluated: bool,
    ok: bool,
    lifecycle_verified: bool,
    stop_verified: bool,
    cleanup_verified: bool,
    scope: str,
    passed: bool,
) -> None:
    evidence = product_acceptance.classify_evidence(
        {"coverage": coverage},
        run_plan_verified=verified,
        acceptance_evaluated=evaluated,
        ok=ok,
        lifecycle_verified=lifecycle_verified,
        stop_verified=stop_verified,
        cleanup_verified=cleanup_verified,
        rollback_verified=True,
    )

    assert evidence == {
        "acceptance_evaluated": evaluated,
        "evidence_scope": scope,
        "product_acceptance_passed": passed,
    }


def test_product_evidence_lifecycle_gates_default_to_unverified() -> None:
    evidence = product_acceptance.classify_evidence(
        {"coverage": "product"},
        run_plan_verified=True,
        acceptance_evaluated=True,
        ok=True,
    )

    assert evidence == {
        "acceptance_evaluated": True,
        "evidence_scope": "product_lifecycle_incomplete",
        "product_acceptance_passed": False,
    }


@pytest.mark.parametrize(
    ("field", "truthy_value", "scope"),
    (
        ("run_plan_verified", 1, "diagnostic_component"),
        ("acceptance_evaluated", "true", "preflight"),
        ("ok", object(), "product_e2e"),
        ("lifecycle_verified", 1, "product_lifecycle_incomplete"),
        ("stop_verified", "true", "product_lifecycle_incomplete"),
        ("cleanup_verified", object(), "product_lifecycle_incomplete"),
        ("rollback_verified", object(), "product_lifecycle_incomplete"),
    ),
)
def test_product_evidence_truthy_non_boolean_inputs_fail_closed(
    field: str,
    truthy_value: object,
    scope: str,
) -> None:
    inputs: dict[str, object] = {
        "run_plan_verified": True,
        "acceptance_evaluated": True,
        "ok": True,
        "lifecycle_verified": True,
        "stop_verified": True,
        "cleanup_verified": True,
        "rollback_verified": True,
    }
    inputs[field] = truthy_value

    evidence = product_acceptance.classify_evidence(
        {"coverage": "product"},
        **inputs,  # type: ignore[arg-type]
    )

    assert evidence == {
        "acceptance_evaluated": field != "acceptance_evaluated",
        "evidence_scope": scope,
        "product_acceptance_passed": False,
    }


@pytest.mark.parametrize("product", ("teleop", "teleop_avoid"))
def test_run_owns_switch_scenario_stop_and_cleanup_for_component_acceptance(
    product: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / product).resolve()
    state_root.mkdir()
    plan = linux_sim_plan(product)
    monkeypatch.setattr(
        product_acceptance,
        "load_motion_evidence",
        lambda **_kwargs: {
            "commanded_motion_observed": True,
            "nonzero_command_count": 8,
            "nonzero_physics_steps": 8,
            "last_output_sequence": 8,
            "path_length_xy_m": 0.3,
        },
    )
    runner = _Runner(state_root)
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    def resolve(requested: str, *_args: object, **_kwargs: object) -> RunPlan:
        assert requested == product
        return plan

    monkeypatch.setattr(control, "_resolve", resolve)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: not runner.stopped,
    )
    observed: dict[str, object] = {}

    def scenario(exact_plan: RunPlan, plan_path: Path, session_id: str) -> dict:
        observed.update(
            plan=exact_plan,
            plan_path=plan_path,
            session_id=session_id,
            current_exists=(state_root / "current.json").is_file(),
        )
        return {"ok": True, "mode": "scenario", "min_path_length_m": 0.15}

    report = product_acceptance.run(
        control,
        _target(product),
        state_root,
        scenario,
        check=(product_acceptance._check_motion if product == "teleop" else None),
    )

    assert report["ok"] is True, report
    assert report["lifecycle_verified"] is True
    assert report["stop_verified"] is True, report
    assert report["cleanup_verified"] is True
    assert report["evidence_verified"] is True
    assert report["rollback_verified"] is None
    assert report["rollback"] == {
        "ok": None,
        "skipped": True,
        "reason": "component_coverage",
    }
    assert report["runtime_status"]["mujoco_feeder"]["state"] == "stopped"
    assert report["sensor_runtime_evidence"]["ok"] is True
    assert report["sensor_runtime_evidence"]["measurement_scope"] == "feeder_scheduler"
    assert report["sensor_runtime_evidence"]["dds_delivery_verified"] is False
    assert report["sensor_runtime_evidence"]["published_count_scope"] == "endpoint_write"
    assert report["sensor_runtime_evidence"]["window_s"] == 10.0
    assert report["sensor_runtime_evidence"]["min_window_s"] == 3.0
    assert report["sensor_runtime_evidence"]["max_schedule_lateness_ms"] == (1.0 if product == "teleop_avoid" else None)
    assert set(report["sensor_runtime_evidence"]["streams"]) == (
        {"imu", "lidar"} if product == "teleop_avoid" else set()
    )
    assert report["evidence_scope"] == "component_e2e"
    assert report["product_acceptance_passed"] is False
    assert observed["plan"] == plan
    assert observed["current_exists"] is True
    assert runner.calls == ["apply", "stop"]
    assert not (state_root / "current.json").exists()
    assert SimChildLedger(state_root).load() is None


@pytest.mark.parametrize(
    ("case", "expected"),
    (
        ("running", "terminal state"),
        ("extra_stream", "stream set"),
        ("missing_stream", "stream set"),
        ("expected_hz", "expected_hz"),
        ("slow", "actual_hz"),
        ("drops", "drop rate"),
    ),
)
def test_sensor_runtime_evidence_rejects_bad_terminal_status(
    case: str,
    expected: str,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    status = _feeder_status_payload(plan)
    streams = cast(dict[str, dict[str, object]], status["streams"])
    if case == "running":
        status["state"] = "running"
    elif case == "extra_stream":
        streams["camera_rgbd"] = dict(streams["lidar"])
    elif case == "missing_stream":
        streams.pop("imu")
    elif case == "expected_hz":
        streams["imu"]["expected_hz"] = 199.0
    elif case == "slow":
        streams["lidar"]["actual_hz"] = 8.0
    else:
        streams["lidar"].update(
            scheduled_count=100,
            published_count=98,
            dropped_count=2,
            actual_hz=9.8,
        )

    with pytest.raises(RuntimeError, match=expected):
        product_acceptance._sensor_runtime_evidence(plan, status)


def test_truth_localized_product_records_but_does_not_gate_on_imu_rate(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    status = _feeder_status_payload(plan)
    streams = cast(dict[str, dict[str, object]], status["streams"])
    streams["imu"].update(
        scheduled_count=2000,
        published_count=1250,
        dropped_count=750,
        actual_hz=125.0,
    )

    evidence = product_acceptance._sensor_runtime_evidence(plan, status)

    assert evidence["localization_authority"] == "mujoco_truth"
    assert evidence["streams"]["imu"]["required_for_gate"] is False
    assert evidence["streams"]["imu"]["within_limits"] is False
    assert evidence["streams"]["imu"]["gate_passed"] is True
    assert evidence["streams"]["lidar"]["required_for_gate"] is True


def test_sensor_runtime_evidence_rejects_a_short_observation_window(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    status = _feeder_status_payload(plan)
    status["window_s"] = 2.99

    with pytest.raises(RuntimeError, match="insufficient observation"):
        product_acceptance._sensor_runtime_evidence(plan, status)


def test_sensor_runtime_status_load_is_bound_to_the_exact_launch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    product_session_id = "c" * 32
    feeder = next(process for process in plan.processes if process.name == "mujoco_feeder")
    child = SimpleNamespace(
        target=feeder.target,
        launch_id="d" * 64,
        started_wall_ns=1,
    )
    status = _feeder_status_payload(plan, product_session_id=product_session_id)
    observed: dict[str, object] = {}

    def load(**kwargs: object) -> dict[str, object]:
        observed.update(kwargs)
        return status

    monkeypatch.setattr(product_acceptance, "load_feeder_status", load)

    raw, checked = product_acceptance._load_sensor_runtime(
        tmp_path,
        plan,
        (child,),
        product_session_id=product_session_id,
    )

    assert raw == status
    assert checked["ok"] is True
    assert observed == {
        "session_root": tmp_path,
        "product": plan.product,
        "product_session_id": product_session_id,
        "process": "mujoco_feeder",
        "launch_id": "d" * 64,
    }


def test_sensor_runtime_status_load_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    feeder = next(process for process in plan.processes if process.name == "mujoco_feeder")
    child = SimpleNamespace(
        target=feeder.target,
        launch_id="e" * 64,
        started_wall_ns=1,
    )

    def reject(**_kwargs: object) -> dict[str, object]:
        raise SimFeederStatusError("missing or wrong identity")

    monkeypatch.setattr(product_acceptance, "load_feeder_status", reject)

    with pytest.raises(RuntimeError, match="terminal status is invalid"):
        product_acceptance._load_sensor_runtime(
            tmp_path,
            plan,
            (child,),
            product_session_id="f" * 32,
        )


def test_sensor_runtime_status_rejects_a_prelaunch_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop_avoid")
    feeder = next(process for process in plan.processes if process.name == "mujoco_feeder")
    child = SimpleNamespace(
        target=feeder.target,
        launch_id="1" * 64,
        started_wall_ns=2,
    )
    status = _feeder_status_payload(plan)
    status["updated_wall_ns"] = 1
    monkeypatch.setattr(product_acceptance, "load_feeder_status", lambda **_kwargs: status)

    with pytest.raises(RuntimeError, match="predates its launch"):
        product_acceptance._load_sensor_runtime(
            tmp_path,
            plan,
            (child,),
            product_session_id="a" * 32,
        )


def test_rollback_probe_never_stops_an_unowned_replacement(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    root = (tmp_path / "rollback-guard").resolve()
    root.mkdir()
    plan = linux_sim_plan("teleop")
    control, runner = _control(plan, root, monkeypatch)
    monkeypatch.setattr(ProcessIdentity, "matches", lambda _identity: not runner.stopped)
    original_switch = control.switch
    calls = 0

    def switch_then_fail(*args: object, **kwargs: object):
        nonlocal calls
        report = original_switch(*args, **kwargs)
        calls += 1
        if calls == 2:
            raise RuntimeError("replacement committed before failure")
        return report

    monkeypatch.setattr(control, "switch", switch_then_fail)

    report = product_acceptance.check_rollback(control, root, "teleop")

    assert report["ok"] is False
    assert report["probe_cleanup_verified"] is False
    assert "compare-and-stop identity" in report["cleanup_error"]
    assert runner.calls == ["apply", "quiesce", "apply"]
    assert (root / "current.json").is_file()


def test_run_rejects_a_dirty_rollback_root_before_switch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state_root = tmp_path / "state"
    rollback_root = tmp_path / "rollback"
    rollback_root.mkdir()
    (rollback_root / "current.json").write_text("{}", encoding="utf-8")
    switched = False

    class Control:
        def switch(self, *_args: object, **_kwargs: object) -> None:
            nonlocal switched
            switched = True

    control = cast(ProductControl, Control())
    monkeypatch.setattr(
        product_acceptance,
        "acceptance_scope",
        lambda _target: {"coverage": "product"},
    )
    with pytest.raises(ValueError, match="empty state_root"):
        product_acceptance.run(
            control,
            _target("teleop"),
            state_root,
            lambda *_args: {"ok": True},
            rollback_control=control,
            rollback_root=rollback_root,
        )

    assert switched is False


@pytest.mark.parametrize("invalid", ("scenario", "check"))
def test_run_rejects_invalid_callback_results(
    invalid: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / "state").resolve()
    rollback_root = (tmp_path / "rollback").resolve()
    state_root.mkdir()
    plan = linux_sim_plan("teleop")
    control, runner = _control(plan, state_root, monkeypatch)
    rollback_control, rollback_runner = _rollback(plan, rollback_root, monkeypatch)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: _live(runner, rollback_runner),
    )

    def scenario(*_args: object) -> Any:
        return None if invalid == "scenario" else {"ok": True}

    def check(*_args: object) -> Any:
        return None if invalid == "check" else {}

    report = product_acceptance.run(
        control,
        _target("teleop"),
        state_root,
        scenario,
        rollback_control=rollback_control,
        rollback_root=rollback_root,
        check=check,
    )

    assert report["ok"] is False
    assert any(f"{invalid} must return a mapping" in blocker for blocker in report["blockers"])
    assert runner.calls == ["apply", "stop"]


def test_exact_teleop_coordinator_stops_when_attach_only_scenario_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / "scenario-failure").resolve()
    state_root.mkdir()
    plan = linux_sim_plan("teleop")
    monkeypatch.setattr(product_acceptance, "load_motion_evidence", lambda **_kwargs: {})
    runner = _Runner(state_root)
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    rollback_root = (tmp_path / "rollback").resolve()
    rollback_control, rollback_runner = _rollback(plan, rollback_root, monkeypatch)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: _live(runner, rollback_runner),
    )

    def fail_scenario(_plan: RunPlan, _path: Path, _session_id: str) -> dict:
        raise RuntimeError("injected scenario failure")

    report = product_acceptance.run(
        control,
        _target("teleop"),
        state_root,
        fail_scenario,
        rollback_control=rollback_control,
        rollback_root=rollback_root,
        check=product_acceptance._check_motion,
    )

    assert report["ok"] is False
    assert report["evidence_verified"] is False
    assert report["rollback_verified"] is None
    assert report["rollback"]["reason"] == "component_coverage"
    assert report["stop_verified"] is True
    assert report["cleanup_verified"] is True
    assert runner.calls == ["apply", "stop"]
    assert rollback_runner.calls == []
    assert not (state_root / "current.json").exists()


def test_exact_teleop_coordinator_does_not_overclaim_incomplete_readiness(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / "r").resolve()
    state_root.mkdir()
    plan = linux_sim_plan("teleop")
    monkeypatch.setattr(product_acceptance, "load_motion_evidence", lambda **_kwargs: {})
    runner = _Runner(state_root, incomplete_ready=True)
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    rollback_root = (tmp_path / "rollback").resolve()
    rollback_control, rollback_runner = _rollback(plan, rollback_root, monkeypatch)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: _live(runner, rollback_runner),
    )

    report = product_acceptance.run(
        control,
        _target("teleop"),
        state_root,
        lambda *_args: {"ok": True},
        rollback_control=rollback_control,
        rollback_root=rollback_root,
        check=product_acceptance._check_motion,
    )

    assert report["ok"] is False
    assert report["lifecycle_verified"] is False
    assert report["stop_verified"] is True, report
    assert report["cleanup_verified"] is True
    assert runner.calls == ["apply", "stop"]


def test_exact_teleop_coordinator_never_stops_replacement_same_product_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / "replace").resolve()
    state_root.mkdir()
    plan = linux_sim_plan("teleop")
    runner = _Runner(state_root)
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    rollback_root = (tmp_path / "rollback").resolve()
    rollback_control, rollback_runner = _rollback(plan, rollback_root, monkeypatch)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: _live(runner, rollback_runner),
    )

    def replace_session(*_args) -> dict:
        replacement = control.switch("teleop", state_dir=state_root)
        return {"ok": True, "replacement": replacement, "min_path_length_m": 0.15}

    report = product_acceptance.run(
        control,
        _target("teleop"),
        state_root,
        replace_session,
        rollback_control=rollback_control,
        rollback_root=rollback_root,
        check=product_acceptance._check_motion,
    )

    assert report["ok"] is False
    assert any("current Product session changed" in blocker for blocker in report["blockers"])
    assert any("compare-and-stop identity" in blocker for blocker in report["blockers"])
    assert runner.calls == ["apply", "quiesce", "apply"]
    current = json.loads((state_root / "current.json").read_text(encoding="utf-8"))
    assert current["product_session_id"] != report["lifecycle"]["ledger"]["product_session_id"]


def test_exact_teleop_rejects_session_replaced_before_switch_returns(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    state_root = (tmp_path / "x").resolve()
    state_root.mkdir()
    plan = linux_sim_plan("teleop")
    runner = _Runner(state_root)
    control = ProductControl(
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    rollback_root = (tmp_path / "rollback").resolve()
    rollback_control, rollback_runner = _rollback(plan, rollback_root, monkeypatch)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: _live(runner, rollback_runner),
    )
    original_switch = control.switch
    replacement_session_id = ""
    first_call = True

    def switch_then_replace(*args, **kwargs):
        nonlocal first_call, replacement_session_id
        original_report = original_switch(*args, **kwargs)
        if first_call:
            first_call = False
            replacement_session_id = "f" * 32
            original = product_acceptance._load_committed_plan(state_root, {})
            assert original is not None
            exact_path = plan.write(state_root / f"plan-{replacement_session_id}.json")
            SimChildLedger(state_root).clear()
            runner.apply(exact_path, product_session_id=replacement_session_id)
            sim_switch._commit_current(
                exact_path,
                plan,
                {},
                state_root,
                product_session_id=replacement_session_id,
            )
        return original_report

    monkeypatch.setattr(control, "switch", switch_then_replace)
    scenario_called = False

    def scenario(*_args):
        nonlocal scenario_called
        scenario_called = True
        return {"ok": True, "min_path_length_m": 0.15}

    report = product_acceptance.run(
        control,
        _target("teleop"),
        state_root,
        scenario,
        rollback_control=rollback_control,
        rollback_root=rollback_root,
        check=product_acceptance._check_motion,
    )
    assert report["ok"] is False
    assert scenario_called is False
    assert any("current identity does not match" in value for value in report["blockers"]), report
    assert any("compare-and-stop identity" in value for value in report["blockers"])
    assert runner.calls == ["apply", "apply"]
    current = json.loads((state_root / "current.json").read_text(encoding="utf-8"))
    assert current["product_session_id"] == replacement_session_id


def test_exact_teleop_physical_evidence_requires_applied_output_and_threshold(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = SimpleNamespace(product="teleop")
    child = SimpleNamespace(target="mujoco_feeder", launch_id="b" * 64)
    evidence = {
        "commanded_motion_observed": True,
        "nonzero_command_count": 4,
        "nonzero_physics_steps": 4,
        "last_output_sequence": 7,
        "path_length_xy_m": 0.3,
    }
    monkeypatch.setattr(product_acceptance, "load_motion_evidence", lambda **_kwargs: evidence)

    assert (
        product_acceptance._check_motion(
            tmp_path,
            plan,
            [child],
            {"min_path_length_m": 0.15},
            PRODUCT_SESSION_ID,
        )
        == evidence
    )

    evidence["last_output_sequence"] = 0
    evidence["path_length_xy_m"] = 0.1
    with pytest.raises(RuntimeError, match="APPLIED ACK or path evidence"):
        product_acceptance._check_motion(
            tmp_path,
            plan,
            [child],
            {"min_path_length_m": 0.15},
            PRODUCT_SESSION_ID,
        )
    for invalid in (float("nan"), float("inf"), 0.0, -0.1):
        with pytest.raises(RuntimeError, match="threshold is missing"):
            product_acceptance._check_motion(
                tmp_path,
                plan,
                [child],
                {"min_path_length_m": invalid},
                PRODUCT_SESSION_ID,
            )


def test_exact_teleop_avoid_physical_evidence_requires_clear_detour_and_corridor_restoration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = tmp_path / "teleop_avoid.xml"
    world.write_text(
        """<mujoco><worldbody>
<geom name="detour_obstacle" type="box" pos="2.0 0 0.65" size="0.45 0.55 0.65"/>
</worldbody></mujoco>\n""",
        encoding="utf-8",
    )
    plan = SimpleNamespace(
        product="teleop_avoid",
        simulation={"physics_plan": {"world": {"mjcf": str(world)}}},
    )
    child = SimpleNamespace(target="mujoco_feeder", launch_id="b" * 64)
    evidence = {
        "commanded_motion_observed": True,
        "nonzero_command_count": 40,
        "nonzero_physics_steps": 400,
        "last_output_sequence": 40,
        "path_length_xy_m": 5.2,
        "start_position_m": [0.0, 0.0, 0.4],
        "end_position_m": [4.0, 0.1, 0.4],
        "start_yaw_rad": 0.0,
        "end_yaw_rad": 0.05,
        "trajectory": [
            [0.0, 0.0, 0.0, 0.4, 0.0],
            [100.0, 0.8, 1.0, 0.4, 0.02],
            [200.0, 2.5, 1.0, 0.4, 0.03],
            [300.0, 3.2, 1.0, 0.4, 0.04],
            [400.0, 4.0, 0.1, 0.4, 0.05],
        ],
    }
    monkeypatch.setattr(product_acceptance, "load_motion_evidence", lambda **_kwargs: evidence)
    scenario = {
        "command": {"vx": 0.5, "vy": 0.0, "wz": 0.0},
        "physical_acceptance": {
            "obstacle_geom": "detour_obstacle",
            "minimum_forward_progress_m": 3.2,
            "minimum_lateral_detour_m": 0.7,
            "minimum_clearance_m": 0.08,
            "maximum_restored_corridor_offset_m": 0.25,
            "maximum_restored_heading_error_rad": 0.35,
            "minimum_restoration_m": 0.35,
            "minimum_pass_margin_m": 0.2,
        },
        "robot_geometry": {
            "vehicle_length_m": 1.0,
            "vehicle_width_m": 0.6,
        },
    }

    checked = product_acceptance._check_teleop_avoid_motion(
        tmp_path,
        plan,
        [child],
        scenario,
        PRODUCT_SESSION_ID,
    )

    assert checked["metrics"]["forward_progress_m"] == pytest.approx(4.0)
    assert checked["metrics"]["max_lateral_detour_m"] == pytest.approx(1.0)
    assert checked["metrics"]["restored_corridor_offset_m"] == pytest.approx(0.1)
    assert checked["metrics"]["minimum_obstacle_clearance_m"] >= 0.08
    assert checked["metrics"]["obstacle_pass_margin_m"] >= 0.2
    assert checked["obstacle"]["geom"] == "detour_obstacle"

    evidence["end_position_m"] = [4.0, 0.8, 0.4]
    evidence["trajectory"][-1] = [400.0, 4.0, 0.8, 0.4, 0.05]
    with pytest.raises(RuntimeError, match="corridor_restoration"):
        product_acceptance._check_teleop_avoid_motion(
            tmp_path,
            plan,
            [child],
            scenario,
            PRODUCT_SESSION_ID,
        )

    evidence["end_position_m"] = [4.0, 0.1, 0.4]
    evidence["trajectory"][-1] = [400.0, 4.0, 0.1, 0.4, 0.05]
    evidence["trajectory"][2] = [200.0, 2.0, 0.0, 0.4, 0.03]
    with pytest.raises(RuntimeError, match="obstacle_clearance"):
        product_acceptance._check_teleop_avoid_motion(
            tmp_path,
            plan,
            [child],
            scenario,
            PRODUCT_SESSION_ID,
        )

    evidence["trajectory"][2] = [200.0, 2.5, 1.0, 0.4, 0.03]
    scenario["physical_acceptance"]["minimum_pass_margin_m"] = 1.1
    with pytest.raises(RuntimeError, match="obstacle_pass_margin"):
        product_acceptance._check_teleop_avoid_motion(
            tmp_path,
            plan,
            [child],
            scenario,
            PRODUCT_SESSION_ID,
        )


def test_exact_teleop_readiness_rejects_wrong_typed_identity(
    linux_sim_plan: Callable[..., RunPlan],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = linux_sim_plan("teleop")

    def ready() -> dict[str, dict[str, object]]:
        result = {}
        for process in plan.processes:
            expectation = product_acceptance.readiness_expectation_for_process(
                process.name,
                process.command.readiness.target,
            )
            assert expectation is not None
            result[process.name] = {
                "kind": "file",
                "target": process.target,
                "adapter": expectation.adapter,
                "payload": {
                    "adapter": expectation.adapter,
                    "product_session_id": PRODUCT_SESSION_ID,
                    "product": plan.product,
                    "process": process.name,
                    "ready": True,
                },
            }
        return result

    product_acceptance._check_ready(plan, ready(), PRODUCT_SESSION_ID)
    for field, value in (
        ("product_session_id", "b" * 32),
        ("product", "nav"),
        ("process", "other"),
        ("ready", False),
    ):
        invalid = ready()
        invalid["nav_runtime"]["payload"][field] = value  # type: ignore[index]
        with pytest.raises(RuntimeError, match="typed readiness"):
            product_acceptance._check_ready(
                plan,
                invalid,
                PRODUCT_SESSION_ID,
            )
    missing = ready()
    del missing["nav_runtime"]["payload"]
    with pytest.raises(RuntimeError, match="file readiness"):
        product_acceptance._check_ready(plan, missing, PRODUCT_SESSION_ID)


def test_map_preflight_report_preserves_the_component_claim_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        map_native_acceptance,
        "LingTuClient",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("nonstrict attach-only rejection must not call Gateway")
        ),
    )

    assert map_native_acceptance.main(["--preflight-only"]) == 2


def test_map_runner_accepts_dispatcher_strict_argument() -> None:
    args = map_native_acceptance._parser().parse_args(["--strict"])

    assert args.strict is True


def test_map_strict_runner_attaches_to_current_and_uses_gateway_save_api(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("map")
    product_session_id = "a" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=product_session_id,
    )
    current_path = tmp_path / "current.json"
    current_before = current_path.read_bytes()
    current_mtime_ns = current_path.stat().st_mtime_ns
    report_path = tmp_path / "strict-map-report.json"
    observed: dict[str, object] = {}
    feeder_started_ns = current_mtime_ns - 1_000_000
    _write_exact_feeder_ledger(
        tmp_path,
        plan,
        product_session_id=product_session_id,
        started_wall_ns=feeder_started_ns,
    )
    _write_camera_shm(
        tmp_path,
        timestamp_ns=time.time_ns(),
        info_timestamp_ns=current_mtime_ns - 1,
    )

    def load_readiness(path: Path, **kwargs: object) -> dict[str, object]:
        observed.setdefault("readiness", []).append((path, kwargs))  # type: ignore[attr-defined]
        return {
            "adapter": "mapd_status",
            "product_session_id": product_session_id,
            "product": "map",
            "process": "map_runtime",
            "ready": True,
            "details": {
                "live": True,
                "reset_epoch": 1,
                "observation_sequence": 4,
                "processed_observations": 4,
            },
        }

    class GatewayClient:
        def __init__(self, host: str, port: int) -> None:
            observed["gateway"] = (host, port)

        def save_map_and_wait(self, name: str, **kwargs: object) -> dict[str, object]:
            observed["save"] = (name, kwargs)
            _write_camera_shm(
                tmp_path,
                timestamp_ns=time.time_ns() + 1_000_000,
                include=("color", "depth"),
                repeats=2,
            )
            return {"operation": {"state": "SUCCEEDED"}}

        def maps(self) -> SimpleNamespace:
            name = str(observed["save"][0])  # type: ignore[index]
            return SimpleNamespace(
                maps=[
                    SimpleNamespace(
                        name=name,
                        has_pcd=True,
                        size_bytes=128,
                        raw={"size_mb": 0.001},
                    )
                ]
            )

    monkeypatch.delenv("LINGTU_CURRENT_FILE", raising=False)
    monkeypatch.setattr(map_native_acceptance, "load_typed_readiness", load_readiness)
    monkeypatch.setattr(map_native_acceptance, "LingTuClient", GatewayClient)

    assert (
        map_native_acceptance.main(
            [
                "--run-plan",
                str(plan_path),
                "--manifest",
                str(map_native_acceptance.DEFAULT_MANIFEST),
                "--strict",
                "--json-out",
                str(report_path),
            ]
        )
        == 0
    )

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["attached_identity"] == {
        "run_plan": str(plan_path.resolve()),
        "product_session_id": product_session_id,
    }
    assert report["map_save"] == {
        "name": f"mujoco-map-{product_session_id}",
        "operation_state": "SUCCEEDED",
        "has_pcd": True,
        "size_bytes": 128,
        "size_mb": 0.001,
    }
    assert report["camera"]["after"]["ready"] is True
    assert set(report["children"]["before_save"]) == set(map_native_acceptance._ATTACHED_PROCESSES)
    assert report["children"]["before_save"] == report["children"]["after_save"]
    assert all(
        set(item["process_identity"]) == {"pid", "platform", "start_identity"}
        for item in report["children"]["after_save"].values()
    )
    assert set(report["camera"]["after"]["streams"]) == {"color", "depth", "info"}
    assert report["camera"]["after"]["streams"]["info"]["timestamp_ns"] < current_mtime_ns
    assert report["evidence_scope"] == "component_e2e"
    assert report["product_acceptance_passed"] is False
    assert observed["gateway"] == ("127.0.0.1", 5050)
    readiness = cast(list[tuple[Path, dict[str, object]]], observed["readiness"])
    assert {path.name for path, _kwargs in readiness} == {
        "lidar.ready.json",
        "imu.ready.json",
        "camera.ready.json",
        "slam.status.json",
        "mapd.status.json",
    }
    assert all(kwargs["product_session_id"] == product_session_id for _path, kwargs in readiness)
    assert current_path.read_bytes() == current_before


@pytest.mark.parametrize(
    ("has_pcd", "size_bytes", "raw", "expected_blocker"),
    (
        (False, 0, {}, "Gateway map list does not expose the saved PCD"),
        (True, 0, {"size_mb": 0.0}, "Gateway map list exposes an empty saved PCD"),
    ),
)
def test_map_strict_runner_fails_when_saved_map_pcd_is_missing_or_empty(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
    has_pcd: bool,
    size_bytes: int,
    raw: dict[str, object],
    expected_blocker: str,
) -> None:
    plan = linux_sim_plan("map")
    product_session_id = "b" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=product_session_id,
    )
    report_path = tmp_path / "strict-map-failure.json"
    monkeypatch.setattr(
        map_native_acceptance,
        "_attached_children",
        lambda *_args, **_kwargs: {
            name: SimpleNamespace(started_wall_ns=1) for name in map_native_acceptance._ATTACHED_PROCESSES
        },
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "_revalidate_children",
        lambda *_args, **_kwargs: {"validated": True},
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "_readiness",
        lambda *_args, **_kwargs: {"map_runtime": {"details": {"live": True}}},
    )
    camera_sequence = 0

    def camera(*_args: object, **_kwargs: object) -> dict[str, object]:
        nonlocal camera_sequence
        camera_sequence += 1
        return {
            "ready": True,
            "streams": {
                name: {"sequence": camera_sequence, "timestamp_ns": time.time_ns() + camera_sequence}
                for name in ("color", "depth", "info")
            },
        }

    monkeypatch.setattr(
        map_native_acceptance,
        "_camera",
        camera,
    )
    monkeypatch.delenv("LINGTU_CURRENT_FILE", raising=False)

    class GatewayClient:
        def __init__(self, _host: str, _port: int) -> None:
            pass

        def save_map_and_wait(self, _name: str, **_kwargs: object) -> dict[str, object]:
            return {"operation": {"state": "SUCCEEDED"}}

        def maps(self) -> SimpleNamespace:
            return SimpleNamespace(
                maps=[
                    SimpleNamespace(
                        name=f"mujoco-map-{product_session_id}",
                        has_pcd=has_pcd,
                        size_bytes=size_bytes,
                        raw=raw,
                    )
                ]
            )

    monkeypatch.setattr(map_native_acceptance, "LingTuClient", GatewayClient)

    assert (
        map_native_acceptance.main(
            [
                "--run-plan",
                str(plan_path),
                "--strict",
                "--json-out",
                str(report_path),
            ]
        )
        == 1
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert report["product_acceptance_passed"] is False
    assert report["blockers"] == [expected_blocker]


def test_map_strict_runner_rejects_current_session_change_during_gateway_save(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("map")
    original_session_id = "c" * 32
    plan_path = plan.write(tmp_path / f"plan-{original_session_id}.json")
    replacement_session_id = "d" * 32
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=original_session_id,
    )
    report_path = tmp_path / "strict-map-session-change.json"
    monkeypatch.setattr(
        map_native_acceptance,
        "_attached_children",
        lambda *_args, **_kwargs: {
            name: SimpleNamespace(started_wall_ns=1) for name in map_native_acceptance._ATTACHED_PROCESSES
        },
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "_revalidate_children",
        lambda *_args, **_kwargs: {"validated": True},
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "_readiness",
        lambda *_args, **_kwargs: {"map_runtime": {"details": {"live": True}}},
    )
    camera_sequence = 0

    def camera(*_args: object, **_kwargs: object) -> dict[str, object]:
        nonlocal camera_sequence
        camera_sequence += 1
        return {
            "ready": True,
            "streams": {
                name: {"sequence": camera_sequence, "timestamp_ns": time.time_ns() + camera_sequence}
                for name in ("color", "depth", "info")
            },
        }

    monkeypatch.setattr(
        map_native_acceptance,
        "_camera",
        camera,
    )
    monkeypatch.delenv("LINGTU_CURRENT_FILE", raising=False)

    class GatewayClient:
        def __init__(self, _host: str, _port: int) -> None:
            pass

        def save_map_and_wait(self, _name: str, **_kwargs: object) -> dict[str, object]:
            return {"operation": {"state": "SUCCEEDED"}}

        def maps(self) -> SimpleNamespace:
            sim_switch._commit_current(
                plan_path,
                plan,
                {},
                tmp_path,
                product_session_id=replacement_session_id,
            )
            return SimpleNamespace(
                maps=[
                    SimpleNamespace(
                        name=f"mujoco-map-{original_session_id}",
                        has_pcd=True,
                        size_bytes=128,
                        raw={"size_mb": 0.001},
                    )
                ]
            )

    monkeypatch.setattr(map_native_acceptance, "LingTuClient", GatewayClient)

    assert (
        map_native_acceptance.main(
            [
                "--run-plan",
                str(plan_path),
                "--strict",
                "--json-out",
                str(report_path),
            ]
        )
        == 1
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert report["product_acceptance_passed"] is False
    assert report["blockers"] == ["current RunPlan path must match its Product session"]


@pytest.mark.parametrize(
    ("include", "timestamp_offset_ns", "expected_blocker"),
    (
        (("color", "info"), 0, "camera SHM is missing"),
        (("color", "depth", "info"), -2_000_000_000, "camera SHM frame exceeded"),
    ),
)
def test_map_strict_runner_rejects_missing_or_stale_camera_at_save_time(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
    include: tuple[str, ...],
    timestamp_offset_ns: int,
    expected_blocker: str,
) -> None:
    plan = linux_sim_plan("map")
    product_session_id = "e" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=product_session_id,
    )
    feeder_started_ns = time.time_ns() - 3_000_000_000
    _write_exact_feeder_ledger(
        tmp_path,
        plan,
        product_session_id=product_session_id,
        started_wall_ns=feeder_started_ns,
    )
    _write_camera_shm(
        tmp_path,
        timestamp_ns=time.time_ns() + timestamp_offset_ns,
        include=include,
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "load_typed_readiness",
        lambda *_args, **_kwargs: {
            "details": {
                "live": True,
                "reset_epoch": 1,
                "observation_sequence": 1,
                "processed_observations": 1,
            }
        },
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "LingTuClient",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("camera gate must run before Gateway SaveMap")),
    )
    report_path = tmp_path / "strict-map-camera-failure.json"

    assert (
        map_native_acceptance.main(
            [
                "--run-plan",
                str(plan_path),
                "--strict",
                "--json-out",
                str(report_path),
            ]
        )
        == 1
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert report["product_acceptance_passed"] is False
    assert expected_blocker in report["blockers"][0]


@pytest.mark.parametrize("failure", ("missing", "foreign", "dead"))
def test_map_attached_children_fail_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure: str,
) -> None:
    identity = ProcessIdentity.current()
    plan = SimpleNamespace(
        processes=tuple(SimpleNamespace(name=name, target=name) for name in map_native_acceptance._ATTACHED_PROCESSES),
    )
    records = tuple(
        SimChildRecord(
            target=name,
            process_identity=identity,
            process_group=identity.pid,
            started_wall_ns=1,
            launch_id=f"launch-{name}",
        )
        for name in map_native_acceptance._ATTACHED_PROCESSES
        if not (failure == "missing" and name == "map_runtime")
    )
    expected_session_id = "c" * 32
    snapshot_session_id = "b" * 32 if failure == "foreign" else expected_session_id
    snapshot = SimChildSnapshot.create(
        product_session_id=snapshot_session_id,
        children=records,
    )
    monkeypatch.setattr(SimChildLedger, "load", lambda _self: snapshot)
    if failure == "dead":
        monkeypatch.setattr(
            ProcessIdentity,
            "current",
            lambda *_args: (_ for _ in ()).throw(ProcessIdentityError("dead")),
        )
    with pytest.raises(RuntimeError, match=r"ledger identity|child is missing|child is not live"):
        map_native_acceptance._attached_children(
            plan,
            tmp_path,
            expected_session_id,
        )


def test_map_child_identity_is_revalidated_after_save(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    identity = ProcessIdentity.current()
    record = SimChildRecord(
        target="map_runtime",
        process_identity=identity,
        process_group=identity.pid,
        started_wall_ns=1,
        launch_id="a" * 64,
    )
    observed = iter((identity, object()))
    monkeypatch.setattr(ProcessIdentity, "current", lambda *_args: next(observed))

    assert map_native_acceptance._revalidate_children({"map_runtime": record})
    with pytest.raises(RuntimeError, match="identity changed"):
        map_native_acceptance._revalidate_children({"map_runtime": record})


def test_map_camera_rejects_frozen_color_or_depth() -> None:
    frozen = {
        "streams": {
            "color": {"sequence": 4, "timestamp_ns": 20},
            "depth": {"sequence": 4, "timestamp_ns": 20},
            "info": {"sequence": 1, "timestamp_ns": 1},
        }
    }
    with pytest.raises(RuntimeError, match="did not advance"):
        map_native_acceptance._require_camera_advanced(frozen, frozen, 10)


@pytest.mark.parametrize(
    "missing",
    ("lidar_publisher", "imu_publisher", "camera_publisher", "slam_runtime"),
)
def test_map_typed_readiness_requires_sensor_and_slam(
    monkeypatch: pytest.MonkeyPatch,
    missing: str,
) -> None:
    targets = {
        "lidar_publisher": "lidar.ready.json",
        "imu_publisher": "imu.ready.json",
        "camera_publisher": "camera.ready.json",
        "slam_runtime": "slam.status.json",
        "map_runtime": "mapd.status.json",
    }
    plan = SimpleNamespace(
        processes=tuple(
            SimpleNamespace(
                name=name,
                command=SimpleNamespace(readiness=SimpleNamespace(kind="file", target=targets[name])),
            )
            for name in (
                "lidar_publisher",
                "imu_publisher",
                "camera_publisher",
                "slam_runtime",
                "map_runtime",
            )
            if name != missing
        ),
    )
    monkeypatch.setattr(
        map_native_acceptance,
        "load_typed_readiness",
        lambda *_args, **_kwargs: {
            "details": {
                "live": True,
                "reset_epoch": 1,
                "observation_sequence": 1,
                "processed_observations": 1,
            }
        },
    )
    with pytest.raises(RuntimeError, match=missing):
        map_native_acceptance._readiness(
            plan,
            Path("."),
            {
                name: SimpleNamespace(started_wall_ns=1)
                for name in (
                    "lidar_publisher",
                    "imu_publisher",
                    "camera_publisher",
                    "slam_runtime",
                    "map_runtime",
                )
            },
            "b" * 32,
        )


def test_tracking_gate_is_attach_only_and_preflight_does_not_call_gateway(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("tracking")
    product_session_id = "e" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=product_session_id,
    )
    monkeypatch.setattr(
        tracking_native_acceptance,
        "LingTuClient",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("preflight must not call Gateway")
        ),
    )

    assert tracking_native_acceptance.main(
        ["--run-plan", str(plan_path), "--strict", "--preflight-only"]
    ) == 0
    source = Path(tracking_native_acceptance.__file__).read_text(encoding="utf-8")
    assert "native.main" not in source
    assert "Popen" not in source


def test_tracking_attached_runner_rejects_identity_before_http(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(
        tracking_native_acceptance,
        "_current_identity",
        lambda *_args: (_ for _ in ()).throw(ValueError("identity mismatch")),
    )
    monkeypatch.setattr(
        tracking_native_acceptance,
        "_client",
        lambda *_args: (_ for _ in ()).throw(AssertionError("HTTP must not run")),
    )

    with pytest.raises(ValueError, match="identity mismatch"):
        tracking_native_acceptance._run_attached(
            SimpleNamespace(),
            tmp_path / "plan.json",
            {"thresholds": {}},
            1.0,
        )


def test_tracking_attached_runner_follows_moving_person_then_cancels_native_goal(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    product_session_id = "e" * 32

    class Gateway:
        def __init__(self) -> None:
            self.scene_reads = 0
            self.state_reads = 0
            self.posts: list[tuple[str, dict[str, Any]]] = []

        def _get(self, path: str) -> dict[str, Any]:
            if path == "/ready":
                return {
                    "product_contract": {
                        "product": "tracking",
                        "product_session_id": product_session_id,
                    }
                }
            if path == "/api/v1/runtime/dataflow":
                return {
                    "authoritative": True,
                    "available": True,
                    "run_plan": {"identity": {"product": "tracking", "env": "sim"}},
                }
            if path == "/api/v1/scene_graph":
                self.scene_reads += 1
                return {
                    "objects": [
                        {"id": "track_person_01", "label": "person"},
                        {"id": "crate-1", "label": "crate"},
                    ]
                }
            if path == "/api/v1/state":
                self.state_reads += 1
                if self.state_reads == 1:
                    return {
                        "visual_servo": {
                            "mode": "follow",
                            "state": "following",
                            "frame_id": "map",
                            "target_id": "track_person_01",
                            "target_visible": True,
                            "navigation_state": "accepted",
                            "navigation_task_id": "task-1",
                            "robot_position": [3.0, 4.0, 0.0],
                            "goal_position": [4.5, 4.0, 0.0],
                            "person": {"id": "track_person_01", "position": [6.0, 4.0, 0.0]},
                        }
                    }
                if self.state_reads == 2:
                    return {
                        "visual_servo": {
                            "mode": "follow",
                            "state": "following",
                            "frame_id": "map",
                            "target_id": "track_person_01",
                            "target_visible": True,
                            "navigation_state": "planning",
                            "navigation_task_id": "task-1",
                            "robot_position": [3.05, 4.0, 0.0],
                            "goal_position": [4.6, 4.0, 0.0],
                            "person": {"id": "track_person_01", "position": [6.1, 4.0, 0.0]},
                        }
                    }
                if self.state_reads == 3:
                    return {
                        "visual_servo": {
                            "mode": "follow",
                            "state": "following",
                            "frame_id": "map",
                            "target_id": "track_person_01",
                            "target_visible": True,
                            "navigation_state": "path_active",
                            "navigation_task_id": "task-2",
                            "robot_position": [3.3, 4.0, 0.0],
                            "goal_position": [4.8, 4.0, 0.0],
                            "person": {"id": "track_person_01", "position": [6.3, 4.0, 0.0]},
                        }
                    }
                return {
                    "visual_servo": {
                        "mode": "idle",
                        "state": "idle",
                        "target_id": None,
                        "target_visible": False,
                        "navigation_state": "cancelled",
                        "person": {"id": None},
                    }
                }
            raise AssertionError(path)

        def _post(self, path: str, body: dict[str, Any]) -> dict[str, Any]:
            self.posts.append((path, body))
            return {
                "ok": True,
                "command": {"name": "visual_servo", "accepted": True},
            }

    gateway = Gateway()
    plan = SimpleNamespace(host_config={"gateway_port": 5050})
    monkeypatch.setattr(
        tracking_native_acceptance,
        "_current_identity",
        lambda *_args: (tmp_path, product_session_id),
    )
    monkeypatch.setattr(tracking_native_acceptance, "_client", lambda *_args: gateway)

    result = tracking_native_acceptance._run_attached(
        plan,
        tmp_path / "plan.json",
        {"thresholds": {"poll_interval_s": 0.001}},
        0.1,
    )

    assert gateway.scene_reads == 2
    assert result["person_id"] == "track_person_01"
    assert result["follow"]["navigation_state"] == "accepted"
    assert result["motion"]["person_motion_m"] == pytest.approx(0.3)
    assert result["motion"]["robot_motion_m"] == pytest.approx(0.3)
    assert result["motion"]["goal_motion_m"] == pytest.approx(0.3)
    assert result["motion"]["navigation_tasks_observed"] == 2
    assert result["stop"]["navigation_state"] == "cancelled"
    assert [(path, body["mode"]) for path, body in gateway.posts] == [
        ("/api/v1/visual_servo", "follow"),
        ("/api/v1/visual_servo", "stop"),
    ]
    assert gateway.posts[0][1]["target_id"] == "track_person_01"


def test_tracking_manifest_keeps_attached_success_at_component_e2e() -> None:
    manifest = json.loads(tracking_native_acceptance.DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    evidence = product_acceptance.classify_evidence(
        manifest["acceptance_scope"],
        run_plan_verified=True,
        acceptance_evaluated=True,
        ok=True,
    )

    assert evidence["evidence_scope"] == "component_e2e"
    assert evidence["product_acceptance_passed"] is False
    assert "moving scenario person remains visible while native navigation updates goals" in manifest[
        "acceptance_scope"
    ]["claims"]
    assert "the robot moves while follow distance does not diverge" in manifest[
        "acceptance_scope"
    ]["claims"]
    assert "Driver stop acknowledgement or command forwarding" in manifest["acceptance_scope"][
        "excluded_claims"
    ]


def test_inspection_gate_is_attach_only_and_preflight_does_not_call_gateway(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("inspection")
    product_session_id = "f" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    sim_switch._commit_current(
        plan_path,
        plan,
        {},
        tmp_path,
        product_session_id=product_session_id,
    )
    monkeypatch.setattr(
        inspection_native_acceptance,
        "LingTuClient",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("preflight must not call Gateway")),
    )

    assert inspection_native_acceptance.main(["--run-plan", str(plan_path), "--strict", "--preflight-only"]) == 0
    source = Path(inspection_native_acceptance.__file__).read_text(encoding="utf-8")
    assert "native.main" not in source
    assert "Popen" not in source


def test_inspection_attached_runner_rejects_identity_before_http(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    plan = SimpleNamespace()
    monkeypatch.setattr(
        inspection_native_acceptance,
        "_current_identity",
        lambda *_args: (_ for _ in ()).throw(ValueError("identity mismatch")),
    )
    monkeypatch.setattr(
        inspection_native_acceptance,
        "_client",
        lambda *_args: (_ for _ in ()).throw(AssertionError("HTTP must not run")),
    )

    with pytest.raises(ValueError, match="identity mismatch"):
        inspection_native_acceptance._run_attached(
            plan,
            tmp_path / "plan.json",
            {"thresholds": {}},
            1.0,
        )


def test_inspection_manifest_keeps_attached_success_at_component_e2e() -> None:
    manifest = json.loads(inspection_native_acceptance.DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    evidence = product_acceptance.classify_evidence(
        manifest["acceptance_scope"],
        run_plan_verified=True,
        acceptance_evaluated=True,
        ok=True,
    )

    assert evidence["evidence_scope"] == "component_e2e"
    assert evidence["product_acceptance_passed"] is False
    assert "binaries" not in manifest
    assert "offline MCAP replay or timeline verification" in manifest["acceptance_scope"]["excluded_claims"]
    source = Path(inspection_native_acceptance.__file__).read_text(encoding="utf-8")
    assert "subprocess" not in source
    assert "lingtu_dds_player" not in source


@pytest.mark.parametrize(
    ("failure", "message"),
    (
        ("ack", "business ACK identity mismatch"),
        ("response_lost", "connection lost"),
        ("cleanup", "cleanup failed"),
        ("cleanup_recording", "recording did not complete"),
        ("identity", "status identity mismatch"),
        ("post_status", "status identity mismatch"),
        ("report_exact", "COMPLETE and ACCEPTABLE"),
        ("report", "COMPLETE and ACCEPTABLE"),
        ("evidence", "valid RGB"),
        ("recording", "recording failed"),
        ("motion", "displacement"),
    ),
)
def test_inspection_attached_runner_fails_closed_and_cancels_only_own_task(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    failure: str,
    message: str,
) -> None:
    product_session_id = "f" * 32

    class Position:
        def __init__(self, x: float) -> None:
            self.x = x
            self.y = 0.0

    class Gateway:
        def __init__(self) -> None:
            self.posts: list[tuple[str, dict[str, object]]] = []
            self.positions = iter((0.0, 0.0 if failure == "motion" else 0.3))
            self.task_id = ""
            self.route: dict[str, object] = {}
            self.status_calls = 0

        def position(self) -> Position:
            return Position(next(self.positions))

        def _post(self, path: str, body: dict[str, object]) -> dict[str, object]:
            self.posts.append((path, body))
            if path.endswith("/cancel"):
                if failure == "cleanup":
                    raise RuntimeError("cancel transport failed")
                return {
                    "ok": True,
                    "accepted": True,
                    "action": "cancel",
                    "task_id": self.task_id,
                    "request_id": body["request_id"],
                }
            if path == "/api/v1/inspection/routes":
                self.route = body
                return {
                    "ok": True,
                    "route": {
                        "id": body["id"],
                        "map_id": body["map_id"],
                        "revision": body["revision"],
                    },
                }
            if path == "/api/v1/inspection/tasks":
                self.task_id = inspection_native_acceptance._expected_task_id(str(body["request_id"]))
                if failure == "response_lost":
                    raise ConnectionError("connection lost after submission")
                return {
                    "ok": True,
                    "accepted": True,
                    "action": "start",
                    "lifecycle": "submission_accepted",
                    "terminal": False,
                    "request_id": body["request_id"],
                    "task_id": ("foreign-task" if failure in {"ack", "cleanup", "cleanup_recording"} else self.task_id),
                    "route_id": body["route_id"],
                    "revision": body["revision"],
                    "map_id": body["map_id"],
                }
            raise AssertionError(path)

        def _get(self, path: str) -> dict[str, object]:
            if path == "/ready":
                return {
                    "product_contract": {
                        "product": "inspection",
                        "product_session_id": product_session_id,
                    }
                }
            if path == "/api/v1/runtime/dataflow":
                return {
                    "authoritative": True,
                    "available": True,
                    "run_plan": {
                        "identity": {
                            "product": "inspection",
                            "env": "sim",
                            "product_session_id": product_session_id,
                        }
                    },
                }
            if path == f"/api/v1/inspection/tasks/{self.task_id}":
                self.status_calls += 1
                route_id = str(self.route["id"])
                request_id = next(
                    body["request_id"] for post_path, body in self.posts if post_path == "/api/v1/inspection/tasks"
                )
                return {
                    "task_id": self.task_id,
                    "terminal": True,
                    "current_state": "SUCCESS",
                    "execution_confirmed": True,
                    "terminal_source": "native_task_event",
                    "identity": {
                        "task_id": self.task_id,
                        "route_id": (
                            "foreign-route"
                            if failure == "identity" or (failure == "post_status" and self.status_calls > 1)
                            else route_id
                        ),
                        "route_revision": 1,
                        "map_id": "saved-map",
                    },
                    "last_submission": {"request_id": request_id, "action": "start"},
                    "latest_event": {
                        "task_id": self.task_id,
                        "request_id": request_id,
                        "route_id": route_id,
                        "route_revision": 1,
                        "map_id": "saved-map",
                        "point_id": "overview-1",
                        "action": "capture:overview",
                        "action_request_id": "evidence-own",
                    },
                    "delivery": {"history_complete": True},
                    "recording": (
                        None
                        if failure == "cleanup_recording"
                        else {
                            "state": (
                                "stopping" if failure == "post_status" and self.status_calls == 1 else "completed"
                            ),
                            "session_id": "recording-own",
                            "product_session_id": product_session_id,
                        }
                    ),
                }
            if path.endswith("/report"):
                return {
                    "task_id": self.task_id,
                    "report_status": "PARTIAL" if failure == "report" else "COMPLETE",
                    "acceptance": "ACCEPTABLE",
                    "terminal": True,
                    "execution": {
                        "terminal": True,
                        "confirmed": True,
                        "history_complete": True,
                    },
                    "issues": [{"code": "unexpected"}] if failure == "report_exact" else [],
                    "coverage": {
                        "required_points": 1,
                        "completed_points": 1,
                        "required_evidence": 1,
                        "verified_evidence": 1,
                        "missing_evidence": 0,
                        "invalid_evidence": 0,
                        "unavailable_evidence": 0,
                        "unknown_evidence": 0,
                    },
                    "identity": {
                        "route_id": self.route["id"],
                        "route_revision": 1,
                        "map_id": "saved-map",
                    },
                    "points": [
                        {
                            "point_id": "overview-1",
                            "point_index": 0,
                            "loop_index": 0,
                            "action": "capture:overview",
                            "status": "COMPLETED",
                            "evidence_status": "VERIFIED",
                            "evidence_id": "evidence-own",
                        }
                    ],
                }
            if path.endswith("/evidence/evidence-own"):
                return {
                    "evidence": {
                        "evidence_id": "evidence-own",
                        "request": {
                            "request_id": "evidence-own",
                            "run_id": self.task_id,
                            "route_id": self.route["id"],
                            "route_revision": 1,
                            "map_id": "saved-map",
                            "point_id": "overview-1",
                            "point_index": 0,
                            "action": "capture:overview",
                        },
                        "artifacts": []
                        if failure == "evidence"
                        else [{"kind": "rgb", "bytes": 10, "sha256": "b" * 64}],
                    }
                }
            raise AssertionError(path)

    gateway = Gateway()
    plan = SimpleNamespace(
        host_config={"gateway_port": 5050},
        lifecycle={"requires_map": True},
        native_process_environment={"LINGTU_MAP_NAME": "saved-map"},
    )
    monkeypatch.setattr(
        inspection_native_acceptance,
        "_current_identity",
        lambda *_args: (tmp_path, product_session_id, b"current"),
    )
    monkeypatch.setattr(inspection_native_acceptance, "_client", lambda *_args: gateway)
    if failure == "recording":
        monkeypatch.setattr(
            inspection_native_acceptance,
            "_recording_mcap",
            lambda *_args: (_ for _ in ()).throw(RuntimeError("recording failed")),
        )
    else:
        monkeypatch.setattr(
            inspection_native_acceptance,
            "_recording_mcap",
            lambda *_args: ("recording-own", tmp_path / "evidence.mcap"),
        )
    with pytest.raises(RuntimeError, match=message):
        inspection_native_acceptance._run_attached(
            plan,
            tmp_path / "plan.json",
            {"thresholds": {"minimum_displacement_m": 0.1, "poll_interval_s": 0.001}},
            0.1,
        )
    cancel_paths = [path for path, _body in gateway.posts if path.endswith("/cancel")]
    assert cancel_paths == [f"/api/v1/inspection/tasks/{gateway.task_id}/cancel"]


def test_mujoco_product_dispatcher_accepts_only_a_published_run_plan(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert product_acceptance.main(["nav", "--dry-run", "--json"]) == 2
    error = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert "--run-plan" in error["error"]


def test_mujoco_product_dispatcher_uses_explicit_tool_inputs(
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop")
    explicit = _target("teleop")

    target = product_acceptance.resolve_target(
        plan,
        runner=explicit.runner,
        manifest=explicit.manifest,
    )

    assert "acceptance" not in plan.as_dict()["launch"]
    assert target.runner == explicit.runner
    assert target.manifest == explicit.manifest


@pytest.mark.parametrize(
    ("product", "runner"),
    (
        ("teleop", "teleop_native_acceptance.py"),
        ("teleop_avoid", "teleop_avoid_native_acceptance.py"),
        ("map", "map_native_acceptance.py"),
        ("nav", "native_navigation_acceptance.py"),
        ("tracking", "tracking_native_acceptance.py"),
        ("inspection", "inspection_native_acceptance.py"),
    ),
)
def test_mujoco_product_dispatcher_resolves_persistent_product_run_plans(
    product: str,
    runner: str,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan(product)
    plan_path = plan.write(tmp_path / f"plan-{PRODUCT_SESSION_ID}.json")

    assert product_acceptance.main(
        _dispatcher_args(product, plan_path, "--dry-run", "--json")
    ) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["backend"] == "mujoco"
    assert payload["product"] == product
    assert payload["acceptance_scope"]["coverage"] == "component"
    if product in {"teleop", "teleop_avoid", "map"}:
        assert payload["executed"] is False
        assert "command" not in payload
        assert payload["exact_transaction"]["steps"] == [
            "ProductControl.switch",
            f"{product} scenario",
            "ProductControl.stop_current compare-and-stop",
        ]
        assert payload["exact_transaction"]["rollback_state_root"] is None
    else:
        assert payload["runner"].endswith(runner)
        assert payload["command"][1].endswith(runner)
        assert payload["command"][-1] == "--strict"
    if product == "teleop_avoid":
        assert list(target_role for target_role in _target(product).roles) == []
        assert sorted({role for process in plan.processes for role in process.provides}) == [
            "driver",
            "host",
            "imu",
            "lidar",
            "maps",
            "nav",
            "slam",
            "traversability",
        ]
        assert "roles" not in payload


def test_mujoco_product_dispatcher_propagates_strict_runner_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("nav")
    plan_path = plan.write(tmp_path / f"plan-{PRODUCT_SESSION_ID}.json")
    observed: list[str] = []

    def run(command: list[str], **_kwargs: object) -> SimpleNamespace:
        observed.extend(command)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(product_acceptance.subprocess, "run", run)

    assert product_acceptance.main(_dispatcher_args("nav", plan_path)) == 1
    assert observed[-1] == "--strict"


def test_runner_case_uses_the_committed_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = product_acceptance.AcceptanceTarget(
        product="map",
        runner=tmp_path / "runner.py",
        manifest=tmp_path / "manifest.json",
        run_plan=tmp_path / "input.json",
    )
    committed = tmp_path / "session" / "run-plan.json"
    observed: list[str] = []

    def run(command: list[str], **_kwargs: object) -> SimpleNamespace:
        observed.extend(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(product_acceptance.subprocess, "run", run)

    report = product_acceptance._runner_case(target)(
        cast(RunPlan, SimpleNamespace()),
        committed,
        "a" * 32,
    )

    assert report["ok"] is True
    assert observed[2:4] == ["--run-plan", str(committed)]


def test_avoid_case_uses_the_committed_product(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as avoid

    target = product_acceptance.AcceptanceTarget(
        product="teleop_avoid",
        runner=Path(avoid.__file__).resolve(),
        manifest=avoid.DEFAULT_MANIFEST.resolve(),
    )
    plan = SimpleNamespace(
        processes=(
            SimpleNamespace(
                name="nav_runtime",
                command=SimpleNamespace(
                    argv=("navd", "--domain-id", "225"),
                    env={},
                    readiness=SimpleNamespace(kind="file", target="nav.status.json"),
                ),
            ),
        ),
    )
    path = (tmp_path / "run-plan.json").resolve()
    observed: dict[str, object] = {}
    monkeypatch.setattr(
        avoid,
        "prepare_runtime",
        lambda args: {"ok": True, "blockers": [], "binaries": {}, "args": args},
    )

    def run_attached(**kwargs: object) -> dict[str, object]:
        observed.update(kwargs)
        return {"ok": True, "mode": "attach_only"}

    monkeypatch.setattr(avoid, "run_attached", run_attached)

    report = product_acceptance._avoid_case(target)(cast(RunPlan, plan), path, "s" * 32)

    assert report == {"ok": True, "mode": "attach_only"}
    assert observed["plan"] is plan
    assert observed["run_plan_path"] == path
    assert observed["product_session_id"] == "s" * 32
    args = observed["args"]
    assert args.domain_base == 225


@pytest.mark.parametrize(
    ("product", "expected_exit"),
    (("teleop", 0), ("teleop_avoid", 0), ("map", 0)),
)
def test_dispatcher_runs_component_lifecycle_without_a_rollback_process_owner(
    product: str,
    expected_exit: int,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan(product)
    plan_path = plan.write(tmp_path / f"plan-{PRODUCT_SESSION_ID}.json")
    state_root = (tmp_path / "exact-cli").resolve()
    runner = _Runner(state_root)
    control = ProductControl(
        robot=plan.robot,
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=runner,
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: plan)
    control_args: list[dict[str, object]] = []

    def product_control(**kwargs: object) -> ProductControl:
        control_args.append(dict(kwargs))
        return control

    monkeypatch.setattr(product_acceptance, "ProductControl", product_control)
    monkeypatch.setattr(
        ProcessIdentity,
        "matches",
        lambda _identity: not runner.stopped,
    )
    monkeypatch.setattr(
        product_acceptance,
        "load_motion_evidence",
        lambda **_kwargs: {
            "commanded_motion_observed": True,
            "nonzero_command_count": 8,
            "nonzero_physics_steps": 8,
            "last_output_sequence": 8,
            "path_length_xy_m": 0.3,
        },
    )
    monkeypatch.setattr(
        product_acceptance,
        "_teleop_case",
        lambda _target: (
            lambda *_args: {
                "ok": True,
                "mode": "attach_only",
                "min_path_length_m": 0.15,
            }
        ),
    )
    monkeypatch.setattr(
        product_acceptance,
        "_avoid_case",
        lambda _target: lambda *_args: {"ok": True, "mode": "attach_only"},
    )
    monkeypatch.setattr(
        product_acceptance,
        "_check_teleop_avoid_motion",
        lambda *_args: {},
    )
    monkeypatch.setattr(
        product_acceptance,
        "_runner_case",
        lambda _target: lambda *_args: {"ok": True, "mode": "attach_only"},
    )
    monkeypatch.setattr(
        product_acceptance.subprocess,
        "run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("the dispatcher must not create a second process owner")
        ),
    )

    assert (
        product_acceptance.main(
            _dispatcher_args(
                product,
                plan_path,
                "--state-root",
                str(state_root),
            )
        )
        == expected_exit
    )
    assert runner.calls == ["apply", "stop"]
    assert control_args[0]["robot"] == plan.robot


def test_product_scope_rejects_missing_rollback_root_before_control_construction(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    linux_sim_plan: Callable[..., RunPlan],
) -> None:
    plan = linux_sim_plan("teleop")
    plan_path = plan.write(tmp_path / f"plan-{PRODUCT_SESSION_ID}.json")
    constructed = False

    def reject_construction(**_kwargs):
        nonlocal constructed
        constructed = True
        raise AssertionError("ProductControl construction must not occur")

    monkeypatch.setattr(product_acceptance, "ProductControl", reject_construction)
    monkeypatch.setattr(
        product_acceptance,
        "acceptance_scope",
        lambda _target: {"coverage": "product"},
    )

    assert (
        product_acceptance.main(
            _dispatcher_args(
                "teleop",
                plan_path,
                "--state-root",
                str(tmp_path / "state"),
                "--json",
            )
        )
        == 2
    )
    assert constructed is False
