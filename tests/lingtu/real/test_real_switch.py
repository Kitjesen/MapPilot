
from __future__ import annotations

import io
import json
import threading
import urllib.error
from pathlib import Path
from types import SimpleNamespace

import pytest

from lingtu.control import ProductControl
from lingtu.control_server import ControlOperations
from lingtu.real.backend import (
    FieldBackend,
    MapActivationToken,
    SessionFile,
    SessionStage,
)
from lingtu.run_plan import RunPlan
from lingtu.switch_contracts import (
    MapArtifactIdentity,
    MapIdentity,
    ProcessFailed,
    ProcessReport,
    SwitchFailed,
    SwitchRequest,
)


class FakeRunner:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def apply(self, plan: RunPlan, *, dry_run: bool = False, on_process_ready=None, on_process_started=None) -> ProcessReport:
        self.calls.append("apply")
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
            dry_run=dry_run,
        )
        if on_process_ready is not None and not dry_run:
            try:
                for process in sorted(plan.processes, key=lambda item: (item.order, item.name)):
                    if on_process_started is not None:
                        on_process_started(process)
                    on_process_ready(process)
            except Exception as exc:
                from lingtu.switch_contracts import ProcessFailed
                report.ok = False
                report.status = "failed"
                report.error = str(exc)
                raise ProcessFailed(report) from exc
        return report

    def apply_deferred(self, plan: RunPlan, *, dry_run: bool = False, on_process_ready=None, on_process_started=None) -> ProcessReport:
        return self.apply(plan, dry_run=dry_run, on_process_ready=on_process_ready, on_process_started=on_process_started)

    def transition(
        self,
        previous: RunPlan,
        plan: RunPlan,
        *,
        dry_run: bool = False,
        defer_rollback: bool = False,
        on_process_ready=None,
        on_process_started=None,
    ) -> ProcessReport:
        assert defer_rollback is True
        return self.apply(plan, dry_run=dry_run, on_process_ready=on_process_ready, on_process_started=on_process_started)

    def ensure_transition_process_active(self, plan, transition, process_name):
        return transition

    def stop_transition_target(self, plan, transition):
        self.calls.append("stop_target")
        return transition

    def restore_transition_previous(self, previous, transition, *, on_process_ready=None):
        self.calls.append("restore_previous")
        if on_process_ready is not None:
            for process in previous.processes:
                on_process_ready(process)
        return transition

    def quiesce(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        self.calls.append("quiesce")
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            ok=True,
            status="stopped",
            dry_run=dry_run,
        )


class FakeBackend:
    def __init__(self, *, fail_at: str | None = None) -> None:
        self.fail_at = fail_at
        self.events: list[str] = []
        self.map_save_idle_calls = 0
        self.recording_idle_calls = 0

    def _event(self, name: str) -> None:
        self.events.append(name)
        if name == self.fail_at:
            self.fail_at = None
            raise RuntimeError(f"failed at {name}")

    def assert_map_save_idle(self) -> None:
        self.map_save_idle_calls += 1

    def assert_recording_idle(self) -> None:
        self.recording_idle_calls += 1

    def stop_motion(self, current_product=None) -> None:
        self._event("stop_motion")

    def prepare_map(self, map_name: str):
        self._event("prepare_map")
        return _map_identity(map_name, 7)

    def stage_map(self, map_name: str) -> MapActivationToken:
        self._event("stage_map")
        return MapActivationToken(
            target=_map_identity(map_name, 7),
            previous=_map_identity("warehouse", 3),
            changed=True,
            activation_token="activation-token",
        )

    def restore_map(self, token: MapActivationToken) -> None:
        self._event("restore_map")

    def commit_map(self, token: MapActivationToken) -> None:
        self._event("commit_map")

    def stage_session(
        self,
        run_plan_path: Path,
        plan: RunPlan,
        native_environment,
        **_kwargs,
    ) -> SessionStage:
        assert run_plan_path.is_file()
        self._event("stage_session")
        return SessionStage(files=(SessionFile(path="/run/lingtu/session.env"),))

    def rollback_session(self, staged: SessionStage) -> None:
        self._event("rollback_session")

    def clear_runtime_status(self) -> None:
        self._event("clear_runtime_status")

    def wait_native_nav(self, native_environment, *, timeout_s: float) -> None:
        self._event("wait_native_nav")

    def wait_slam(
        self,
        mode: str,
        *,
        require_map: bool,
        require_localization: bool = False,
        timeout_s: float,
    ) -> None:
        self._event("wait_slam_localized" if require_localization else "wait_slam")

    def prepare_localization(self, lifecycle, **_kwargs) -> None:
        self._event("prepare_localization")

    def wait_navigation(self, **_kwargs) -> None:
        self._event("wait_navigation")

    def wait_exploration(self, route: str, **_kwargs) -> None:
        self._event("wait_exploration")

    def wait_inspection(self, *, timeout_s: float) -> None:
        self._event("wait_inspection")


def _control(runner: FakeRunner) -> ProductControl:
    return ProductControl(
        runner,
        robot="unitree/go2",
        env="real",
        process_env={},
    )  # type: ignore[arg-type]


def _request() -> SwitchRequest:
    return SwitchRequest(
        target_product="nav",
        map_name="plant-a",
        relocalize=True,
        initial_pose=(1.0, 2.0, 0.3),
    )


def _map_identity(name: str, version: int) -> MapIdentity:
    root = f"/maps/{name}"
    return MapIdentity(
        map_id=name,
        content_epoch=version,
        frame_id="map",
        artifacts=(
            MapArtifactIdentity("POINTCLOUD", f"{root}/map.pcd"),
            MapArtifactIdentity("OCTOMAP_3D", f"{root}/octomap.ot"),
            MapArtifactIdentity("OCCUPANCY_2D", f"{root}/occupancy.npz"),
        ),
    )


def _map_payload(name: str, version: int) -> dict[str, object]:
    identity = _map_identity(name, version)
    return {
        "present": True,
        "map_id": identity.map_id,
        "content_epoch": identity.content_epoch,
        "frame_id": identity.frame_id,
        "artifacts": [
            {"type": artifact.artifact_type, "uri": artifact.uri}
            for artifact in identity.artifacts
        ],
    }


def test_explore_readiness_uses_only_top_level_product_session(tmp_path: Path) -> None:
    status_path = tmp_path / "explore.status.json"
    status = {
        "schema_version": "lingtu.explore.status.v2",
        "endpoint": "lingtu_explore_dds",
        "product_session_id": "product-session-a",
        "route": "live",
        "ready": True,
        "active": False,
        "paused": False,
        "state": "idle",
        "pending_goal": None,
        "pending_segment": None,
        "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.1},
        "map": {
            "frame_id": "map",
            "map_id": "",
            "map_content_epoch": 0,
            "reset_epoch": 1,
            "generation": 1,
            "live": True,
        },
        "counters": {"odometry_messages": 1, "snapshot_messages": 1},
    }
    status_path.write_text(json.dumps(status), encoding="utf-8")
    written_at = status_path.stat().st_mtime
    FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        monotonic=lambda: 0.0,
        wall_clock=lambda: written_at,
    ).wait_exploration(
        "live",
        map_identity=None,
        product_session_id="product-session-a",
        timeout_s=1.0,
    )

    status["product_session_id"] = "product-session-b"
    status_path.write_text(json.dumps(status), encoding="utf-8")
    clock = iter((0.0, 1.0))
    with pytest.raises(RuntimeError, match="did not become ready"):
        FieldBackend(
            environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
            monotonic=lambda: next(clock),
            wall_clock=lambda: status_path.stat().st_mtime,
            sleep=lambda _seconds: None,
        ).wait_exploration(
            "live",
            map_identity=None,
            product_session_id="product-session-a",
            timeout_s=0.5,
        )


def test_explore_readiness_accepts_dispatching_goal(tmp_path: Path) -> None:
    status_path = tmp_path / "explore.status.json"
    status = {
        "schema_version": "lingtu.explore.status.v2",
        "endpoint": "lingtu_explore_dds",
        "product_session_id": "product-session-a",
        "route": "live",
        "ready": True,
        "active": True,
        "paused": False,
        "state": "dispatching",
        "pending_goal": {"request_id": "goal-request-a"},
        "pending_segment": None,
        "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.1},
        "map": {
            "frame_id": "map",
            "map_id": "",
            "map_content_epoch": 0,
            "reset_epoch": 1,
            "generation": 1,
            "live": True,
        },
        "counters": {"odometry_messages": 1, "snapshot_messages": 1},
    }
    status_path.write_text(json.dumps(status), encoding="utf-8")
    written_at = status_path.stat().st_mtime
    clock = iter((0.0, 0.0, 2.0))

    FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        monotonic=lambda: next(clock),
        wall_clock=lambda: written_at,
        sleep=lambda _seconds: None,
    ).wait_exploration(
        "live",
        map_identity=None,
        product_session_id="product-session-a",
        timeout_s=1.0,
        allow_active=True,
    )


def test_cold_switch_commits_map_and_current_product_identity(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()

    report = _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert report.ok is True
    assert "goal_acceptance_ready" in report.phases
    assert "navigation_ready" not in report.phases
    assert report.product_session_id == current["product_session_id"]
    assert current["product"] == "nav"
    assert current["map_identity"]["content_epoch"] == 7
    assert runner.calls == ["apply"]
    assert backend.map_save_idle_calls == 0
    assert backend.recording_idle_calls == 0
    assert backend.events == [
        "stop_motion",
        "prepare_map",
        "stage_session",
        "clear_runtime_status",
        "wait_slam",
        "prepare_localization",
        "wait_slam_localized",
        "stage_map",
        "wait_native_nav",
        "wait_navigation",
        "commit_map",
    ]


@pytest.mark.parametrize("ready", [True, False])
def test_remote_switch_waits_for_readiness(tmp_path, monkeypatch, ready) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at=None if ready else "wait_navigation")
    entered = threading.Event()
    release = threading.Event()

    def wait_navigation(**_kwargs):
        entered.set()
        assert release.wait(10), "test did not release readiness"
        backend._event("wait_navigation")

    monkeypatch.setattr(backend, "wait_navigation", wait_navigation)
    def field_backend(**kwargs):
        assert kwargs["gateway_url"] == "http://127.0.0.1:5053"
        return backend

    monkeypatch.setattr("lingtu.real.switch.FieldBackend", field_backend)
    control = _control(runner)
    operations = ControlOperations(control, tmp_path)
    request_id = "remote-switch"
    try:
        operations.submit({
            "request_id": request_id,
            "product": "nav",
            "map_name": "plant-a",
            "expected_product_session_id": "",
        })
        assert entered.wait(30), "switch did not reach readiness"
        assert operations.get(request_id)["state"] == "running"
        assert not (tmp_path / "current.json").exists()
    finally:
        release.set()
        operations.close()

    result = operations.get(request_id)
    assert result["state"] == ("succeeded" if ready else "failed")
    assert result["result"]["ok"] is ready
    assert (tmp_path / "current.json").exists() is ready
    if ready:
        current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
        assert current["product_session_id"] == result["result"]["product_session_id"]
        assert "commit_map" in backend.events
    else:
        assert "failed at wait_navigation" in result["message"]
        assert "commit_map" not in backend.events
        assert "rollback_session" in backend.events

    restored = ControlOperations(control, tmp_path)
    try:
        assert restored.get(request_id) == result
    finally:
        restored.close()


def test_switch_reuses_verified_plan(tmp_path, monkeypatch) -> None:
    loads = []
    original_load = RunPlan.load

    def load(cls, path):
        loads.append(Path(path))
        return original_load(path)

    monkeypatch.setattr(RunPlan, "load", classmethod(load))
    report = _control(FakeRunner())._switch(_request(), backend=FakeBackend(), state_dir=tmp_path)

    assert report.ok
    assert loads == [Path(report.run_plan_path)]


def test_staged_plan_change_blocks_start(tmp_path, monkeypatch) -> None:
    runner = FakeRunner()
    backend = FakeBackend()
    stage_session = backend.stage_session

    def stage(path, plan, environment, **kwargs):
        staged = stage_session(path, plan, environment, **kwargs)
        plan.with_process_environment({**plan.process_environment, "LINGTU_CONTROL_TEST": "changed"}).write(path)
        return staged

    monkeypatch.setattr(backend, "stage_session", stage)
    with pytest.raises(SwitchFailed, match="published RunPlan changed after staging"):
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert "apply" not in runner.calls
    assert "rollback_session" in backend.events
    assert not (tmp_path / "current.json").exists()


def test_mapping_restore_passes_archive_and_seed_without_activating_fixed_map(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()
    report = _control(runner)._switch(
        SwitchRequest(target_product="map", map_name="plant-a", initial_pose=(1.0, 2.0, 0.4, 0.3)),
        backend=backend, state_dir=tmp_path,
    )
    assert report.ok
    assert "prepare_map" in backend.events
    assert "stage_map" not in backend.events
    assert "prepare_localization" not in backend.events
    plan = RunPlan.load(report.run_plan_path)
    environment = FieldBackend(environment={"NAV_MAP_DIR": "/maps"})._session_environment(
        run_plan_path=Path(report.run_plan_path), plan=plan,
        native_environment=plan.native_process_environment, slam_mode="mapping",
        map_identity=_map_identity("plant-a", 7), product_session_id=report.product_session_id,
    )
    assert environment["LINGTU_SLAM_MODE"] == "mapping"
    assert environment["LINGTU_SLAM_MAP"] == "/maps/plant-a/map.pcd"
    assert environment["LINGTU_SLAM_TRACK_INITIAL_Z"] == "0.4"
    assert environment["OCTOPLANNER_MAP_PATH"] == ""


def test_initial_pose_cannot_be_silently_ignored(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()

    with pytest.raises(SwitchFailed, match="initial_pose requires relocalize=True"):
        _control(runner)._switch(
            SwitchRequest(
                target_product="nav",
                map_name="plant-a",
                relocalize=False,
                initial_pose=(1.0, 2.0, 0.3),
            ),
            backend=backend,
            state_dir=tmp_path,
        )

    assert runner.calls == []
    assert backend.events == []


def test_wait_slam_distinguishes_frontend_tracking_from_saved_map_alignment(
    tmp_path: Path,
) -> None:
    status_path = tmp_path / "slam.status.json"
    status = {
        "mode": "localization",
        "runtime_instance_id": "runtime-1",
        "alive": True,
        "snapshot_written_at_s": 100.0,
        "map_loaded": True,
        "state": "TRACKING",
        "map_odom_tf": None,
    }
    status_path.write_text(json.dumps(status), encoding="utf-8")
    environment = {"LINGTU_SLAM_STATUS_JSON": str(status_path)}

    FieldBackend(
        environment=environment,
        monotonic=lambda: 0.0,
        wall_clock=lambda: 100.0,
    ).wait_slam(
        "localization",
        require_map=True,
        require_localization=False,
        timeout_s=1.0,
    )

    clock = iter((0.0, 0.0, 1.0))
    backend = FieldBackend(
        environment=environment,
        monotonic=lambda: next(clock),
        wall_clock=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    with pytest.raises(RuntimeError, match="require_localization=True"):
        backend.wait_slam(
            "localization",
            require_map=True,
            require_localization=True,
            timeout_s=0.5,
        )

    status["map_odom_tf"] = {"valid": True}
    status["track_against_map"] = {
        "enabled": True,
        "successes": 1,
        "degraded": False,
    }
    status_path.write_text(json.dumps(status), encoding="utf-8")
    FieldBackend(
        environment=environment,
        monotonic=lambda: 0.0,
        wall_clock=lambda: 100.0,
    ).wait_slam(
        "localization",
        require_map=True,
        require_localization=True,
        timeout_s=1.0,
    )


def test_map_save_idle_accepts_product_without_host_map_service(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: {
            "_http_status": 503,
            "detail": "maps.service is unavailable; ProductGraph/HostBuilder must inject maps.service",
        },
    )

    backend.assert_map_save_idle()


def test_http_returns_explicitly_allowed_error_status(monkeypatch) -> None:
    payload = {"detail": "maps.service is unavailable"}

    def urlopen(*_args, **_kwargs):
        raise urllib.error.HTTPError(
            "http://127.0.0.1:5050/api/v1/maps/operations?limit=1000",
            503,
            "Service Unavailable",
            {},
            io.BytesIO(json.dumps(payload).encode("utf-8")),
        )

    monkeypatch.setattr("lingtu.real.backend.urllib.request.urlopen", urlopen)

    response = FieldBackend(environment={})._http(
        "GET",
        "/api/v1/maps/operations?limit=1000",
        timeout_s=1.0,
        allowed_error_statuses=(503,),
    )

    assert response == {"detail": "maps.service is unavailable", "_http_status": 503}




def test_navigation_startup_uses_status_after_map_activation(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    calls: list[str] = []

    def http(_method, path, *, timeout_s):
        assert timeout_s == 3.0
        calls.append(path)
        if path == "/api/v1/navigation/status":
            return {
                "task": {"state": "IDLE", "task_id": "", "reason": ""},
                "goal_admission": {"state": "ACCEPTING"},
                "control": {"authority": "NONE", "resume_required": False, "reason": ""},
                "motion": {
                    "permission": "CLEAR",
                    "observation": "QUIET",
                    "stop_confirmation": "NOT_REQUESTED",
                    "reason": "",
                },
            }
        return {"active_map": "plant-a"}

    monkeypatch.setattr(backend, "_http", http)

    backend.wait_navigation(
        map_name="plant-a",
        control_mode="autonomy",
        timeout_s=1.0,
    )

    assert calls == ["/api/v1/navigation/status"]




def test_navigation_startup_fails_closed_when_navigation_state_is_unknown(monkeypatch) -> None:
    clock = iter((0.0, 0.0, 2.0))
    backend = FieldBackend(
        environment={},
        monotonic=lambda: next(clock),
        sleep=lambda _seconds: None,
    )

    def http(_method, path, *, timeout_s):
        assert timeout_s == 3.0
        if path == "/api/v1/readiness":
            return {
                "runtime": {
                    "navigation": {
                        "native_endpoint": {
                            "ok": True,
                            "status_available": True,
                            "blockers": [],
                        },
                    },
                },
            }
        return {
            "task": {"state": "UNKNOWN", "task_id": "", "reason": "status stale"},
            "goal_admission": {"state": "UNKNOWN"},
            "control": {"authority": "UNKNOWN", "resume_required": False, "reason": ""},
            "motion": {
                "permission": "UNKNOWN",
                "observation": "UNKNOWN",
                "stop_confirmation": "UNKNOWN",
                "reason": "",
            },
        }

    monkeypatch.setattr(backend, "_http", http)

    with pytest.raises(RuntimeError, match="state=UNKNOWN"):
        backend.wait_navigation(
            map_name="",
            control_mode="teleop",
            timeout_s=1.0,
        )


def test_motion_stop_confirmation_blocks_process_mutation(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="stop_motion")

    with pytest.raises(SwitchFailed, match="failed at stop_motion") as failure:
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "stop_unconfirmed"
    assert runner.calls == []
    assert backend.events == ["stop_motion"]


@pytest.mark.parametrize(("nav_active", "stop_fails"), [(False, False), (True, False), (True, True)])
def test_first_start_slam_failure_stops_present_endpoints_without_requiring_unstarted_nav(
    tmp_path, monkeypatch, nav_active, stop_fails,
) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="wait_slam")
    active = {"nav": False}
    commands = []
    requested_products = []

    def run_stop(command, **_kwargs):
        commands.append(command)
        if stop_fails:
            raise RuntimeError("native navigation stop rejected")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    field_backend = FieldBackend(environment={}, runner=run_stop)
    monkeypatch.setattr(field_backend, "_unit_active", lambda _unit: active["nav"])

    def stop_motion(product=None):
        requested_products.append(product)
        field_backend.stop_motion(product)

    monkeypatch.setattr(backend, "stop_motion", stop_motion)

    def fail_at_slam(plan, *, on_process_ready, **_kwargs):
        report = ProcessReport(
            product=plan.product, env=plan.env, action="apply", status="failed",
            started=[plan.process("slam").target],
        )
        active["nav"] = nav_active
        try:
            on_process_ready(plan.process("slam"))
        except Exception as exc:
            report.error = str(exc)
            raise ProcessFailed(report) from exc
        pytest.fail("SLAM startup failure was not reached")

    monkeypatch.setattr(runner, "apply_deferred", fail_at_slam)
    with pytest.raises(SwitchFailed, match="failed at wait_slam") as failure:
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert requested_products == [None, None]
    assert len(commands) == int(nav_active)
    assert failure.value.report.status == ("stop_unconfirmed" if stop_fails else "failed_stopped")
    assert not (tmp_path / "current.json").exists()


def test_failed_switch_restores_map_quiesces_processes_and_removes_session(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="prepare_localization")

    with pytest.raises(SwitchFailed, match="failed at prepare_localization") as failure:
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "failed_stopped"
    assert runner.calls == ["apply", "quiesce"]
    assert backend.events[-2:] == ["stop_motion", "rollback_session"]
    assert "stage_map" not in backend.events
    assert "restore_map" not in backend.events
    assert not (tmp_path / "current.json").exists()


@pytest.mark.parametrize("previous_product", ["teleop", "map"])
def test_failed_switch_restores_previous_run_and_keeps_committed_record(tmp_path, previous_product) -> None:
    runner = FakeRunner()
    control = _control(runner)
    control._switch(
        SwitchRequest(target_product=previous_product, map_name="plant-a" if previous_product == "map" else None),
        backend=FakeBackend(),
        state_dir=tmp_path,
    )
    current_path = tmp_path / "current.json"
    committed = current_path.read_bytes()

    backend = FakeBackend(fail_at="prepare_localization")
    with pytest.raises(SwitchFailed) as failure:
        control._switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "failed_rolled_back"
    assert current_path.read_bytes() == committed
    assert runner.calls[-2:] == ["stop_target", "restore_previous"]
    assert "previous_session:active" in failure.value.report.cleanup
    assert backend.map_save_idle_calls == 1
    assert backend.recording_idle_calls == 1


@pytest.mark.parametrize(
    ("nav_evidence", "nav_active", "stop_fails", "restores_previous"),
    [
        ("stopped", False, False, True),
        ("not_stopped", False, False, False),
        ("start_attempted", False, False, False),
        ("preserved", False, False, False),
        ("stopped", True, False, True),
        ("stopped", True, True, False),
    ],
)
def test_localization_failure_rolls_back_only_with_navigation_stop_evidence(
    tmp_path, monkeypatch, nav_evidence, nav_active, stop_fails, restores_previous,
) -> None:
    runner = FakeRunner()
    control = _control(runner)
    control._switch(
        SwitchRequest(target_product="map", variant="camera"),
        backend=FakeBackend(),
        state_dir=tmp_path,
    )
    current_path = tmp_path / "current.json"
    committed = current_path.read_bytes()
    backend = FakeBackend(fail_at="wait_slam_localized")
    active = {"nav": True}
    stop_commands = []

    def run_stop(command, **_kwargs):
        stop_commands.append(command)
        if stop_fails and len(stop_commands) > 1:
            raise RuntimeError("native navigation stop rejected")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    field_backend = FieldBackend(environment={}, runner=run_stop)
    monkeypatch.setattr(field_backend, "_unit_active", lambda _unit: active["nav"])
    monkeypatch.setattr(backend, "stop_motion", field_backend.stop_motion)
    monkeypatch.setattr(backend, "remove_session", lambda _plan: None, raising=False)

    def fail_localization(previous, plan, *, on_process_ready, **_kwargs):
        runner.calls.append("apply")
        transition = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="transition",
            status="failed",
            started=[plan.process("slam").target],
        )
        if nav_evidence != "not_stopped":
            transition.stopped.append(previous.process("nav").target)
        if nav_evidence == "start_attempted":
            transition.started.append(plan.process("nav").target)
        if nav_evidence == "preserved":
            transition.preserved.append(plan.process("nav").target)
        active["nav"] = nav_active
        try:
            on_process_ready(plan.process("slam"))
        except Exception as exc:
            transition.error = str(exc)
            raise ProcessFailed(transition) from exc
        pytest.fail("localization readiness failure was not reached")

    monkeypatch.setattr(runner, "transition", fail_localization)
    with pytest.raises(SwitchFailed, match="failed at wait_slam_localized") as failure:
        control._switch(
            SwitchRequest(target_product="nav", variant="camera", map_name="plant-a"),
            backend=backend,
            state_dir=tmp_path,
        )

    # Even an unexpected live endpoint must receive the real native stop.
    assert len(stop_commands) == (2 if nav_active else 1)
    assert all(command[1:3] == ["stop", "product_mode_switch"] for command in stop_commands)
    assert "stage_map" not in backend.events
    if restores_previous:
        assert failure.value.report.status == "failed_rolled_back"
        assert current_path.read_bytes() == committed
        assert runner.calls[-2:] == ["stop_target", "restore_previous"]
        assert "previous_session:active" in failure.value.report.cleanup
        assert not any(item.startswith("motion_session_failed:") for item in failure.value.report.cleanup)
    else:
        assert failure.value.report.status == "rollback_failed"
        assert not current_path.exists()
        assert "restore_previous" not in runner.calls
        assert runner.calls[-2:] == ["quiesce", "quiesce"]
        assert any(item.startswith("motion_session_failed:") for item in failure.value.report.cleanup)


def test_stage_map_hands_native_identity_to_product_control(tmp_path) -> None:
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        payload = {
            "schema_version": "lingtu.map_activation.v2",
            "request_id": "mapctl-test",
            "operation": "stage",
            "accepted": True,
            "message": "ok",
            "changed": True,
            "producer_boot_id": "mapd-boot",
            "activation_token": "activation-token",
            "target": _map_payload("plant-a", 7),
            "previous": _map_payload("warehouse", 3),
            "active": _map_payload("plant-a", 7),
        }
        return SimpleNamespace(returncode=0, stdout=json.dumps(payload), stderr="")

    backend = FieldBackend(
        environment={
            "NAV_MAP_DIR": str(tmp_path),
            "LINGTU_MAPCTL_BIN": "/release/lingtu-mapctl",
            "LINGTU_DDS_DOMAIN_ID": "17",
        },
        runner=run,
    )

    token = backend.stage_map("plant-a")

    assert token.target == _map_identity("plant-a", 7)
    assert token.previous == _map_identity("warehouse", 3)
    assert calls[0][0:3] == ["/release/lingtu-mapctl", "stage", "plant-a"]


@pytest.mark.parametrize("failure", ["busy_once", "busy_forever", "invalid_artifact"])
def test_map_commit_waits_for_read_lock_without_accepting_invalid_artifacts(failure):
    clock = [0.0]
    calls = []

    def sleep(seconds):
        clock[0] += seconds

    def run(command, **kwargs):
        calls.append((command, kwargs))
        accepted = failure == "busy_once" and len(calls) > 1
        message = ("artifact_gate_failed:map write in progress: plant-a"
                   if failure != "invalid_artifact" else "artifact_gate_failed:missing octomap")
        return SimpleNamespace(returncode=0 if accepted else 1, stderr="", stdout=json.dumps({
            "schema_version": "lingtu.map_activation.v2", "operation": "verify",
            "accepted": accepted, "message": message, "active": _map_payload("plant-a", 7),
        }))

    backend = FieldBackend(environment={}, runner=run, sleep=sleep, monotonic=lambda: clock[0])
    token = MapActivationToken(target=_map_identity("plant-a", 7), previous=None,
                               changed=True, activation_token="activation-token")
    if failure == "busy_once":
        backend.commit_map(token)
        assert len(calls) == 2
        assert clock[0] == 0.2
    else:
        with pytest.raises(RuntimeError, match="artifact_gate_failed"):
            backend.commit_map(token)
        assert clock[0] <= 10.0
        if failure == "invalid_artifact":
            assert len(calls) == 1


@pytest.mark.parametrize("initial_pose", [None, (1.0, 2.0, 0.3), (1.0, 2.0, 0.45, 0.3)])
@pytest.mark.parametrize("accepted", [True, False])
def test_startup_localization_uses_native_dds_before_host(initial_pose, accepted):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0 if accepted else 1,
                               stdout=json.dumps({"success": accepted, "message": "rejected"}), stderr="")

    backend = FieldBackend(environment={"LINGTU_DDS_DOMAIN_ID": "17"}, runner=run)
    lifecycle = SimpleNamespace(slam_mode="localization")
    if accepted:
        backend.prepare_localization(lifecycle, map_name="plant-a", relocalize=True, initial_pose=initial_pose)
    else:
        with pytest.raises(RuntimeError, match="tracking start failed"):
            backend.prepare_localization(lifecycle, map_name="plant-a", relocalize=True, initial_pose=initial_pose)
    expected = ["/opt/lingtu/current/bin/slamctl", "track-against-map", "--domain-id", "17", "--timeout-s", "10"]
    if initial_pose is not None:
        expected += ["--x", "1.0", "--y", "2.0", "--yaw", "0.3"]
        if len(initial_pose) == 4:
            expected += ["--z", "0.45"]
    assert calls[0][0] == expected
    assert calls[0][1]["timeout"] == 12.0


def test_field_map_root_ignores_legacy_map_dir(tmp_path: Path) -> None:
    backend = FieldBackend(
        environment={
            "HOME": str(tmp_path),
            "MAP_DIR": str(tmp_path / "legacy"),
        }
    )

    assert backend._maps_root() == Path("/var/lib/lingtu/maps")


def test_session_stage_and_rollback_install_then_remove_one_session(monkeypatch, tmp_path) -> None:
    files: dict[str, str] = {}
    backend = FieldBackend(
        environment={
            "LINGTU_SESSION_ROOT": str(tmp_path),
            "NAV_MAP_DIR": "/maps",
        }
    )
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: files.__setitem__(target, content),
    )
    monkeypatch.setattr(
        backend,
        "_remove_runtime_file",
        lambda target, **_kwargs: files.pop(target, None),
    )
    plan = _control(FakeRunner())._resolve("nav")
    plan_path = tmp_path / "run-plan.json"
    plan.write(plan_path)

    staged = backend.stage_session(
        plan_path,
        plan,
        plan.native_process_environment,
        slam_mode="localization",
        map_identity=_map_identity("plant-a", 7),
        product_session_id="product-session-1234",
    )

    assert list(files) == [str(tmp_path / "session.env")]
    session_environment = next(iter(files.values()))
    assert 'LINGTU_PRODUCT_SESSION_ID="product-session-1234"' in session_environment
    assert 'LINGTU_NAV_STATUS_S="0.2"' in session_environment
    assert 'LINGTU_DRIVER_NETWORK_ADDRESS="192.168.123.18/24"' in session_environment
    assert 'LINGTU_DRIVER_PROBE_IP="192.168.123.161"' in session_environment
    assert '<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>' in session_environment
    assert 'OCTOPLANNER_MAP_PATH="/maps/plant-a/octomap.ot"' in session_environment
    assert 'FAR_OCCUPANCY_PATH=""' in session_environment
    assert 'EXPLORE_OCCUPANCY_PATH=""' in session_environment
    assert 'LINGTU_NAV_SEGMENT_MAX_DISTANCE_M="5.0"' in session_environment
    assert "LINGTU_ACTIVE_" not in session_environment
    backend.rollback_session(staged)
    assert files == {}


def test_stop_confirms_native_zero_without_gateway_session_call(monkeypatch) -> None:
    commands: list[object] = []
    backend = FieldBackend(environment={})
    monkeypatch.setattr(backend, "_unit_active", lambda _target: True)
    monkeypatch.setattr(
        backend,
        "_run",
        lambda command, **_kwargs: commands.append(command),
    )
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: pytest.fail("stop must not call Gateway session APIs"),
    )

    backend.stop_motion("nav")

    assert commands == [
        [
            "/opt/lingtu/current/bin/lingtu_nav_control",
            "stop",
            "product_mode_switch",
            "--timeout-ms",
            "7000",
        ]
    ]


@pytest.mark.parametrize("control_mode", ["teleop", "teleop_avoid", "autonomy"])
@pytest.mark.parametrize("permission", ["CLEAR", "HELD", "ESTOPPED"])
def test_navigation_startup_does_not_rejudge_motion(monkeypatch, control_mode, permission):
    backend = FieldBackend(environment={})
    navigation = {
        "task": {"state": "IDLE"},
        "goal_admission": {"state": "BLOCKED"},
        "motion": {"permission": permission, "stop_confirmation": "FAILED"},
    }
    def http(method, path, *, timeout_s):
        assert method == "GET"
        assert path == "/api/v1/navigation/status"
        return navigation
    monkeypatch.setattr(backend, "_http", http)
    backend.wait_navigation(map_name="", control_mode=control_mode, timeout_s=1.0)
    assert navigation["goal_admission"]["state"] == "BLOCKED"
    assert navigation["motion"]["permission"] == permission
