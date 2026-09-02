# ruff: noqa: S106

from __future__ import annotations

import io
import json
import urllib.error
from pathlib import Path
from types import SimpleNamespace

import pytest

from lingtu.control import ProductControl
from lingtu.real.backend import (
    FieldBackend,
    MapActivationToken,
    SessionFile,
    SessionStage,
    motion_output_startup_evidence,
)
from lingtu.run_plan import RunPlan
from lingtu.switch_contracts import (
    MapArtifactIdentity,
    MapIdentity,
    ProcessReport,
    SwitchFailed,
    SwitchRequest,
)


def _go2_motion_startup_status() -> tuple[dict, dict]:
    producer = "nav-boot-1"
    sequence = 17
    zero = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    nav = {
        "schema_version": "lingtu.nav.endpoint.status.v1",
        "stamp_s": 100.0,
        "control_mode": "teleop_avoid",
        "final_cmd_vel": zero,
        "operator_motion": {
            "interface_enabled": True,
            "authority_owner": "native_endpoint",
            "control_mode": "teleop_avoid",
            "control_ack_scope": "claim_hold_release",
            "sample_evidence": "status_sequences",
            "ack_publish_failed": 0,
            "status_publish_failed": 0,
            "status": {
                "has_active_authority": False,
                "has_active_sample": False,
                "final_cmd_vel": zero,
            },
        },
        "final_output": {
            "published": True,
            "producer_boot_id": producer,
            "output_sequence": sequence,
            "driver_delivery_accepted": True,
        },
        "driver_control": {
            "received": True,
            "ready": True,
            "fresh": True,
            "last_command_accepted": True,
            "accepted_producer_boot_id": producer,
            "accepted_output_sequence": sequence,
        },
    }
    driver = {
        "schema_version": "lingtu.driver.status.v2",
        "role": "driver",
        "backend": "go2",
        "stamp_s": 100.0,
        "connected": True,
        "ready": True,
        "dds": {
            "topic": "/nav/cmd_vel",
            "wire_topic": "rt/nav/cmd_vel",
            "cmd_vel_writer_ready": True,
            "matched_cmd_vel_writers": 1,
        },
        "adapter": {
            "protocol": "unitree_sdk2",
            "target": "dds://eth0/rt/api/sport/request",
            "control_owner": "none",
            "control_owner_id": "",
        },
        "control": {
            "initial_zero_acknowledged": True,
            "motors_enabled": True,
            "critical_fault": False,
            "control_assured": True,
            "lease_valid": False,
            "fsm": "standing",
        },
        "output_ack": {
            "accepted": True,
            "producer_boot_id": producer,
            "output_sequence": sequence,
        },
    }
    return nav, driver


def test_go2_startup_uses_control_assurance_without_fabricated_sdk_lease() -> None:
    nav, driver = _go2_motion_startup_status()

    reason, _ = motion_output_startup_evidence(
        nav,
        driver,
        expected_mode="teleop_avoid",
        expected_driver={"backend": "go2", "network_interface": "eth0"},
        wall_clock_s=100.1,
        nav_max_age_s=1.0,
        driver_max_age_s=1.0,
    )

    assert reason == ""

    driver["adapter"]["control_owner"] = "sdk2"
    driver["adapter"]["control_owner_id"] = "lingtu-driver@robot"
    driver["control"]["lease_valid"] = True
    reason, _ = motion_output_startup_evidence(
        nav,
        driver,
        expected_mode="teleop_avoid",
        expected_driver={"backend": "go2", "network_interface": "eth0"},
        wall_clock_s=100.1,
        nav_max_age_s=1.0,
        driver_max_age_s=1.0,
    )
    assert reason == "driver_control_not_ready"


def test_startup_zero_accepts_fresh_ack_snapshots_that_lag_the_20hz_output() -> None:
    nav, driver = _go2_motion_startup_status()
    nav["final_output"]["output_sequence"] = 20
    nav["final_output"]["driver_delivery_accepted"] = False
    nav["driver_control"]["accepted_output_sequence"] = 19
    driver["output_ack"]["output_sequence"] = 9

    reason, _ = motion_output_startup_evidence(
        nav,
        driver,
        expected_mode="teleop_avoid",
        expected_driver={"backend": "go2", "network_interface": "eth0"},
        wall_clock_s=100.1,
        nav_max_age_s=1.0,
        driver_max_age_s=1.0,
    )

    assert reason == ""


def test_startup_zero_rejects_a_driver_control_ack_more_than_two_outputs_behind() -> None:
    nav, driver = _go2_motion_startup_status()
    nav["final_output"]["output_sequence"] = 20
    nav["final_output"]["driver_delivery_accepted"] = False
    nav["driver_control"]["accepted_output_sequence"] = 17
    driver["output_ack"]["output_sequence"] = 17

    reason, _ = motion_output_startup_evidence(
        nav,
        driver,
        expected_mode="teleop_avoid",
        expected_driver={"backend": "go2", "network_interface": "eth0"},
        wall_clock_s=100.1,
        nav_max_age_s=1.0,
        driver_max_age_s=1.0,
    )

    assert reason == "startup_zero_ack_identity_mismatch"


class FakeRunner:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        self.calls.append("apply")
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
            dry_run=dry_run,
        )

    def apply_deferred(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        return self.apply(plan, dry_run=dry_run)

    def transition(
        self,
        previous: RunPlan,
        plan: RunPlan,
        *,
        dry_run: bool = False,
        defer_rollback: bool = False,
    ) -> ProcessReport:
        assert defer_rollback is True
        return self.apply(plan, dry_run=dry_run)

    def ensure_transition_process_active(self, plan, transition, process_name):
        return transition

    def stop_transition_target(self, plan, transition):
        self.calls.append("stop_target")
        return transition

    def restore_transition_previous(self, previous, transition):
        self.calls.append("restore_previous")
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

    def wait_motion_output(self, control_mode: str, *, timeout_s: float) -> None:
        self._event("wait_motion_output")

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
        "stage_map",
        "stage_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "prepare_localization",
        "wait_slam_localized",
        "wait_navigation",
        "commit_map",
    ]


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
            "detail": "maps.service is unavailable; ProductGraph/Blueprint must inject maps.service",
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


def test_localization_reuse_requires_healthy_saved_map_tracking(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    status = {
        "state": "ready",
        "ready": True,
        "active_map": "plant-a",
        "map_loaded": True,
        "pose_fresh": True,
        "map_odom_tf": {
            "valid": True,
            "frame_id": "map",
            "child_frame_id": "odom",
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "ts": 1.0,
        },
        "raw": {},
    }
    monkeypatch.setattr(backend, "_http", lambda *_args, **_kwargs: status)

    assert backend._localization_reusable("plant-a") is False

    status["raw"] = {
        "track_against_map": {
            "enabled": True,
            "successes": 1,
            "degraded": False,
        }
    }
    assert backend._localization_reusable("plant-a") is True


def test_prepare_localization_relocalizes_when_saved_map_tracking_is_not_reusable(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    status = {
        "state": "ready",
        "ready": True,
        "active_map": "plant-a",
        "map_loaded": True,
        "pose_fresh": True,
        "map_odom_tf": {
            "valid": True,
            "frame_id": "map",
            "child_frame_id": "odom",
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "ts": 1.0,
        },
        "raw": {"track_against_map": {"enabled": True, "successes": 0, "degraded": False}},
    }

    def http(method, path, *_args, **_kwargs):
        assert (method, path) == ("GET", "/api/v1/localization/status")
        return status

    relocalizations = []
    monkeypatch.setattr(backend, "_http", http)
    monkeypatch.setattr(
        backend,
        "_relocalize",
        lambda map_name, *, initial_pose: relocalizations.append((map_name, initial_pose)),
    )

    backend.prepare_localization(
        SimpleNamespace(session_mode="navigating", product="nav", slam_mode="localization"),
        map_name="plant-a",
        relocalize=False,
        initial_pose=None,
    )

    assert relocalizations == [("plant-a", None)]


def test_teleop_avoid_startup_accepts_safe_native_hold_before_commit(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    calls: list[str] = []
    navigation = {
        "readiness": {
            "blockers": ["real_runtime_evidence_missing_or_stale"],
            "native_endpoint": {
                "ok": False,
                "status_available": True,
                "blockers": ["native_resume_required"],
                "input_gate": {"ready": True},
                "control_loop_health": {"ready": True, "healthy": True},
            },
        }
    }
    def http(_method, path, *, timeout_s):
        assert timeout_s == 3.0
        calls.append(path)
        return navigation

    monkeypatch.setattr(backend, "_http", http)

    backend.wait_navigation(
        map_name="",
        control_mode="teleop_avoid",
        timeout_s=1.0,
    )

    assert calls == ["/api/v1/navigation/status"]


def test_saved_map_navigation_startup_still_checks_active_map(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    calls: list[str] = []

    def http(_method, path, *, timeout_s):
        assert timeout_s == 3.0
        calls.append(path)
        if path == "/api/v1/navigation/status":
            return {"readiness": {"can_accept_goal": True}}
        return {"active_map": "plant-a"}

    monkeypatch.setattr(backend, "_http", http)

    backend.wait_navigation(
        map_name="plant-a",
        control_mode="autonomy",
        timeout_s=1.0,
    )

    assert calls == ["/api/v1/navigation/status", "/api/v1/session"]


def test_motion_stop_confirmation_blocks_process_mutation(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="stop_motion")

    with pytest.raises(SwitchFailed, match="failed at stop_motion") as failure:
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "stop_unconfirmed"
    assert runner.calls == []
    assert backend.events == ["stop_motion"]


def test_failed_switch_restores_map_quiesces_processes_and_removes_session(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="prepare_localization")

    with pytest.raises(SwitchFailed, match="failed at prepare_localization") as failure:
        _control(runner)._switch(_request(), backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "failed_stopped"
    assert runner.calls == ["apply", "quiesce"]
    assert backend.events[-3:] == [
        "stop_motion",
        "restore_map",
        "rollback_session",
    ]
    assert not (tmp_path / "current.json").exists()


def test_failed_switch_restores_previous_run_and_keeps_committed_record(tmp_path) -> None:
    runner = FakeRunner()
    control = _control(runner)
    control._switch(
        SwitchRequest(target_product="teleop"),
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


@pytest.mark.parametrize(
    ("initial_pose", "expected_payload"),
    [
        (None, {"map_name": "plant-a", "mode": "global"}),
        (
            (1.0, 2.0, 0.3),
            {
                "map_name": "plant-a",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            },
        ),
    ],
)
def test_relocalize_uses_localization_domain_api(
    monkeypatch,
    initial_pose,
    expected_payload,
) -> None:
    calls: list[tuple[str, str, object, float]] = []
    backend = FieldBackend(environment={})

    def http(method, path, payload, *, timeout_s):
        calls.append((method, path, payload, timeout_s))
        return {"ok": True, "success": True}

    monkeypatch.setattr(backend, "_http", http)

    backend._relocalize("plant-a", initial_pose=initial_pose)

    assert calls == [
        (
            "POST",
            "/api/v1/localization/relocalizations",
            expected_payload,
            35.0,
        )
    ]


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
