# ruff: noqa: S101, S108

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

import lingtu.product_switch as product_switch
from lingtu.control import ProductControl
from lingtu.product_switch import (
    FieldBackend,
    MapActivationToken,
    MapArtifactIdentity,
    MapIdentity,
    SessionFile,
    SessionStage,
    SwitchFailed,
    SwitchRequest,
    session_explanation,
)
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA, RunPlan
from lingtu.systemd import ProcessReport
from runtime.graph.processes import ProcessSpec


class FakeRunner:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def apply(self, plan, *, dry_run: bool = False):
        self.calls.append(("apply", plan.fingerprint))
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
            dry_run=dry_run,
        )

    def quiesce(self, plan, *, dry_run: bool = False):
        self.calls.append(("quiesce", plan.fingerprint))
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

    def _event(self, name: str) -> None:
        self.events.append(name)
        if self.fail_at == name:
            raise RuntimeError(f"failed at {name}")

    def current_product(self) -> str:
        self._event("current_product")
        return "teleop"

    def assert_map_save_idle(self) -> None:
        return None

    def assert_recording_idle(self) -> None:
        return None

    def stop_motion_and_session(self, current_product=None) -> None:
        assert current_product in {"teleop", "nav", None}
        self._event("stop_motion_and_session")

    def stage_map(self, map_name: str) -> MapActivationToken:
        assert map_name == "plant-a"
        self._event("stage_map")
        return MapActivationToken(
            target=_exact_identity(map_name, 7),
            previous=MapIdentity(
                map_id="warehouse",
                version_id="warehouse:v3",
                frame_id="map",
                map_dir="/maps/warehouse/.versions/3",
                artifacts=(
                    MapArtifactIdentity(
                        artifact_type="POINTCLOUD",
                        uri="/maps/warehouse/.versions/3/map.pcd",
                        sha256="warehouse-v3-sha",
                    ),
                ),
            ),
            changed=True,
            activation_token="opaque-activation-token",
        )

    def restore_map(self, token: MapActivationToken) -> None:
        assert token.target.map_id == "plant-a"
        assert token.previous is not None
        assert token.previous.map_id == "warehouse"
        self._event("restore_map")

    def commit_map(self, token: MapActivationToken) -> None:
        assert token.target.map_id == "plant-a"
        self._event("commit_map")

    def stage_session(
        self,
        run_plan_path: Path,
        plan,
        native_environment,
        *,
        slam_mode: str,
        map_path: str,
        map_identity: MapIdentity | None = None,
        product_session_id: str | None = None,
        parameter_overrides=None,
    ) -> SessionStage:
        assert run_plan_path.is_file()
        assert plan.product == "nav"
        assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
        assert slam_mode == "localization"
        assert map_path == "/maps/plant-a/.versions/7/map.pcd"
        assert map_identity == _exact_identity("plant-a", 7)
        assert parameter_overrides == {}
        assert product_session_id is not None
        assert len(product_session_id) == 32
        self._event("stage_session")
        return SessionStage(files=(SessionFile(path="/run/lingtu/session.env"),))

    def rollback_session(self, _staged: SessionStage) -> None:
        self._event("rollback_session")

    def clear_runtime_status(self) -> None:
        self._event("clear_runtime_status")

    def wait_native_nav(self, native_environment, *, timeout_s: float) -> None:
        assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
        assert timeout_s == 10.0
        self._event("wait_native_nav")

    def wait_slam(self, mode: str, *, require_map: bool, timeout_s: float) -> None:
        assert mode == "localization"
        assert require_map is True
        assert timeout_s == 35.0
        self._event("wait_slam")

    def start_session(
        self,
        contract,
        *,
        map_name: str,
        relocalize: bool,
        initial_pose,
    ) -> None:
        assert contract.product == "nav"
        assert map_name == "plant-a"
        assert relocalize is True
        assert initial_pose == (1.0, 2.0, 0.3)
        self._event("start_session")

    def wait_navigation(
        self,
        *,
        map_name: str,
        control_mode: str,
        timeout_s: float,
    ) -> None:
        assert map_name == "plant-a"
        assert control_mode == "autonomy"
        assert timeout_s == 45.0
        self._event("wait_navigation")

    def wait_inspection(self, *, timeout_s: float) -> None:
        self._event("wait_inspection")


def _control(runner: FakeRunner) -> ProductControl:
    return ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]


def _field_plan(*, include_explore: bool = False) -> RunPlan:
    available = (
        ProcessSpec("slam", "systemd", "lingtu-slam-dds.service", 20, 35, "mode"),
        ProcessSpec("nav", "systemd", "lingtu-nav-dds.service", 40, 20, "mode"),
        ProcessSpec("driver", "systemd", "lingtu-driver.service", 50, 20, "persistent"),
        ProcessSpec("explore", "systemd", "lingtu-explore-dds.service", 70, 20, "mode"),
        ProcessSpec("host", "systemd", "lingtu.service", 90, 45, "mode"),
    )
    selected_names = {"slam", "nav", "driver", "host"}
    if include_explore:
        selected_names.add("explore")
    selected = tuple(process for process in available if process.name in selected_names)
    product = "explore" if include_explore else "nav"
    product_variant = "live" if include_explore else None
    return RunPlan(
        schema_version=RUN_PLAN_SCHEMA,
        product=product,
        product_variant=product_variant,
        env="real",
        process_control="systemd",
        modules=("GatewayModule",),
        processes=selected,
        available_processes=available,
        stop_targets=tuple(process.target for process in reversed(available) if process.lifecycle == "mode"),
        contracts=(),
        critical_modules=("GatewayModule",),
        route_contract=None,
        module_transport="local",
        parameter_profile=None,
        _native_process_environment_json="{}",
        _host_config_json=json.dumps(
            {
                "_endpoint_transport": "dds",
                "_endpoint_contract": "thunder_dds_v1",
                "command_output_mode": "endpoint_only",
                "hardware_control_boundary": "driver",
            }
        ),
        _lifecycle_json=json.dumps({"product": product, "product_variant": product_variant}),
        _parameter_overrides_json="{}",
        _compiled_against_json="{}",
        fingerprint="a" * 64,
    )


def _request() -> SwitchRequest:
    return SwitchRequest(
        target_product="nav",
        current_product="teleop",
        map_name="plant-a",
        relocalize=True,
        initial_pose=(1.0, 2.0, 0.3),
    )


def _exact_identity(name: str, version: int) -> MapIdentity:
    return MapIdentity(
        map_id=name,
        version_id=f"{name}:v{version}",
        frame_id="map",
        map_dir=f"/maps/{name}/.versions/{version}",
        artifacts=(
            MapArtifactIdentity(
                artifact_type="POINTCLOUD",
                uri=f"/maps/{name}/.versions/{version}/map.pcd",
                sha256=f"{name}-v{version}-sha",
            ),
        ),
    )


def _exact_identity_payload(name: str, version: int) -> dict[str, object]:
    return {
        "present": True,
        "map_id": name,
        "version_id": f"{name}:v{version}",
        "frame_id": "map",
        "map_dir": f"/maps/{name}/.versions/{version}",
        "artifacts": [
            {
                "type": "POINTCLOUD",
                "uri": f"/maps/{name}/.versions/{version}/map.pcd",
                "sha256": f"{name}-v{version}-sha",
            }
        ],
    }


def _write_committed_explore(
    control: ProductControl,
    state_dir: Path,
    *,
    map_name: str | None = None,
    map_id: str | None = None,
) -> tuple[SwitchRequest, RunPlan, MapIdentity | None, str]:
    request = SwitchRequest(target_product="explore", map_name=map_name)
    plan = control.resolve(
        request.target_product,
        product_variant=request.product_variant,
    )
    plan_path = state_dir / f"plan-{plan.fingerprint}.json"
    plan.write(plan_path)
    map_identity = _exact_identity(map_id or map_name, 7) if map_name else None
    product_session_id = "e" * 32
    (state_dir / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": "explore",
                "product_variant": request.product_variant,
                "env": plan.env,
                "run_plan_path": str(plan_path),
                "fingerprint": plan.fingerprint,
                "product_session_id": product_session_id,
                "map_name": map_name,
                "map_identity": (
                    {
                        "map_id": map_identity.map_id,
                        "version_id": map_identity.version_id,
                        "frame_id": map_identity.frame_id,
                        "map_dir": map_identity.map_dir,
                        "artifacts": [
                            {
                                "artifact_type": artifact.artifact_type,
                                "uri": artifact.uri,
                                "sha256": artifact.sha256,
                            }
                            for artifact in map_identity.artifacts
                        ],
                    }
                    if map_identity is not None
                    else None
                ),
                "committed_at": 1.0,
            }
        ),
        encoding="utf-8",
    )
    return request, plan, map_identity, product_session_id


class HealthyCommittedExploreBackend(FakeBackend):
    def __init__(
        self,
        plan: RunPlan,
        *,
        map_identity: MapIdentity | None,
        product_session_id: str,
        fail_readiness: str | None = None,
    ) -> None:
        super().__init__()
        self.plan = plan
        self.map_identity = map_identity
        self.product_session_id = product_session_id
        self.fail_readiness = fail_readiness

    def assert_map_save_idle(self) -> None:
        raise AssertionError("already-active detection must precede switch effects")

    def assert_recording_idle(self) -> None:
        raise AssertionError("already-active detection must precede switch effects")

    def stop_motion_and_session(self, current_product=None) -> None:
        raise AssertionError("a proven healthy Explore Product must not be stopped")

    def wait_native_nav(self, native_environment, *, timeout_s: float) -> None:
        assert native_environment == self.plan.native_process_environment
        assert timeout_s > 0.0
        self.events.append("wait_native_nav")
        if self.fail_readiness == "nav":
            raise RuntimeError("native nav unhealthy")

    def wait_slam(self, mode: str, *, require_map: bool, timeout_s: float) -> None:
        assert mode == ("localization" if self.map_identity is not None else "mapping")
        assert require_map is (self.map_identity is not None)
        assert timeout_s > 0.0
        self.events.append("wait_slam")
        if self.fail_readiness == "slam":
            raise RuntimeError("SLAM unhealthy")

    def wait_exploration(
        self,
        route,
        *,
        map_identity,
        product_session_id,
        timeout_s,
        allow_active=False,
    ) -> None:
        assert route == ("map" if self.map_identity is not None else "live")
        assert map_identity == self.map_identity
        assert product_session_id == self.product_session_id
        assert timeout_s > 0.0
        assert allow_active is True
        self.events.append("wait_exploration")
        if self.fail_readiness == "explore":
            raise RuntimeError("Explore endpoint unhealthy")


def _mapctl_response(
    operation: str,
    *,
    active: dict[str, object],
    target: dict[str, object] | None = None,
    previous: dict[str, object] | None = None,
    changed: bool = True,
    activation_token: str = "opaque-activation-token",
) -> SimpleNamespace:
    payload = {
        "schema_version": "lingtu.map_activation.v2",
        "request_id": "mapctl-test",
        "operation": operation,
        "accepted": True,
        "message": "ok",
        "changed": changed,
        "producer_boot_id": "mapd-boot",
        "activation_token": activation_token,
        "target": target or {"present": False},
        "previous": previous or {"present": False},
        "active": active,
    }
    return SimpleNamespace(returncode=0, stdout=json.dumps(payload), stderr="")


def test_field_backend_stages_map_through_native_mapctl(tmp_path) -> None:
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        return _mapctl_response(
            "stage",
            target=_exact_identity_payload("plant-a", 7),
            previous=_exact_identity_payload("warehouse", 3),
            active=_exact_identity_payload("plant-a", 7),
        )

    backend = FieldBackend(
        environment={
            "NAV_MAP_DIR": str(tmp_path),
            "LINGTU_MAPCTL_BIN": "/release/lingtu-mapctl",
            "LINGTU_DDS_DOMAIN_ID": "17",
        },
        runner=run,
    )

    token = backend.stage_map("plant-a")

    assert token.target == _exact_identity("plant-a", 7)
    assert token.previous == _exact_identity("warehouse", 3)
    assert token.activation_token == "opaque-activation-token"
    assert calls == [
        [
            "/release/lingtu-mapctl",
            "stage",
            "plant-a",
            "--map-root",
            str(tmp_path.resolve()),
            "--caller",
            "product-control",
            "--domain-id",
            "17",
        ]
    ]


def test_field_backend_restores_exact_map_through_opaque_token(tmp_path) -> None:
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        return _mapctl_response(
            "restore",
            active=_exact_identity_payload("warehouse", 3),
        )

    backend = FieldBackend(environment={"NAV_MAP_DIR": str(tmp_path)}, runner=run)
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=_exact_identity("warehouse", 3),
        changed=True,
        activation_token="opaque-activation-token",
    )

    backend.restore_map(token)

    assert calls[0][1:3] == ["restore", "opaque-activation-token"]


def test_field_backend_restore_verifies_cleared_previous_map(tmp_path) -> None:
    backend = FieldBackend(
        environment={"NAV_MAP_DIR": str(tmp_path)},
        runner=lambda *_args, **_kwargs: _mapctl_response(
            "restore",
            active={"present": False},
        ),
    )
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=None,
        changed=True,
        activation_token="opaque-activation-token",
    )

    backend.restore_map(token)


def test_field_backend_commit_is_native_identity_verification(tmp_path) -> None:
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        return _mapctl_response(
            "verify",
            active=_exact_identity_payload("plant-a", 7),
        )

    backend = FieldBackend(environment={"NAV_MAP_DIR": str(tmp_path)}, runner=run)
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=None,
        changed=True,
        activation_token="opaque-activation-token",
    )

    backend.commit_map(token)

    assert calls[0][1:3] == ["verify", "opaque-activation-token"]


def test_field_backend_rejects_native_map_identity_drift(tmp_path) -> None:
    backend = FieldBackend(
        environment={"NAV_MAP_DIR": str(tmp_path)},
        runner=lambda *_args, **_kwargs: _mapctl_response(
            "verify",
            active=_exact_identity_payload("plant-a", 8),
        ),
    )
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=None,
        changed=True,
        activation_token="opaque-activation-token",
    )

    with pytest.raises(RuntimeError, match="identity changed"):
        backend.commit_map(token)


def test_field_backend_surfaces_native_mapctl_rejection(tmp_path) -> None:
    response = _mapctl_response("stage", active={"present": False})
    payload = json.loads(response.stdout)
    payload.update({"accepted": False, "message": "request_id_conflict"})
    response.returncode = 2
    response.stdout = json.dumps(payload)
    backend = FieldBackend(
        environment={"NAV_MAP_DIR": str(tmp_path)},
        runner=lambda *_args, **_kwargs: response,
    )

    with pytest.raises(RuntimeError, match="request_id_conflict"):
        backend.stage_map("plant-a")


def test_product_control_owns_the_complete_cold_switch_order(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()

    report = _control(runner).switch(
        _request(),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    payload = report.as_dict()
    assert payload["current_product"] == "teleop"
    assert payload["target_product"] == "nav"
    assert report.phases == [
        "preflight",
        "map_save_idle",
        "recording_idle",
        "plan_published",
        "motion_stopped",
        "previous_run_cleared",
        "map_prepared",
        "session_staged",
        "stale_status_cleared",
        "processes_active",
        "native_nav_ready",
        "slam_ready",
        "session_active",
        "navigation_ready",
        "map_committed",
        "committed",
    ]
    assert backend.events == [
        "stop_motion_and_session",
        "stage_map",
        "stage_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
        "wait_navigation",
        "commit_map",
    ]
    assert [call[0] for call in runner.calls] == ["apply"]
    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["product"] == "nav"
    assert current["product_variant"] is None
    assert current["map_identity"]["map_id"] == "plant-a"
    assert current["map_identity"]["version_id"] == "plant-a:v7"


def test_explore_switch_waits_for_idle_route_before_commit(tmp_path) -> None:
    class ExploreBackend(FakeBackend):
        def __init__(self) -> None:
            super().__init__()
            self.product_session_id = ""

        def stage_session(
            self,
            run_plan_path,
            plan,
            native_environment,
            *,
            slam_mode,
            map_path,
            map_identity=None,
            product_session_id=None,
            parameter_overrides=None,
        ):
            assert run_plan_path.is_file()
            assert plan.product == "explore"
            assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
            assert slam_mode == "mapping"
            assert map_path == ""
            assert map_identity is None
            assert parameter_overrides == {}
            assert product_session_id is not None
            self.product_session_id = product_session_id
            self._event("stage_session")
            return SessionStage(files=(SessionFile(path="/run/lingtu/session.env"),))

        def wait_slam(self, mode, *, require_map, timeout_s):
            assert mode == "mapping"
            assert require_map is False
            assert timeout_s == 35.0
            self._event("wait_slam")

        def start_session(self, lifecycle, *, map_name, relocalize, initial_pose):
            assert lifecycle.product == "explore"
            assert lifecycle.session_mode == "exploring"
            assert map_name == ""
            assert initial_pose is None
            self._event("start_session")

        def wait_exploration(
            self,
            route,
            *,
            map_identity,
            product_session_id,
            timeout_s,
        ):
            assert route == "live"
            assert map_identity is None
            assert product_session_id == self.product_session_id
            assert timeout_s == 45.0
            self._event("wait_exploration")

    runner = FakeRunner()
    backend = ExploreBackend()

    report = _control(runner).switch(
        SwitchRequest(target_product="explore", current_product="teleop"),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.phases[-3:] == ["session_active", "exploration_ready", "committed"]
    assert backend.events[-2:] == ["start_session", "wait_exploration"]
    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["product"] == "explore"
    assert current["product_variant"] == "live"
    assert current["map_identity"] is None


@pytest.mark.parametrize(
    ("map_name", "map_id"),
    [(None, None), ("plant-a", None), ("customer-alias", "01MAPRESOURCE")],
)
def test_repeated_healthy_explore_switch_is_already_active(
    tmp_path,
    map_name,
    map_id,
) -> None:
    runner = FakeRunner()
    control = _control(runner)
    request, plan, map_identity, product_session_id = _write_committed_explore(
        control,
        tmp_path,
        map_name=map_name,
        map_id=map_id,
    )
    backend = HealthyCommittedExploreBackend(
        plan,
        map_identity=map_identity,
        product_session_id=product_session_id,
    )

    report = control.switch(request, backend=backend, state_dir=tmp_path)

    assert report.ok is True
    assert report.status == "already_active"
    assert report.fingerprint == plan.fingerprint
    assert report.product_variant == request.product_variant
    assert report.phases == [
        "preflight",
        "existing_native_nav_ready",
        "existing_slam_ready",
        "existing_exploration_ready",
        "already_active",
    ]
    assert backend.events == [
        "wait_native_nav",
        "wait_slam",
        "wait_exploration",
    ]
    assert runner.calls == []
    committed = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert committed["product_session_id"] == product_session_id


def test_explore_saved_map_change_performs_cold_switch(tmp_path) -> None:
    class MapSwitchBackend(FakeBackend):
        def __init__(self) -> None:
            super().__init__()
            self.product_session_id = ""
            self.target_identity = _exact_identity("01MAP-B", 9)

        def stop_motion_and_session(self, current_product=None) -> None:
            assert current_product == "explore"
            self._event("stop_motion_and_session")

        def stage_map(self, map_name: str) -> MapActivationToken:
            assert map_name == "customer-map-b"
            self._event("stage_map")
            return MapActivationToken(
                target=self.target_identity,
                previous=_exact_identity("01MAP-A", 7),
                changed=True,
                activation_token="map-b-activation",
            )

        def stage_session(
            self,
            run_plan_path,
            plan,
            native_environment,
            *,
            slam_mode,
            map_path,
            map_identity=None,
            product_session_id=None,
            parameter_overrides=None,
        ):
            assert run_plan_path.is_file()
            assert plan.product == "explore"
            assert plan.product_variant == "map"
            assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
            assert slam_mode == "localization"
            assert map_path == "/maps/01MAP-B/.versions/9/map.pcd"
            assert map_identity == self.target_identity
            assert parameter_overrides == {}
            assert product_session_id is not None
            self.product_session_id = product_session_id
            self._event("stage_session")
            return SessionStage(files=(SessionFile(path="/run/lingtu/session.env"),))

        def wait_native_nav(self, native_environment, *, timeout_s):
            assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
            assert timeout_s == 10.0
            self._event("wait_native_nav")

        def wait_slam(self, mode, *, require_map, timeout_s):
            assert mode == "localization"
            assert require_map is True
            assert timeout_s == 35.0
            self._event("wait_slam")

        def start_session(self, lifecycle, *, map_name, relocalize, initial_pose):
            assert lifecycle.product == "explore"
            assert map_name == "customer-map-b"
            assert relocalize is True
            assert initial_pose is None
            self._event("start_session")

        def wait_exploration(
            self,
            route,
            *,
            map_identity,
            product_session_id,
            timeout_s,
        ):
            assert route == "map"
            assert map_identity == self.target_identity
            assert product_session_id == self.product_session_id
            assert timeout_s == 45.0
            self._event("wait_exploration")

        def commit_map(self, token):
            assert token.target == self.target_identity
            self._event("commit_map")

    runner = FakeRunner()
    control = _control(runner)
    _write_committed_explore(
        control,
        tmp_path,
        map_name="customer-map-a",
        map_id="01MAP-A",
    )
    backend = MapSwitchBackend()

    report = control.switch(
        SwitchRequest(target_product="explore", map_name="customer-map-b"),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    assert backend.events == [
        "stop_motion_and_session",
        "stage_map",
        "stage_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
        "wait_exploration",
        "commit_map",
    ]
    assert runner.calls == [("apply", report.fingerprint)]
    committed = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert committed["map_name"] == "customer-map-b"
    assert committed["map_identity"]["map_id"] == "01MAP-B"


@pytest.mark.parametrize(
    ("field_name", "replacement"),
    [
        ("product", "nav"),
        ("product_variant", "map"),
        ("fingerprint", "f" * 64),
        ("product_session_id", ""),
        ("map_identity", {"map_id": "foreign"}),
    ],
)
def test_explore_identity_mismatch_uses_cold_switch(
    tmp_path,
    field_name,
    replacement,
) -> None:
    runner = FakeRunner()
    control = _control(runner)
    request, plan, map_identity, product_session_id = _write_committed_explore(
        control,
        tmp_path,
    )
    current_path = tmp_path / "current.json"
    committed = json.loads(current_path.read_text(encoding="utf-8"))
    committed[field_name] = replacement
    current_path.write_text(json.dumps(committed), encoding="utf-8")
    backend = HealthyCommittedExploreBackend(
        plan,
        map_identity=map_identity,
        product_session_id=product_session_id,
    )
    backend.assert_map_save_idle = lambda: (_ for _ in ()).throw(RuntimeError("cold switch reached"))

    with pytest.raises(SwitchFailed, match="cold switch reached") as failure:
        control.switch(request, backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "failed"
    assert backend.events == []
    assert runner.calls == []


def test_legacy_explore_record_without_map_name_uses_cold_switch(tmp_path) -> None:
    runner = FakeRunner()
    control = _control(runner)
    request, plan, map_identity, product_session_id = _write_committed_explore(
        control,
        tmp_path,
        map_name="customer-alias",
        map_id="01MAPRESOURCE",
    )
    current_path = tmp_path / "current.json"
    committed = json.loads(current_path.read_text(encoding="utf-8"))
    del committed["map_name"]
    current_path.write_text(json.dumps(committed), encoding="utf-8")
    backend = HealthyCommittedExploreBackend(
        plan,
        map_identity=map_identity,
        product_session_id=product_session_id,
    )
    backend.assert_map_save_idle = lambda: (_ for _ in ()).throw(RuntimeError("cold switch reached"))

    with pytest.raises(SwitchFailed, match="cold switch reached"):
        control.switch(request, backend=backend, state_dir=tmp_path)

    assert backend.events == []
    assert runner.calls == []


def test_unhealthy_committed_explore_uses_cold_switch(tmp_path) -> None:
    runner = FakeRunner()
    control = _control(runner)
    request, plan, map_identity, product_session_id = _write_committed_explore(
        control,
        tmp_path,
    )
    backend = HealthyCommittedExploreBackend(
        plan,
        map_identity=map_identity,
        product_session_id=product_session_id,
        fail_readiness="explore",
    )
    backend.assert_map_save_idle = lambda: (_ for _ in ()).throw(RuntimeError("cold switch reached"))

    with pytest.raises(SwitchFailed, match="cold switch reached") as failure:
        control.switch(request, backend=backend, state_dir=tmp_path)

    assert failure.value.report.status == "failed"
    assert backend.events == [
        "wait_native_nav",
        "wait_slam",
        "wait_exploration",
    ]
    assert runner.calls == []


def test_product_control_switches_map_through_session_staging_and_commit(
    monkeypatch,
    tmp_path,
) -> None:
    runner = FakeRunner()
    installed: list[tuple[str, str]] = []
    events: list[str] = []

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("field switch must not invoke a real command runner")

    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=reject_real_command,
    )
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: installed.append((target, content)),
    )
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)
    monkeypatch.setattr(
        backend,
        "stop_motion_and_session",
        lambda _current: events.append("stop_motion_and_session"),
    )
    monkeypatch.setattr(backend, "assert_map_save_idle", lambda: None)
    monkeypatch.setattr(backend, "assert_recording_idle", lambda: None)
    monkeypatch.setattr(
        backend,
        "clear_runtime_status",
        lambda: events.append("clear_runtime_status"),
    )
    monkeypatch.setattr(
        backend,
        "wait_native_nav",
        lambda _environment, **_kwargs: events.append("wait_native_nav"),
    )
    monkeypatch.setattr(
        backend,
        "wait_slam",
        lambda _mode, **_kwargs: events.append("wait_slam"),
    )
    monkeypatch.setattr(
        backend,
        "start_session",
        lambda _contract, **_kwargs: events.append("start_session"),
    )
    report = _control(runner).switch(
        SwitchRequest(
            target_product="map",
            current_product="teleop",
        ),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    assert report.run_plan is not None
    assert report.run_plan["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert report.phases == [
        "preflight",
        "map_save_idle",
        "recording_idle",
        "plan_published",
        "motion_stopped",
        "previous_run_cleared",
        "session_staged",
        "stale_status_cleared",
        "processes_active",
        "native_nav_ready",
        "slam_ready",
        "session_active",
        "committed",
    ]
    assert events == [
        "stop_motion_and_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
    ]
    assert runner.calls == [("apply", report.fingerprint)]

    assert report.run_plan_path is not None
    run_plan_path = Path(report.run_plan_path)
    plan = RunPlan.load(run_plan_path)
    host_config = plan.host_config
    assert len(installed) == 1
    target, session_environment = installed[0]
    assert target == str(tmp_path / "session.env")
    escaped_state_dir = str(tmp_path).replace("\\", "\\\\")
    assert f'LINGTU_SESSION_ROOT="{escaped_state_dir}"' in session_environment
    for config_key, environment_key in (
        ("_endpoint_transport", "LINGTU_ENDPOINT_TRANSPORT"),
        ("_endpoint_contract", "LINGTU_ENDPOINT_CONTRACT"),
        ("command_output_mode", "LINGTU_COMMAND_OUTPUT_MODE"),
        ("hardware_control_boundary", "LINGTU_HARDWARE_CONTROL_BOUNDARY"),
    ):
        assert f'{environment_key}="{host_config[config_key]}"' in session_environment

    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["schema_version"] == CURRENT_RUN_SCHEMA
    assert current["product"] == "map"
    assert current["product_variant"] is None
    assert current["run_plan_path"] == str(run_plan_path)
    assert current["fingerprint"] == report.fingerprint
    assert len(current["product_session_id"]) == 32
    assert current["map_name"] is None
    assert current["map_identity"] is None


def test_product_switch_removes_staged_session_and_uncommitted_plan(
    monkeypatch,
    tmp_path,
) -> None:
    class FailingRunner(FakeRunner):
        def apply(self, product, *, dry_run: bool = False):
            super().apply(product, dry_run=dry_run)
            raise RuntimeError("apply failed after staging")

    runner = FailingRunner()
    session_files: dict[str, str] = {}

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("rollback test must not invoke a real command runner")

    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=reject_real_command,
    )
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: session_files.__setitem__(target, content),
    )
    monkeypatch.setattr(
        backend,
        "_remove_runtime_file",
        lambda target, **_kwargs: session_files.pop(target, None),
    )
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)
    monkeypatch.setattr(backend, "assert_map_save_idle", lambda: None)
    monkeypatch.setattr(backend, "assert_recording_idle", lambda: None)
    monkeypatch.setattr(backend, "stop_motion_and_session", lambda _current: None)
    monkeypatch.setattr(backend, "clear_runtime_status", lambda: None)

    with pytest.raises(SwitchFailed, match="apply failed after staging") as failure:
        _control(runner).switch(
            SwitchRequest(
                target_product="map",
                current_product="teleop",
            ),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.run_plan_path is not None
    assert not Path(report.run_plan_path).exists()
    assert not (tmp_path / "current.json").exists()
    assert session_files == {}
    assert report.cleanup == [
        "motion_session:stopped",
        "processes:stopped",
        "session:removed",
        "plan:removed",
    ]


def test_product_switch_reports_staging_and_rollback_failures(
    monkeypatch,
    tmp_path,
) -> None:
    runner = FakeRunner()

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("staging failure test must not invoke a real command runner")

    def fail_install(_target: str, _content: str) -> None:
        raise RuntimeError("staging install failed")

    def fail_remove(_target: str, **_kwargs) -> None:
        raise RuntimeError("rollback remove failed")

    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=reject_real_command,
    )
    monkeypatch.setattr(backend, "_install_runtime_file", fail_install)
    monkeypatch.setattr(backend, "_remove_runtime_file", fail_remove)
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)
    monkeypatch.setattr(backend, "assert_map_save_idle", lambda: None)
    monkeypatch.setattr(backend, "assert_recording_idle", lambda: None)
    monkeypatch.setattr(backend, "stop_motion_and_session", lambda _current: None)
    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(
            SwitchRequest(
                target_product="map",
                current_product="teleop",
            ),
            backend=backend,
            state_dir=tmp_path,
        )

    assert failure.value.report.error is not None
    assert "staging install failed" in failure.value.report.error
    assert "rollback remove failed" in failure.value.report.error
    assert not (tmp_path / "current.json").exists()


def test_product_switch_applies_the_verified_persisted_plan(tmp_path) -> None:
    runner = FakeRunner()

    class TamperingBackend(FakeBackend):
        def stage_session(self, run_plan_path, plan, native_environment, **kwargs):
            staged = super().stage_session(
                run_plan_path,
                plan,
                native_environment,
                **kwargs,
            )
            payload = json.loads(run_plan_path.read_text(encoding="utf-8"))
            payload["identity"]["product"] = "teleop"
            run_plan_path.write_text(json.dumps(payload), encoding="utf-8")
            return staged

    backend = TamperingBackend()

    with pytest.raises(SwitchFailed, match="fingerprint mismatch"):
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    assert [call[0] for call in runner.calls] == ["quiesce"]
    assert not (tmp_path / "current.json").exists()


def test_product_switch_failure_quiesces_processes_and_removes_session(tmp_path) -> None:
    trace: list[str] = []

    class TracedRunner(FakeRunner):
        def apply(self, product, *, dry_run: bool = False):
            trace.append("runner.apply")
            return super().apply(product, dry_run=dry_run)

        def quiesce(self, product, *, dry_run: bool = False):
            trace.append("runner.quiesce")
            return super().quiesce(product, dry_run=dry_run)

    class TracedBackend(FakeBackend):
        def _event(self, name: str) -> None:
            trace.append(name)
            super()._event(name)

    runner = TracedRunner()
    backend = TracedBackend(fail_at="start_session")
    (tmp_path / "current.json").write_text('{"product": "teleop"}', encoding="utf-8")

    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.ok is False
    assert report.status == "failed_stopped"
    assert report.error == "failed at start_session"
    assert [call[0] for call in runner.calls] == ["apply", "quiesce"]
    assert trace[-5:] == [
        "start_session",
        "stop_motion_and_session",
        "restore_map",
        "runner.quiesce",
        "rollback_session",
    ]
    assert report.cleanup == [
        "motion_session:stopped",
        "map:restored",
        "processes:stopped",
        "session:removed",
        "plan:removed",
    ]
    assert not (tmp_path / "current.json").exists()


def test_explore_commit_failure_stops_and_quiesces_target(
    monkeypatch,
    tmp_path,
) -> None:
    runner = FakeRunner()
    backend = FakeBackend()

    def stop_motion(current_product=None) -> None:
        backend.events.append(f"stop:{current_product}")

    def stage_session(
        run_plan_path,
        plan,
        native_environment,
        **kwargs,
    ) -> SessionStage:
        assert run_plan_path.is_file()
        assert plan.product == "explore"
        assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
        assert kwargs["slam_mode"] == "mapping"
        assert kwargs["map_identity"] is None
        backend.events.append("stage_session")
        return SessionStage(files=(SessionFile(path="/run/lingtu/session.env"),))

    monkeypatch.setattr(backend, "stop_motion_and_session", stop_motion)
    monkeypatch.setattr(backend, "stage_session", stage_session)
    monkeypatch.setattr(
        backend,
        "wait_slam",
        lambda *_args, **_kwargs: backend.events.append("wait_slam"),
    )
    monkeypatch.setattr(
        backend,
        "start_session",
        lambda *_args, **_kwargs: backend.events.append("start_session"),
    )
    monkeypatch.setattr(
        backend,
        "wait_exploration",
        lambda *_args, **_kwargs: backend.events.append("wait_exploration"),
        raising=False,
    )
    monkeypatch.setattr(
        product_switch,
        "_commit_current_run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("current record write failed")),
    )

    with pytest.raises(SwitchFailed, match="current record write failed") as failure:
        _control(runner).switch(
            SwitchRequest(target_product="explore", current_product="teleop"),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.status == "failed_stopped"
    assert report.phases[-1] == "exploration_ready"
    assert backend.events == [
        "stop:teleop",
        "stage_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
        "wait_exploration",
        "stop:explore",
        "rollback_session",
    ]
    assert [call[0] for call in runner.calls] == ["apply", "quiesce"]
    assert "motion_session:stopped" in report.cleanup
    assert "processes:stopped" in report.cleanup
    assert "session:removed" in report.cleanup
    assert not (tmp_path / "current.json").exists()


def test_product_switch_does_not_quiesce_when_stop_is_unconfirmed(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at="stop_motion_and_session")
    current_path = tmp_path / "current.json"
    current_path.write_text('{"still_may_be_active": true}', encoding="utf-8")

    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.status == "stop_unconfirmed"
    assert report.phases[-1] == "plan_published"
    assert runner.calls == []
    assert backend.events == ["stop_motion_and_session"]
    assert report.cleanup == ["plan:removed"]
    assert current_path.read_text(encoding="utf-8") == '{"still_may_be_active": true}'


@pytest.mark.parametrize(
    "fail_at",
    [
        "stage_session",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
        "wait_navigation",
        "commit_map",
    ],
)
def test_every_failure_after_map_prepare_restores_the_map_exactly_once(
    fail_at: str,
    tmp_path,
) -> None:
    runner = FakeRunner()
    backend = FakeBackend(fail_at=fail_at)

    with pytest.raises(SwitchFailed):
        _control(runner).switch(_request(), backend=backend, state_dir=tmp_path)

    assert backend.events.count("restore_map") == 1
    cleanup_stop = len(backend.events) - 1 - backend.events[::-1].index("stop_motion_and_session")
    restore = backend.events.index("restore_map")
    assert cleanup_stop < restore
    assert [call[0] for call in runner.calls][-1] == "quiesce"
    assert not (tmp_path / "current.json").exists()


def test_runner_failure_after_map_prepare_restores_the_map_exactly_once(tmp_path) -> None:
    class FailingRunner(FakeRunner):
        def apply(self, product, *, dry_run: bool = False):
            super().apply(product, dry_run=dry_run)
            raise RuntimeError("runner apply failed")

    runner = FailingRunner()
    backend = FakeBackend()

    with pytest.raises(SwitchFailed, match="runner apply failed"):
        _control(runner).switch(_request(), backend=backend, state_dir=tmp_path)

    assert backend.events.count("restore_map") == 1
    assert [call[0] for call in runner.calls] == ["apply", "quiesce"]


def test_map_restore_failure_is_fail_closed_and_reports_rollback_failed(tmp_path) -> None:
    class RestoreFailingBackend(FakeBackend):
        def restore_map(self, token: MapActivationToken) -> None:
            super().restore_map(token)
            raise RuntimeError("restored map identity did not verify")

    runner = FakeRunner()
    backend = RestoreFailingBackend(fail_at="start_session")

    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(_request(), backend=backend, state_dir=tmp_path)

    report = failure.value.report
    assert report.status == "rollback_failed"
    assert backend.events.count("restore_map") == 1
    assert "map_failed:restored map identity did not verify" in report.cleanup
    assert backend.events.count("stop_motion_and_session") == 2
    assert [call[0] for call in runner.calls] == ["apply", "quiesce"]
    assert not (tmp_path / "current.json").exists()


def test_map_restore_failure_outranks_unconfirmed_cleanup_stop(tmp_path) -> None:
    class StopAndRestoreFailingBackend(FakeBackend):
        def __init__(self) -> None:
            super().__init__(fail_at="start_session")
            self.stop_calls = 0

        def stop_motion_and_session(self, current_product=None) -> None:
            self.stop_calls += 1
            super().stop_motion_and_session(current_product)
            if self.stop_calls > 1:
                raise RuntimeError("motion cleanup failed")

        def restore_map(self, token: MapActivationToken) -> None:
            super().restore_map(token)
            raise RuntimeError("map rollback failed")

    runner = FakeRunner()
    backend = StopAndRestoreFailingBackend()

    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(_request(), backend=backend, state_dir=tmp_path)

    report = failure.value.report
    assert report.status == "rollback_failed"
    assert "motion_session_failed:motion cleanup failed" in report.cleanup
    assert "map_failed:map rollback failed" in report.cleanup


def test_product_switch_cleanup_attempts_all_steps_when_each_step_fails(tmp_path) -> None:
    trace: list[str] = []

    class CleanupFailingRunner(FakeRunner):
        def apply(self, product, *, dry_run: bool = False):
            trace.append("runner.apply")
            return super().apply(product, dry_run=dry_run)

        def quiesce(self, product, *, dry_run: bool = False):
            trace.append("runner.quiesce")
            self.calls.append(("quiesce", product.fingerprint))
            raise RuntimeError("quiesce cleanup failed")

    class CleanupFailingBackend(FakeBackend):
        def __init__(self) -> None:
            super().__init__(fail_at="start_session")
            self.stop_calls = 0

        def _event(self, name: str) -> None:
            trace.append(name)
            super()._event(name)

        def stop_motion_and_session(self, current_product=None) -> None:
            self.stop_calls += 1
            super().stop_motion_and_session(current_product)
            if self.stop_calls > 1:
                raise RuntimeError("motion cleanup failed")

    runner = CleanupFailingRunner()
    backend = CleanupFailingBackend()

    with pytest.raises(SwitchFailed) as failure:
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.error == "failed at start_session"
    assert report.status == "stop_unconfirmed"
    assert trace[-5:] == [
        "start_session",
        "stop_motion_and_session",
        "restore_map",
        "runner.quiesce",
        "rollback_session",
    ]
    assert report.cleanup == [
        "motion_session_failed:motion cleanup failed",
        "map:restored",
        "processes_failed:quiesce cleanup failed",
        "session:removed",
        "plan:removed",
    ]
    assert [call[0] for call in runner.calls] == ["apply", "quiesce"]
    assert not (tmp_path / "current.json").exists()


def test_product_switch_rejects_missing_map_before_mutation(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()
    request = SwitchRequest(
        target_product="nav",
        current_product="teleop",
    )

    with pytest.raises(SwitchFailed, match="requires a map") as failure:
        _control(runner).switch(request, backend=backend, state_dir=tmp_path)

    assert failure.value.report.cleanup == []
    assert runner.calls == []
    assert backend.events == []
    assert not (tmp_path / "current.json").exists()
    assert not list(tmp_path.glob("runtime-*.json"))


def test_product_switch_dry_run_has_no_runtime_side_effects(tmp_path) -> None:
    runner = FakeRunner()
    backend = FakeBackend()

    report = _control(runner).switch(
        _request(),
        backend=backend,
        state_dir=tmp_path,
        dry_run=True,
    )

    assert report.ok is True
    assert report.status == "planned"
    assert report.phases == ["preflight"]
    assert backend.events == []
    assert runner.calls == []
    assert list(tmp_path.iterdir()) == []


def test_product_switch_blocks_active_map_save_before_any_mutation(tmp_path) -> None:
    class SaveBusyBackend(FakeBackend):
        def assert_map_save_idle(self) -> None:
            raise RuntimeError("map_save_in_progress: save-42:RUNNING; wait for completion or explicitly cancel")

    runner = FakeRunner()
    backend = SaveBusyBackend()

    with pytest.raises(SwitchFailed, match="map_save_in_progress") as failure:
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    assert failure.value.report.phases == ["preflight"]
    assert failure.value.report.cleanup == []
    assert backend.events == []
    assert runner.calls == []
    assert not (tmp_path / "current.json").exists()
    assert not list(tmp_path.glob("runtime-*.json"))


def test_product_switch_blocks_active_recording_before_any_mutation(tmp_path) -> None:
    class RecordingBusyBackend(FakeBackend):
        def assert_recording_idle(self) -> None:
            raise RuntimeError("recording_in_progress: session=inspection-42 state=recording")

    runner = FakeRunner()
    backend = RecordingBusyBackend()

    with pytest.raises(SwitchFailed, match="recording_in_progress") as failure:
        _control(runner).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    assert failure.value.report.phases == ["preflight", "map_save_idle"]
    assert failure.value.report.cleanup == []
    assert backend.events == []
    assert runner.calls == []
    assert not (tmp_path / "current.json").exists()
    assert not list(tmp_path.glob("runtime-*.json"))


@pytest.mark.parametrize("state", ["WAITING_SNAPSHOT", "QUEUED", "RUNNING"])
def test_field_backend_rejects_active_map_save_operations(
    monkeypatch,
    state,
) -> None:
    backend = FieldBackend(environment={"LINGTU_MAP_API_KEY": "map-client-key"})
    calls: list[tuple[str, str, dict[str, str] | None]] = []

    def http(method, path, **kwargs):
        calls.append((method, path, kwargs.get("headers")))
        return {
            "ok": True,
            "operations": [
                {
                    "operation_id": "save-42",
                    "state": state,
                }
            ],
        }

    monkeypatch.setattr(backend, "_http", http)

    with pytest.raises(RuntimeError, match=rf"save-42:{state}"):
        backend.assert_map_save_idle()

    assert calls == [
        (
            "GET",
            "/api/v1/maps/operations?limit=1000",
            {"X-API-Key": "map-client-key"},
        )
    ]


def test_field_backend_uses_operator_key_for_internal_map_save_status(
    monkeypatch,
) -> None:
    backend = FieldBackend(environment={"LINGTU_API_KEY": "operator-key"})
    calls: list[dict[str, str] | None] = []

    def http(_method, _path, **kwargs):
        calls.append(kwargs.get("headers"))
        return {"ok": True, "count": 0}

    monkeypatch.setattr(backend, "_http", http)

    backend.assert_map_save_idle()

    assert calls == [{"X-API-Key": "operator-key"}]


def test_field_backend_http_adds_operator_api_key(monkeypatch) -> None:
    captured_headers: dict[str, str] = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self) -> bytes:
            return b'{"ok":true}'

    def urlopen(request, **_kwargs):
        captured_headers.update(dict(request.header_items()))
        return Response()

    monkeypatch.setattr(product_switch.urllib.request, "urlopen", urlopen)
    backend = FieldBackend(environment={"LINGTU_API_KEY": "operator-key"})

    assert backend._http("GET", "/health", timeout_s=1.0) == {"ok": True}
    assert captured_headers["X-api-key"] == "operator-key"


def test_field_backend_http_explicit_api_key_overrides_operator_key(monkeypatch) -> None:
    captured_headers: dict[str, str] = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self) -> bytes:
            return b'{"ok":true}'

    def urlopen(request, **_kwargs):
        captured_headers.update(dict(request.header_items()))
        return Response()

    monkeypatch.setattr(product_switch.urllib.request, "urlopen", urlopen)
    backend = FieldBackend(environment={"LINGTU_API_KEY": "operator-key"})

    backend._http(
        "POST",
        "/api/v1/session/end",
        timeout_s=1.0,
        headers={
            "X-API-Key": "map-client-key",
            "X-LingTu-Product-Session": "session-token",
        },
    )

    assert captured_headers["X-api-key"] == "map-client-key"
    assert captured_headers["X-lingtu-product-session"] == "session-token"


@pytest.mark.parametrize(
    "response",
    [
        {"ok": False, "operations": []},
        {"ok": True},
        {"ok": True, "operations": {}},
        {"ok": True, "operations": [None]},
        {"ok": True, "operations": [{"operation_id": "save-42"}]},
        {"ok": True, "operations": [{"operation_id": "save-42", "state": "CANCELLING"}]},
        {"ok": True, "operations": [{"operation_id": "save-42", "state": "RECOVERING"}]},
        {"ok": True, "operations": [{"operation_id": "save-42", "state": "COMPLETED"}]},
    ],
)
def test_field_backend_fails_closed_on_invalid_map_save_status(
    monkeypatch,
    response,
) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(backend, "_http", lambda *_args, **_kwargs: response)

    with pytest.raises(RuntimeError, match="map_save_status_"):
        backend.assert_map_save_idle()


def test_field_backend_accepts_terminal_or_empty_map_save_status(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    responses = iter(
        (
            {"ok": True, "count": 0},
            {
                "success": True,
                "operations": [
                    {"operation_id": "saved", "state": "SUCCEEDED"},
                    {"operation_id": "failed", "state": "FAILED"},
                    {"operation_id": "cancelled", "state": "CANCELLED"},
                ],
            },
        )
    )
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: next(responses),
    )

    backend.assert_map_save_idle()
    backend.assert_map_save_idle()


@pytest.mark.parametrize("state", ["preparing", "recording", "stopping"])
def test_field_backend_rejects_active_native_recording(monkeypatch, tmp_path, state) -> None:
    backend = FieldBackend(environment={"LINGTU_RECORDING_ROOT": str(tmp_path)})
    monkeypatch.setattr(backend, "_recording_binary", lambda: Path("/native/recorder"))
    monkeypatch.setattr(
        backend,
        "_run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                {
                    "control_version": 1,
                    "ok": True,
                    "healthy": True,
                    "state": state,
                    "session": {"session_id": "inspection-42"},
                }
            ),
            stderr="",
        ),
    )

    with pytest.raises(RuntimeError, match=rf"inspection-42.*state={state}"):
        backend.assert_recording_idle()


def test_field_backend_rejects_unhealthy_active_native_recording(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(backend, "_recording_binary", lambda: Path("/native/recorder"))
    monkeypatch.setattr(
        backend,
        "_run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=4,
            stdout=json.dumps(
                {
                    "control_version": 1,
                    "ok": True,
                    "healthy": False,
                    "state": "recording",
                    "session": {"session_id": "stale-42"},
                }
            ),
            stderr="",
        ),
    )

    with pytest.raises(RuntimeError, match="recording_recovery_required"):
        backend.assert_recording_idle()


@pytest.mark.parametrize(
    ("state", "healthy", "returncode"),
    [("idle", True, 0), ("completed", True, 0), ("failed", False, 4)],
)
def test_field_backend_accepts_terminal_native_recording_status(
    monkeypatch,
    state,
    healthy,
    returncode,
) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(backend, "_recording_binary", lambda: Path("/native/recorder"))
    monkeypatch.setattr(
        backend,
        "_run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=returncode,
            stdout=json.dumps(
                {
                    "control_version": 1,
                    "ok": True,
                    "healthy": healthy,
                    "state": state,
                    "session": None,
                }
            ),
            stderr="",
        ),
    )

    backend.assert_recording_idle()


@pytest.mark.parametrize(
    "payload",
    [
        "",
        "not-json",
        json.dumps({"control_version": 2, "ok": True}),
        json.dumps({"control_version": 1, "ok": False}),
        json.dumps(
            {
                "control_version": 1,
                "ok": True,
                "healthy": True,
                "state": "unknown",
            }
        ),
    ],
)
def test_field_backend_fails_closed_on_invalid_recording_status(
    monkeypatch,
    payload,
) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(backend, "_recording_binary", lambda: Path("/native/recorder"))
    monkeypatch.setattr(
        backend,
        "_run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=3,
            stdout=payload,
            stderr="",
        ),
    )

    with pytest.raises(RuntimeError, match="recording_"):
        backend.assert_recording_idle()


def test_map_service_artifact_must_be_an_absolute_local_path() -> None:
    identity = MapIdentity(
        map_id="demo",
        version_id="demo:v1",
        frame_id="map",
        map_dir="/maps/demo/.versions/1",
        artifacts=(
            MapArtifactIdentity(
                artifact_type="POINTCLOUD",
                uri="../outside/map.pcd",
                sha256="demo-v1",
            ),
        ),
    )

    with pytest.raises(RuntimeError, match="must be absolute"):
        product_switch._pointcloud_artifact_path(identity)


def test_field_backend_rejects_non_http_gateway_url() -> None:
    with pytest.raises(ValueError, match="HTTP"):
        FieldBackend(gateway_url="file:///tmp/control.json")


def test_field_backend_uses_only_the_injected_session_root(tmp_path) -> None:
    commands: list[list[str]] = []

    def run(command, **_kwargs):
        commands.append(list(command))
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=run,
    )

    target = str(tmp_path.resolve() / "session.env")
    backend._install_runtime_file(target, 'LINGTU_PRODUCT="nav"\n')
    backend._remove_runtime_file(target)

    assert [command[-1] for command in commands] == [target, target]
    install = commands[0]
    assert install[install.index("-o") + 1] == "root"
    assert install[install.index("-g") + 1] == "sunrise"
    assert install[install.index("-m") + 1] == "0640"
    assert all("/etc/systemd/system" not in " ".join(command) for command in commands)


def test_session_explanation_is_pure_and_names_session_ownership() -> None:
    plan = _field_plan()

    explanation = session_explanation(plan)

    assert explanation["session_root"] == "/run/lingtu"
    assert explanation["session_file"] == "/run/lingtu/session.env"
    assert explanation["run_plan_fingerprint"] == plan.fingerprint
    assert explanation["persistent_boot_units"] == ["lingtu-driver.service"]
    assert set(explanation["session_guarded_units"]) == {
        "lingtu.service",
        "lingtu-nav-dds.service",
        "lingtu-slam-dds.service",
    }
    assert all(item["path"].startswith("/run/systemd/system/") for item in explanation["legacy_dropins"])


def test_field_backend_removes_session_and_legacy_transient_dropins(tmp_path) -> None:
    commands: list[list[str]] = []

    def run(command, **_kwargs):
        commands.append(list(command))
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    plan = _field_plan()
    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=run,
    )

    backend.remove_session(plan)

    removed = {command[-1] for command in commands if command[:3] == ["sudo", "-n", "rm"]}
    assert removed == {
        str(tmp_path.resolve() / "session.env"),
        *{
            f"/run/systemd/system/{unit}.d/{name}"
            for unit, name in (
                ("lingtu.service", "product-mode.conf"),
                ("lingtu-nav-dds.service", "product-mode.conf"),
                ("lingtu-slam-dds.service", "runtime-mode.conf"),
                ("lingtu-explore-dds.service", "product-mode.conf"),
            )
        },
    }
    assert ["sudo", "-n", "systemctl", "daemon-reload"] in commands


def test_field_backend_does_not_persist_product_mode_in_systemd() -> None:
    assert not hasattr(FieldBackend, "persist_boot_ownership")
    assert not hasattr(FieldBackend, "disable_boot_ownership")


def test_guarded_runtime_entrypoints_require_both_session_values() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    deploy_root = repo_root / "scripts" / "deploy" / "thunder"
    helper_path = deploy_root / "require_product_session.sh"
    helper = helper_path.read_text(encoding="utf-8")

    assert "${LINGTU_PRODUCT_SESSION_ID:?" in helper
    assert "${LINGTU_RUN_PLAN_FINGERPRINT:?" in helper
    for script_name in ("run_nav_dds.sh", "run_slam_dds.sh", "run_explore_dds.sh"):
        script = (deploy_root / script_name).read_text(encoding="utf-8")
        assert "source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh" in script
    host_unit = (deploy_root / "lingtu.service").read_text(encoding="utf-8")
    assert (
        "ExecStartPre=/bin/bash /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh host"
    ) in host_unit


def test_field_backend_stages_one_session_identity_for_every_guarded_unit(
    monkeypatch,
    tmp_path,
) -> None:
    installed: list[tuple[str, str]] = []
    backend = FieldBackend(environment={"LINGTU_SESSION_ROOT": str(tmp_path)})
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: installed.append((target, content)),
    )
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)
    plan = _field_plan(include_explore=True)

    backend.stage_session(
        tmp_path / "runtime.json",
        plan,
        {key: "configured" for key in product_switch._NAV_ENV_KEYS},
        slam_mode="mapping",
        map_path="",
        product_session_id="shared-product-session",
    )

    assert len(installed) == 1
    target, content = installed[0]
    assert target == str(tmp_path / "session.env")
    assert 'LINGTU_PRODUCT_SESSION_ID="shared-product-session"' in content
    assert f'LINGTU_RUN_PLAN_FINGERPRINT="{plan.fingerprint}"' in content


def test_field_backend_runtime_session_rollback_removes_the_new_session(
    monkeypatch,
    tmp_path,
) -> None:
    files: dict[str, str] = {}
    backend = FieldBackend(environment={"LINGTU_SESSION_ROOT": str(tmp_path)})
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
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)
    plan = _field_plan(include_explore=True)

    staged = backend.stage_session(
        tmp_path / "runtime.json",
        plan,
        {key: "configured" for key in product_switch._NAV_ENV_KEYS},
        slam_mode="mapping",
        map_path="",
        product_session_id="replacement-session",
    )
    assert files

    backend.rollback_session(staged)

    assert files == {}


def test_field_backend_stages_compiled_host_control_contract(monkeypatch, tmp_path) -> None:
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import compile_run_plan
    from runtime.profiles.native_nav_config import compile_native_nav_config

    resolved = resolve_product_host_runtime("teleop_avoid", "real")
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )
    installed: list[tuple[str, str]] = []
    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout="",
            stderr="",
        ),
    )
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: installed.append((target, content)),
    )
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)

    backend.stage_session(
        tmp_path / "runtime.json",
        plan,
        compile_native_nav_config(
            plan.product,
            {
                **plan.host_config,
                "native_control_mode": plan.native_nav.get("control_mode"),
                "native_nav": plan.native_nav,
            },
        ).environment,
        slam_mode="none",
        map_path="",
    )

    assert len(installed) == 1
    target, session = installed[0]
    assert target == str(tmp_path / "session.env")
    assert 'LINGTU_PRODUCT="teleop_avoid"' in session
    assert "LINGTU_PROFILE" not in session
    assert 'LINGTU_COMMAND_OUTPUT_MODE="endpoint_only"' in session
    assert 'LINGTU_HARDWARE_CONTROL_BOUNDARY="driver"' in session
    assert f'LINGTU_RUN_PLAN_FINGERPRINT="{plan.fingerprint}"' in session


@pytest.mark.parametrize(
    ("product_variant", "slam_mode", "route"),
    (("live", "mapping", "live"), ("map", "localization", "map")),
)
def test_field_backend_stages_explore_route(
    monkeypatch,
    tmp_path,
    product_variant: str,
    slam_mode: str,
    route: str,
) -> None:
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import compile_run_plan
    from runtime.profiles.native_nav_config import compile_native_nav_config

    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant=product_variant,
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
    )
    installed: list[tuple[str, str]] = []
    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stdout="", stderr=""),
    )
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: installed.append((target, content)),
    )
    monkeypatch.setattr(backend, "_remove_legacy_runtime_config", lambda _plan: None)

    backend.stage_session(
        tmp_path / f"explore-{product_variant}.json",
        plan,
        compile_native_nav_config(
            plan.product,
            {
                **plan.host_config,
                "native_control_mode": plan.native_nav.get("control_mode"),
                "native_nav": plan.native_nav,
            },
        ).environment,
        slam_mode=slam_mode,
        map_path="/maps/demo" if slam_mode == "localization" else "",
        map_identity=(_exact_identity("demo", 7) if slam_mode == "localization" else None),
        product_session_id="shared-product-session",
    )

    assert len(installed) == 1
    target, session = installed[0]
    assert target == str(tmp_path / "session.env")
    assert f'LINGTU_EXPLORE_ROUTE="{route}"' in session
    assert plan.product_variant == product_variant
    assert 'LINGTU_PRODUCT_SESSION_ID="shared-product-session"' in session
    assert f'LINGTU_RUN_PLAN_FINGERPRINT="{plan.fingerprint}"' in session


def test_field_backend_rejects_unknown_plan_schema_before_staging_dropins(
    monkeypatch,
) -> None:
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import compile_run_plan
    from runtime.profiles.native_nav_config import compile_native_nav_config

    resolved = resolve_product_host_runtime("map", "real")
    runtime = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )
    plan = replace(
        runtime,
        schema_version="lingtu.run_plan.unknown",
    )
    installed: list[tuple[str, str]] = []

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("unknown schema must fail before reading systemd state")

    backend = FieldBackend(environment={}, runner=reject_real_command)
    monkeypatch.setattr(
        backend,
        "_install_runtime_file",
        lambda target, content: installed.append((target, content)),
    )

    with pytest.raises(RuntimeError, match="unsupported RunPlan schema"):
        backend.stage_session(
            Path("unused-product.json"),
            plan,
            compile_native_nav_config(
                runtime.product,
                {
                    **runtime.host_config,
                    "native_control_mode": runtime.native_nav.get("control_mode"),
                    "native_nav": runtime.native_nav,
                },
            ).environment,
            slam_mode="mapping",
            map_path="",
        )

    assert installed == []


def test_field_backend_prefers_explicit_product_identity() -> None:
    backend = FieldBackend(
        environment={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_PROFILE": "sim_mujoco_live",
        },
        runner=lambda *_args, **_kwargs: pytest.fail("managed unit lookup was not expected"),
    )

    assert backend.current_product() == "nav"


def test_field_backend_allows_clean_bootstrap_without_current_product() -> None:
    def run(_command, **_kwargs):
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(environment={}, runner=run)

    assert backend.current_product() is None


@pytest.mark.parametrize(
    ("native_identity", "accepted"),
    [
        (
            {
                "native_product": {
                    "product": "nav",
                    "config_fingerprint": "nav-config",
                }
            },
            True,
        ),
        (
            {
                "native_profile": {
                    "profile": "nav",
                    "config_fingerprint": "nav-config",
                }
            },
            False,
        ),
        (
            {
                "native_product": {
                    "product": "map",
                    "config_fingerprint": "nav-config",
                }
            },
            False,
        ),
    ],
)
def test_field_backend_requires_native_product_identity(tmp_path, native_identity, accepted) -> None:
    status_path = tmp_path / "nav-status.json"
    status_path.write_text(
        json.dumps(
            {
                "stamp_s": 10.0,
                "control_mode": "autonomy",
                "publish_cmd_vel": True,
                "check_obstacle": True,
                "use_traversability_cost": True,
                "allow_teleop_takeover": True,
                "teleop_local_planner": False,
                **native_identity,
            }
        ),
        encoding="utf-8",
    )
    ticks = iter((0.0, 0.0, 1.0))
    native_environment = {
        "LINGTU_PRODUCT": "nav",
        "LINGTU_NAV_CONTROL_MODE": "autonomy",
        "LINGTU_NAV_CONFIG_FINGERPRINT": "nav-config",
        "LINGTU_NAV_PUBLISH_CMD_VEL": "1",
        "LINGTU_NAV_CHECK_OBSTACLE": "1",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST": "1",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "1",
        "LINGTU_TELEOP_LOCAL_PLANNER": "0",
    }
    backend = FieldBackend(
        environment={"LINGTU_NAV_STATUS_FILE": str(status_path)},
        sleep=lambda _seconds: None,
        monotonic=lambda: next(ticks),
        wall_clock=lambda: 10.0,
    )

    if accepted:
        backend.wait_native_nav(native_environment, timeout_s=0.5)
    else:
        with pytest.raises(RuntimeError, match="native navigation config was not confirmed"):
            backend.wait_native_nav(native_environment, timeout_s=0.5)


def test_product_switch_bootstraps_without_a_running_host(tmp_path) -> None:
    class BootstrapBackend(FakeBackend):
        def current_product(self):
            self._event("current_product")
            return None

    runner = FakeRunner()
    backend = BootstrapBackend()

    report = _control(runner).switch(
        replace(_request(), current_product=None),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.current_product is None
    assert backend.events[:3] == [
        "current_product",
        "stop_motion_and_session",
        "stage_map",
    ]


def test_field_backend_stops_native_motion_before_host_session(monkeypatch) -> None:
    commands: list[list[str]] = []
    http_calls: list[tuple[str, str, dict[str, str] | None]] = []

    def run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(
        environment={
            "LINGTU_NAV_CONTROL_BIN": "/release/lingtu_nav_control",
            "LINGTU_DDS_DOMAIN_ID": "23",
            "LINGTU_PRODUCT_SESSION_ID": "session-token-1234",
        },
        runner=run,
    )

    def http(method, path, **kwargs):
        http_calls.append((method, path, kwargs.get("headers")))
        return {"ok": True, "success": True}

    monkeypatch.setattr(backend, "_http", http)

    backend.stop_motion_and_session("nav")

    assert commands == [
        ["systemctl", "is-active", "--quiet", "lingtu-nav-dds.service"],
        [
            "/release/lingtu_nav_control",
            "stop",
            "product_mode_switch",
            "--domain-id",
            "23",
            "--timeout-ms",
            "3000",
        ],
    ]
    assert http_calls == [
        (
            "POST",
            "/api/v1/session/end",
            {"X-LingTu-Product-Session": "session-token-1234"},
        )
    ]


def test_field_backend_clean_bootstrap_does_not_require_gateway(monkeypatch) -> None:
    def run(command, **_kwargs):
        assert command == [
            "systemctl",
            "is-active",
            "--quiet",
            "lingtu-nav-dds.service",
        ]
        return SimpleNamespace(returncode=3, stdout="", stderr="")

    backend = FieldBackend(environment={}, runner=run)
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("Gateway unavailable")),
    )

    backend.stop_motion_and_session(None)


def test_field_backend_refuses_active_product_stop_without_native_nav() -> None:
    def run(command, **_kwargs):
        assert command == [
            "systemctl",
            "is-active",
            "--quiet",
            "lingtu-nav-dds.service",
        ]
        return SimpleNamespace(returncode=3, stdout="", stderr="")

    backend = FieldBackend(environment={}, runner=run)

    with pytest.raises(RuntimeError, match="native navigation stop is unavailable"):
        backend.stop_motion_and_session("explore")


def test_field_backend_clears_all_stale_runtime_evidence() -> None:
    commands: list[list[str]] = []

    def run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(
        environment={
            "LINGTU_NAV_STATUS_FILE": "/dev/shm/lingtu/nav.json",
            "LINGTU_EXPLORE_STATUS_FILE": "/dev/shm/lingtu/explore.json",
            "LINGTU_SLAM_STATUS_JSON": "/tmp/lingtu-slam.json",
            "LINGTU_SLAM_CLOUD_SNAPSHOT_DIR": "/dev/shm/lingtu-slam",
        },
        runner=run,
    )

    backend.clear_runtime_status()

    assert commands == [
        [
            "sudo",
            "-n",
            "rm",
            "-f",
            "--",
            "/dev/shm/lingtu/nav.json",
            "/tmp/lingtu-slam.json",
            "/tmp/lingtu-slam.json.tmp",
            "/dev/shm/lingtu/explore.json",
            "/dev/shm/lingtu/explore.json.tmp",
        ],
        ["sudo", "-n", "rm", "-rf", "--", "/dev/shm/lingtu-slam"],
    ]


def test_field_backend_rejects_unsafe_runtime_cleanup_path() -> None:
    backend = FieldBackend(environment={"LINGTU_SLAM_CLOUD_SNAPSHOT_DIR": "/"})

    with pytest.raises(RuntimeError, match="unsafe runtime cleanup path"):
        backend.clear_runtime_status()


def test_wait_slam_requires_fresh_snapshot_from_live_runtime(tmp_path) -> None:
    status_path = tmp_path / "slam.json"
    status_path.write_text(
        """{
          "runtime_instance_id": "runtime-new",
          "mode": "localization",
          "state": "TRACKING",
          "alive": true,
          "map_loaded": true,
          "snapshot_written_at_s": 99.5
        }""",
        encoding="utf-8",
    )
    backend = FieldBackend(
        environment={"LINGTU_SLAM_STATUS_JSON": str(status_path)},
        wall_clock=lambda: 100.0,
    )

    backend.wait_slam("localization", require_map=True, timeout_s=0.1)


def test_wait_slam_rejects_stale_snapshot(tmp_path) -> None:
    status_path = tmp_path / "slam.json"
    status_path.write_text(
        """{
          "runtime_instance_id": "runtime-old",
          "mode": "localization",
          "state": "TRACKING",
          "alive": true,
          "map_loaded": true,
          "snapshot_written_at_s": 90.0
        }""",
        encoding="utf-8",
    )
    clock = {"now": 0.0}

    def monotonic() -> float:
        return clock["now"]

    def sleep(duration_s: float) -> None:
        clock["now"] += duration_s

    backend = FieldBackend(
        environment={"LINGTU_SLAM_STATUS_JSON": str(status_path)},
        monotonic=monotonic,
        sleep=sleep,
        wall_clock=lambda: 100.0,
    )

    with pytest.raises(RuntimeError, match="SLAM did not become ready"):
        backend.wait_slam("localization", require_map=True, timeout_s=0.1)


@pytest.mark.parametrize("route", ["live", "map"])
def test_wait_exploration_requires_exact_idle_session_binding(tmp_path, route) -> None:
    status_path = tmp_path / "explore.json"
    session_id = "product-session-7"
    identity = _exact_identity("plant-a", 7) if route == "map" else None
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.explore.status.v2",
                "endpoint": "lingtu_explore_dds",
                "route": route,
                "state": "idle",
                "active": False,
                "paused": False,
                "ready": True,
                "pending_goal": None,
                "pending_segment": None,
                "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.2},
                "map": {
                    "frame_id": "map",
                    "session_id": session_id,
                    "map_id": "plant-a" if route == "map" else "",
                    "map_version": 7 if route == "map" else 0,
                    "artifact_hash": "plant-a-v7-sha" if route == "map" else "",
                    "reset_epoch": 1,
                    "generation": 3,
                    "live": route == "live",
                },
                "counters": {"odometry_messages": 2, "snapshot_messages": 1},
            }
        ),
        encoding="utf-8",
    )
    backend = FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        wall_clock=lambda: status_path.stat().st_mtime + 0.1,
    )

    backend.wait_exploration(
        route,
        map_identity=identity,
        product_session_id=session_id,
        timeout_s=0.1,
    )


@pytest.mark.parametrize(
    ("state", "paused"),
    [
        ("planning", False),
        ("executing", False),
        ("segment_executing", False),
        ("waiting_segment_snapshot", False),
        ("paused", True),
    ],
)
def test_wait_exploration_accepts_healthy_existing_task(
    tmp_path,
    state,
    paused,
) -> None:
    status_path = tmp_path / "explore.json"
    session_id = "product-session-7"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.explore.status.v2",
                "endpoint": "lingtu_explore_dds",
                "route": "live",
                "state": state,
                "active": True,
                "paused": paused,
                "ready": True,
                "pending_goal": (
                    {"task_id": "explore-task", "request_id": "explore-request"}
                    if state in {"dispatching", "executing"}
                    else None
                ),
                "pending_segment": (
                    {"request_id": "explore-segment", "session_id": session_id}
                    if state
                    in {
                        "segment_dispatching",
                        "segment_executing",
                    }
                    else None
                ),
                "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.2},
                "map": {
                    "frame_id": "map",
                    "session_id": session_id,
                    "map_id": "",
                    "map_version": 0,
                    "artifact_hash": "",
                    "reset_epoch": 1,
                    "generation": 3,
                    "live": True,
                },
                "counters": {"odometry_messages": 2, "snapshot_messages": 1},
            }
        ),
        encoding="utf-8",
    )
    backend = FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        wall_clock=lambda: status_path.stat().st_mtime + 0.1,
    )

    backend.wait_exploration(
        "live",
        map_identity=None,
        product_session_id=session_id,
        timeout_s=0.1,
        allow_active=True,
    )


@pytest.mark.parametrize(
    ("state", "odometry_age_s"),
    [("cancelling", 0.1), ("planning", 5.0)],
)
def test_wait_exploration_rejects_stopping_or_stale_existing_task(
    tmp_path,
    state,
    odometry_age_s,
) -> None:
    status_path = tmp_path / "explore.json"
    session_id = "product-session-7"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.explore.status.v2",
                "endpoint": "lingtu_explore_dds",
                "route": "live",
                "state": state,
                "active": state != "cancelling",
                "paused": False,
                "ready": True,
                "pending_goal": None,
                "pending_segment": None,
                "input": {
                    "odometry_age_s": odometry_age_s,
                    "snapshot_age_s": 0.2,
                },
                "map": {
                    "frame_id": "map",
                    "session_id": session_id,
                    "map_id": "",
                    "map_version": 0,
                    "artifact_hash": "",
                    "reset_epoch": 1,
                    "generation": 3,
                    "live": True,
                },
                "counters": {"odometry_messages": 2, "snapshot_messages": 1},
            }
        ),
        encoding="utf-8",
    )
    clock = {"now": 0.0}
    backend = FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        monotonic=lambda: clock["now"],
        sleep=lambda duration_s: clock.__setitem__("now", clock["now"] + duration_s),
        wall_clock=lambda: status_path.stat().st_mtime + 0.1,
    )

    with pytest.raises(RuntimeError, match="exploration did not become ready"):
        backend.wait_exploration(
            "live",
            map_identity=None,
            product_session_id=session_id,
            timeout_s=0.1,
            allow_active=True,
        )


@pytest.mark.parametrize("case", ["missing_pending_segment", "idle_pending_goal"])
def test_wait_exploration_rejects_incomplete_or_busy_idle_status(
    tmp_path,
    case,
) -> None:
    status_path = tmp_path / "explore.json"
    session_id = "product-session-7"
    payload = {
        "schema_version": "lingtu.explore.status.v2",
        "endpoint": "lingtu_explore_dds",
        "route": "live",
        "state": "idle",
        "active": False,
        "paused": False,
        "ready": True,
        "pending_goal": None,
        "pending_segment": None,
        "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.2},
        "map": {
            "frame_id": "map",
            "session_id": session_id,
            "map_id": "",
            "map_version": 0,
            "artifact_hash": "",
            "reset_epoch": 1,
            "generation": 3,
            "live": True,
        },
        "counters": {"odometry_messages": 2, "snapshot_messages": 1},
    }
    if case == "missing_pending_segment":
        del payload["pending_segment"]
    else:
        payload["pending_goal"] = {
            "task_id": "unexpected-task",
            "request_id": "unexpected-request",
        }
    status_path.write_text(json.dumps(payload), encoding="utf-8")
    clock = {"now": 0.0}
    backend = FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        monotonic=lambda: clock["now"],
        sleep=lambda duration_s: clock.__setitem__("now", clock["now"] + duration_s),
        wall_clock=lambda: status_path.stat().st_mtime + 0.1,
    )

    with pytest.raises(RuntimeError, match="exploration did not become ready"):
        backend.wait_exploration(
            "live",
            map_identity=None,
            product_session_id=session_id,
            timeout_s=0.1,
            allow_active=True,
        )


def test_wait_exploration_rejects_stale_or_foreign_session(tmp_path) -> None:
    status_path = tmp_path / "explore.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.explore.status.v2",
                "endpoint": "lingtu_explore_dds",
                "route": "live",
                "state": "idle",
                "active": False,
                "paused": False,
                "ready": True,
                "pending_goal": None,
                "pending_segment": None,
                "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.2},
                "map": {
                    "frame_id": "map",
                    "session_id": "old-session",
                    "map_id": "",
                    "map_version": 0,
                    "artifact_hash": "",
                    "reset_epoch": 1,
                    "generation": 3,
                    "live": True,
                },
                "counters": {"odometry_messages": 2, "snapshot_messages": 1},
            }
        ),
        encoding="utf-8",
    )
    clock = {"now": 0.0}
    backend = FieldBackend(
        environment={"LINGTU_EXPLORE_STATUS_FILE": str(status_path)},
        monotonic=lambda: clock["now"],
        sleep=lambda duration_s: clock.__setitem__("now", clock["now"] + duration_s),
        wall_clock=lambda: status_path.stat().st_mtime + 0.1,
    )

    with pytest.raises(RuntimeError, match="exploration did not become ready"):
        backend.wait_exploration(
            "live",
            map_identity=None,
            product_session_id="new-session",
            timeout_s=0.1,
        )


def test_field_backend_activates_explore_session_without_starting_task(monkeypatch) -> None:
    calls: list[tuple[str, str, dict | None]] = []
    backend = FieldBackend(environment={})

    def http(method: str, path: str, payload=None, **_kwargs):
        calls.append((method, path, payload))
        return {"ok": True, "success": True}

    monkeypatch.setattr(backend, "_http", http)
    lifecycle = SimpleNamespace(
        product="explore",
        product_session="exploration",
        session_mode="exploring",
        slam_mode="mapping",
    )

    backend.start_session(
        lifecycle,
        map_name="",
        relocalize=False,
        initial_pose=None,
    )

    assert calls == [
        (
            "POST",
            "/api/v1/session/start",
            {
                "profile": "explore",
                "mode": "exploring",
                "product_session": "exploration",
                "start_task": False,
            },
        )
    ]


def test_wait_navigation_rejects_teleop_avoid_with_aggregate_blocker(monkeypatch) -> None:
    clock = {"now": 0.0}

    def monotonic() -> float:
        return clock["now"]

    def sleep(duration_s: float) -> None:
        clock["now"] += duration_s

    backend = FieldBackend(
        environment={},
        monotonic=monotonic,
        sleep=sleep,
    )

    def response(_method: str, path: str, **_kwargs):
        if path == "/api/v1/navigation/status":
            return {
                "state": "IDLE",
                "readiness": {
                    "native_endpoint": {"ok": True, "blockers": []},
                    "blockers": ["safety_stop"],
                },
            }
        return {"mode": "idle", "active_map": None}

    monkeypatch.setattr(backend, "_http", response)

    with pytest.raises(RuntimeError, match="navigation did not become ready"):
        backend.wait_navigation(
            map_name="",
            control_mode="teleop_avoid",
            timeout_s=0.1,
        )


def test_wait_navigation_teleop_ignores_only_autonomy_blockers(monkeypatch) -> None:
    clock = {"now": 0.0}
    blockers = {
        "value": [
            "odometry_missing",
            "navigation_session_inactive",
            "localization_lost",
            "map_artifact_gate_failed",
            "native_global_planner_missing",
        ]
    }
    backend = FieldBackend(
        environment={},
        monotonic=lambda: clock["now"],
        sleep=lambda duration_s: clock.__setitem__("now", clock["now"] + duration_s),
    )

    def response(_method: str, path: str, **_kwargs):
        if path == "/api/v1/navigation/status":
            return {
                "state": "IDLE",
                "readiness": {
                    "native_endpoint": {"ok": True, "blockers": []},
                    "blockers": list(blockers["value"]),
                },
            }
        return {"mode": "idle", "active_map": None}

    monkeypatch.setattr(backend, "_http", response)

    backend.wait_navigation(map_name="", control_mode="teleop", timeout_s=0.1)

    blockers["value"] = ["safety_stop"]
    clock["now"] = 0.0
    with pytest.raises(RuntimeError, match="navigation did not become ready"):
        backend.wait_navigation(map_name="", control_mode="teleop", timeout_s=0.1)
