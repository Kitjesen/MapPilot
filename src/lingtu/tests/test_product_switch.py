# ruff: noqa: S101, S108

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from lingtu.control import ProductControl
from lingtu.launcher import LaunchReport
from lingtu.product import ProductManifest
from lingtu.product_switch import (
    FieldBackend,
    MapActivationToken,
    MapArtifactIdentity,
    MapIdentity,
    SwitchFailed,
    SwitchRequest,
)


class FakeLauncher:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def apply(self, product, *, dry_run: bool = False):
        self.calls.append(("apply", product.fingerprint))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
            action="apply",
            ok=True,
            status="active",
            dry_run=dry_run,
        )

    def quiesce(self, product, *, dry_run: bool = False):
        self.calls.append(("quiesce", product.fingerprint))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
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

    def current_profile(self) -> str:
        self._event("current_profile")
        return "teleop"

    def resolve_map(self, map_name: str) -> tuple[str, str]:
        self._event("resolve_map")
        return map_name, f"/maps/{map_name}/map.pcd"

    def stop_motion_and_session(self) -> None:
        self._event("stop_motion_and_session")

    def stage_map(self, map_name: str) -> MapActivationToken:
        assert map_name == "plant-a"
        self._event("stage_map")
        return MapActivationToken(
            target=MapIdentity(map_id=map_name, version=7, version_id="plant-a:v7", frame_id="map"),
            previous=MapIdentity(
                map_id="warehouse",
                version=3,
                version_id="warehouse:v3",
                frame_id="map",
            ),
            changed=True,
        )

    def restore_map(self, token: MapActivationToken) -> None:
        assert token.target.map_id == "plant-a"
        assert token.previous is not None
        assert token.previous.map_id == "warehouse"
        self._event("restore_map")

    def commit_map(self, token: MapActivationToken) -> None:
        assert token.target.map_id == "plant-a"
        self._event("commit_map")

    def stage_runtime_config(
        self,
        manifest_path: Path,
        manifest,
        native_environment,
        *,
        slam_mode: str,
        map_path: str,
    ) -> None:
        assert manifest_path.is_file()
        assert manifest.profile == "nav"
        assert native_environment["LINGTU_NAV_CONTROL_MODE"] == "autonomy"
        assert slam_mode == "localization"
        assert map_path == "/maps/plant-a/map.pcd"
        self._event("stage_runtime_config")

    def persist_boot_ownership(self, manifest) -> None:
        assert manifest.profile == "nav"
        self._event("persist_boot_ownership")

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
        assert contract.profile == "nav"
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

    def disable_boot_ownership(self, manifest) -> None:
        assert manifest.profile == "nav"
        self._event("disable_boot_ownership")


def _control(launcher: FakeLauncher) -> ProductControl:
    return ProductControl(launcher, environment={})  # type: ignore[arg-type]


def _request() -> SwitchRequest:
    return SwitchRequest(
        target_profile="nav",
        current_profile="teleop",
        endpoint="thunder_field",
        map_name="plant-a",
        relocalize=True,
        initial_pose=(1.0, 2.0, 0.3),
    )


def _exact_identity(name: str, version: int) -> MapIdentity:
    return MapIdentity(
        map_id=name,
        version=version,
        version_id=f"{name}:v{version}",
        frame_id="map",
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
        "map_id": name,
        "version": version,
        "version_id": f"{name}:v{version}",
        "frame_id": "map",
        "artifacts": [
            {
                "type": "POINTCLOUD",
                "uri": f"/maps/{name}/.versions/{version}/map.pcd",
                "hash": f"{name}-v{version}-sha",
            }
        ],
    }


def test_field_backend_stage_map_returns_typed_activation_identity(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: {
            "ok": True,
            "success": True,
            "active": "plant-a",
            "activation_token": {
                "schema_version": "lingtu.map_activation.v1",
                "changed": True,
                "target": {
                    "map_id": "plant-a",
                    "version": 7,
                    "version_id": "plant-lineage:v7",
                    "frame_id": "map",
                    "artifacts": [
                        {
                            "type": "POINTCLOUD",
                            "uri": "/maps/plant-a/.versions/7/map.pcd",
                            "hash": "pcd-sha",
                        }
                    ],
                },
                "previous": {
                    "map_id": "warehouse",
                    "version": 3,
                    "version_id": "warehouse-lineage:v3",
                    "frame_id": "map",
                    "artifacts": [
                        {
                            "type": "POINTCLOUD",
                            "uri": "/maps/warehouse/.versions/3/map.pcd",
                            "hash": "warehouse-sha",
                        }
                    ],
                },
            },
            "transaction": {
                "operation": "stage_map_for_runtime_switch",
                "state": "staged",
                "target_map": "plant-a",
                "previous_active": "warehouse",
                "runtime_consistent": False,
                "restart_required": True,
            },
        },
    )

    token = backend.stage_map("plant-a")

    assert isinstance(token, MapActivationToken)
    assert token.changed is True
    assert token.target.map_id == "plant-a"
    assert token.target.version == 7
    assert token.target.version_id == "plant-lineage:v7"
    assert token.target.frame_id == "map"
    assert token.target.artifacts[0].sha256 == "pcd-sha"
    assert token.previous is not None
    assert token.previous.map_id == "warehouse"


def test_field_backend_restores_and_verifies_the_previous_map_identity(monkeypatch) -> None:
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=_exact_identity("warehouse", 3),
        changed=True,
    )
    calls: list[tuple[str, str, object]] = []
    backend = FieldBackend(environment={})

    def fake_http(method, path, payload=None, **_kwargs):
        calls.append((method, path, payload))
        return {
            "ok": True,
            "success": True,
            "active": "warehouse",
            "restored_identity": {
                **_exact_identity_payload("warehouse", 3),
            },
            "transaction": {
                "operation": "restore_staged_map_for_runtime_switch",
                "state": "rolled_back",
                "target_map": "plant-a",
                "previous_active": "warehouse",
                "verified": True,
            },
        }

    monkeypatch.setattr(backend, "_http", fake_http)

    backend.restore_map(token)

    assert calls == [
        (
            "POST",
            "/api/v1/map/restore-staged-runtime-switch",
            {
                "activation_token": {
                    "schema_version": "lingtu.map_activation.v1",
                    "changed": True,
                    "target": _exact_identity_payload("plant-a", 7),
                    "previous": _exact_identity_payload("warehouse", 3),
                }
            },
        )
    ]


def test_field_backend_restore_with_no_previous_uses_verified_clear_primitive(monkeypatch) -> None:
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=None,
        changed=True,
    )
    payloads: list[object] = []
    backend = FieldBackend(environment={})

    def fake_http(_method, _path, payload=None, **_kwargs):
        payloads.append(payload)
        return {
            "ok": True,
            "success": True,
            "active": "",
            "transaction": {
                "operation": "restore_staged_map_for_runtime_switch",
                "state": "rolled_back",
                "target_map": "plant-a",
                "previous_active": None,
                "verified": True,
            },
        }

    monkeypatch.setattr(backend, "_http", fake_http)

    backend.restore_map(token)

    assert payloads == [
        {
            "activation_token": {
                "schema_version": "lingtu.map_activation.v1",
                "changed": True,
                "target": _exact_identity_payload("plant-a", 7),
                "previous": None,
            }
        }
    ]


def test_field_backend_rejects_incomplete_restore_token_before_http(monkeypatch) -> None:
    backend = FieldBackend(environment={})
    calls: list[object] = []
    monkeypatch.setattr(backend, "_http", lambda *_args, **_kwargs: calls.append(_args))
    token = MapActivationToken(
        target=MapIdentity(
            map_id="plant-a",
            version=7,
            version_id="plant-a:v7",
            frame_id="map",
        ),
        previous=None,
        changed=True,
    )

    with pytest.raises(RuntimeError, match="artifacts must be a non-empty list"):
        backend.restore_map(token)

    assert calls == []


def test_field_backend_rejects_restore_identity_verification_mismatch(monkeypatch) -> None:
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=_exact_identity("warehouse", 3),
        changed=True,
    )
    backend = FieldBackend(environment={})
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: {
            "ok": True,
            "success": True,
            "active": "warehouse",
            "restored_identity": {
                **_exact_identity_payload("warehouse", 3),
                "artifacts": [
                    {
                        "type": "POINTCLOUD",
                        "uri": "/maps/warehouse/.versions/3/map.pcd",
                        "hash": "warehouse-drifted-sha",
                    }
                ],
            },
            "transaction": {
                "operation": "restore_staged_map_for_runtime_switch",
                "state": "rolled_back",
                "target_map": "plant-a",
                "previous_active": "warehouse",
                "verified": True,
            },
        },
    )

    with pytest.raises(RuntimeError, match="restored map identity did not match"):
        backend.restore_map(token)


def test_field_backend_commit_verifies_the_exact_target_identity(monkeypatch) -> None:
    token = MapActivationToken(
        target=_exact_identity("plant-a", 7),
        previous=None,
        changed=True,
    )
    backend = FieldBackend(environment={})
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: {
            "ok": True,
            "success": True,
            "active": "plant-a",
            "active_identity": {
                **_exact_identity_payload("plant-a", 7),
            },
            "transaction": {
                "operation": "commit_staged_map_for_runtime_switch",
                "state": "commit_verified",
                "target_map": "plant-a",
                "verified": True,
            },
        },
    )

    backend.commit_map(token)


def test_field_backend_does_not_compensate_with_names_when_stage_token_is_invalid(monkeypatch) -> None:
    calls: list[tuple[str, object]] = []
    backend = FieldBackend(environment={})

    def fake_http(_method, path, payload=None, **_kwargs):
        calls.append((path, payload))
        if path.endswith("stage-for-runtime-switch"):
            return {
                "ok": True,
                "success": True,
                "active": "plant-a",
                "activation_token": {
                    "schema_version": "lingtu.map_activation.v1",
                    "changed": True,
                    "target": {"map_id": "plant-a", "version": 0, "artifacts": []},
                    "previous": {"map_id": "warehouse", "version": 3, "artifacts": []},
                },
                "transaction": {
                    "operation": "stage_map_for_runtime_switch",
                    "state": "staged",
                    "target_map": "plant-a",
                    "previous_active": "warehouse",
                    "runtime_consistent": False,
                    "restart_required": True,
                },
            }
        raise AssertionError("an invalid token must not be downgraded to map names")

    monkeypatch.setattr(backend, "_http", fake_http)

    with pytest.raises(RuntimeError, match="complete typed compensation token"):
        backend.stage_map("plant-a")

    assert calls == [("/api/v1/map/stage-for-runtime-switch", {"name": "plant-a"})]


def test_invalid_staged_response_with_failed_internal_compensation_is_fail_closed(
    monkeypatch,
    tmp_path,
) -> None:
    launcher = FakeLauncher()
    backend = FieldBackend(environment={})
    events: list[str] = []

    monkeypatch.setattr(
        backend,
        "resolve_map",
        lambda map_name: (map_name, f"/maps/{map_name}/map.pcd"),
    )
    monkeypatch.setattr(
        backend,
        "stop_motion_and_session",
        lambda: events.append("stop_motion_and_session"),
    )
    monkeypatch.setattr(
        backend,
        "disable_boot_ownership",
        lambda _manifest: events.append("disable_boot_ownership"),
    )

    def fake_http(_method, path, _payload=None, **_kwargs):
        if path.endswith("stage-for-runtime-switch"):
            return {
                "ok": True,
                "success": True,
                "active": "plant-a",
                "activation_token": {
                    "schema_version": "lingtu.map_activation.v1",
                    "changed": True,
                    "target": {"map_id": "plant-a", "version": 0, "artifacts": []},
                    "previous": {"map_id": "warehouse", "version": 3, "artifacts": []},
                },
                "transaction": {
                    "operation": "stage_map_for_runtime_switch",
                    "state": "staged",
                    "target_map": "plant-a",
                    "previous_active": "warehouse",
                    "runtime_consistent": False,
                    "restart_required": True,
                },
            }
        raise AssertionError(path)

    monkeypatch.setattr(backend, "_http", fake_http)

    with pytest.raises(SwitchFailed) as failure:
        _control(launcher).switch(_request(), backend=backend, state_dir=tmp_path)

    report = failure.value.report
    assert report.status == "rollback_failed"
    assert "complete typed compensation token" in str(report.error)
    assert events == [
        "stop_motion_and_session",
        "stop_motion_and_session",
        "disable_boot_ownership",
    ]
    assert [call[0] for call in launcher.calls] == ["quiesce"]
    assert not (tmp_path / "active-product.json").exists()


def test_product_control_owns_the_complete_cold_switch_order(tmp_path) -> None:
    launcher = FakeLauncher()
    backend = FakeBackend()

    report = _control(launcher).switch(
        _request(),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    assert report.phases == [
        "preflight",
        "manifest_published",
        "motion_stopped",
        "map_prepared",
        "runtime_config_staged",
        "boot_ownership_staged",
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
        "resolve_map",
        "stop_motion_and_session",
        "stage_map",
        "stage_runtime_config",
        "persist_boot_ownership",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
        "wait_navigation",
        "commit_map",
    ]
    assert [call[0] for call in launcher.calls] == ["apply"]
    assert (tmp_path / "active-product.json").is_file()


def test_product_control_switches_map_v5_through_runtime_staging_and_commit(
    monkeypatch,
    tmp_path,
) -> None:
    launcher = FakeLauncher()
    installed: list[tuple[str, str, str]] = []
    events: list[str] = []

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("field switch must not invoke a real command runner")

    backend = FieldBackend(environment={}, runner=reject_real_command)
    monkeypatch.setattr(backend, "_read_dropin", lambda _unit, _name: None)
    monkeypatch.setattr(
        backend,
        "_install_dropin",
        lambda target, name, content: installed.append((target, name, content)),
    )
    monkeypatch.setattr(backend, "_remove_dropin", lambda *_args: None)
    monkeypatch.setattr(backend, "_sudo", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        backend,
        "stop_motion_and_session",
        lambda: events.append("stop_motion_and_session"),
    )
    monkeypatch.setattr(
        backend,
        "persist_boot_ownership",
        lambda _manifest: events.append("persist_boot_ownership"),
    )
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
    monkeypatch.setattr(
        backend,
        "disable_boot_ownership",
        lambda _manifest: events.append("disable_boot_ownership"),
    )

    report = _control(launcher).switch(
        SwitchRequest(
            target_profile="map",
            current_profile="teleop",
            endpoint="thunder_field",
        ),
        backend=backend,
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    assert report.product is not None
    assert report.product["schema_version"] == "lingtu.product.v5"
    assert report.phases == [
        "preflight",
        "manifest_published",
        "motion_stopped",
        "runtime_config_staged",
        "boot_ownership_staged",
        "stale_status_cleared",
        "processes_active",
        "native_nav_ready",
        "slam_ready",
        "session_active",
        "committed",
    ]
    assert events == [
        "stop_motion_and_session",
        "persist_boot_ownership",
        "clear_runtime_status",
        "wait_native_nav",
        "wait_slam",
        "start_session",
    ]
    assert launcher.calls == [("apply", report.fingerprint)]

    assert report.manifest_path is not None
    manifest_path = Path(report.manifest_path)
    manifest = ProductManifest.load(manifest_path)
    host_process = manifest.process("host")
    assert host_process.application == "map_control_plane"
    host_config = host_process.config
    host_dropin = next(content for target, _name, content in installed if target == "lingtu.service")
    for config_key, environment_key in (
        ("_endpoint_transport", "LINGTU_ENDPOINT_TRANSPORT"),
        ("_endpoint_contract", "LINGTU_ENDPOINT_CONTRACT"),
        ("command_output_mode", "LINGTU_COMMAND_OUTPUT_MODE"),
        ("hardware_control_boundary", "LINGTU_HARDWARE_CONTROL_BOUNDARY"),
    ):
        assert f'Environment="{environment_key}={host_config[config_key]}"' in host_dropin

    active = json.loads((tmp_path / "active-product.json").read_text(encoding="utf-8"))
    assert active["profile"] == "map"
    assert active["manifest_path"] == str(manifest_path)
    assert active["fingerprint"] == report.fingerprint


def test_product_switch_rolls_back_staged_config_and_uncommitted_manifest(
    monkeypatch,
    tmp_path,
) -> None:
    class FailingLauncher(FakeLauncher):
        def apply(self, product, *, dry_run: bool = False):
            super().apply(product, dry_run=dry_run)
            raise RuntimeError("apply failed after staging")

    launcher = FailingLauncher()
    original_dropins = {
        (
            "lingtu.service",
            "product-mode.conf",
        ): '[Service]\nEnvironment="LINGTU_PROFILE=teleop"\n'
    }
    dropins = dict(original_dropins)

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("rollback test must not invoke a real command runner")

    backend = FieldBackend(environment={}, runner=reject_real_command)
    monkeypatch.setattr(
        backend,
        "_read_dropin",
        lambda unit, name: dropins.get((unit, name)),
        raising=False,
    )
    monkeypatch.setattr(
        backend,
        "_install_dropin",
        lambda unit, name, content: dropins.__setitem__((unit, name), content),
    )
    monkeypatch.setattr(
        backend,
        "_remove_dropin",
        lambda unit, name: dropins.pop((unit, name), None),
    )
    monkeypatch.setattr(backend, "_sudo", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(backend, "stop_motion_and_session", lambda: None)
    monkeypatch.setattr(backend, "persist_boot_ownership", lambda _manifest: None)
    monkeypatch.setattr(backend, "clear_runtime_status", lambda: None)
    monkeypatch.setattr(backend, "disable_boot_ownership", lambda _manifest: None)

    with pytest.raises(SwitchFailed, match="apply failed after staging") as failure:
        _control(launcher).switch(
            SwitchRequest(
                target_profile="map",
                current_profile="teleop",
                endpoint="thunder_field",
            ),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.manifest_path is not None
    assert not Path(report.manifest_path).exists()
    assert not (tmp_path / "active-product.json").exists()
    assert dropins == original_dropins
    assert report.cleanup == [
        "motion_session:stopped",
        "processes:stopped",
        "boot_ownership:disabled",
        "runtime_config:restored",
        "manifest:removed",
    ]


def test_product_switch_reports_staging_and_rollback_failures(
    monkeypatch,
    tmp_path,
) -> None:
    launcher = FakeLauncher()

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("staging failure test must not invoke a real command runner")

    def fail_install(_unit: str, _name: str, _content: str) -> None:
        raise RuntimeError("staging install failed")

    def fail_remove(_unit: str, _name: str) -> None:
        raise RuntimeError("rollback remove failed")

    backend = FieldBackend(environment={}, runner=reject_real_command)
    monkeypatch.setattr(backend, "_read_dropin", lambda _unit, _name: None)
    monkeypatch.setattr(backend, "_install_dropin", fail_install)
    monkeypatch.setattr(backend, "_remove_dropin", fail_remove)
    monkeypatch.setattr(backend, "_sudo", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(backend, "stop_motion_and_session", lambda: None)
    monkeypatch.setattr(backend, "disable_boot_ownership", lambda _manifest: None)

    with pytest.raises(SwitchFailed) as failure:
        _control(launcher).switch(
            SwitchRequest(
                target_profile="map",
                current_profile="teleop",
                endpoint="thunder_field",
            ),
            backend=backend,
            state_dir=tmp_path,
        )

    assert failure.value.report.error is not None
    assert "staging install failed" in failure.value.report.error
    assert "rollback remove failed" in failure.value.report.error
    assert not (tmp_path / "active-product.json").exists()


def test_product_switch_applies_the_verified_persisted_manifest(tmp_path) -> None:
    launcher = FakeLauncher()

    class TamperingBackend(FakeBackend):
        def stage_runtime_config(self, manifest_path, manifest, native_environment, **kwargs) -> None:
            super().stage_runtime_config(manifest_path, manifest, native_environment, **kwargs)
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            payload["profile"] = "teleop"
            manifest_path.write_text(json.dumps(payload), encoding="utf-8")

    backend = TamperingBackend()

    with pytest.raises(SwitchFailed, match="fingerprint mismatch"):
        _control(launcher).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    assert [call[0] for call in launcher.calls] == ["quiesce"]
    assert not (tmp_path / "active-product.json").exists()


def test_product_switch_failure_quiesces_processes_and_boot_ownership(tmp_path) -> None:
    trace: list[str] = []

    class TracedLauncher(FakeLauncher):
        def apply(self, product, *, dry_run: bool = False):
            trace.append("launcher.apply")
            return super().apply(product, dry_run=dry_run)

        def quiesce(self, product, *, dry_run: bool = False):
            trace.append("launcher.quiesce")
            return super().quiesce(product, dry_run=dry_run)

    class TracedBackend(FakeBackend):
        def _event(self, name: str) -> None:
            trace.append(name)
            super()._event(name)

    launcher = TracedLauncher()
    backend = TracedBackend(fail_at="start_session")

    with pytest.raises(SwitchFailed) as failure:
        _control(launcher).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.ok is False
    assert report.status == "failed"
    assert report.error == "failed at start_session"
    assert [call[0] for call in launcher.calls] == ["apply", "quiesce"]
    assert trace[-5:] == [
        "start_session",
        "stop_motion_and_session",
        "restore_map",
        "launcher.quiesce",
        "disable_boot_ownership",
    ]
    assert report.cleanup == [
        "motion_session:stopped",
        "map:restored",
        "processes:stopped",
        "boot_ownership:disabled",
        "manifest:removed",
    ]
    assert not (tmp_path / "active-product.json").exists()


@pytest.mark.parametrize(
    "fail_at",
    [
        "stage_runtime_config",
        "persist_boot_ownership",
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
    launcher = FakeLauncher()
    backend = FakeBackend(fail_at=fail_at)

    with pytest.raises(SwitchFailed):
        _control(launcher).switch(_request(), backend=backend, state_dir=tmp_path)

    assert backend.events.count("restore_map") == 1
    cleanup_stop = len(backend.events) - 1 - backend.events[::-1].index("stop_motion_and_session")
    restore = backend.events.index("restore_map")
    assert cleanup_stop < restore
    assert [call[0] for call in launcher.calls][-1] == "quiesce"
    assert not (tmp_path / "active-product.json").exists()


def test_launcher_failure_after_map_prepare_restores_the_map_exactly_once(tmp_path) -> None:
    class FailingLauncher(FakeLauncher):
        def apply(self, product, *, dry_run: bool = False):
            super().apply(product, dry_run=dry_run)
            raise RuntimeError("launcher apply failed")

    launcher = FailingLauncher()
    backend = FakeBackend()

    with pytest.raises(SwitchFailed, match="launcher apply failed"):
        _control(launcher).switch(_request(), backend=backend, state_dir=tmp_path)

    assert backend.events.count("restore_map") == 1
    assert [call[0] for call in launcher.calls] == ["apply", "quiesce"]


def test_map_restore_failure_is_fail_closed_and_reports_rollback_failed(tmp_path) -> None:
    class RestoreFailingBackend(FakeBackend):
        def restore_map(self, token: MapActivationToken) -> None:
            super().restore_map(token)
            raise RuntimeError("restored map identity did not verify")

    launcher = FakeLauncher()
    backend = RestoreFailingBackend(fail_at="start_session")

    with pytest.raises(SwitchFailed) as failure:
        _control(launcher).switch(_request(), backend=backend, state_dir=tmp_path)

    report = failure.value.report
    assert report.status == "rollback_failed"
    assert backend.events.count("restore_map") == 1
    assert "map_failed:restored map identity did not verify" in report.cleanup
    assert backend.events.count("stop_motion_and_session") == 2
    assert [call[0] for call in launcher.calls] == ["apply", "quiesce"]
    assert not (tmp_path / "active-product.json").exists()


def test_product_switch_cleanup_attempts_all_steps_when_each_step_fails(tmp_path) -> None:
    trace: list[str] = []

    class CleanupFailingLauncher(FakeLauncher):
        def apply(self, product, *, dry_run: bool = False):
            trace.append("launcher.apply")
            return super().apply(product, dry_run=dry_run)

        def quiesce(self, product, *, dry_run: bool = False):
            trace.append("launcher.quiesce")
            self.calls.append(("quiesce", product.fingerprint))
            raise RuntimeError("quiesce cleanup failed")

    class CleanupFailingBackend(FakeBackend):
        def __init__(self) -> None:
            super().__init__(fail_at="start_session")
            self.stop_calls = 0

        def _event(self, name: str) -> None:
            trace.append(name)
            super()._event(name)

        def stop_motion_and_session(self) -> None:
            self.stop_calls += 1
            super().stop_motion_and_session()
            if self.stop_calls > 1:
                raise RuntimeError("motion cleanup failed")

        def disable_boot_ownership(self, manifest) -> None:
            super().disable_boot_ownership(manifest)
            raise RuntimeError("boot cleanup failed")

    launcher = CleanupFailingLauncher()
    backend = CleanupFailingBackend()

    with pytest.raises(SwitchFailed) as failure:
        _control(launcher).switch(
            _request(),
            backend=backend,
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.error == "failed at start_session"
    assert trace[-5:] == [
        "start_session",
        "stop_motion_and_session",
        "restore_map",
        "launcher.quiesce",
        "disable_boot_ownership",
    ]
    assert report.cleanup == [
        "motion_session_failed:motion cleanup failed",
        "map:restored",
        "processes_failed:quiesce cleanup failed",
        "boot_ownership_failed:boot cleanup failed",
        "manifest:removed",
    ]
    assert [call[0] for call in launcher.calls] == ["apply", "quiesce"]
    assert not (tmp_path / "active-product.json").exists()


def test_product_switch_rejects_missing_map_before_mutation(tmp_path) -> None:
    launcher = FakeLauncher()
    backend = FakeBackend()
    request = SwitchRequest(
        target_profile="nav",
        current_profile="teleop",
        endpoint="thunder_field",
    )

    with pytest.raises(SwitchFailed, match="requires a map") as failure:
        _control(launcher).switch(request, backend=backend, state_dir=tmp_path)

    assert failure.value.report.cleanup == []
    assert launcher.calls == []
    assert backend.events == []
    assert list(tmp_path.iterdir()) == []


def test_product_switch_dry_run_has_no_runtime_side_effects(tmp_path) -> None:
    launcher = FakeLauncher()
    backend = FakeBackend()

    report = _control(launcher).switch(
        _request(),
        backend=backend,
        state_dir=tmp_path,
        dry_run=True,
    )

    assert report.ok is True
    assert report.status == "planned"
    assert report.phases == ["preflight"]
    assert backend.events == ["resolve_map"]
    assert launcher.calls == []
    assert list(tmp_path.iterdir()) == []


def test_field_backend_rejects_map_path_escape(tmp_path) -> None:
    maps = tmp_path / "maps"
    maps.mkdir()
    backend = FieldBackend(environment={"NAV_MAP_DIR": str(maps), "HOME": str(tmp_path)})

    with pytest.raises(RuntimeError, match="unsafe map name"):
        backend.resolve_map("../outside")


def test_field_backend_rejects_non_http_gateway_url() -> None:
    with pytest.raises(ValueError, match="HTTP"):
        FieldBackend(gateway_url="file:///tmp/control.json")


def test_field_backend_stages_compiled_host_control_contract(monkeypatch, tmp_path) -> None:
    from lingtu.assembly.profile_builder import compile_product
    from runtime.profiles.native_nav_config import native_nav_profile_config
    from runtime.profiles.resolver import resolve_runtime_config

    resolved = resolve_runtime_config("teleop_avoid")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    manifest = product.manifest()
    installed: list[tuple[str, str, str]] = []
    backend = FieldBackend(
        environment={},
        runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout="",
            stderr="",
        ),
    )
    monkeypatch.setattr(
        backend,
        "_install_dropin",
        lambda target, name, content: installed.append((target, name, content)),
    )
    monkeypatch.setattr(backend, "_remove_dropin", lambda *_args: None)

    backend.stage_runtime_config(
        tmp_path / "product.json",
        manifest,
        native_nav_profile_config(product.profile, product.config).environment,
        slam_mode="none",
        map_path="",
    )

    host_dropin = next(content for target, _name, content in installed if target == "lingtu.service")
    assert 'Environment="LINGTU_PRODUCT_PROFILE=teleop_avoid"' in host_dropin
    assert 'Environment="LINGTU_COMMAND_OUTPUT_MODE=endpoint_only"' in host_dropin
    assert 'Environment="LINGTU_HARDWARE_CONTROL_BOUNDARY=driver"' in host_dropin
    assert f'Environment="LINGTU_PRODUCT_FINGERPRINT={manifest.fingerprint}"' in host_dropin


def test_field_backend_rejects_unknown_manifest_schema_before_staging_dropins(
    monkeypatch,
) -> None:
    from lingtu.assembly.profile_builder import compile_product
    from runtime.profiles.native_nav_config import native_nav_profile_config
    from runtime.profiles.resolver import resolve_runtime_config

    resolved = resolve_runtime_config("map")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    manifest = replace(
        product.manifest(),
        schema_version="lingtu.product.unknown",
    )
    installed: list[tuple[str, str, str]] = []

    def reject_real_command(*_args, **_kwargs):
        raise AssertionError("unknown schema must fail before reading systemd state")

    backend = FieldBackend(environment={}, runner=reject_real_command)
    monkeypatch.setattr(
        backend,
        "_install_dropin",
        lambda target, name, content: installed.append((target, name, content)),
    )

    with pytest.raises(RuntimeError, match="unsupported Product manifest schema"):
        backend.stage_runtime_config(
            Path("unused-product.json"),
            manifest,
            native_nav_profile_config(product.profile, product.config).environment,
            slam_mode="mapping",
            map_path="",
        )

    assert installed == []


def test_field_backend_does_not_guess_current_product() -> None:
    def run(_command, **_kwargs):
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(environment={}, runner=run)

    with pytest.raises(RuntimeError, match="pass --current explicitly"):
        backend.current_profile()


def test_field_backend_clears_all_stale_runtime_evidence() -> None:
    commands: list[list[str]] = []

    def run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    backend = FieldBackend(
        environment={
            "LINGTU_NAV_STATUS_FILE": "/dev/shm/lingtu/nav.json",
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
