# ruff: noqa: S101,S603,S607

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.coordinator import (
    STATIC_PLAN_FILES,
    RunAllocation,
    RunAllocationError,
    RunAllocationErrorCode,
    create_run_allocation as _create_run_allocation,
    load_resolved_session_bundle,
    load_run_allocation,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
RUN_ALLOCATION_SCHEMA = (
    REPO_ROOT / "sim" / "contracts" / "schemas" / "run-allocation.v1.json"
)
CATALOG_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunder_omni_contract"
    / "session.yaml"
)
def _canonical_json(value: object) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def _validate_schema(instance: object, schema: dict[str, object]) -> None:
    expected_type = schema.get("type")
    if expected_type == "object" and not isinstance(instance, dict):
        raise AssertionError("expected object")
    if expected_type == "string" and not isinstance(instance, str):
        raise AssertionError("expected string")
    if expected_type == "integer" and (
        isinstance(instance, bool) or not isinstance(instance, int)
    ):
        raise AssertionError("expected integer")
    if "const" in schema and instance != schema["const"]:
        raise AssertionError("const mismatch")
    if isinstance(instance, str):
        if len(instance) < int(schema.get("minLength", 0)):
            raise AssertionError("string too short")
        if "maxLength" in schema and len(instance) > int(schema["maxLength"]):
            raise AssertionError("string too long")
        if "pattern" in schema and re.search(str(schema["pattern"]), instance) is None:
            raise AssertionError("pattern mismatch")
    if isinstance(instance, int) and not isinstance(instance, bool):
        if "minimum" in schema and instance < int(schema["minimum"]):
            raise AssertionError("below minimum")
        if "maximum" in schema and instance > int(schema["maximum"]):
            raise AssertionError("above maximum")
    if isinstance(instance, dict):
        required = set(schema.get("required", []))
        if not required.issubset(instance):
            raise AssertionError("missing required property")
        properties = schema.get("properties", {})
        assert isinstance(properties, dict)
        if schema.get("additionalProperties") is False and set(instance) - set(properties):
            raise AssertionError("unknown property")
        property_names = schema.get("propertyNames")
        additional = schema.get("additionalProperties")
        for key, value in instance.items():
            if isinstance(property_names, dict):
                _validate_schema(key, property_names)
            child = properties.get(key)
            if isinstance(child, dict):
                _validate_schema(value, child)
            elif isinstance(additional, dict):
                _validate_schema(value, additional)


def _camera_streams() -> dict[str, object]:
    return {
        "rgb": [
            {
                "instance_id": "thunder_01",
                "sensor_id": "thunder_01.front_rgb",
                "owner": "visual",
                "source": "unreal_camera",
                "frame_id": "thunder_01/front_camera",
                "rate_hz": 30,
                "transport": "camera_shm",
                "message_type": "lingtu.dds.Image",
                "width": 1280,
                "height": 720,
                "encoding": "rgb8",
            }
        ],
        "depth": [
            {
                "instance_id": "thunder_01",
                "sensor_id": "thunder_01.front_depth",
                "owner": "visual",
                "source": "unreal_camera",
                "frame_id": "thunder_01/front_camera",
                "rate_hz": 30,
                "transport": "camera_shm",
                "message_type": "lingtu.dds.Image",
                "width": 640,
                "height": 480,
                "encoding": "32FC1",
                "unit": "m",
            }
        ],
        "imu": [
            {
                "instance_id": "thunder_01",
                "sensor_id": "thunder_01.imu",
                "owner": "physics",
                "source": "mujoco_imu",
                "frame_id": "thunder_01/imu",
                "rate_hz": 200,
                "transport": "typed_dds",
                "message_type": "lingtu.dds.Imu",
                "fields": ["orientation", "angular_velocity", "linear_acceleration"],
            }
        ],
        "mid360": [],
        "truth_odom": [],
    }


def create_run_allocation(*args: object, **kwargs: object) -> RunAllocation:
    bundle_dir = Path(kwargs.get("bundle_dir", args[0] if args else ""))
    kwargs.setdefault("repo_root", bundle_dir.parent)
    return _create_run_allocation(*args, **kwargs)


def _write_bundle(
    root: Path,
    *,
    sensor_streams: object | None = None,
    physics_timestep_s: float | None = None,
) -> tuple[Path, str]:
    if sensor_streams is not None and physics_timestep_s is None:
        physics_timestep_s = 0.001
    bundle = root / "bundle"
    bundle.mkdir()
    session_id = "allocation-contract"
    (bundle / "session.yaml").write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.session.v1",
                "session_id": session_id,
                "mujoco_version": "3.10.0",
                "seed": 7,
                "world": "test.world@1.0.0",
                "robots": [],
                "runtime": {
                    "backend": "mujoco",
                    "mode": "headless",
                    "required_bindings": ["physics"],
                },
            }
        ),
        encoding="utf-8",
    )
    schemas = {
        "physics.plan.json": "lingtu.sim.physics-plan.v1",
        "visual.plan.json": "lingtu.sim.visual-plan.v1",
        "sensor.plan.json": "lingtu.sim.sensor-plan.v1",
        "control.plan.json": "lingtu.sim.control-plan.v1",
        "transport.intent.json": "lingtu.sim.transport-intent.v1",
    }
    for filename, schema in schemas.items():
        document: dict[str, object] = {
            "schema": schema,
            "session_id": session_id,
        }
        if filename == "sensor.plan.json" and sensor_streams is not None:
            document.update(
                {
                    "env": "sim",
                    "backends": {"physics": "mujoco", "visual": "unreal"},
                    "streams": sensor_streams,
                }
            )
        if filename == "physics.plan.json" and physics_timestep_s is not None:
            document["global_policy"] = {
                "owner": "world",
                "timestep_s": physics_timestep_s,
            }
        if filename == "transport.intent.json":
            document["allocation_boundary"] = {
                "owner": "RunAllocation",
                "runtime_values_external": True,
            }
        (bundle / filename).write_text(json.dumps(document), encoding="utf-8")
    return bundle, session_id


def _default_camera_shm(run_id: str, sensor_id: str) -> str:
    return f"lingtu.sim.camera_shm.{run_id}.{sensor_id}"


def _write_runtime_manifest(
    allocation: RunAllocation,
    *,
    state: str,
    physics_pid: int | None,
) -> None:
    manifest = {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": allocation.run_id,
        "session_id": allocation.session_id,
        "state": state,
        "allocation": {
                "run_dir": str(allocation.run_dir),
                "log_dir": str(allocation.log_dir),
            "boot_id": allocation.boot_id,
            "physics_pid": physics_pid,
            "dds_domain": allocation.dds_domain,
            "ports": dict(allocation.ports),
            "shm": dict(allocation.shm),
        },
    }
    temporary = allocation.run_dir / "session.runtime.json.tmp"
    temporary.write_text(json.dumps(manifest), encoding="utf-8")
    temporary.replace(allocation.run_dir / "session.runtime.json")


def test_create_run_allocation_from_valid_resolved_bundle(tmp_path: Path) -> None:
    bundle, session_id = _write_bundle(tmp_path)

    resolved = load_resolved_session_bundle(bundle, repo_root=tmp_path)
    allocation = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs",
        run_id="run-contract-001",
        boot_id="123e4567-e89b-12d3-a456-426614174000",
        dds_domain=83,
        ports={"gateway": 15050, "mcp": 18090},
        shm={"front_camera": "lingtu-run-contract-001-camera"},
    )

    assert tuple(resolved.plans) == STATIC_PLAN_FILES
    assert resolved.repo_root == tmp_path.resolve()
    assert resolved.session_id == session_id
    assert allocation.session_id == session_id
    assert allocation.artifact_root == tmp_path.resolve()
    payload = json.loads(allocation.path.read_text(encoding="utf-8"))
    assert payload == {
        "schema": "lingtu.sim.run-allocation.v1",
        "run_id": "run-contract-001",
        "session_id": session_id,
        "artifact_root": str(tmp_path.resolve()),
        "boot_id": "123e4567-e89b-12d3-a456-426614174000",
        "dds_domain": 83,
        "ports": {"gateway": 15050, "mcp": 18090},
        "shm": {"front_camera": "lingtu-run-contract-001-camera"},
        "log_dir": str((tmp_path / "runs" / "run-contract-001" / "logs").resolve()),
    }
    assert allocation.log_dir.is_dir()
    assert (allocation.run_dir / "screenshots").is_dir()


def test_create_run_allocation_can_bind_artifact_root_to_owned_run_directory(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)

    allocation = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs",
        run_id="run-owned-artifacts",
        boot_id="boot-owned-artifacts",
        dds_domain=83,
        artifact_root_mode="run",
    )

    assert allocation.artifact_root == allocation.run_dir
    payload = json.loads(allocation.path.read_text(encoding="utf-8"))
    assert payload["artifact_root"] == str(allocation.run_dir)


def test_create_run_allocation_rejects_unknown_artifact_root_mode(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=tmp_path / "runs",
            run_id="run-invalid-artifact-mode",
            boot_id="boot-invalid-artifact-mode",
            dds_domain=83,
            artifact_root_mode="foreign",  # type: ignore[arg-type]
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert not (tmp_path / "runs" / "run-invalid-artifact-mode").exists()


def test_create_run_allocation_rejects_non_text_artifact_root_mode(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=tmp_path / "runs",
            run_id="run-nontext-artifact-mode",
            boot_id="boot-nontext-artifact-mode",
            dds_domain=83,
            artifact_root_mode=[],  # type: ignore[arg-type]
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert not (tmp_path / "runs" / "run-nontext-artifact-mode").exists()


def test_run_artifact_root_rejects_noncanonical_traversal_run_root(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    foreign = tmp_path / "foreign-runs"
    traversal_root = tmp_path / "approved-runs" / ".." / foreign.name

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=traversal_root,
            run_id="run-traversal-artifacts",
            boot_id="boot-traversal-artifacts",
            dds_domain=83,
            artifact_root_mode="run",
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert not foreign.exists()


def test_run_artifact_root_rejects_relative_owner_root(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    monkeypatch.chdir(tmp_path)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=Path("relative-runs"),
            run_id="run-relative-artifacts",
            boot_id="boot-relative-artifacts",
            dds_domain=83,
            artifact_root_mode="run",
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert not (tmp_path / "relative-runs").exists()


def test_run_artifact_root_rejects_symlink_owner_root(tmp_path: Path) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    foreign = tmp_path / "foreign-symlink-runs"
    foreign.mkdir()
    run_root = tmp_path / "symlink-runs"
    try:
        run_root.symlink_to(foreign, target_is_directory=True)
    except (NotImplementedError, OSError):
        pytest.skip("directory symbolic links are unavailable in this test environment")

    try:
        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle_dir=bundle,
                run_root=run_root,
                run_id="run-symlink-artifacts",
                boot_id="boot-symlink-artifacts",
                dds_domain=83,
                artifact_root_mode="run",
            )
    finally:
        run_root.unlink(missing_ok=True)

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert list(foreign.iterdir()) == []


def test_run_artifact_root_rejects_windows_junction_run_root(
    tmp_path: Path,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")
    bundle, _session_id = _write_bundle(tmp_path)
    foreign = tmp_path / "foreign-runs"
    foreign.mkdir()
    run_root = tmp_path / "linked-runs"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(run_root), str(foreign)],
        capture_output=True,
        text=True,
        check=False,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    try:
        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle_dir=bundle,
                run_root=run_root,
                run_id="run-junction-artifacts",
                boot_id="boot-junction-artifacts",
                dds_domain=83,
                artifact_root_mode="run",
            )
    finally:
        run_root.rmdir()

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert list(foreign.iterdir()) == []


def test_run_artifact_root_revalidates_new_run_directory_before_writing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction race regression test")
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    run_dir = run_root / "run-raced-artifacts"
    displaced = tmp_path / "displaced-run-artifacts"
    foreign = tmp_path / "foreign-run-artifacts"
    foreign.mkdir()
    original_mkdir = Path.mkdir
    raced = False

    def mkdir_with_junction_swap(
        path: Path,
        *args: object,
        **kwargs: object,
    ) -> None:
        nonlocal raced
        original_mkdir(path, *args, **kwargs)
        if path != run_dir or raced:
            return
        raced = True
        path.rename(displaced)
        created = subprocess.run(
            ["cmd", "/c", "mklink", "/J", str(path), str(foreign)],
            capture_output=True,
            text=True,
            check=False,
        )
        if created.returncode != 0:
            displaced.rename(path)
            pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    monkeypatch.setattr(Path, "mkdir", mkdir_with_junction_swap)
    try:
        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle_dir=bundle,
                run_root=run_root,
                run_id=run_dir.name,
                boot_id="boot-raced-artifacts",
                dds_domain=83,
                artifact_root_mode="run",
            )
    finally:
        if run_dir.exists():
            run_dir.rmdir()

    assert raced
    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert list(foreign.iterdir()) == []


def test_create_run_allocation_rejects_existing_run_directory_by_default(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-strict-existing"
    run_dir.mkdir(parents=True)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-strict-existing",
            dds_domain=83,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert list(run_dir.iterdir()) == []


def test_create_run_allocation_adopts_trusted_existing_empty_run_directory(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-empty"
    run_dir.mkdir(parents=True)

    allocation = create_run_allocation(
        bundle,
        run_dir.parent,
        run_id=run_dir.name,
        boot_id="boot-adopt-empty",
        dds_domain=83,
        adopt_existing_empty_run_dir=True,
    )

    assert allocation.run_dir == run_dir.resolve()
    assert allocation.log_dir.is_dir()
    assert (allocation.run_dir / "screenshots").is_dir()
    assert allocation.path.is_file()


def test_create_run_allocation_rejects_nonempty_directory_when_adoption_enabled(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-nonempty"
    run_dir.mkdir(parents=True)
    sentinel = run_dir / "service-owned.txt"
    sentinel.write_text("preserve me\n", encoding="utf-8")

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-adopt-nonempty",
            dds_domain=83,
            adopt_existing_empty_run_dir=True,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert sentinel.read_text(encoding="utf-8") == "preserve me\n"
    assert sorted(path.name for path in run_dir.iterdir()) == [sentinel.name]


def test_create_run_allocation_rejects_preexisting_screenshot_directory(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-stale-screenshots"
    screenshot_dir = run_dir / "screenshots"
    screenshot_dir.mkdir(parents=True)
    sentinel = screenshot_dir / "stale.png"
    sentinel.write_bytes(b"not a current-run capture")

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-adopt-stale-screenshots",
            dds_domain=83,
            adopt_existing_empty_run_dir=True,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert sentinel.read_bytes() == b"not a current-run capture"
    assert sorted(path.name for path in run_dir.iterdir()) == ["screenshots"]


def test_create_run_allocation_rejects_screenshot_link_without_following_it(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-linked-screenshots"
    run_dir.mkdir(parents=True)
    foreign = tmp_path / "foreign-screenshots"
    foreign.mkdir()
    sentinel = foreign / "preserve.txt"
    sentinel.write_text("preserve me\n", encoding="utf-8")
    screenshot_link = run_dir / "screenshots"
    try:
        screenshot_link.symlink_to(foreign, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"cannot create directory symlink: {exc}")

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-adopt-linked-screenshots",
            dds_domain=83,
            adopt_existing_empty_run_dir=True,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert sentinel.read_text(encoding="utf-8") == "preserve me\n"
    assert screenshot_link.is_symlink()


def test_create_run_allocation_rejects_symlink_directory_when_adoption_enabled(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-symlink"
    run_dir.parent.mkdir()
    foreign = tmp_path / "foreign-run"
    foreign.mkdir()
    try:
        run_dir.symlink_to(foreign, target_is_directory=True)
    except (NotImplementedError, OSError):
        pytest.skip("directory symbolic links are unavailable in this test environment")

    try:
        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle,
                run_dir.parent,
                run_id=run_dir.name,
                boot_id="boot-adopt-symlink",
                dds_domain=83,
                adopt_existing_empty_run_dir=True,
            )
    finally:
        run_dir.unlink(missing_ok=True)

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert list(foreign.iterdir()) == []


def test_create_run_allocation_rejects_windows_junction_when_adoption_enabled(
    tmp_path: Path,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-junction"
    run_dir.parent.mkdir()
    foreign = tmp_path / "foreign-junction-run"
    foreign.mkdir()
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(run_dir), str(foreign)],
        capture_output=True,
        text=True,
        check=False,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    try:
        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle,
                run_dir.parent,
                run_id=run_dir.name,
                boot_id="boot-adopt-junction",
                dds_domain=83,
                adopt_existing_empty_run_dir=True,
            )
    finally:
        run_dir.rmdir()

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert list(foreign.iterdir()) == []


def test_adoption_revalidates_every_directory_below_the_trusted_root(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    studio_root = tmp_path / "studio"
    run_id = "run-adopt-ancestor-link"
    artifacts_root = studio_root / "artifacts"
    runs_root = artifacts_root / "runs"
    run_dir = runs_root / run_id
    run_dir.mkdir(parents=True)
    displaced = tmp_path / "displaced-artifacts"
    foreign = tmp_path / "foreign-artifacts"
    (foreign / "runs" / run_id).mkdir(parents=True)
    artifacts_root.rename(displaced)
    link_created = False
    try:
        if sys.platform == "win32":
            created = subprocess.run(
                ["cmd", "/c", "mklink", "/J", str(artifacts_root), str(foreign)],
                capture_output=True,
                text=True,
                check=False,
            )
            if created.returncode != 0:
                displaced.rename(artifacts_root)
                pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
        else:
            try:
                artifacts_root.symlink_to(foreign, target_is_directory=True)
            except (NotImplementedError, OSError):
                displaced.rename(artifacts_root)
                pytest.skip("directory symbolic links are unavailable in this test environment")
        link_created = True

        with pytest.raises(RunAllocationError) as captured:
            create_run_allocation(
                bundle,
                runs_root,
                run_id=run_id,
                boot_id="boot-adopt-ancestor-link",
                dds_domain=83,
                adopt_existing_empty_run_dir=True,
                trusted_root=studio_root,
            )

        assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
        assert not (foreign / "runs" / run_id / "logs").exists()
        assert not (foreign / "runs" / run_id / "run-allocation.json").exists()
    finally:
        if link_created:
            if sys.platform == "win32":
                os.rmdir(artifacts_root)
            else:
                artifacts_root.unlink()
            displaced.rename(artifacts_root)


def test_create_run_allocation_rejects_nondirectory_when_adoption_enabled(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-file"
    run_dir.parent.mkdir()
    run_dir.write_text("preserve me\n", encoding="utf-8")

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-adopt-file",
            dds_domain=83,
            adopt_existing_empty_run_dir=True,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert run_dir.read_text(encoding="utf-8") == "preserve me\n"


def test_adopted_windows_run_directory_is_pinned_during_materialization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows run-directory handle regression test")
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-pinned"
    run_dir.mkdir(parents=True)
    displaced = tmp_path / "displaced-run"
    log_dir = run_dir / "logs"
    original_mkdir = Path.mkdir
    race_attempted = False
    race_blocked = False

    def mkdir_with_swap(path: Path, *args: object, **kwargs: object) -> None:
        nonlocal race_attempted, race_blocked
        if path == log_dir and not race_attempted:
            race_attempted = True
            try:
                run_dir.rename(displaced)
            except OSError:
                race_blocked = True
            else:
                displaced.rename(run_dir)
                raise AssertionError("adopted run directory was swappable after validation")
        original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", mkdir_with_swap)

    allocation = create_run_allocation(
        bundle,
        run_dir.parent,
        run_id=run_dir.name,
        boot_id="boot-adopt-pinned",
        dds_domain=83,
        adopt_existing_empty_run_dir=True,
    )

    assert allocation.run_dir == run_dir.resolve()
    assert race_attempted
    assert race_blocked
    assert not displaced.exists()


def test_adopted_windows_ancestor_chain_is_pinned_during_materialization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows ancestor-handle regression test")
    bundle, _session_id = _write_bundle(tmp_path)
    studio_root = tmp_path / "studio"
    artifacts_root = studio_root / "artifacts"
    run_dir = artifacts_root / "runs" / "run-adopt-chain-pinned"
    run_dir.mkdir(parents=True)
    displaced = tmp_path / "displaced-artifacts-chain"
    log_dir = run_dir / "logs"
    original_mkdir = Path.mkdir
    race_attempted = False
    race_blocked = False

    def mkdir_with_ancestor_swap(path: Path, *args: object, **kwargs: object) -> None:
        nonlocal race_attempted, race_blocked
        if path == log_dir and not race_attempted:
            race_attempted = True
            try:
                artifacts_root.rename(displaced)
            except OSError:
                race_blocked = True
            else:
                displaced.rename(artifacts_root)
                raise AssertionError("trusted artifact ancestor was swappable after validation")
        original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", mkdir_with_ancestor_swap)

    allocation = create_run_allocation(
        bundle,
        run_dir.parent,
        run_id=run_dir.name,
        boot_id="boot-adopt-chain-pinned",
        dds_domain=83,
        adopt_existing_empty_run_dir=True,
        trusted_root=studio_root,
    )

    assert allocation.run_dir == run_dir
    assert race_attempted
    assert race_blocked
    assert not displaced.exists()


def test_adopted_run_directory_rolls_back_if_it_changes_during_materialization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_dir = tmp_path / "runs" / "run-adopt-raced"
    run_dir.mkdir(parents=True)
    log_dir = run_dir / "logs"
    foreign = run_dir / "foreign-entry.txt"
    original_mkdir = Path.mkdir

    def mkdir_with_foreign_entry(
        path: Path,
        *args: object,
        **kwargs: object,
    ) -> None:
        original_mkdir(path, *args, **kwargs)
        if path == log_dir:
            foreign.write_text("preserve me\n", encoding="utf-8")

    monkeypatch.setattr(Path, "mkdir", mkdir_with_foreign_entry)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_dir.parent,
            run_id=run_dir.name,
            boot_id="boot-adopt-raced",
            dds_domain=83,
            adopt_existing_empty_run_dir=True,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_EXISTS
    assert foreign.read_text(encoding="utf-8") == "preserve me\n"
    assert sorted(path.name for path in run_dir.iterdir()) == [foreign.name]


def test_create_run_allocation_fills_camera_shm_defaults_from_sensor_plan(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path, sensor_streams=_camera_streams())

    allocation = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs",
        run_id="run-camera-001",
        boot_id="boot-camera",
        dds_domain=83,
    )

    assert allocation.shm == {
        "thunder_01.front_rgb": _default_camera_shm(
            "run-camera-001",
            "thunder_01.front_rgb",
        ),
        "thunder_01.front_depth": _default_camera_shm(
            "run-camera-001",
            "thunder_01.front_depth",
        ),
    }
    assert all(len(name) <= 255 for name in allocation.shm.values())
    assert all(not any(char.isspace() or char == "\x00" for char in name) for name in allocation.shm.values())
    assert load_run_allocation(allocation.path) == allocation


def test_camera_shm_defaults_are_run_unique_and_deterministic(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path, sensor_streams=_camera_streams())

    first = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs-a",
        run_id="run-camera-repeat",
        boot_id="boot-a",
        dds_domain=83,
    )
    repeat = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs-b",
        run_id="run-camera-repeat",
        boot_id="boot-b",
        dds_domain=83,
    )
    second = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs-c",
        run_id="run-camera-other",
        boot_id="boot-c",
        dds_domain=83,
    )

    assert first.shm == repeat.shm
    assert first.shm["thunder_01.front_rgb"] != second.shm["thunder_01.front_rgb"]


def test_explicit_camera_shm_overrides_matching_default_and_missing_is_filled(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path, sensor_streams=_camera_streams())

    allocation = create_run_allocation(
        bundle_dir=bundle,
        run_root=tmp_path / "runs",
        run_id="run-camera-override",
        boot_id="boot-camera",
        dds_domain=83,
        shm={"thunder_01.front_rgb": "explicit-front-rgb"},
    )

    assert allocation.shm == {
        "thunder_01.front_rgb": "explicit-front-rgb",
        "thunder_01.front_depth": _default_camera_shm(
            "run-camera-override",
            "thunder_01.front_depth",
        ),
    }


def test_create_run_allocation_rejects_camera_shm_name_collisions(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path, sensor_streams=_camera_streams())

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=tmp_path / "runs",
            run_id="run-camera-collision",
            boot_id="boot-camera",
            dds_domain=83,
            shm={
                "thunder_01.front_depth": _default_camera_shm(
                    "run-camera-collision",
                    "thunder_01.front_rgb",
                )
            },
        )

    assert captured.value.code is RunAllocationErrorCode.RESOURCE_CONFLICT
    assert "shm names conflict" in captured.value.message
    assert not (tmp_path / "runs" / "run-camera-collision").exists()


def test_create_run_allocation_rejects_invalid_camera_sensor_ids(
    tmp_path: Path,
) -> None:
    streams = _camera_streams()
    assert isinstance(streams["rgb"], list)
    streams["rgb"][0]["sensor_id"] = "thunder_01/front_rgb"
    bundle, _session_id = _write_bundle(tmp_path, sensor_streams=streams)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle_dir=bundle,
            run_root=tmp_path / "runs",
            run_id="run-camera-invalid",
            boot_id="boot-camera",
            dds_domain=83,
        )

    assert captured.value.code is RunAllocationErrorCode.ALLOCATION_INVALID
    assert "sensor_id" in captured.value.message
    assert not (tmp_path / "runs" / "run-camera-invalid").exists()


def test_loader_accepts_a_catalog_generated_resolved_session_bundle(
    tmp_path: Path,
) -> None:
    generated = CatalogResolver.from_repository(REPO_ROOT).resolve(CATALOG_SESSION)
    bundle = generated.write_bundle(tmp_path / "catalog-bundle")

    loaded = load_resolved_session_bundle(bundle)

    assert loaded.session_id == generated.session_id
    assert tuple(loaded.plans) == STATIC_PLAN_FILES


def test_loader_rejects_a_physics_sensor_off_the_bundle_timebase(
    tmp_path: Path,
) -> None:
    streams = _camera_streams()
    assert isinstance(streams["imu"], list)
    streams["imu"][0]["rate_hz"] = 200
    bundle, _session_id = _write_bundle(
        tmp_path,
        sensor_streams=streams,
        physics_timestep_s=0.002,
    )

    with pytest.raises(
        RunAllocationError,
        match=r"thunder_01\.imu.*200 Hz.*physics timestep 0\.002 s",
    ) as captured:
        load_resolved_session_bundle(bundle)

    assert captured.value.code is RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID
    assert captured.value.artifact == "sensor.plan.json"


def test_load_bundle_reports_a_stable_error_for_a_missing_plan(tmp_path: Path) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    (bundle / "control.plan.json").unlink()

    with pytest.raises(RunAllocationError) as captured:
        load_resolved_session_bundle(bundle)

    assert captured.value.code is RunAllocationErrorCode.BUNDLE_ARTIFACT_MISSING
    assert captured.value.to_dict() == {
        "schema": "lingtu.sim.run-allocation-error.v1",
        "code": "bundle_artifact_missing",
        "message": "required bundle artifact is missing: control.plan.json",
        "artifact": "control.plan.json",
    }


@pytest.mark.parametrize("filename", STATIC_PLAN_FILES)
def test_load_bundle_rejects_a_session_id_mismatch_in_every_static_plan(
    tmp_path: Path,
    filename: str,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    plan_path = bundle / filename
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["session_id"] = "different-session"
    plan_path.write_text(json.dumps(plan), encoding="utf-8")

    with pytest.raises(RunAllocationError) as captured:
        load_resolved_session_bundle(bundle)

    assert captured.value.code is RunAllocationErrorCode.SESSION_ID_MISMATCH
    assert captured.value.artifact == filename


def test_load_bundle_rejects_duplicate_json_keys_as_an_invalid_artifact(
    tmp_path: Path,
) -> None:
    bundle, session_id = _write_bundle(tmp_path)
    (bundle / "physics.plan.json").write_text(
        "{"
        '"schema":"lingtu.sim.physics-plan.v1",'
        f'"session_id":"{session_id}",'
        f'"session_id":"{session_id}"'
        "}",
        encoding="utf-8",
    )

    with pytest.raises(RunAllocationError) as captured:
        load_resolved_session_bundle(bundle)

    assert captured.value.code is RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID
    assert captured.value.artifact == "physics.plan.json"


@pytest.mark.parametrize(
    ("dds_domain", "ports", "shm", "resource"),
    [
        (83, {"metrics": 16000}, {"rear_camera": "other-camera-shm"}, "dds_domain"),
        (84, {"metrics": 15050}, {"rear_camera": "other-camera-shm"}, "port"),
        (84, {"metrics": 16000}, {"rear_camera": "camera-shm"}, "shm"),
    ],
)
def test_create_run_allocation_rejects_resources_reserved_on_the_same_boot(
    tmp_path: Path,
    dds_domain: int,
    ports: dict[str, int],
    shm: dict[str, str],
    resource: str,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    create_run_allocation(
        bundle,
        run_root,
        run_id="run-a",
        boot_id="boot-test",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_root,
            run_id="run-b",
            boot_id="boot-test",
            dds_domain=dds_domain,
            ports=ports,
            shm=shm,
        )

    assert captured.value.code is RunAllocationErrorCode.RESOURCE_CONFLICT
    assert resource in captured.value.message
    assert not (run_root / "run-b").exists()


@pytest.mark.parametrize("state", ["STOPPED", "FAILED"])
def test_terminal_runtime_manifest_releases_same_boot_resources(
    tmp_path: Path,
    state: str,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    first = create_run_allocation(
        bundle,
        run_root,
        run_id="run-terminal",
        boot_id="boot-terminal",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )
    _write_runtime_manifest(first, state=state, physics_pid=4242)

    replacement = create_run_allocation(
        bundle,
        run_root,
        run_id="run-replacement",
        boot_id="boot-terminal",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )

    assert replacement.dds_domain == first.dds_domain
    assert replacement.ports == first.ports
    assert replacement.shm == first.shm


@pytest.mark.parametrize(
    ("dds_domain", "ports", "shm", "resource"),
    [
        (83, {"metrics": 16000}, {"rear_camera": "other-camera-shm"}, "dds_domain"),
        (84, {"metrics": 15050}, {"rear_camera": "other-camera-shm"}, "port"),
        (84, {"metrics": 16000}, {"rear_camera": "camera-shm"}, "shm"),
    ],
)
def test_ready_runtime_manifest_keeps_same_boot_resources_reserved_without_a_pid(
    tmp_path: Path,
    dds_domain: int,
    ports: dict[str, int],
    shm: dict[str, str],
    resource: str,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    first = create_run_allocation(
        bundle,
        run_root,
        run_id="run-ready",
        boot_id="boot-ready",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )
    _write_runtime_manifest(first, state="READY", physics_pid=None)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_root,
            run_id="run-conflict",
            boot_id="boot-ready",
            dds_domain=dds_domain,
            ports=ports,
            shm=shm,
        )

    assert captured.value.code is RunAllocationErrorCode.RESOURCE_CONFLICT
    assert resource in captured.value.message


def test_unreleased_previous_boot_keeps_runtime_resources_reserved(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    create_run_allocation(
        bundle,
        run_root,
        run_id="run-previous-boot",
        boot_id="boot-previous",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_root,
            run_id="run-current-boot",
            boot_id="boot-current",
            dds_domain=83,
            ports={"gateway": 15050},
            shm={"front_camera": "camera-shm"},
        )

    assert captured.value.code is RunAllocationErrorCode.RESOURCE_CONFLICT
    assert "dds_domain" in captured.value.message


def test_terminal_runtime_manifest_must_match_the_immutable_allocation(
    tmp_path: Path,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    first = create_run_allocation(
        bundle,
        run_root,
        run_id="run-terminal",
        boot_id="boot-terminal",
        dds_domain=83,
    )
    _write_runtime_manifest(first, state="STOPPED", physics_pid=None)
    manifest_path = first.run_dir / "session.runtime.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["allocation"]["boot_id"] = "different-boot"
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest), encoding="utf-8")
    temporary.replace(manifest_path)

    with pytest.raises(RunAllocationError) as captured:
        create_run_allocation(
            bundle,
            run_root,
            run_id="run-conflict",
            boot_id="boot-terminal",
            dds_domain=83,
        )

    assert captured.value.code is RunAllocationErrorCode.RESOURCE_CONFLICT
    assert "dds_domain" in captured.value.message


def test_run_allocation_has_a_schema_checked_stable_serialization(tmp_path: Path) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    allocation = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="run-serialize",
        boot_id="boot-serialize",
        dds_domain=82,
        ports={"gateway": 25050},
        shm={"front_camera": "serialize-camera-shm"},
    )

    serialized = allocation.to_json()
    assert allocation.path.read_text(encoding="utf-8") == serialized
    assert load_run_allocation(allocation.path) == allocation

    schema = json.loads(RUN_ALLOCATION_SCHEMA.read_text(encoding="utf-8"))
    payload = json.loads(serialized)
    assert schema["$id"] == "lingtu.sim.run-allocation.v1"
    assert schema["additionalProperties"] is False
    assert set(schema["required"]) == set(payload)
    _validate_schema(payload, schema)
    invalid = {**payload, "artifact_root": "relative/artifacts"}
    with pytest.raises(AssertionError, match="pattern mismatch"):
        _validate_schema(invalid, schema)


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({}, "exactly the v1 fields"),
        ({"artifact_root": "relative/artifacts"}, "artifact_root"),
        ({"artifact_root": "C:/not-normalized"}, "artifact_root"),
        ({"artifact_root": "C:\\bad\x00root"}, "artifact_root"),
    ],
)
def test_load_run_allocation_rejects_missing_relative_or_malformed_artifact_root(
    tmp_path: Path,
    override: dict[str, str],
    message: str,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    allocation = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="run-bad-artifact-root",
        boot_id="boot-bad-artifact-root",
        dds_domain=82,
    )
    payload = allocation.to_dict()
    if override:
        payload.update(override)
    else:
        payload.pop("artifact_root")
    allocation.path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(RunAllocationError, match=message):
        load_run_allocation(allocation.path)


def test_run_allocation_isolates_one_boot_identity_for_all_children(
    tmp_path: Path,
) -> None:
    allocation = RunAllocation(
        run_id="run-child-environment",
        run_dir=tmp_path / "run",
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "run" / "logs",
        ports={},
        shm={},
        session_id="contract-session",
        boot_id="coordinator-boot-42",
        dds_domain=42,
    )
    parent = {"KEEP_ME": "yes", "LINGTU_HOST_BOOT_ID": "stale-parent"}

    first = allocation.child_environment(parent)
    second = allocation.child_environment(parent)

    assert parent["LINGTU_HOST_BOOT_ID"] == "stale-parent"
    assert first == second == {
        "KEEP_ME": "yes",
        "LINGTU_HOST_BOOT_ID": "coordinator-boot-42",
    }
    assert first is not second
    assert first["LINGTU_HOST_BOOT_ID"] != allocation.session_id


def test_default_allocation_inherits_the_acceptance_runner_boot_id(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bundle, _session_id = _write_bundle(tmp_path)
    monkeypatch.setenv("LINGTU_HOST_BOOT_ID", "acceptance-run-boot")

    allocation = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="run-inherited-boot",
        dds_domain=82,
    )

    assert allocation.boot_id == "acceptance-run-boot"


def test_every_coordinator_owned_child_uses_the_run_boot_identity() -> None:
    mujoco = (REPO_ROOT / "sim/runtime/coordinator/mujoco_process.py").read_text(
        encoding="utf-8"
    )
    unreal = (REPO_ROOT / "sim/runtime/coordinator/unreal_process.py").read_text(
        encoding="utf-8"
    )
    sensors = (REPO_ROOT / "sim/runtime/sensors/dds_adapter.py").read_text(
        encoding="utf-8"
    )

    assert "env=allocation.child_environment()" in mujoco
    assert "env=self._child_environment(allocation)" in unreal
    assert "return dict(allocation.child_environment())" in unreal
    assert sensors.count(
        "self._child_environment = allocation.child_environment()"
    ) == 3
    assert sensors.count("env=self._child_environment") == 3


def test_runtime_resources_do_not_change_the_session_id(tmp_path: Path) -> None:
    bundle, session_id = _write_bundle(tmp_path)
    bundle_before = {path.name: path.read_bytes() for path in bundle.iterdir()}

    first = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="run-one",
        boot_id="boot-one",
        dds_domain=80,
        ports={"gateway": 15050},
        shm={"camera": "camera-one"},
    )
    second = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="run-two",
        boot_id="boot-two",
        dds_domain=90,
        ports={"gateway": 25050},
        shm={"camera": "camera-two"},
    )

    assert first.session_id == second.session_id == session_id
    assert first.to_dict() != second.to_dict()
    assert {path.name: path.read_bytes() for path in bundle.iterdir()} == bundle_before
