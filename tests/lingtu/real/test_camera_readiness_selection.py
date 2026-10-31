from __future__ import annotations

from functools import lru_cache

import pytest

from lingtu.control import ProductControl
from lingtu.real.systemd import ServiceReadiness, SystemdRunner, _ServiceInspector
from lingtu.switch_contracts import ProcessError, ProcessReport


@lru_cache(maxsize=2)
def _plan(robot):
    return ProductControl(env="real", robot=robot, process_env={})._resolve(
        "nav", product_variant="camera", local_planner="scan",
    )


class _NoProcessMutation:
    def available(self, _target):
        return True

    def active(self, _target):
        return True

    def start(self, *_args):
        pytest.fail("readiness selection must not start processes")

    def stop(self, *_args):
        pytest.fail("readiness selection must not stop processes")


def _setup(tmp_path, monkeypatch, *, binaries):
    for name in binaries:
        path = tmp_path / name
        path.write_bytes(b"fixture")
        path.chmod(0o755)
    environment = {
        "LINGTU_CAMERA_DRIVER": "orbbec_native",
        "LINGTU_CAMERA_DDS_BIN": str(tmp_path / "lingtu_camera_dds"),
        "LINGTU_ORBBEC_CAPTURE_BIN": str(tmp_path / "orbbec_capture"),
        "LINGTU_REALSENSE_CAPTURE_BIN": str(tmp_path / "realsense_capture"),
    }
    inspector = _ServiceInspector(systemd=_NoProcessMutation(), environment=environment)
    monkeypatch.setattr(inspector, "_status_files", lambda _spec, _blockers: {"ok": True, "files": []})
    monkeypatch.setattr(inspector, "_dds", lambda _spec, _blockers: {"ok": True})
    readiness = ServiceReadiness(inspector)
    runner = SystemdRunner(_NoProcessMutation(), readiness, environment={})
    return runner, readiness, inspector


@pytest.mark.parametrize("robot,capture", [("unitree/go2", "realsense_capture"), ("doso/thunder_v4", "orbbec_capture")])
def test_selected_robot_camera_passes_without_unselected_sdk(tmp_path, monkeypatch, robot, capture):
    runner, readiness, _ = _setup(tmp_path, monkeypatch, binaries=["lingtu_camera_dds", capture])
    plan = _plan(robot)
    runner.apply(plan, dry_run=True)
    detail = readiness.inspect(plan.process("camera"))
    assert detail["ready"] is True
    assert [item["name"] for item in detail["observed"]["native_binary"]["binaries"]] == ["camera_dds", capture]
    assert [item["name"] for item in detail["contract"]["binaries"]] == ["camera_dds", capture]


@pytest.mark.parametrize("robot,selected,other", [
    ("unitree/go2", "realsense_capture", "orbbec_capture"),
    ("doso/thunder_v4", "orbbec_capture", "realsense_capture"),
])
def test_missing_selected_sdk_fails_even_when_other_sdk_exists(tmp_path, monkeypatch, robot, selected, other):
    runner, readiness, _ = _setup(tmp_path, monkeypatch, binaries=["lingtu_camera_dds", other])
    plan = _plan(robot)
    runner.apply(plan, dry_run=True)
    with pytest.raises(ProcessError, match=f"native_binary_missing_or_not_executable:{selected}"):
        readiness.inspect(plan.process("camera"))


def test_selected_capture_binary_override_is_honored(tmp_path, monkeypatch):
    runner, readiness, inspector = _setup(tmp_path, monkeypatch, binaries=["lingtu_camera_dds", "custom-capture"])
    inspector._environment["LINGTU_REALSENSE_CAPTURE_BIN"] = str(tmp_path / "custom-capture")
    plan = _plan("unitree/go2")
    runner.apply(plan, dry_run=True)
    detail = readiness.inspect(plan.process("camera"))
    assert detail["observed"]["native_binary"]["binaries"][1]["path"] == str(tmp_path / "custom-capture")


def test_restore_rebinds_previous_plan_camera_selection(tmp_path, monkeypatch):
    runner, readiness, inspector = _setup(tmp_path, monkeypatch, binaries=["lingtu_camera_dds", "orbbec_capture", "realsense_capture"])
    inspect = inspector.status_details
    monkeypatch.setattr(inspector, "status_details", lambda service, **kwargs:
                        inspect(service, **kwargs) if service == "camera" else {service: {"ready": True}})
    previous, other = _plan("unitree/go2"), _plan("doso/thunder_v4")
    runner.apply(other, dry_run=True)
    assert readiness.inspect(other.process("camera"))["contract"]["binaries"][1]["name"] == "orbbec_capture"
    runner.restore_transition_previous(previous, ProcessReport(product="nav", env="real", action="transition"))
    assert readiness.inspect(previous.process("camera"))["contract"]["binaries"][1]["name"] == "realsense_capture"
