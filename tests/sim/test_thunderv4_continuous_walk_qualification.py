# ruff: noqa: S101

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from sim.scripts.mujoco import continuous_walk as qualification_module
from sim.scripts.mujoco.continuous_walk import run_qualification

REPO_ROOT = Path(__file__).resolve().parents[2]
V101_ROBOT_XML = (
    REPO_ROOT
    / "sim"
    / "packages"
    / "robots"
    / "thunderv4"
    / "1.0.1"
    / "mjcf"
    / "thunderv4.xml"
)


def test_thunderv4_headless_qualification_reports_nominal_stand_warmup() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["checks"]["nominal_stand_warmup"] is True


def test_thunderv4_headless_qualification_reports_continuous_forward_walk() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["checks"]["continuous_forward_walk"] is True
    assert report["walk"]["signed_forward_displacement_m"] > 0.30
    assert report["walk"]["active_average_forward_speed_mps"] >= 0.50
    assert report["walk"]["phase_average_forward_speed_mps"] > 0.0
    assert report["walk"]["command"]["linear_x"] == pytest.approx(0.6)


def test_policy_1119_qualification_uses_its_reference_control_timing() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["timing"] == {
        "physics_timestep_s": pytest.approx(0.005),
        "low_level_hz": pytest.approx(200.0),
        "inference_hz": pytest.approx(50.0),
        "startup_stand_hold_s": pytest.approx(0.5),
    }


def test_thunderv4_headless_qualification_reports_reasonable_height_and_attitude() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["checks"]["base_height_and_attitude"] is True


def test_thunderv4_headless_qualification_rejects_non_finite_runtime_state() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["checks"]["all_state_values_finite"] is True


def test_thunderv4_headless_qualification_reports_stop_after_command_release() -> None:
    pytest.importorskip("mujoco")

    report = run_qualification(seed=7)

    assert report["checks"]["release_stops"] is True
    assert report["release"]["horizontal_displacement_m"] <= qualification_module.MAX_RELEASE_DISPLACEMENT_M
    assert report["release"]["final_horizontal_speed_mps"] <= 0.05


def test_thunderv4_headless_qualification_is_repeatable_for_a_fixed_seed() -> None:
    pytest.importorskip("mujoco")

    first = run_qualification(seed=7)
    second = run_qualification(seed=7)

    assert first["qualified"] is True
    assert second["qualified"] is True
    assert first["warmup"]["end_position_m"] == pytest.approx(second["warmup"]["end_position_m"])
    assert first["walk"]["signed_forward_displacement_m"] == pytest.approx(
        second["walk"]["signed_forward_displacement_m"]
    )
    assert first["release"]["final_horizontal_speed_mps"] == pytest.approx(
        second["release"]["final_horizontal_speed_mps"]
    )


def test_explicit_v101_mjcf_is_loaded_and_cross_bound_in_the_report(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("mujoco")
    original_build_engine = qualification_module.build_engine
    observed: dict[str, Path] = {}

    def recording_build_engine(**kwargs: Any):
        observed["world"] = Path(kwargs["world"])
        observed["robot_xml"] = Path(kwargs["robot_xml"])
        observed["policy_path"] = Path(kwargs["policy_path"])
        return original_build_engine(**kwargs)

    monkeypatch.setattr(qualification_module, "build_engine", recording_build_engine)

    report = run_qualification(seed=7, robot_xml=V101_ROBOT_XML)

    assert observed["world"] == qualification_module.WORLD_XML
    assert observed["robot_xml"] == V101_ROBOT_XML
    assert observed["policy_path"] == qualification_module.POLICY
    assert report["artifacts"] == {
        "world_mjcf": "sim/packages/worlds/open_field/physics/open_field.xml",
        "robot_mjcf": "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml",
        "policy": "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx",
    }


def test_close_failure_still_restores_numpy_random_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    before = repr(np.random.get_state())

    class CloseFailureEngine:
        control_dt = 1.0

        def __init__(self) -> None:
            self._state = qualification_module.RobotState(
                position=np.array([0.0, 0.0, 0.4]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
                joint_positions=np.zeros(16),
                joint_velocities=np.zeros(16),
                imu_gyro=np.zeros(3),
                imu_projected_gravity=np.array([0.0, 0.0, -1.0]),
            )

        def get_robot_state(self) -> qualification_module.RobotState:
            return self._state

        def step(
            self,
            _command: qualification_module.VelocityCommand | None = None,
        ) -> qualification_module.RobotState:
            return self._state

        def close(self) -> None:
            raise RuntimeError("close failed")

    monkeypatch.setattr(
        qualification_module,
        "_build_engine",
        lambda **_kwargs: CloseFailureEngine(),
    )

    with pytest.raises(RuntimeError, match="close failed"):
        run_qualification(warmup_s=1.0, walk_s=1.0, release_s=1.0)

    assert repr(np.random.get_state()) == before


def test_robot_xml_cli_forwards_explicit_path_and_preserves_the_legacy_default(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    calls: list[dict[str, object]] = []

    def fake_run_qualification(**kwargs: object) -> dict[str, object]:
        calls.append(dict(kwargs))
        return {"qualified": True}

    monkeypatch.setattr(qualification_module, "run_qualification", fake_run_qualification)

    assert qualification_module.main(["--robot-xml", V101_ROBOT_XML.as_posix()]) == 0
    assert calls[-1]["robot_xml"] == V101_ROBOT_XML
    assert qualification_module.main([]) == 0
    assert calls[-1]["robot_xml"] == qualification_module.ROBOT_XML
    capsys.readouterr()


@pytest.mark.parametrize(
    "unsafe_path",
    (
        Path(os.__file__),
        REPO_ROOT / "sim",
    ),
)
def test_robot_xml_rejects_repo_external_and_nonregular_paths(
    unsafe_path: Path,
) -> None:
    with pytest.raises(ValueError, match="regular repository file"):
        run_qualification(robot_xml=unsafe_path)


def test_cli_failure_report_does_not_expose_an_unsafe_absolute_path(
    capsys: pytest.CaptureFixture[str],
) -> None:
    unsafe_path = Path(os.__file__)

    assert qualification_module.main(["--robot-xml", str(unsafe_path)]) == 1

    report = json.loads(capsys.readouterr().out)
    assert report["qualified"] is False
    assert str(unsafe_path) not in report["error"]["message"]


def test_cli_unexpected_failure_does_not_expose_an_absolute_path(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    def unexpected_failure(**_kwargs: object) -> dict[str, object]:
        raise RuntimeError(f"engine failed while loading {REPO_ROOT}")

    monkeypatch.setattr(
        qualification_module,
        "run_qualification",
        unexpected_failure,
    )

    assert qualification_module.main([]) == 1

    report = json.loads(capsys.readouterr().out)
    assert report["qualified"] is False
    assert report["error"]["type"] == "RuntimeError"
    assert str(REPO_ROOT) not in report["error"]["message"]
