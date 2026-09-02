from types import SimpleNamespace

from lingtu.assembly.stacks.system import run_startup_preflight


def test_slam_preflight_does_not_require_camera_without_semantics(monkeypatch) -> None:
    observed: list[dict[str, bool]] = []

    def fake_calibration_check(**requirements: bool) -> SimpleNamespace:
        observed.append(requirements)
        return SimpleNamespace(ok=True, errors=[])

    monkeypatch.setattr(
        "runtime.utils.calibration_check.run_calibration_check",
        fake_calibration_check,
    )

    run_startup_preflight(enable_semantic=False, slam_profile="native_dds")

    assert observed == [{"require_camera": False, "require_slam": True}]


def test_semantic_preflight_still_requires_camera(monkeypatch) -> None:
    observed: list[dict[str, bool]] = []

    def fake_calibration_check(**requirements: bool) -> SimpleNamespace:
        observed.append(requirements)
        return SimpleNamespace(ok=True, errors=[])

    monkeypatch.setattr(
        "runtime.utils.calibration_check.run_calibration_check",
        fake_calibration_check,
    )

    run_startup_preflight(enable_semantic=True, slam_profile="none")

    assert observed == [{"require_camera": True, "require_slam": False}]
