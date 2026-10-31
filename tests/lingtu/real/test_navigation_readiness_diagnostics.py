import pytest

from lingtu.real.backend import FieldBackend


def test_autonomy_start_failure_reports_actual_goal_admission(monkeypatch):
    ticks = iter((0.0, 0.0, 2.0))
    backend = FieldBackend(environment={}, monotonic=lambda: next(ticks), sleep=lambda _: None)
    monkeypatch.setattr(backend, "_http", lambda *_args, **_kwargs: {
        "task": {"state": "IDLE"},
        "goal_admission": {"state": "BLOCKED", "reason": "native_product_parameters_mismatch"},
        "control": {"authority": "NONE"},
        "motion": {"permission": "HELD", "observation": "QUIET", "stop_confirmation": "NOT_REQUESTED"},
    })
    with pytest.raises(RuntimeError, match=r"goal_admission=.*native_product_parameters_mismatch"):
        backend.wait_navigation(map_name="plant-a", control_mode="autonomy", timeout_s=1.0)
