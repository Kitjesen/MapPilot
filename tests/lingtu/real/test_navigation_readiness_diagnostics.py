import pytest

from lingtu.real.backend import FieldBackend


@pytest.mark.parametrize("state", ["IDLE", "UNKNOWN"])
def test_navigation_endpoint_readiness_uses_task_state_without_motion_admission(monkeypatch, state):
    ticks = iter((0.0, 0.0, 2.0))
    backend = FieldBackend(environment={}, monotonic=lambda: next(ticks), sleep=lambda _: None)
    monkeypatch.setattr(backend, "_http", lambda *_args, **_kwargs: {
        "task": {"state": state},
        "goal_admission": {"state": "BLOCKED", "reason": "native_product_parameters_mismatch"},
        "control": {"authority": "NONE"},
        "motion": {"permission": "HELD", "observation": "QUIET", "stop_confirmation": "NOT_REQUESTED"},
    })
    if state == "UNKNOWN":
        with pytest.raises(RuntimeError, match=r"mode=autonomy map=plant-a state=UNKNOWN"):
            backend.wait_navigation(map_name="plant-a", control_mode="autonomy", timeout_s=1.0)
    else:
        backend.wait_navigation(map_name="plant-a", control_mode="autonomy", timeout_s=1.0)
