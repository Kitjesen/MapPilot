"""An explicitly authored UE session remains inside the single Product plan."""

import pytest

from lingtu.assembly.compile_plan import compile_run_plan

pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


def test_explicit_unreal_session_is_not_silently_changed_to_headless(monkeypatch):
    from sim.catalog import CatalogResolver

    original = CatalogResolver.resolve
    sources = []

    def resolve(self, source):
        sources.append(source)
        return original(self, source)

    monkeypatch.setattr(CatalogResolver, "resolve", resolve)
    plan = compile_run_plan("inspection", "sim", robot="doso/thunder_v4", env_config={
        "backend": "mujoco", "session": "sim/sessions/examples/thunderv4_unreal/session.yaml",
    })
    assert plan.simulation["session"]["runtime"]["mode"] == "unreal"
    assert len(sources) == 1
    assert sum(process.name == "mujoco_feeder" for process in plan.processes) == 1
    session_id = plan.simulation["session"]["session_id"]
    for field in ("physics_plan", "visual_plan", "sensor_plan", "control_plan", "transport_intent"):
        assert plan.simulation[field]["session_id"] == session_id


def test_unreal_session_does_not_open_a_second_mujoco_viewer():
    with pytest.raises(ValueError, match=r"Unreal.*viewer"):
        compile_run_plan("inspection", "sim", robot="doso/thunder_v4", env_config={
            "backend": "mujoco", "viewer": True,
            "session": "sim/sessions/examples/thunderv4_unreal/session.yaml",
        })
