"""The existing Product feeder owns optional UE presentation and its cleanup."""

import pytest

from lingtu.run_plan import RunPlan
from tests.sim.test_formal_feeder import FakeServices, _run, _run_plan


class Visual:
    def __init__(self, services, *, ready_after=2, failure=None):
        self.services = services
        self.ready_after = ready_after
        self.failure = failure
        self.frames = 0

    def start(self):
        self.services.events.append("visual_start")
        if self.failure == "start":
            raise RuntimeError("synthetic visual startup failure")

    def publish(self):
        self.frames += 1
        self.services.events.append("visual_frame")
        if self.failure == "runtime" and self.services.readiness_payload is not None:
            raise RuntimeError("owned Unreal exited")

    def ready(self):
        return self.failure != "timeout" and self.frames >= self.ready_after

    def republish(self):
        self.services.events.append("visual_repeat")

    def close(self):
        self.services.events.append("visual_close")


def unreal_plan():
    payload = _run_plan().as_dict()
    payload["launch"]["simulation"]["session"]["runtime"]["mode"] = "unreal"
    return RunPlan.from_dict(payload)


def services_with_visual(**options):
    services = FakeServices(stop_after_readiness=False, stop_after_waits=25)
    visual = Visual(services, **options)

    def create_visual(**kwargs):
        assert kwargs["engine"] is services.engine
        assert kwargs["product_session_id"] == services.expected_product_session_id
        services.events.append("create_visual")
        return visual

    services.create_visual = create_visual
    return services, visual


def test_same_feeder_waits_for_ue_evidence_before_product_ready(monkeypatch, tmp_path):
    services, visual = services_with_visual()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=unreal_plan())
    assert rc == 0
    assert services.events.count("build_engine") == 1
    assert visual.frames >= 2
    assert services.events.index("visual_start") < services.events.index("activate")
    assert services.events.index("visual_frame") < services.events.index("publish_readiness")
    assert services.events.index("visual_close") < services.events.index("engine_close")
    assert services.events.count("visual_close") == 1


def test_headless_never_constructs_or_operates_visual_component(monkeypatch, tmp_path):
    services, visual = services_with_visual()
    rc, _, _ = _run(monkeypatch, tmp_path, services)
    assert rc == 0
    assert visual.frames == 0
    assert "create_visual" not in services.events


@pytest.mark.parametrize("camera_ready", [True, False])
@pytest.mark.parametrize("close_failure", [False, True])
def test_ue_camera_is_owned_by_same_feeder_and_blocks_ready_until_real_pixels(
    monkeypatch, tmp_path, camera_ready, close_failure,
):
    import json
    from types import SimpleNamespace

    from sim.scripts.mujoco import formal_feeder

    from tests.sim.test_formal_camera_selection import selected_plan
    services, visual = services_with_visual(ready_after=1)
    document = selected_plan(rgb_hz=30, depth_hz=30).as_dict()
    simulation = document["launch"]["simulation"]
    simulation["session"]["runtime"]["mode"] = "unreal"
    for kind in ("rgb", "depth"):
        for stream in simulation["sensor_plan"]["streams"][kind]:
            stream.update(owner="visual", source="unreal_camera", transport="camera_shm")
    selected = RunPlan.from_dict(document)
    visual.allocation = SimpleNamespace(session_id=simulation["sensor_plan"]["session_id"])
    original_publish = visual.publish

    def publish():
        original_publish()
        return dict(session_id=visual.allocation.session_id, sequence=visual.frames,
                    sim_time_ns=round(services.engine.sim_time * 1e9), model_generation=0, reset_generation=0)

    visual.publish = publish

    class Relay:
        def __init__(self, **kwargs):
            assert kwargs["allocation"] is visual.allocation
            assert kwargs["client"] is services.sensors["camera_publisher"]
            assert kwargs["sensor_plan"] == simulation["sensor_plan"]
            self.observed = 0
            self.startup_join = None
            self.closed = False
            services.events.append("camera_relay_create")

        def observe_truth(self, snapshot, *, epoch_s):
            assert snapshot["sequence"] == visual.frames and epoch_s > 0
            if services.readiness_payload is None:
                join = (snapshot["sequence"], snapshot["sim_time_ns"], epoch_s)
                if self.startup_join is not None:
                    assert join == self.startup_join
                self.startup_join = join
            self.observed += 1

        def publish_due(self, **kwargs):
            services.events.append("camera_relay_poll")

        def ready(self):
            return camera_ready and self.observed >= 4

        def begin(self, **kwargs):
            services.events.append("camera_relay_begin")

        def evidence(self):
            return dict(source="unreal_camera", observed=self.observed)

        def close(self):
            if not self.closed:
                services.events.append("camera_relay_close")
                self.closed = True
            if close_failure:
                raise RuntimeError("synthetic endpoint publication failure")

    services.create_camera_relay = Relay
    visual.republish = lambda: dict(session_id=visual.allocation.session_id, sequence=visual.frames,
        sim_time_ns=0, model_generation=0, reset_generation=0)
    monkeypatch.setattr(formal_feeder, "_VISUAL_STARTUP_TIMEOUT_S", 0.2)
    rc, identity, _ = _run(monkeypatch, tmp_path, services, plan=selected)
    assert rc == (0 if camera_ready and not close_failure else 1)
    evidence = json.loads((identity.session_root / "camera_relay.evidence.json").read_text())
    assert evidence["source"] == "unreal_camera" and evidence["observed"] >= 4
    assert services.build_kwargs["camera_configs"] is None
    assert services.engine.camera_render_threads == []
    assert services.events.count("build_engine") == services.events.count("camera_relay_create") == 1
    assert services.events.index("camera_relay_close") < services.events.index("visual_close")
    assert services.events.count("camera_relay_close") == 1
    assert (services.readiness_payload is not None) == camera_ready
    if camera_ready:
        assert services.events.index("camera_relay_poll") < services.events.index("activate")
    else:
        assert "activate" not in services.events


def test_visual_truth_keeps_thirty_hz_remainder_on_two_hundred_hz_physics(monkeypatch, tmp_path):
    services, visual = services_with_visual(ready_after=1)
    services.stop_after_waits = 230
    observed = []
    publish = visual.publish

    def record():
        result = publish()
        if services.readiness_payload is not None:
            observed.append(services.now)
        return result

    visual.publish = record
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=unreal_plan())
    assert rc == 0 and len(observed) > 30
    actual_hz = (len(observed) - 1) / (observed[-1] - observed[0])
    assert 29.5 <= actual_hz <= 30.5


@pytest.mark.parametrize("reason", ["timeout", "cancel"])
def test_visual_startup_is_bounded_and_does_not_activate_driver(monkeypatch, tmp_path, reason):
    from sim.scripts.mujoco import formal_feeder

    services, _ = services_with_visual(failure="timeout")
    if reason == "timeout":
        monkeypatch.setattr(formal_feeder, "_VISUAL_STARTUP_TIMEOUT_S", 0.2)
    else:
        services.stop_after_waits = 2
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=unreal_plan())
    assert rc == 1
    assert services.readiness_payload is None
    assert "activate" not in services.events
    assert services.events.count("visual_close") == 1
    assert services.events.count("engine_close") == 1


def test_explicit_unreal_rejects_cli_mujoco_viewer_before_engine(monkeypatch, tmp_path):
    services, visual = services_with_visual()
    rc, _, _ = _run(monkeypatch, tmp_path, services, "--viewer", plan=unreal_plan())
    assert rc == 1
    assert services.build_kwargs is None
    assert visual.frames == 0


def test_visual_owner_preserves_same_product_measurement_control_and_raw_archive(monkeypatch, tmp_path):
    from tests.sim import test_formal_measurement_feeder as measurement_case

    original_services = measurement_case.FakeServices
    original_plan = measurement_case._with_measurements
    created = []

    def services(**kwargs):
        instance = original_services(**kwargs)
        visual = Visual(instance, ready_after=1)

        def create_visual(**binding):
            assert binding["engine"] is instance.engine
            assert binding["product_session_id"] == instance.expected_product_session_id
            return visual

        instance.create_visual = create_visual
        created.append((instance, visual))
        return instance

    def selected_plan():
        payload = original_plan().as_dict()
        payload["launch"]["simulation"]["session"]["runtime"]["mode"] = "unreal"
        return RunPlan.from_dict(payload)

    monkeypatch.setattr(measurement_case, "FakeServices", services)
    monkeypatch.setattr(measurement_case, "_with_measurements", selected_plan)
    # Reuse the actual Kelvin measurement/ACK/archive assertions, with only the
    # optional UE process boundary replaced; no extra physics owner is created.
    measurement_case.test_formal_feeder_sends_temperature_over_owned_camera_not_imu(monkeypatch, tmp_path, late=False)
    assert len(created) == 1
    owner, visual = created[0]
    assert visual.frames > 1
    assert owner.events.count("build_engine") == 1
    assert owner.events.count("visual_close") == 1


@pytest.mark.parametrize("failure", ["start", "runtime"])
def test_visual_failure_reuses_feeder_shutdown_and_closes_owned_component(monkeypatch, tmp_path, failure):
    services, _ = services_with_visual(ready_after=1, failure=failure)
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=unreal_plan())
    assert rc == 1
    assert services.events.count("visual_close") == 1
    assert services.events.count("engine_close") == 1
    if failure == "start":
        assert services.readiness_payload is None
        assert "activate" not in services.events
    else:
        assert services.evidence_payload["terminal_ack"] is True
        assert "deactivate_zero" in services.events
