"""Offline presentation contracts over real compiled plans and local MuJoCo."""

import json
import socket
from contextlib import contextmanager, nullcontext
from pathlib import Path

import mujoco
import numpy as np
import pytest
from sim.runtime.coordinator.run_allocation import load_run_allocation
from sim.runtime.coordinator.unreal_process import UnrealProcess

from lingtu.assembly.compile_plan import compile_run_plan
from lingtu.assembly.graph import ProcessArtifact
from lingtu.run_plan import RunPlan

ROOT = Path(__file__).resolve().parents[2]
PRODUCT = "a" * 32


def make_visual(plan, tmp_path, read_state, **kwargs):
    from sim.scripts.mujoco.formal_visual import FormalVisual

    return FormalVisual(
        plan,
        tmp_path,
        PRODUCT,
        read_state,
        editor=tmp_path / "UnrealEditor.exe",
        repo_root=ROOT,
        model_generation=3,
        reset_generation=7,
        **kwargs,
    )


class Process:
    def __init__(self, *args, **kwargs):
        self.real = UnrealProcess(*args, **kwargs)
        self.exit_code = None
        self.closed = 0

    def start(self, **kwargs):
        self.launch = kwargs
        self.argv = self.real.command(
            bundle_dir=kwargs["bundle_dir"],
            allocation=kwargs["allocation"],
            snapshot_port=kwargs["snapshot_port"],
            model_generation=kwargs["plan"].model_generation,
            reset_generation=kwargs["plan"].reset_generation,
        )

    def poll(self):
        return self.exit_code

    def terminate(self):
        self.closed += 1


class Publisher:
    def __init__(self, port):
        self.port = port
        self.events = []
        self.closed = 0

    def publish(self, event):
        self.events.append(event)

    def close(self):
        self.closed += 1


@pytest.fixture(scope="module")
def plan():
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(ProcessArtifact, "from_repository_path", classmethod(lambda cls, root, path: cls(str(path))))
        return compile_run_plan("tracking", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})


@pytest.fixture
def runtime(plan, tmp_path):
    robot = plan.simulation["physics_plan"]["robots"][0]
    model = mujoco.MjModel.from_xml_path(str(ROOT / robot["model"]["mjcf"]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    leases = []

    @contextmanager
    def read_state():
        leases.append(True)
        try:
            yield model, data
        finally:
            leases.pop()

    processes, publishers = [], []

    def process_factory(*args, **kwargs):
        value = Process(*args, **kwargs)
        processes.append(value)
        return value

    def publisher_factory(port):
        value = Publisher(port)
        publishers.append(value)
        return value

    visual = make_visual(
        plan,
        tmp_path,
        read_state,
        process_factory=process_factory,
        publisher_factory=publisher_factory,
    )
    yield visual, model, data, processes, publishers, leases
    visual.close()


def test_start_projects_existing_plan_and_canonical_allocation(runtime, plan, tmp_path):
    visual, _, _, processes, publishers, _ = runtime
    before = plan.as_dict()
    visual.start()
    run_dir = tmp_path / "visual" / PRODUCT
    allocation = load_run_allocation(run_dir)
    assert allocation.run_dir == run_dir
    assert allocation.run_id == allocation.boot_id == PRODUCT
    assert allocation.log_dir == run_dir / "logs"
    assert allocation.artifact_root == ROOT
    assert allocation.session_id == plan.simulation["session"]["session_id"]
    assert allocation.shm == {}
    assert allocation.ports == {"visual_snapshot_udp": publishers[0].port}
    assert 0 < publishers[0].port <= 65535
    for key in ("physics_plan", "visual_plan", "sensor_plan", "control_plan", "transport_intent", "scenario_plan"):
        if plan.simulation.get(key) is not None:
            filename = key.replace("_", ".") + ".json"
            assert json.loads((run_dir / "bundle" / filename).read_text()) == plan.simulation[key]
    assert plan.as_dict() == before
    argv = processes[0].argv
    assert plan.simulation["visual_plan"]["world"]["level"] in argv
    assert "-UnattendedInput" in argv and "-LingTuRuntimeUI" in argv
    assert f"-LingTuMeasurementProductSessionId={PRODUCT}" in argv
    assert f"-LingTuMeasurementGatewayUrl=http://127.0.0.1:{plan.host_config['gateway_port']}" in argv
    assert "-LingTuModelGeneration=3" in argv and "-LingTuResetGeneration=7" in argv


@pytest.mark.parametrize("pose", [(1.2, -0.3, 0.8, 0), (-1, 2, 0.9, 0.7), (0, 0, 1.1, -1.2)])
def test_publish_copies_same_lease_without_changing_physics(runtime, plan, pose):
    visual, model, data, _, publishers, leases = runtime
    visual.start()
    data.qpos[:3] = pose[:3]
    data.qpos[3:7] = [np.cos(pose[3] / 2), 0, 0, np.sin(pose[3] / 2)]
    data.qvel[:6] = [0.2, -0.1, 0.3, 0.4, 0.5, -0.6]
    data.time = 0.125
    mujoco.mj_forward(model, data)
    before = {name: getattr(data, name).copy() for name in ("qpos", "qvel", "ctrl", "act", "xpos", "cvel")}
    state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
    raw_before = np.zeros(mujoco.mj_stateSize(model, state_spec))
    mujoco.mj_getState(model, data, raw_before, state_spec)
    visual.publish()
    event = publishers[0].events[0]
    assert visual.republish() is event
    assert publishers[0].events[-1] is event
    assert len(publishers[0].events) == 2
    assert not leases
    assert event["session_id"] == plan.simulation["session"]["session_id"]
    assert (event["model_generation"], event["reset_generation"]) == (3, 7)
    assert event["sequence"] == 1
    assert event["sim_time_ns"] == 125_000_000
    assert event["physics_step"] == round(data.time / plan.simulation["physics_plan"]["global_policy"]["timestep_s"])
    expected = set()
    for robot in plan.simulation["visual_plan"]["robots"]:
        projection = json.loads((ROOT / robot["projection"]["path"]).read_text())
        expected.update(f"{robot['instance_id']}/{c['local_body_id']}" for c in projection["components"])
    assert {b["stable_id"] for b in event["bodies"]} == expected
    for body in event["bodies"]:
        assert set(body) == {
            "stable_id",
            "instance_id",
            "frame_id",
            "position_m",
            "quaternion_wxyz",
            "linear_velocity_mps",
            "angular_velocity_rps",
        }
        local = body["stable_id"].split("/", 1)[1]
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, local)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0)
        assert body["frame_id"] == local
        assert body["position_m"] == pytest.approx(data.xpos[body_id])
        assert body["quaternion_wxyz"] == pytest.approx(data.xquat[body_id])
        assert body["linear_velocity_mps"] == pytest.approx(velocity[3:])
        assert body["angular_velocity_rps"] == pytest.approx(velocity[:3])
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(data, name), value)
    raw_after = np.empty_like(raw_before)
    mujoco.mj_getState(model, data, raw_after, state_spec)
    np.testing.assert_array_equal(raw_after, raw_before)
    saved = json.dumps(event)
    data.qpos[0] += 1
    data.time += 0.005
    visual.publish()
    assert json.dumps(event) == saved
    assert publishers[0].events[2]["sequence"] == 2
    assert data.time == pytest.approx(0.130)


def test_readiness_is_owned_visual_evidence_not_sensor_or_product_readiness(runtime, plan, tmp_path):
    visual, _, _, _, _, _ = runtime
    assert not visual.ready()
    visual.start()
    assert not visual.ready()
    path = tmp_path / "visual" / PRODUCT / "logs" / "visual-readiness.json"
    evidence = {
        "schema": "lingtu.sim.sensor-readiness-evidence.v1",
        "session_id": plan.simulation["session"]["session_id"],
        "model_generation": 3,
        "reset_generation": 7,
        "source_id": "robotsimue-visual",
        "basis": "truth_snapshot_applied_to_visual_bindings",
        "visual": {"state": "PREPARED"},
        "sensors": {"overall": "PREPARING"},
        "streams": [],
    }
    path.write_text(json.dumps(evidence))
    assert not visual.ready()
    evidence["visual"]["state"] = "ACTIVE"
    path.write_text(json.dumps(evidence))
    assert visual.ready()
    for field, value in (
        ("schema", "wrong"),
        ("source_id", "other"),
        ("basis", "prepared"),
        ("session_id", "other"),
        ("model_generation", 4),
        ("reset_generation", 8),
        ("model_generation", True),
        ("visual", {}),
    ):
        path.write_text(json.dumps({**evidence, field: value}))
        with pytest.raises(ValueError):
            visual.ready()
    for malformed in ("{", "[]", "null"):
        path.write_text(malformed)
        with pytest.raises(ValueError):
            visual.ready()


def test_malformed_unreal_camera_route_is_rejected_before_any_launch(plan, tmp_path):
    document = plan.as_dict()
    document["launch"]["simulation"]["sensor_plan"]["streams"]["rgb"][0]["source"] = "unreal_camera"

    def unexpected(*args, **kwargs):
        pytest.fail("unsupported source must fail before acquiring state or resources")

    visual = make_visual(
        RunPlan.from_dict(document), tmp_path, unexpected, process_factory=unexpected, publisher_factory=unexpected
    )
    with pytest.raises(ValueError, match="unreal_camera"):
        visual.start()
    visual.close()


def test_unreal_camera_mappings_belong_to_same_visual_allocation(runtime, plan):
    visual, _, _, processes, publishers, _ = runtime
    document = plan.as_dict()
    document["launch"]["simulation"]["session"]["runtime"]["mode"] = "unreal"
    sensor_plan = document["launch"]["simulation"]["sensor_plan"]
    expected = {}
    for kind in ("rgb", "depth"):
        for stream in sensor_plan["streams"][kind]:
            stream.update(owner="visual", source="unreal_camera", transport="camera_shm")
            expected[stream["sensor_id"]] = f"lingtu.sim.camera_shm.{PRODUCT}.{stream['sensor_id']}"
    visual._plan = RunPlan.from_dict(document)
    visual.start()
    assert visual.allocation is processes[0].launch["allocation"]
    assert visual.allocation.shm == expected
    snapshot = visual.publish()
    assert snapshot is publishers[0].events[-1]
    assert snapshot["model_generation"] == 3 and snapshot["reset_generation"] == 7


def test_duplicate_start_preserves_active_child_and_fresh_allocation(runtime, plan, tmp_path):
    visual, model, data, processes, publishers, _ = runtime
    visual.start()
    with pytest.raises(RuntimeError, match="already"):
        visual.start()
    assert processes[0].closed == publishers[0].closed == 0
    visual.publish()
    other = make_visual(
        plan, tmp_path, lambda: nullcontext((model, data)), process_factory=Process, publisher_factory=Publisher
    )
    assert not other.ready()
    with pytest.raises(FileExistsError):
        other.start()
    assert not other.ready()
    other.close()
    visual.close()
    visual.close()
    assert processes[0].closed == publishers[0].closed == 1


@pytest.mark.parametrize("stage", ["publisher", "process", "start", "cleanup"])
def test_partial_start_failure_closes_only_owned_resources(runtime, plan, tmp_path, stage):
    _, model, data, _, _, _ = runtime
    publishers, processes = [], []

    def publisher_factory(port):
        if stage == "publisher":
            raise RuntimeError("original start failure")
        publisher = Publisher(port)
        publishers.append(publisher)
        return publisher

    class FailingProcess(Process):
        def start(self, **kwargs):
            super().start(**kwargs)
            raise RuntimeError("original start failure")

        def terminate(self):
            super().terminate()
            if stage == "cleanup":
                raise RuntimeError("cleanup failure")

    def process_factory(*args, **kwargs):
        if stage == "process":
            raise RuntimeError("original start failure")
        process = FailingProcess(*args, **kwargs)
        processes.append(process)
        return process

    visual = make_visual(
        plan,
        tmp_path,
        lambda: nullcontext((model, data)),
        process_factory=process_factory,
        publisher_factory=publisher_factory,
    )
    with pytest.raises(RuntimeError, match="original start failure"):
        visual.start()
    visual.close()
    visual.close()
    assert all(value.closed == 1 for value in publishers + processes)
    assert not visual.ready()


def test_missing_visual_body_fails_before_launch(plan, tmp_path):
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="different"><geom size=".1"/></body></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)

    def unexpected(*args, **kwargs):
        pytest.fail("missing body must fail before launch")

    visual = make_visual(
        plan, tmp_path, lambda: nullcontext((model, data)), process_factory=unexpected, publisher_factory=unexpected
    )
    with pytest.raises(ValueError, match="missing visual body binding"):
        visual.start()
    visual.close()


def test_exited_child_prevents_reading_or_publishing_more_physics(runtime):
    visual, _, _, processes, publishers, _ = runtime
    visual.start()
    processes[0].exit_code = 1
    with pytest.raises(RuntimeError, match="exited"):
        visual.publish()
    assert not publishers[0].events


def test_real_loopback_wire_has_only_truth_identity(runtime, plan, tmp_path):
    _, model, data, _, _, _ = runtime
    visual = make_visual(plan, tmp_path, lambda: nullcontext((model, data)), process_factory=Process)
    try:
        visual.start()
        allocation = load_run_allocation(tmp_path / "visual" / PRODUCT)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", allocation.ports["visual_snapshot_udp"]))
            receiver.settimeout(2)
            visual.publish()
            wire, _ = receiver.recvfrom(60000)
        document = json.loads(wire)
        assert set(document) == {
            "schema",
            "session_id",
            "model_generation",
            "reset_generation",
            "sequence",
            "physics_step",
            "sim_time_ns",
            "bodies",
        }
        assert document["schema"] == "lingtu.sim.truth-snapshot.v1"
        assert document["session_id"] == plan.simulation["session"]["session_id"]
    finally:
        visual.close()


def test_malformed_visual_state_raises_value_error(runtime, plan, tmp_path):
    visual, _, _, _, _, _ = runtime
    visual.start()
    path = tmp_path / "visual" / PRODUCT / "logs" / "visual-readiness.json"
    evidence = dict(
        schema="lingtu.sim.sensor-readiness-evidence.v1",
        source_id="robotsimue-visual",
        basis="truth_snapshot_applied_to_visual_bindings",
        session_id=plan.simulation["session"]["session_id"],
        model_generation=3,
        reset_generation=7,
    )
    for malformed in ([], {}, None, 1, "UNKNOWN"):
        path.write_text(json.dumps({**evidence, "visual": {"state": malformed}}))
        with pytest.raises(ValueError):
            visual.ready()


@pytest.mark.parametrize("camera_override", [False, True])
def test_allocation_retains_declared_product_dds_domain(runtime, plan, tmp_path, camera_override):
    _, model, data, _, _, _ = runtime
    document = plan.as_dict()
    document["launch"]["process_environment"]["LINGTU_DDS_DOMAIN_ID"] = "41"
    for group in ("selected", "available"):
        for process in document["launch"]["process_catalog"][group]:
            if "camera" in process["provides"]:
                if camera_override:
                    process["command"]["env"]["LINGTU_DDS_DOMAIN_ID"] = "53"
                else:
                    process["command"]["env"].pop("LINGTU_DDS_DOMAIN_ID", None)
    selected = RunPlan.from_dict(document)
    visual = make_visual(
        selected, tmp_path, lambda: nullcontext((model, data)), process_factory=Process, publisher_factory=Publisher
    )
    try:
        visual.start()
        allocation = load_run_allocation(tmp_path / "visual" / PRODUCT)
        assert allocation.dds_domain == (53 if camera_override else 41)
    finally:
        visual.close()
