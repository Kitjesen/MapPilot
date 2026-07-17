from __future__ import annotations

import sys
import threading
from pathlib import Path

# ruff: noqa: S101
import pytest

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco.lift import MuJoCoLiftCommandAdapter, MuJoCoLiftConfig

mujoco = pytest.importorskip("mujoco")


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, dt_s: float) -> None:
        self.now += dt_s


class EngineWrapper:
    def __init__(self) -> None:
        xml = """
        <mujoco model="lift_test">
          <option timestep="0.01"/>
          <worldbody>
            <body name="lift_cabin" pos="0 0 0">
              <joint name="lift_cabin_z" type="slide" axis="0 0 1" limited="true" range="0 4"/>
              <geom type="box" size="0.5 0.5 0.05"/>
            </body>
            <body name="lift_door_left" pos="0 0 0">
              <joint name="lift_door_left_slide" type="slide" axis="1 0 0" limited="true" range="-1 0"/>
              <geom type="box" size="0.05 0.5 0.5"/>
            </body>
            <body name="lift_door_right" pos="0 0 0">
              <joint name="lift_door_right_slide" type="slide" axis="1 0 0" limited="true" range="0 1"/>
              <geom type="box" size="0.05 0.5 0.5"/>
            </body>
          </worldbody>
        </mujoco>
        """
        self._model = mujoco.MjModel.from_xml_string(xml)
        self._data = mujoco.MjData(self._model)
        self._data_lock = threading.RLock()
        mujoco.mj_forward(self._model, self._data)


def joint_qpos(engine: EngineWrapper, name: str) -> float:
    joint_id = mujoco.mj_name2id(engine._model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return float(engine._data.qpos[int(engine._model.jnt_qposadr[joint_id])])


def make_adapter() -> tuple[MuJoCoLiftCommandAdapter, EngineWrapper, FakeClock]:
    engine = EngineWrapper()
    clock = FakeClock()
    adapter = MuJoCoLiftCommandAdapter(engine, clock=clock)
    return adapter, engine, clock


def test_claims_one_session_and_rejects_other_sessions_or_unknown_ids() -> None:
    adapter, _engine, _clock = make_adapter()

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-1", session_id="session-a") == (
        True,
        "lift_request_accepted",
    )
    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-2", session_id="session-b") == (
        False,
        "lift_session_busy",
    )
    assert adapter.request(lift_id="bad-lift", destination_floor_id="floor-1", session_id="session-a") == (
        False,
        "unknown_lift_id",
    )
    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-9", session_id="session-a") == (
        False,
        "unknown_floor_id",
    )
    assert adapter.snapshot("bad-lift") is None


def test_closes_doors_travels_to_exact_floor_and_opens_doors() -> None:
    adapter, engine, clock = make_adapter()

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-1", session_id="session-a")[0]
    adapter.step(2.0)
    assert adapter.snapshot("lift-a").door_state == "open"

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-2", session_id="session-a")[0]
    adapter.step(0.25)
    state = adapter.snapshot("lift-a")
    assert state.door_state == "closing"
    assert state.motion_state == "stopped"

    adapter.step(2.0)
    state = adapter.snapshot("lift-a")
    assert state.door_state == "closed"
    assert state.motion_state == "moving"
    assert joint_qpos(engine, "lift_door_left_slide") == pytest.approx(0.0)
    assert joint_qpos(engine, "lift_door_right_slide") == pytest.approx(0.0)

    for _ in range(10):
        clock.advance(0.5)
        adapter.step(0.5)

    state = adapter.snapshot("lift-a")
    assert state.current_floor_id == "floor-2"
    assert state.destination_floor_id == "floor-2"
    assert state.motion_state == "stopped"
    assert state.door_state == "open"
    assert state.session_id == "session-a"
    assert state.stamp_s == pytest.approx(clock.now)
    assert joint_qpos(engine, "lift_cabin_z") == pytest.approx(3.5)
    assert joint_qpos(engine, "lift_door_left_slide") == pytest.approx(-0.65)
    assert joint_qpos(engine, "lift_door_right_slide") == pytest.approx(0.65)


def test_rejects_release_while_cabin_is_moving_then_releases_after_arrival() -> None:
    adapter, _engine, _clock = make_adapter()

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-2", session_id="session-a")[0]
    adapter.step(2.0)
    assert adapter.snapshot("lift-a").motion_state == "moving"
    assert adapter.release(lift_id="lift-a", session_id="session-a") == (False, "lift_moving")

    adapter.step(10.0)
    assert adapter.snapshot("lift-a").motion_state == "stopped"
    assert adapter.release(lift_id="lift-a", session_id="session-a") == (True, "lift_released")
    assert adapter.snapshot("lift-a").session_id == ""


@pytest.mark.parametrize("phase", ["opening", "closing"])
def test_rejects_release_while_door_transition_is_in_progress(phase: str) -> None:
    adapter, _engine, _clock = make_adapter()

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-1", session_id="session-a")[0]
    if phase == "closing":
        adapter.step(2.0)
        assert adapter.snapshot("lift-a").door_state == "open"
        assert adapter.request(lift_id="lift-a", destination_floor_id="floor-2", session_id="session-a")[0]

    state_before = adapter.snapshot("lift-a")
    assert state_before.door_state == phase
    assert state_before.motion_state == "stopped"
    assert adapter.release(lift_id="lift-a", session_id="session-a") == (
        False,
        "lift_transition_in_progress",
    )

    adapter.step(10.0)
    state_after = adapter.snapshot("lift-a")
    assert state_after.session_id == "session-a"


def test_availability_fault_rejects_requests_and_marks_snapshot_unavailable() -> None:
    adapter, _engine, _clock = make_adapter()

    adapter.set_available(False)

    assert adapter.request(lift_id="lift-a", destination_floor_id="floor-1", session_id="session-a") == (
        False,
        "lift_unavailable",
    )
    state = adapter.snapshot("lift-a")
    assert state.available is False
    assert state.motion_state == "stopped"


def test_config_fields_are_overrideable() -> None:
    engine = EngineWrapper()
    config = MuJoCoLiftConfig(
        lift_id="custom-lift",
        floors=(("ground", 0.0), ("roof", 1.0)),
        cabin_speed_mps=2.0,
        door_speed_mps=2.0,
    )
    adapter = MuJoCoLiftCommandAdapter(engine, config=config)

    assert adapter.request(lift_id="custom-lift", destination_floor_id="roof", session_id="session-a")[0]
    adapter.step(3.0)
    state = adapter.snapshot("custom-lift")
    assert state.current_floor_id == "roof"
    assert joint_qpos(engine, "lift_cabin_z") == pytest.approx(1.0)
