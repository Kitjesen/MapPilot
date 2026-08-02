from __future__ import annotations

import json

import pytest

from lingtu import Robot


class _FakeNav:
    def __init__(self) -> None:
        self.calls: list[tuple[tuple, dict]] = []
        self.reloads: list[str] = []
        self.instructions: list[str] = []

    def navigate_to(self, *args, **kwargs) -> str:
        self.calls.append((args, kwargs))
        return "ok"

    def send_instruction(self, instruction: str) -> str:
        self.instructions.append(instruction)
        return json.dumps({"status": "sent", "instruction": instruction})

    def get_navigation_status(self) -> str:
        return json.dumps(
            {
                "state": "EXECUTING",
                "position": {"x": 1.0, "y": 2.0, "z": 0.4, "yaw": 0.1},
            }
        )

    def reload_planner_tomogram(self, tomogram: str) -> dict:
        self.reloads.append(tomogram)
        return {"ok": True}


class _FailingMapManager:
    def save_map(self, name: str) -> str:
        return json.dumps({"success": False, "message": f"failed: {name}"})


class _MapAckManager:
    def __init__(self, result) -> None:
        self.result = result

    def save_map(self, name: str):
        del name
        return self.result


class _FakeSemanticPlanner:
    def get_scene_objects(self) -> str:
        return json.dumps(
            [
                {
                    "label": "chair",
                    "confidence": 0.8,
                    "position": [1.0, 2.0, 0.3],
                }
            ]
        )


class _FakeSafety:
    def __init__(self) -> None:
        self.calls = 0

    def emergency_stop(self) -> str:
        self.calls += 1
        return "estop"


class _FakeSystem:
    def __init__(self, nav: _FakeNav, map_manager=None) -> None:
        self.modules = {"nav.mission": nav}
        if map_manager is not None:
            self.modules["maps.service"] = map_manager
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def get_module(self, name: str):
        try:
            return self.modules[name]
        except KeyError:
            raise KeyError(name) from None


def test_robot_go_to_passes_explicit_z_as_keyword() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    robot._system = _FakeSystem(nav)

    assert robot.go_to(1.0, 2.0, yaw=0.3, z=0.4) == "ok"
    assert nav.calls == [((1.0, 2.0, 0.3), {"z": 0.4})]


def test_robot_go_to_3d_is_explicit_xyz_facade() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    robot._system = _FakeSystem(nav)

    assert robot.go_to_3d(1.0, 2.0, 0.4, yaw=0.3) == "ok"
    assert nav.calls == [((1.0, 2.0, 0.3), {"z": 0.4})]


def test_robot_go_to_keeps_old_2d_call_compatible() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    robot._system = _FakeSystem(nav)

    assert robot.go_to(1.0, 2.0) == "ok"
    assert nav.calls == [((1.0, 2.0, 0.0), {})]


def test_robot_navigation_commands_prefer_nav_skills() -> None:
    mission = _FakeNav()
    skills = _FakeNav()
    robot = Robot("nav")
    system = _FakeSystem(mission)
    system.modules["nav.skills"] = skills
    robot._system = system

    assert robot.go_to(1.0, 2.0) == "ok"
    assert robot.status() == "EXECUTING"
    assert skills.calls == [((1.0, 2.0, 0.0), {})]
    assert mission.calls == []


def test_robot_stop_motion_prefers_safety_estop() -> None:
    nav = _FakeNav()
    safety = _FakeSafety()
    robot = Robot("nav")
    system = _FakeSystem(nav)
    system.modules["nav.safety"] = safety
    robot._system = system

    assert robot.stop_motion() == "estop"
    assert safety.calls == 1


def test_robot_status_and_pose_use_navigation_status() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    robot._system = _FakeSystem(nav)

    assert robot.status() == "EXECUTING"
    assert robot.get_pose() == (1.0, 2.0, 0.4)


def test_robot_detect_uses_semantic_planner_skill() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    system = _FakeSystem(nav)
    system.modules["SemanticPlannerModule"] = _FakeSemanticPlanner()
    robot._system = system

    assert robot.detect() == [{"label": "chair", "confidence": 0.8, "position": [1.0, 2.0, 0.3]}]


def test_robot_map_skill_honors_success_false() -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    system = _FakeSystem(nav)
    system.modules["maps.service"] = _FailingMapManager()
    robot._system = system

    assert robot.save_map("bad") is False


@pytest.mark.parametrize(
    "result",
    [None, 1, "queued", "{}", '{"success": "true"}', '{"ok": 1}', '{"message": "done"}'],
)
def test_robot_map_skill_rejects_malformed_ack(result) -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    system = _FakeSystem(nav)
    system.modules["maps.service"] = _MapAckManager(result)
    robot._system = system

    assert robot.save_map("map") is False


@pytest.mark.parametrize(
    ("result", "expected"),
    [
        (True, True),
        (False, False),
        ({"success": True}, True),
        ({"success": False}, False),
        ('{"ok": true}', True),
        ('{"ok": false}', False),
        ('{"ok": true, "success": false}', False),
    ],
)
def test_robot_map_skill_requires_explicit_consistent_boolean_ack(result, expected: bool) -> None:
    nav = _FakeNav()
    robot = Robot("nav")
    system = _FakeSystem(nav)
    system.modules["maps.service"] = _MapAckManager(result)
    robot._system = system

    assert robot.save_map("map") is expected
