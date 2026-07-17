from __future__ import annotations

from dataclasses import dataclass

from nav.building import (
    CorrelatedNativeNavigationPort,
    GoalProgress,
    PoseTarget,
)


class FakeNavigationClient:
    def __init__(self) -> None:
        self.goals = []
        self.cancels = []
        self.stops = []

    def send_goal(self, x, y, z, yaw, *, request_id=None):
        self.goals.append((x, y, z, yaw, request_id))

    def cancel(self, reason="cancel", *, request_id=None):
        self.cancels.append((reason, request_id))

    def stop(self, reason="stop", *, request_id=None):
        self.stops.append((reason, request_id))


@dataclass
class Snapshot:
    map_id: str = "factory-a-floor-1"
    x: float = 1.0
    y: float = 2.0
    z: float = 0.0
    yaw: float = 0.0
    stamp_s: float = 100.0
    native_request_id: str = ""
    native_command_kind: str = ""
    native_command_accepted: bool = False
    native_status_stamp_s: float = 100.0
    native_control_mode: str = "autonomy"
    native_estop_latched: bool = False
    native_operator_takeover_latched: bool = False
    native_plan_seen: bool = False
    native_plan_accepted: bool = False
    native_plan_reason: str = ""
    native_plan_goal: tuple[float, float, float] | None = None
    native_goal_reached: bool = False


def test_correlated_native_navigation_requires_reset_before_goal_success() -> None:
    client = FakeNavigationClient()
    snapshot = Snapshot()
    port = CorrelatedNativeNavigationPort(
        client=client,
        snapshot=lambda: snapshot,
        clock=lambda: 100.1,
    )
    target = PoseTarget("map", 1.0, 2.0, 0.0, 0.0)
    port.send_goal(1.0, 2.0, 0.0, 0.0, request_id="building-1:goal")
    snapshot.native_request_id = "building-1:goal"
    snapshot.native_command_kind = "goal"
    snapshot.native_command_accepted = True
    snapshot.native_plan_seen = True
    snapshot.native_plan_accepted = True
    snapshot.native_plan_goal = (1.0, 2.0, 0.0)
    snapshot.native_goal_reached = True

    assert port.observe_goal(
        request_id="building-1:goal",
        map_id="factory-a-floor-1",
        target=target,
    ) is GoalProgress.PENDING

    snapshot.native_goal_reached = False
    assert port.observe_goal(
        request_id="building-1:goal",
        map_id="factory-a-floor-1",
        target=target,
    ) is GoalProgress.EXECUTING
    snapshot.native_goal_reached = True
    assert port.observe_goal(
        request_id="building-1:goal",
        map_id="factory-a-floor-1",
        target=target,
    ) is GoalProgress.SUCCEEDED


def test_correlated_native_navigation_fails_readiness_on_mode_or_stale_status() -> None:
    snapshot = Snapshot(native_control_mode="teleop_avoid")
    port = CorrelatedNativeNavigationPort(
        client=FakeNavigationClient(),
        snapshot=lambda: snapshot,
        clock=lambda: 100.1,
        max_native_status_age_s=1.0,
    )

    assert port.autonomy_ready() == (False, "native_control_mode_teleop_avoid")
    snapshot.native_control_mode = "autonomy"
    snapshot.native_status_stamp_s = 90.0
    assert port.autonomy_ready() == (False, "native_navigation_status_stale")


def test_correlated_native_navigation_blocks_mismatched_target_or_map() -> None:
    snapshot = Snapshot()
    port = CorrelatedNativeNavigationPort(
        client=FakeNavigationClient(),
        snapshot=lambda: snapshot,
        clock=lambda: 100.1,
    )
    port.send_goal(1.0, 2.0, 0.0, 0.0, request_id="building-1:goal")

    assert port.observe_goal(
        request_id="building-1:goal",
        map_id="other-map",
        target=PoseTarget("map", 1.0, 2.0, 0.0, 0.0),
    ) is GoalProgress.BLOCKED
    assert port.observe_goal(
        request_id="building-1:goal",
        map_id="factory-a-floor-1",
        target=PoseTarget("map", 9.0, 2.0, 0.0, 0.0),
    ) is GoalProgress.BLOCKED
