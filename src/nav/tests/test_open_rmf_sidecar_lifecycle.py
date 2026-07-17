import importlib
import sys
from pathlib import Path

from external_adapters.open_rmf import (
    FloorBinding,
    GatewayClientConfig,
    RobotSnapshot,
    SingleRobotRmfBridge,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "integrations" / "open_rmf" / "ros_ws" / "src" / "lingtu_rmf_adapter"


def _adapter_module():
    package_path = str(PACKAGE_ROOT)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    return importlib.import_module("lingtu_rmf_adapter.adapter")


class FakeLogger:
    def info(self, message):
        return None

    def warning(self, message):
        return None

    def error(self, message):
        return None


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeRmfEasy:
    class RobotCallbacks:
        def __init__(self, navigate, stop, action):
            self.navigate = navigate
            self.stop = stop
            self.action = action

    class RobotState:
        def __init__(self, map_name, position, battery_soc):
            self.map_name = map_name
            self.position = position
            self.battery_soc = battery_soc


class FakeUpdateHandle:
    def __init__(self):
        self.updates = []

    def update(self, state, activity):
        self.updates.append((state, activity))


class FakeFleetHandle:
    def __init__(self):
        self.update_handle = FakeUpdateHandle()
        self.reassign_count = 0

    def add_robot(self, name, state, configuration, callbacks):
        return self.update_handle

    def more(self):
        return self

    def reassign_dispatched_tasks(self):
        self.reassign_count += 1


class FakeMissionPort:
    def __init__(
        self,
        *,
        submit_result=(True, "accepted"),
        release_results=None,
        cancel_results=None,
        renew_result=(True, "renewed"),
    ):
        self.submit_result = submit_result
        self.release_results = list(release_results or [(True, "released")])
        self.cancel_results = list(cancel_results or [(True, "cancelled")])
        self.renew_result = renew_result
        self.submissions = []
        self.release_calls = []
        self.cancel_calls = []
        self.renew_calls = []

    def submit(self, request):
        self.submissions.append(request)
        return self.submit_result

    def release_lease(self, request_id):
        self.release_calls.append(request_id)
        if len(self.release_results) > 1:
            return self.release_results.pop(0)
        return self.release_results[0]

    def cancel(self, request_id, *, reason):
        self.cancel_calls.append((request_id, reason))
        if len(self.cancel_results) > 1:
            return self.cancel_results.pop(0)
        return self.cancel_results[0]

    def renew_lease(self, request_id, *, renewal_sequence):
        self.renew_calls.append((request_id, renewal_sequence))
        return self.renew_result


class FakeStateSource:
    def __init__(self, snapshot):
        self.current = snapshot

    def snapshot(self):
        return self.current


class FakeIdentifier:
    def is_same(self, other):
        return other is self


class FakeExecution:
    def __init__(self):
        self.identifier = FakeIdentifier()
        self.okay = True
        self.finished_count = 0

    def finished(self):
        self.finished_count += 1


class FakeDestination:
    def __init__(self, map_name="L1", position=(1.0, 2.0, 0.0)):
        self.map = map_name
        self.position = position


def _snapshot(
    *,
    request_id="",
    goal_reached=False,
    plan_accepted=False,
):
    return RobotSnapshot(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        x=1.0,
        y=2.0,
        z=0.0,
        yaw=0.0,
        battery_soc=1.0,
        navigation_state="IDLE",
        stamp_s=100.0,
        native_request_id=request_id,
        native_command_kind="goal" if request_id else "",
        native_command_accepted=bool(request_id),
        native_status_stamp_s=100.0,
        native_control_mode="autonomy",
        native_plan_seen=bool(request_id),
        native_plan_accepted=plan_accepted,
        native_plan_reason="path_found" if plan_accepted else "planning",
        native_plan_goal=(1.0, 2.0, 0.0) if request_id else None,
        native_goal_reached=goal_reached,
    )


def _controller(mission_port):
    adapter = _adapter_module()
    binding = FloorBinding(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
    )
    second_floor = FloorBinding(
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
    )
    floor_bindings = {
        "L1": binding,
        "L2": second_floor,
    }
    settings = adapter.BridgeSettings(
        fleet_name="lingtu",
        robot_name="thunder-01",
        gateway=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
            allow_insecure_http=True,
        ),
        floor_bindings=floor_bindings,
        poll_period_s=0.5,
        goal_tolerance_m=0.1,
    )
    fleet = FakeFleetHandle()
    controller = adapter.LingTuRobotAdapter(
        node=FakeNode(),
        rmf_easy=FakeRmfEasy,
        fleet_handle=fleet,
        robot_configuration=object(),
        settings=settings,
    )
    controller._mission_port = mission_port
    controller._bridge = SingleRobotRmfBridge(
        fleet_name="lingtu",
        robot_name="thunder-01",
        mission_port=mission_port,
        floor_bindings=floor_bindings,
    )
    controller._state_source = FakeStateSource(_snapshot())
    controller.update()
    return controller, fleet


def test_sidecar_finishes_once_only_after_confirmed_release() -> None:
    mission = FakeMissionPort(
        release_results=[
            (False, "gateway_lease_release_unconfirmed"),
            (True, "gateway_lease_released"),
        ]
    )
    controller, _fleet = _controller(mission)
    execution = FakeExecution()
    controller.navigate(FakeDestination(), execution)
    request_id = controller._request_id

    controller._state_source.current = _snapshot(
        request_id=request_id,
        goal_reached=False,
        plan_accepted=True,
    )
    controller.update()
    controller._state_source.current = _snapshot(
        request_id=request_id,
        goal_reached=True,
        plan_accepted=True,
    )
    controller.update()

    assert execution.finished_count == 0
    assert controller._request_id == request_id

    controller.update()

    assert execution.finished_count == 1
    assert controller._request_id == ""
    assert mission.release_calls == [request_id, request_id]


def test_sidecar_retains_active_command_when_cancel_fails() -> None:
    mission = FakeMissionPort(
        cancel_results=[
            (False, "gateway_transport_error"),
            (True, "gateway_navigation_cancelled"),
        ]
    )
    controller, _fleet = _controller(mission)
    execution = FakeExecution()
    controller.navigate(FakeDestination(), execution)
    request_id = controller._request_id

    controller.stop(execution.identifier)

    assert controller._request_id == request_id
    assert controller._execution is execution

    controller.stop(execution.identifier)

    assert controller._request_id == ""
    assert controller._execution is None
    assert len(mission.cancel_calls) == 2


def test_sidecar_retries_unknown_goal_cancel_without_entering_completion_gate() -> None:
    mission = FakeMissionPort(
        submit_result=(
            False,
            "gateway_goal_outcome_unknown_cancel_pending",
        ),
        cancel_results=[
            (False, "gateway_transport_error"),
            (True, "gateway_navigation_cancelled"),
        ],
    )
    controller, fleet = _controller(mission)
    execution = FakeExecution()

    controller.navigate(FakeDestination(), execution)
    request_id = controller._request_id

    assert request_id
    assert controller._execution is execution
    assert controller._completion_gate is None
    assert fleet.reassign_count == 1

    controller._state_source.current = _snapshot(
        request_id=request_id,
        goal_reached=True,
        plan_accepted=True,
    )
    controller.update()

    assert execution.finished_count == 0
    assert controller._request_id == request_id
    assert controller._execution is execution
    assert mission.cancel_calls == [
        (request_id, "open_rmf_goal_outcome_unknown"),
    ]

    controller.update()

    assert execution.finished_count == 0
    assert controller._request_id == ""
    assert controller._execution is None
    assert mission.cancel_calls == [
        (request_id, "open_rmf_goal_outcome_unknown"),
        (request_id, "open_rmf_goal_outcome_unknown"),
    ]


def test_sidecar_retries_rejected_goal_release_before_reassigning() -> None:
    mission = FakeMissionPort(
        submit_result=(False, "gateway_goal_rejected_release_pending"),
        release_results=[
            (False, "gateway_lease_release_unconfirmed"),
            (True, "gateway_lease_released"),
        ],
    )
    controller, fleet = _controller(mission)
    execution = FakeExecution()

    controller.navigate(FakeDestination(), execution)
    request_id = controller._request_id

    assert request_id
    assert controller._execution is execution
    assert execution.finished_count == 0
    assert fleet.reassign_count == 0

    controller.update()

    assert controller._request_id == request_id
    assert controller._execution is execution
    assert execution.finished_count == 0
    assert mission.cancel_calls == []
    assert mission.release_calls == [request_id]
    assert fleet.reassign_count == 0

    controller.update()

    assert controller._request_id == ""
    assert controller._execution is None
    assert execution.finished_count == 0
    assert mission.cancel_calls == []
    assert mission.release_calls == [request_id, request_id]
    assert fleet.reassign_count == 1


def test_sidecar_blocks_replacement_while_rejected_goal_release_is_pending() -> None:
    mission = FakeMissionPort(
        submit_result=(False, "gateway_goal_rejected_release_pending"),
        release_results=[
            (False, "gateway_lease_release_unconfirmed"),
            (True, "gateway_lease_released"),
        ],
    )
    controller, fleet = _controller(mission)
    original_execution = FakeExecution()
    replacement_execution = FakeExecution()

    controller.navigate(FakeDestination(), original_execution)
    original_request_id = controller._request_id
    controller.navigate(FakeDestination(), replacement_execution)

    assert controller._request_id == original_request_id
    assert controller._execution is original_execution
    assert len(mission.submissions) == 1
    assert mission.cancel_calls == []
    assert mission.release_calls == [original_request_id]
    assert fleet.reassign_count == 0

    controller.update()

    assert controller._request_id == ""
    assert controller._execution is None
    assert original_execution.finished_count == 0
    assert replacement_execution.finished_count == 0
    assert mission.cancel_calls == []
    assert mission.release_calls == [original_request_id, original_request_id]
    assert fleet.reassign_count == 1


def test_sidecar_cancels_and_reassigns_when_lease_renewal_is_lost() -> None:
    mission = FakeMissionPort(
        renew_result=(False, "gateway_lease_not_owned"),
    )
    controller, fleet = _controller(mission)
    execution = FakeExecution()
    controller.navigate(FakeDestination(), execution)
    request_id = controller._request_id
    controller._last_renew_monotonic = 0.0
    controller._state_source.current = _snapshot(
        request_id=request_id,
        goal_reached=False,
        plan_accepted=True,
    )

    controller.update()

    assert controller._request_id == ""
    assert execution.finished_count == 0
    assert mission.renew_calls == [(request_id, 1)]
    assert mission.cancel_calls == [(request_id, "rmf_lease_lost")]
    assert fleet.reassign_count == 1


def test_sidecar_cleans_rejected_cross_floor_execution() -> None:
    mission = FakeMissionPort(
        submit_result=(False, "floor_transition_executor_unavailable"),
    )
    controller, fleet = _controller(mission)
    execution = FakeExecution()

    controller.navigate(
        FakeDestination(map_name="L2"),
        execution,
    )

    assert controller._execution is None
    assert controller._request_id == ""
    assert execution.finished_count == 0
    assert fleet.reassign_count == 1
