import io
import json
from urllib.error import HTTPError

import pytest

import nav.adapters.open_rmf.single_robot as single_robot
from nav.adapters.open_rmf import (
    ActiveFloor,
    BuildingMissionRequest,
    FloorBinding,
    GatewayClientConfig,
    GatewayHttpTransport,
    GatewayMissionPort,
    GatewayRobotStateSource,
    NativeGoalCompletionGate,
    NativeSingleFloorMissionPort,
    PoseTarget,
    RmfDestination,
    RobotSnapshot,
    SingleRobotRmfBridge,
)


class RecordingMissionPort:
    def __init__(self) -> None:
        self.requests = []

    def submit(self, request):
        self.requests.append(request)
        return True, "accepted"


class RecordingNavigationClient:
    def __init__(self) -> None:
        self.goals = []
        self.ready = (True, "autonomy_ready")

    def autonomy_ready(self):
        return self.ready

    def send_goal(self, x, y, z, yaw, *, request_id=None):
        self.goals.append((x, y, z, yaw, request_id))


class RecordingGatewayTransport:
    def __init__(self) -> None:
        self.calls = []

    def request(self, method, path, *, body=None, headers=None):
        self.calls.append((method, path, body, headers))
        return {"ok": True}


class ScriptedGatewayTransport:
    def __init__(self, responses) -> None:
        self.responses = list(responses)
        self.calls = []

    def request(self, method, path, *, body=None, headers=None):
        self.calls.append((method, path, body, headers))
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


class FailingGatewayTransport:
    def request(self, method, path, *, body=None, headers=None):
        raise OSError("gateway unavailable")


class FakeHttpResponse:
    def __init__(self, payload) -> None:
        self._payload = json.dumps(payload).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def read(self):
        return self._payload


def test_open_rmf_destination_submits_floor_aware_building_mission() -> None:
    port = RecordingMissionPort()
    bridge = SingleRobotRmfBridge(
        fleet_name="lingtu",
        robot_name="thunder-01",
        mission_port=port,
        floor_bindings={
            "L2": FloorBinding(
                building_id="factory-a",
                floor_id="floor-2",
                map_id="factory-a-floor-2",
            ),
        },
    )

    result = bridge.navigate(
        RmfDestination(map_name="L2", x=12.5, y=-3.0, yaw=1.2),
        request_id="rmf-task-42",
    )

    assert result.accepted is True
    assert result.request_id == "rmf-task-42"
    assert len(port.requests) == 1
    request = port.requests[0]
    assert request.source == "open_rmf"
    assert request.fleet_name == "lingtu"
    assert request.robot_name == "thunder-01"
    assert request.building_id == "factory-a"
    assert request.floor_id == "floor-2"
    assert request.map_id == "factory-a-floor-2"
    assert request.target.frame_id == "map"
    assert (request.target.x, request.target.y, request.target.yaw) == (12.5, -3.0, 1.2)


@pytest.mark.parametrize("malformed_ack", ["true", 1])
def test_open_rmf_bridge_rejects_truthy_non_boolean_mission_ack(malformed_ack) -> None:
    class MalformedMissionPort:
        def submit(self, request):
            return malformed_ack, "malformed"

    bridge = SingleRobotRmfBridge(
        fleet_name="lingtu",
        robot_name="thunder-01",
        mission_port=MalformedMissionPort(),
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
        },
    )

    result = bridge.navigate(
        RmfDestination(map_name="L1", x=1.0, y=2.0, yaw=0.0),
        request_id="rmf-malformed-mission-ack",
    )

    assert result.accepted is False
    assert result.reason == "malformed"


def test_single_robot_same_floor_mission_uses_native_navigation_goal() -> None:
    navigation_client = RecordingNavigationClient()
    port = NativeSingleFloorMissionPort(
        navigation_client=navigation_client,
        active_floor=ActiveFloor(
            building_id="factory-a",
            floor_id="floor-2",
            map_id="factory-a-floor-2",
        ),
    )
    request = BuildingMissionRequest(
        request_id="rmf-task-43",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=PoseTarget(frame_id="map", x=5.0, y=6.0, z=0.2, yaw=-0.5),
    )

    accepted, reason = port.submit(request)

    assert accepted is True
    assert reason == "native_navigation_goal_accepted"
    assert navigation_client.goals == [(5.0, 6.0, 0.2, -0.5, "rmf-task-43")]


def test_single_robot_mission_uses_canonical_map_frame_contract(monkeypatch) -> None:
    monkeypatch.setattr(single_robot, "map_frame_id", lambda: "contract-map")
    navigation_client = RecordingNavigationClient()
    port = NativeSingleFloorMissionPort(
        navigation_client=navigation_client,
        active_floor=ActiveFloor(
            building_id="factory-a",
            floor_id="floor-2",
            map_id="factory-a-floor-2",
        ),
    )
    request = BuildingMissionRequest(
        request_id="rmf-canonical-frame",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=PoseTarget(
            frame_id="contract-map",
            x=5.0,
            y=6.0,
            z=0.2,
            yaw=-0.5,
        ),
    )

    assert port.submit(request) == (True, "native_navigation_goal_accepted")


def test_single_robot_cross_floor_mission_fails_closed() -> None:
    navigation_client = RecordingNavigationClient()
    port = NativeSingleFloorMissionPort(
        navigation_client=navigation_client,
        active_floor=ActiveFloor(
            building_id="factory-a",
            floor_id="floor-1",
            map_id="factory-a-floor-1",
        ),
    )
    request = BuildingMissionRequest(
        request_id="rmf-cross-floor",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=PoseTarget(frame_id="map", x=5.0, y=6.0, z=0.0, yaw=0.0),
    )

    accepted, reason = port.submit(request)

    assert accepted is False
    assert reason == "floor_transition_executor_unavailable"
    assert navigation_client.goals == []


@pytest.mark.parametrize(
    "readiness",
    [
        (False, "native_control_mode_teleop"),
        (False, "native_control_mode_teleop_avoid"),
        (False, "operator_takeover_latched"),
    ],
)
def test_single_robot_mission_requires_exclusive_native_autonomy(readiness) -> None:
    navigation_client = RecordingNavigationClient()
    navigation_client.ready = readiness
    port = NativeSingleFloorMissionPort(
        navigation_client=navigation_client,
        active_floor=ActiveFloor(
            building_id="factory-a",
            floor_id="floor-1",
            map_id="factory-a-floor-1",
        ),
    )
    request = BuildingMissionRequest(
        request_id="rmf-mode-gate",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=5.0, y=6.0, z=0.0, yaw=0.0),
    )

    assert port.submit(request) == (False, f"autonomy_not_ready:{readiness[1]}")
    assert navigation_client.goals == []


@pytest.mark.parametrize("malformed_ready", ["true", 1])
def test_single_robot_mission_rejects_truthy_non_boolean_autonomy_readiness(
    malformed_ready,
) -> None:
    navigation_client = RecordingNavigationClient()
    navigation_client.ready = (malformed_ready, "malformed_readiness")
    port = NativeSingleFloorMissionPort(
        navigation_client=navigation_client,
        active_floor=ActiveFloor(
            building_id="factory-a",
            floor_id="floor-1",
            map_id="factory-a-floor-1",
        ),
    )
    request = BuildingMissionRequest(
        request_id="rmf-malformed-readiness",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=5.0, y=6.0, z=0.0, yaw=0.0),
    )

    assert port.submit(request) == (
        False,
        "autonomy_not_ready:malformed_readiness",
    )
    assert navigation_client.goals == []


def test_single_robot_state_reports_explicit_rmf_level_identity() -> None:
    bridge = SingleRobotRmfBridge(
        fleet_name="lingtu",
        robot_name="thunder-01",
        mission_port=RecordingMissionPort(),
        floor_bindings={
            "L2": FloorBinding(
                building_id="factory-a",
                floor_id="floor-2",
                map_id="factory-a-floor-2",
            ),
        },
    )

    state = bridge.report_state(
        RobotSnapshot(
            building_id="factory-a",
            floor_id="floor-2",
            map_id="factory-a-floor-2",
            x=4.0,
            y=8.0,
            z=3.6,
            yaw=0.25,
            battery_soc=0.82,
            navigation_state="EXECUTING",
            stamp_s=123.0,
        )
    )

    assert state.connected is True
    assert state.map_name == "L2"
    assert state.position == (4.0, 8.0, 0.25)
    assert state.battery_soc == 0.82
    assert state.navigation_state == "EXECUTING"
    assert state.stamp_s == 123.0


def test_navigation_success_requires_reset_then_correlated_native_goal_reached() -> None:
    gate = NativeGoalCompletionGate(
        request_id="rmf-current",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=1.0, y=2.0, z=0.0, yaw=0.0),
    )
    stale_success = RobotSnapshot(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        x=1.0,
        y=2.0,
        z=0.0,
        yaw=0.0,
        battery_soc=1.0,
        navigation_state="SUCCESS",
        stamp_s=20.0,
        native_request_id="previous-goal",
        native_command_kind="goal",
        native_command_accepted=True,
        native_control_mode="autonomy",
        native_plan_accepted=True,
        native_plan_goal=(1.0, 2.0, 0.0),
        native_goal_reached=True,
    )
    reset_observation = RobotSnapshot(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        x=1.0,
        y=2.0,
        z=0.0,
        yaw=0.0,
        battery_soc=1.0,
        navigation_state="SUCCESS",
        stamp_s=21.0,
        native_request_id="rmf-current",
        native_command_kind="goal",
        native_command_accepted=True,
        native_control_mode="autonomy",
        native_plan_accepted=True,
        native_plan_goal=(1.0, 2.0, 0.0),
        native_goal_reached=False,
    )
    correlated_success = RobotSnapshot(
        **{
            **reset_observation.__dict__,
            "stamp_s": 22.0,
            "native_goal_reached": True,
        }
    )

    assert gate.observe(stale_success) == "pending"
    assert gate.observe(reset_observation) == "executing"
    assert gate.observe(correlated_success) == "success"


@pytest.mark.parametrize("control_mode", ["", "teleop", "teleop_avoid"])
def test_navigation_completion_requires_native_autonomy_mode(
    control_mode: str,
) -> None:
    gate = NativeGoalCompletionGate(
        request_id="rmf-current",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=1.0, y=2.0, z=0.0, yaw=0.0),
    )
    snapshot = RobotSnapshot(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        x=1.0,
        y=2.0,
        z=0.0,
        yaw=0.0,
        battery_soc=1.0,
        navigation_state="SUCCESS",
        stamp_s=22.0,
        native_request_id="rmf-current",
        native_command_kind="goal",
        native_command_accepted=True,
        native_control_mode=control_mode,
        native_plan_accepted=True,
        native_plan_goal=(1.0, 2.0, 0.0),
        native_goal_reached=True,
    )

    assert gate.observe(snapshot) == "blocked"


@pytest.mark.parametrize(
    "snapshot_updates",
    [
        {"x": float("nan")},
        {"yaw": float("inf")},
        {"native_plan_goal": (float("nan"), 2.0, 0.0)},
        {"native_plan_goal": (1.0, float("inf"), 0.0)},
    ],
)
def test_navigation_completion_gate_blocks_non_finite_pose_or_native_goal(
    snapshot_updates,
) -> None:
    gate = NativeGoalCompletionGate(
        request_id="rmf-current",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=1.0, y=2.0, z=0.0, yaw=0.0),
    )
    snapshot = RobotSnapshot(
        **{
            "building_id": "factory-a",
            "floor_id": "floor-1",
            "map_id": "factory-a-floor-1",
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "yaw": 0.0,
            "battery_soc": 1.0,
            "navigation_state": "SUCCESS",
            "stamp_s": 22.0,
            "native_request_id": "rmf-current",
            "native_command_kind": "goal",
            "native_command_accepted": True,
            "native_plan_accepted": True,
            "native_control_mode": "autonomy",
            "native_plan_goal": (1.0, 2.0, 0.0),
            "native_goal_reached": True,
            **snapshot_updates,
        }
    )

    assert gate.observe(snapshot) == "blocked"


def test_gateway_mission_port_defaults_to_shadow_mode() -> None:
    transport = RecordingGatewayTransport()
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-task-shadow",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=1.0, y=2.0, z=0.0, yaw=0.0),
    )

    accepted, reason = port.submit(request)

    assert accepted is False
    assert reason == "open_rmf_shadow_mode"
    assert transport.calls == []


def test_gateway_mission_port_fails_closed_on_transport_error() -> None:
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=FailingGatewayTransport(),
    )
    request = BuildingMissionRequest(
        request_id="rmf-transport-failure",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=1.0, y=2.0, z=0.0, yaw=0.0),
    )

    assert port.submit(request) == (False, "gateway_transport_error")


def test_gateway_mission_port_acquires_lease_and_submits_same_map_goal() -> None:
    transport = ScriptedGatewayTransport(
        [
            {
                "mode": "navigating",
                "product_session": "navigating",
                "active_map": "factory-a-floor-1",
            },
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            {"ok": True, "status": "ok"},
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-task-live",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.1, yaw=0.4),
    )

    accepted, reason = port.submit(request)

    assert accepted is True
    assert reason == "gateway_navigation_goal_accepted"
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
    ]
    _, _, lease_body, lease_headers = transport.calls[1]
    assert lease_headers == {"X-API-Key": "secret"}
    assert lease_body["client_id"] == "rmf:lingtu:thunder-01"
    assert lease_body["action"] == "acquire"
    _, _, goal_body, goal_headers = transport.calls[2]
    assert goal_headers == {"X-API-Key": "secret"}
    assert goal_body["request_id"] == "rmf-task-live"
    assert goal_body["client_id"] == "rmf:lingtu:thunder-01"
    assert (goal_body["x"], goal_body["y"], goal_body["z"], goal_body["yaw"]) == (7.0, 9.0, 0.1, 0.4)
    assert goal_body["metadata"] == {
        "source": "open_rmf",
        "building_id": "factory-a",
        "floor_id": "floor-1",
        "map_id": "factory-a-floor-1",
        "map_name": "factory-a-floor-1",
    }


@pytest.mark.parametrize("malformed_ack", ["true", 1])
def test_gateway_mission_port_rejects_truthy_non_boolean_lease_ack(
    malformed_ack,
) -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": malformed_ack,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-malformed-lease-ack",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    accepted, _reason = port.submit(request)

    assert accepted is False
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
    ]


@pytest.mark.parametrize("malformed_ack", ["true", 1])
def test_gateway_mission_port_rejects_truthy_non_boolean_goal_ack(
    malformed_ack,
) -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            {"ok": malformed_ack, "status": "submitted"},
            {"ok": True, "status": "released", "holder": None, "active": False},
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-malformed-goal-ack",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    accepted, _reason = port.submit(request)

    assert accepted is False
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/lease"),
    ]


def test_gateway_mission_port_rejects_cross_floor_before_lease() -> None:
    transport = ScriptedGatewayTransport(
        [
            {
                "mode": "navigating",
                "active_map": "factory-a-floor-1",
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-cross-floor-gateway",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    assert port.submit(request) == (
        False,
        "floor_transition_executor_unavailable",
    )
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
    ]


def test_gateway_mission_port_releases_lease_when_goal_is_rejected() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            {"ok": False, "error": "navigation_not_ready"},
            {"ok": True, "status": "released", "holder": None, "active": False},
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-rejected",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    accepted, reason = port.submit(request)

    assert accepted is False
    assert reason == "navigation_not_ready"
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/lease"),
    ]
    assert transport.calls[-1][2] == {
        "action": "release",
        "client_id": "rmf:lingtu:thunder-01",
        "request_id": "rmf-rejected:lease-release",
        "ttl": 30.0,
    }


def test_gateway_mission_port_reports_cleanup_pending_when_rejected_goal_lease_release_fails() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            {"ok": False, "error": "navigation_not_ready"},
            {
                "ok": True,
                "status": "released",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-rejected-release-pending",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    accepted, reason = port.submit(request)

    assert accepted is False
    assert reason == "gateway_goal_rejected_release_pending"


def test_gateway_mission_port_cancels_then_releases_after_goal_timeout() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            TimeoutError("goal timeout"),
            {"ok": True, "status": "cancelled"},
            {"ok": True, "status": "released", "holder": None, "active": False},
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-goal-timeout",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    assert port.submit(request) == (
        False,
        "gateway_goal_outcome_unknown_cancelled",
    )
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/navigation/cancel"),
        ("POST", "/api/v1/lease"),
    ]
    assert transport.calls[-2][2] == {
        "reason": "open_rmf_goal_outcome_unknown",
        "request_id": "rmf-goal-timeout:cancel",
        "client_id": "rmf:lingtu:thunder-01",
    }


def test_gateway_mission_port_keeps_unknown_goal_pending_when_cancel_times_out() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"mode": "navigating", "active_map": "factory-a-floor-1"},
            {
                "ok": True,
                "status": "acquired",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            TimeoutError("goal timeout"),
            TimeoutError("cancel timeout"),
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )
    request = BuildingMissionRequest(
        request_id="rmf-goal-and-cancel-timeout",
        source="open_rmf",
        fleet_name="lingtu",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
        target=PoseTarget(frame_id="map", x=7.0, y=9.0, z=0.0, yaw=0.4),
    )

    assert port.submit(request) == (
        False,
        "gateway_goal_outcome_unknown_cancel_pending",
    )
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("GET", "/api/v1/session"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/navigation/cancel"),
    ]


def test_gateway_mission_port_cancel_then_releases_lease() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"ok": True, "status": "cancelled"},
            {"ok": True, "status": "released", "holder": None, "active": False},
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )

    accepted, reason = port.cancel("rmf-live", reason="rmf_stop_request")

    assert accepted is True
    assert reason == "gateway_navigation_cancelled"
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("POST", "/api/v1/navigation/cancel"),
        ("POST", "/api/v1/lease"),
    ]
    assert transport.calls[0][2] == {
        "reason": "rmf_stop_request",
        "request_id": "rmf-live:cancel",
        "client_id": "rmf:lingtu:thunder-01",
    }


@pytest.mark.parametrize("malformed_ack", ["true", 1])
def test_gateway_mission_port_rejects_truthy_non_boolean_cancel_ack(
    malformed_ack,
) -> None:
    transport = ScriptedGatewayTransport(
        [{"ok": malformed_ack, "status": "cancelled"}]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )

    cancelled, _reason = port.cancel("rmf-malformed-cancel-ack")

    assert cancelled is False
    assert [(method, path) for method, path, _, _ in transport.calls] == [
        ("POST", "/api/v1/navigation/cancel"),
    ]


def test_gateway_mission_port_cancel_requires_confirmed_lease_release() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"ok": True, "status": "cancelled"},
            {
                "ok": True,
                "status": "released",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )

    cancelled, reason = port.cancel(
        "rmf-live",
        reason="rmf_stop_request",
    )

    assert cancelled is False
    assert "lease_release_unconfirmed" in reason


def test_gateway_mission_port_renews_with_unique_request_ids() -> None:
    transport = ScriptedGatewayTransport(
        [
            {
                "ok": True,
                "status": "renewed",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
            {
                "ok": True,
                "status": "renewed",
                "holder": "rmf:lingtu:thunder-01",
                "active": True,
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )

    assert port.renew_lease("rmf-live", renewal_sequence=1)[0] is True
    assert port.renew_lease("rmf-live", renewal_sequence=2)[0] is True
    assert transport.calls[0][2]["request_id"] == "rmf-live:lease-renew:1"
    assert transport.calls[1][2]["request_id"] == "rmf-live:lease-renew:2"


@pytest.mark.parametrize("malformed_active", ["true", 1])
def test_gateway_mission_port_rejects_non_boolean_lease_state(
    malformed_active,
) -> None:
    transport = ScriptedGatewayTransport(
        [
            {
                "ok": True,
                "status": "renewed",
                "holder": "rmf:lingtu:thunder-01",
                "active": malformed_active,
            },
        ]
    )
    port = GatewayMissionPort(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            commands_enabled=True,
        ),
        transport=transport,
    )

    assert port.renew_lease("rmf-live", renewal_sequence=1) == (
        False,
        "gateway_lease_state_invalid",
    )


def test_gateway_state_source_uses_active_map_binding_for_floor_identity() -> None:
    transport = ScriptedGatewayTransport(
        [
            {
                "mode": "navigating",
                "active_map": "factory-a-floor-2",
            },
            {
                "ts": 125.0,
                "nav_endpoint": {
                    "stamp_s": 124.9,
                    "control_mode": "autonomy",
                    "control_authority": {
                        "estop_latched": False,
                        "operator_takeover_latched": False,
                    },
                    "command_boundary": {
                        "last_request_id": "rmf-task-live",
                        "last_kind": "goal",
                        "last_accepted": True,
                    },
                    "last_plan": {
                        "seen": True,
                        "accepted": True,
                        "reason": "path_found",
                        "goal": {"x": 3.0, "y": 4.0, "z": 3.5},
                        "reached_goal": True,
                    },
                    "last_local": {
                        "active": False,
                        "goal_reached": True,
                    },
                },
                "global_path": {
                    "robot": {
                        "x": 3.0,
                        "y": 4.0,
                        "z": 3.5,
                        "yaw": 0.75,
                        "frame_id": "map",
                        "ts": 124.8,
                    },
                },
            },
            {"state": "EXECUTING", "ts": 125.0},
            {
                "mode": "navigating",
                "active_map": "factory-a-floor-2",
            },
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
        ),
        transport=transport,
        floor_bindings={
            "L2": FloorBinding(
                building_id="factory-a",
                floor_id="floor-2",
                map_id="factory-a-floor-2",
            ),
        },
        clock=lambda: 125.0,
    )

    snapshot = source.snapshot()

    assert snapshot == RobotSnapshot(
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        x=3.0,
        y=4.0,
        z=3.5,
        yaw=0.75,
        battery_soc=1.0,
        navigation_state="EXECUTING",
        stamp_s=124.8,
        native_request_id="rmf-task-live",
        native_command_kind="goal",
        native_command_accepted=True,
        native_status_stamp_s=124.9,
        native_control_mode="autonomy",
        native_plan_seen=True,
        native_plan_accepted=True,
        native_plan_reason="path_found",
        native_plan_goal=(3.0, 4.0, 3.5),
        native_goal_reached=True,
    )


@pytest.mark.parametrize("malformed_evidence", ["true", 1])
def test_gateway_state_source_does_not_coerce_truthy_non_boolean_success_evidence(
    malformed_evidence,
) -> None:
    transport = ScriptedGatewayTransport(
        [
            {"active_map": "factory-a-floor-1"},
            {
                "nav_endpoint": {
                    "stamp_s": 100.0,
                    "control_mode": "autonomy",
                    "command_boundary": {
                        "last_request_id": "rmf-malformed-evidence",
                        "last_kind": "goal",
                        "last_accepted": malformed_evidence,
                    },
                    "last_plan": {
                        "seen": malformed_evidence,
                        "accepted": malformed_evidence,
                        "reason": "path_found",
                        "goal": {"x": 1.0, "y": 2.0, "z": 0.0},
                    },
                    "last_local": {
                        "active": False,
                        "goal_reached": malformed_evidence,
                    },
                },
                "global_path": {
                    "robot": {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "frame_id": "map",
                        "ts": 100.0,
                    },
                },
            },
            {"state": "EXECUTING"},
            {"active_map": "factory-a-floor-1"},
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
        ),
        transport=transport,
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
        },
        clock=lambda: 100.0,
    )

    snapshot = source.snapshot()

    assert snapshot.native_command_accepted is False
    assert snapshot.native_plan_seen is False
    assert snapshot.native_plan_accepted is False
    assert snapshot.native_goal_reached is False


@pytest.mark.parametrize(
    ("robot_x", "robot_yaw", "goal_x", "goal_y", "message"),
    [
        (float("nan"), 0.75, 3.0, 4.0, "robot pose contains non-finite values"),
        (3.0, float("inf"), 3.0, 4.0, "robot pose contains non-finite values"),
        (3.0, 0.75, float("nan"), 4.0, "native plan goal contains non-finite values"),
        (3.0, 0.75, 3.0, float("inf"), "native plan goal contains non-finite values"),
    ],
)
def test_gateway_state_source_rejects_non_finite_pose_or_native_plan_goal(
    robot_x,
    robot_yaw,
    goal_x,
    goal_y,
    message,
) -> None:
    transport = ScriptedGatewayTransport(
        [
            {"active_map": "factory-a-floor-1"},
            {
                "nav_endpoint": {
                    "stamp_s": 100.0,
                    "last_plan": {
                        "goal": {"x": goal_x, "y": goal_y, "z": 0.0},
                    },
                },
                "global_path": {
                    "robot": {
                        "x": robot_x,
                        "y": 2.0,
                        "z": 0.0,
                        "yaw": robot_yaw,
                        "frame_id": "map",
                        "ts": 100.0,
                    },
                },
            },
            {"state": "EXECUTING"},
            {"active_map": "factory-a-floor-1"},
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
        ),
        transport=transport,
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
        },
        clock=lambda: 100.0,
    )

    with pytest.raises(RuntimeError, match=message):
        source.snapshot()


def test_gateway_state_source_rejects_map_switch_during_snapshot() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"active_map": "factory-a-floor-1"},
            {
                "nav_endpoint": {"stamp_s": 100.0},
                "global_path": {
                    "robot": {
                        "x": 1.0,
                        "y": 2.0,
                        "yaw": 0.0,
                        "frame_id": "map",
                        "ts": 100.0,
                    },
                },
            },
            {"state": "EXECUTING"},
            {"active_map": "factory-a-floor-2"},
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
        ),
        transport=transport,
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
            "L2": FloorBinding(
                building_id="factory-a",
                floor_id="floor-2",
                map_id="factory-a-floor-2",
            ),
        },
        clock=lambda: 100.0,
    )

    with pytest.raises(RuntimeError, match="changed"):
        source.snapshot()


def test_gateway_state_source_rejects_stale_native_status() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"active_map": "factory-a-floor-1"},
            {
                "global_path": {
                    "robot": {
                        "x": 1.0,
                        "y": 2.0,
                        "yaw": 0.0,
                        "frame_id": "map",
                        "ts": 100.0,
                    },
                },
                "nav_endpoint": {"stamp_s": 100.0},
            },
            {"state": "IDLE"},
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            max_pose_age_s=2.0,
            max_native_status_age_s=2.0,
        ),
        transport=transport,
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
        },
        clock=lambda: 110.0,
    )

    with pytest.raises(RuntimeError, match="native navigation status is stale"):
        source.snapshot()


def test_gateway_state_source_rejects_stale_pose() -> None:
    transport = ScriptedGatewayTransport(
        [
            {"active_map": "factory-a-floor-1"},
            {
                "global_path": {
                    "robot": {
                        "x": 1.0,
                        "y": 2.0,
                        "yaw": 0.0,
                        "frame_id": "map",
                        "ts": 100.0,
                    },
                },
                "nav_endpoint": {"stamp_s": 109.5},
            },
        ]
    )
    source = GatewayRobotStateSource(
        config=GatewayClientConfig(
            base_url="http://robot:5050",
            api_key="secret",
            client_id="rmf:lingtu:thunder-01",
            max_pose_age_s=2.0,
            max_native_status_age_s=2.0,
        ),
        transport=transport,
        floor_bindings={
            "L1": FloorBinding(
                building_id="factory-a",
                floor_id="floor-1",
                map_id="factory-a-floor-1",
            ),
        },
        clock=lambda: 110.0,
    )

    with pytest.raises(RuntimeError, match="robot pose is stale"):
        source.snapshot()


def test_gateway_http_transport_sends_json_with_header_auth() -> None:
    captured = {}

    def opener(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return FakeHttpResponse({"ok": True, "status": "accepted"})

    transport = GatewayHttpTransport(
        base_url="http://robot:5050/",
        timeout_s=2.5,
        allow_insecure_http=True,
        opener=opener,
    )

    result = transport.request(
        "POST",
        "/api/v1/goal",
        body={"request_id": "rmf-1", "x": 1.0},
        headers={"X-API-Key": "secret"},
    )

    request = captured["request"]
    assert result == {"ok": True, "status": "accepted"}
    assert request.full_url == "http://robot:5050/api/v1/goal"
    assert request.get_method() == "POST"
    assert json.loads(request.data.decode("utf-8")) == {"request_id": "rmf-1", "x": 1.0}
    assert dict(request.header_items()) == {
        "Content-type": "application/json",
        "X-api-key": "secret",
    }
    assert captured["timeout"] == 2.5


def test_gateway_http_transport_rejects_unscoped_route() -> None:
    transport = GatewayHttpTransport(
        base_url="http://robot:5050/",
        allow_insecure_http=True,
        opener=lambda *_args, **_kwargs: None,
    )

    with pytest.raises(PermissionError, match="not allowed"):
        transport.request(
            "POST",
            "/api/v1/cmd_vel",
            body={"vx": 1.0, "wz": 0.0},
            headers={"X-API-Key": "secret"},
        )


def test_gateway_http_transport_decodes_http_error_json() -> None:
    def opener(request, timeout):
        raise HTTPError(
            request.full_url,
            409,
            "conflict",
            {},
            io.BytesIO(
                json.dumps(
                    {
                        "ok": False,
                        "error": "lease_conflict",
                    }
                ).encode()
            ),
        )

    transport = GatewayHttpTransport(
        base_url="http://robot:5050/",
        allow_insecure_http=True,
        opener=opener,
    )

    result = transport.request(
        "POST",
        "/api/v1/lease",
        body={"action": "acquire", "client_id": "rmf"},
    )

    assert result == {
        "ok": False,
        "error": "lease_conflict",
        "http_status": 409,
    }
