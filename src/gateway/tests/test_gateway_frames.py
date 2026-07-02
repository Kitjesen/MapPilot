from __future__ import annotations

import pytest

pytest.importorskip("fastapi")

from gateway.gateway_module import GatewayModule
from gateway.routes.diagnostics import _frame_contract_snapshot
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.runtime_interface import TOPICS, topic_default_frame_id


def test_gateway_map_odom_tf_updates_frame_tree() -> None:
    gateway = GatewayModule(manage_session_services=False)

    gateway._on_map_odom_tf({
        "tx": 1.0,
        "ty": 2.0,
        "tz": 3.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
        "ts": 123.0,
        "valid": True,
    })

    transform = gateway._frame_tree.lookup(
        topic_default_frame_id(TOPICS.map_cloud),
        topic_default_frame_id(TOPICS.odometry),
        ts=123.0,
    )
    snapshot = _frame_contract_snapshot(gateway)
    edges = {
        (edge["parent"], edge["child"]): edge
        for edge in snapshot["frame_tree"]["edges"]
    }

    assert gateway._has_map_odom_tf
    assert gateway._T_map_odom[0, 3] == pytest.approx(1.0)
    assert transform.translation.x == pytest.approx(1.0)
    assert transform.translation.y == pytest.approx(2.0)
    assert transform.translation.z == pytest.approx(3.0)
    assert edges[("map", "odom")]["latest_ts"] == pytest.approx(123.0)


def test_gateway_odometry_updates_frame_tree_for_diagnostics() -> None:
    gateway = GatewayModule(manage_session_services=False)

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(1.0, 2.0, 0.0),
                orientation=Quaternion(),
            ),
            ts=124.0,
            frame_id="odom",
            child_frame_id="body",
        )
    )

    transform = gateway._frame_tree.lookup("odom", "body", ts=124.0)

    assert transform.translation.x == pytest.approx(1.0)
    assert transform.translation.y == pytest.approx(2.0)
