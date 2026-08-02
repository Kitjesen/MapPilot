from __future__ import annotations

from types import SimpleNamespace

import pytest

from maps.modules import elevation, occupancy
from runtime.msgs.geometry import Pose, Quaternion, Transform, Vector3
from runtime.msgs.map import MapObservationFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np


def _observation(*, epoch: int = 7, sequence: int = 3) -> MapObservationFrame:
    transform = Transform(
        translation=Vector3(10.0, 20.0, 1.0),
        rotation=Quaternion.from_yaw(0.5),
        frame_id="map",
        child_frame_id="body",
        ts=12.5,
    )
    return MapObservationFrame(
        points=np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
        reset_epoch=epoch,
        sequence=sequence,
        ts=12.5,
        frame_id="map",
        sensor_frame_id="body",
        sensor_origin=transform.translation,
        map_sensor_pose=Pose(transform.translation, transform.rotation),
        map_sensor_transform=transform,
    )


def test_occupancy_uses_observation_pose_not_latest_odometry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[dict] = []

    class Runtime:
        def build_occupancy_grid(self, points, **kwargs):
            calls.append({"points": points.copy(), **kwargs})
            return {
                "occupancy": np.asarray([[100]], dtype=np.int8),
                "cost": np.asarray([[1.0]], dtype=np.float32),
                "origin": np.asarray([9.0, 19.0], dtype=np.float64),
                "counts": {"occupied": 1},
            }

    monkeypatch.setattr(
        occupancy,
        "create_map_kernel_backend",
        lambda: SimpleNamespace(runtime=Runtime()),
    )
    module = occupancy.OccupancyGridModule()
    module.setup()
    module._on_odom(
        Odometry(
            pose=Pose(
                Vector3(-100.0, -200.0, 0.0),
                Quaternion.from_yaw(-1.0),
            )
        )
    )

    module._on_observation(_observation())
    module._on_observation(_observation())
    module._on_observation(_observation(epoch=6, sequence=99))

    assert len(calls) == 1
    assert calls[0]["robot_x"] == pytest.approx(10.0)
    assert calls[0]["robot_y"] == pytest.approx(20.0)
    assert calls[0]["robot_yaw"] == pytest.approx(0.5)
    assert calls[0]["points"][0].tolist() == pytest.approx(
        [10.8775826, 20.4794255, 1.0]
    )


def test_elevation_uses_observation_origin_and_rejects_duplicate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[float, float]] = []
    outputs: list[dict] = []

    class Runtime:
        def build_elevation_map(
            self,
            _points,
            robot_x,
            robot_y,
            _resolution,
            _radius,
            _z_floor,
            _z_ceil,
        ):
            calls.append((robot_x, robot_y))
            return object()

    monkeypatch.setattr(
        elevation,
        "create_map_kernel_backend",
        lambda: SimpleNamespace(runtime=Runtime()),
    )
    monkeypatch.setattr(
        elevation,
        "elevation_result_to_payload",
        lambda _result, *, ts, frame_id: {"ts": ts, "frame_id": frame_id},
    )
    module = elevation.ElevationMapModule()
    module.elevation_map._add_callback(outputs.append)
    module.setup()
    module._on_odom(Odometry(pose=Pose(Vector3(-100.0, -200.0, 0.0))))

    module._on_observation(_observation())
    module._on_observation(_observation())

    assert calls == [(10.0, 20.0)]
    assert outputs == [{"ts": 12.5, "frame_id": "map"}]
