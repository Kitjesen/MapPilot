"""src/runtime/msgs nav + semantic 消息类型单元测试。"""

from __future__ import annotations

import numpy as np
import pytest

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import OccupancyGrid, Odometry, Path
from runtime.msgs.semantic import (
    Detection3D,
    GoalResult,
    NavigationCommand,
    Region,
    Relation,
    SceneGraph,
)

# ===== Odometry =====

class TestOdometry:
    def test_convenience_properties(self):
        odom = Odometry(
            pose=Pose(Vector3(1.0, 2.0, 3.0), Quaternion()),
            twist=Twist(Vector3(0.5, 0.1, 0.0), Vector3(0.0, 0.0, 0.3)),
        )
        assert odom.x == pytest.approx(1.0)
        assert odom.y == pytest.approx(2.0)
        assert odom.z == pytest.approx(3.0)
        assert odom.vx == pytest.approx(0.5)
        assert odom.vy == pytest.approx(0.1)
        assert odom.wz == pytest.approx(0.3)

    def test_encode_decode_roundtrip(self):
        odom = Odometry(
            pose=Pose(Vector3(1.5, -2.3, 0.1), Quaternion(0, 0, 0.383, 0.924)),
            twist=Twist(Vector3(0.8, 0, 0), Vector3(0, 0, -0.2)),
            ts=1234567890.123,
            frame_id="odom",
            child_frame_id="body",
        )
        data = odom.encode()
        restored = Odometry.decode(data)
        assert restored.x == pytest.approx(odom.x)
        assert restored.vx == pytest.approx(odom.vx)
        assert restored.frame_id == "odom"
        assert restored.child_frame_id == "body"

    def test_to_from_dict(self):
        odom = Odometry(
            pose=Pose(Vector3(5, 6, 7), Quaternion()),
            twist=Twist(Vector3(1, 0, 0), Vector3(0, 0, 0.5)),
            ts=100.0,
        )
        d = odom.to_dict()
        restored = Odometry.from_dict(d)
        assert restored.x == pytest.approx(5.0)
        assert restored.wz == pytest.approx(0.5)


# ===== Path =====

class TestPath:
    def _make_path(self):
        poses = [
            PoseStamped(pose=Pose(Vector3(0, 0, 0)), ts=1.0),
            PoseStamped(pose=Pose(Vector3(3, 0, 0)), ts=2.0),
            PoseStamped(pose=Pose(Vector3(3, 4, 0)), ts=3.0),
        ]
        return Path(poses=poses, ts=1.0, frame_id="map")

    def test_total_length(self):
        path = self._make_path()
        # 3 + 4 = 7
        assert path.total_length() == pytest.approx(7.0)

    def test_len_getitem_iter(self):
        path = self._make_path()
        assert len(path) == 3
        assert path[0].x == pytest.approx(0.0)
        assert path[-1].y == pytest.approx(4.0)
        xs = [p.x for p in path]
        assert xs == [pytest.approx(0), pytest.approx(3), pytest.approx(3)]

    def test_head_last_reverse(self):
        path = self._make_path()
        assert path.head().x == pytest.approx(0.0)
        assert path.last().y == pytest.approx(4.0)
        rev = path.reverse()
        assert rev.head().y == pytest.approx(4.0)
        assert rev.last().x == pytest.approx(0.0)

    def test_empty_path(self):
        path = Path()
        assert len(path) == 0
        assert path.head() is None
        assert path.last() is None
        assert path.total_length() == 0.0

    def test_to_from_dict(self):
        path = self._make_path()
        d = path.to_dict()
        restored = Path.from_dict(d)
        assert len(restored) == 3
        assert restored.total_length() == pytest.approx(7.0)


# ===== OccupancyGrid =====

class TestOccupancyGrid:
    def test_coordinate_conversion(self):
        grid = OccupancyGrid(
            grid=np.zeros((100, 200), dtype=np.int8),
            resolution=0.1,
            origin=Pose(Vector3(-10.0, -5.0, 0.0)),
        )
        assert grid.width == 200
        assert grid.height == 100
        # world (0, 0) -> grid
        row, col = grid.world_to_grid(0.0, 0.0)
        assert col == 100  # (0 - (-10)) / 0.1
        assert row == 50   # (0 - (-5)) / 0.1
        # grid -> world roundtrip
        x, y = grid.grid_to_world(row, col)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)

    def test_cell_value(self):
        grid_data = np.full((10, 10), 0, dtype=np.int8)
        grid_data[5, 5] = 100
        og = OccupancyGrid(grid=grid_data, resolution=1.0, origin=Pose())
        assert og.cell_value(5.0, 5.0) == 100
        assert og.cell_value(0.0, 0.0) == 0
        assert og.cell_value(100.0, 100.0) == -1  # out of bounds

    def test_constants(self):
        og = OccupancyGrid()
        assert og.FREE == 0
        assert og.OCCUPIED == 100
        assert og.UNKNOWN == -1

    def test_encode_decode_roundtrip(self):
        data = np.array([[0, 100, -1], [50, 0, 100]], dtype=np.int8)
        og = OccupancyGrid(grid=data, resolution=0.05, ts=999.0, frame_id="map")
        encoded = og.encode()
        restored = OccupancyGrid.decode(encoded)
        assert restored.width == 3
        assert restored.height == 2
        assert restored.resolution == pytest.approx(0.05)
        np.testing.assert_array_equal(restored.grid, data)


# ===== SceneGraph =====

class TestSceneGraph:
    def _make_sg(self):
        objs = [
            Detection3D(id="obj_1", label="chair", confidence=0.9,
                        position=Vector3(1, 2, 0)),
            Detection3D(id="obj_2", label="table", confidence=0.85,
                        position=Vector3(3, 4, 0)),
            Detection3D(id="obj_3", label="Chair", confidence=0.7,
                        position=Vector3(5, 6, 0)),
        ]
        rels = [Relation(subject_id="obj_1", predicate="near", object_id="obj_2")]
        regs = [Region(name="living_room", object_ids=["obj_1", "obj_2"])]
        return SceneGraph(objects=objs, relations=rels, regions=regs)

    def test_get_object_by_id(self):
        sg = self._make_sg()
        assert sg.get_object_by_id("obj_2").label == "table"
        assert sg.get_object_by_id("nonexistent") is None

    def test_get_objects_by_label(self):
        sg = self._make_sg()
        chairs = sg.get_objects_by_label("chair")
        assert len(chairs) == 2  # case-insensitive

    def test_json_roundtrip(self):
        sg = self._make_sg()
        j = sg.to_json()
        restored = SceneGraph.from_json(j)
        assert len(restored.objects) == 3
        assert len(restored.relations) == 1
        assert restored.relations[0].predicate == "near"

    def test_encode_decode(self):
        sg = self._make_sg()
        data = sg.encode()
        restored = SceneGraph.decode(data)
        assert len(restored.objects) == 3
        assert restored.get_object_by_id("obj_1").label == "chair"


# ===== GoalResult =====

class TestGoalResult:
    def test_as_pose_stamped(self):
        gr = GoalResult(
            action="navigate", target_label="kitchen",
            target_x=10.0, target_y=5.0, target_z=0.0,
            confidence=0.95, is_valid=True, path="fast",
        )
        ps = gr.as_pose_stamped()
        assert ps.x == pytest.approx(10.0)
        assert ps.y == pytest.approx(5.0)
        assert ps.frame_id == "map"

    def test_encode_decode(self):
        gr = GoalResult(
            action="explore", target_label="bed",
            target_x=2.5, target_y=3.5,
            confidence=0.8, is_valid=True, path="slow",
            hint_room="bedroom", score_entropy=1.2,
        )
        data = gr.encode()
        restored = GoalResult.decode(data)
        assert restored.action == "explore"
        assert restored.target_label == "bed"
        assert restored.hint_room == "bedroom"
        assert restored.score_entropy == pytest.approx(1.2)


# ===== Detection3D =====

class TestDetection3D:
    def test_distance_to(self):
        a = Detection3D(id="a", position=Vector3(0, 0, 0))
        b = Detection3D(id="b", position=Vector3(3, 4, 0))
        assert a.distance_to(b) == pytest.approx(5.0)

    def test_encode_decode_with_clip(self):
        clip = np.random.randn(512).astype(np.float32)
        det = Detection3D(
            id="obj_99", label="door", confidence=0.92,
            position=Vector3(1.1, 2.2, 0.3),
            bbox_2d=[10.0, 20.0, 100.0, 200.0],
            clip_feature=clip, ts=12345.0,
        )
        data = det.encode()
        restored = Detection3D.decode(data)
        assert restored.id == "obj_99"
        assert restored.label == "door"
        assert restored.confidence == pytest.approx(0.92)
        np.testing.assert_allclose(restored.clip_feature, clip, rtol=1e-5)


# ===== NavigationCommand =====

class TestNavigationCommand:
    def test_to_twist(self):
        cmd = NavigationCommand(linear_x=0.5, angular_z=0.3)
        tw = cmd.to_twist()
        assert tw.linear.x == pytest.approx(0.5)
        assert tw.angular.z == pytest.approx(0.3)
