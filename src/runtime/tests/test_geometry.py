"""Tests for lingtu.runtime.msgs.geometry — Vector3, Quaternion, Pose, Twist, Transform."""

import math

import numpy as np

from runtime.msgs.geometry import (
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    TwistStamped,
    Vector3,
)

# ---------------------------------------------------------------------------
# Vector3
# ---------------------------------------------------------------------------

class TestVector3:
    def test_default_zero(self):
        v = Vector3()
        assert v.x == 0.0 and v.y == 0.0 and v.z == 0.0

    def test_init_three_floats(self):
        v = Vector3(1.0, 2.0, 3.0)
        assert v.to_list() == [1.0, 2.0, 3.0]

    def test_init_list(self):
        v = Vector3([4, 5, 6])
        assert v.x == 4.0 and v.z == 6.0

    def test_init_numpy(self):
        v = Vector3(np.array([7, 8, 9]))
        assert v.y == 8.0

    def test_init_copy(self):
        a = Vector3(1, 2, 3)
        b = Vector3(a)
        assert a == b
        b.x = 99
        assert a.x == 1.0  # no aliasing

    def test_init_kwargs(self):
        v = Vector3(x=10, z=30)
        assert v.x == 10.0 and v.y == 0.0 and v.z == 30.0

    def test_add_sub(self):
        a = Vector3(1, 2, 3)
        b = Vector3(4, 5, 6)
        assert a + b == Vector3(5, 7, 9)
        assert b - a == Vector3(3, 3, 3)

    def test_mul_div(self):
        v = Vector3(2, 4, 6)
        assert v * 0.5 == Vector3(1, 2, 3)
        assert 2.0 * v == Vector3(4, 8, 12)
        assert v / 2 == Vector3(1, 2, 3)

    def test_neg(self):
        assert -Vector3(1, -2, 3) == Vector3(-1, 2, -3)

    def test_dot(self):
        assert Vector3(1, 0, 0).dot(Vector3(0, 1, 0)) == 0.0
        assert Vector3(2, 3, 4).dot(Vector3(1, 1, 1)) == 9.0

    def test_cross(self):
        assert Vector3(1, 0, 0).cross(Vector3(0, 1, 0)) == Vector3(0, 0, 1)
        assert Vector3(0, 1, 0).cross(Vector3(1, 0, 0)) == Vector3(0, 0, -1)

    def test_length_normalize(self):
        v = Vector3(3, 4, 0)
        assert abs(v.length() - 5.0) < 1e-12
        n = v.normalize()
        assert abs(n.length() - 1.0) < 1e-12
        assert abs(n.x - 0.6) < 1e-12

    def test_distance(self):
        a = Vector3(0, 0, 0)
        b = Vector3(3, 4, 0)
        assert abs(a.distance(b) - 5.0) < 1e-12

    def test_angle(self):
        a = Vector3(1, 0, 0)
        b = Vector3(0, 1, 0)
        assert abs(a.angle(b) - math.pi / 2) < 1e-10

    def test_is_zero(self):
        assert Vector3().is_zero()
        assert not Vector3(0.1, 0, 0).is_zero()

    def test_encode_decode(self):
        v = Vector3(1.5, -2.7, 3.14)
        v2 = Vector3.decode(v.encode())
        assert v == v2

    def test_to_from_dict(self):
        v = Vector3(1, 2, 3)
        assert Vector3.from_dict(v.to_dict()) == v

    def test_to_numpy(self):
        v = Vector3(1, 2, 3)
        np.testing.assert_array_equal(v.to_numpy(), [1.0, 2.0, 3.0])

    def test_from_numpy(self):
        v = Vector3.from_numpy(np.array([5, 6, 7]))
        assert v == Vector3(5, 6, 7)


# ---------------------------------------------------------------------------
# Quaternion
# ---------------------------------------------------------------------------

class TestQuaternion:
    def test_identity(self):
        q = Quaternion()
        assert q.x == 0 and q.w == 1.0

    def test_init_four_floats(self):
        q = Quaternion(0, 0, 0.1, 0.995)
        assert abs(q.z - 0.1) < 1e-12

    def test_init_list(self):
        q = Quaternion([0, 0, 0, 1])
        assert q.w == 1.0

    def test_hamilton_product_identity(self):
        q = Quaternion.from_euler(0.1, 0.2, 0.3)
        ident = Quaternion()
        assert q * ident == q

    def test_hamilton_product_inverse(self):
        q = Quaternion.from_euler(0.3, 0.1, 0.5)
        result = q * q.inverse()
        assert result == Quaternion()

    def test_rotate_vector(self):
        # 90 deg yaw should rotate X-axis to Y-axis
        q = Quaternion.from_euler(0, 0, math.pi / 2)
        v = q.rotate_vector(Vector3(1, 0, 0))
        assert abs(v.x) < 1e-10
        assert abs(v.y - 1.0) < 1e-10
        assert abs(v.z) < 1e-10

    def test_euler_roundtrip(self):
        roll, pitch, yaw = 0.1, 0.2, 0.3
        q = Quaternion.from_euler(roll, pitch, yaw)
        e = q.to_euler()
        assert abs(e.x - roll) < 1e-10
        assert abs(e.y - pitch) < 1e-10
        assert abs(e.z - yaw) < 1e-10

    def test_from_yaw(self):
        q = Quaternion.from_yaw(math.pi / 4)
        assert abs(q.yaw - math.pi / 4) < 1e-10

    def test_rotation_matrix(self):
        q = Quaternion()  # identity
        R = q.to_rotation_matrix()
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_normalize(self):
        q = Quaternion(1, 1, 1, 1)
        qn = q.normalize()
        norm = math.sqrt(qn.x**2 + qn.y**2 + qn.z**2 + qn.w**2)
        assert abs(norm - 1.0) < 1e-12

    def test_encode_decode(self):
        q = Quaternion(0.1, 0.2, 0.3, 0.927)
        q2 = Quaternion.decode(q.encode())
        assert q == q2

    def test_to_from_dict(self):
        q = Quaternion(0.1, 0.2, 0.3, 0.927)
        assert Quaternion.from_dict(q.to_dict()) == q


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------

class TestPose:
    def test_default(self):
        p = Pose()
        assert p.x == 0 and p.yaw == 0

    def test_three_args(self):
        p = Pose(1, 2, 3)
        assert p.position == Vector3(1, 2, 3)
        assert p.orientation == Quaternion()

    def test_seven_args(self):
        p = Pose(1, 2, 3, 0, 0, 0, 1)
        assert p.x == 1 and p.orientation.w == 1

    def test_composition(self):
        # Robot at (1,0,0) facing forward, object 2m ahead
        p1 = Pose(Vector3(1, 0, 0), Quaternion())
        p2 = Pose(Vector3(2, 0, 0), Quaternion())
        result = p1 + p2
        assert abs(result.x - 3.0) < 1e-10

    def test_subtraction(self):
        p1 = Pose(1, 2, 3)
        p2 = Pose(4, 5, 6)
        delta = p2 - p1
        assert delta.position == Vector3(3, 3, 3)

    def test_encode_decode(self):
        p = Pose(1.5, 2.5, 3.5, 0, 0, 0.1, 0.995)
        p2 = Pose.decode(p.encode())
        assert p == p2

    def test_to_from_dict(self):
        p = Pose(1, 2, 3)
        assert Pose.from_dict(p.to_dict()) == p


# ---------------------------------------------------------------------------
# PoseStamped
# ---------------------------------------------------------------------------

class TestPoseStamped:
    def test_basic(self):
        ps = PoseStamped(pose=Pose(1, 2, 3), ts=100.0, frame_id="odom")
        assert ps.x == 1.0 and ps.frame_id == "odom"
        assert abs(ps.ts - 100.0) < 1e-9

    def test_encode_decode(self):
        ps = PoseStamped(pose=Pose(1, 2, 3, 0, 0, 0.1, 0.995), ts=99.0, frame_id="map")
        ps2 = PoseStamped.decode(ps.encode())
        assert ps == ps2
        assert ps2.frame_id == "map"

    def test_to_from_dict(self):
        ps = PoseStamped(pose=Pose(1, 2, 3), ts=50.0, frame_id="odom")
        ps2 = PoseStamped.from_dict(ps.to_dict())
        assert ps == ps2


# ---------------------------------------------------------------------------
# Twist
# ---------------------------------------------------------------------------

class TestTwist:
    def test_zero(self):
        t = Twist.zero()
        assert t.is_zero()

    def test_add_sub(self):
        a = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0.5))
        b = Twist(Vector3(0, 1, 0), Vector3(0, 0, 0.5))
        s = a + b
        assert s.linear == Vector3(1, 1, 0)
        assert s.angular == Vector3(0, 0, 1.0)

    def test_encode_decode(self):
        t = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0.1))
        t2 = Twist.decode(t.encode())
        assert t == t2

    def test_to_from_dict(self):
        t = Twist(Vector3(1, 2, 3), Vector3(4, 5, 6))
        assert Twist.from_dict(t.to_dict()) == t


# ---------------------------------------------------------------------------
# TwistStamped
# ---------------------------------------------------------------------------

class TestTwistStamped:
    def test_encode_decode(self):
        ts = TwistStamped(Vector3(0.5, 0, 0), Vector3(0, 0, 0.1),
                          ts=123.0, frame_id="body")
        ts2 = TwistStamped.decode(ts.encode())
        assert ts == ts2
        assert ts2.frame_id == "body"


# ---------------------------------------------------------------------------
# Transform
# ---------------------------------------------------------------------------

class TestTransform:
    def test_identity_matrix(self):
        t = Transform.identity()
        np.testing.assert_allclose(t.to_matrix(), np.eye(4), atol=1e-12)

    def test_translation_matrix(self):
        t = Transform(translation=Vector3(1, 2, 3))
        m = t.to_matrix()
        np.testing.assert_allclose(m[:3, 3], [1, 2, 3], atol=1e-12)
        np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-12)

    def test_compose_translations(self):
        t1 = Transform(translation=Vector3(1, 0, 0))
        t2 = Transform(translation=Vector3(2, 0, 0))
        t3 = t1 + t2
        assert abs(t3.translation.x - 3.0) < 1e-10

    def test_inverse(self):
        q = Quaternion.from_euler(0, 0, math.pi / 4)
        t = Transform(translation=Vector3(1, 2, 0), rotation=q,
                       frame_id="map", child_frame_id="body")
        ti = t.inverse()
        # compose should give identity
        result = t + ti
        assert result.translation.is_zero() or result.translation.length() < 1e-10
        assert abs(result.rotation.w - 1.0) < 1e-10 or abs(result.rotation.w + 1.0) < 1e-10

    def test_inverse_swaps_frames(self):
        t = Transform(frame_id="map", child_frame_id="body")
        ti = t.inverse()
        assert ti.frame_id == "body"
        assert ti.child_frame_id == "map"

    def test_encode_decode(self):
        t = Transform(
            translation=Vector3(1.5, -2.0, 3.0),
            rotation=Quaternion.from_euler(0.1, 0.2, 0.3),
            frame_id="odom",
            child_frame_id="base_link",
            ts=500.0,
        )
        t2 = Transform.decode(t.encode())
        assert t == t2
        assert t2.frame_id == "odom"
        assert t2.child_frame_id == "base_link"
        assert abs(t2.ts - 500.0) < 1e-9

    def test_to_from_dict(self):
        t = Transform(
            translation=Vector3(1, 2, 3),
            rotation=Quaternion(),
            frame_id="a",
            child_frame_id="b",
            ts=10.0,
        )
        t2 = Transform.from_dict(t.to_dict())
        assert t == t2
        assert t2.frame_id == "a"

    def test_pose_composition_with_transform(self):
        pose = Pose(Vector3(1, 0, 0), Quaternion())
        tf = Transform(translation=Vector3(2, 0, 0))
        result = pose + tf
        assert abs(result.x - 3.0) < 1e-10
