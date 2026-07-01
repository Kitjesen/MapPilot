from __future__ import annotations

from runtime.msgs.geometry import Quaternion, Vector3


def test_vector3_equality_preserves_near_zero_tolerance():
    assert Vector3(1e-9, -1e-9, 1e-10) == Vector3()
    assert Vector3(1e-9, -1e-9, 1e-10).is_zero()


def test_quaternion_equality_preserves_near_zero_tolerance():
    assert Quaternion(1e-9, -1e-9, 1e-10, 1.0 + 1e-9) == Quaternion()
