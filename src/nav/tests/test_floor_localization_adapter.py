from __future__ import annotations

from pathlib import Path

from nav.building import ActiveFloor, NativeFloorLocalizationAdapter, PoseTarget
from runtime.relocalization import RelocalizationResult


class FakeMapsService:
    def __init__(self, *, active: str, pcd_path: Path) -> None:
        self.active = active
        self.pcd_path = pcd_path
        self.calls: list[tuple] = []
        self.activation_ok = True
        self.bundle_ok = True

    def get_active_map(self):
        self.calls.append(("get_active_map",))
        return {"success": True, "active": self.active}

    def get_map_bundle(self, name, capability):
        self.calls.append(("get_map_bundle", name, capability))
        if not self.bundle_ok:
            return {"success": False, "message": "source point cloud missing"}
        return {
            "success": True,
            "map_id": name,
            "artifact": {"uri": str(self.pcd_path)},
        }

    def set_active_map(self, name):
        self.calls.append(("set_active_map", name))
        if not self.activation_ok:
            return {"success": False, "message": "artifact gate failed"}
        self.active = name
        return {"success": True, "active": name}


class FakeRelocalizationService:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.result = RelocalizationResult(True, "native relocalized", quality=0.03)
        self.after_call = None

    def relocalize_saved_map(self, pcd_path, x, y, yaw, *, timeout_s=30.0):
        self.calls.append((str(pcd_path), x, y, yaw, timeout_s))
        if self.after_call is not None:
            self.after_call()
        return self.result


def _floor(number: int) -> ActiveFloor:
    return ActiveFloor(
        building_id="factory-a",
        floor_id=f"floor-{number}",
        map_id=f"factory-a-floor-{number}",
    )


def test_native_floor_switch_validates_artifact_then_activates_and_relocalizes(tmp_path) -> None:
    pcd = tmp_path / "floor-2" / "map.pcd"
    pcd.parent.mkdir()
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=pcd)
    relocalization = FakeRelocalizationService()
    localization_status = {
        "state": "TRACKING",
        "ready": True,
        "pose_fresh": True,
        "has_odometry": True,
        "active_map": _floor(2).map_id,
        "relocalization_state": "completed",
        "ts": 100.2,
    }
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=relocalization,
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: localization_status,
        clock=lambda: 100.2,
        max_status_age_s=1.0,
    )
    seed = PoseTarget("map", 5.0, 1.0, 3.6, 0.25)

    assert adapter.switch_and_relocalize(_floor(2), seed) == (
        True,
        "native_relocalization_completed",
    )
    assert relocalization.calls == [(str(pcd), 5.0, 1.0, 0.25, 30.0)]
    assert maps.calls[:3] == [
        ("get_map_bundle", _floor(2).map_id, "source_pointcloud"),
        ("set_active_map", _floor(2).map_id),
        ("get_active_map",),
    ]
    assert adapter.active_floor() == _floor(2)
    assert adapter.is_localized(_floor(2)) is True


def test_floor_switch_rejects_missing_source_cloud_before_active_map_changes(tmp_path) -> None:
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=tmp_path / "missing.pcd")
    maps.bundle_ok = False
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=FakeRelocalizationService(),
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: {},
    )

    accepted, reason = adapter.switch_and_relocalize(
        _floor(2),
        PoseTarget("map", 0.0, 0.0, 0.0, 0.0),
    )

    assert accepted is False
    assert reason == "target_source_pointcloud_unavailable"
    assert ("set_active_map", _floor(2).map_id) not in maps.calls
    assert maps.active == _floor(1).map_id


def test_floor_switch_fails_closed_when_native_relocalization_fails(tmp_path) -> None:
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=pcd)
    relocalization = FakeRelocalizationService()
    relocalization.result = RelocalizationResult(False, "icp did not converge")
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=relocalization,
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: {},
    )

    accepted, reason = adapter.switch_and_relocalize(
        _floor(2),
        PoseTarget("map", 1.0, 2.0, 0.0, 0.0),
    )

    assert accepted is False
    assert reason == "native_relocalization_failed:icp did not converge"
    assert maps.active == _floor(2).map_id
    assert adapter.is_localized(_floor(2)) is False


def test_floor_switch_fails_closed_when_target_map_activation_is_rejected(tmp_path) -> None:
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=pcd)
    maps.activation_ok = False
    relocalization = FakeRelocalizationService()
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=relocalization,
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: {},
    )

    accepted, reason = adapter.switch_and_relocalize(
        _floor(2),
        PoseTarget("map", 1.0, 2.0, 0.0, 0.0),
    )

    assert accepted is False
    assert reason == "target_map_activation_failed:artifact gate failed"
    assert relocalization.calls == []
    assert maps.active == _floor(1).map_id


def test_floor_switch_detects_active_map_change_during_relocalization(tmp_path) -> None:
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=pcd)
    relocalization = FakeRelocalizationService()
    relocalization.after_call = lambda: setattr(maps, "active", _floor(1).map_id)
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=relocalization,
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: {},
    )

    accepted, reason = adapter.switch_and_relocalize(
        _floor(2),
        PoseTarget("map", 1.0, 2.0, 0.0, 0.0),
    )

    assert accepted is False
    assert reason == "target_map_changed_during_relocalization"
    assert adapter.is_localized(_floor(2)) is False


def test_target_localization_requires_fresh_matching_tracking_status(tmp_path) -> None:
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(2).map_id, pcd_path=pcd)
    state = {
        "state": "TRACKING",
        "ready": True,
        "pose_fresh": True,
        "has_odometry": True,
        "active_map": _floor(2).map_id,
        "relocalization_state": "completed",
        "ts": 90.0,
    }
    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=FakeRelocalizationService(),
        floors=[_floor(1), _floor(2)],
        localization_status=lambda: state,
        clock=lambda: 100.0,
        max_status_age_s=1.0,
    )

    assert adapter.is_localized(_floor(2)) is False
    state["ts"] = 100.0
    state["active_map"] = _floor(1).map_id
    assert adapter.is_localized(_floor(2)) is False
    state["active_map"] = _floor(2).map_id
    state["pose_fresh"] = False
    assert adapter.is_localized(_floor(2)) is False


def test_target_localization_fails_closed_on_status_dropout(tmp_path) -> None:
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    maps = FakeMapsService(active=_floor(1).map_id, pcd_path=pcd)
    relocalization = FakeRelocalizationService()

    def unavailable_status():
        raise RuntimeError("native status stream unavailable")

    adapter = NativeFloorLocalizationAdapter(
        maps=maps,
        relocalization=relocalization,
        floors=[_floor(1), _floor(2)],
        localization_status=unavailable_status,
    )
    assert adapter.switch_and_relocalize(
        _floor(2),
        PoseTarget("map", 1.0, 2.0, 0.0, 0.0),
    )[0]

    assert adapter.is_localized(_floor(2)) is False
