from gateway.services.loc_cache import LocCache


def test_cache_preserves_native_distance_across_map_corrections() -> None:
    cache = LocCache()
    for ts, x in enumerate([0.0, 0.4, 0.0], start=1):
        sample = {
            "x": x, "y": 0.0, "ts": ts, "frame_id": "map",
            "session_distance_m": 12.5, "session_distance_source": "slam_odom",
        }
        cache.record_odometry(sample, ts=ts)
        assert cache._odom is sample
        assert sample["session_distance_m"] == 12.5


def test_cache_does_not_invent_distance_without_localization_telemetry() -> None:
    cache = LocCache()
    for ts, x in enumerate([0.0, 0.5, 1.0], start=1):
        sample = {"x": x, "y": 0.0, "ts": ts, "frame_id": "map"}
        cache.record_odometry(sample, ts=ts)
        assert "session_distance_m" not in sample


def test_clear_discards_cached_localization() -> None:
    cache = LocCache()
    cache.record_odometry({"session_distance_m": 12.5}, ts=1.0)
    cache.clear()
    assert cache._odom is None
    assert cache._odom_timestamps == []
