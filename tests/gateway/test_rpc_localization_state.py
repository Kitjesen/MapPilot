import pytest

from gateway.rpc import telemetry
from gateway.rpc.generated import localization_pb2 as pb


@pytest.mark.parametrize(("state", "quality"), [
    ("ready", pb.LOCALIZATION_QUALITY_LOCALIZED),
    ("tracking", pb.LOCALIZATION_QUALITY_TRACKING),
    ("no_odometry", pb.LOCALIZATION_QUALITY_NO_ODOMETRY),
    ("initializing", pb.LOCALIZATION_QUALITY_INITIALIZING),
    ("relocalizing", pb.LOCALIZATION_QUALITY_RELOCALIZING),
    ("lost", pb.LOCALIZATION_QUALITY_LOST),
    ("degraded", pb.LOCALIZATION_QUALITY_DEGRADED),
    ("future_state", pb.LOCALIZATION_QUALITY_UNKNOWN),
])
def test_state_preserves_source_and_presence(monkeypatch, state, quality):
    monkeypatch.setattr(telemetry, "build_localization_status", lambda _: {
        "state": state, "active_map": "lab", "ready": False, "reasons": ["owner_reason"],
    })
    result = telemetry.localization_state(object())
    assert result.quality == quality
    assert result.source_state == state and result.map_name == "lab"
    assert result.HasField("ready") and result.ready is False
    assert not result.HasField("pose_fresh") and not result.HasField("time")
    assert result.reason == "owner_reason"
