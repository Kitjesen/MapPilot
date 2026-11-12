"""Project the existing GNSS reader into the SDK geographic message."""

from gateway.services import geography

from .generated import localization_pb2 as pb


def read():
    snapshot = geography.geographic_location()
    result = pb.GeographicLocation(
        state={
            "ready": pb.GEOGRAPHIC_STATE_READY,
            "unavailable": pb.GEOGRAPHIC_STATE_UNAVAILABLE,
            "no_fix": pb.GEOGRAPHIC_STATE_NO_FIX,
            "stale": pb.GEOGRAPHIC_STATE_STALE,
            "invalid": pb.GEOGRAPHIC_STATE_INVALID,
        }.get(snapshot["state"], pb.GEOGRAPHIC_STATE_UNKNOWN),
        age_s=snapshot["age_s"],
    )
    if result.state == pb.GEOGRAPHIC_STATE_READY:
        fix = snapshot["location"]
        result.fix.CopyFrom(pb.GeographicFix(
            latitude=fix["latitude"], longitude=fix["longitude"],
            fix_type=fix["fix_type"], accuracy_m=fix["accuracy_m"],
        ))
        seconds, fraction = divmod(fix["timestamp"], 1)
        result.fix.observed_at.FromNanoseconds(int(seconds) * 1_000_000_000 + round(fraction * 1_000_000_000))
    return result
