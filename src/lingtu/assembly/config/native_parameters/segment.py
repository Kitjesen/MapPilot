"""Rolling segment limits and risk thresholds."""

from .schema import ParameterSpec, _integer

SEGMENT_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec(
        "segment.max_distance_m",
        "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M",
        float,
        0.25,
        50.0,
    ),
    ParameterSpec(
        "segment.max_waypoints",
        "LINGTU_NAV_SEGMENT_MAX_WAYPOINTS",
        _integer,
        2,
        4096,
    ),
)

RISK_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec(
        "risk.stop_threshold",
        "LINGTU_NAV_SEGMENT_RISK_STOP",
        float,
        0.0,
        100.0,
    ),
    ParameterSpec(
        "risk.resume_threshold",
        "LINGTU_NAV_SEGMENT_RISK_RESUME",
        float,
        0.0,
        100.0,
    ),
)
