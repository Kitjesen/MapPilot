"""SCAN path tracking parameters."""

import math

from .schema import ParameterSpec, _positive_float

TRACKING_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec("scan_follower.time_forward_s", "LINGTU_NAV_SCAN_TIME_FORWARD_S", float, 0.0, math.inf),
    ParameterSpec("scan_follower.heading_error_rad", "LINGTU_NAV_SCAN_HEADING_ERROR_RAD", float, 0.0, math.pi),
    ParameterSpec("scan_follower.position_gain", "LINGTU_NAV_SCAN_POSITION_GAIN", float, 0.0, math.inf),
    ParameterSpec("scan_follower.yaw_gain", "LINGTU_NAV_SCAN_YAW_GAIN", float, 0.0, math.inf),
    ParameterSpec("scan_follower.max_vx_mps", "LINGTU_NAV_SCAN_MAX_VX_MPS", _positive_float, 0.0, math.inf),
    ParameterSpec("scan_follower.max_vy_mps", "LINGTU_NAV_SCAN_MAX_VY_MPS", _positive_float, 0.0, math.inf),
    ParameterSpec("scan_follower.min_vy_mps", "LINGTU_NAV_SCAN_MIN_VY_MPS", float, 0.0, math.inf),
    ParameterSpec("scan_follower.max_yaw_rate_rad_s", "LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S", float, 0.0, 1.0),
    ParameterSpec("scan_follower.finish_distance_m", "LINGTU_NAV_SCAN_FINISH_DISTANCE_M", float, 0.01, math.inf),
)
