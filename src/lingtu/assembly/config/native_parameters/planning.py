"""SCAN planning parameters."""

import math

from .schema import ParameterSpec

PLANNING_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec("scan_planner.control_point_spacing_m", "LINGTU_NAV_SCAN_CONTROL_POINT_SPACING_M", float, 0.05, math.inf),
    ParameterSpec("scan_planner.replan_distance_m", "LINGTU_NAV_SCAN_REPLAN_DISTANCE_M", float, 0.01, math.inf),
    ParameterSpec("scan_planner.no_replan_distance_m", "LINGTU_NAV_SCAN_NO_REPLAN_DISTANCE_M", float, 0.01, math.inf),
    ParameterSpec("scan_planner.max_velocity_mps", "LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS", float, 0.05, math.inf),
    ParameterSpec("scan_planner.max_acceleration_mps2", "LINGTU_NAV_SCAN_PLANNER_MAX_ACCELERATION_MPS2", float, 0.05, math.inf),
    ParameterSpec("scan_planner.planning_horizon_m", "LINGTU_NAV_SCAN_PLANNING_HORIZON_M", float, 0.5, math.inf),
    ParameterSpec("scan_planner.smooth_weight", "LINGTU_NAV_SCAN_SMOOTH_WEIGHT", float, 0.0, math.inf),
    ParameterSpec("scan_planner.collision_weight", "LINGTU_NAV_SCAN_COLLISION_WEIGHT", float, 0.0, math.inf),
    ParameterSpec("scan_planner.feasibility_weight", "LINGTU_NAV_SCAN_FEASIBILITY_WEIGHT", float, 0.0, math.inf),
    ParameterSpec("scan_planner.fitness_weight", "LINGTU_NAV_SCAN_FITNESS_WEIGHT", float, 0.0, math.inf),
    ParameterSpec("scan_planner.feasibility_tolerance", "LINGTU_NAV_SCAN_FEASIBILITY_TOLERANCE", float, 0.0, math.inf),
    ParameterSpec("scan_planner.velocity_tolerance", "LINGTU_NAV_SCAN_VELOCITY_TOLERANCE", float, 0.0, math.inf),
    ParameterSpec("scan_planner.acceleration_tolerance", "LINGTU_NAV_SCAN_ACCELERATION_TOLERANCE", float, 0.0, math.inf),
    ParameterSpec("scan_planner.collision_distance_m", "LINGTU_NAV_SCAN_COLLISION_DISTANCE_M", float, 0.01, math.inf),
)
