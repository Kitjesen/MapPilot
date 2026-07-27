"""Native LingTu DDS SLAM service types."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .geometry import Pose


@dataclass
class RelocalizationRequest(
    IdlStruct, typename="lingtu::dds::RelocalizationRequest"
):
    request_id: str
    action: str
    engine: str
    map_path: str
    has_initial_pose: bool
    initial_pose: Pose
    timeout_s: types.float64


@dataclass
class RelocalizationResponse(
    IdlStruct, typename="lingtu::dds::RelocalizationResponse"
):
    request_id: str
    action: str
    engine: str
    success: bool
    message: str
    quality: types.float64
    map_loaded: bool
    has_map_body: bool
    map_body: Pose
    has_map_odom: bool
    map_odom: Pose
    state: str
    refine_backend: str
    refine_iterations: types.int32
    refine_inliers: types.int32
    refine_input_points: types.int32
    refine_evaluated_points: types.int32
    refine_support_ratio: types.float64
    refine_overlap_inlier_ratio: types.float64
    refine_converged: bool
    refine_pos_cov_trace: types.float64
    track_against_map_supported: bool
    track_against_map_enabled: bool
    track_against_map_failures: types.int32


DDS_RelocalizationRequest = RelocalizationRequest
DDS_RelocalizationResponse = RelocalizationResponse
