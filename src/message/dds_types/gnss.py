"""Native LingTu DDS GNSS types used by diagnostics and replay."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header


@dataclass
class GnssFix(IdlStruct, typename="lingtu::dds::GnssFix"):
    header: Header
    latitude: types.float64
    longitude: types.float64
    altitude: types.float64
    fix_type: types.int32
    position_covariance: types.array[types.float64, 9]
    num_sat: types.uint32
    num_sat_used: types.uint32
    hdop: types.float64
    rtcm_age_s: types.float64


@dataclass
class GnssStatus(IdlStruct, typename="lingtu::dds::GnssStatus"):
    header: Header
    device: str
    fix_type: types.int32
    link_ok: types.boolean
    rtk: types.boolean
    num_sat: types.uint32
    num_sat_used: types.uint32
    hdop: types.float64
    rtcm_age_s: types.float64
    error: str


DDS_GnssFix = GnssFix
DDS_GnssStatus = GnssStatus
