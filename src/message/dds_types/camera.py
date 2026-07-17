"""Native LingTu DDS camera types."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header


@dataclass
class Image(IdlStruct, typename="lingtu::dds::Image"):
    header: Header
    height: types.uint32
    width: types.uint32
    encoding: str
    is_bigendian: bool
    step: types.uint32
    data: types.sequence[types.uint8]


@dataclass
class CameraInfo(IdlStruct, typename="lingtu::dds::CameraInfo"):
    header: Header
    height: types.uint32
    width: types.uint32
    depth_scale: types.float64
    distortion_model: str
    d: types.sequence[types.float64]
    k: types.array[types.float64, 9]
    r: types.array[types.float64, 9]
    p: types.array[types.float64, 12]


DDS_Image = Image
DDS_CameraInfo = CameraInfo
