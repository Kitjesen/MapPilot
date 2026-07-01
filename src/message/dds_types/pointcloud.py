"""Native LingTu DDS PointCloud2 type."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header


@dataclass
class PointField(IdlStruct, typename="lingtu::dds::PointField"):
    name: str
    offset: types.uint32
    datatype: types.uint8
    count: types.uint32


@dataclass
class PointCloud2(IdlStruct, typename="lingtu::dds::PointCloud2"):
    header: Header
    height: types.uint32
    width: types.uint32
    fields: types.sequence[PointField]
    is_bigendian: bool
    point_step: types.uint32
    row_step: types.uint32
    data: types.sequence[types.uint8]
    is_dense: bool


DDS_PointField = PointField
DDS_PointCloud2 = PointCloud2
