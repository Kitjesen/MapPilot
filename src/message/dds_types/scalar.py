"""Native LingTu DDS scalar/status types."""

from dataclasses import dataclass

from ._base import IdlStruct, types


@dataclass
class Float32(IdlStruct, typename="lingtu::dds::Float32"):
    data: types.float32


@dataclass
class Text(IdlStruct, typename="lingtu::dds::Text"):
    data: str


@dataclass
class String(IdlStruct, typename="std_msgs::msg::dds_::String_"):
    data: str


DDS_Float32 = Float32
DDS_String = String
