"""Native LingTu DDS scalar/status types."""

from dataclasses import dataclass

from ._base import IdlStruct, types

_bool_type = getattr(types, "boolean", bool)


@dataclass
class Float32(IdlStruct, typename="lingtu::dds::Float32"):
    data: types.float32


@dataclass
class Bool(IdlStruct, typename="lingtu::dds::Bool"):
    data: _bool_type


@dataclass
class Text(IdlStruct, typename="lingtu::dds::Text"):
    data: str


String = Text


DDS_Float32 = Float32
DDS_Bool = Bool
DDS_String = String
