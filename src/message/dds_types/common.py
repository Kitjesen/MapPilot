"""Native LingTu DDS primitive types."""

from dataclasses import dataclass

from ._base import IdlStruct, types


@dataclass
class Time(IdlStruct, typename="lingtu::dds::Time"):
    sec: types.int32
    nanosec: types.uint32


@dataclass
class Header(IdlStruct, typename="lingtu::dds::Header"):
    stamp: Time
    frame_id: str


@dataclass
class Vector3(IdlStruct, typename="lingtu::dds::Vector3"):
    x: types.float64
    y: types.float64
    z: types.float64


@dataclass
class Quaternion(IdlStruct, typename="lingtu::dds::Quaternion"):
    x: types.float64
    y: types.float64
    z: types.float64
    w: types.float64


@dataclass
class Point(IdlStruct, typename="lingtu::dds::Point"):
    x: types.float64
    y: types.float64
    z: types.float64


DDS_Time = Time
DDS_Header = Header
DDS_Vector3 = Vector3
DDS_Quaternion = Quaternion
DDS_Point = Point
