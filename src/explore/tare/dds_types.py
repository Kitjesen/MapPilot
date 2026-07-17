"""Native CycloneDDS IDL types for the TARE exploration topic set.

Centralizes the small set of ROS2-compatible DDS IDL structs shared by the
in-process TARE explorer (``explore.tare.module``) and the native DDS bridge
(``nav.adapters.dds.tare_bridge``). Topic names come from
``runtime.runtime_interface.TOPICS`` at the call sites; this module only owns
the wire type definitions.

Importing this module never fails when cyclonedds is missing: the type objects
fall back to ``None`` and ``HAS_CYCLONEDDS`` reports availability, mirroring the
tolerant import strategy used by ``runtime.adapters.dds.reader``.

NOTE: Do NOT add ``from __future__ import annotations`` -- cyclonedds IdlStruct
needs real type objects at class definition time, not string annotations.
"""

import logging

logger = logging.getLogger(__name__)

try:
    from dataclasses import dataclass

    from cyclonedds.idl import IdlStruct, types

    HAS_CYCLONEDDS = True

    @dataclass
    class DDS_Time(IdlStruct):
        sec: types.int32
        nanosec: types.uint32

    @dataclass
    class DDS_Header(IdlStruct):
        stamp: DDS_Time
        frame_id: str

    @dataclass
    class DDS_Point(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Quaternion(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64
        w: types.float64

    @dataclass
    class DDS_Pose(IdlStruct):
        position: DDS_Point
        orientation: DDS_Quaternion

    @dataclass
    class DDS_PoseStamped(IdlStruct):
        header: DDS_Header
        pose: DDS_Pose

    @dataclass
    class DDS_PointStamped(IdlStruct, typename="geometry_msgs::msg::dds_::PointStamped_"):
        header: DDS_Header
        point: DDS_Point

    @dataclass
    class DDS_Path(IdlStruct, typename="nav_msgs::msg::dds_::Path_"):
        header: DDS_Header
        poses: types.sequence[DDS_PoseStamped]

    @dataclass
    class DDS_Float32(IdlStruct, typename="std_msgs::msg::dds_::Float32_"):
        data: types.float32

    @dataclass
    class DDS_Bool(IdlStruct, typename="std_msgs::msg::dds_::Bool_"):
        data: bool

except ImportError:
    HAS_CYCLONEDDS = False
    DDS_Time = None
    DDS_Header = None
    DDS_Point = None
    DDS_Quaternion = None
    DDS_Pose = None
    DDS_PoseStamped = None
    DDS_PointStamped = None
    DDS_Path = None
    DDS_Float32 = None
    DDS_Bool = None


def load_tare_dds_types():
    """Return the four TARE-specific DDS IDL types, or ``None`` when cyclonedds
    is not installed (stub mode).

    The tuple order ``(DDS_PointStamped, DDS_Path, DDS_Float32, DDS_Bool)`` is
    the contract consumed by both the in-process explorer and the DDS bridge.
    """
    if not HAS_CYCLONEDDS:
        return None
    return (DDS_PointStamped, DDS_Path, DDS_Float32, DDS_Bool)
