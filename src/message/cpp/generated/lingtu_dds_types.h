// lingtu_dds_types.h - Compatibility header for TARE DDS transport
//
// This file aggregates all idlc-generated type headers needed by the
// TARE exploration DDS transport (tare_dds.hpp).
//
// explore_types.h is generated with #include "lingtu_slam.idl" so it
// already #includes "lingtu_slam.h" transitively.  Including this
// single header pulls in the complete type set:
//   - lingtu_slam.h: Time, Header, Point, Pose*, Path, Float32, Bool
//   - explore_types.h: PointStamped (TARE-specific)

#pragma once

#include "explore_types.h"
