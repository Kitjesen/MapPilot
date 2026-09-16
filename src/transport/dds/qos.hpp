#pragma once
// DDS QoS profiles used by LingTu native services.

#include "dds/dds.h"
#include "message/generated/topics.hpp"
#include <memory>
#include <string_view>

namespace lingtu::dds {

/// Apply a named QoS profile to an existing dds_qos_t*.
/// Does nothing for QosProfile::Default.
inline void apply_qos_profile(dds_qos_t* qos, QosProfile profile) {
  switch (profile) {
    case QosProfile::Default:
      break;
    case QosProfile::SensorStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 256);
      break;
    case QosProfile::RawLidarStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 2);
      dds_qset_lifespan(qos, DDS_MSECS(350));
      dds_qset_resource_limits(qos, 2, 1, 2);
      break;
    case QosProfile::HighFreqState:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 5);
      dds_qset_deadline(qos, DDS_MSECS(20));
      break;
    case QosProfile::LocalizationHealth:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);
      break;
    case QosProfile::ControlCommand:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      break;
    case QosProfile::FinalVelocityCommand:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      dds_qset_lifespan(qos, DDS_MSECS(200));
      break;
    case QosProfile::CommandRequest:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      break;
    case QosProfile::CommandAck:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 64);
      break;
    case QosProfile::OperatorMotionControl:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      break;
    case QosProfile::OperatorMotionSample:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      dds_qset_lifespan(qos, DDS_MSECS(350));
      break;
    case QosProfile::OperatorMotionAck:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 64);
      break;
    case QosProfile::OperatorMotionStatus:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::InspectionEvidence:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      dds_qset_deadline(qos, DDS_SECS(5));
      dds_qset_lifespan(qos, DDS_SECS(35));
      break;
    case QosProfile::TaskEvent:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 512);
      break;
    case QosProfile::CameraStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::CameraInfo:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::GlobalPath:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::SystemStatus:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::Event:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::MapGrid:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::LocalRiskGrid:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_MSECS(500));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
    case QosProfile::TfDynamic:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 100);
      break;
    case QosProfile::TfStatic:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::LidarPointcloud:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 2);
      dds_qset_lifespan(qos, DDS_MSECS(200));
      break;
    case QosProfile::MapCloud:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_MSECS(500));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
    case QosProfile::MapScene:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(100));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_SECS(2));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
  }
}

/// Lookup recommended QoS profile by DDS topic name (rt/ prefix).
inline QosProfile qos_for_topic(std::string_view dds_topic) {
  for (const auto& contract : message::kTopicContracts) {
    if (dds_topic == contract.dds_topic) return contract.qos_profile;
  }
  return QosProfile::Default;
}

/// RAII convenience: create a QoS object with the given profile applied.
struct QosDeleter { void operator()(dds_qos_t* q) const { if (q) dds_delete_qos(q); } };
using UniqueQos = std::unique_ptr<dds_qos_t, QosDeleter>;

inline UniqueQos make_qos(QosProfile profile) {
  UniqueQos qos(dds_create_qos());
  if (qos && profile != QosProfile::Default) {
    apply_qos_profile(qos.get(), profile);
  }
  return qos;
}

}  // namespace lingtu::dds
