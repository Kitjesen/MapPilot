#include "status/planning_map_writer.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace lingtu::nav::endpoint {

PlanningMapWriter::PlanningMapWriter(
    std::string status_path, std::string product_session_id,
    std::shared_ptr<ActiveOctomapGate> gate, std::string map_path,
    lingtu::nav::plan::GlobalPlannerOptions options)
    : product_session_id_(std::move(product_session_id)), gate_(std::move(gate)),
      map_path_(std::move(map_path)), options_(std::move(options)),
      writer_(status_path.empty() ? std::string{} : status_path + ".planning-map.json") {}

void PlanningMapWriter::update(std::optional<lingtu::nav::plan::MapIdentity> identity,
                               std::optional<double> reference_z, double stamp_s) {
  writer_.submitFactory([this, identity = std::move(identity), reference_z, stamp_s] {
    return build(identity, reference_z, stamp_s);
  });
}

std::string PlanningMapWriter::build(
    const std::optional<lingtu::nav::plan::MapIdentity> &identity,
    std::optional<double> reference_z, double stamp_s) {
  octoplanner3d::runtime::PlanningMapProjection projection;
  if (identity) projection.map_identity = *identity;
  projection.reference_z = reference_z && std::isfinite(*reference_z) ? *reference_z : 0.0;
  if (!gate_) {
    projection.reason = "global_planner_not_octoplanner3d";
  } else if (!identity || !identity->valid()) {
    projection.reason = "active_map_not_ready";
  } else if (!reference_z || !std::isfinite(*reference_z)) {
    projection.reason = "localization_not_ready";
  } else {
    try {
      const bool same_layer = cached_.resolution > 0.0 &&
          std::floor(*reference_z / cached_.resolution) ==
          std::floor(cached_.reference_z / cached_.resolution);
      if (cached_.available && same_layer &&
          lingtu::nav::plan::sameMapIdentity(cached_.map_identity, *identity)) {
        projection = cached_;
        stamp_s = cached_stamp_s_;
      } else {
        const auto prepared = gate_->prepare(map_path_);
        if (!prepared.ok() || !lingtu::nav::plan::sameMapIdentity(
                                  prepared.artifact->identity(), *identity)) {
          projection.reason = "active_map_changed";
        } else {
          projection = session_.project(prepared.artifact->loadPath(), *identity,
                                         options_, *reference_z);
          cached_ = projection;
          cached_stamp_s_ = stamp_s;
        }
      }
    } catch (...) {
      projection.reason = "planning_map_projection_failed";
    }
  }
  std::ostringstream out;
  out << std::setprecision(12)
      << "{\"schema_version\":1,\"available\":" << (projection.available ? "true" : "false")
      << ",\"reason\":" << std::quoted(projection.reason)
      << ",\"frame_id\":\"map\",\"map_id\":" << std::quoted(projection.map_identity.map_id)
      << ",\"map_content_epoch\":" << projection.map_identity.content_epoch
      << ",\"product_session_id\":" << std::quoted(product_session_id_)
      << ",\"stamp_s\":" << stamp_s
      << ",\"resolution\":" << projection.resolution
      << ",\"rows\":" << projection.rows << ",\"cols\":" << projection.cols
      << ",\"origin\":[" << projection.origin.x << ',' << projection.origin.y << ','
      << projection.origin.z << "],\"reference_z\":" << projection.reference_z
      << ",\"cells\":[";
  for (std::size_t i = 0; i < projection.cells.size(); ++i) {
    if (i) out << ',';
    out << static_cast<unsigned>(projection.cells[i]);
  }
  out << "]}\n";
  return out.str();
}

}  // namespace lingtu::nav::endpoint
