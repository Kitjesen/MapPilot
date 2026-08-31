#include "explore/directed_intent.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

bool sameMapEpoch(const lingtu::explore::ExploreMapIdentity &left,
                  const lingtu::explore::ExploreMapIdentity &right) {
  return left.sameSource(right) && left.reset_epoch == right.reset_epoch;
}

bool validCoordinate(double value) {
  return std::isfinite(value) &&
         std::abs(value) <= DirectedExplorationIntentStore::kMaxCoordinateMeters;
}

}  // namespace

std::uint64_t DirectedExplorationIntentStore::AdvanceRevision() noexcept {
  if (revision_ == std::numeric_limits<std::uint64_t>::max()) {
    revision_ = 1U;
  } else {
    ++revision_;
    if (revision_ == 0U) {
      revision_ = 1U;
    }
  }
  return revision_;
}

DirectedExplorationIntentResult
DirectedExplorationIntentStore::Set(const std::string &product_session_id,
                                    const lingtu::explore::ExploreMapIdentity &map, double target_x,
                                    double target_y, double ttl_s, double now_s) {
  DirectedExplorationIntentResult result;
  if (product_session_id.empty()) {
    result.reason = "directed_target_product_session_empty";
    return result;
  }
  if (!map.valid()) {
    result.reason = "directed_target_map_identity_invalid";
    return result;
  }
  if (!validCoordinate(target_x) || !validCoordinate(target_y)) {
    result.reason = "directed_target_coordinate_invalid";
    return result;
  }
  if (!std::isfinite(ttl_s) || ttl_s <= 0.0 || ttl_s > kMaxTtlSeconds) {
    result.reason = "directed_target_ttl_invalid";
    return result;
  }
  if (!std::isfinite(now_s)) {
    result.reason = "directed_target_clock_invalid";
    return result;
  }

  const std::uint64_t revision = AdvanceRevision();
  current_ = DirectedExplorationIntent{
      {target_x, target_y, 0.0}, map, product_session_id, revision, now_s + ttl_s,
  };
  result.accepted = true;
  result.changed = true;
  result.reason = "directed_exploration_target_set";
  result.revision = revision;
  return result;
}

DirectedExplorationIntentResult DirectedExplorationIntentStore::Clear() {
  DirectedExplorationIntentResult result;
  if (!current_.has_value()) {
    result.accepted = true;
    result.reason = "directed_exploration_target_already_clear";
    result.revision = revision_;
    return result;
  }
  current_.reset();
  result.accepted = true;
  result.changed = true;
  result.reason = "directed_exploration_target_cleared";
  result.revision = AdvanceRevision();
  return result;
}

bool DirectedExplorationIntentStore::Expire(double now_s) {
  if (!current_.has_value() || !std::isfinite(now_s) || now_s < current_->expires_at_s) {
    return false;
  }
  current_.reset();
  static_cast<void>(AdvanceRevision());
  return true;
}

void DirectedExplorationIntentStore::Reset() {
  current_.reset();
  revision_ = 0U;
}

std::optional<DirectedExplorationIntent>
DirectedExplorationIntentStore::current(const std::string &product_session_id,
                                        const lingtu::explore::ExploreMapIdentity &map,
                                        double now_s) const {
  if (!current_.has_value() || !std::isfinite(now_s) || now_s >= current_->expires_at_s ||
      product_session_id != current_->product_session_id || !sameMapEpoch(map, current_->map)) {
    return std::nullopt;
  }
  return current_;
}

}  // namespace lingtu::nav::endpoint
