#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <optional>

#include "input/active/octomap.hpp"
#include "planning/global/octoplanner/octoplanner3d_core.hpp"
#include "planning/semantic/contract.hpp"

namespace lingtu::nav::endpoint {

struct SemanticViewContext {
  semantic::ViewQuery query;
  std::string boot_id;
  plan::MapIdentity map;
  std::uint64_t frame_epoch{0U};
  plan::GlobalPlanPoint robot;
  double yaw{0.0};
  double timestamp_s{0.0};
};

// One read-only worker with a private planner session; it cannot own motion.
class SemanticViewQuery {
 public:
  SemanticViewQuery(std::shared_ptr<ActiveOctomapGate> gate, std::string map_path,
                    plan::GlobalPlannerOptions options);
  ~SemanticViewQuery();
  bool start(SemanticViewContext context);
  std::optional<semantic::ViewResult> poll();
  void cancel();
  bool busy() const { return future_.valid(); }

 private:
  semantic::ViewResult run(const SemanticViewContext& context);
  std::shared_ptr<ActiveOctomapGate> gate_;
  std::string map_path_;
  plan::GlobalPlannerOptions options_;
  octoplanner3d::runtime::PlannerSession session_;
  std::atomic_bool cancelled_{false};
  std::future<semantic::ViewResult> future_;
};

}  // namespace lingtu::nav::endpoint
