#pragma once

#include "planning/global/contract.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>

namespace octoplanner3d::runtime {

using Point = lingtu::nav::plan::GlobalPlanPoint;
using PlannerOptions = lingtu::nav::plan::GlobalPlannerOptions;
using PlanRequest = lingtu::nav::plan::GlobalPlanRequest;
using PlanResult = lingtu::nav::plan::GlobalPlanResult;
using CancelCheck = lingtu::nav::plan::GlobalPlanCancelCheck;

struct PlanningMapProjection {
  bool available{false};
  std::string reason;
  lingtu::nav::plan::MapIdentity map_identity;
  double resolution{0.0};
  int rows{0};
  int cols{0};
  Point origin{};
  double reference_z{0.0};
  std::vector<std::uint8_t> cells;
};

bool pcdConversionEnabled();

// Serial planning session that reuses the immutable in-memory OcTree while the
// validated map identity remains unchanged. OctoMap types stay private here.
class PlannerSession {
public:
  explicit PlannerSession(std::filesystem::path cache_directory = {});
  ~PlannerSession();

  PlannerSession(const PlannerSession &) = delete;
  PlannerSession & operator=(const PlannerSession &) = delete;

  // Prepare only immutable map-derived state at activation, without a goal,
  // route search or motion. run() still checks map/options/overlay identity.
  void prepare(
    const std::filesystem::path & map_path,
    const lingtu::nav::plan::MapIdentity & map_identity,
    const PlannerOptions & options);

  PlanResult run(
    const std::filesystem::path & map_path,
    const lingtu::nav::plan::MapIdentity & map_identity,
    const PlanRequest & request,
    const CancelCheck & cancel_check = {});

  std::size_t mapLoadCount() const;
  std::size_t prepareCount() const;
  std::size_t cacheLoadCount() const;
  PlanningMapProjection project(
    const std::filesystem::path & map_path,
    const lingtu::nav::plan::MapIdentity & map_identity,
    const PlannerOptions & options, double reference_z,
    const CancelCheck & cancel_check = {});

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

PlanResult runPlan(
  const std::filesystem::path & map_path,
  const PlanRequest & request,
  const CancelCheck & cancel_check = {});

}  // namespace octoplanner3d::runtime
