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

bool pcdConversionEnabled();

// Serial planning session that reuses the immutable in-memory OcTree while the
// validated map identity remains unchanged. OctoMap types stay private here.
class PlannerSession {
public:
  PlannerSession();
  ~PlannerSession();

  PlannerSession(const PlannerSession &) = delete;
  PlannerSession & operator=(const PlannerSession &) = delete;

  PlanResult run(
    const std::filesystem::path & map_path,
    const lingtu::nav::plan::MapIdentity & map_identity,
    const PlanRequest & request,
    const CancelCheck & cancel_check = {});

  std::size_t mapLoadCount() const;
  std::size_t prepareCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

PlanResult runPlan(
  const std::filesystem::path & map_path,
  const PlanRequest & request,
  const CancelCheck & cancel_check = {});

}  // namespace octoplanner3d::runtime
