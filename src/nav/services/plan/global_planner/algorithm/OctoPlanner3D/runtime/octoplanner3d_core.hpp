#pragma once

#include "global_planner_contract.hpp"

namespace octoplanner3d::runtime {

using Point = lingtu::nav::plan::GlobalPlanPoint;
using PlannerOptions = lingtu::nav::plan::GlobalPlannerOptions;
using PlanRequest = lingtu::nav::plan::GlobalPlanRequest;
using PlanResult = lingtu::nav::plan::GlobalPlanResult;
using CancelCheck = lingtu::nav::plan::GlobalPlanCancelCheck;

bool pcdConversionEnabled();
PlanResult runPlan(
  const PlanRequest & request,
  const CancelCheck & cancel_check = {});

}  // namespace octoplanner3d::runtime
