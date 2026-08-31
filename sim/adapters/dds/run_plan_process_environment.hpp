#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace lingtu::sim::run_plan_process {

class EnvironmentError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct EndpointFiles final {
  std::string readiness_basename;
  std::string auth_basename;
};

struct RunPlanProcessEnvironment final {
  std::string product_session_id;
  std::filesystem::path run_plan_path;
  std::filesystem::path readiness_path;
  std::string auth_file_name;
};

RunPlanProcessEnvironment loadRunPlanProcessEnvironment(const EndpointFiles &files);

}  // namespace lingtu::sim::run_plan_process
