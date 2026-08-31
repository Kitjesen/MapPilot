#include "run_plan_process_environment.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace lingtu::sim::run_plan_process {
namespace {

std::string environmentValue(const char *const name) {
#ifdef _WIN32
  char *value = nullptr;
  std::size_t size = 0;
  if (_dupenv_s(&value, &size, name) != 0 || value == nullptr) {
    std::free(value);
    throw EnvironmentError(std::string("missing RunPlan process environment: ") + name);
  }
  const std::string result(value, size > 0 ? size - 1 : 0);
  std::free(value);
#else
  const char *const value = std::getenv(name);
  if (value == nullptr) {
    throw EnvironmentError(std::string("missing RunPlan process environment: ") + name);
  }
  const std::string result(value);
#endif
  if (result.empty() ||
      std::isspace(static_cast<unsigned char>(result.front())) != 0 ||
      std::isspace(static_cast<unsigned char>(result.back())) != 0) {
    throw EnvironmentError(std::string("invalid RunPlan process environment: ") + name);
  }
  return result;
}

bool isSafeBasename(const std::string &value) {
  if (value.empty() || value.size() > 128U || value == "." || value == "..") {
    return false;
  }
  const std::filesystem::path path(value);
  if (path.has_root_path() || path.has_parent_path() || path.filename().string() != value) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const unsigned char character) {
    return (character >= 'a' && character <= 'z') ||
           (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') || character == '_' ||
           character == '-' || character == '.';
  });
}

std::filesystem::path exactAbsolutePath(const std::string &value, const char *const label) {
  const std::filesystem::path path(value);
  if (!path.is_absolute() || path.lexically_normal() != path) {
    throw EnvironmentError(std::string(label) + " must be an absolute normalized path");
  }
  return path;
}

void requireRegularFile(const std::filesystem::path &path, const char *const label) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    throw EnvironmentError(std::string(label) + " must be an existing file");
  }
}

bool isProductSessionId(const std::string &value) {
  if (value.empty() || value.size() > 63U ||
      !((value.front() >= 'a' && value.front() <= 'z') ||
        (value.front() >= 'A' && value.front() <= 'Z') ||
        (value.front() >= '0' && value.front() <= '9'))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const unsigned char character) {
    return (character >= 'a' && character <= 'z') ||
           (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') || character == '_' ||
           character == '-' || character == '.';
  });
}

}  // namespace

RunPlanProcessEnvironment loadRunPlanProcessEnvironment(const EndpointFiles &files) {
  if (!isSafeBasename(files.readiness_basename) ||
      !isSafeBasename(files.auth_basename) ||
      files.readiness_basename == files.auth_basename) {
    throw EnvironmentError("local endpoint files must be distinct safe basenames");
  }

  RunPlanProcessEnvironment result;
  result.run_plan_path = exactAbsolutePath(environmentValue("LINGTU_RUN_PLAN"),
                                           "LINGTU_RUN_PLAN");
  result.product_session_id = environmentValue("LINGTU_PRODUCT_SESSION_ID");

  if (!isProductSessionId(result.product_session_id)) {
    throw EnvironmentError("LINGTU_PRODUCT_SESSION_ID is invalid");
  }
  requireRegularFile(result.run_plan_path, "LINGTU_RUN_PLAN");

  result.readiness_path = result.run_plan_path.parent_path() / files.readiness_basename;
  result.auth_file_name = files.auth_basename;
  return result;
}

}  // namespace lingtu::sim::run_plan_process
