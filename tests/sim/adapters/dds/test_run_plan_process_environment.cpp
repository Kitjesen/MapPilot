#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "run_plan_process_environment.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace {

namespace run_plan_process = lingtu::sim::run_plan_process;

class TemporaryDirectory final {
 public:
  TemporaryDirectory() {
    const auto suffix =
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    path_ = std::filesystem::temp_directory_path() /
            ("lingtu-run-plan-process-environment-" + suffix);
    std::filesystem::create_directories(path_);
    path_ = std::filesystem::canonical(path_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path &path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

class ScopedEnvironment final {
 public:
  ScopedEnvironment(const char *name, std::string value) : name_(name) {
#ifdef _WIN32
    char *current = nullptr;
    std::size_t size = 0;
    if (_dupenv_s(&current, &size, name) != 0) {
      std::free(current);
      throw std::runtime_error("failed to read test environment");
    }
    if (current != nullptr) {
      previous_.assign(current, size > 0 ? size - 1 : 0);
      had_previous_ = true;
    }
    std::free(current);
#else
    const char *current = std::getenv(name);
    if (current != nullptr) {
      previous_ = current;
      had_previous_ = true;
    }
#endif
    set(std::move(value));
  }

  ~ScopedEnvironment() {
    if (had_previous_) {
      set(previous_);
    } else {
#ifdef _WIN32
      _putenv_s(name_.c_str(), "");
#else
      unsetenv(name_.c_str());
#endif
    }
  }

  ScopedEnvironment(const ScopedEnvironment &) = delete;
  ScopedEnvironment &operator=(const ScopedEnvironment &) = delete;

 private:
  void set(const std::string &value) const {
#ifdef _WIN32
    if (_putenv_s(name_.c_str(), value.c_str()) != 0) {
      throw std::runtime_error("failed to set test environment");
    }
#else
    if (setenv(name_.c_str(), value.c_str(), 1) != 0) {
      throw std::runtime_error("failed to set test environment");
    }
#endif
  }

  std::string name_;
  std::string previous_;
  bool had_previous_{false};
};

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Callable>
void requireRejected(Callable &&callable, const char *message) {
  try {
    callable();
  } catch (const run_plan_process::EnvironmentError &) {
    return;
  }
  throw std::runtime_error(message);
}

}  // namespace

int main() {
  try {
    TemporaryDirectory session;
    const std::string product_session_id("product-0123456789abcdef0123456789abcdef");
    const auto plan = session.path() / "plan.json";
    std::ofstream(plan, std::ios::binary) << "{}\n";

    ScopedEnvironment run_plan("LINGTU_RUN_PLAN", plan.string());
    ScopedEnvironment product_session_environment(
        "LINGTU_PRODUCT_SESSION_ID", product_session_id);

    const auto identity = run_plan_process::loadRunPlanProcessEnvironment(
        run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.auth"});
    require(identity.product_session_id == product_session_id, "Product session identity mismatch");
    require(identity.run_plan_path == plan, "RunPlan path mismatch");
    require(identity.readiness_path == session.path() / "sensor.ready.json",
            "readiness must be fixed below session root");
    require(identity.auth_file_name == "sensor.auth", "auth basename mismatch");

    for (const char *name : {"LINGTU_RUN_PLAN", "LINGTU_PRODUCT_SESSION_ID"}) {
      ScopedEnvironment missing(name, "");
      requireRejected(
          [] {
            static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
                run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.auth"}));
          },
          "missing RunPlan process environment was accepted");
    }
    {
      ScopedEnvironment unsafe_session("LINGTU_PRODUCT_SESSION_ID", "unsafe/session");
      requireRejected(
          [] {
            static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
                run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.auth"}));
          },
          "unsafe Product session id was accepted");
    }
    {
      ScopedEnvironment wrong_plan("LINGTU_RUN_PLAN",
                                   (session.path() / "different.json").string());
      requireRejected(
          [] {
            static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
                run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.auth"}));
          },
          "missing RunPlan path was accepted");
    }
    requireRejected(
        [] {
          static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
              run_plan_process::EndpointFiles{"../sensor.ready.json", "sensor.auth"}));
        },
        "unsafe readiness basename was accepted");
    requireRejected(
        [] {
          static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
              run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.ready.json"}));
        },
        "shared readiness/auth basename was accepted");

    std::filesystem::remove(plan);
    requireRejected(
        [] {
          static_cast<void>(run_plan_process::loadRunPlanProcessEnvironment(
              run_plan_process::EndpointFiles{"sensor.ready.json", "sensor.auth"}));
        },
        "missing RunPlan artifact was accepted");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_run_plan_process_environment failed: %s\n", error.what());
    return 1;
  }
}
