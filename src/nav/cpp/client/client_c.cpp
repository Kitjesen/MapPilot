#include "client_c.h"

#include "client.hpp"

#include <memory>
#include <string>

namespace {

struct Handle {
  explicit Handle(int domain_id)
      : client(std::make_unique<lingtu::nav::commands::Client>(domain_id)) {}

  std::unique_ptr<lingtu::nav::commands::Client> client;
};

// C ABI calls can execute concurrently on one handle. Keep the error paired
// with the calling thread so a successful command cannot erase another
// command's failure before ctypes reads it.
thread_local std::string thread_error;

Handle* asHandle(lingtu_nav_client_handle handle) {
  return static_cast<Handle*>(handle);
}

template <typename Operation>
int invoke(lingtu_nav_client_handle raw_handle, Operation&& operation) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr) {
    thread_error = "navigation client handle is null";
    return -1;
  }
  try {
    operation(*handle->client);
    thread_error.clear();
    return 0;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation command failure";
    return -1;
  }
}

}  // namespace

extern "C" {

uint32_t lingtu_nav_client_abi_version(void) {
  return LINGTU_NAV_CLIENT_ABI_VERSION;
}

uint64_t lingtu_nav_client_capabilities(void) {
  return LINGTU_NAV_CLIENT_CAP_NAVIGATION |
      LINGTU_NAV_CLIENT_CAP_INSPECTION |
      LINGTU_NAV_CLIENT_CAP_EXPLORATION;
}

lingtu_nav_client_handle lingtu_nav_client_create(int domain_id) {
  try {
    thread_error.clear();
    return new Handle(domain_id);
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return nullptr;
  } catch (...) {
    thread_error = "unknown native navigation client creation failure";
    return nullptr;
  }
}

void lingtu_nav_client_destroy(lingtu_nav_client_handle handle) {
  delete asHandle(handle);
}

int lingtu_nav_client_send_goal(
    lingtu_nav_client_handle handle,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendGoal(x, y, z, yaw, timeout_ms);
  });
}

int lingtu_nav_client_send_goal_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendGoal(
        x, y, z, yaw, timeout_ms, request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_cancel(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().cancel(reason == nullptr ? "cancel" : reason, timeout_ms);
  });
}

int lingtu_nav_client_cancel_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().cancel(
        reason == nullptr ? "cancel" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_send_teleop(
    lingtu_nav_client_handle handle,
    double vx,
    double vy,
    double wz,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendTeleop(vx, vy, wz, timeout_ms);
  });
}

int lingtu_nav_client_send_teleop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double vx,
    double vy,
    double wz,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendTeleop(
        vx, vy, wz, timeout_ms, request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_stop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().stop(reason == nullptr ? "stop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_stop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().stop(
        reason == nullptr ? "stop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().estop(reason == nullptr ? "estop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().estop(
        reason == nullptr ? "estop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_clear_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().clearEstop(
        reason == nullptr ? "clear_estop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_clear_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().clearEstop(
        reason == nullptr ? "clear_estop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_autonomy(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().resumeAutonomy(
        reason == nullptr ? "resume_autonomy" : reason,
        timeout_ms);
  });
}

int lingtu_nav_client_resume_autonomy_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().resumeAutonomy(
        reason == nullptr ? "resume_autonomy" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_start_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* session_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().start(
        session_id == nullptr ? "" : session_id,
        reason == nullptr ? "operator_start" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_pause_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().pause(
        reason == nullptr ? "operator_pause" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().resume(
        reason == nullptr ? "operator_resume" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_stop_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().stop(
        reason == nullptr ? "operator_stop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_start_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* route_id,
    unsigned long long route_revision,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().start(
        route_id == nullptr ? "" : route_id,
        route_revision,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_pause_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().pause(
        reason == nullptr ? "operator_pause" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().resume(
        reason == nullptr ? "operator_resume" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_cancel_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().cancel(
        reason == nullptr ? "operator_cancel" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

const char* lingtu_nav_client_last_error(lingtu_nav_client_handle handle) {
  (void)handle;
  return thread_error.c_str();
}

}  // extern "C"
