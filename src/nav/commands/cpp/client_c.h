#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* lingtu_nav_client_handle;

// Command functions are safe to call concurrently on one handle. The caller
// must join/finish all command calls before destroying that handle.

enum {
  LINGTU_NAV_CLIENT_ABI_VERSION = 1,
};

enum {
  LINGTU_NAV_CLIENT_CAP_NAVIGATION = 1ULL << 0,
  LINGTU_NAV_CLIENT_CAP_INSPECTION = 1ULL << 1,
};

uint32_t lingtu_nav_client_abi_version(void);
uint64_t lingtu_nav_client_capabilities(void);

lingtu_nav_client_handle lingtu_nav_client_create(int domain_id);
void lingtu_nav_client_destroy(lingtu_nav_client_handle handle);

int lingtu_nav_client_send_goal(
    lingtu_nav_client_handle handle,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms);

int lingtu_nav_client_send_goal_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms);

int lingtu_nav_client_cancel(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_cancel_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_send_teleop(
    lingtu_nav_client_handle handle,
    double vx,
    double vy,
    double wz,
    int timeout_ms);

int lingtu_nav_client_send_teleop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double vx,
    double vy,
    double wz,
    int timeout_ms);

int lingtu_nav_client_stop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_stop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_clear_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_clear_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_resume_autonomy(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_resume_autonomy_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_start_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* route_id,
    unsigned long long route_revision,
    int timeout_ms);
int lingtu_nav_client_pause_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_resume_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_cancel_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

const char* lingtu_nav_client_last_error(lingtu_nav_client_handle handle);

#ifdef __cplusplus
}
#endif
