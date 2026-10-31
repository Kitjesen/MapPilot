#pragma once

#include <stdint.h>

#if defined(_WIN32)
#if defined(LINGTU_SENSOR_CONTROL_BRIDGE_BUILD)
#define LINGTU_SENSOR_CONTROL_BRIDGE_API __declspec(dllexport)
#else
#define LINGTU_SENSOR_CONTROL_BRIDGE_API __declspec(dllimport)
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define LINGTU_SENSOR_CONTROL_BRIDGE_API __attribute__((visibility("default")))
#else
#define LINGTU_SENSOR_CONTROL_BRIDGE_API
#endif

#define LT_SENSOR_CONTROL_ID_CAP 256
#define LT_SENSOR_CONTROL_UNIT_CAP 16
#define LT_SENSOR_CONTROL_REASON_CAP 256
#define LT_SENSOR_CONTROL_SETTINGS_CAP 8192

#ifdef __cplusplus
extern "C" {
#endif

// Native alignment (no packing). Arrays include the terminating NUL; fields
// follow sensors.idl order. Booleans use int32_t with only 0/1 accepted.
typedef struct lt_sensor_control_request {
  char request_id[LT_SENSOR_CONTROL_ID_CAP];
  char product_session_id[LT_SENSOR_CONTROL_ID_CAP];
  char session_id[LT_SENSOR_CONTROL_ID_CAP];
  char sensor_id[LT_SENSOR_CONTROL_ID_CAP];
  uint64_t model_generation;
  uint64_t reset_generation;
  double deadline_s;
  int32_t operation;
  int32_t enabled;
  double rate_hz;
  double noise_stddev;
  uint64_t noise_seed;
  char noise_unit[LT_SENSOR_CONTROL_UNIT_CAP];
  double pan_deg;
  double tilt_deg;
} lt_sensor_control_request;

typedef struct lt_sensor_control_result {
  char request_id[LT_SENSOR_CONTROL_ID_CAP];
  char product_session_id[LT_SENSOR_CONTROL_ID_CAP];
  char session_id[LT_SENSOR_CONTROL_ID_CAP];
  char sensor_id[LT_SENSOR_CONTROL_ID_CAP];
  uint64_t model_generation;
  uint64_t reset_generation;
  uint64_t applied_sim_time_ns;
  int32_t applied;
  char reason[LT_SENSOR_CONTROL_REASON_CAP];
  char settings_json[LT_SENSOR_CONTROL_SETTINGS_CAP];
} lt_sensor_control_result;

// Bind both nonempty identities explicitly. owner_mode=1: request reader/result
// writer; owner_mode=0: request writer/result reader. No automatic binding.
// Caller serializes all calls on a handle and owns execution, expiry and replay
// policy. Native validation is limited to binding, strings and scalar validity.
LINGTU_SENSOR_CONTROL_BRIDGE_API void* lt_sensor_control_create(
    int domain, const char* product, const char* source, int owner_mode);
// NULL is allowed. Does not clear the original last_error; never use a closed
// handle again. Closing releases all DDS entities owned by this handle.
LINGTU_SENSOR_CONTROL_BRIDGE_API void lt_sensor_control_destroy(void* handle);
// Nonblocking: examines at most 32 samples, skips foreign/disposed samples.
// 1=sample copied, 0=none, -1=error; output is unchanged unless returning 1.
LINGTU_SENSOR_CONTROL_BRIDGE_API int lt_sensor_control_take_request(
    void* handle, lt_sensor_control_request* request);
LINGTU_SENSOR_CONTROL_BRIDGE_API int lt_sensor_control_take_result(
    void* handle, lt_sensor_control_result* result);
// 0=written, -1=error. All strings must fit with a terminating NUL.
LINGTU_SENSOR_CONTROL_BRIDGE_API int lt_sensor_control_write_request(
    void* handle, const lt_sensor_control_request* request);
LINGTU_SENSOR_CONTROL_BRIDGE_API int lt_sensor_control_write_result(
    void* handle, const lt_sensor_control_result* result);
// Thread-local borrowed text until the next create/take/write on this thread.
LINGTU_SENSOR_CONTROL_BRIDGE_API const char* lt_sensor_control_last_error(void);

#ifdef __cplusplus
}
#endif
