#pragma once

#include <stddef.h>

#if defined(_WIN32)
#if defined(LINGTU_MEASUREMENT_BRIDGE_BUILD)
#define LINGTU_MEASUREMENT_BRIDGE_API __declspec(dllexport)
#else
#define LINGTU_MEASUREMENT_BRIDGE_API __declspec(dllimport)
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define LINGTU_MEASUREMENT_BRIDGE_API __attribute__((visibility("default")))
#else
#define LINGTU_MEASUREMENT_BRIDGE_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Bind both identities explicitly; neither may be null or empty.
// Calls on a handle must be serialized by its owner.
LINGTU_MEASUREMENT_BRIDGE_API void* lt_measurement_create(
    int domain, const char* expected_product, const char* expected_source);
// Nonblocking, bounded polling. Copies one complete LTM1 payload into caller storage.
// 1 = copied, 0 = none, -2 = capacity too small (sample retained), -1 = error.
// needed is required, set to the complete size for 1/-2 and zero otherwise.
// A null buffer with zero capacity is a supported size query.
LINGTU_MEASUREMENT_BRIDGE_API int lt_measurement_take(
    void* handle, void* buffer, size_t capacity, size_t* needed);
LINGTU_MEASUREMENT_BRIDGE_API void lt_measurement_destroy(void* handle);
// Thread-local borrowed text, valid until the next bridge call on this thread.
LINGTU_MEASUREMENT_BRIDGE_API const char* lt_measurement_last_error(void);

#ifdef __cplusplus
}
#endif
