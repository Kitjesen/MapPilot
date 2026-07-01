#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && !defined(LT_POSE_GRAPH_OPT_STATIC)
#if defined(LT_POSE_GRAPH_OPT_BUILDING_LIBRARY)
#define LT_POSE_GRAPH_OPT_API __declspec(dllexport)
#else
#define LT_POSE_GRAPH_OPT_API __declspec(dllimport)
#endif
#elif defined(__GNUC__) && !defined(LT_POSE_GRAPH_OPT_STATIC)
#define LT_POSE_GRAPH_OPT_API __attribute__((visibility("default")))
#else
#define LT_POSE_GRAPH_OPT_API
#endif

#define LT_POSE_GRAPH_OPT_ABI_VERSION UINT32_C(2)
#define LT_POSE_GRAPH_OPT_CONFIG_VERSION UINT32_C(1)
#define LT_POSE_GRAPH_OPT_REPORT_VERSION UINT32_C(1)

#define LT_POSE_GRAPH_OPT_OK INT32_C(0)
#define LT_POSE_GRAPH_OPT_NULL_POINTER INT32_C(-1)
#define LT_POSE_GRAPH_OPT_EMPTY_GRAPH INT32_C(-2)
#define LT_POSE_GRAPH_OPT_INVALID_INDEX INT32_C(-3)
#define LT_POSE_GRAPH_OPT_NON_FINITE_INPUT INT32_C(-4)
#define LT_POSE_GRAPH_OPT_SINGULAR_SYSTEM INT32_C(-5)
#define LT_POSE_GRAPH_OPT_BUFFER_TOO_SMALL INT32_C(-6)
#define LT_POSE_GRAPH_OPT_INVALID_CONFIG INT32_C(-7)
#define LT_POSE_GRAPH_OPT_GAUGE_FREEDOM INT32_C(-8)

typedef struct lt_pose_graph_opt_handle lt_pose_graph_opt_handle;
typedef int32_t lt_pose_graph_opt_result;

typedef struct lt_pose_graph_opt_pose3 {
    double t_xyz[3];
    double q_wxyz[4];
} lt_pose_graph_opt_pose3;

typedef struct lt_pose_graph_opt_prior3 {
    uint32_t index;
    uint32_t reserved0;
    lt_pose_graph_opt_pose3 pose;
    double information_upper[21];
} lt_pose_graph_opt_prior3;

typedef struct lt_pose_graph_opt_between3 {
    uint32_t from_index;
    uint32_t to_index;
    lt_pose_graph_opt_pose3 pose_from_to;
    double information_upper[21];
} lt_pose_graph_opt_between3;

typedef struct lt_pose_graph_opt_config {
    uint32_t struct_size;
    uint32_t version;
    uint32_t max_iterations;
    uint32_t method;
    int32_t fixed_pose_index;
    uint32_t auto_anchor;
    double initial_lambda;
    double tolerance;
    double numeric_epsilon;
} lt_pose_graph_opt_config;

typedef struct lt_pose_graph_opt_report {
    uint32_t struct_size;
    uint32_t version;
    int32_t status;
    uint32_t iterations;
    uint32_t accepted_steps;
    uint32_t rejected_steps;
    uint32_t converged;
    uint32_t reserved0;
    double initial_cost;
    double final_cost;
} lt_pose_graph_opt_report;

LT_POSE_GRAPH_OPT_API uint32_t lt_pose_graph_opt_abi_version(void);
LT_POSE_GRAPH_OPT_API uint64_t lt_pose_graph_opt_abi_sizeof_pose3(void);
LT_POSE_GRAPH_OPT_API uint64_t lt_pose_graph_opt_abi_sizeof_prior3(void);
LT_POSE_GRAPH_OPT_API uint64_t lt_pose_graph_opt_abi_sizeof_between3(void);
LT_POSE_GRAPH_OPT_API uint64_t lt_pose_graph_opt_abi_sizeof_config(void);
LT_POSE_GRAPH_OPT_API uint64_t lt_pose_graph_opt_abi_sizeof_report(void);

LT_POSE_GRAPH_OPT_API lt_pose_graph_opt_handle* lt_pose_graph_opt_create(
    const lt_pose_graph_opt_config* config);
LT_POSE_GRAPH_OPT_API void lt_pose_graph_opt_destroy(lt_pose_graph_opt_handle* handle);
LT_POSE_GRAPH_OPT_API lt_pose_graph_opt_result lt_pose_graph_opt_reset(
    lt_pose_graph_opt_handle* handle);
LT_POSE_GRAPH_OPT_API lt_pose_graph_opt_result lt_pose_graph_opt_configure(
    lt_pose_graph_opt_handle* handle,
    const lt_pose_graph_opt_config* config);
LT_POSE_GRAPH_OPT_API lt_pose_graph_opt_result lt_pose_graph_opt_process_se3(
    lt_pose_graph_opt_handle* handle,
    const lt_pose_graph_opt_pose3* poses,
    uint64_t pose_count,
    const lt_pose_graph_opt_prior3* priors,
    uint64_t prior_count,
    const lt_pose_graph_opt_between3* betweens,
    uint64_t between_count,
    lt_pose_graph_opt_report* report);
LT_POSE_GRAPH_OPT_API lt_pose_graph_opt_result lt_pose_graph_opt_copy_result_poses(
    const lt_pose_graph_opt_handle* handle,
    lt_pose_graph_opt_pose3* poses,
    uint64_t capacity,
    uint64_t* written);
LT_POSE_GRAPH_OPT_API int32_t lt_pose_graph_opt_last_error(
    const lt_pose_graph_opt_handle* handle);

#ifdef __cplusplus
}
#endif
