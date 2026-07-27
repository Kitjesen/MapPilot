#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* lingtu_nav_client_handle;

// Command functions are safe to call concurrently on one handle. The caller
// must join/finish all command calls before destroying that handle.

enum {
  // Field Products atomically package this library with its Host bindings.
  // Mixed global ABI versions fail closed; append-only feature discovery
  // within one version uses the capability bits below.
  LINGTU_NAV_CLIENT_ABI_VERSION = 5,
};

enum {
  LINGTU_NAV_CLIENT_CAP_NAVIGATION = 1ULL << 0,
  LINGTU_NAV_CLIENT_CAP_INSPECTION = 1ULL << 1,
  LINGTU_NAV_CLIENT_CAP_EXPLORATION = 1ULL << 2,
  LINGTU_NAV_CLIENT_CAP_DIRECTED_EXPLORATION = 1ULL << 3,
  LINGTU_NAV_CLIENT_CAP_OPERATOR_MOTION = 1ULL << 4,
  LINGTU_NAV_CLIENT_CAP_HOST_STATE = 1ULL << 5,
  LINGTU_NAV_CLIENT_CAP_GOAL_STATUS = 1ULL << 6,
  LINGTU_NAV_CLIENT_CAP_PATH_TELEMETRY = 1ULL << 7,
  LINGTU_NAV_CLIENT_CAP_MAP_SCENE = 1ULL << 8,
  LINGTU_NAV_CLIENT_CAP_OPERATOR_MOTION_RECEIPT = 1ULL << 9,
  LINGTU_NAV_CLIENT_CAP_NAVIGATION_COMMAND_RECEIPT = 1ULL << 10,
  LINGTU_NAV_CLIENT_CAP_NAVIGATION_TASK_STATUS = 1ULL << 11,
  LINGTU_NAV_CLIENT_CAP_NAVIGATION_STATE_V1 = 1ULL << 12,
  LINGTU_NAV_CLIENT_CAP_NAVIGATION_GOAL_STATUS_V1 = 1ULL << 13,
};

enum {
  LINGTU_NAV_MAP_SCENE_ABI_VERSION = 1,
  LINGTU_NAV_MAP_SCENE_MAX_POINTS_PER_LAYER = 300000,
  LINGTU_NAV_MAP_SCENE_MAX_TOTAL_POINTS = 800000,
  LINGTU_NAV_MAP_SCENE_MAX_GRID_CELLS_PER_LAYER = 1000000,
  LINGTU_NAV_MAP_SCENE_MAX_TOTAL_GRID_CELLS = 4000000,
  LINGTU_NAV_MAP_SCENE_MAX_PAYLOAD_BYTES = 32 * 1024 * 1024,
};

enum {
  LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION = 1,
  LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION = 1,
  LINGTU_NAV_NAVIGATION_GOAL_STATUS_ABI_VERSION = 1,
  LINGTU_NAV_NAVIGATION_STATE_ABI_VERSION = 1,
};

typedef struct lingtu_nav_navigation_state {
  double timestamp_s;
  char frame_id[32];
  char boot_id[128];
  unsigned long long sequence;
  int32_t control_mode;
  int32_t lifecycle_state;
  char active_request_id[128];
  unsigned long long goal_epoch;
  char map_id[128];
  int64_t map_version;
  char map_hash[128];
  int32_t planning_state;
  int32_t execution_state;
  int32_t recovery_state;
  float progress;
  char authority[32];
  char hold_reason[128];
  char failure_code[128];
} lingtu_nav_navigation_state;


typedef struct lingtu_nav_navigation_state_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  double timestamp_s;
  char frame_id[32];
  char boot_id[128];
  unsigned long long sequence;
  int32_t control_mode;
  int32_t lifecycle_state;
  char active_task_id[128];
  char active_request_id[128];
  unsigned long long goal_epoch;
  char map_id[128];
  int64_t map_version;
  char map_hash[128];
  int32_t planning_state;
  int32_t execution_state;
  int32_t recovery_state;
  float progress;
  char authority[32];
  char hold_reason[128];
  char failure_code[128];
} lingtu_nav_navigation_state_v1;
typedef struct lingtu_nav_navigation_goal_status {
  double timestamp_s;
  char frame_id[32];
  char boot_id[128];
  unsigned long long sequence;
  char request_id[128];
  int32_t state;
  unsigned long long goal_epoch;
  char reason[256];
} lingtu_nav_navigation_goal_status;

typedef struct lingtu_nav_navigation_command_receipt_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  char task_id[128];
  char request_id[128];
  int32_t accepted;
  int32_t kind;
  char reason[256];
  double endpoint_timestamp_s;
} lingtu_nav_navigation_command_receipt_v1;

typedef struct lingtu_nav_navigation_goal_status_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  double timestamp_s;
  char frame_id[32];
  char boot_id[128];
  unsigned long long sequence;
  char task_id[128];
  char request_id[128];
  int32_t state;
  unsigned long long goal_epoch;
  char reason[256];
} lingtu_nav_navigation_goal_status_v1;

typedef struct lingtu_nav_path_point {
  double x;
  double y;
  double z;
} lingtu_nav_path_point;

typedef struct lingtu_nav_path_header {
  double timestamp_s;
  char frame_id[32];
  unsigned long long receive_sequence;
  unsigned long long point_count;
} lingtu_nav_path_header;

typedef struct lingtu_nav_operator_motion_receipt_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  int32_t accepted;
  int32_t action;
  char request_id[128];
  char source_id[128];
  unsigned long long source_epoch;
  unsigned long long source_sequence;
  unsigned long long accepted_sequence;
  unsigned long long final_output_sequence;
  double endpoint_timestamp_s;
  char reason[256];
} lingtu_nav_operator_motion_receipt_v1;

typedef struct lingtu_nav_map_scene_point_v1 {
  float x;
  float y;
  float z;
  float intensity;
} lingtu_nav_map_scene_point_v1;

typedef struct lingtu_nav_map_scene_grid_header_v1 {
  uint32_t width;
  uint32_t height;
  float resolution;
  double origin_x;
  double origin_y;
  double origin_z;
  double origin_qx;
  double origin_qy;
  double origin_qz;
  double origin_qw;
  unsigned long long cell_count;
} lingtu_nav_map_scene_grid_header_v1;

typedef struct lingtu_nav_map_scene_header_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  double timestamp_s;
  char frame_id[32];
  char producer_boot_id[128];
  unsigned long long receive_sequence;
  unsigned long long reset_epoch;
  unsigned long long observation_sequence;
  unsigned long long generation;
  int32_t live;
  double sensor_x;
  double sensor_y;
  double sensor_z;
  double sensor_qx;
  double sensor_qy;
  double sensor_qz;
  double sensor_qw;
  unsigned long long payload_bytes;
  unsigned long long live_point_count;
  unsigned long long voxel_point_count;
  unsigned long long accumulated_point_count;
  lingtu_nav_map_scene_grid_header_v1 occupancy;
  lingtu_nav_map_scene_grid_header_v1 elevation;
  lingtu_nav_map_scene_grid_header_v1 esdf;
  lingtu_nav_map_scene_grid_header_v1 traversability;
} lingtu_nav_map_scene_header_v1;

typedef struct lingtu_nav_map_scene_buffers_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  lingtu_nav_map_scene_point_v1* live_points;
  unsigned long long live_point_capacity;
  lingtu_nav_map_scene_point_v1* voxel_points;
  unsigned long long voxel_point_capacity;
  lingtu_nav_map_scene_point_v1* accumulated_points;
  unsigned long long accumulated_point_capacity;
  float* occupancy_cells;
  unsigned long long occupancy_cell_capacity;
  float* elevation_cells;
  unsigned long long elevation_cell_capacity;
  float* esdf_cells;
  unsigned long long esdf_cell_capacity;
  float* traversability_cells;
  unsigned long long traversability_cell_capacity;
} lingtu_nav_map_scene_buffers_v1;

typedef struct lingtu_nav_map_scene_health_v1 {
  uint32_t abi_version;
  uint32_t struct_size;
  unsigned long long received_samples;
  unsigned long long valid_samples;
  unsigned long long stale_samples;
  unsigned long long invalid_samples;
  unsigned long long capacity_rejections;
  unsigned long long replaced_samples;
  unsigned long long consumer_buffer_retries;
  unsigned long long last_receive_sequence;
  unsigned long long last_generation;
  double last_sample_timestamp_s;
  int32_t pending;
  char last_error[256];
  unsigned long long state_received_samples;
  unsigned long long state_valid_samples;
  unsigned long long state_stale_samples;
  unsigned long long state_invalid_samples;
  double state_timestamp_s;
  char state_producer_boot_id[128];
  int32_t state_received;
  int32_t state_running;
  int32_t state_live;
  int32_t state_required_publications_ready;
  int32_t state_current_generation_published;
  int32_t state_capacity_limited;
  unsigned long long state_reset_epoch;
  unsigned long long state_observation_sequence;
  unsigned long long state_generation;
  unsigned long long state_scene_published_generation;
  char state_error[256];
} lingtu_nav_map_scene_health_v1;

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

// Returns 0 after receiving a correlated ACK, including accepted=false.
// Transport, timeout, and argument errors return -1.
int lingtu_nav_client_start_task_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* task_id,
    const char* request_id,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms,
    lingtu_nav_navigation_command_receipt_v1* receipt);

int lingtu_nav_client_cancel(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms);

int lingtu_nav_client_cancel_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);

// Cancels the task identified by task_id. Returns 0 after a correlated ACK,
// including accepted=false. Transport, timeout, and argument errors return -1.
int lingtu_nav_client_cancel_task_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* task_id,
    const char* request_id,
    const char* reason,
    int timeout_ms,
    lingtu_nav_navigation_command_receipt_v1* receipt);

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

int lingtu_nav_client_start_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* session_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_pause_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_resume_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_stop_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_set_directed_exploration_target(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double x,
    double y,
    double ttl_s,
    const char* session_id,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_clear_directed_exploration_target(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* session_id,
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

int lingtu_nav_client_operator_motion_claim(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    unsigned int lease_ttl_ms,
    int timeout_ms);
int lingtu_nav_client_operator_motion_sample(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    int deadman,
    double vx,
    double vy,
    double wz,
    unsigned int freshness_budget_ms,
    int timeout_ms);
int lingtu_nav_client_operator_motion_hold(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms);
int lingtu_nav_client_operator_motion_release(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms);

// Returns 0 after receiving a correlated ACK, including accepted=false.
// Transport, timeout, and argument errors return -1.
int lingtu_nav_client_operator_motion_claim_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    unsigned int lease_ttl_ms,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt);
int lingtu_nav_client_operator_motion_hold_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt);
int lingtu_nav_client_operator_motion_release_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt);

// Returns 1 when a state is available, 0 before the first state, and -1 on
// error. The returned value is an owning C snapshot and remains valid after
// this call returns.
int lingtu_nav_client_read_navigation_state(
    lingtu_nav_client_handle handle,
    lingtu_nav_navigation_state* state);

// Versioned state snapshot including active_task_id. The caller must set
// abi_version and struct_size before calling.
int lingtu_nav_client_read_navigation_state_v1(
    lingtu_nav_client_handle handle,
    lingtu_nav_navigation_state_v1* state);

// Pops one deduplicated lifecycle event from the bounded client queue.
// Returns 1 when an event is available, 0 when the queue is empty, and -1 on
// error.
int lingtu_nav_client_take_navigation_goal_status(
    lingtu_nav_client_handle handle,
    lingtu_nav_navigation_goal_status* status);

// Versioned lifecycle event including task_id. The caller must set abi_version
// and struct_size before calling.
int lingtu_nav_client_take_navigation_goal_status_v1(
    lingtu_nav_client_handle handle,
    lingtu_nav_navigation_goal_status_v1* status);

// Reads the retained latest lifecycle state for request_id without consuming
// the event queue. Returns 1 when found, 0 when unknown, and -1 on error.
int lingtu_nav_client_get_navigation_goal_status(
    lingtu_nav_client_handle handle,
    const char* request_id,
    lingtu_nav_navigation_goal_status* status);

// Versioned retained lifecycle state including task_id. The caller must set
// abi_version and struct_size before calling.
int lingtu_nav_client_get_navigation_goal_status_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    lingtu_nav_navigation_goal_status_v1* status);

// Reads the retained latest lifecycle state for task_id without consuming the
// event queue. Returns 1 when found, 0 when unknown, and -1 on error.
int lingtu_nav_client_get_navigation_task_status_v1(
    lingtu_nav_client_handle handle,
    const char* task_id,
    lingtu_nav_navigation_goal_status_v1* status);

// Copies one latest-only path sample. If point_capacity is too small, returns
// 2, stores the required point_count in header, and retains the sample for a
// retry. Returns 1 after a complete copy, 0 when no sample is pending, and -1
// on error. A null points pointer is valid only when point_capacity is zero.
int lingtu_nav_client_take_global_path(
    lingtu_nav_client_handle handle,
    lingtu_nav_path_header* header,
    lingtu_nav_path_point* points,
    unsigned long long point_capacity);

int lingtu_nav_client_take_local_path(
    lingtu_nav_client_handle handle,
    lingtu_nav_path_header* header,
    lingtu_nav_path_point* points,
    unsigned long long point_capacity);

// Copies one latest-only coherent MapScene. Returns 2 with required counts in
// header when any caller-owned buffer is too small and retains the sample for
// retry. Product-limit violations are rejected before this ABI surface and
// are reported through lingtu_nav_client_read_map_scene_health_v1().
int lingtu_nav_client_take_map_scene_v1(
    lingtu_nav_client_handle handle,
    lingtu_nav_map_scene_header_v1* header,
    const lingtu_nav_map_scene_buffers_v1* buffers);

int lingtu_nav_client_read_map_scene_health_v1(
    lingtu_nav_client_handle handle,
    lingtu_nav_map_scene_health_v1* health);

const char* lingtu_nav_client_last_error(lingtu_nav_client_handle handle);

#ifdef __cplusplus
}
#endif
