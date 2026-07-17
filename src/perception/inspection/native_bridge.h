#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  LINGTU_INSPECTION_EVIDENCE_ID_CAP = 128,
  LINGTU_INSPECTION_EVIDENCE_ACTION_CAP = 64,
  LINGTU_INSPECTION_EVIDENCE_REASON_CAP = 256,
};

typedef struct LingtuInspectionEvidenceRequest {
  double requested_at_s;
  char request_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  char run_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  char route_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  uint64_t route_revision;
  char map_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  int64_t map_version;
  uint32_t point_index;
  char point_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  char action[LINGTU_INSPECTION_EVIDENCE_ACTION_CAP];
  double deadline_s;
} LingtuInspectionEvidenceRequest;

typedef struct LingtuInspectionEvidenceResult {
  double result_at_s;
  char request_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  char evidence_id[LINGTU_INSPECTION_EVIDENCE_ID_CAP];
  int32_t persisted;
  char reason[LINGTU_INSPECTION_EVIDENCE_REASON_CAP];
  char analysis_verdict[LINGTU_INSPECTION_EVIDENCE_REASON_CAP];
} LingtuInspectionEvidenceResult;

void* lingtu_inspection_evidence_bridge_create(int32_t domain_id);
void lingtu_inspection_evidence_bridge_destroy(void* handle);

// Non-blocking. Returns 1 when a request was copied into output, 0 when no
// valid request is available now, and -1 on DDS or ABI validation errors.
int32_t lingtu_inspection_evidence_bridge_take_request(
    void* handle,
    LingtuInspectionEvidenceRequest* output);

// Returns 0 on publish success and -1 on DDS or ABI validation errors.
int32_t lingtu_inspection_evidence_bridge_write_result(
    void* handle,
    const LingtuInspectionEvidenceResult* result);

const char* lingtu_inspection_evidence_bridge_last_error(void* handle);

#ifdef __cplusplus
}
#endif
