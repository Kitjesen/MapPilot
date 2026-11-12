#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t status;
  int32_t error;
  uint8_t* data;
  size_t len;
} LingtuSdkReply;

typedef struct {
  const uint8_t* event;
  size_t event_len;
  const uint8_t* id;
  size_t id_len;
  const uint8_t* data;
  size_t data_len;
} LingtuSdkEvent;

typedef int32_t (*LingtuSdkReceive)(const LingtuSdkEvent*, void*);

LingtuSdkReply lingtu_sdk_client_create(const char*, const char*, uint64_t, void**);
LingtuSdkReply lingtu_sdk_client_create_with_media_token(const char*, const char*, uint64_t, const char*, void**);
void lingtu_sdk_client_free(void*);
void lingtu_sdk_buffer_free(uint8_t*, size_t);
LingtuSdkReply lingtu_sdk_request(const void*, const char*, const char*, const char*);
LingtuSdkReply lingtu_sdk_command(const void*, const char*, const char*, const char*);
LingtuSdkReply lingtu_sdk_bytes(const void*, const char*);
LingtuSdkReply lingtu_sdk_download(const void*, const char*, const char*);
LingtuSdkReply lingtu_sdk_merge_json(const char*, const char*);
LingtuSdkReply lingtu_sdk_goal(const void*, double, double, double, const char*, int32_t);
LingtuSdkReply lingtu_sdk_position(const void*);
LingtuSdkReply lingtu_sdk_wait_navigation(const void*, const char*, uint64_t, uint64_t);
LingtuSdkReply lingtu_sdk_save_and_wait(const void*, const char*, uint64_t, uint64_t, int32_t);
LingtuSdkReply lingtu_sdk_events(const void*, const char*, const char*, LingtuSdkReceive, void*);
LingtuSdkReply lingtu_sdk_websocket_open(const void*, const char*, const char*, void**);
LingtuSdkReply lingtu_sdk_websocket_receive(void*, int32_t*);
LingtuSdkReply lingtu_sdk_websocket_close(void*);
void lingtu_sdk_websocket_free(void*);
LingtuSdkReply lingtu_sdk_teleop_open(const void*, const char*, void**);
LingtuSdkReply lingtu_sdk_teleop_move(void*, double, double, double, int32_t, const char*);
LingtuSdkReply lingtu_sdk_teleop_hold(void*, const char*);
LingtuSdkReply lingtu_sdk_teleop_close(void*);
void lingtu_sdk_teleop_free(void*);

#ifdef __cplusplus
}
#endif
