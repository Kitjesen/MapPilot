#pragma once

#include <cstdint>
#include <string>

namespace lingtu::message {

struct PostureRequest {
  std::string producer_boot_id;
  std::string client_id;
  std::string request_id;
  std::int32_t action{0};
  std::int32_t phase{0};
  std::string profile_name;
  std::string external_session_id{};
};

struct PostureAck {
  PostureRequest request;
  bool final_result{false};
  bool accepted{false};
  std::string reason;
};

inline bool samePostureRequest(const PostureRequest &left, const PostureRequest &right) {
  return left.producer_boot_id == right.producer_boot_id &&
         left.client_id == right.client_id && left.request_id == right.request_id;
}

}  // namespace lingtu::message
