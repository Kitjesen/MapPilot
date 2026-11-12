#pragma once

#include <deque>
#include <optional>

#include "body.hpp"
#include "message/protocol/posture.hpp"

namespace lingtu::driver {

struct PostureReply {
  message::PostureAck ack;
  std::optional<Result> result;
  bool new_request{false};
};

class PostureControl {
 public:
  PostureReply request(const message::PostureRequest &request, Body &body,
                       const std::string &zero_producer, bool fresh_zero, bool profile_busy);
  bool holdsMotion() const noexcept { return hold_; }
  void observeBody(const BodyState &state) noexcept;

 private:
  std::optional<message::PostureRequest> preparing_;
  std::deque<message::PostureAck> completed_;
  bool hold_{false};
  bool resume_when_standing_{false};
  Clock::time_point accepted_at_{};
};

}  // namespace lingtu::driver
