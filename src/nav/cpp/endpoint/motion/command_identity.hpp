#pragma once

#include <string>

#include "lingtu_slam.h"
#include "motion/command_ingress_controller.hpp"

namespace lingtu::nav::endpoint {

inline std::string stringValue(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

inline CommandIngressRequest
commandIngressRequestFromDds(const lingtu_dds_NavigationCommandRequest &message) {
  CommandIngressRequest request;
  request.client_id = stringValue(message.client_id);
  request.request_id = stringValue(message.request_id);
  request.raw_kind = message.kind;

  auto &payload = request.payload;
  payload.frame_id = stringValue(message.header.frame_id);
  payload.goal = {
      message.goal.position.x,    message.goal.position.y,    message.goal.position.z,
      message.goal.orientation.x, message.goal.orientation.y, message.goal.orientation.z,
      message.goal.orientation.w,
  };
  payload.velocity = {
      message.velocity.linear.x,  message.velocity.linear.y,  message.velocity.linear.z,
      message.velocity.angular.x, message.velocity.angular.y, message.velocity.angular.z,
  };
  payload.reason = stringValue(message.reason);
  return request;
}

}  // namespace lingtu::nav::endpoint
