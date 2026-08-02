#pragma once

#include "status/nav_status_publisher.hpp"

namespace lingtu::nav::endpoint {

struct EndpointState;

StatusRuntimeState statusRuntimeStateFromEndpoint(const EndpointState &state);

}  // namespace lingtu::nav::endpoint
