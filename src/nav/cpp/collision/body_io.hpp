#pragma once

#include "collision/body.hpp"
#include <istream>
#include <memory>

namespace lingtu::nav::collision {

// BODY2 retains source primitives and ground-contact labels. Count-first OBB
// files remain readable so recorded geometry fixtures can still be replayed.
std::shared_ptr<Body> readBody(std::istream& input);

}  // namespace lingtu::nav::collision
