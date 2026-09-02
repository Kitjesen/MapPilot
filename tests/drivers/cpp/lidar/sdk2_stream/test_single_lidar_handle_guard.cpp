#include "single_lidar_handle_guard.hpp"

#include <cassert>

int main() {
  lingtu::drivers::lidar::SingleLidarHandleGuard guard;
  assert(guard.accept(2986453184U));
  assert(guard.accept(2986453184U));
  assert(!guard.accept(123456789U));
  assert(guard.accept(2986453184U));
  return 0;
}
