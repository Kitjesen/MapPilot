// std_msgs shim
#ifndef STD_MSGS_HPP
#define STD_MSGS_HPP
#include <string>
namespace std_msgs {
  struct Header { std::string frame_id; uint32_t seq = 0; };
}
#endif
