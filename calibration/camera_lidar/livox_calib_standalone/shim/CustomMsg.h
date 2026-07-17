// CustomMsg.h shim — stub for standalone build
#ifndef LIVOX_CUSTOM_MSG_H
#define LIVOX_CUSTOM_MSG_H
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include "CustomPoint.h"

namespace std_msgs { struct Header { std::string frame_id; uint32_t seq = 0; }; }

namespace livox_ros_driver {

template <class ContainerAllocator = std::allocator<void>>
struct CustomMsg_ {
  typedef CustomMsg_<ContainerAllocator> Type;
  std_msgs::Header header;
  uint64_t timebase = 0;
  uint32_t mid = 0;
  uint32_t point_num = 0;
  uint32_t lidar_id = 0;
  std::vector<livox_ros_driver::CustomPoint> points;
  typedef std::shared_ptr<CustomMsg_> Ptr;
  typedef std::shared_ptr<const CustomMsg_> ConstPtr;
};

typedef CustomMsg_<> CustomMsg;
typedef std::shared_ptr<CustomMsg> CustomMsgPtr;
typedef std::shared_ptr<const CustomMsg> CustomMsgConstPtr;

} // namespace livox_ros_driver
#endif
