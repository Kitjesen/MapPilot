// pcl_conversions shim — no-op conversions
#ifndef PCL_CONVERSIONS_HPP
#define PCL_CONVERSIONS_HPP
#include <pcl/common/io.h>
#include "sensor_msgs/PointCloud2.h"

namespace pcl {
  template<typename T>
  inline void toROSMsg(const pcl::PointCloud<T>&, sensor_msgs::PointCloud2&) {}
  template<typename T>
  inline void fromROSMsg(const sensor_msgs::PointCloud2&, pcl::PointCloud<T>&) {}
}

namespace pcl_conversions {
  inline void toPCL(const sensor_msgs::PointCloud2&, pcl::PCLPointCloud2&) {}
  inline void fromPCL(const pcl::PCLPointCloud2&, sensor_msgs::PointCloud2&) {}
}

#endif
