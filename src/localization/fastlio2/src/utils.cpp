#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pcl2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int lidar_type, int filter_num, int scan_line, int time_unit, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    // Determine time scale: offset_time stored in curvature field as ms
    float time_scale = 1.0f;  // default: ms
    switch (time_unit) {
        case SEC: time_scale = 1e3f; break;
        case MS:  time_scale = 1.0f; break;
        case US:  time_scale = 1e-3f; break;
        case NS:  time_scale = 1e-6f; break;
    }

    if (lidar_type == VELO16)
    {
        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud->reserve(plsize / filter_num + 1);

        for (int i = 0; i < plsize; i += filter_num)
        {
            if (pl_orig.points[i].ring >= scan_line) continue;

            float x = pl_orig.points[i].x;
            float y = pl_orig.points[i].y;
            float z = pl_orig.points[i].z;
            float dist2 = x * x + y * y + z * z;
            if (dist2 < min_range * min_range || dist2 > max_range * max_range)
                continue;
            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                continue;

            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.normal_x = 0;
            p.normal_y = 0;
            p.normal_z = 0;
            p.intensity = pl_orig.points[i].intensity;
            p.curvature = pl_orig.points[i].time * time_scale;  // offset time in ms
            cloud->push_back(p);
        }
    }
    else if (lidar_type == OUST64)
    {
        // Generic PointCloud2 fallback: use XYZ + intensity, zero offset time
        pcl::PointCloud<pcl::PointXYZI> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud->reserve(plsize / filter_num + 1);

        for (int i = 0; i < plsize; i += filter_num)
        {
            float x = pl_orig.points[i].x;
            float y = pl_orig.points[i].y;
            float z = pl_orig.points[i].z;
            float dist2 = x * x + y * y + z * z;
            if (dist2 < min_range * min_range || dist2 > max_range * max_range)
                continue;
            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                continue;

            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.normal_x = 0;
            p.normal_y = 0;
            p.normal_z = 0;
            p.intensity = pl_orig.points[i].intensity;
            p.curvature = 0.0f;  // no per-point timestamp available
            cloud->push_back(p);
        }
    }

    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
