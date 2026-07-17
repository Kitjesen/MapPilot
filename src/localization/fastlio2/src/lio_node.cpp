#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "interface/srv/save_maps.hpp"

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    int lidar_type = AVIA;      // 1=Livox, 2=VLP-16, 3=Ouster
    int scan_line = 4;          // number of scan lines (4 for Mid-360, 16 for VLP-16)
    int timestamp_unit = NS;    // 0=SEC, 1=MS, 2=US, 3=NS
    double acc_scale = 10.0;    // IMU acceleration scale (10.0 for Livox g-units, 1.0 for standard m/s²)
    double livox_scan_window = 0.1; // seconds; 0 disables packet aggregation
    // LI-Init output: positive value means IMU lags LiDAR by this many seconds.
    // Applied IMU-side (subtracted from IMU stamps in imuCB) for parity with
    // upstream FAST-LIO / Point-LIO so published odometry stamps stay on the
    // LiDAR wall clock.
    double time_diff_lidar_to_imu = 0.0;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    double livox_aggregate_start_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox_aggregate_cloud;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        if (m_node_config.lidar_type == AVIA) {
            m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to Livox CustomMsg on [%s]", m_node_config.lidar_topic.c_str());
        } else {
            m_pcl2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_node_config.lidar_topic, 10, std::bind(&LIONode::pcl2CB, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to PointCloud2 (lidar_type=%d) on [%s]", m_node_config.lidar_type, m_node_config.lidar_topic.c_str());
        }

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 10);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
        m_degen_pub = this->create_publisher<std_msgs::msg::Float32>("/slam/degeneracy", 10);
        m_degen_detail_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/slam/degeneracy_detail", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));

        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("save_map", std::bind(&LIONode::saveMapCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    void saveMapCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s", request->file_path.c_str());
        m_builder->saveMap(request->file_path);
        response->success = true;
        response->message = "Map saved successfully";
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        if (config_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "config_path parameter is empty!");
            return;
        }
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_path);
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "FAIL TO LOAD YAML FILE [%s]: %s", config_path.c_str(), e.what());
            return;
        }
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "YAML file loaded but is empty: %s", config_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        if (config["lidar_type"])
            m_node_config.lidar_type = config["lidar_type"].as<int>();
        if (config["scan_line"])
            m_node_config.scan_line = config["scan_line"].as<int>();
        if (config["timestamp_unit"])
            m_node_config.timestamp_unit = config["timestamp_unit"].as<int>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        if (config["degeneracy_max_update_dof"])
            m_builder_config.degeneracy_max_update_dof = config["degeneracy_max_update_dof"].as<int>();
        if (config["degeneracy_max_condition"])
            m_builder_config.degeneracy_max_condition = config["degeneracy_max_condition"].as<double>();
        if (config["max_update_translation_m"])
            m_builder_config.max_update_translation_m = config["max_update_translation_m"].as<double>();
        if (config["max_update_rotation_rad"])
            m_builder_config.max_update_rotation_rad = config["max_update_rotation_rad"].as<double>();
        if (config["reject_nonconverged_update"])
            m_builder_config.reject_nonconverged_update =
                config["reject_nonconverged_update"].as<bool>();
        if (config["reject_degenerate_nonconverged_update"])
            m_builder_config.reject_degenerate_nonconverged_update =
                config["reject_degenerate_nonconverged_update"].as<bool>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        if (config["max_map_points"])
            m_builder_config.max_map_points = config["max_map_points"].as<int>();
        if (config["stationary_thresh"])
            m_builder_config.stationary_thresh = config["stationary_thresh"].as<double>();
        if (config["acc_scale"])
            m_node_config.acc_scale = config["acc_scale"].as<double>();
        if (config["livox_scan_window"])
        {
            m_node_config.livox_scan_window = config["livox_scan_window"].as<double>();
            if (m_node_config.livox_scan_window < 0.0)
                m_node_config.livox_scan_window = 0.0;
        }
        if (config["time_diff_lidar_to_imu"])
        {
            m_node_config.time_diff_lidar_to_imu = config["time_diff_lidar_to_imu"].as<double>();
            if (std::abs(m_node_config.time_diff_lidar_to_imu) > 0.1)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "time_diff_lidar_to_imu = %.6f s exceeds plausible ±0.1s range — likely calibration error",
                    m_node_config.time_diff_lidar_to_imu);
            }
            else if (m_node_config.time_diff_lidar_to_imu != 0.0)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Applying LiDAR→IMU time offset: %.6f s",
                    m_node_config.time_diff_lidar_to_imu);
            }
        }

        // ZUPT parameters (optional, fall back to Config defaults)
        if (config["imu_static_acc_thresh"])
            m_builder_config.imu_static_acc_thresh  = config["imu_static_acc_thresh"].as<double>();
        if (config["imu_static_gyro_thresh"])
            m_builder_config.imu_static_gyro_thresh = config["imu_static_gyro_thresh"].as<double>();
        if (config["zupt_min_static_frames"])
            m_builder_config.zupt_min_static_frames = config["zupt_min_static_frames"].as<int>();
        if (config["zupt_sigma_v"])
            m_builder_config.zupt_sigma_v   = config["zupt_sigma_v"].as<double>();
        if (config["zupt_sigma_pos"])
            m_builder_config.zupt_sigma_pos = config["zupt_sigma_pos"].as<double>();
        if (config["vertical_velocity_constraint_enabled"])
            m_builder_config.vertical_velocity_constraint_enabled =
                config["vertical_velocity_constraint_enabled"].as<bool>();
        if (config["vertical_velocity_sigma_v"])
            m_builder_config.vertical_velocity_sigma_v =
                config["vertical_velocity_sigma_v"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        // Upstream FAST-LIO / Point-LIO subtract `time_diff_lidar_to_imu` from
        // the IMU stamp to align IMU clock onto the LiDAR timeline. We follow
        // the same convention so the published odometry stamps live on the
        // LiDAR wall clock (matches /cloud_registered stamps for TF lookups).
        // Equivalent to LI-Init's documented "SUBTRACT this value from IMU
        // timestamp OR ADD this value to LiDAR timestamp"; we pick the IMU
        // side for parity with upstream.
        double timestamp = Utils::getSec(msg->header) - m_node_config.time_diff_lidar_to_imu;
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        // IMU data NaN guard — corrupted sensor data can poison the entire ESKF
        V3D acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        V3D gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if (!acc.allFinite() || !gyro.allFinite())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "IMU data contains NaN/Inf — dropping frame");
            return;
        }
        m_state_data.imu_buffer.emplace_back(acc * m_node_config.acc_scale, gyro, timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        // LiDAR timestamps stay raw — the time_diff offset is applied on the
        // IMU side in imuCB so output odom stamps remain on the LiDAR clock.
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
            resetLivoxAggregate();
        }
        m_state_data.last_lidar_time = timestamp;
        if (!cloud || cloud->points.empty())
            return;
        if (m_node_config.livox_scan_window <= 0.0)
        {
            m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
            return;
        }
        appendLivoxPacket(timestamp, cloud);
    }
    void pcl2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::pcl2PCL(msg, m_node_config.lidar_type, m_builder_config.lidar_filter_num,
                                               m_node_config.scan_line, m_node_config.timestamp_unit,
                                               m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        // See lidarCB — offset is applied IMU-side, not here.
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    void resetLivoxAggregate()
    {
        m_state_data.livox_aggregate_start_time = -1.0;
        m_state_data.livox_aggregate_cloud.reset();
    }

    void appendLivoxPacket(double timestamp, const CloudType::Ptr &cloud)
    {
        constexpr size_t kMaxAggregatedPoints = 30000;
        const double max_stale_window = std::max(0.3, m_node_config.livox_scan_window * 3.0);
        const bool start_new_scan =
            !m_state_data.livox_aggregate_cloud ||
            m_state_data.livox_aggregate_start_time < 0.0 ||
            timestamp - m_state_data.livox_aggregate_start_time > max_stale_window;

        if (start_new_scan)
        {
            m_state_data.livox_aggregate_start_time = timestamp;
            m_state_data.livox_aggregate_cloud = std::make_shared<CloudType>();
            m_state_data.livox_aggregate_cloud->reserve(4096);
        }

        const double relative_ms = (timestamp - m_state_data.livox_aggregate_start_time) * 1000.0;
        auto &aggregate = *m_state_data.livox_aggregate_cloud;
        aggregate.points.reserve(aggregate.points.size() + cloud->points.size());
        for (const auto &point : cloud->points)
        {
            PointType adjusted = point;
            adjusted.curvature += static_cast<float>(relative_ms);
            aggregate.points.push_back(adjusted);
        }
        aggregate.width = static_cast<uint32_t>(aggregate.points.size());
        aggregate.height = 1;
        aggregate.is_dense = false;

        const double duration = timestamp - m_state_data.livox_aggregate_start_time;
        if (duration >= m_node_config.livox_scan_window || aggregate.points.size() >= kMaxAggregatedPoints)
        {
            m_state_data.lidar_buffer.emplace_back(m_state_data.livox_aggregate_start_time, m_state_data.livox_aggregate_cloud);
            resetLivoxAggregate();
        }
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            if (!m_package.cloud || m_package.cloud->points.empty())
            {
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        q.normalize();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        // Fill pose covariance from IESKF P matrix for downstream drift monitoring.
        // IESKF state: [θ_wi(0:3), t_wi(3:6), ...]; ROS: row-major [x,y,z,roll,pitch,yaw]
        const auto &P = m_kf->P();
        odom.pose.covariance[0]  = P(3, 3);  // x
        odom.pose.covariance[7]  = P(4, 4);  // y
        odom.pose.covariance[14] = P(5, 5);  // z
        odom.pose.covariance[21] = P(0, 0);  // roll
        odom.pose.covariance[28] = P(1, 1);  // pitch
        odom.pose.covariance[35] = P(2, 2);  // yaw

        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        q.normalize();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);

        // 限制 path 历史长度，防止长时间运行后消息体积无限增长
        // 保留最近 5000 个位姿 (~8 分钟 @10Hz)
        constexpr size_t kMaxPathPoses = 5000;
        if (m_state_data.path.poses.size() > kMaxPathPoses)
        {
            m_state_data.path.poses.erase(
                m_state_data.path.poses.begin(),
                m_state_data.path.poses.begin() + (m_state_data.path.poses.size() - kMaxPathPoses));
        }

        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        q.normalize();
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        if (m_first_sync_time == 0.0)
            m_first_sync_time = m_package.cloud_end_time;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();
        warnIfStuck();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);

        publishDegeneracy();
    }

    void publishDegeneracy()
    {
        const auto &degen = m_kf->degeneracy();

        // Publish effective_ratio for external localization health consumers.
        // effective_ratio: 1.0 = all DOFs well-constrained, 0.0 = fully degenerate
        std_msgs::msg::Float32 ratio_msg;
        ratio_msg.data = static_cast<float>(degen.effective_ratio);
        m_degen_pub->publish(ratio_msg);

        // Publish detailed degeneracy info as Float32MultiArray.
        // Layout (older subscribers ignore extra trailing fields):
        // [0]   condition_number
        // [1]   min_eigenvalue
        // [2]   max_eigenvalue
        // [3]   effective_ratio
        // [4]   degenerate_dof_count
        // [5-10] dof_mask (rx,ry,rz,tx,ty,tz)
        // [11]  pos_cov_trace (m²)             — IEKF position covariance trace
        // [12]  iter_num                       — IEKF iterations actually used
        // [13]  converged (1.0 if converged, 0.0 if hit max_iter)
        std_msgs::msg::Float32MultiArray detail_msg;
        detail_msg.data.resize(14);
        detail_msg.data[0]  = static_cast<float>(degen.condition_number);
        detail_msg.data[1]  = static_cast<float>(degen.min_eigenvalue);
        detail_msg.data[2]  = static_cast<float>(degen.max_eigenvalue);
        detail_msg.data[3]  = static_cast<float>(degen.effective_ratio);
        detail_msg.data[4]  = static_cast<float>(degen.degenerate_dof_count);
        for (int d = 0; d < 6; ++d)
            detail_msg.data[5 + d] = static_cast<float>(degen.dof_mask(d));
        detail_msg.data[11] = static_cast<float>(degen.pos_cov_trace);
        detail_msg.data[12] = static_cast<float>(degen.iter_num);
        detail_msg.data[13] = degen.converged ? 1.0f : 0.0f;
        m_degen_detail_pub->publish(detail_msg);

        if (degen.detected)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "DEGENERACY DETECTED: %d/6 DOFs degenerate, cond=%.1f, eff_ratio=%.2f",
                degen.degenerate_dof_count, degen.condition_number, degen.effective_ratio);
        }
    }

    // Surface two silent failure modes that S0.5 already exposes structurally
    // (via /slam/degeneracy_detail) but that have no human-facing log:
    //   - IEKF iteration loop hits m_max_iter without stop_func firing
    //     → likely ill-conditioned Jacobian for the current observation set
    //   - IMU init buffer never fills → wrong topic / dead driver / saturated IMU
    // Throttled hard so a sustained issue does not drown the log.
    void warnIfStuck()
    {
        // IEKF non-convergence — only meaningful once we are past initialisation.
        if (m_builder->status() == BuilderStatus::MAPPING)
        {
            const auto &degen = m_kf->degeneracy();
            if (!degen.converged)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "IEKF did not converge: iter_num=%d/%d, cond=%.2g — "
                    "Jacobian likely ill-conditioned for current scan",
                    degen.iter_num, m_builder_config.ieskf_max_iter,
                    degen.condition_number);
            }
        }

        // IMU init stuck — if we have synced packages but builder is still in
        // IMU_INIT after a few seconds, something is wrong with the IMU stream.
        if (m_builder->status() == BuilderStatus::IMU_INIT && m_first_sync_time > 0.0)
        {
            const double elapsed = m_package.cloud_end_time - m_first_sync_time;
            if (elapsed > 5.0)
            {
                auto progress = m_builder->imu_processor()->initProgress();
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "IMU init stuck for %.1fs: collected %d/%d samples — "
                    "check /livox/imu topic, IMU sample rate, time_diff_lidar_to_imu",
                    elapsed, progress.first, progress.second);
            }
        }
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcl2_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_degen_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_degen_detail_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;

    // Wall-clock (LiDAR-clock) of the first successfully-synced package; used by
    // warnIfStuck() to detect IMU init never completing.
    double m_first_sync_time = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
