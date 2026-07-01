#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>      // std::snprintf in launchAutoBBS3D reason builder
#include <exception>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/bbs3d_global_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/cloud_registered";
    std::string odom_topic = "/Odometry";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    std::string static_map_path = "";
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = 0.0;
    double initial_yaw = 0.0;
    double update_hz = 1.0;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        // Global (no-guess) relocalization service: branch-and-bound over
        // the full map. Call this when the robot is "kidnapped" or boots in
        // an unknown pose. On success, sets initial_guess so the tracking
        // timer picks up seamlessly.
        m_bbs3d = std::make_shared<BBS3DGlobalLocalizer>(m_bbs3d_config);
        m_global_reloc_srv = this->create_service<std_srvs::srv::Trigger>(
            "global_relocalize",
            std::bind(&LocalizerNode::globalRelocCB, this,
                      std::placeholders::_1, std::placeholders::_2));
        m_global_reloc_status_srv = this->create_service<std_srvs::srv::Trigger>(
            "global_relocalize_status",
            std::bind(&LocalizerNode::globalRelocStatusCB, this,
                      std::placeholders::_1, std::placeholders::_2));
        if (m_bbs3d->available()) {
            updateBBS3DStatus("backend available; waiting for map");
            RCLCPP_INFO(this->get_logger(), "BBS3D backend available");
        } else {
            updateBBS3DStatus("backend unavailable; build with cpu_bbs3d to enable global no-guess relocalization");
            RCLCPP_WARN(
                this->get_logger(),
                "BBS3D backend unavailable; this binary can only use seeded ICP relocalization");
        }

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        m_quality_pub = this->create_publisher<std_msgs::msg::Float32>("/localization_quality", 10);
        // Externalised lost/lock health for SlamBridge -> Gateway consumption.
        // Multi-frame confirmation gate (see updateAndPublishHealth) prevents
        // single-frame ICP hiccups from spuriously alarming Navigation.
        m_health_pub = this->create_publisher<std_msgs::msg::String>("/slam/localization_health", 10);

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));

        // Auto Load Map if configured
        if (!m_config.static_map_path.empty())
        {
            if (std::filesystem::exists(m_config.static_map_path))
            {
                if (m_localizer->loadMap(m_config.static_map_path))
                {
                    // Also feed the same PCD into bbs3d so /global_relocalize
                    // works out of the box without a second load.
                    {
                        CloudType::Ptr raw(new CloudType);
                        pcl::PCDReader reader;
                        if (reader.read(m_config.static_map_path, *raw) == 0) {
                            updateMapBounds(raw);
                            if (m_bbs3d->available()) {
                                if (m_bbs3d->set_map(raw)) {
                                    updateBBS3DStatus(
                                        "map loaded; global no-guess relocalization ready");
                                } else {
                                    updateBBS3DStatus("map load failed for BBS3D");
                                    RCLCPP_WARN(
                                        this->get_logger(),
                                        "BBS3D: failed to build global localizer map index");
                                }
                            } else {
                                updateBBS3DStatus(
                                    "backend unavailable; map loaded for ICP only");
                            }
                        } else {
                            updateBBS3DStatus("failed to read static map pcd");
                            RCLCPP_WARN(this->get_logger(),
                                "BBS3D: failed to read pcd for global localizer");
                        }
                    }
                    // last_known pose file lives next to the map pcd.
                    m_last_pose_path =
                        std::filesystem::path(m_config.static_map_path)
                            .parent_path() / "last_pose.txt";
                    double lx, ly, lyaw;
                    bool seeded_from_last_pose =
                        readLastPose(lx, ly, lyaw);

                    std::lock_guard<std::mutex> lock(m_state.message_mutex);

                    double use_x = seeded_from_last_pose ? lx : m_config.initial_x;
                    double use_y = seeded_from_last_pose ? ly : m_config.initial_y;
                    double use_yaw = seeded_from_last_pose ? lyaw : m_config.initial_yaw;
                    Eigen::AngleAxisd yaw_angle(use_yaw, Eigen::Vector3d::UnitZ());
                    m_state.initial_guess.setIdentity();
                    m_state.initial_guess.block<3, 3>(0, 0) = yaw_angle.toRotationMatrix().cast<float>();
                    m_state.initial_guess.block<3, 1>(0, 3) = V3F(use_x, use_y, m_config.initial_z);

                    m_state.service_received = true;     // Fake service call
                    m_state.localize_success = false;
                    RCLCPP_INFO(this->get_logger(),
                        "Auto-loaded map from %s with init pose [%.2f, %.2f, %.2f, %.2f] %s",
                        m_config.static_map_path.c_str(),
                        use_x, use_y, m_config.initial_z, use_yaw,
                        seeded_from_last_pose ? "(from last_pose.txt)" : "(from config)");

                    // Schedule an auto-bbs3d on boot if we had no last pose
                    // Fresh startup in unknown location should not need a
                    // human click.  Delay 4 s to let Fast-LIO2 stabilize.
                    if (!seeded_from_last_pose && m_auto_global_relocalize_on_boot) {
                        std::thread([this]() {
                            std::this_thread::sleep_for(std::chrono::seconds(4));
                            if (!m_bbs3d || !m_bbs3d->available() || !m_bbs3d->has_map()) return;
                            if (m_bbs3d_running.exchange(true)) return;
                            if (!m_state.message_received) { m_bbs3d_running = false; return; }
                            CloudType::Ptr scan_copy(new CloudType);
                            { std::lock_guard<std::mutex> lk(m_state.message_mutex);
                              *scan_copy = *m_state.last_cloud; }
                            updateBBS3DStatus("boot auto-relocalize running");
                            RCLCPP_INFO(this->get_logger(),
                                "BBS3D: boot auto-relocalize (%zu pts)", scan_copy->size());
                            auto r = runBBS3D(scan_copy, "boot auto-relocalize");
                            if (r.success) {
                                std::lock_guard<std::mutex> svc(m_state.service_mutex);
                                m_state.initial_guess = r.pose;
                                m_state.service_received = true;
                                m_state.localize_success = false;
                                RCLCPP_INFO(this->get_logger(),
                                    "BBS3D boot OK: t=[%.2f,%.2f,%.2f]",
                                    r.pose(0,3), r.pose(1,3), r.pose(2,3));
                                updateBBS3DResult("boot auto-relocalize", r);
                            } else {
                                RCLCPP_WARN(this->get_logger(),
                                    "BBS3D boot failed: %s", r.message.c_str());
                                updateBBS3DResult("boot auto-relocalize", r);
                            }
                            m_bbs3d_running = false;
                        }).detach();
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s", m_config.static_map_path.c_str());
                }
            } else {
                 RCLCPP_WARN(this->get_logger(), "Map file not found: %s", m_config.static_map_path.c_str());
            }
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        
        // Allow overriding map path via ROS param
        this->declare_parameter("static_map_path", "");
        std::string ros_map_path;
        this->get_parameter("static_map_path", ros_map_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.static_map_path = config["static_map_path"].as<std::string>(); // Manual edit here if needed, or better, declared
        m_config.update_hz = config["update_hz"].as<double>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        this->declare_parameter("rough_score_thresh", m_localizer_config.rough_score_thresh);
        this->declare_parameter("refine_score_thresh", m_localizer_config.refine_score_thresh);
        m_localizer_config.rough_score_thresh =
            this->get_parameter("rough_score_thresh").as_double();
        m_localizer_config.refine_score_thresh =
            this->get_parameter("refine_score_thresh").as_double();
        RCLCPP_INFO(
            this->get_logger(),
            "Localizer ICP score thresholds: rough=%.4f refine=%.4f",
            m_localizer_config.rough_score_thresh,
            m_localizer_config.refine_score_thresh);

        if (!ros_map_path.empty()) {
            m_config.static_map_path = ros_map_path;
            RCLCPP_INFO(this->get_logger(), "Override static map path: %s", m_config.static_map_path.c_str());
        }

        // Initial Pose Parameters
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_yaw", 0.0);
        this->declare_parameter("auto_global_relocalize_on_boot", true);
        this->declare_parameter("auto_global_relocalize_on_lost", true);
        this->declare_parameter("bbs3d_min_level_res", m_bbs3d_config.min_level_res);
        this->declare_parameter("bbs3d_max_level", m_bbs3d_config.max_level);
        this->declare_parameter("bbs3d_num_threads", m_bbs3d_config.num_threads);
        this->declare_parameter("bbs3d_timeout_ms", m_bbs3d_config.timeout_ms);
        
        m_config.initial_x = this->get_parameter("initial_x").as_double();
        m_config.initial_y = this->get_parameter("initial_y").as_double();
        m_config.initial_z = this->get_parameter("initial_z").as_double();
        m_config.initial_yaw = this->get_parameter("initial_yaw").as_double();
        m_auto_global_relocalize_on_boot =
            this->get_parameter("auto_global_relocalize_on_boot").as_bool();
        m_auto_global_relocalize_on_lost =
            this->get_parameter("auto_global_relocalize_on_lost").as_bool();
        m_bbs3d_config.min_level_res =
            this->get_parameter("bbs3d_min_level_res").as_double();
        m_bbs3d_config.max_level =
            static_cast<int>(this->get_parameter("bbs3d_max_level").as_int());
        m_bbs3d_config.num_threads =
            static_cast<int>(this->get_parameter("bbs3d_num_threads").as_int());
        m_bbs3d_config.timeout_ms =
            static_cast<int>(this->get_parameter("bbs3d_timeout_ms").as_int());
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> svc_lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> msg_lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> msg_lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        float fitness = m_localizer->getLastFitnessScore();

        // Log only when tracking state flips (loss / recovery).
        if (result != m_state.localize_success) {
            RCLCPP_INFO(this->get_logger(),
                "ICP tracking %s: fitness=%.4f t=[%.3f,%.3f,%.3f]",
                result ? "LOCKED" : "LOST",
                fitness,
                initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
        }

        // (B) Persist last_known_pose on strong lock.
        // Throttle to ~1 Hz, only save when fitness is genuinely good.
        if (result && fitness > 0.0f && fitness < 0.05f) {
            if (m_last_pose_save_throttle.fetch_add(1) % 10 == 0) {
                double yaw_est = std::atan2(
                    (double)initial_guess(1, 0), (double)initial_guess(0, 0));
                writeLastPose(initial_guess(0, 3), initial_guess(1, 3), yaw_est);
            }
        }

        // (C) Drift watchdog: N consecutive bad frames -> auto BBS3D.
        // Fast-reaction path: looks at raw fitness streak per frame.
        // Independent from the multi-frame health gate (P3) which fires
        // BBS3D from updateAndPublishHealth() on a slower, more
        // conservative LOST decision. Both call launchAutoBBS3D() so the
        // 60s cooldown + m_bbs3d_running mutex are shared.
        if (result && fitness > m_drift_bad_thresh) {
            m_drift_bad_count++;
        } else {
            m_drift_bad_count = 0;
        }
        if (m_drift_bad_count >= m_drift_trigger_frames) {
            char reason[96];
            std::snprintf(reason, sizeof(reason),
                "drift_streak fitness>%.2f for %d frames",
                m_drift_bad_thresh, m_drift_trigger_frames);
            if (launchAutoBBS3D(reason)) {
                m_drift_bad_count = 0;
            }
        }

        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex> svc_lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }

        // Publish ICP fitness score (lower = better, higher = localization degraded)
        {
            std_msgs::msg::Float32 quality_msg;
            quality_msg.data = static_cast<float>(m_localizer->getLastFitnessScore());
            m_quality_pub->publish(quality_msg);
        }

        // Externalised LOCKED/LOST health (multi-frame confirmed)
        updateAndPublishHealth(result, fitness);

        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }

    // HDL-Localization style multi-frame health gate. The internal
    // m_state.localize_success can flip per frame; this layer only flips
    // the externally-published health after N consecutive same-direction
    // frames so navigation does not see ICP single-frame jitter.
    void updateAndPublishHealth(bool result, float fitness)
    {
        if (result)
        {
            m_consec_locked++;
            m_consec_lost = 0;
        }
        else
        {
            m_consec_lost++;
            m_consec_locked = 0;
        }

        std::string desired = m_published_health;
        if (m_consec_lost >= LOST_CONFIRM_FRAMES)
            desired = "LOST";
        else if (m_consec_locked >= RECOVER_CONFIRM_FRAMES) {
            if (m_published_health == "UNKNOWN")
                desired = "LOCKED";
            else if (m_published_health == "LOST")
                desired = "RECOVERED";
            else
                desired = "LOCKED";
        }

        const auto now = std::chrono::steady_clock::now();
        const bool state_changed = desired != m_published_health;
        const bool heartbeat_due =
            now - m_last_health_publish >= m_health_heartbeat_interval;
        const bool publishable = desired != "UNKNOWN" || m_published_health != "UNKNOWN";

        if (publishable && (state_changed || heartbeat_due))
        {
            std_msgs::msg::String msg;
            // R4: extended payload format
            //   "<state>|fitness=<v>|iter=<n>|cov=<v>"
            // Keys are pipe-separated and order-insensitive on the
            // subscriber side. Adding fields is backward-compatible.
            // older parsers ignore unknown keys after the first '|'.
            // iter and cov come from small_gicp's RegistrationResult /
            // Hessian respectively, replacing the single-axis fitness
            // gate the original P3 commit was forced to settle for.
            const int  iter = m_localizer->getLastIterations();
            const double cov = m_localizer->getLastPosCovTrace();
            msg.data = desired
                     + "|fitness=" + std::to_string(fitness)
                     + "|iter="    + std::to_string(iter)
                     + "|cov="     + std::to_string(cov);
            m_health_pub->publish(msg);
            m_last_health_publish = now;
            if (state_changed) {
                RCLCPP_INFO(this->get_logger(),
                    "Localization health -> %s (fitness=%.4f iter=%d cov=%.4f)",
                    desired.c_str(), fitness, iter, cov);
            } else {
                RCLCPP_DEBUG(this->get_logger(),
                    "Localization health heartbeat -> %s (fitness=%.4f iter=%d cov=%.4f)",
                    desired.c_str(), fitness, iter, cov);
            }

            // P7a: on a fresh LOST transition, kick BBS3D for global
            // re-localization. Without this, the multi-frame health
            // gate would only *publish* LOST (so navigation slows /
            // stops) but never *recover*; the recovery path was tied
            // exclusively to the (C) drift watchdog's raw-fitness
            // streak. Now LOST triggers recovery via the same shared
            // helper (60s cooldown + atomic mutex prevent double-fire).
            // We deliberately do not block the publish itself on the
            // BBS3D launch. Navigation must see LOST before recovery
            // begins so it can stop / hold position.
            if (state_changed && desired == "LOST" && m_published_health != "LOST") {
                launchAutoBBS3D("multi-frame LOST (P3 health gate)");
            }

            m_published_health = desired;
            // After a RECOVERED transition the next steady state is LOCKED; do
            // not re-publish each frame, just collapse on next state change.
            if (desired == "RECOVERED") m_published_health = "LOCKED";
        }
    }

    // P7a: shared launch path for auto BBS3D recovery. Returns true if
    // this call actually started a BBS3D worker thread, false if the
    // call was throttled (cooldown still active, BBS3D already running,
    // map not loaded, or no scan received yet). Caller should NOT
    // assume recovery happens; it is best-effort. Inputs:
    //   reason - short tag included in logs to attribute which
    //            trigger fired (drift_streak vs multi-frame LOST etc).
    //
    // The 60-second cooldown is the same value the (C) drift watchdog
    // had: long enough that BBS3D (1-3 s on CPU) cannot self-trigger,
    // short enough that a genuinely lost robot recovers within ~1 min.
    bool launchAutoBBS3D(const std::string& reason)
    {
        if (!m_auto_global_relocalize_on_lost) return false;
        auto _now = std::chrono::steady_clock::now();
        auto _since = std::chrono::duration_cast<std::chrono::seconds>(
            _now - m_last_auto_reloc).count();
        if (_since < 60) return false;
        if (!m_bbs3d || !m_bbs3d->available() || !m_bbs3d->has_map()) return false;
        if (!m_state.message_received) return false;
        if (m_bbs3d_running.exchange(true)) return false;

        m_last_auto_reloc = _now;
        updateBBS3DStatus("auto recovery running: " + reason);
        RCLCPP_WARN(this->get_logger(),
            "Auto BBS3D triggered: %s", reason.c_str());

        CloudType::Ptr scan_copy(new CloudType);
        { std::lock_guard<std::mutex> lk(m_state.message_mutex);
          *scan_copy = *m_state.last_cloud; }
        std::thread([this, scan_copy, reason]() {
            auto r = runBBS3D(scan_copy, "auto recovery");
            if (r.success) {
                std::lock_guard<std::mutex> svc(m_state.service_mutex);
                m_state.initial_guess = r.pose;
                m_state.service_received = true;
                m_state.localize_success = false;
                RCLCPP_INFO(this->get_logger(),
                    "Auto-recovery OK (%s): t=[%.2f,%.2f,%.2f]",
                    reason.c_str(), r.pose(0,3), r.pose(1,3), r.pose(2,3));
                // Reset the LOST counter so the next LOCKED frame can
                // promote to RECOVERED quickly. Without this reset the
                // health gate would still see m_consec_lost >=
                // LOST_CONFIRM_FRAMES until enough good frames pass.
                m_consec_lost = 0;
                updateBBS3DResult("auto recovery", r);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Auto-recovery failed (%s): %s",
                    reason.c_str(), r.message.c_str());
                updateBBS3DResult("auto recovery", r);
            }
            m_bbs3d_running = false;
        }).detach();
        return true;
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> msg_lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }
    
    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            CloudType::Ptr raw(new CloudType);
            pcl::PCDReader reader;
            if (reader.read(pcd_path, *raw) == 0) {
                updateMapBounds(raw);
                m_last_pose_path =
                    std::filesystem::path(pcd_path).parent_path() / "last_pose.txt";
                if (m_bbs3d && m_bbs3d->available()) {
                    if (m_bbs3d->set_map(raw)) {
                        updateBBS3DStatus(
                            "map loaded from relocalize service; global no-guess relocalization ready");
                    } else {
                        updateBBS3DStatus("relocalize service map load failed for BBS3D");
                    }
                } else {
                    updateBBS3DStatus(
                        "map loaded from relocalize service for ICP only; BBS3D unavailable");
                }
            } else {
                updateBBS3DStatus("relocalize service failed to read pcd for BBS3D");
            }
        }
        {
            std::lock_guard<std::mutex> msg_lock(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> svc_lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }

    // One-shot global relocalization: BBS3D over the full map, then hand
    // coarse pose to the tracking timer as initial_guess. No arguments:
    // uses the already-loaded map and the most recent scan.
    //
    // BBS3D CPU takes ~1-3 s; we run it on a detached worker thread so the
    // single-threaded executor's 10ms timer keeps serving tracking. The
    // service returns immediately with "accepted" and the worker posts the
    // result via a status flag you can poll with /slam/global_relocalize_status
    // (or just watch /localization_quality; it drops to a real number when
    // bbs3d finishes and the tracking ICP picks up).
    bool readLastPose(double &x, double &y, double &yaw) {
        if (m_last_pose_path.empty()) return false;
        std::ifstream f(m_last_pose_path);
        if (!f.is_open()) return false;
        // Accept stale last_pose up to 24 h old; older than that,
        // environment likely shifted; safer to do a fresh BBS3D.
        auto ftime = std::filesystem::last_write_time(m_last_pose_path);
        auto now = decltype(ftime)::clock::now();
        auto age_hr = std::chrono::duration_cast<std::chrono::hours>(
            now - ftime).count();
        if (age_hr > 24) {
            RCLCPP_INFO(this->get_logger(),
                "last_pose.txt is %lldh old; ignoring, will run BBS3D",
                (long long)age_hr);
            return false;
        }
        if (!(f >> x >> y >> yaw)) return false;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
            return false;
        }
        if (!lastPoseInsideMapBounds(x, y)) {
            RCLCPP_WARN(this->get_logger(),
                "last_pose.txt pose [%.2f, %.2f] is outside map bbox "
                "[%.2f, %.2f]..[%.2f, %.2f] (+%.1fm margin); ignoring, will run BBS3D",
                x, y, m_map_min_x, m_map_min_y, m_map_max_x, m_map_max_y,
                kLastPoseMapMarginM);
            return false;
        }
        return true;
    }

    void writeLastPose(double x, double y, double yaw) {
        if (m_last_pose_path.empty()) return;
        std::ofstream f(m_last_pose_path);
        if (!f.is_open()) return;
        f << std::fixed << std::setprecision(6)
          << x << " " << y << " " << yaw << "\n";
    }

    void globalRelocCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!m_bbs3d || !m_bbs3d->available()) {
            response->success = false;
            response->message =
                "bbs3d backend unavailable; rebuild with CPU_BBS3D_ROOT or LINGTU_REQUIRE_BBS3D=ON";
            return;
        }
        if (m_bbs3d_running.exchange(true)) {
            response->success = false;
            response->message = "bbs3d already running";
            return;
        }
        if (!m_bbs3d->has_map()) {
            m_bbs3d_running = false;
            response->success = false;
            response->message = "bbs3d map not loaded (static_map_path empty?)";
            return;
        }
        if (!m_state.message_received) {
            m_bbs3d_running = false;
            response->success = false;
            response->message = "no scan received yet";
            return;
        }
        CloudType::Ptr scan_copy(new CloudType);
        {
            std::lock_guard<std::mutex> msg_lock(m_state.message_mutex);
            *scan_copy = *m_state.last_cloud;
        }
        RCLCPP_INFO(this->get_logger(),
            "BBS3D: spawning worker for %zu scan pts", scan_copy->size());
        updateBBS3DStatus("manual global relocalize running");

        std::thread([this, scan_copy]() {
            auto r = runBBS3D(scan_copy, "global relocalize");
            if (!r.success) {
                RCLCPP_WARN(this->get_logger(), "BBS3D: %s", r.message.c_str());
                updateBBS3DResult("manual global relocalize", r);
                m_bbs3d_running = false;
                return;
            }
            {
                std::lock_guard<std::mutex> svc_lock(m_state.service_mutex);
                m_state.initial_guess = r.pose;
                m_state.service_received = true;
                m_state.localize_success = false;
            }
            RCLCPP_INFO(this->get_logger(),
                "BBS3D: ok in %.0f ms score=%.2f t=[%.2f,%.2f,%.2f]",
                r.elapsed_ms, r.score_percentage,
                r.pose(0, 3), r.pose(1, 3), r.pose(2, 3));
            updateBBS3DResult("manual global relocalize", r);
            m_bbs3d_running = false;
        }).detach();

        response->success = true;
        response->message =
            "bbs3d dispatched; poll global_relocalize_status or watch /localization_quality";
    }

    void globalRelocStatusCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = m_bbs3d && m_bbs3d->available() &&
                            m_bbs3d->has_map() && !m_bbs3d_running.load();
        response->message = bbs3dStatusMessage();
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    BBS3DGlobalLocalizer::Config m_bbs3d_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    std::shared_ptr<BBS3DGlobalLocalizer> m_bbs3d;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_global_reloc_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_global_reloc_status_srv;
    std::atomic<bool> m_bbs3d_running{false};
    std::mutex m_bbs3d_mutex;
    std::mutex m_bbs3d_status_mutex;
    std::string m_bbs3d_status = "not initialized";
    bool m_bbs3d_last_success = false;
    double m_bbs3d_last_score = 0.0;
    double m_bbs3d_last_elapsed_ms = 0.0;

    // Last-known pose persistence. Written on successful tracking lock
    // (fitness < 0.05). Read on boot to seed initial_guess; skips a 2-3 s
    // BBS3D scan if the robot hasn't moved. Lives next to the map pcd.
    std::string m_last_pose_path;
    std::atomic<int> m_last_pose_save_throttle{0};
    static constexpr double kLastPoseMapMarginM = 1.0;
    bool m_map_bounds_valid = false;
    double m_map_min_x = 0.0;
    double m_map_min_y = 0.0;
    double m_map_max_x = 0.0;
    double m_map_max_y = 0.0;

    // Drift watchdog: if ICP fitness stays bad for N consecutive frames,
    // auto-trigger BBS3D once. Prevents silent drift turning into lost.
    int m_drift_bad_count = 0;
    int m_drift_trigger_frames = 30;   // ~3 s at 10 Hz
    double m_drift_bad_thresh = 0.30;
    std::chrono::steady_clock::time_point m_last_auto_reloc =
        std::chrono::steady_clock::now() - std::chrono::seconds(60);
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_quality_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_health_pub;

    // Multi-frame confirmation state for /slam/localization_health.
    // HDL-Localization pattern: a single bad ICP frame is not enough to
    // declare LOST; require N consecutive bad frames first, and likewise N
    // consecutive good frames before re-declaring RECOVERED. Otherwise a
    // momentary scan match dip would flap the public health state.
    // P7a: m_consec_lost can now be reset from the BBS3D worker thread
    // (in launchAutoBBS3D's lambda on success), in addition to the
    // per-frame increment/reset on the timer thread. Atomic prevents
    // torn reads on the comparison `m_consec_lost >= LOST_CONFIRM_FRAMES`
    // while the worker is mid-write. m_consec_locked is timer-only but
    // declared atomic for symmetry; relaxed semantics are sufficient.
    std::atomic<int> m_consec_lost{0};
    std::atomic<int> m_consec_locked{0};
    std::string m_published_health = "UNKNOWN";  // last value sent on m_health_pub
    bool m_auto_global_relocalize_on_boot = true;
    bool m_auto_global_relocalize_on_lost = true;
    std::chrono::steady_clock::time_point m_last_health_publish =
        std::chrono::steady_clock::now() - std::chrono::seconds(60);
    std::chrono::steady_clock::duration m_health_heartbeat_interval =
        std::chrono::milliseconds(500);
    static constexpr int LOST_CONFIRM_FRAMES = 5;
    static constexpr int RECOVER_CONFIRM_FRAMES = 3;

    void updateBBS3DStatus(const std::string& status)
    {
        std::lock_guard<std::mutex> lock(m_bbs3d_status_mutex);
        m_bbs3d_status = status;
    }

    void updateBBS3DResult(const std::string& context,
                           const BBS3DGlobalLocalizer::Result& result)
    {
        std::ostringstream status;
        status << context << (result.success ? " succeeded" : " failed")
               << ": " << result.message
               << ", elapsed_ms=" << result.elapsed_ms
               << ", score=" << result.score_percentage;

        std::lock_guard<std::mutex> lock(m_bbs3d_status_mutex);
        m_bbs3d_last_success = result.success;
        m_bbs3d_last_score = result.score_percentage;
        m_bbs3d_last_elapsed_ms = result.elapsed_ms;
        m_bbs3d_status = status.str();
    }

    std::string bbs3dStatusMessage()
    {
        const bool enabled = m_bbs3d && m_bbs3d->available();
        const bool map_loaded = m_bbs3d && m_bbs3d->has_map();
        const bool running = m_bbs3d_running.load();

        std::lock_guard<std::mutex> lock(m_bbs3d_status_mutex);
        std::ostringstream out;
        out << "enabled=" << (enabled ? "true" : "false")
            << " map_loaded=" << (map_loaded ? "true" : "false")
            << " running=" << (running ? "true" : "false")
            << " last_success=" << (m_bbs3d_last_success ? "true" : "false")
            << " last_elapsed_ms=" << m_bbs3d_last_elapsed_ms
            << " last_score=" << m_bbs3d_last_score
            << " status=\"" << m_bbs3d_status << "\"";
        return out.str();
    }

    void updateMapBounds(const CloudType::Ptr& map_cloud)
    {
        if (!map_cloud || map_cloud->empty()) {
            m_map_bounds_valid = false;
            return;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto& p : map_cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
                continue;
            }
            min_x = std::min(min_x, static_cast<double>(p.x));
            min_y = std::min(min_y, static_cast<double>(p.y));
            max_x = std::max(max_x, static_cast<double>(p.x));
            max_y = std::max(max_y, static_cast<double>(p.y));
            found = true;
        }

        m_map_bounds_valid = found;
        if (!found) {
            return;
        }
        m_map_min_x = min_x;
        m_map_min_y = min_y;
        m_map_max_x = max_x;
        m_map_max_y = max_y;
    }

    bool lastPoseInsideMapBounds(double x, double y) const
    {
        if (!m_map_bounds_valid) {
            return true;
        }
        return x >= (m_map_min_x - kLastPoseMapMarginM)
            && x <= (m_map_max_x + kLastPoseMapMarginM)
            && y >= (m_map_min_y - kLastPoseMapMarginM)
            && y <= (m_map_max_y + kLastPoseMapMarginM);
    }

    BBS3DGlobalLocalizer::Result runBBS3D(const CloudType::Ptr& scan_cloud,
                                          const char* context)
    {
        BBS3DGlobalLocalizer::Result r;
        std::lock_guard<std::mutex> lock(m_bbs3d_mutex);
        try {
            if (!m_bbs3d) {
                r.message = "bbs3d object not initialized";
                return r;
            }
            if (!m_bbs3d->available()) {
                r.message = "bbs3d backend unavailable at build time";
                return r;
            }
            if (!m_bbs3d->has_map()) {
                r.message = "bbs3d map not loaded";
                return r;
            }
            return m_bbs3d->localize(scan_cloud);
        } catch (const std::exception& e) {
            r.message = std::string("bbs3d worker exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(),
                "BBS3D %s exception: %s", context, e.what());
        } catch (...) {
            r.message = "bbs3d worker unknown exception";
            RCLCPP_ERROR(this->get_logger(),
                "BBS3D %s unknown exception", context);
        }
        return r;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
