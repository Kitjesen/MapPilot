#pragma once
#include "commons.h"
#include "scan_context.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <string>
#include <vector>
#include "lingtu_pose_graph_opt.h"

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

using M6D = Eigen::Matrix<double, 6, 6>;

struct PoseGraphPriorFactor
{
    size_t index;
    M3D r;
    V3D t;
    M6D information;
};

struct PoseGraphBetweenFactor
{
    size_t from_id;
    size_t to_id;
    M3D r_offset;
    V3D t_offset;
    M6D information;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    // Search radius default enlarged from 1.0 → 15.0 once Scan Context is on:
    // SC pre-filter rejects spatially-near-but-not-actually-revisited candidates,
    // so we can afford a much larger spatial net to catch real revisits the
    // 1m radius would have missed.
    double loop_search_radius = 15.0;
    double loop_time_tresh = 60.0;
    double loop_score_tresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;

    // Scan Context loop pre-filter (Kim & Kim 2018). Provides indoor global
    // anchoring when GNSS is unavailable. When enabled, the radius search
    // becomes a sanity check on the SC-proposed candidate, and SC's coarse
    // yaw estimate is fed to ICP as an initial guess for faster/safer
    // convergence. See scan_context.h for tuning knobs.
    bool   enable_scan_context = true;
    double sc_distance_threshold = 0.4;  // <0.4 typical for true matches
    double sc_max_range = 20.0;          // m — points beyond clipped from descriptor
    int    sc_num_candidates = 5;        // top-K from SC KdTree
    int    sc_min_history_gap = 30;      // skip last N keyframes
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }
    bool exportPoseGraphFixture(const std::string &path) const;

private:
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::vector<PoseGraphPriorFactor> m_prior_factors;
    std::vector<PoseGraphBetweenFactor> m_between_factors;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;

    // Scan Context place recognition. Fed in addKeyPose(); queried in
    // searchForLoopPairs() as the primary candidate proposer when enabled.
    ScanContext m_sc;
};
