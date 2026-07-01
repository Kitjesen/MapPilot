#include "simple_pgo.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace
{
constexpr double kMinVariance = 1e-12;

M6D informationFromVariances(const std::array<double, 6> &variances)
{
    M6D information = M6D::Zero();
    for (int i = 0; i < 6; ++i)
    {
        information(i, i) = 1.0 / std::max(variances[static_cast<size_t>(i)], kMinVariance);
    }
    return information;
}

lt_pose_graph_opt_pose3 toLtPose(const M3D &rotation, const V3D &translation)
{
    Eigen::Quaterniond q(rotation);
    q.normalize();
    lt_pose_graph_opt_pose3 pose{};
    pose.t_xyz[0] = translation.x();
    pose.t_xyz[1] = translation.y();
    pose.t_xyz[2] = translation.z();
    pose.q_wxyz[0] = q.w();
    pose.q_wxyz[1] = q.x();
    pose.q_wxyz[2] = q.y();
    pose.q_wxyz[3] = q.z();
    return pose;
}

void fromLtPose(const lt_pose_graph_opt_pose3 &pose, M3D &rotation, V3D &translation)
{
    Eigen::Quaterniond q(pose.q_wxyz[0], pose.q_wxyz[1], pose.q_wxyz[2], pose.q_wxyz[3]);
    q.normalize();
    rotation = q.toRotationMatrix();
    translation = V3D(pose.t_xyz[0], pose.t_xyz[1], pose.t_xyz[2]);
}

void fillInformationUpper(const M6D &information, double (&upper)[21])
{
    size_t out = 0;
    for (int row = 0; row < 6; ++row)
    {
        for (int col = row; col < 6; ++col)
        {
            upper[out++] = 0.5 * (information(row, col) + information(col, row));
        }
    }
}

lt_pose_graph_opt_config defaultPoseGraphConfig()
{
    lt_pose_graph_opt_config config{};
    config.struct_size = sizeof(lt_pose_graph_opt_config);
    config.version = LT_POSE_GRAPH_OPT_CONFIG_VERSION;
    config.max_iterations = 30;
    config.method = 1;
    config.fixed_pose_index = 0;
    config.auto_anchor = 1;
    config.initial_lambda = 1e-3;
    config.tolerance = 1e-9;
    config.numeric_epsilon = 1e-6;
    return config;
}

void writeDoubleArray(std::ostream &out, const double *values, size_t count)
{
    out << "[";
    for (size_t i = 0; i < count; ++i)
    {
        if (i > 0)
            out << ",";
        out << values[i];
    }
    out << "]";
}

void writeVector3Json(std::ostream &out, const V3D &vector)
{
    const double values[3] = {vector.x(), vector.y(), vector.z()};
    writeDoubleArray(out, values, 3);
}

void writeMatrix3Json(std::ostream &out, const M3D &matrix)
{
    out << "[";
    for (int row = 0; row < 3; ++row)
    {
        if (row > 0)
            out << ",";
        const double values[3] = {matrix(row, 0), matrix(row, 1), matrix(row, 2)};
        writeDoubleArray(out, values, 3);
    }
    out << "]";
}

void writePoseJson(std::ostream &out, const lt_pose_graph_opt_pose3 &pose)
{
    out << "{\"t_xyz\":";
    writeDoubleArray(out, pose.t_xyz, 3);
    out << ",\"q_wxyz\":";
    writeDoubleArray(out, pose.q_wxyz, 4);
    out << "}";
}

void writePoseJson(std::ostream &out, const M3D &rotation, const V3D &translation)
{
    writePoseJson(out, toLtPose(rotation, translation));
}

void writeInformationUpperJson(std::ostream &out, const M6D &information)
{
    double upper[21]{};
    fillInformationUpper(information, upper);
    writeDoubleArray(out, upper, 21);
}

void writePoseGraphConfigJson(std::ostream &out, const lt_pose_graph_opt_config &config)
{
    out << "{"
        << "\"max_iterations\":" << config.max_iterations << ","
        << "\"method\":" << config.method << ","
        << "\"fixed_pose_index\":" << config.fixed_pose_index << ","
        << "\"auto_anchor\":" << (config.auto_anchor != 0 ? "true" : "false") << ","
        << "\"initial_lambda\":" << config.initial_lambda << ","
        << "\"tolerance\":" << config.tolerance << ","
        << "\"numeric_epsilon\":" << config.numeric_epsilon
        << "}";
}

void writePriorJson(std::ostream &out, const PoseGraphPriorFactor &factor)
{
    out << "{\"index\":" << factor.index << ",\"pose\":";
    writePoseJson(out, factor.r, factor.t);
    out << ",\"information_upper\":";
    writeInformationUpperJson(out, factor.information);
    out << "}";
}

void writeBetweenJson(
    std::ostream &out,
    size_t from_index,
    size_t to_index,
    const M3D &rotation,
    const V3D &translation,
    const M6D &information)
{
    out << "{\"from_index\":" << from_index << ",\"to_index\":" << to_index << ",\"pose_from_to\":";
    writePoseJson(out, rotation, translation);
    out << ",\"information_upper\":";
    writeInformationUpperJson(out, information);
    out << "}";
}
}

SimplePGO::SimplePGO(const Config &config) : m_config(config),
    m_sc(ScanContext::Config{
        config.sc_max_range,
        /*ring_key_threshold=*/0.4,
        config.sc_distance_threshold,
        config.sc_num_candidates,
        config.sc_min_history_gap,
    })
{
    m_prior_factors.clear();
    m_between_factors.clear();
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    if (idx == 0)
    {
        // 添加先验约束
        PoseGraphPriorFactor prior;
        prior.index = idx;
        prior.r = init_r;
        prior.t = init_t;
        prior.information = informationFromVariances({1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12});
        m_prior_factors.push_back(prior);
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        PoseGraphBetweenFactor between;
        between.from_id = idx - 1;
        between.to_id = idx;
        between.r_offset = r_between;
        between.t_offset = t_between;
        between.information = informationFromVariances({1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6});
        m_between_factors.push_back(between);
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);

    // Feed Scan Context with the body-frame cloud so descriptor indices stay
    // 1:1 with m_key_poses indices. Only when SC is enabled — the descriptor
    // build is non-trivial cost (~1ms per scan).
    if (m_config.enable_scan_context && cloud_with_pose.cloud)
        m_sc.add(cloud_with_pose.cloud);

    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();

    // ── Candidate proposal ────────────────────────────────────────────────
    // Two paths:
    //   (A) Scan Context primary (default for indoor). SC scores by descriptor
    //       similarity over the entire history, then we sanity-check the
    //       chosen candidate is within `loop_search_radius` so we do not add
    //       a wildly-wrong loop on the rare false-positive descriptor match.
    //   (B) Pure radius+time fallback (legacy). Used when SC is disabled.
    int loop_idx = -1;
    float sc_yaw = 0.0f;

    if (m_config.enable_scan_context && m_sc.size() == m_key_poses.size())
    {
        auto sc_result = m_sc.query(cur_idx);
        loop_idx = sc_result.first;
        sc_yaw = sc_result.second;
        if (loop_idx >= 0)
        {
            // Time gap check — same as legacy (avoid matching recent frames).
            if (std::abs(last_item.time - m_key_poses[loop_idx].time) <= m_config.loop_time_tresh)
                loop_idx = -1;
        }
        if (loop_idx >= 0)
        {
            // Spatial sanity: SC matches that are physically far away are
            // almost always bogus (similar geometry in unrelated places —
            // common in long corridors / repeated rooms). Reject.
            const V3D dt = m_key_poses[loop_idx].t_global - last_item.t_global;
            if (dt.norm() > m_config.loop_search_radius)
                loop_idx = -1;
        }
    }
    else
    {
        // Legacy radius+time path (preserved for outdoor / debug fallback).
        pcl::PointXYZ last_pose_pt;
        last_pose_pt.x = last_item.t_global(0);
        last_pose_pt.y = last_item.t_global(1);
        last_pose_pt.z = last_item.t_global(2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < m_key_poses.size() - 1; i++)
        {
            pcl::PointXYZ pt;
            pt.x = m_key_poses[i].t_global(0);
            pt.y = m_key_poses[i].t_global(1);
            pt.z = m_key_poses[i].t_global(2);
            key_poses_cloud->push_back(pt);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);
        std::vector<int> ids;
        std::vector<float> sqdists;
        int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
        if (neighbors == 0)
            return;
        for (size_t i = 0; i < ids.size(); i++)
        {
            int idx = ids[i];
            if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
            {
                loop_idx = idx;
                break;
            }
        }
    }

    if (loop_idx == -1)
        return;

    // ── ICP refinement ────────────────────────────────────────────────────
    // Same submap approach as before. With SC enabled we feed the coarse yaw
    // as ICP's initial guess so it converges within 5-10 iterations even with
    // significant orientation mismatch (otherwise ICP can stall in a yaw-flip
    // local minimum).
    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
    CloudType::Ptr align_cloud(new CloudType);

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);

    if (m_config.enable_scan_context && std::abs(sc_yaw) > 1e-3f)
    {
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        const float c = std::cos(sc_yaw);
        const float s = std::sin(sc_yaw);
        init_guess(0, 0) = c; init_guess(0, 1) = -s;
        init_guess(1, 0) = s; init_guess(1, 1) =  c;
        m_icp.align(*align_cloud, init_guess);
    }
    else
    {
        m_icp.align(*align_cloud);
    }

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    M4F loop_transform = m_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    if (!has_loop)
        return;
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            PoseGraphBetweenFactor between;
            between.from_id = pair.target_id;
            between.to_id = pair.source_id;
            between.r_offset = pair.r_offset;
            between.t_offset = pair.t_offset;
            const double variance = std::max(pair.score, kMinVariance);
            between.information = informationFromVariances({variance, variance, variance, variance, variance, variance});
            m_between_factors.push_back(between);
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    if (m_key_poses.empty())
        return;

    std::vector<lt_pose_graph_opt_pose3> poses;
    poses.reserve(m_key_poses.size());
    for (const KeyPoseWithCloud &pose : m_key_poses)
    {
        poses.push_back(toLtPose(pose.r_global, pose.t_global));
    }

    std::vector<lt_pose_graph_opt_prior3> priors;
    priors.reserve(m_prior_factors.size());
    for (const PoseGraphPriorFactor &factor : m_prior_factors)
    {
        lt_pose_graph_opt_prior3 prior{};
        prior.index = static_cast<uint32_t>(factor.index);
        prior.pose = toLtPose(factor.r, factor.t);
        fillInformationUpper(factor.information, prior.information_upper);
        priors.push_back(prior);
    }

    std::vector<lt_pose_graph_opt_between3> betweens;
    betweens.reserve(m_between_factors.size());
    for (const PoseGraphBetweenFactor &factor : m_between_factors)
    {
        lt_pose_graph_opt_between3 between{};
        between.from_index = static_cast<uint32_t>(factor.from_id);
        between.to_index = static_cast<uint32_t>(factor.to_id);
        between.pose_from_to = toLtPose(factor.r_offset, factor.t_offset);
        fillInformationUpper(factor.information, between.information_upper);
        betweens.push_back(between);
    }

    lt_pose_graph_opt_config config = defaultPoseGraphConfig();
    lt_pose_graph_opt_handle *handle = lt_pose_graph_opt_create(&config);
    if (handle == nullptr)
    {
        std::cerr << "pose_graph_opt create failed" << std::endl;
        return;
    }

    lt_pose_graph_opt_report report{};
    lt_pose_graph_opt_result status = lt_pose_graph_opt_process_se3(
        handle,
        poses.data(),
        static_cast<uint64_t>(poses.size()),
        priors.empty() ? nullptr : priors.data(),
        static_cast<uint64_t>(priors.size()),
        betweens.empty() ? nullptr : betweens.data(),
        static_cast<uint64_t>(betweens.size()),
        &report);
    if (status != LT_POSE_GRAPH_OPT_OK)
    {
        std::cerr << "pose_graph_opt process_se3 failed: " << status << std::endl;
        lt_pose_graph_opt_destroy(handle);
        return;
    }

    uint64_t written = 0;
    status = lt_pose_graph_opt_copy_result_poses(handle, poses.data(), static_cast<uint64_t>(poses.size()), &written);
    lt_pose_graph_opt_destroy(handle);
    if (status != LT_POSE_GRAPH_OPT_OK || written != poses.size())
    {
        std::cerr << "pose_graph_opt copy_result_poses failed: " << status << std::endl;
        return;
    }

    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        fromLtPose(poses[i], m_key_poses[i].r_global, m_key_poses[i].t_global);
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}

bool SimplePGO::exportPoseGraphFixture(const std::string &path) const
{
    if (m_key_poses.empty())
        return false;

    std::ofstream out(path);
    if (!out.is_open())
        return false;

    const lt_pose_graph_opt_config config = defaultPoseGraphConfig();
    const size_t cached_loop_count = m_cache_pairs.size();
    const size_t between_count = m_between_factors.size() + cached_loop_count;
    const size_t factor_count = m_prior_factors.size() + between_count;

    out << std::setprecision(17);
    out << "{";
    out << "\"schema\":\"lingtu.pose_graph_opt.fixture.v1\",";
    out << "\"schema_version\":1,";
    out << "\"case\":\"pgo_loop\",";
    out << "\"coverage\":\"pgo_loop_prior_between\",";
    out << "\"pose_format\":\"t_xyz_q_wxyz\",";
    out << "\"tangent_order\":[\"rx\",\"ry\",\"rz\",\"tx\",\"ty\",\"tz\"],";
    out << "\"information_upper_order\":\"row_major_upper_6x6\",";
    out << "\"information_packing\":\"upper_triangle_row_major_6x6\",";
    out << "\"config\":";
    writePoseGraphConfigJson(out, config);

    out << ",\"poses\":[";
    for (size_t i = 0; i < m_key_poses.size(); ++i)
    {
        if (i > 0)
            out << ",";
        writePoseJson(out, m_key_poses[i].r_global, m_key_poses[i].t_global);
    }
    out << "]";

    out << ",\"priors\":[";
    for (size_t i = 0; i < m_prior_factors.size(); ++i)
    {
        if (i > 0)
            out << ",";
        writePriorJson(out, m_prior_factors[i]);
    }
    out << "]";

    out << ",\"betweens\":[";
    bool wrote_between = false;
    for (const PoseGraphBetweenFactor &factor : m_between_factors)
    {
        if (wrote_between)
            out << ",";
        writeBetweenJson(out, factor.from_id, factor.to_id, factor.r_offset, factor.t_offset, factor.information);
        wrote_between = true;
    }
    for (const LoopPair &pair : m_cache_pairs)
    {
        if (wrote_between)
            out << ",";
        const double variance = std::max(pair.score, kMinVariance);
        const M6D information = informationFromVariances({variance, variance, variance, variance, variance, variance});
        writeBetweenJson(out, pair.target_id, pair.source_id, pair.r_offset, pair.t_offset, information);
        wrote_between = true;
    }
    out << "]";

    out << ",\"expected\":{"
        << "\"status\":\"ok\","
        << "\"pose_count\":" << m_key_poses.size() << ","
        << "\"factor_count\":" << factor_count << ","
        << "\"iterations_max\":" << config.max_iterations << ","
        << "\"accepted_steps_min\":0"
        << "}";
    out << ",\"baseline_tolerances\":{"
        << "\"final_cost_delta_abs_max\":1e-06,"
        << "\"final_residual_rms_delta_abs_max\":1e-06,"
        << "\"final_residual_max_delta_abs_max\":1e-06"
        << "}";
    out << ",\"metadata\":{"
        << "\"generator\":\"SimplePGO::exportPoseGraphFixture\","
        << "\"source\":\"src/localization/pgo\","
        << "\"key_pose_count\":" << m_key_poses.size() << ","
        << "\"prior_count\":" << m_prior_factors.size() << ","
        << "\"between_count\":" << between_count << ","
        << "\"cached_loop_count\":" << cached_loop_count << ","
        << "\"history_pair_count\":" << m_history_pairs.size() << ","
        << "\"offset\":{"
        << "\"offsetR\":";
    writeMatrix3Json(out, m_r_offset);
    out << ",\"offsetT\":";
    writeVector3Json(out, m_t_offset);
    out << "}}";
    out << "}";
    return out.good();
}
