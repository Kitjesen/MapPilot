#include "hba.h"

#include "lingtu_pose_graph_opt.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
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

M6D symmetricInformation(const M6D &information)
{
    return 0.5 * (information + information.transpose());
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

void writeBetweenJson(
    std::ostream &out,
    size_t from_index,
    size_t to_index,
    const Pose &relative_pose,
    const M6D &information)
{
    out << "{\"from_index\":" << from_index << ",\"to_index\":" << to_index << ",\"pose_from_to\":";
    writePoseJson(out, relative_pose.r, relative_pose.t);
    out << ",\"information_upper\":";
    writeInformationUpperJson(out, information);
    out << "}";
}
}

HBA::HBA(const HBAConfig config) : m_config(config)
{
    m_poses.clear();
    m_clouds.clear();
    m_lbas.clear();
}
void HBA::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_clouds.push_back(cloud);
    m_poses.push_back(pose);
}
void HBA::optimize()
{
    m_levels = calcLevels();

    // for (size_t i = 0; i < m_config.hba_iter; i++)
    // {
    Vec<Vec<BLAM>>().swap(m_lbas);
    m_lbas.resize(m_levels, Vec<BLAM>());
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds = m_clouds;
    Vec<Pose> poses = m_poses;
    // 设置初始值

    std::cout << "INIT POSE ESTIMATE" << std::endl;

    for (int level = 0; level < m_levels; level++)
    {
        constructHierarchy(clouds, poses, level);
        std::cout << "LBA OPTIMIZE LEVEL: " << level << std::endl;

        updateCloudsAndPose(clouds, poses, level);
    }
    Vec<std::pair<size_t, size_t>> between_factors_id;
    Vec<Pose> between_factors_pose;
    Vec<M6D> between_factors_info;
    getAllFactors(between_factors_id, between_factors_pose, between_factors_info);
    // 添加二元因子

    std::cout << "CONSTRUCT BETWEEN FACTORS" << std::endl;
    std::vector<lt_pose_graph_opt_pose3> opt_poses;
    opt_poses.reserve(m_poses.size());
    for (const Pose &pose : m_poses)
    {
        opt_poses.push_back(toLtPose(pose.r, pose.t));
    }

    std::vector<lt_pose_graph_opt_between3> opt_betweens;
    opt_betweens.reserve(between_factors_id.size());
    for (size_t j = 0; j < between_factors_id.size(); j++)
    {
        lt_pose_graph_opt_between3 between{};
        between.from_index = static_cast<uint32_t>(between_factors_id[j].first);
        between.to_index = static_cast<uint32_t>(between_factors_id[j].second);
        between.pose_from_to = toLtPose(between_factors_pose[j].r, between_factors_pose[j].t);
        M6D information = symmetricInformation(between_factors_info[j]);
        fillInformationUpper(information, between.information_upper);
        opt_betweens.push_back(between);
    }
    // 优化
    std::cout << "LM OPTIMIZE " << std::endl;
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
        opt_poses.data(),
        static_cast<uint64_t>(opt_poses.size()),
        nullptr,
        0,
        opt_betweens.empty() ? nullptr : opt_betweens.data(),
        static_cast<uint64_t>(opt_betweens.size()),
        &report);
    if (status != LT_POSE_GRAPH_OPT_OK)
    {
        std::cerr << "pose_graph_opt process_se3 failed: " << status << std::endl;
        lt_pose_graph_opt_destroy(handle);
        return;
    }

    uint64_t written = 0;
    status = lt_pose_graph_opt_copy_result_poses(handle, opt_poses.data(), static_cast<uint64_t>(opt_poses.size()), &written);
    lt_pose_graph_opt_destroy(handle);
    if (status != LT_POSE_GRAPH_OPT_OK || written != opt_poses.size())
    {
        std::cerr << "pose_graph_opt copy_result_poses failed: " << status << std::endl;
        return;
    }
    // 更新位姿
    std::cout << "UPDATE POSE ESTIMATE" << std::endl;
    for (size_t j = 0; j < m_poses.size(); j++)
    {
        fromLtPose(opt_poses[j], m_poses[j].r, m_poses[j].t);
    }
    // }
}
void HBA::updateCloudsAndPose(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level)
{
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(clouds);
    Vec<Pose>().swap(poses);
    for (BLAM &lba : m_lbas[level])
    {
        poses.push_back(lba.poses()[0]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = lba.getLocalCloud();
        if (m_config.down_sample > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
            voxel_grid.setLeafSize(m_config.down_sample, m_config.down_sample, m_config.down_sample);
            voxel_grid.setInputCloud(cloud);
            voxel_grid.filter(*cloud);
        }
        clouds.push_back(cloud);
    }
}
void HBA::constructHierarchy(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level)
{
    BLAMConfig config;
    config.voxel_size = m_config.voxel_size;
    config.max_iter = m_config.ba_max_iter;
    config.max_layer = m_config.max_layer;
    config.min_point_num = m_config.min_point_num;
    for (size_t i = 0; i < clouds.size(); i += m_config.stride)
    {
        m_lbas[level].emplace_back(config);
        bool drop_last = false;
        for (int j = 0; j < m_config.window_size; j++)
        {
            size_t idx = i + j;
            if (i + j >= clouds.size())
            {
                drop_last = true;
                break;
            }
            m_lbas[level].back().insert(clouds[idx], poses[idx]);
        }
        // std::cout << "level: " << level << " window: " << i << std::endl;
        m_lbas[level].back().optimize();
        if (drop_last)
            break;
    }
}
void HBA::getAllFactors(Vec<std::pair<size_t, size_t>> &ids, Vec<Pose> &poses, Vec<M6D> &infos)
{
    for (size_t level_idx = 0; level_idx < static_cast<size_t>(m_levels); level_idx++)
    {
        for (size_t stride_idx = 0; stride_idx < m_lbas[level_idx].size(); stride_idx++)
        {
            BLAM &blam = m_lbas[level_idx][stride_idx];
            for (size_t pose_idx = 0; pose_idx + 1 < blam.poses().size(); pose_idx++)
            {
                size_t from = pose_idx + stride_idx * m_config.stride;
                size_t to = pose_idx + 1 + stride_idx * m_config.stride;
                from = from * std::pow(m_config.stride, level_idx);
                to = to * std::pow(m_config.stride, level_idx);
                ids.emplace_back(std::make_pair(from, to));
                Pose &pose_from = blam.poses()[pose_idx];
                Pose &pose_to = blam.poses()[pose_idx + 1];
                Pose pose_fc;
                pose_fc.r = pose_from.r.transpose() * pose_to.r;
                pose_fc.t = pose_from.r.transpose() * (pose_to.t - pose_from.t);
                // std::cout << "level: " << level_idx << " from: " << from << " to: " << to << std::endl;
                poses.push_back(pose_fc);
                infos.push_back(blam.H().block<6, 6>(6 * pose_idx, 6 * (pose_idx + 1)));
            }
        }
    }
}
int HBA::calcLevels()
{

    assert(m_poses.size() > 0 && m_config.window_size > 0 && m_config.stride > 0);
    return static_cast<int>(0.5 * std::log(3.0 * std::pow(static_cast<double>(m_poses.size()), 2) * (std::pow(m_config.stride, 3) - m_config.stride) / std::pow(static_cast<double>(m_config.window_size), 3)) / std::log(m_config.stride));
}
pcl::PointCloud<pcl::PointXYZI>::Ptr HBA::getMapPoints()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        Pose &pose = m_poses[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_clouds[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud, *transformed, pose.t, Eigen::Quaterniond(pose.r));
        *ret += *transformed;
    }
    return ret;
}
void HBA::writePoses(const std::string &path)
{
    std::ofstream txt_file(path);
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        Pose &pose = m_poses[i];
        V3D t = pose.t;
        Eigen::Quaterniond q(pose.r);
        txt_file << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    }
    txt_file.close();
}

bool HBA::exportPoseGraphFixture(const std::string &path)
{
    if (m_poses.empty())
        return false;
    if (m_lbas.size() < static_cast<size_t>(m_levels))
        return false;

    std::ofstream out(path);
    if (!out.is_open())
        return false;

    Vec<std::pair<size_t, size_t>> between_factors_id;
    Vec<Pose> between_factors_pose;
    Vec<M6D> between_factors_info;
    getAllFactors(between_factors_id, between_factors_pose, between_factors_info);

    const lt_pose_graph_opt_config config = defaultPoseGraphConfig();
    out << std::setprecision(17);
    out << "{";
    out << "\"schema\":\"lingtu.pose_graph_opt.fixture.v1\",";
    out << "\"schema_version\":1,";
    out << "\"case\":\"hba_full_info\",";
    out << "\"coverage\":\"hba_full_info_between_only\",";
    out << "\"pose_format\":\"t_xyz_q_wxyz\",";
    out << "\"tangent_order\":[\"rx\",\"ry\",\"rz\",\"tx\",\"ty\",\"tz\"],";
    out << "\"information_upper_order\":\"row_major_upper_6x6\",";
    out << "\"information_packing\":\"upper_triangle_row_major_6x6\",";
    out << "\"config\":";
    writePoseGraphConfigJson(out, config);

    out << ",\"poses\":[";
    for (size_t i = 0; i < m_poses.size(); ++i)
    {
        if (i > 0)
            out << ",";
        writePoseJson(out, m_poses[i].r, m_poses[i].t);
    }
    out << "]";

    out << ",\"priors\":[]";

    out << ",\"betweens\":[";
    for (size_t i = 0; i < between_factors_id.size(); ++i)
    {
        if (i > 0)
            out << ",";
        const M6D information = symmetricInformation(between_factors_info[i]);
        writeBetweenJson(
            out,
            between_factors_id[i].first,
            between_factors_id[i].second,
            between_factors_pose[i],
            information);
    }
    out << "]";

    out << ",\"expected\":{"
        << "\"status\":\"ok\","
        << "\"pose_count\":" << m_poses.size() << ","
        << "\"factor_count\":" << between_factors_id.size() << ","
        << "\"iterations_max\":" << config.max_iterations << ","
        << "\"accepted_steps_min\":0"
        << "}";
    out << ",\"baseline_tolerances\":{"
        << "\"final_cost_delta_abs_max\":1e-06,"
        << "\"final_residual_rms_delta_abs_max\":1e-06,"
        << "\"final_residual_max_delta_abs_max\":1e-06"
        << "}";
    out << ",\"metadata\":{"
        << "\"generator\":\"HBA::exportPoseGraphFixture\","
        << "\"source\":\"src/localization/hba\","
        << "\"pose_count\":" << m_poses.size() << ","
        << "\"prior_count\":0,"
        << "\"between_count\":" << between_factors_id.size() << ","
        << "\"levels\":" << m_levels << ","
        << "\"window_size\":" << m_config.window_size << ","
        << "\"stride\":" << m_config.stride
        << "}";
    out << "}";
    return out.good();
}
