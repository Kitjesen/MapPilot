#include "icp_localizer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>

#ifdef LINGTU_ENABLE_SMALL_GICP
namespace {

double positionCovarianceTrace(const Eigen::Matrix<double, 6, 6> &hessian)
{
    if (!hessian.allFinite()) {
        return -1.0;
    }
    const double det = hessian.determinant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return -1.0;
    }
    const Eigen::Matrix<double, 6, 6> cov = hessian.inverse();
    const double trace = cov.block<3, 3>(3, 3).trace();
    return std::isfinite(trace) ? trace : -1.0;
}

}  // namespace
#endif

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);

#ifdef LINGTU_ENABLE_SMALL_GICP
    m_rough_icp.setRegistrationType("GICP");
    m_refine_icp.setRegistrationType("GICP");
    m_rough_icp.setNumThreads(std::max(1, m_config.num_threads));
    m_refine_icp.setNumThreads(std::max(1, m_config.num_threads));
#endif
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }

    // 预构建 KD-tree: setInputTarget 触发 O(N logN) 的 KD-tree 构建。
    // 在 loadMap 时做一次, 而非每次 align() 都重复, 节省 10-50ms/帧。
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputTarget(m_refine_tgt);

    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0) {
        m_last_fitness_score = -1.0;
        m_last_iterations = -1;
        m_last_inliers = -1;
        m_last_converged = false;
        m_last_pos_cov_trace = -1.0;
        return false;
    }
    try {
        // setInputTarget + setMaximumIterations were done in loadMap()
        // (KD-tree already cached).
        m_rough_icp.setInputSource(m_rough_inp);
        m_rough_icp.align(*aligned_cloud, guess);
        if (!m_rough_icp.hasConverged() ||
            m_rough_icp.getFitnessScore() > m_config.rough_score_thresh) {
            m_last_fitness_score = m_rough_icp.getFitnessScore();
            m_last_iterations = -1;
            m_last_inliers = -1;
            m_last_converged = false;
            m_last_pos_cov_trace = -1.0;
            return false;
        }
        m_refine_icp.setInputSource(m_refine_inp);
        m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
        m_last_fitness_score = m_refine_icp.getFitnessScore();
        m_last_iterations = -1;
        m_last_inliers = -1;
        m_last_converged = m_refine_icp.hasConverged();
        m_last_pos_cov_trace = -1.0;
#ifdef LINGTU_ENABLE_SMALL_GICP
        const auto &detail = m_refine_icp.getRegistrationResult();
        m_last_iterations = static_cast<int>(detail.iterations);
        m_last_inliers = static_cast<int>(detail.num_inliers);
        m_last_converged = detail.converged;
        m_last_pos_cov_trace = positionCovarianceTrace(detail.H);
#endif

        if (!m_last_converged || m_last_fitness_score > m_config.refine_score_thresh)
            return false;
        guess = m_refine_icp.getFinalTransformation();
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[ICPLocalizer] align exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ICPLocalizer] align unknown exception" << std::endl;
    }
    m_last_fitness_score = -1.0;
    m_last_iterations = -1;
    m_last_inliers = -1;
    m_last_converged = false;
    m_last_pos_cov_trace = -1.0;
    return false;
}
