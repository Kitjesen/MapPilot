#pragma once
#include "commons.h"
#include <filesystem>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#ifdef LINGTU_ENABLE_SMALL_GICP
#include <small_gicp/pcl/pcl_registration.hpp>
template <typename SourceT, typename TargetT>
using LingTuIcpBackend = small_gicp::RegistrationPCL<SourceT, TargetT>;
#else
#include <pcl/registration/gicp.h>
template <typename SourceT, typename TargetT>
using LingTuIcpBackend = pcl::GeneralizedIterativeClosestPoint<SourceT, TargetT>;
#endif

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;

    // Reserved for accelerated ICP backends. The runtime path currently uses
    // PCL ICP because small_gicp has shown uncaught worker-thread exceptions
    // on real robot scans.
    int num_threads = 4;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path);

    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }

    /// Last ICP fitness score (lower = better alignment, 0 = perfect)
    double getLastFitnessScore() const { return m_last_fitness_score; }

    /// R4: three-axis health diagnostics from small_gicp. Together with
    /// fitness these give the multi-frame health gate (P3) the iter +
    /// covariance signals it was missing under PCL ICP. All values come
    /// from the *refine* stage's RegistrationResult — rough is a coarse
    /// pre-step whose iteration count is uninformative.
    int    getLastIterations()  const { return m_last_iterations; }
    bool   getLastConverged()   const { return m_last_converged; }
    int    getLastInliers()     const { return m_last_inliers; }
    const std::string &getBackendName() const { return m_backend_name; }

    /// Trace of the position covariance estimated from the registration
    /// Hessian. Larger = more uncertain
    /// translation. Returns -1 when no align() has succeeded yet.
    double getLastPosCovTrace() const { return m_last_pos_cov_trace; }

private:
    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    LingTuIcpBackend<PointType, PointType> m_refine_icp;
    LingTuIcpBackend<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
    std::string m_backend_name{
#ifdef LINGTU_ENABLE_SMALL_GICP
        "small_gicp"
#else
        "pcl_gicp"
#endif
    };
    double m_last_fitness_score{-1.0};   // -1 = not yet computed
    int    m_last_iterations{-1};
    int    m_last_inliers{-1};
    bool   m_last_converged{false};
    double m_last_pos_cov_trace{-1.0};   // -1 = not yet computed
};
