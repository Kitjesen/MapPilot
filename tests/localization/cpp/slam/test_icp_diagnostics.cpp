#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

#include "icp_localizer.h"

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CloudType::Ptr makeCloud()
{
    CloudType::Ptr cloud(new CloudType);
    for (int x = 0; x < 8; ++x) {
        for (int y = 0; y < 7; ++y) {
            for (int z = 0; z < 5; ++z) {
                PointType point;
                point.x = static_cast<float>(x) * 0.17F;
                point.y = static_cast<float>(y) * 0.19F;
                point.z = static_cast<float>(z) * 0.13F;
                point.intensity = static_cast<float>((x + 2 * y + 3 * z) % 11);
                cloud->push_back(point);
            }
        }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
}

CloudType::Ptr makeSmallCloud(const CloudType::Ptr &source)
{
    CloudType::Ptr cloud(new CloudType);
    cloud->points.assign(source->points.begin(), source->points.begin() + 16);
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
}

CloudType::Ptr transformedCloud(const CloudType::Ptr &source, const M4F &transform)
{
    CloudType::Ptr cloud(new CloudType);
    cloud->reserve(source->size());
    for (const PointType &point : source->points) {
        const Eigen::Vector4f transformed = transform * Eigen::Vector4f(
            point.x, point.y, point.z, 1.0F);
        PointType out = point;
        out.x = transformed.x();
        out.y = transformed.y();
        out.z = transformed.z();
        cloud->push_back(out);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
}

CloudType::Ptr cloudWithOutsideMapPoints(const CloudType::Ptr &source)
{
    CloudType::Ptr cloud(new CloudType(*source));
    for (int i = 0; i < 400; ++i) {
        PointType point;
        point.x = 10.0F + static_cast<float>(i % 20) * 0.05F;
        point.y = 10.0F + static_cast<float>((i / 20) % 10) * 0.05F;
        point.z = 5.0F + static_cast<float>(i / 200) * 0.05F;
        cloud->push_back(point);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
}

}  // namespace

int main()
{
    ICPConfig config;
    config.refine_scan_resolution = 0.0;
    config.refine_map_resolution = 0.0;
    config.rough_scan_resolution = 0.0;
    config.rough_map_resolution = 0.0;

    ICPLocalizer localizer(config);
    const CloudType::Ptr cloud = makeCloud();
    require(localizer.setMap(cloud), "map should load");
    localizer.setInput(cloud);
    M4F pose = M4F::Identity();
    require(localizer.evaluate(pose, 0.3), "identity seed should verify");
    require(localizer.getLastInputPoints() == static_cast<int>(cloud->size()),
            "seed verification must retain the finite input count");
    require(localizer.getLastEvaluatedPoints() == static_cast<int>(cloud->size()),
            "seed verification must score the full finite input");
    require(localizer.getLastInliers() == static_cast<int>(cloud->size()),
            "identity seed must retain all points as inliers");
    require(localizer.getLastFitnessScore() < 1e-8,
            "identity seed fitness must be near zero");

    const CloudType::Ptr boundary_scan = cloudWithOutsideMapPoints(cloud);
    localizer.setInput(boundary_scan);
    require(localizer.evaluate(pose, 0.3),
            "finite-map overlap should ignore scan points outside map support");
    require(localizer.getLastInputPoints() == static_cast<int>(boundary_scan->size()),
            "finite-map diagnostics must retain the full input count");
    require(localizer.getLastEvaluatedPoints() == static_cast<int>(cloud->size()),
            "only points inside the finite map support must enter the overlap denominator");
    require(localizer.getLastInliers() == static_cast<int>(cloud->size()),
            "supported identity points must remain inliers");

    M4F true_pose = M4F::Identity();
    const float true_yaw = 0.006F;
    true_pose(0, 0) = std::cos(true_yaw);
    true_pose(0, 1) = -std::sin(true_yaw);
    true_pose(1, 0) = std::sin(true_yaw);
    true_pose(1, 1) = std::cos(true_yaw);
    true_pose(0, 3) = 0.04F;
    true_pose(1, 3) = -0.03F;
    true_pose(2, 3) = 0.02F;
    localizer.setInput(transformedCloud(cloud, true_pose.inverse()));
    M4F planar_guess = M4F::Identity();
    planar_guess(0, 3) = 0.01F;
    require(localizer.alignPlanar(planar_guess, 0.3),
            "planar registration should refine a nearby seed");
    require((planar_guess.block<3, 1>(0, 3) - true_pose.block<3, 1>(0, 3)).norm() < 0.02F,
            "planar registration should recover translation");
    require(std::abs(std::atan2(planar_guess(1, 0), planar_guess(0, 0)) - true_yaw) < 0.003F,
            "planar registration should recover yaw");

    localizer.setInput(cloud);
    pose = M4F::Identity();
    require(localizer.getLastInputPoints() == static_cast<int>(cloud->size()),
            "registration input must retain every finite point");
    require(localizer.refineMap()->size() == cloud->size(),
            "refine target must remain immutable across scan updates");
    require(localizer.roughMap()->size() == cloud->size(),
            "rough target must remain immutable across scan updates");
    require(localizer.align(pose), "identity registration should converge");
    require(localizer.getLastConverged(), "registration must report convergence");
    require(localizer.getLastInliers() >= 30, "registration must report inliers");
    require(
        std::isfinite(localizer.getLastPosCovTrace()) &&
            localizer.getLastPosCovTrace() >= 0.0,
        "registration must report finite position covariance");

    CloudType::Ptr cloud_with_nan = makeCloud();
    cloud_with_nan->points.front().x = std::numeric_limits<float>::quiet_NaN();
    localizer.setInput(cloud_with_nan);
    pose = M4F::Identity();
    require(localizer.align(pose), "non-finite scan points must be filtered");

    require(localizer.setMap(cloud_with_nan),
            "a resized map snapshot must replace cached target search structures");
    localizer.setInput(cloud_with_nan);
    pose = M4F::Identity();
    require(localizer.align(pose),
            "registration must survive a repeated map update with a changed point count");

    require(localizer.setMap(cloud), "the original map snapshot should be restorable");
    localizer.setInput(makeSmallCloud(cloud));
    pose = M4F::Identity();
    require(!localizer.align(pose), "undersized GICP scans must be rejected");

    localizer.setInput(cloud);
    pose = M4F::Identity();
    pose(0, 3) = std::numeric_limits<float>::quiet_NaN();
    require(!localizer.align(pose), "non-finite initial guesses must be rejected");

    // As registration improves, newly matched points may increase inlier-only
    // MSE. They must still lower the capped error over the supported scan.
    CloudType::Ptr partial_target(new CloudType);
    CloudType::Ptr partial_scan(new CloudType);
    for (int plane = 0; plane < 2; ++plane) {
        for (int y = -4; y <= 4; ++y) {
            for (int z = -4; z <= 4; ++z) {
                PointType point;
                point.x = 4.F * plane;
                point.y = 0.4F * y;
                point.z = 0.4F * z;
                partial_target->push_back(point);
                point.x += plane == 0 ? 0.1F : 0.25F;
                partial_scan->push_back(point);
            }
        }
    }
    // The mapped environment extends beyond the observed two surfaces.
    PointType far_corner;
    far_corner.x = 6.F; far_corner.y = 4.F; far_corner.z = 4.F;
    partial_target->push_back(far_corner);
    ICPLocalizer partial(config);
    require(partial.setMap(partial_target), "partial-correspondence map should load");
    partial.setInput(partial_scan);
    M4F partial_guess = M4F::Identity();
    require(partial.evaluate(partial_guess, 0.2) && partial.getLastInliers() == 81,
            "only the nearer surface should initially have correspondences");
    require(partial.alignPlanar(partial_guess, 0.2, 20, 0.4, 0.03, 0.1),
            "planar search must improve partial correspondences");
    require(partial.getLastInliers() == 162 && partial_guess(0, 3) < -0.12F,
            "new correspondences must not stop a better planar registration");
    return 0;
}
