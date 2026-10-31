#include "ltlio/frontend.hpp"
#include "ltlio/detail/Context.h"
#include "ltlio/detail/ESEKF.h"
#include "ltlio/detail/ProbabilityPlane.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>

namespace {
namespace detail=lingtu::ltlio::detail;
using Mat3=Eigen::Matrix3d;
using Vec3=Eigen::Vector3d;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void near(double value,double expected,double tolerance,const char* message) {
    require(std::isfinite(value) && std::abs(value-expected)<=tolerance,message);
}
lingtu::ltlio::Config config() {
    lingtu::ltlio::Config cfg;
    cfg.elevator_enabled=false; cfg.automatic_door=false;
    cfg.map_resolution=0.01; cfg.ieskf_max_iter=1;
    return cfg;
}
detail::PointCloudXYZIPtr plane(float z) {
    auto result=detail::PointCloudXYZIPtr(new detail::PointCloudXYZI);
    for(int x=-3;x<=3;++x) for(int y=-2;y<=2;++y) {
        detail::PointType p{}; p.x=x*0.3F; p.y=y*0.3F; p.z=z; result->push_back(p);
    }
    return result;
}
void actualMapUsesConfiguredIsotropicVarianceAndPose() {
    auto cfg=config();
    auto ctx=std::make_unique<detail::Context>(cfg);
    detail::EskfEstimator estimator(*ctx);
    auto state=estimator.get_cur_state();
    state.q=Eigen::Quaterniond(Eigen::AngleAxisd(0.37,Vec3(1,2,3).normalized()));
    state.P.setZero(); state.P.diagonal().setConstant(0.0002);
    state.P(0,3)=state.P(3,0)=0.00003;
    state.P(1,5)=state.P(5,1)=-0.00002;
    estimator.set_cur_state(state);
    detail::PointCloudXYZI cloud;
    for(const Vec3& v:{Vec3(0.3,-0.2,1.0),Vec3(6.0,3.0,-2.0),Vec3(-0.8,2.0,10.0)}) {
        detail::PointType p{}; p.x=v.x(); p.y=v.y(); p.z=v.z(); cloud.push_back(p);
    }
    const auto covariances=estimator.mapPointCovariances(cloud);
    const Mat3 rotation=state.q.toRotationMatrix();
    const double old_variance=1.0/cfg.lidar_cov_inv;
    near(ctx->lidar_prior_scale,old_variance,0.0,"legacy configured variance was changed");
    for(std::size_t i=0;i<cloud.size();++i) {
        const Vec3 lidar(cloud[i].x,cloud[i].y,cloud[i].z);
        const Vec3 imu=ctx->imu_R_lidar*lidar+ctx->imu_t_lidar;
        const Mat3 expected=old_variance*Mat3::Identity()+detail::matchingPointPoseCovariance(imu,rotation,state.P);
        require((covariances[i]-expected).norm()<1e-12,"actual map path lost configured isotropic calibration or pose cross covariance");
    }
}
struct Update { double correction; double posterior; int points; bool accepted; };
Update queryUpdate(bool uncertain_map) {
    auto cfg=config();
    auto ctx=std::make_unique<detail::Context>(cfg);
    detail::EskfEstimator estimator(*ctx);
    auto state=estimator.get_cur_state();
    state.P=Eigen::MatrixXd::Identity(21,21)*1e-6;
    state.P(5,5)=0.04;
    estimator.set_cur_state(state);
    auto map=plane(1.0F);
    if(uncertain_map) ctx->ikd_map->add(map,std::vector<Mat3>(map->size(),Mat3::Identity()*ctx->lidar_prior_scale));
    else ctx->ikd_map->add(map);
    auto query=plane(1.02F);
    estimator.UpdateByLidarIESKF(*query);
    const auto updated=estimator.get_cur_state();
    return {updated.p.z(),updated.P(5,5),estimator.validPointCount(),estimator.lastUpdateAccepted()};
}
void actualQueryMatchesOldConfiguredResidualVariance() {
    const auto cfg=config();
    const double scalar=1.0/cfg.lidar_cov_inv;
    const auto result=queryUpdate(false);
    const double precision=35.0/scalar;
    const double posterior=1.0/(1.0/0.04+precision);
    const double residual=static_cast<double>(1.02F)-1.0;
    require(result.accepted && result.points==35,"actual isotropic query did not accept its exact plane correspondences");
    near(result.posterior,posterior,1e-10,"actual query posterior does not use old configured physical variance");
    near(result.correction,-residual*precision*posterior,1e-10,"actual query correction does not use old configured physical variance");
    std::cout<<std::setprecision(17)<<"METRIC scalar_variance="<<scalar<<" exact_plane_posterior="<<result.posterior<<'\n';
}
void planeUncertaintyStillContributesToActualQuery() {
    const auto exact=queryUpdate(false),uncertain=queryUpdate(true);
    require(uncertain.accepted && uncertain.points==35,"uncertain-map query fixture failed correspondence acceptance");
    require(uncertain.posterior>exact.posterior+1e-10,"plane uncertainty disappeared from actual query residual covariance");
    std::cout<<std::setprecision(17)<<"METRIC uncertain_plane_posterior="<<uncertain.posterior
             <<" exact_plane_posterior="<<exact.posterior<<'\n';
}
}
int main() {
    struct Test { const char* name; void(*run)(); };
    const Test tests[]={{"actual map preserves legacy isotropic calibration and pose uncertainty",actualMapUsesConfiguredIsotropicVarianceAndPose},
        {"actual query posterior preserves legacy scalar calibration",actualQueryMatchesOldConfiguredResidualVariance},
        {"actual query still includes probability-plane uncertainty",planeUncertaintyStillContributesToActualQuery}};
    int failed=0;
    for(const auto& test:tests) try { test.run(); std::cout<<"PASS "<<test.name<<'\n'; }
        catch(const std::exception& error) { ++failed; std::cerr<<"FAIL "<<test.name<<": "<<error.what()<<'\n'; }
    std::cout<<3-failed<<"/3 isotropic-calibration tests passed\n";
    return failed?1:0;
}
