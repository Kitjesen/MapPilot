#include "bbs3d_global_localizer.h"

#include <chrono>
#include <cstdio>
#include <exception>

#ifdef LINGTU_ENABLE_BBS3D
#include <cpu_bbs3d/bbs3d.hpp>
#else
namespace cpu { class BBS3D {}; }
#endif

namespace {
inline std::vector<Eigen::Vector3d> to_eigen(const CloudType::Ptr& c) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(c->size());
    for (const auto& p : c->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        out.emplace_back(p.x, p.y, p.z);
    }
    return out;
}
}  // namespace

#ifdef LINGTU_ENABLE_BBS3D
BBS3DGlobalLocalizer::BBS3DGlobalLocalizer(const Config& cfg)
    : cfg_(cfg), bbs_(std::make_unique<cpu::BBS3D>()) {
    bbs_->set_num_threads(cfg_.num_threads);
    bbs_->set_angular_search_range(
        Eigen::Vector3d(-cfg_.roll_pitch_margin, -cfg_.roll_pitch_margin, cfg_.yaw_search_min),
        Eigen::Vector3d( cfg_.roll_pitch_margin,  cfg_.roll_pitch_margin, cfg_.yaw_search_max));
    if (cfg_.score_threshold > 0.0) {
        bbs_->set_score_threshold_percentage(cfg_.score_threshold);
    }
    if (cfg_.timeout_ms > 0) {
        bbs_->enable_timeout();
        bbs_->set_timeout_duration_in_msec(cfg_.timeout_ms);
    }
}

BBS3DGlobalLocalizer::~BBS3DGlobalLocalizer() = default;

bool BBS3DGlobalLocalizer::available() const {
    return true;
}

bool BBS3DGlobalLocalizer::set_map(const CloudType::Ptr& map_cloud) {
    if (!map_cloud || map_cloud->empty()) return false;

    auto pts = to_eigen(map_cloud);
    if (pts.empty()) return false;

    // Compute map bbox for translation search range.
    Eigen::Vector3d mn = pts[0], mx = pts[0];
    for (const auto& p : pts) {
        mn = mn.cwiseMin(p);
        mx = mx.cwiseMax(p);
    }

    try {
        bbs_->set_tar_points(pts, cfg_.min_level_res, cfg_.max_level);
        bbs_->set_trans_search_range(
            mn - Eigen::Vector3d::Constant(cfg_.trans_search_margin),
            mx + Eigen::Vector3d::Constant(cfg_.trans_search_margin));
    } catch (const std::exception& e) {
        has_map_ = false;
        std::fprintf(stderr, "[BBS3D] set_map exception: %s\n", e.what());
        return false;
    } catch (...) {
        has_map_ = false;
        std::fprintf(stderr, "[BBS3D] set_map unknown exception\n");
        return false;
    }
    has_map_ = true;
    std::fprintf(stderr,
        "[BBS3D] map set: %zu pts, bbox=[%.1f,%.1f,%.1f]..[%.1f,%.1f,%.1f]\n",
        pts.size(), mn.x(), mn.y(), mn.z(), mx.x(), mx.y(), mx.z());
    return true;
}

BBS3DGlobalLocalizer::Result BBS3DGlobalLocalizer::localize(const CloudType::Ptr& scan_cloud) {
    Result r;
    if (!has_map_) { r.message = "map not set"; return r; }
    if (!scan_cloud || scan_cloud->empty()) { r.message = "scan empty"; return r; }

    auto scan_pts = to_eigen(scan_cloud);
    if (scan_pts.empty()) { r.message = "scan has no finite points"; return r; }

    try {
        bbs_->set_src_points(scan_pts);

        auto t0 = std::chrono::steady_clock::now();
        bbs_->localize();
        auto t1 = std::chrono::steady_clock::now();
        r.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (!bbs_->has_localized()) {
            r.message = "bbs3d failed to converge / timed out";
            return r;
        }

        Eigen::Matrix4d pose_d = bbs_->get_global_pose();
        r.pose = pose_d.cast<float>();
        r.score_percentage = bbs_->get_best_score_percentage();
    } catch (const std::exception& e) {
        r.message = std::string("bbs3d exception: ") + e.what();
        std::fprintf(stderr, "[BBS3D] localize exception: %s\n", e.what());
        return r;
    } catch (...) {
        r.message = "bbs3d unknown exception";
        std::fprintf(stderr, "[BBS3D] localize unknown exception\n");
        return r;
    }
    r.success = true;
    r.message = "ok";
    std::fprintf(stderr,
        "[BBS3D] localize ok: %.0f ms, score=%.3f, t=[%.2f,%.2f,%.2f]\n",
        r.elapsed_ms, r.score_percentage,
        r.pose(0, 3), r.pose(1, 3), r.pose(2, 3));
    return r;
}
#else
BBS3DGlobalLocalizer::BBS3DGlobalLocalizer(const Config& cfg)
    : cfg_(cfg), bbs_(nullptr) {}

BBS3DGlobalLocalizer::~BBS3DGlobalLocalizer() = default;

bool BBS3DGlobalLocalizer::available() const {
    return false;
}

bool BBS3DGlobalLocalizer::set_map(const CloudType::Ptr& map_cloud) {
    (void)map_cloud;
    has_map_ = false;
    return false;
}

BBS3DGlobalLocalizer::Result BBS3DGlobalLocalizer::localize(const CloudType::Ptr& scan_cloud) {
    Result r;
    if (!scan_cloud || scan_cloud->empty()) {
        r.message = "scan empty";
        return r;
    }
    r.message = "BBS3D disabled at build time";
    return r;
}
#endif
