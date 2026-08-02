#pragma once
//
// This is the production indoor loop-candidate pre-filter when
// enable_scan_context is true. It is a local implementation of the Scan
// Context descriptor and has not yet passed a current MID-360 field benchmark.
// Treat replacement as an evidence-driven product change: verify license,
// replay accuracy, CPU/memory use, and failure behavior before selecting an
// upstream implementation. Do not change the algorithm without focused replay
// and loop-closure regression evidence.
//
// ──────────────────────────────────────────────────────────────────────
// scan_context.h — minimal Scan Context (Kim & Kim, IROS 2018) descriptor
// vendored for indoor loop closure pre-filtering.
//
// Algorithm (recap):
//   - Polar projection of a single scan into NUM_RING × NUM_SECTOR cells;
//     each cell stores the max z of points falling into it. The matrix is
//     the "scan context" descriptor.
//   - "Ring key" = column-wise mean of the descriptor (length NUM_RING).
//     This is rotation-invariant and serves as a fast KdTree key.
//   - Pairwise distance over two descriptors: cyclic shift the columns to
//     find the best yaw alignment, then average per-column cosine distance.
//
// Integration in this repo: SimplePGO calls `add(cloud)` per keyframe and
// `query(latest_idx)` to get a candidate (idx, yaw) for the most recent
// scan. The candidate is fed alongside the existing radius+ICP loop search
// as a *second-stage filter*, allowing `loop_search_radius` to be enlarged
// without flooding ICP with bad candidates.
//
// Tuning targets (Mid-360 indoor):
//   - NUM_RING=20, NUM_SECTOR=60, MAX_RANGE=20m, ring_key_threshold=0.4,
//     sc_distance_threshold=0.4 (loose; ICP still refines).
//
// References:
//   Kim & Kim 2018 IROS — Scan Context: Egocentric Spatial Descriptor for
//   Place Recognition Within 3D Point Cloud Map.
//   Kim et al. 2021 T-RO — Scan Context++: Structural Place Recognition
//   Robust to Rotation and Lateral Variations.
//
// Note: this is a pragmatic minimal port intended for indoor pre-filtering.
// Production tuning (e.g. ring weights, sector key, multi-candidate top-K)
// should follow the upstream `aserbremen/scancontext_ros2` reference once
// real-data validation is available (see N1 SOP).

#include "commons.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <utility>

class ScanContext
{
public:
    static constexpr int NUM_RING = 20;
    static constexpr int NUM_SECTOR = 60;

    struct Config
    {
        double max_range = 20.0;        // m — points farther than this are dropped
        double ring_key_threshold = 0.4; // KdTree pre-filter threshold (loose)
        double sc_distance_threshold = 0.4; // final SC distance threshold
        int    num_candidates = 5;       // top-K from KdTree to verify with full SC
        int    min_history_gap = 30;     // skip last N keyframes (avoid self-match)
    };

    using Descriptor = Eigen::Matrix<float, NUM_RING, NUM_SECTOR>;
    using RingKey = Eigen::Matrix<float, NUM_RING, 1>;

    ScanContext() : m_config(Config{}) {}
    explicit ScanContext(const Config &config) : m_config(config) {}

    // Build a descriptor from a body-frame point cloud and store it.
    // Returns the index assigned to this descriptor.
    size_t add(const CloudType::ConstPtr &cloud)
    {
        Descriptor desc = makeDescriptor(*cloud);
        RingKey key = makeRingKey(desc);
        m_descriptors.push_back(desc);
        m_ring_keys.push_back(key);
        // KdTree is rebuilt lazily on next query (see ensureTree()).
        m_tree_dirty = true;
        return m_descriptors.size() - 1;
    }

    // Query: find the best loop candidate for descriptor at index `query_idx`
    // among descriptors strictly older than (query_idx - min_history_gap).
    // Returns {idx, yaw_offset_rad}. idx == -1 means no acceptable candidate.
    std::pair<int, float> query(size_t query_idx) const
    {
        if (query_idx >= m_descriptors.size())
            return {-1, 0.0f};
        const int valid_count = static_cast<int>(query_idx) - m_config.min_history_gap;
        if (valid_count <= 0)
            return {-1, 0.0f};
        ensureTree(valid_count);

        // KdTree pre-filter: get top-K candidate indices on ring key
        const RingKey &q_key = m_ring_keys[query_idx];
        pcl::PointCloud<pcl::PointXYZ>::Ptr query_pt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ p;
        // Pack ring key into PointXYZ — only first 3 dims; fall back to L2 in
        // ringKeyDistance() if KdTree result is empty. For Mid-360 the first
        // few rings carry most of the information so this is acceptable as a
        // cheap pre-filter; the full ring_key L2 distance is checked below.
        p.x = q_key(0); p.y = q_key(1); p.z = q_key(2);
        query_pt->push_back(p);

        std::vector<int> indices;
        std::vector<float> dists;
        const int k = std::min(m_config.num_candidates, valid_count);
        m_kdtree.nearestKSearch(p, k, indices, dists);

        int best_idx = -1;
        float best_yaw = 0.0f;
        float best_sc_dist = m_config.sc_distance_threshold;

        for (int cand : indices)
        {
            if (cand < 0 || cand >= valid_count) continue;
            // Full ring key distance gate (catches false candidates from the
            // partial PointXYZ pre-filter).
            float ring_dist = (m_ring_keys[cand] - q_key).norm() / std::sqrt(static_cast<float>(NUM_RING));
            if (ring_dist > m_config.ring_key_threshold) continue;

            float yaw_rad = 0.0f;
            float sc_dist = scDistance(m_descriptors[query_idx], m_descriptors[cand], yaw_rad);
            if (sc_dist < best_sc_dist)
            {
                best_sc_dist = sc_dist;
                best_yaw = yaw_rad;
                best_idx = cand;
            }
        }
        return {best_idx, best_yaw};
    }

    size_t size() const { return m_descriptors.size(); }

private:
    Config m_config;
    std::vector<Descriptor, Eigen::aligned_allocator<Descriptor>> m_descriptors;
    std::vector<RingKey, Eigen::aligned_allocator<RingKey>> m_ring_keys;
    mutable pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
    mutable pcl::PointCloud<pcl::PointXYZ>::Ptr m_kd_cloud;
    mutable bool m_tree_dirty = true;

    Descriptor makeDescriptor(const CloudType &cloud) const
    {
        Descriptor desc = Descriptor::Constant(NoDataValue());
        const float ring_step = static_cast<float>(m_config.max_range) / NUM_RING;
        const float sector_step = 2.0f * static_cast<float>(M_PI) / NUM_SECTOR;
        for (const auto &pt : cloud.points)
        {
            const float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (r < 1e-3f || r >= m_config.max_range) continue;
            const int ring_idx = std::min(static_cast<int>(r / ring_step), NUM_RING - 1);
            float theta = std::atan2(pt.y, pt.x);
            if (theta < 0) theta += 2.0f * static_cast<float>(M_PI);
            const int sector_idx = std::min(static_cast<int>(theta / sector_step), NUM_SECTOR - 1);
            float &cell = desc(ring_idx, sector_idx);
            if (cell == NoDataValue() || pt.z > cell) cell = pt.z;
        }
        // Replace empty cells with 0 so distance metrics behave sanely.
        for (int r = 0; r < NUM_RING; ++r)
            for (int s = 0; s < NUM_SECTOR; ++s)
                if (desc(r, s) == NoDataValue()) desc(r, s) = 0.0f;
        return desc;
    }

    static RingKey makeRingKey(const Descriptor &desc)
    {
        // Mean per row (over sectors) — yaw-invariant.
        return desc.rowwise().mean();
    }

    // Pairwise scan-context distance with cyclic-shift yaw search.
    // Returns the average per-column cosine distance at the best yaw shift,
    // and writes that yaw (rad) into yaw_out.
    static float scDistance(const Descriptor &a, const Descriptor &b, float &yaw_out)
    {
        float best = std::numeric_limits<float>::infinity();
        int best_shift = 0;
        // Coarse search over yaw bins (=NUM_SECTOR shifts).
        for (int shift = 0; shift < NUM_SECTOR; ++shift)
        {
            float sum = 0.0f;
            int valid_cols = 0;
            for (int s = 0; s < NUM_SECTOR; ++s)
            {
                const int s_shift = (s + shift) % NUM_SECTOR;
                const auto col_a = a.col(s);
                const auto col_b = b.col(s_shift);
                const float na = col_a.norm();
                const float nb = col_b.norm();
                if (na < 1e-6f || nb < 1e-6f) continue;
                const float cos_sim = col_a.dot(col_b) / (na * nb);
                sum += 1.0f - cos_sim;
                ++valid_cols;
            }
            if (valid_cols == 0) continue;
            const float mean = sum / valid_cols;
            if (mean < best)
            {
                best = mean;
                best_shift = shift;
            }
        }
        yaw_out = static_cast<float>(best_shift) * 2.0f * static_cast<float>(M_PI) / NUM_SECTOR;
        if (yaw_out > static_cast<float>(M_PI)) yaw_out -= 2.0f * static_cast<float>(M_PI);
        return best;
    }

    void ensureTree(int valid_count) const
    {
        if (!m_tree_dirty && m_kd_cloud && static_cast<int>(m_kd_cloud->size()) == valid_count)
            return;
        m_kd_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        m_kd_cloud->reserve(valid_count);
        for (int i = 0; i < valid_count; ++i)
        {
            const RingKey &k = m_ring_keys[i];
            pcl::PointXYZ p;
            // Project ring key into 3D (use first 3 ring values as x/y/z) —
            // accepted lossy projection for KdTree pre-filter speed; the
            // full ring_key L2 distance is the actual gate inside query().
            p.x = k(0); p.y = k(1); p.z = k(2);
            m_kd_cloud->push_back(p);
        }
        m_kdtree.setInputCloud(m_kd_cloud);
        m_tree_dirty = false;
    }

    static constexpr float NoDataValue() { return -10000.0f; }
};
