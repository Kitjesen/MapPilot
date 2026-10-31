#pragma once

#include <vector>

#include "Context.h"
namespace lingtu::ltlio::detail {
/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Project-level nearest-neighbor query wrapper backed by ikd-tree.
 */


class IkdNearestQuery final {
public:
    explicit IkdNearestQuery(Context& context) : ctx_(context) {}
    bool searchKnn(const PointType &query_point_world,
                   int k_nearest,
                   PointVector &nearest_points,
                   std::vector<float> &distance_sq) const {
        ctx_.ikdtree.Nearest_Search(query_point_world, k_nearest, nearest_points, distance_sq);
        return nearest_points.size() >= static_cast<size_t>(k_nearest);
    }
private:
    Context& ctx_;
};


} // namespace lingtu::ltlio::detail
