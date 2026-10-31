#include "ltlio/detail/ikd_Tree.h"

#include <algorithm>
#include <chrono>
#include <atomic>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace lingtu::ltlio::detail {
template <typename PointType>
struct KD_TREE_TEST_ACCESS
{
    static typename KD_TREE<PointType>::RebuildDiagnostics diagnostics(
        const KD_TREE<PointType> &tree)
    {
        return tree.rebuild_diagnostics();
    }

    static bool wait_started(
        KD_TREE<PointType> &tree, std::uint64_t count, std::chrono::milliseconds timeout)
    {
        return tree.wait_for_rebuild_started(count, timeout);
    }

    static bool wait_completed(
        KD_TREE<PointType> &tree, std::uint64_t count, std::chrono::milliseconds timeout)
    {
        return tree.wait_for_rebuild_completed(count, timeout);
    }

    static void wait_completed_without_timeout(KD_TREE<PointType> &tree, std::uint64_t count)
    {
        std::unique_lock<std::mutex> lock(tree.rebuild_diagnostics_mutex);
        tree.rebuild_diagnostics_changed.wait(lock, [&] {
            return tree.rebuild_diagnostics_state.completed >= count ||
                   !tree.rebuild_diagnostics_state.accepting;
        });
    }

    static bool wait_queued(KD_TREE<PointType> &tree, std::chrono::milliseconds timeout)
    {
        return tree.wait_for_rebuild_queued(timeout);
    }

    static bool wait_active(KD_TREE<PointType> &tree, std::chrono::milliseconds timeout)
    {
        return tree.wait_for_rebuild_active(timeout);
    }

    static void pause_before_start(KD_TREE<PointType> &tree, bool paused)
    {
        tree.set_rebuild_start_gate_paused(paused);
    }

    static void pause_after_start(KD_TREE<PointType> &tree, bool paused)
    {
        tree.set_rebuild_diagnostic_gate_paused(paused);
    }

    static bool wait_flattened(
        KD_TREE<PointType> &tree, std::uint64_t count, std::chrono::milliseconds timeout)
    {
        return tree.wait_for_rebuild_flattened(count, timeout);
    }

    static void pause_after_flatten(KD_TREE<PointType> &tree, bool paused)
    {
        tree.set_rebuild_flatten_gate_paused(paused);
    }

    static PointType queue_non_root(KD_TREE<PointType> &tree)
    {
        auto **slot = &tree.Root_Node->left_son_ptr;
        PointType target_point = (*slot)->point;
        tree.Rebuild(slot);
        return target_point;
    }

    struct AncestorReplayFixture
    {
        typename KD_TREE<PointType>::KD_TREE_NODE *ancestor = nullptr;
        typename KD_TREE<PointType>::KD_TREE_NODE **target_slot = nullptr;
        PointType ancestor_point{};
        int root_size = 0;
        int target_size = 0;
    };

    static AncestorReplayFixture left_ancestor_replay_fixture(KD_TREE<PointType> &tree)
    {
        AncestorReplayFixture fixture;
        fixture.ancestor = tree.Root_Node->left_son_ptr;
        fixture.target_slot = &fixture.ancestor->left_son_ptr;
        fixture.ancestor_point = fixture.ancestor->point;
        fixture.root_size = tree.Root_Node->TreeSize;
        fixture.target_size = (*fixture.target_slot)->TreeSize;
        return fixture;
    }

    static bool all_working_flags_clear(const KD_TREE<PointType> &tree)
    {
        return all_working_flags_clear(tree.Root_Node);
    }

    static void queue_rebuild(KD_TREE<PointType> &tree,
                              typename KD_TREE<PointType>::KD_TREE_NODE **slot)
    {
        tree.Rebuild(slot);
    }

    static void enqueue_add(KD_TREE<PointType> &tree, const PointType &point)
    {
        typename KD_TREE<PointType>::Operation_Logger_Type operation{};
        operation.op = ADD_POINT;
        operation.point = point;
        std::lock_guard<std::mutex> lock(tree.rebuild_logger_mutex_lock);
        tree.Rebuild_Logger.push(operation);
    }

    static int subtree_size(typename KD_TREE<PointType>::KD_TREE_NODE **slot)
    {
        return *slot == nullptr ? 0 : (*slot)->TreeSize;
    }

    static bool left_split_tie_point(KD_TREE<PointType> &tree, PointType &point)
    {
        if (tree.Root_Node == nullptr)
            return false;
        const int axis = tree.Root_Node->division_axis;
        const float split = coordinate(tree.Root_Node->point, axis);
        return find_coordinate(tree.Root_Node->left_son_ptr, axis, split, point);
    }

private:
    static bool all_working_flags_clear(
        const typename KD_TREE<PointType>::KD_TREE_NODE *node)
    {
        if (node == nullptr)
            return true;
        return !node->working_flag && all_working_flags_clear(node->left_son_ptr) &&
               all_working_flags_clear(node->right_son_ptr);
    }

    static float coordinate(const PointType &point, int axis)
    {
        return axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
    }

    static bool find_coordinate(
        typename KD_TREE<PointType>::KD_TREE_NODE *node,
        int axis,
        float value,
        PointType &point)
    {
        if (node == nullptr)
            return false;
        if (coordinate(node->point, axis) == value)
        {
            point = node->point;
            return true;
        }
        return find_coordinate(node->left_son_ptr, axis, value, point) ||
               find_coordinate(node->right_son_ptr, axis, value, point);
    }

public:

    static void queue_root(KD_TREE<PointType> &tree)
    {
        tree.Rebuild(&tree.Root_Node);
    }

    static void enqueue_push_down(KD_TREE<PointType> &tree)
    {
        typename KD_TREE<PointType>::Operation_Logger_Type operation{};
        operation.op = PUSH_DOWN;
        std::lock_guard<std::mutex> lock(tree.rebuild_logger_mutex_lock);
        tree.Rebuild_Logger.push(operation);
    }

    static void pause_searches(KD_TREE<PointType> &tree, bool paused)
    {
        {
            std::lock_guard<std::mutex> lock(tree.test_search_gate_mutex);
            tree.test_search_gate_paused = paused;
        }
        tree.test_search_gate_changed.notify_all();
    }

    static void wait_searches_in_flight(KD_TREE<PointType> &tree, int count)
    {
        std::unique_lock<std::mutex> lock(tree.test_search_gate_mutex);
        tree.test_search_gate_changed.wait(lock, [&] {
            return tree.test_searches_in_flight.load(std::memory_order_acquire) >= count;
        });
    }

    static bool wait_searches_in_flight(
        KD_TREE<PointType> &tree, int count, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(tree.test_search_gate_mutex);
        return tree.test_search_gate_changed.wait_for(lock, timeout, [&] {
            return tree.test_searches_in_flight.load(std::memory_order_acquire) >= count;
        });
    }

    static std::unique_lock<std::mutex> hold_working_lock(KD_TREE<PointType> &tree)
    {
        return std::unique_lock<std::mutex>(tree.working_flag_mutex);
    }

    static std::unique_lock<std::mutex> hold_right_child_lock(KD_TREE<PointType> &tree)
    {
        return std::unique_lock<std::mutex>(tree.Root_Node->right_son_ptr->push_down_mutex_lock);
    }

    static bool wait_for_pending_writer(KD_TREE<PointType> &tree, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        do
        {
            {
                std::lock_guard<std::mutex> lock(tree.search_flag_mutex);
                if (tree.search_writers_pending > 0) return true;
            }
            std::this_thread::yield();
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
    }

    static void wake_search_waiters(KD_TREE<PointType> &tree)
    {
        tree.search_state_changed.notify_all();
    }

    static int max_searches_in_flight(const KD_TREE<PointType> &tree)
    {
        return tree.test_max_searches_in_flight.load(std::memory_order_acquire);
    }

    static BoxPointType lazy_non_leaf_box(KD_TREE<PointType> &tree)
    {
        auto *node = tree.Root_Node->left_son_ptr;
        if (node == nullptr || node->left_son_ptr == nullptr || node->right_son_ptr == nullptr)
            return {};
        BoxPointType box{};
        box.vertex_min[0] = node->node_range_x[0];
        box.vertex_min[1] = node->node_range_y[0];
        box.vertex_min[2] = node->node_range_z[0];
        box.vertex_max[0] = node->node_range_x[1] + 0.001F;
        box.vertex_max[1] = node->node_range_y[1] + 0.001F;
        box.vertex_max[2] = node->node_range_z[1] + 0.001F;
        return box;
    }

    static PointType lazy_non_leaf_point(KD_TREE<PointType> &tree)
    {
        return tree.Root_Node->left_son_ptr->point;
    }

    static bool lazy_flags_present(KD_TREE<PointType> &tree)
    {
        auto *node = tree.Root_Node->left_son_ptr;
        return node->need_push_down_to_left && node->need_push_down_to_right;
    }

    static bool lazy_delete_propagated(KD_TREE<PointType> &tree)
    {
        auto *node = tree.Root_Node->left_son_ptr;
        return !node->need_push_down_to_left && !node->need_push_down_to_right &&
               node->left_son_ptr->tree_deleted && node->right_son_ptr->tree_deleted;
    }
};
} // namespace lingtu::ltlio::detail

namespace {

using namespace lingtu::ltlio::detail;
using Tree = KD_TREE<pcl::PointXYZ>;
using TreeTestAccess = KD_TREE_TEST_ACCESS<pcl::PointXYZ>;

static_assert(!std::is_copy_constructible_v<Tree>);
static_assert(!std::is_copy_assignable_v<Tree>);
static_assert(!std::is_move_constructible_v<Tree>);
static_assert(!std::is_move_assignable_v<Tree>);

Tree::PointVector make_points(int begin, int end)
{
    Tree::PointVector points;
    points.reserve(static_cast<std::size_t>(end - begin));
    for (int index = begin; index < end; ++index)
    {
        pcl::PointXYZ point;
        point.x = static_cast<float>(index % 50) * 0.2F;
        point.y = static_cast<float>((index / 50) % 30) * 0.2F;
        point.z = static_cast<float>(index / 1500) * 0.2F;
        points.push_back(point);
    }
    return points;
}

Tree::PointVector make_unbalanced_tail(int count)
{
    Tree::PointVector points;
    points.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index)
    {
        pcl::PointXYZ point;
        point.x = 1000.0F + static_cast<float>(index) * 0.1F;
        point.y = 0.0F;
        point.z = 0.0F;
        points.push_back(point);
    }
    return points;
}

BoxPointType whole_tree_box()
{
    BoxPointType box{};
    for (int axis = 0; axis < 3; ++axis)
    {
        box.vertex_min[axis] = -2000.0F;
        box.vertex_max[axis] = 2000.0F;
    }
    return box;
}

bool nearest_is_exact(Tree &tree, const pcl::PointXYZ &query)
{
    Tree::PointVector nearest;
    std::vector<float> distances;
    tree.Nearest_Search(query, 1, nearest, distances, 1.0F);
    return nearest.size() == 1 && distances.size() == 1 &&
           std::abs(distances.front()) < 1.0e-6F;
}

bool range_queries_find_exact(Tree &tree, const pcl::PointXYZ &query)
{
    BoxPointType box{};
    for (int axis = 0; axis < 3; ++axis)
    {
        const float coordinate = axis == 0 ? query.x : (axis == 1 ? query.y : query.z);
        box.vertex_min[axis] = coordinate - 0.01F;
        box.vertex_max[axis] = coordinate + 0.01F;
    }
    auto contains_query = [&](const Tree::PointVector &points) {
        return std::any_of(points.begin(), points.end(), [&](const pcl::PointXYZ &point) {
            return point.x == query.x && point.y == query.y && point.z == query.z;
        });
    };
    Tree::PointVector box_points;
    tree.Box_Search(box, box_points);
    Tree::PointVector radius_points;
    tree.Radius_Search(query, 0.01F, radius_points);
    return contains_query(box_points) && contains_query(radius_points);
}

Tree::PointVector corner_points()
{
    // A sampled wall/floor intersection. Binary-exact spacing makes all six
    // neighbors genuinely equidistant, without an epsilon-based tie oracle.
    pcl::PointXYZ center;
    center.x = center.z = 1.0F;
    center.y = 0.0F;
    Tree::PointVector points{center};
    for (int axis = 0; axis < 3; ++axis)
        for (float sign : {-1.0F, 1.0F})
        {
            auto point = center;
            if (axis == 0) point.x += sign * 0.0625F;
            else if (axis == 1) point.y += sign * 0.0625F;
            else point.z += sign * 0.0625F;
            points.push_back(point);
        }
    return points;
}

auto coordinates(const pcl::PointXYZ &point)
{
    return std::tuple(point.x, point.y, point.z);
}

bool same_points(Tree::PointVector a, Tree::PointVector b)
{
    if (a.size() != b.size()) return false;
    const auto less = [](const auto &x, const auto &y) { return coordinates(x) < coordinates(y); };
    std::sort(a.begin(), a.end(), less);
    std::sort(b.begin(), b.end(), less);
    for (std::size_t i = 0; i < a.size(); ++i)
        if (coordinates(a[i]) != coordinates(b[i])) return false;
    return true;
}

bool canonical_nearest(Tree &tree, const Tree::PointVector &points,
                       const pcl::PointXYZ &query, int count, const char *stage,
                       double radius = INFINITY)
{
    // Independent brute-force oracle: inspect every point, then order the
    // exact float squared distances and XYZ. No tree/heap comparator is reused.
    using Candidate = std::tuple<float, float, float, float>;
    std::vector<Candidate> expected;
    for (const auto &point : points)
    {
        const float x = point.x - query.x;
        const float y = point.y - query.y;
        const float z = point.z - query.z;
        const float squared_distance = x * x + y * y + z * z;
        if (squared_distance <= radius * radius)
            expected.emplace_back(squared_distance, point.x, point.y, point.z);
    }
    std::sort(expected.begin(), expected.end());
    expected.resize(std::min(expected.size(), static_cast<std::size_t>(count)));
    Tree::PointVector actual;
    std::vector<float> distances;
    tree.Nearest_Search(query, count, actual, distances, radius);
    if (actual.size() != expected.size() || distances.size() != expected.size())
    {
        std::cerr << stage << ": canonical KNN count differs\n";
        return false;
    }
    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        const Candidate found{distances[i], actual[i].x, actual[i].y, actual[i].z};
        if (found != expected[i])
        {
            const auto &[distance, x, y, z] = expected[i];
            std::cerr << stage << ": canonical KNN mismatch at " << i
                      << " expected=(" << x << ',' << y << ',' << z << ") d2=" << distance
                      << " actual=(" << actual[i].x << ',' << actual[i].y << ',' << actual[i].z
                      << ") d2=" << distances[i] << '\n';
            return false;
        }
    }
    return true;
}

bool stable_corner_neighbors()
{
    const auto source = corner_points();
    const auto query = source.front();
    auto tree = std::make_unique<Tree>();
    bool passed = true;
    auto reordered = source;
    for (int order = 0; order < 3; ++order)
    {
        if (order == 1) std::reverse(reordered.begin(), reordered.end());
        if (order == 2) std::rotate(reordered.begin(), reordered.begin() + 2, reordered.end());
        tree->Build(reordered);
        passed = canonical_nearest(*tree, source, query, 5, "corner Build") && passed;
        passed = canonical_nearest(*tree, source, query, 7, "corner full tie order") && passed;
        Tree::PointVector snapshot;
        tree->Snapshot(snapshot);
        if (!same_points(source, snapshot)) return false;
        tree->Build(snapshot);
        passed = canonical_nearest(*tree, source, query, 5, "corner Snapshot/Build") && passed;
    }
    return passed;
}

bool stable_neighbors_across_async_rebuild()
{
    auto points = corner_points();
    const auto query = points.front();
    auto distant = make_points(0, Multi_Thread_Rebuild_Point_Num + 100);
    for (auto &point : distant)
    {
        point.x += 20.0F; point.y += 20.0F; point.z += 20.0F;
    }
    points.insert(points.end(), distant.begin(), distant.end());
    auto tree = std::make_unique<Tree>();
    tree->Build(points);
    bool passed = canonical_nearest(*tree, points, query, 5, "before async rebuild");
    TreeTestAccess::pause_after_flatten(*tree, true);
    TreeTestAccess::queue_root(*tree);
    if (!TreeTestAccess::wait_flattened(*tree, 1, std::chrono::seconds(5)))
    {
        TreeTestAccess::pause_after_flatten(*tree, false);
        std::cerr << "equal-distance async rebuild did not reach its real flatten phase\n";
        return false;
    }
    passed = canonical_nearest(*tree, points, query, 5, "during async rebuild") && passed;
    TreeTestAccess::pause_after_flatten(*tree, false);
    if (!TreeTestAccess::wait_completed(*tree, 1, std::chrono::seconds(5)))
    {
        std::cerr << "equal-distance async rebuild did not commit\n";
        return false;
    }
    passed = canonical_nearest(*tree, points, query, 5, "after async rebuild") && passed;
    Tree::PointVector snapshot;
    tree->Snapshot(snapshot);
    if (!same_points(points, snapshot))
    {
        std::cerr << "equal-distance rebuild changed the input geometry\n";
        return false;
    }
    return passed;
}

bool unequal_near_distances_keep_distance_priority()
{
    const pcl::PointXYZ query(0.0F, 0.0F, 0.0F);
    Tree::PointVector points{
        pcl::PointXYZ(0.01F, 0.0F, 0.0F),
        pcl::PointXYZ(-std::nextafter(0.01F, INFINITY), 0.0F, 0.0F)};
    const float nearer = points[0].x * points[0].x;
    const float farther = points[1].x * points[1].x;
    if (!(nearer < farther && farther - nearer < 1e-10F))
    {
        std::cerr << "near-distance fixture did not straddle the former epsilon tie\n";
        return false;
    }
    auto tree = std::make_unique<Tree>();
    bool passed = true;
    for (int order = 0; order < 2; ++order)
    {
        if (order) std::reverse(points.begin(), points.end());
        tree->Build(points);
        passed = canonical_nearest(*tree, points, query, 2,
                                   "different distances inside former epsilon") && passed;
    }
    return passed;
}

bool stable_voxel_representative()
{
    const pcl::PointXYZ center(0.0625F, 0.0625F, 0.0625F);
    Tree::PointVector source;
    for (int axis = 0; axis < 3; ++axis)
        for (float sign : {-1.0F, 1.0F})
        {
            auto point = center;
            if (axis == 0) point.x += sign * 0.03125F;
            else if (axis == 1) point.y += sign * 0.03125F;
            else point.z += sign * 0.03125F;
            source.push_back(point);
        }
    const pcl::PointXYZ incoming_point(0.015625F, 0.015625F, 0.015625F);
    auto candidates = source;
    candidates.push_back(incoming_point);
    auto tree = std::make_unique<Tree>();
    tree->set_downsample_param(0.125F);
    bool passed = true;
    for (int order = 0; order < 6; ++order)
        for (bool rebuild : {false, true})
        {
            auto points = source;
            std::rotate(points.begin(), points.begin() + order, points.end());
            tree->Build(points);
            if (rebuild)
            {
                Tree::PointVector snapshot;
                tree->Snapshot(snapshot);
                if (!same_points(source, snapshot)) return false;
                tree->Build(snapshot);
            }
            Tree::PointVector incoming{incoming_point};
            tree->Add_Points(incoming, true);
            Tree::PointVector kept;
            tree->Snapshot(kept);
            if (kept.size() != 1)
            {
                std::cerr << "voxel representative count differs: " << kept.size() << '\n';
                return false;
            }
            // The brute-force distance/XYZ oracle includes the incoming point,
            // so either insertion order or traversal-order winner selection
            // is detected. Its unique canonical representative is
            // (0.03125, 0.0625, 0.0625), with no approximate tie comparison.
            passed = canonical_nearest(*tree, candidates, center, 1,
                rebuild ? "voxel Snapshot/Build representative" : "voxel Build representative") && passed;
        }
    return passed;
}

bool stable_office_cutoff_neighbors()
{
    // Office1, 1774195436.2404275, iteration 2, point 1: the first four
    // neighbors agree, while the fifth candidates tied with non-fused float
    // arithmetic. ARM FMA can distinguish them by one ULP; keep the recorded
    // geometry and check its exact distance/XYZ order on either platform.
    // Binary-exact corner fixtures separately require genuine distance ties.
    // Recorded pose/velocity and query agreed;
    // the trace did not record full prior covariance or bias state.
    const pcl::PointXYZ query(-5.0920877456665039F, 3.0569560527801514F, 0.53805756568908691F);
    const Tree::PointVector source{
        pcl::PointXYZ(-5.088346004486084F, 3.041006326675415F, 0.52612859010696411F),
        pcl::PointXYZ(-5.0790390968322754F, 3.0597071647644043F, 0.5208442211151123F),
        pcl::PointXYZ(-5.121025562286377F, 3.0457756519317627F, 0.52539926767349243F),
        pcl::PointXYZ(-5.0559577941894531F, 3.0430610179901123F, 0.52631258964538574F),
        pcl::PointXYZ(-5.0675106048583984F, 3.0442454814910889F, 0.56833255290985107F),
        pcl::PointXYZ(-5.0889315605163574F, 3.0601253509521484F, 0.57882720232009888F)};
    const auto distance = [&](const auto &point) {
        const float x = point.x - query.x;
        const float y = point.y - query.y;
        const float z = point.z - query.z;
        return x * x + y * y + z * z;
    };
    const float first_cutoff = distance(source[4]);
    const float second_cutoff = distance(source[5]);
    const float cutoff = std::min(first_cutoff, second_cutoff);
    if (!std::isfinite(first_cutoff) || !std::isfinite(second_cutoff) || cutoff <= 0.0F ||
        !(coordinates(source[5]) < coordinates(source[4])))
    {
        std::cerr << "Office fixture has invalid finite cutoff candidates\n";
        return false;
    }
    for (std::size_t i = 0; i < 4; ++i)
        if (!(distance(source[i]) < cutoff)) return false;
    auto tree = std::make_unique<Tree>();
    bool passed = true;
    for (int order = 0; order < 6; ++order)
    {
        auto reordered = source;
        std::rotate(reordered.begin(), reordered.begin() + order, reordered.end());
        tree->Build(reordered);
        passed = canonical_nearest(*tree, source, query, 5, "Office cutoff Build") && passed;
        Tree::PointVector snapshot;
        tree->Snapshot(snapshot);
        if (!same_points(source, snapshot)) return false;
        tree->Build(snapshot);
        passed = canonical_nearest(*tree, source, query, 5, "Office cutoff Snapshot/Build") && passed;
    }
    return passed;
}

bool reused_neighbor_outputs_match_each_query()
{
    const Tree::PointVector source{
        pcl::PointXYZ(-3.0F, 0.0F, 0.0F), pcl::PointXYZ(-1.0F, 0.0F, 0.0F),
        pcl::PointXYZ(-0.5F, 0.0F, 0.0F), pcl::PointXYZ(0.25F, 0.0F, 0.0F),
        pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
        pcl::PointXYZ(4.0F, 0.0F, 0.0F)};
    const pcl::PointXYZ query(0.0F, 0.0F, 0.0F);
    auto tree = std::make_unique<Tree>();
    tree->Build(source);
    Tree::PointVector actual(17, pcl::PointXYZ(99.0F, 99.0F, 99.0F));
    std::vector<float> distances(17, -1.0F);
    using Candidate = std::tuple<float, float, float, float>;
    const std::vector<std::pair<int, double>> requests{
        {7, INFINITY}, {2, INFINITY}, {7, 0.5}, {7, 0.1}, {9, INFINITY}};
    for (std::size_t step = 0; step < requests.size(); ++step)
    {
        const auto [count, radius] = requests[step];
        std::vector<Candidate> expected;
        for (const auto &point : source)
        {
            const float x = point.x - query.x;
            const float y = point.y - query.y;
            const float z = point.z - query.z;
            const float squared_distance = x * x + y * y + z * z;
            if (squared_distance <= radius * radius)
                expected.emplace_back(squared_distance, point.x, point.y, point.z);
        }
        std::sort(expected.begin(), expected.end());
        expected.resize(std::min(expected.size(), static_cast<std::size_t>(count)));
        tree->Nearest_Search(query, count, actual, distances, radius);
        if (actual.size() != expected.size() || distances.size() != expected.size())
        {
            std::cerr << "reused KNN outputs retained an incorrect size at step " << step << '\n';
            return false;
        }
        for (std::size_t i = 0; i < expected.size(); ++i)
        {
            const Candidate found{distances[i], actual[i].x, actual[i].y, actual[i].z};
            if (found != expected[i])
            {
                std::cerr << "reused KNN outputs differ from brute force at step " << step
                          << " neighbor " << i << '\n';
                return false;
            }
        }
    }
    return true;
}

bool alternating_trees_keep_queries_independent()
{
    auto first = std::make_unique<Tree>();
    auto second = std::make_unique<Tree>();
    const auto first_points = make_points(0, 192);
    auto second_points = make_points(500, 730);
    for (auto &point : second_points)
    {
        point.x += 1000.0F;
        point.z += 4.0F;
    }
    first->Build(first_points);
    second->Build(second_points);
    const auto first_query = first_points[77];
    const auto second_query = second_points[121];
    bool passed = true;
    // All calls run on this same thread. Alternating disjoint maps and growing
    // then shrinking k exposes stale candidates regardless of scratch storage.
    for (int count : {1, 5, 30, 2, 60, 160, 3})
    {
        passed = canonical_nearest(*first, first_points, first_query, count,
                                   "alternating first tree") && passed;
        passed = canonical_nearest(*second, second_points, second_query, count,
                                   "alternating second tree") && passed;
    }
    passed = canonical_nearest(*first, first_points, first_query, 60,
                               "first tree radius has fewer than k", 0.05) && passed;
    passed = canonical_nearest(*second, second_points, second_query, 2,
                               "second tree after radius restriction") && passed;
    auto outside = first_query;
    outside.z += 1.0F;
    passed = canonical_nearest(*first, first_points, outside, 5,
                               "first tree empty radius result", 0.1) && passed;
    passed = canonical_nearest(*second, second_points, second_query, 160,
                               "second tree after empty result") && passed;

    Tree::PointVector empty;
    first->Build(empty);
    passed = canonical_nearest(*first, empty, first_query, 60,
                               "first tree cleared") && passed;
    passed = canonical_nearest(*second, second_points, second_query, 5,
                               "second tree after other tree cleared") && passed;
    auto rebuilt_points = make_points(400, 592);
    for (auto &point : rebuilt_points)
    {
        point.x -= 20.0F;
        point.y += 7.0F;
        point.z -= 3.0F;
    }
    first->Build(rebuilt_points);
    passed = canonical_nearest(*first, rebuilt_points, rebuilt_points[73], 160,
                               "first tree rebuilt with different geometry") && passed;
    passed = canonical_nearest(*second, second_points, second_query, 2,
                               "second tree after other tree rebuilt") && passed;
    passed = canonical_nearest(*first, rebuilt_points, first_query, 30,
                               "old first-tree geometry absent", 0.1) && passed;
    return passed;
}

bool clean_queries_do_not_wait_for_working_lock()
{
    const Tree::PointVector source{
        pcl::PointXYZ(-0.25F, 0.0F, 0.0F), pcl::PointXYZ(0.5F, 0.0F, 0.0F),
        pcl::PointXYZ(1.0F, 0.0F, 0.0F)};
    const pcl::PointXYZ query(0.0F, 0.0F, 0.0F);
    auto tree = std::make_unique<Tree>();
    tree->Build(source);
    TreeTestAccess::pause_searches(*tree, true);
    auto working_lock = TreeTestAccess::hold_working_lock(*tree);
    std::vector<std::thread> readers;
    std::vector<std::future<bool>> results;
    for (int reader = 0; reader < 3; ++reader)
    {
        std::promise<bool> completion;
        results.push_back(completion.get_future());
        readers.emplace_back([&, completion = std::move(completion)]() mutable {
            completion.set_value(canonical_nearest(*tree, source, query, 3, "clean concurrent query"));
        });
    }
    const bool all_entered = TreeTestAccess::wait_searches_in_flight(
        *tree, 3, std::chrono::seconds(5));
    TreeTestAccess::pause_searches(*tree, false);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool completed_while_locked = all_entered;
    for (auto &result : results)
        if (result.wait_until(deadline) != std::future_status::ready)
            completed_while_locked = false;
    // Release before joining even on failure, so the old blocking behavior
    // produces a bounded regression failure instead of hanging the suite.
    working_lock.unlock();
    for (auto &reader : readers) reader.join();
    bool correct = true;
    for (auto &result : results) correct = result.get() && correct;
    if (!completed_while_locked)
        std::cerr << "clean concurrent queries waited for the unrelated working lock\n";
    return completed_while_locked && correct;
}

bool pruned_child_does_not_block_nearest_query()
{
    const Tree::PointVector source{
        pcl::PointXYZ(-10.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 0.0F, 0.0F),
        pcl::PointXYZ(10.0F, 0.0F, 0.0F)};
    const auto query = source.front();
    auto tree = std::make_unique<Tree>();
    tree->Build(source);
    TreeTestAccess::pause_searches(*tree, true);
    auto far_child_lock = TreeTestAccess::hold_right_child_lock(*tree);
    std::promise<bool> completion;
    auto result = completion.get_future();
    std::thread reader([&] {
        completion.set_value(canonical_nearest(*tree, source, query, 1, "pruned child query"));
    });
    const bool entered = TreeTestAccess::wait_searches_in_flight(
        *tree, 1, std::chrono::seconds(5));
    TreeTestAccess::pause_searches(*tree, false);
    const bool completed_while_locked =
        result.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    far_child_lock.unlock();
    reader.join();
    const bool correct = result.get();
    if (!entered || !completed_while_locked)
        std::cerr << "nearest query waited for a pruned child's mutex\n";
    return entered && completed_while_locked && correct;
}

bool last_reader_wakes_waiting_snapshot()
{
    const auto source = make_points(0, 32);
    const auto query = source.front();
    auto tree = std::make_unique<Tree>();
    tree->Build(source);
    TreeTestAccess::pause_searches(*tree, true);
    std::vector<std::thread> readers;
    std::vector<std::future<bool>> results;
    for (int reader = 0; reader < 3; ++reader)
    {
        std::promise<bool> completion;
        results.push_back(completion.get_future());
        readers.emplace_back([&, completion = std::move(completion)]() mutable {
            completion.set_value(canonical_nearest(*tree, source, query, 5, "snapshot waiting query"));
        });
    }
    const bool all_entered = TreeTestAccess::wait_searches_in_flight(
        *tree, 3, std::chrono::seconds(5));
    std::promise<Tree::PointVector> snapshot_completion;
    auto snapshot_result = snapshot_completion.get_future();
    std::thread snapshot_writer([&] {
        Tree::PointVector snapshot;
        tree->Snapshot(snapshot);
        snapshot_completion.set_value(std::move(snapshot));
    });
    // Observe the real exclusive-wait predicate before releasing the readers;
    // thread creation or a delay alone would not establish this interleaving.
    const bool writer_waiting = TreeTestAccess::wait_for_pending_writer(
        *tree, std::chrono::seconds(5));
    TreeTestAccess::pause_searches(*tree, false);
    for (auto &reader : readers) reader.join();
    const bool snapshot_woke =
        snapshot_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    // On a missed notification the predicate is now true. Wake it only for
    // cleanup, after recording failure, so the worker can always be joined.
    if (!snapshot_woke) TreeTestAccess::wake_search_waiters(*tree);
    snapshot_writer.join();
    bool correct = same_points(source, snapshot_result.get());
    for (auto &result : results) correct = result.get() && correct;
    if (!all_entered || !writer_waiting || !snapshot_woke || !correct)
        std::cerr << "last-reader snapshot wake failed: readers=" << all_entered
                  << " waiting=" << writer_waiting << " woke=" << snapshot_woke
                  << " geometry=" << correct << '\n';
    return all_entered && writer_waiting && snapshot_woke && correct;
}

}  // namespace

int main()
{
    const bool stable_corner = stable_corner_neighbors();
    const bool stable_async = stable_neighbors_across_async_rebuild();
    const bool distance_priority = unequal_near_distances_keep_distance_priority();
    const bool stable_voxel = stable_voxel_representative();
    const bool stable_office = stable_office_cutoff_neighbors();
    const bool reused_outputs = reused_neighbor_outputs_match_each_query();
    const bool independent_queries = alternating_trees_keep_queries_independent();
    const bool clean_queries = clean_queries_do_not_wait_for_working_lock();
    const bool pruned_child = pruned_child_does_not_block_nearest_query();
    const bool snapshot_wake = last_reader_wakes_waiting_snapshot();
    if (!stable_corner || !stable_async || !distance_priority || !stable_voxel ||
        !stable_office || !reused_outputs || !independent_queries || !clean_queries ||
        !pruned_child || !snapshot_wake) return 1;

    // A map-save snapshot may run while the tree's own rebuild worker is
    // between flattening and commit. It must retain exactly the live points.
    auto snapshot_tree = std::make_unique<Tree>();
    auto snapshot_points = make_points(0, 2000);
    snapshot_tree->Build(snapshot_points);
    snapshot_tree->Set_balance_criterion_param(0.51F);
    TreeTestAccess::pause_before_start(*snapshot_tree, true);
    TreeTestAccess::pause_after_flatten(*snapshot_tree, true);
    auto snapshot_tail = make_unbalanced_tail(400);
    snapshot_tree->Add_Points(snapshot_tail, false);
    if (!TreeTestAccess::wait_queued(*snapshot_tree, std::chrono::seconds(5)))
        return 1;
    TreeTestAccess::pause_before_start(*snapshot_tree, false);
    if (!TreeTestAccess::wait_flattened(*snapshot_tree, 1, std::chrono::seconds(5)))
        return 1;
    Tree::PointVector during_rebuild;
    snapshot_tree->Snapshot(during_rebuild);
    TreeTestAccess::pause_after_flatten(*snapshot_tree, false);
    if (!TreeTestAccess::wait_completed(*snapshot_tree, 1, std::chrono::seconds(5)))
        return 1;
    Tree::PointVector after_rebuild;
    snapshot_tree->Snapshot(after_rebuild);
    const auto point_less = [](const auto& a, const auto& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    snapshot_points.insert(snapshot_points.end(), snapshot_tail.begin(), snapshot_tail.end());
    std::sort(snapshot_points.begin(), snapshot_points.end(), point_less);
    for (auto* snapshot : {&during_rebuild, &after_rebuild}) {
        std::sort(snapshot->begin(), snapshot->end(), point_less);
        if (snapshot->size() != snapshot_points.size()) return 1;
        for (std::size_t i = 0; i < snapshot->size(); ++i)
            if ((*snapshot)[i].x != snapshot_points[i].x ||
                (*snapshot)[i].y != snapshot_points[i].y ||
                (*snapshot)[i].z != snapshot_points[i].z) return 1;
    }
    snapshot_tree.reset();

    auto tree = std::make_unique<Tree>();
    auto initial_points = make_points(0, 1490);
    tree->Build(initial_points);
    if (!nearest_is_exact(*tree, initial_points[777]))
        return 1;

    auto added_points = make_points(1490, 1750);
    Tree::PointVector deleted_points(initial_points.begin(), initial_points.begin() + 25);
    tree->Add_Points(added_points, false);
    tree->Delete_Points(deleted_points);
    if (tree->size() < 1500 || !nearest_is_exact(*tree, initial_points[777]))
        return 1;

    tree.reset();

    auto split_tie_tree = std::make_unique<Tree>();
    Tree::PointVector split_tie_points;
    for (const auto &[x, y] : std::vector<std::pair<float, float>>{
             {0.0F, 0.0F}, {1.0F, 0.1F}, {1.0F, 0.2F},
             {1.0F, 0.3F}, {2.0F, 0.0F}})
    {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = 0.0F;
        split_tie_points.push_back(point);
    }
    split_tie_tree->Build(split_tie_points);
    pcl::PointXYZ split_tie_point;
    if (!TreeTestAccess::left_split_tie_point(*split_tie_tree, split_tie_point))
    {
        std::cerr << "split-tie fixture did not place an equal-axis point in the left subtree\n";
        return 1;
    }
    Tree::PointVector split_tie_delete{split_tie_point};
    split_tie_tree->Delete_Points(split_tie_delete);
    if (nearest_is_exact(*split_tie_tree, split_tie_point))
    {
        std::cerr << "equal-axis point in the left subtree was not deleted\n";
        return 1;
    }
    split_tie_tree.reset();

    auto working_flag_tree = std::make_unique<Tree>();
    auto working_flag_points = make_points(0, 5000);
    working_flag_tree->Build(working_flag_points);
    Tree::PointVector exact_delete{working_flag_tree->Root_Node->point};
    working_flag_tree->Delete_Points(exact_delete);
    const bool exact_delete_flags_clear =
        TreeTestAccess::all_working_flags_clear(*working_flag_tree);
    BoxPointType outside_box{};
    for (int axis = 0; axis < 3; ++axis)
    {
        outside_box.vertex_min[axis] = 3000.0F;
        outside_box.vertex_max[axis] = 4000.0F;
    }
    std::vector<BoxPointType> outside_boxes{outside_box};
    working_flag_tree->Delete_Point_Boxes(outside_boxes);
    const bool delete_range_miss_flags_clear =
        TreeTestAccess::all_working_flags_clear(*working_flag_tree);
    working_flag_tree->Add_Point_Boxes(outside_boxes);
    const bool add_range_miss_flags_clear =
        TreeTestAccess::all_working_flags_clear(*working_flag_tree);
    std::vector<BoxPointType> full_boxes{whole_tree_box()};
    working_flag_tree->Delete_Point_Boxes(full_boxes);
    const bool delete_range_full_flags_clear =
        TreeTestAccess::all_working_flags_clear(*working_flag_tree);
    working_flag_tree->Add_Point_Boxes(full_boxes);
    const bool add_range_full_flags_clear =
        TreeTestAccess::all_working_flags_clear(*working_flag_tree);
    working_flag_tree.reset();

    auto ancestor_metadata_tree = std::make_unique<Tree>();
    auto ancestor_metadata_points = make_points(0, 10000);
    ancestor_metadata_tree->Build(ancestor_metadata_points);
    auto ancestor_fixture =
        TreeTestAccess::left_ancestor_replay_fixture(*ancestor_metadata_tree);
    Tree::PointVector ancestor_delete{ancestor_fixture.ancestor_point};
    ancestor_metadata_tree->Delete_Points(ancestor_delete);
    const bool ancestor_delete_flags_clear =
        TreeTestAccess::all_working_flags_clear(*ancestor_metadata_tree);

    TreeTestAccess::pause_before_start(*ancestor_metadata_tree, true);
    TreeTestAccess::pause_after_flatten(*ancestor_metadata_tree, true);
    TreeTestAccess::queue_rebuild(*ancestor_metadata_tree, ancestor_fixture.target_slot);
    if (!TreeTestAccess::wait_queued(*ancestor_metadata_tree, std::chrono::seconds(5)))
    {
        std::cerr << "working-flag descendant rebuild was not queued\n";
        return 1;
    }
    TreeTestAccess::pause_before_start(*ancestor_metadata_tree, false);
    if (!TreeTestAccess::wait_flattened(*ancestor_metadata_tree, 1,
                                        std::chrono::seconds(5)))
    {
        std::cerr << "working-flag descendant rebuild did not flatten\n";
        return 1;
    }
    pcl::PointXYZ metadata_replay_add = (*ancestor_fixture.target_slot)->point;
    metadata_replay_add.z += 0.03125F;
    TreeTestAccess::enqueue_add(*ancestor_metadata_tree, metadata_replay_add);
    TreeTestAccess::pause_after_flatten(*ancestor_metadata_tree, false);
    const bool ancestor_rebuild_completed = TreeTestAccess::wait_completed(
        *ancestor_metadata_tree, 1, std::chrono::seconds(5));
    const int replayed_subtree_size =
        TreeTestAccess::subtree_size(ancestor_fixture.target_slot);
    const int replayed_root_size = ancestor_metadata_tree->size();
    const bool descendant_metadata_updated =
        replayed_subtree_size == ancestor_fixture.target_size + 1 &&
        replayed_root_size == ancestor_fixture.root_size + 1;
    if (!exact_delete_flags_clear || !delete_range_miss_flags_clear ||
        !add_range_miss_flags_clear || !delete_range_full_flags_clear ||
        !add_range_full_flags_clear || !ancestor_delete_flags_clear ||
        !ancestor_rebuild_completed || !descendant_metadata_updated)
    {
        std::cerr << "working-flag lifetime failed: exact_clear="
                  << exact_delete_flags_clear
                  << " delete_range_miss_clear=" << delete_range_miss_flags_clear
                  << " add_range_miss_clear=" << add_range_miss_flags_clear
                  << " delete_range_full_clear=" << delete_range_full_flags_clear
                  << " add_range_full_clear=" << add_range_full_flags_clear
                  << " ancestor_clear=" << ancestor_delete_flags_clear
                  << " rebuild_completed=" << ancestor_rebuild_completed
                  << " subtree_size=" << replayed_subtree_size
                  << " expected_subtree_size=" << ancestor_fixture.target_size + 1
                  << " root_size=" << replayed_root_size
                  << " expected_root_size=" << ancestor_fixture.root_size + 1 << '\n';
        return 1;
    }
    ancestor_metadata_tree.reset();

    auto lifecycle_tree = std::make_unique<Tree>();
    auto lifecycle_points = make_points(0, 2000);
    auto unbalanced_tail = make_unbalanced_tail(400);
    lifecycle_tree->Build(lifecycle_points);
    lifecycle_tree->Set_balance_criterion_param(0.51F);
    TreeTestAccess::pause_before_start(*lifecycle_tree, true);
    lifecycle_tree->Add_Points(unbalanced_tail, false);
    if (!TreeTestAccess::wait_queued(*lifecycle_tree, std::chrono::seconds(5)))
        return 1;
    TreeTestAccess::pause_before_start(*lifecycle_tree, false);
    if (!TreeTestAccess::wait_started(*lifecycle_tree, 1, std::chrono::seconds(5)) ||
        !TreeTestAccess::wait_completed(*lifecycle_tree, 1, std::chrono::seconds(5)))
        return 1;

    const auto completed = TreeTestAccess::diagnostics(*lifecycle_tree);
    if (completed.started < 1 || completed.completed < 1 || completed.active)
        return 1;

    auto concurrent_read_tree = std::make_unique<Tree>();
    auto concurrent_points = make_points(0, 5000);
    concurrent_read_tree->Build(concurrent_points);
    concurrent_read_tree->Set_balance_criterion_param(0.51F);
    TreeTestAccess::pause_before_start(*concurrent_read_tree, true);
    TreeTestAccess::pause_after_start(*concurrent_read_tree, true);
    concurrent_read_tree->Add_Points(unbalanced_tail, false);
    if (!TreeTestAccess::wait_queued(*concurrent_read_tree, std::chrono::seconds(5)))
    {
        std::cerr << "concurrent rebuild was not queued\n";
        return 1;
    }
    TreeTestAccess::pause_before_start(*concurrent_read_tree, false);
    if (!TreeTestAccess::wait_active(*concurrent_read_tree, std::chrono::seconds(5)))
    {
        std::cerr << "concurrent rebuild did not become active\n";
        return 1;
    }

    std::atomic<bool> stop_readers{false};
    std::atomic<bool> read_failed{false};
    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        readers.emplace_back([&, reader] {
            int query_index = reader + 100;
            while (!stop_readers.load(std::memory_order_acquire))
            {
                if (!nearest_is_exact(*concurrent_read_tree, concurrent_points[query_index]) ||
                    !range_queries_find_exact(
                        *concurrent_read_tree, concurrent_points[query_index]))
                    read_failed.store(true, std::memory_order_release);
                query_index = (query_index + 97) % static_cast<int>(concurrent_points.size());
                if (query_index == 0)
                    query_index = 1;
            }
        });
    }
    TreeTestAccess::pause_after_start(*concurrent_read_tree, false);
    TreeTestAccess::wait_completed_without_timeout(*concurrent_read_tree, 1);
    stop_readers.store(true, std::memory_order_release);
    for (auto &reader : readers)
        reader.join();
    if (read_failed.load(std::memory_order_acquire))
    {
        std::cerr << "concurrent nearest failed\n";
        return 1;
    }
    concurrent_read_tree.reset();

    auto lazy_tree = std::make_unique<Tree>();
    lazy_tree->Build(concurrent_points);
    auto lazy_query = TreeTestAccess::lazy_non_leaf_point(*lazy_tree);
    std::vector<BoxPointType> lazy_delete{
        TreeTestAccess::lazy_non_leaf_box(*lazy_tree)};
    lazy_tree->Delete_Point_Boxes(lazy_delete);
    if (!TreeTestAccess::lazy_flags_present(*lazy_tree))
        return 1;
    TreeTestAccess::pause_searches(*lazy_tree, true);
    std::vector<std::thread> lazy_readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        lazy_readers.emplace_back([&] {
            Tree::PointVector nearest;
            std::vector<float> distances;
            lazy_tree->Nearest_Search(lazy_query, 1, nearest, distances, INFINITY);
        });
    }
    TreeTestAccess::wait_searches_in_flight(*lazy_tree, 4);
    TreeTestAccess::pause_searches(*lazy_tree, false);
    for (auto &reader : lazy_readers)
        reader.join();
    if (TreeTestAccess::max_searches_in_flight(*lazy_tree) < 2 ||
        !TreeTestAccess::lazy_delete_propagated(*lazy_tree))
        return 1;
    lazy_tree.reset();

    auto non_root_tree = std::make_unique<Tree>();
    non_root_tree->Build(concurrent_points);
    TreeTestAccess::pause_before_start(*non_root_tree, true);
    TreeTestAccess::pause_after_flatten(*non_root_tree, true);
    pcl::PointXYZ same_path_point = TreeTestAccess::queue_non_root(*non_root_tree);
    if (!TreeTestAccess::wait_queued(*non_root_tree, std::chrono::seconds(5)))
        return 1;
    TreeTestAccess::pause_before_start(*non_root_tree, false);
    if (!TreeTestAccess::wait_flattened(*non_root_tree, 1, std::chrono::seconds(5)))
        return 1;
    Tree::PointVector same_path_delete{same_path_point};
    pcl::PointXYZ same_path_add = same_path_point;
    same_path_add.z += 0.03125F;
    Tree::PointVector same_path_adds{same_path_add};
    non_root_tree->Delete_Points(same_path_delete);
    non_root_tree->Add_Points(same_path_adds, false);
    TreeTestAccess::pause_after_flatten(*non_root_tree, false);
    const bool non_root_completed =
        TreeTestAccess::wait_completed(*non_root_tree, 1, std::chrono::seconds(5));
    const bool old_point_present = nearest_is_exact(*non_root_tree, same_path_point);
    const bool new_point_present = nearest_is_exact(*non_root_tree, same_path_add);
    if (!non_root_completed || old_point_present || !new_point_present)
    {
        std::cerr << "non-root replay failed: completed=" << non_root_completed
                  << " old_point_present=" << old_point_present
                  << " new_point_present=" << new_point_present << '\n';
        return 1;
    }
    non_root_tree.reset();

    auto empty_replay_tree = std::make_unique<Tree>();
    empty_replay_tree->Build(concurrent_points);
    TreeTestAccess::pause_before_start(*empty_replay_tree, true);
    TreeTestAccess::pause_after_flatten(*empty_replay_tree, true);
    TreeTestAccess::queue_root(*empty_replay_tree);
    if (!TreeTestAccess::wait_queued(*empty_replay_tree, std::chrono::seconds(5)))
        return 1;
    std::vector<BoxPointType> empty_snapshot_delete{whole_tree_box()};
    empty_replay_tree->Delete_Point_Boxes(empty_snapshot_delete);
    TreeTestAccess::pause_before_start(*empty_replay_tree, false);
    if (!TreeTestAccess::wait_flattened(*empty_replay_tree, 1, std::chrono::seconds(5)))
        return 1;
    pcl::PointXYZ replay_add{};
    replay_add.x = -3000.0F;
    Tree::PointVector replay_adds{replay_add};
    TreeTestAccess::enqueue_push_down(*empty_replay_tree);
    empty_replay_tree->Add_Points(replay_adds, false);
    TreeTestAccess::pause_after_flatten(*empty_replay_tree, false);
    if (!TreeTestAccess::wait_completed(*empty_replay_tree, 1, std::chrono::seconds(5)) ||
        !nearest_is_exact(*empty_replay_tree, replay_add))
        return 1;
    empty_replay_tree.reset();

    auto empty_rebuild_tree = std::make_unique<Tree>();
    empty_rebuild_tree->Build(lifecycle_points);
    empty_rebuild_tree->Set_balance_criterion_param(0.51F);
    TreeTestAccess::pause_before_start(*empty_rebuild_tree, true);
    empty_rebuild_tree->Add_Points(unbalanced_tail, false);
    if (!TreeTestAccess::wait_queued(*empty_rebuild_tree, std::chrono::seconds(5)))
        return 1;
    std::vector<BoxPointType> delete_everything{whole_tree_box()};
    empty_rebuild_tree->Delete_Point_Boxes(delete_everything);
    TreeTestAccess::pause_before_start(*empty_rebuild_tree, false);
    if (!TreeTestAccess::wait_started(*empty_rebuild_tree, 1, std::chrono::seconds(5)) ||
        !TreeTestAccess::wait_completed(*empty_rebuild_tree, 1, std::chrono::seconds(5)) ||
        empty_rebuild_tree->size() != 0)
        return 1;
    empty_rebuild_tree.reset();

    lifecycle_tree->Build(lifecycle_points);
    TreeTestAccess::pause_before_start(*lifecycle_tree, true);
    lifecycle_tree->Add_Points(unbalanced_tail, false);
    if (!TreeTestAccess::wait_queued(*lifecycle_tree, std::chrono::seconds(5)))
        return 1;

    Tree::PointVector empty;
    lifecycle_tree->Build(empty);
    const auto after_empty_build = TreeTestAccess::diagnostics(*lifecycle_tree);
    if (lifecycle_tree->size() != 0 || after_empty_build.active ||
        after_empty_build.canceled < 1 ||
        after_empty_build.queued != after_empty_build.started + after_empty_build.canceled)
        return 1;
    lifecycle_tree->Build(initial_points);
    if (!nearest_is_exact(*lifecycle_tree, initial_points[777]))
        return 1;

    lifecycle_tree.reset();

    auto shutdown_tree = std::make_unique<Tree>();
    auto shutdown_points = make_points(0, 5000);
    shutdown_tree->Build(shutdown_points);
    shutdown_tree->Set_balance_criterion_param(0.51F);
    TreeTestAccess::pause_before_start(*shutdown_tree, true);
    TreeTestAccess::pause_after_start(*shutdown_tree, true);
    auto shutdown_tail = make_unbalanced_tail(600);
    shutdown_tree->Add_Points(shutdown_tail, false);
    if (!TreeTestAccess::wait_queued(*shutdown_tree, std::chrono::seconds(5)))
        return 1;
    TreeTestAccess::pause_before_start(*shutdown_tree, false);
    if (!TreeTestAccess::wait_started(*shutdown_tree, 1, std::chrono::seconds(5)) ||
        !TreeTestAccess::wait_active(*shutdown_tree, std::chrono::seconds(5)))
        return 1;

    shutdown_tree.reset();  // Joins a rebuild observed in the active state.
    return 0;
}
