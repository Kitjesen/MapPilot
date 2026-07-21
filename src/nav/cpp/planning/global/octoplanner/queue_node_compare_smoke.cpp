#include "global_planner.h"

#include <cstdio>
#include <queue>
#include <vector>

int main()
{
  using global_planner::GridIndex;
  using global_planner::QueueNode;
  using global_planner::QueueNodeCompare;

  const QueueNode farther_from_goal{GridIndex{1, 1, 0}, 10.0, 2.0};
  const QueueNode closer_to_goal{GridIndex{2, 2, 0}, 10.0, 8.0};
  const QueueNode lower_total_cost{GridIndex{3, 3, 0}, 9.0, 1.0};
  QueueNodeCompare compare;

  if (!compare(farther_from_goal, closer_to_goal) ||
      compare(closer_to_goal, farther_from_goal)) {
    std::fprintf(stderr, "equal-f nodes must prefer greater g (closer to goal)\n");
    return 1;
  }
  if (!compare(closer_to_goal, lower_total_cost) ||
      compare(lower_total_cost, closer_to_goal)) {
    std::fprintf(stderr, "lower f must remain the primary priority\n");
    return 1;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> queue;
  queue.push(farther_from_goal);
  queue.push(closer_to_goal);
  queue.push(lower_total_cost);
  if (queue.top().idx.x != lower_total_cost.idx.x) {
    std::fprintf(stderr, "priority queue did not preserve lowest-f priority\n");
    return 1;
  }
  queue.pop();
  if (queue.top().idx.x != closer_to_goal.idx.x) {
    std::fprintf(stderr, "priority queue did not apply equal-f goal tie-break\n");
    return 1;
  }

  std::puts("queue_node_compare_smoke passed");
  return 0;
}
