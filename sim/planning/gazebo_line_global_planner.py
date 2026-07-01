#!/usr/bin/env python3
"""Gazebo-room global path publisher for simulation gates.

The node is intentionally narrow: it converts Gazebo odometry, the latest
/nav/goal_pose, and the LiDAR-derived /nav/exploration_grid into a safe
/nav/global_path for the native pct_path_adapter -> localPlanner ->
pathFollower chain. It never publishes velocity commands.
"""

from __future__ import annotations

import math
from heapq import heappop, heappush


def main() -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.node import Node

    rclpy.init()
    node = Node("lingtu_gazebo_line_global_planner")
    node.declare_parameter("path_step_m", 0.20)
    node.declare_parameter("publish_hz", 2.0)
    node.declare_parameter("path_frame", "map")
    node.declare_parameter("goal_reached_radius_m", 0.45)
    node.declare_parameter("require_occupancy_grid", False)
    node.declare_parameter("allow_unknown_cells", False)
    node.declare_parameter("occupied_threshold", 50)
    node.declare_parameter("inflation_radius_m", 0.20)
    node.declare_parameter("goal_snap_radius_m", 0.85)
    path_step = max(0.05, float(node.get_parameter("path_step_m").value))
    publish_hz = max(0.2, float(node.get_parameter("publish_hz").value))
    path_frame = str(node.get_parameter("path_frame").value or "map")
    goal_reached_radius = max(
        0.05,
        float(node.get_parameter("goal_reached_radius_m").value),
    )
    require_occupancy_grid = bool(node.get_parameter("require_occupancy_grid").value)
    allow_unknown_cells = bool(node.get_parameter("allow_unknown_cells").value)
    occupied_threshold = int(node.get_parameter("occupied_threshold").value)
    inflation_radius_m = max(
        0.0,
        float(node.get_parameter("inflation_radius_m").value),
    )
    goal_snap_radius_m = max(
        0.0,
        float(node.get_parameter("goal_snap_radius_m").value),
    )

    state: dict[str, object | None] = {
        "pose": None,
        "goal": None,
        "grid": None,
    }
    reached = {"value": False}
    fallback_warned = {"value": False}
    waiting_for_grid_warned = {"value": False}
    missing_goal_warned = {"value": False}
    pub = node.create_publisher(Path, "/nav/global_path", 10)

    def on_odom(msg: Odometry) -> None:
        pos = msg.pose.pose.position
        state["pose"] = (float(pos.x), float(pos.y), float(pos.z))

    def on_goal(msg: PoseStamped) -> None:
        pos = msg.pose.position
        state["goal"] = (float(pos.x), float(pos.y), float(pos.z))
        reached["value"] = False
        missing_goal_warned["value"] = False
        publish_path()

    def on_grid(msg: OccupancyGrid) -> None:
        state["grid"] = msg
        waiting_for_grid_warned["value"] = False

    def world_to_cell(grid: OccupancyGrid, x: float, y: float) -> tuple[int, int] | None:
        res = float(grid.info.resolution)
        if res <= 0.0:
            return None
        col = int((x - float(grid.info.origin.position.x)) / res)
        row = int((y - float(grid.info.origin.position.y)) / res)
        if 0 <= row < int(grid.info.height) and 0 <= col < int(grid.info.width):
            return row, col
        return None

    def cell_to_world(grid: OccupancyGrid, row: int, col: int) -> tuple[float, float]:
        res = float(grid.info.resolution)
        return (
            float(grid.info.origin.position.x) + (float(col) + 0.5) * res,
            float(grid.info.origin.position.y) + (float(row) + 0.5) * res,
        )

    def build_blocked_grid(
        grid: OccupancyGrid,
        *,
        inflation_radius: float,
    ) -> list[list[bool]] | None:
        width = int(grid.info.width)
        height = int(grid.info.height)
        res = float(grid.info.resolution)
        data = list(grid.data)
        if width <= 0 or height <= 0 or res <= 0.0 or len(data) < width * height:
            return None

        blocked = [[False for _ in range(width)] for _ in range(height)]
        occupied: list[tuple[int, int]] = []
        for row in range(height):
            base = row * width
            for col in range(width):
                value = int(data[base + col])
                if value < 0 and not allow_unknown_cells:
                    blocked[row][col] = True
                elif value >= occupied_threshold:
                    blocked[row][col] = True
                    occupied.append((row, col))

        inflate_cells = int(math.ceil(max(0.0, inflation_radius) / res))
        if inflate_cells <= 0:
            return blocked
        inflated = [item[:] for item in blocked]
        for row, col in occupied:
            for dr in range(-inflate_cells, inflate_cells + 1):
                for dc in range(-inflate_cells, inflate_cells + 1):
                    if math.hypot(dr, dc) * res > inflation_radius:
                        continue
                    rr = row + dr
                    cc = col + dc
                    if 0 <= rr < height and 0 <= cc < width:
                        inflated[rr][cc] = True
        return inflated

    def nearest_open_cell(
        blocked: list[list[bool]],
        start: tuple[int, int],
        max_radius_cells: int,
    ) -> tuple[int, int] | None:
        height = len(blocked)
        width = len(blocked[0]) if height else 0
        row0, col0 = start
        if not (0 <= row0 < height and 0 <= col0 < width):
            return None
        if not blocked[row0][col0]:
            return row0, col0
        best: tuple[float, tuple[int, int]] | None = None
        for radius in range(1, max(1, max_radius_cells) + 1):
            for row in range(row0 - radius, row0 + radius + 1):
                for col in range(col0 - radius, col0 + radius + 1):
                    if not (0 <= row < height and 0 <= col < width):
                        continue
                    if blocked[row][col]:
                        continue
                    dist = math.hypot(row - row0, col - col0)
                    if dist > radius:
                        continue
                    if best is None or dist < best[0]:
                        best = (dist, (row, col))
            if best is not None:
                return best[1]
        return None

    def reconstruct_path(
        parents: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        cells = [current]
        while current in parents:
            current = parents[current]
            cells.append(current)
        cells.reverse()
        return cells

    def grid_path(
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]] | None:
        inflation_attempts = []
        for item in (inflation_radius_m, 0.0):
            if all(abs(item - existing) > 1e-6 for existing in inflation_attempts):
                inflation_attempts.append(item)
        for inflation in inflation_attempts:
            points = grid_path_with_inflation(grid, pose, goal, inflation)
            if points is not None:
                if inflation < inflation_radius_m and not fallback_warned["value"]:
                    fallback_warned["value"] = True
                    node.get_logger().warn(
                        "Gazebo grid global planner fell back to an uninflated "
                        "known-occupied avoidance path"
                    )
                return points
        return None

    def grid_path_with_inflation(
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
        inflation_radius: float,
    ) -> list[tuple[float, float, float]] | None:
        blocked = build_blocked_grid(grid, inflation_radius=inflation_radius)
        if blocked is None:
            return None
        res = float(grid.info.resolution)
        start_cell = world_to_cell(grid, pose[0], pose[1])
        goal_cell = world_to_cell(grid, goal[0], goal[1])
        if start_cell is None or goal_cell is None:
            return None
        start = nearest_open_cell(
            blocked,
            start_cell,
            max(1, int(math.ceil(0.5 / res))),
        )
        goal_open = nearest_open_cell(
            blocked,
            goal_cell,
            max(1, int(math.ceil(goal_snap_radius_m / res))),
        )
        if start is None or goal_open is None:
            return None

        height = len(blocked)
        width = len(blocked[0]) if height else 0
        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        heappush(open_heap, (0.0, 0, start))
        parents: dict[tuple[int, int], tuple[int, int]] = {}
        costs = {start: 0.0}
        closed: set[tuple[int, int]] = set()
        seq = 0
        neighbours = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        while open_heap:
            _, _, current = heappop(open_heap)
            if current in closed:
                continue
            if current == goal_open:
                cells = reconstruct_path(parents, current)
                points: list[tuple[float, float, float]] = []
                for row, col in cells:
                    wx, wy = cell_to_world(grid, row, col)
                    points.append((wx, wy, goal[2]))
                if points:
                    points[0] = (pose[0], pose[1], goal[2])
                return points
            closed.add(current)
            row, col = current
            for dr, dc, step_cost in neighbours:
                nr = row + dr
                nc = col + dc
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                if blocked[nr][nc]:
                    continue
                next_cell = (nr, nc)
                new_cost = costs[current] + step_cost
                if new_cost >= costs.get(next_cell, float("inf")):
                    continue
                costs[next_cell] = new_cost
                parents[next_cell] = current
                seq += 1
                heuristic = math.hypot(goal_open[0] - nr, goal_open[1] - nc)
                heappush(open_heap, (new_cost + heuristic, seq, next_cell))
        return None

    def straight_path_points(
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        sx, sy, _ = pose
        gx, gy, gz = goal
        distance = math.hypot(gx - sx, gy - sy)
        steps = max(2, int(math.ceil(distance / path_step)) + 1)
        return [
            (
                sx + (gx - sx) * (i / float(steps - 1)),
                sy + (gy - sy) * (i / float(steps - 1)),
                gz,
            )
            for i in range(steps)
        ]

    def publish_path() -> None:
        if reached["value"]:
            return
        pose = state["pose"]
        goal = state["goal"]
        if pose is None:
            return
        if goal is None:
            if not missing_goal_warned["value"]:
                missing_goal_warned["value"] = True
                node.get_logger().warn(
                    "Gazebo grid global planner has odometry but no /nav/goal_pose yet"
                )
            return
        assert isinstance(pose, tuple)
        assert isinstance(goal, tuple)
        sx, sy, _ = pose
        gx, gy, gz = goal
        distance = math.hypot(gx - sx, gy - sy)
        if distance <= goal_reached_radius:
            reached["value"] = True
            node.get_logger().info(
                "Gazebo line global planner reached goal; stopping path publication"
            )
            return
        points: list[tuple[float, float, float]] | None = None
        grid = state.get("grid")
        if isinstance(grid, OccupancyGrid):
            points = grid_path(grid, pose, goal)
            if points is None:
                node.get_logger().warn(
                    "Gazebo grid global planner has no safe occupancy path to goal; "
                    "suppressing path"
                )
                return
        elif require_occupancy_grid:
            if not waiting_for_grid_warned["value"]:
                waiting_for_grid_warned["value"] = True
                node.get_logger().warn(
                    "Gazebo grid global planner is waiting for /nav/exploration_grid"
                )
            return
        else:
            points = straight_path_points(pose, goal)

        path = Path()
        path.header.frame_id = path_frame
        path.header.stamp = node.get_clock().now().to_msg()
        for x, y, z in points:
            item = PoseStamped()
            item.header = path.header
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = z
            item.pose.orientation.w = 1.0
            path.poses.append(item)
        pub.publish(path)

    node.create_subscription(Odometry, "/slam/odometry", on_odom, 10)
    node.create_subscription(PoseStamped, "/nav/goal_pose", on_goal, 10)
    node.create_subscription(OccupancyGrid, "/nav/exploration_grid", on_grid, 2)
    node.create_timer(1.0 / publish_hz, publish_path)
    node.get_logger().info(
        "Gazebo grid global planner: /nav/exploration_grid + /nav/goal_pose "
        "+ /slam/odometry -> /nav/global_path"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
