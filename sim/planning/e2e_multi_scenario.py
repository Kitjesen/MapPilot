#!/usr/bin/env python3
"""
E2E Multi-Scenario Navigation Test — 论文级数据收集
3 scenarios, 20Hz dense logging → /tmp/e2e_results/scenario_{id}.csv
"""
import math, os, csv, json, signal, sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# ── Scenario definitions ───────────────────────────────────────────────────────
SCENARIOS = [
    {"id": 1, "name": "diagonal_NE",  "goal": (5.0,  2.0), "timeout": 70.0, "thre": 0.3},
    {"id": 2, "name": "straight_E",   "goal": (6.0,  0.0), "timeout": 70.0, "thre": 0.3},
    {"id": 3, "name": "lateral_N",    "goal": (0.0,  5.0), "timeout": 90.0, "thre": 0.3},
]
OUT_DIR = "/tmp/e2e_results"
DT = 0.05        # 20 Hz
TERRAIN_R = 8.0
TERRAIN_S = 0.4
WARMUP_S  = 3.5  # wait before publishing global path


# ── helpers ───────────────────────────────────────────────────────────────────

def _terrain(cx, cy):
    pts, r = [], int(TERRAIN_R / TERRAIN_S)
    for ix in range(-r, r+1):
        for iy in range(-r, r+1):
            pts.append([cx + ix*TERRAIN_S, cy + iy*TERRAIN_S, 0.0, 0.0])
    arr = np.array(pts, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    msg.height = 1; msg.width = len(arr)
    msg.is_dense = True; msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16; msg.row_step = 16 * len(arr)
    msg.data = arr.tobytes()
    return msg

def _straight_path(gx, gy, n=15):
    msg = Path()
    msg.header.frame_id = 'map'
    for i in range(n):
        t = i / (n-1)
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = t * gx
        ps.pose.position.y = t * gy
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg

def _path_rms(traj_xy, waypoints):
    """Cross-track RMS error from trajectory to piecewise-linear reference."""
    errs = []
    pts = np.array(traj_xy)
    wps = np.array(waypoints)
    for p in pts:
        min_d = float('inf')
        for i in range(len(wps)-1):
            a, b = wps[i], wps[i+1]
            ab = b - a
            denom = np.dot(ab, ab)
            if denom < 1e-9:
                d = np.linalg.norm(p - a)
            else:
                t = np.clip(np.dot(p - a, ab) / denom, 0, 1)
                d = np.linalg.norm(p - (a + t * ab))
            min_d = min(min_d, d)
        errs.append(min_d)
    return float(np.sqrt(np.mean(np.array(errs)**2))) if errs else 0.0


# ── Main node ─────────────────────────────────────────────────────────────────

class MultiScenarioNode(Node):
    def __init__(self):
        super().__init__('e2e_multi_scenario')
        os.makedirs(OUT_DIR, exist_ok=True)

        # robot state
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.cmd_count = 0

        # scenario state
        self.sc_idx    = 0
        self.phase     = 'warmup'   # warmup|running|done|finished
        self.t_start   = None
        self.t_reached = None
        self.goal_reached = False
        self._csv_rows = []         # current scenario rows
        self._all_done = threading.Event()

        # TF
        self.static_br = StaticTransformBroadcaster(self)
        self.tf_br = TransformBroadcaster(self)
        self._pub_static_tf()

        # Pubs
        self.pub_odom    = self.create_publisher(Odometry,    '/slam/odometry',    10)
        self.pub_cloud   = self.create_publisher(PointCloud2, '/slam/map_cloud',   10)
        self.pub_terrain = self.create_publisher(PointCloud2, '/nav/terrain_map', 10)
        self.pub_path    = self.create_publisher(Path,        '/nav/global_path', 10)

        # cmd_vel sub (BEST_EFFORT, pathFollower)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub_cmd = self.create_subscription(
            TwistStamped, '/nav/cmd_vel', self._on_cmd, be)

        self.create_timer(DT, self._tick)
        sc = SCENARIOS[0]
        self.get_logger().info(
            f'Multi-scenario E2E started. '
            f'Scenarios: {[s["name"] for s in SCENARIOS]}')

    # ── TF & odometry ─────────────────────────────────────────────────────────

    def _pub_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'; t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_br.sendTransform(t)

    def _pub_odom(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now; t.header.frame_id = 'odom'; t.child_frame_id = 'body'
        t.transform.translation.x = self.x; t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw/2)
        t.transform.rotation.w = math.cos(self.yaw/2)
        self.tf_br.sendTransform(t)
        od = Odometry()
        od.header.stamp = now; od.header.frame_id = 'odom'; od.child_frame_id = 'body'
        od.pose.pose.position.x = self.x; od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw/2)
        od.pose.pose.orientation.w = math.cos(self.yaw/2)
        od.twist.twist.linear.x = self.vx; od.twist.twist.linear.y = self.vy
        od.twist.twist.angular.z = self.wz
        self.pub_odom.publish(od)

    # ── cmd_vel ───────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.wz = msg.twist.angular.z
        self.cmd_count += 1

    # ── kinematics ────────────────────────────────────────────────────────────

    def _integrate(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x   += (c * self.vx - s * self.vy) * DT
        self.y   += (s * self.vx + c * self.vy) * DT
        self.yaw += self.wz * DT
        self.yaw  = (self.yaw + math.pi) % (2*math.pi) - math.pi

    # ── main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        if self.phase == 'finished':
            return

        sc = SCENARIOS[self.sc_idx]
        gx, gy = sc['goal']
        now = self.get_clock().now().to_msg()

        # Always: odom + terrain
        self._pub_odom()
        ter = _terrain(self.x, self.y); ter.header.stamp = now
        self.pub_cloud.publish(ter); self.pub_terrain.publish(ter)

        if self.phase == 'warmup':
            if self.t_start is None:
                self.t_start = time.time()
            elapsed = time.time() - self.t_start
            if elapsed >= WARMUP_S:
                self.phase = 'running'
                path = _straight_path(gx, gy); path.header.stamp = now
                self.pub_path.publish(path)
                self.get_logger().info(
                    f'[SC{sc["id"]}] {sc["name"]} START → goal=({gx},{gy})')

        elif self.phase == 'running':
            elapsed = time.time() - self.t_start

            # Re-publish path every 2s
            if int(elapsed * 0.5) > int((elapsed - DT) * 0.5):
                path = _straight_path(gx, gy); path.header.stamp = now
                self.pub_path.publish(path)

            # Integrate kinematics
            self._integrate()

            # Dense 20Hz logging
            dist = math.hypot(self.x - gx, self.y - gy)
            self._csv_rows.append([
                round(elapsed, 3),
                round(self.x, 4), round(self.y, 4),
                round(math.degrees(self.yaw), 2),
                round(self.vx, 4), round(self.vy, 4), round(self.wz, 4),
                round(dist, 4), self.cmd_count,
            ])

            # Check goal
            if dist < sc['thre'] and self.t_reached is None:
                self.t_reached = elapsed
                self.goal_reached = True
                self.get_logger().info(
                    f'[SC{sc["id"]}] *** GOAL REACHED t={elapsed:.1f}s dist={dist:.3f}m ***')
                self.phase = 'done'
                return

            if elapsed > sc['timeout']:
                self.get_logger().warn(
                    f'[SC{sc["id"]}] TIMEOUT {elapsed:.0f}s dist={dist:.2f}m')
                self.t_reached = elapsed
                self.phase = 'done'
                return

        elif self.phase == 'done':
            self._save_scenario()

        if self.phase == 'finished':
            self._all_done.set()

    # ── save & advance ────────────────────────────────────────────────────────

    def _save_scenario(self):
        sc = SCENARIOS[self.sc_idx]
        gx, gy = sc['goal']
        csv_path = f'{OUT_DIR}/scenario_{sc["id"]}.csv'
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw_deg', 'vx', 'vy', 'wz',
                        'dist_to_goal', 'cmd_count'])
            w.writerows(self._csv_rows)
        # Path RMS error
        xy = [(r[1], r[2]) for r in self._csv_rows]
        ref_pts = [(i/(14)*gx, i/(14)*gy) for i in range(15)]
        prms = _path_rms(xy, ref_pts)
        # Final dist
        final_dist = math.hypot(self._csv_rows[-1][1]-gx,
                                self._csv_rows[-1][2]-gy) if self._csv_rows else 999
        meta = {
            "id": sc["id"], "name": sc["name"],
            "goal": list(sc["goal"]),
            "goal_reached": self.goal_reached,
            "t_reached": round(self.t_reached, 2) if self.t_reached else None,
            "final_dist": round(final_dist, 4),
            "path_rms": round(prms, 4),
            "cmd_count": self.cmd_count,
            "n_samples": len(self._csv_rows),
        }
        with open(f'{OUT_DIR}/scenario_{sc["id"]}_meta.json', 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(
            f'[SC{sc["id"]}] Saved {csv_path} ({len(self._csv_rows)} rows)')
        self.get_logger().info(
            f'  goal_reached={meta["goal_reached"]} t={meta["t_reached"]}s '
            f'dist={meta["final_dist"]}m rms={meta["path_rms"]}m')

        # Advance to next scenario
        self.sc_idx += 1
        if self.sc_idx >= len(SCENARIOS):
            self.phase = 'finished'
            self._all_done.set()
            self.get_logger().info('=== ALL SCENARIOS DONE ===')
            return

        # Reset for next scenario
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.cmd_count = 0
        self.t_start = None
        self.t_reached = None
        self.goal_reached = False
        self._csv_rows = []
        self.phase = 'warmup'
        sc2 = SCENARIOS[self.sc_idx]
        self.get_logger().info(
            f'--- Switching to SC{sc2["id"]}: {sc2["name"]} ---')


# ── entry ──────────────────────────────────────────────────────────────────────

def main():
    os.system('pkill -f pub_foxglove_demo.py 2>/dev/null; sleep 0.3')
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = MultiScenarioNode()

    def _sighandler(sig, frame):
        node.get_logger().warn('Signal received — saving and exiting')
        if node._csv_rows:
            node._save_scenario()
        rclpy.shutdown(); sys.exit(0)
    signal.signal(signal.SIGTERM, _sighandler)
    signal.signal(signal.SIGINT,  _sighandler)

    try:
        while rclpy.ok() and not node._all_done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass
    print('[multi] All scenarios complete.')


if __name__ == '__main__':
    main()
