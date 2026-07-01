#!/usr/bin/env python3
"""
E2E 闭环导航验证
Robot: (0,0) → Goal: (5,2)
Loop: cmd_vel → kinematics → odometry → pct_adapter → waypoint → localPlanner → pathFollower → cmd_vel
"""
import math
import os
import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

GOAL = (5.0, 2.0)
GOAL_DIST_THRESH = 0.3
MAX_SIM_SECS = 90.0
DT = 0.05        # 20 Hz loop
TERRAIN_RADIUS = 8.0
TERRAIN_STEP = 0.4


# ── helpers ───────────────────────────────────────────────────────────────────

def _make_terrain(cx, cy):
    """Flat PointCloud2 XYZI centered at (cx,cy) in odom frame."""
    pts = []
    r = int(TERRAIN_RADIUS / TERRAIN_STEP)
    for ix in range(-r, r + 1):
        for iy in range(-r, r + 1):
            pts.append([cx + ix * TERRAIN_STEP, cy + iy * TERRAIN_STEP, 0.0, 0.0])
    arr = np.array(pts, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * len(arr)
    msg.data = arr.tobytes()
    return msg


def _make_path(gx, gy, n=15):
    """Straight-line Path from (0,0) to (gx,gy) in map frame."""
    msg = Path()
    msg.header.frame_id = 'map'
    for i in range(n):
        t = i / (n - 1)
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = t * gx
        ps.pose.position.y = t * gy
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


# ── main node ─────────────────────────────────────────────────────────────────

class E2ENavTest(Node):
    def __init__(self):
        super().__init__('e2e_nav_test')

        # Robot state
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.cmd_count = 0
        self.traj = []
        self.t_start = None
        self.phase = 'warmup'   # warmup → running → done → saved
        self.goal_reached = False
        self._done = threading.Event()

        # TF
        self.static_br = StaticTransformBroadcaster(self)
        self.tf_br = TransformBroadcaster(self)
        self._send_static_tf()

        # Publishers
        self.pub_odom    = self.create_publisher(Odometry,    '/slam/odometry',    10)
        self.pub_cloud   = self.create_publisher(PointCloud2, '/slam/map_cloud',   10)
        self.pub_terrain = self.create_publisher(PointCloud2, '/nav/terrain_map', 10)
        self.pub_path    = self.create_publisher(Path,        '/nav/global_path', 10)

        # Subscribe cmd_vel (pathFollower publishes BEST_EFFORT 50Hz)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub_cmd = self.create_subscription(
            TwistStamped, '/nav/cmd_vel', self._on_cmd, be)

        # 20 Hz main loop
        self.create_timer(DT, self._tick)
        self.get_logger().info(f'E2E test ready. Goal={GOAL}')

    # ── TF ────────────────────────────────────────────────────────────────────

    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_br.sendTransform(t)

    def _send_odom_tf(self):
        now = self.get_clock().now().to_msg()
        # odom → body
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'body'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_br.sendTransform(t)

        # Odometry msg
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'body'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw / 2)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2)
        od.twist.twist.linear.x = self.vx
        od.twist.twist.linear.y = self.vy
        od.twist.twist.angular.z = self.wz
        self.pub_odom.publish(od)

    # ── cmd_vel callback ──────────────────────────────────────────────────────

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
        # normalise
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

    # ── main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        if self.phase == 'saved':
            return
        if self.phase == 'done':
            self._finish()
            return

        if self.t_start is None:
            self.t_start = time.time()
        elapsed = time.time() - self.t_start

        now = self.get_clock().now().to_msg()

        # Always: odometry + TF + terrain
        self._send_odom_tf()
        terrain = _make_terrain(self.x, self.y)
        terrain.header.stamp = now
        self.pub_cloud.publish(terrain)
        self.pub_terrain.publish(terrain)

        if self.phase == 'warmup':
            # Warmup 3 s: let TF buffer fill + joyToSpeedDelay
            if elapsed >= 3.0:
                self.phase = 'running'
                path = _make_path(*GOAL)
                path.header.stamp = now
                self.pub_path.publish(path)
                self.get_logger().info(
                    f'== RUNNING: global_path published (0,0)→{GOAL} ==')

        elif self.phase == 'running':
            # Publish global path every 2 s
            if int(elapsed * 0.5) > int((elapsed - DT) * 0.5):
                path = _make_path(*GOAL)
                path.header.stamp = now
                self.pub_path.publish(path)

            # Integrate kinematics
            self._integrate()

            # Log + goal check at ~2 Hz
            if int(elapsed * 2) > int((elapsed - DT) * 2):
                dist = math.hypot(self.x - GOAL[0], self.y - GOAL[1])
                deg  = math.degrees(self.yaw)
                self.traj.append(dict(
                    t=round(elapsed,2),
                    x=round(self.x,3), y=round(self.y,3),
                    yaw_deg=round(deg,1),
                    vx=round(self.vx,3), vy=round(self.vy,3), wz=round(self.wz,3),
                    dist=round(dist,3), cmds=self.cmd_count
                ))
                self.get_logger().info(
                    f't={elapsed:5.1f}s  pos=({self.x:5.2f},{self.y:5.2f})'
                    f'  yaw={deg:6.1f}°  v=({self.vx:.2f},{self.vy:.2f})'
                    f'  w={self.wz:.3f}  dist={dist:.2f}m  cmds={self.cmd_count}')

                if dist < GOAL_DIST_THRESH:
                    self.goal_reached = True
                    self.get_logger().info(
                        f'*** GOAL REACHED at t={elapsed:.1f}s dist={dist:.3f}m ***')
                    self.phase = 'done'
                    return

            if elapsed > MAX_SIM_SECS:
                dist = math.hypot(self.x - GOAL[0], self.y - GOAL[1])
                self.get_logger().warn(
                    f'TIMEOUT {elapsed:.0f}s  final_dist={dist:.2f}m')
                self.phase = 'done'
                return

        if self.phase == 'done':
            self._finish()

    # ── save results ──────────────────────────────────────────────────────────

    def _finish(self):
        self.phase = 'saved'
        result = dict(
            goal_reached=self.goal_reached,
            goal=list(GOAL),
            final_pos=[round(self.x,3), round(self.y,3)],
            final_yaw_deg=round(math.degrees(self.yaw),1),
            cmd_count=self.cmd_count,
            trajectory=self.traj,
        )
        with open('/tmp/e2e_result.json', 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info('Saved /tmp/e2e_result.json')
        self._plot(result)
        self._done.set()

    def _plot(self, result):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            traj = result['trajectory']
            ts  = [p['t']    for p in traj]
            xs  = [p['x']    for p in traj]
            ys  = [p['y']    for p in traj]
            ds  = [p['dist'] for p in traj]
            vxs = [p['vx']   for p in traj]
            wzs = [p['wz']   for p in traj]

            fig, axes = plt.subplots(1, 2, figsize=(14, 6))

            # Panel 1: XY trajectory
            ax = axes[0]
            ax.plot(xs, ys, 'b-', lw=2, label='Robot path')
            ax.plot(0, 0, 'go', ms=12, label='Start (0,0)')
            ax.plot(GOAL[0], GOAL[1], 'r*', ms=18, label=f'Goal {GOAL}')
            if xs:
                ax.plot(xs[-1], ys[-1], 'bs', ms=10,
                        label=f'Final ({xs[-1]:.2f},{ys[-1]:.2f})')
            ax.plot([0, GOAL[0]], [0, GOAL[1]], 'g--', alpha=0.35,
                    label='Straight-line ref')
            status = 'GOAL REACHED ✓' if self.goal_reached else f'TIMEOUT (dist={ds[-1]:.2f}m)'
            ax.set_title(f'E2E Closed-Loop Navigation — {status}', fontsize=11)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.legend(fontsize=8)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)

            # Panel 2: time series
            ax2 = axes[1]
            ax2.plot(ts, ds,  'r-',  lw=2, label='Distance to goal (m)')
            ax2.plot(ts, vxs, 'b-',  lw=1.5, label='vx (m/s)')
            ax2.plot(ts, wzs, 'g--', lw=1.2, label='ωz (rad/s)')
            ax2.axhline(GOAL_DIST_THRESH, color='r', ls=':', alpha=0.5,
                        label=f'Threshold {GOAL_DIST_THRESH}m')
            ax2.set_xlabel('Time (s)')
            ax2.set_title('Navigation Metrics')
            ax2.legend(fontsize=8)
            ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig('/tmp/e2e_trajectory.png', dpi=130, bbox_inches='tight')
            self.get_logger().info('Saved /tmp/e2e_trajectory.png')
        except Exception as e:
            self.get_logger().error(f'Plot error: {e}')


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    # Kill old demo
    os.system('pkill -f pub_foxglove_demo.py 2>/dev/null; sleep 0.5')
    print('[e2e] Old demo killed. Starting closed-loop test...')

    # Install matplotlib if missing
    try:
        import matplotlib
    except ImportError:
        import subprocess, sys
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'matplotlib', '-q'],
                       check=False)

    rclpy.init()
    node = E2ENavTest()

    try:
        while rclpy.ok() and not node._done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('[e2e] Interrupted')
    finally:
        if node.phase not in ('done', 'saved') and node.traj:
            node._finish()
        node.destroy_node()
        rclpy.shutdown()
    print('[e2e] Done.')


if __name__ == '__main__':
    main()
