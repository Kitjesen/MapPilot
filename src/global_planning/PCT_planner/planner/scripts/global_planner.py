#!/usr/bin/env python3
import sys
import argparse
import numpy as np
import threading
import time
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException

# Add library paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '../'))
sys.path.append(os.path.join(current_dir, '../lib'))

from utils import traj2ros
from planner_wrapper import TomogramPlanner
from config import Config

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('pct_global_planner')

        # 1. Concurrency control
        self.callback_group = ReentrantCallbackGroup()
        self.plan_lock = threading.Lock()

        # 2. Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_file', 'spiral0.3_2'),
                ('map_frame', 'map'),
                ('robot_frame', 'body'),
                ('min_plan_interval', 1.0)
            ]
        )
        
        self.tomo_file = self.get_parameter('map_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_plan_interval = self.get_parameter('min_plan_interval').value
        
        # 3. Initialize Planner Core
        cfg = Config()
        self.planner = TomogramPlanner(cfg)
        self.planner.loadTomogram(self.tomo_file)
        
        # 4. Publishers
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, "/pct_path", qos_latched)
        self.status_pub = self.create_publisher(String, '/pct_planner/status', 10)
        
        # 5. TF (Increased cache time for robustness)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 6. Subscribers
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10, 
            callback_group=self.callback_group)
        self.goal_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.goal_point_callback, 10, 
            callback_group=self.callback_group)
        
        # State
        self.last_plan_time = self.get_clock().now()

        self.get_logger().info(f'PCT Global Planner Ready | Map: {self.tomo_file}')
        self.publish_status("IDLE")
        
    def get_robot_pose(self):
        """Optimized TF lookup"""
        try:
            # Lookup latest available transform
            # Using rclpy.time.Time() creates a time point with 0, requesting the latest available transform
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            pos = np.array([t.transform.translation.x, t.transform.translation.y], dtype=np.float32)
            return pos, t.transform.translation.z, True
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            # Throttle warning log to avoid spamming the console
            self.get_logger().warn(f"TF Wait: {e}", throttle_duration_sec=5.0)
            return None, None, False
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None, None, False
        
    def goal_pose_callback(self, msg):
        self._handle_goal(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, "goal_pose")
    
    def goal_point_callback(self, msg):
        self._handle_goal(msg.point.x, msg.point.y, msg.point.z, "clicked_point")
    
    def _handle_goal(self, x, y, z, source):
        current_time = self.get_clock().now()
        
        # Debouncing
        if self.last_plan_time is not None:
            elapsed = (current_time - self.last_plan_time).nanoseconds / 1e9
            if elapsed < self.min_plan_interval:
                self.get_logger().warn(f"Goal rejected: too fast ({elapsed:.1f}s < {self.min_plan_interval}s)", throttle_duration_sec=2.0)
                return
        
        # Check lock
        if self.plan_lock.locked():
            self.get_logger().warn("Planner is busy, dropping new goal")
            return

        # Get Start Position
        start_pos, start_height, success = self.get_robot_pose()
        if not success:
            self.publish_status("NO_LOCALIZATION")
            return
        
        self.last_plan_time = current_time
        self.get_logger().info(f"Goal received ({source}): ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Execute Plan
        self.pct_plan(start_pos, start_height, np.array([x, y], dtype=np.float32), z)
        
    def pct_plan(self, start_pos, start_h, end_pos, end_h):
        """Thread-safe planning execution"""
        with self.plan_lock:
            self.publish_status("PLANNING")
            start_tick = time.time()
            
            try:
                traj_3d = self.planner.plan(start_pos, end_pos, start_h, end_h)
                
                duration_ms = (time.time() - start_tick) * 1000
                
                if traj_3d is not None and len(traj_3d) > 0:
                    # Publish Path
                    path_msg = traj2ros(traj_3d)
                    path_msg.header.stamp = self.get_clock().now().to_msg()
                    path_msg.header.frame_id = self.map_frame
                    
                    self.path_pub.publish(path_msg)
                    self.get_logger().info(f"✓ Plan Success: {len(traj_3d)} pts in {duration_ms:.1f}ms")
                    self.publish_status("SUCCESS")
                else:
                    self.get_logger().error(f"✗ Plan Failed: No path found ({duration_ms:.1f}ms)")
                    self.publish_status("FAILED")
                    
            except Exception as e:
                self.get_logger().error(f"☠️ Planner Core Internal Error: {e}")
                import traceback
                traceback.print_exc()
                self.publish_status("ERROR")
    
    def publish_status(self, status: str):
        msg = String(data=status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPlanner()
    
    # Use MultiThreadedExecutor to handle reentrant callbacks
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(planner, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
