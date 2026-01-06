#!/usr/bin/env python3
import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
import tf2_geometry_msgs

from utils import *
from planner_wrapper import TomogramPlanner

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '../'))
sys.path.append(os.path.join(current_dir, '../lib')) # Ensure compiled libs are found
from config import Config


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('pct_global_planner')

        # Declare parameters
        self.declare_parameter('map_file', 'scans') 
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        self.tomo_file = map_file
        self.start_pos = np.zeros(2, dtype=np.float32)
        self.start_height = 0.0
        self.end_pos = np.zeros(2, dtype=np.float32)
        self.end_height = 0.0
        
        cfg = Config()
        self.planner = TomogramPlanner(cfg)
        self.planner.loadTomogram(self.tomo_file)
        
        # ROS 2 Publisher with QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, "/pct_path", qos_profile)
    
        # Status Publisher for planning state feedback
        self.status_pub = self.create_publisher(String, '/pct_planner/status', 10)
        self.planning_status = "IDLE"
        
        # TF Listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frame names from FASTLIO2
        self.map_frame = 'map'
        self.robot_frame = 'body' 
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.goal_callback, 10
        )
        
        # State variables
        self.current_start_pos = self.start_pos.copy()
        self.current_end_pos = self.end_pos.copy()
        self.current_start_height = self.start_height
        self.current_end_height = self.end_height
        
        # Debouncing parameters
        self.declare_parameter('min_plan_interval', 1.0)  # Minimum time between replanning (seconds)
        self.min_plan_interval = self.get_parameter('min_plan_interval').value
        self.last_plan_time = None

        self.get_logger().info(f'PCT Global Planner Initialized with map: {self.tomo_file}. Waiting for TF and Goal...')
        self.publish_status("IDLE")
        
    def get_robot_pose(self):
        """Get current robot pose from TF (map -> body)"""
        try:
            # Check if transform exists
            if not self.tf_buffer.can_transform(self.map_frame, self.robot_frame, rclpy.time.Time()):
                return None, None, False
                
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            return np.array([x, y], dtype=np.float32), z, True
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            # Throttle warning to avoid spam (every 5 seconds)
            self.get_logger().warn(
                f"TF unavailable ({self.map_frame} -> {self.robot_frame}): {e}",
                throttle_duration_sec=5.0
            )
            return None, None, False
        except Exception as e:
            # Unexpected errors should be logged as errors
            self.get_logger().error(f"Unexpected TF error: {e}")
            raise

        
    def goal_callback(self, msg):
        """Callback for new goal from Rviz"""
        # Debouncing: Prevent too frequent goal updates
        current_time = self.get_clock().now()
        if self.last_plan_time is not None:
            elapsed = (current_time - self.last_plan_time).nanoseconds / 1e9
            if elapsed < self.min_plan_interval:
                self.get_logger().warn(
                    f"⏱️ Goal update too frequent, ignoring "
                    f"(wait {self.min_plan_interval - elapsed:.1f}s)"
                )
                return
        
        self.get_logger().info(f"Received new goal: {msg.point.x:.2f}, {msg.point.y:.2f}")
        self.current_end_pos = np.array([msg.point.x, msg.point.y], dtype=np.float32)
        self.current_end_height = msg.point.z
        
        # Update start position from TF before planning
        pos, height, success = self.get_robot_pose()
        if success:
            self.current_start_pos = pos
            self.current_start_height = height
            self.last_plan_time = current_time  # Record planning time
            self.pct_plan()
        else:
            self.get_logger().warn("Cannot plan: TF (Localization) not available yet.")
        
    def pct_plan(self):
        """Execute path planning with status feedback and performance monitoring"""
        self.get_logger().info(f"Planning from {self.current_start_pos} to {self.current_end_pos}")
        
        # Publish planning status
        self.publish_status("PLANNING")
        
        # Performance monitoring
        start_time = time.time()
        
        traj_3d = self.planner.plan(
            self.current_start_pos, 
            self.current_end_pos, 
            self.current_start_height, 
            self.current_end_height
        )
        
        elapsed_ms = (time.time() - start_time) * 1000
        
        if traj_3d is not None:
            self.path_pub.publish(traj2ros(traj_3d))
            self.get_logger().info(
                f"✓ Path published to /pct_path ({len(traj_3d)} points, {elapsed_ms:.1f}ms)"
            )
            self.publish_status("SUCCESS")
        else:
            self.get_logger().error(
                f"✗ Planning FAILED: No valid path found from "
                f"({self.current_start_pos[0]:.2f}, {self.current_start_pos[1]:.2f}) to "
                f"({self.current_end_pos[0]:.2f}, {self.current_end_pos[1]:.2f}). "
                f"Check if goal is reachable in the tomogram. ({elapsed_ms:.1f}ms)"
            )
            self.publish_status("FAILED")
    
    def publish_status(self, status: str):
        """Publish planning status for external monitoring"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.planning_status = status

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    planner = GlobalPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
