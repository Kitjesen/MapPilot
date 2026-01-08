#!/usr/bin/env python3
"""
PCT Path to Waypoint Adapter
----------------------------
Role: Bridges the Global Planner (PCT) and the Local Planner via Closed-Loop Control.

1. Subscribes to `/pct_path` (Global Path, static sequence of poses)
2. Subscribes to `/Odometry` (Robot state)
3. Publishes `/way_point` (Single target) for the Local Planner

Logic:
- Instead of blindly publishing all points time-based, it monitors robot progress.
- Only when the robot reaches the current waypoint (within threshold) does it publish the next one.
- This acts as a "carrot on a stick", ensuring the robot stays within the global path corridor/stairs
  while allowing the Local Planner to handle micro-movements for obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, Point
import time
import math
import numpy as np
# Monkey patch for transforms3d compatibility with newer numpy
if not hasattr(np, 'float'):
    np.float = float

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from numpy import dot

class PCTPathAdapter(Node):
    def __init__(self):
        super().__init__('pct_path_adapter')
        
        # --- Subscribers ---
        # 1. Global Path from PCT Planner
        self.path_sub = self.create_subscription(
            Path,
            '/pct_path',
            self.path_callback,
            10
        )
        
        # 2. Robot Odometry for closed-loop checking
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # --- Publishers ---
        # Target for Local Planner
        self.waypoint_pub = self.create_publisher(
            PointStamped,
            '/way_point',
            10
        )
        
        # --- Parameters ---
        self.declare_parameter('waypoint_distance', 0.5)  # Spacing between waypoints (meters)
        self.declare_parameter('arrival_threshold', 0.5)  # Distance to consider "arrived"
        self.declare_parameter('lookahead_dist', 1.0)     # How far ahead to look

        self.waypoint_distance = self.get_parameter('waypoint_distance').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        
        self.current_path = []
        self.current_waypoint_idx = 0
        self.robot_pos = None
        self.path_received = False
        self.odom_frame = None
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control loop timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('PCT Path Adapter initialized')
    
    def odom_callback(self, msg):
        """Update robot current position"""
        self.robot_pos = msg.pose.pose.position
        # Store frame_id of Odometry to know what frame the local planner expects
        self.odom_frame = msg.header.frame_id
        
    def path_callback(self, msg):
        """Receive and process new global path"""
        if len(msg.poses) == 0:
            return
            
        self.get_logger().info(f'Received new Global Path with {len(msg.poses)} points')
        
        # Downsample path to create manageable waypoints
        self.current_path = self.downsample_path(msg)
        self.current_waypoint_idx = 0
        self.path_received = True
        
        self.get_logger().info(f'Path processed into {len(self.current_path)} waypoints')
        
    def downsample_path(self, path):
        """
        Simplifies the dense global path into sparser waypoints 
        based on 'waypoint_distance' parameter.
        """
        if len(path.poses) == 0:
            return []
        
        downsampled = [path.poses[0]]
        last_pose = path.poses[0]
        
        for pose in path.poses[1:]:
            dx = pose.pose.position.x - last_pose.pose.position.x
            dy = pose.pose.position.y - last_pose.pose.position.y
            dist = math.hypot(dx, dy)
            
            if dist >= self.waypoint_distance:
                downsampled.append(pose)
                last_pose = pose
        
        # Always include the final goal
        downsampled.append(path.poses[-1])
            
        return downsampled
    
    def get_distance(self, p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def transform_point(self, point_in_map):
        """Transform a point from Map frame to Odom frame"""
        if not self.odom_frame:
            return None
            
        try:
            # Look up transform from map to odom_frame
            # We want to convert Point(Map) -> Point(Odom)
            # So we need transform "Map -> Odom"
            # In TF terms: source=Map, target=Odom
            t = self.tf_buffer.lookup_transform(
                self.odom_frame,
                'map',
                rclpy.time.Time()
            )
            
            # Apply transform
            # P_odom = R * P_map + T
            
            from tf_transformations import quaternion_matrix
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            mat = quaternion_matrix(q)
            mat[0, 3] = t.transform.translation.x
            mat[1, 3] = t.transform.translation.y
            mat[2, 3] = t.transform.translation.z
            
            p_map = [point_in_map.x, point_in_map.y, point_in_map.z, 1.0]
            p_odom = dot(mat, p_map)
            
            new_point = Point()
            new_point.x = p_odom[0]
            new_point.y = p_odom[1]
            new_point.z = p_odom[2]
            return new_point

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform waypoint: {ex}', throttle_duration_sec=2.0)
            return None
    
    def control_loop(self):
        """
        Main Logic Loop:
        Checks if robot reached current waypoint -> Updates to next waypoint
        """
        if not self.path_received or not self.current_path or self.robot_pos is None or self.odom_frame is None:
            return
            
        # Get current target in MAP frame
        target_pose_map = self.current_path[self.current_waypoint_idx].pose.position
        
        # Transform target to ODOM frame
        target_pose_odom = self.transform_point(target_pose_map)
        
        if target_pose_odom is None:
            return

        # Check distance to target (Both in ODOM frame now)
        dist_to_target = self.get_distance(self.robot_pos, target_pose_odom)
        
        # Logic: If close enough AND not the last point, switch to next point
        if dist_to_target < self.arrival_threshold:
            if self.current_waypoint_idx < len(self.current_path) - 1:
                self.get_logger().info(f'Reached Waypoint {self.current_waypoint_idx}. Proceeding to next.')
                self.current_waypoint_idx += 1
                return # Give one cycle to update
            else:
                self.get_logger().info('Goal Reached!', throttle_duration_sec=5.0)
                # Keep publishing the last point
                
        # Publish current target to Local Planner (in ODOM frame)
        waypoint_msg = PointStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.odom_frame # Use Odom frame
        waypoint_msg.point = target_pose_odom
        
        self.waypoint_pub.publish(waypoint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PCTPathAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
