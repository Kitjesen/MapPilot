#!/usr/bin/env python3
"""
Generic Robot Driver Node Template
----------------------------------
This node serves as a template for interfacing with your specific robot hardware.
It handles:
1. Subscribing to velocity commands (/cmd_vel) -> sending to motors
2. Reading hardware sensors (Encoders/IMU) -> publishing odometry (/wheel_odom)

You need to implement the 'real' communication logic in the marked sections.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Point, Vector3, TwistStamped, Twist
from tf_transformations import quaternion_from_euler
from math import sin, cos
import time
import threading

class GenericRobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # --- Configuration Parameters ---
        self.declare_parameter('enable_odom_tf', False) # Usually provided by FASTLIO2
        self.declare_parameter('control_rate', 50.0)    # Hz
        self.declare_parameter('robot_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # --- Publishers ---
        # Wheel odometry (useful for local planning / loop closure / velocity feedback)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        # IMU data (optional, if your robot chassis provides it)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # --- Subscribers ---
        # Velocity commands from local planner
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # --- Internal State ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        
        # --- Hardware Connection (Placeholder) ---
        self.connect_hardware()

        # Timer for reading hardware loop
        self.timer = self.create_timer(
            1.0 / self.get_parameter('control_rate').value,
            self.hardware_loop
        )
        
        self.get_logger().info('Generic Robot Driver Initialized')

    def connect_hardware(self):
        """
        [IMPLEMENT THIS]
        Initialize your serial port, TCP connection, or CAN bus here.
        """
        port = self.get_parameter('robot_port').value
        baud = self.get_parameter('baudrate').value
        self.get_logger().info(f"Connecting to hardware on {port} at {baud}...")
        # e.g. self.serial = serial.Serial(port, baud)

    def cmd_vel_callback(self, msg):
        """
        Handle incoming velocity commands from local planner.
        Input: geometry_msgs/TwistStamped
        """
        # 1. Extract velocities
        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y  # For omni/mecanum robots
        angular_z = msg.twist.angular.z
        
        # 2. [IMPLEMENT THIS] Send to hardware
        # self.get_logger().info(f"CMD: vx={linear_x:.2f}, vy={linear_y:.2f}, wz={angular_z:.2f}")
        self.send_to_motors(linear_x, linear_y, angular_z)

    def send_to_motors(self, vx, vy, wz):
        """
        [IMPLEMENT THIS] 
        Convert metric velocities to your motor commands (PWM, RPM, etc.)
        """
        pass
        # protocol_buffer = create_packet(vx, vy, wz)
        # self.serial.write(protocol_buffer)

    def hardware_loop(self):
        """
        Periodic loop to read from hardware and publish feedback
        """
        # 1. [IMPLEMENT THIS] Read from hardware
        # data = self.serial.read(...)
        # encoder_vals = parse(data)
        
        # 2. Update status (Placeholder values)
        # In a real driver, calculate vx, vy, vth from encoders
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Dummy integration (replace with real encoder readings)
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 3. Publish Odometry
        self.publish_odometry(current_time)
        
        self.last_time = current_time

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = GenericRobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
