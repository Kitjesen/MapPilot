#!/usr/bin/env python3
import json
import uuid
import threading
import time
import websocket
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Point, Vector3

# Replace this ACCID value with your robot's actual serial number (SN)
ACCID = "WF_TRON1A_131"

class LIMXOdomPublisher(Node):
    def __init__(self):
        super().__init__('limx_odom_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/limx/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/limx/imu', 10)
        
        # WebSocket client instance
        self.ws_client = None
        
        self.get_logger().info('LIMX Odom Publisher initialized')
        
    def generate_guid(self):
        return str(uuid.uuid4())
    
    def send_request(self, title, data=None):
        """Send WebSocket request with title and data"""
        if data is None:
            data = {}
        
        message = {
            "accid": ACCID,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": self.generate_guid(),
            "data": data
        }
        
        message_str = json.dumps(message)
        
        if self.ws_client:
            self.ws_client.send(message_str)
    
    def handle_commands(self):
        """Handle initial commands to enable IMU and odometry"""
        self.send_request("request_enable_imu", {"enable": True})
        time.sleep(1)
        self.send_request("request_enable_odom", {"enable": True})
        time.sleep(1)
    
    def on_open(self, ws):
        """WebSocket on_open callback"""
        self.get_logger().info("Connected to LIMX robot!")
        threading.Thread(target=self.handle_commands, daemon=True).start()
    
    def on_message(self, ws, message):
        """WebSocket on_message callback"""
        try:
            data = json.loads(message)
            
            if data["title"] == "notify_odom":
                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                
                # Set position
                odom_msg.pose.pose.position = Point(
                    x=data["data"]["pose_position"][0],
                    y=data["data"]["pose_position"][1],
                    z=data["data"]["pose_position"][2]
                )
                
                # Set orientation
                odom_msg.pose.pose.orientation = Quaternion(
                    x=data["data"]["pose_orientation"][0],
                    y=data["data"]["pose_orientation"][1],
                    z=data["data"]["pose_orientation"][2],
                    w=data["data"]["pose_orientation"][3]
                )
                
                # Set velocity
                odom_msg.twist.twist.linear = Vector3(
                    x=data["data"]["twist_linear"][0],
                    y=data["data"]["twist_linear"][1],
                    z=data["data"]["twist_linear"][2]
                )
                odom_msg.twist.twist.angular = Vector3(
                    x=data["data"]["twist_angular"][0],
                    y=data["data"]["twist_angular"][1],
                    z=data["data"]["twist_angular"][2]
                )
                
                # Publish odometry message
                self.odom_pub.publish(odom_msg)
                
            elif data["title"] == "notify_imu":
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                # Set orientation
                imu_msg.orientation = Quaternion(
                    x=data["data"]["quat"][0],
                    y=data["data"]["quat"][1],
                    z=data["data"]["quat"][2],
                    w=data["data"]["quat"][3]
                )
                
                # Set angular velocity
                imu_msg.angular_velocity = Vector3(
                    x=data["data"]["gyro"][0],
                    y=data["data"]["gyro"][1],
                    z=data["data"]["gyro"][2]
                )
                
                # Set linear acceleration
                imu_msg.linear_acceleration = Vector3(
                    x=data["data"]["acc"][0],
                    y=data["data"]["acc"][1],
                    z=data["data"]["acc"][2]
                )
                
                # Publish IMU message
                self.imu_pub.publish(imu_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")
    
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket on_close callback"""
        self.get_logger().warn("Connection closed.")
    
    def connect_websocket(self):
        """Create and run WebSocket connection"""
        self.ws_client = websocket.WebSocketApp(
            "ws://10.192.1.2:5000",
            on_open=self.on_open,
            on_message=self.on_message,
            on_close=self.on_close
        )
        
        self.get_logger().info("Starting WebSocket connection...")
        self.ws_client.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = LIMXOdomPublisher()
    
    # Start WebSocket in separate thread
    ws_thread = threading.Thread(target=node.connect_websocket, daemon=True)
    ws_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ws_client:
            node.ws_client.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()