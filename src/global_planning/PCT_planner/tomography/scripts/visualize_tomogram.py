#!/usr/bin/python3
import os
import sys
import pickle
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# Add parent directory to path to import config
sys.path.append('../')
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config

class TomogramVisualizer(Node):
    def __init__(self, scene_name):
        super().__init__('tomogram_visualizer')
        
        # Load Config
        self.cfg = Config()
        
        # Determine paths
        # Assuming run from tomography/scripts/
        rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'
        self.export_dir = rsg_root + self.cfg.map.export_dir
        self.pickle_path = os.path.join(self.export_dir, scene_name + '.pickle')

        if not os.path.exists(self.pickle_path):
            self.get_logger().error(f"Pickle file not found: {self.pickle_path}")
            self.get_logger().error("Please run tomography.py first or copy the .pickle file to rsc/tomogram/")
            sys.exit(1)

        self.get_logger().info(f"Loading tomogram from: {self.pickle_path}")
        
        # Load Data
        self.load_tomogram()
        
        # Initialize ROS Publishers
        self.init_ros()
        
        # Publish
        self.get_logger().info("Publishing tomogram visualization...")
        self.publish_all()
        
        # Create a timer to keep publishing (optional, for late joiners if not using latching correctly)
        # self.create_timer(2.0, self.publish_all)

    def load_tomogram(self):
        with open(self.pickle_path, 'rb') as handle:
            data_dict = pickle.load(handle)
        
        # Restore metadata
        self.resolution = data_dict['resolution']
        self.center = data_dict['center']
        self.slice_h0 = data_dict['slice_h0']
        self.slice_dh = data_dict['slice_dh']
        tomogram_data = data_dict['data'] # Shape: (5, n_slice, dim_x, dim_y)
        
        # Unpack layers (matching tomography.py structure)
        # 0: layers_t (traversability)
        # 1: trav_grad_x
        # 2: trav_grad_y
        # 3: layers_g (geometric elevation)
        # 4: layers_c (ceiling elevation, optional for vis but usually included)
        
        self.layers_t = tomogram_data[0]
        self.layers_g = tomogram_data[3]
        
        # Handle shapes
        self.n_slice = self.layers_g.shape[0]
        self.map_dim_x = self.layers_g.shape[1]
        self.map_dim_y = self.layers_g.shape[2]
        
        self.get_logger().info(f"Loaded map: {self.map_dim_x}x{self.map_dim_y} with {self.n_slice} slices")

        # Prepare grid points for visualization
        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

    def init_ros(self):
        self.map_frame = self.cfg.ros.map_frame
        
        # QoS setup
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Important for static map
        )

        # Publishers
        self.tomogram_pub = self.create_publisher(
            PointCloud2, self.cfg.ros.tomogram_topic, qos_profile)
            
        # Optional: Layer publishers if you want to see individual slices
        # self.layer_G_pub_list = []
        # layer_G_topic = self.cfg.ros.layer_G_topic
        # for i in range(self.n_slice):
        #     pub = self.create_publisher(PointCloud2, layer_G_topic + str(i), qos_profile)
        #     self.layer_G_pub_list.append(pub)

    def publish_all(self):
        # We focus on the combined tomogram visualization
        self.publish_tomogram(self.layers_g, self.layers_t)

    def publish_tomogram(self, layers_g, layers_t):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy() 
        
        # Prepare points container
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
        
        # Logic copied from tomography.py to merge layers for visualization
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            
            layer_points[:, 2] = vis_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = vis_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            
            if global_points is None:
                global_points = valid_points
            else:
                global_points = np.concatenate((global_points, valid_points), axis=0)

        # Last layer
        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        
        if global_points is not None:
            global_points = np.concatenate((global_points, valid_points), axis=0)
        else:
            global_points = valid_points
        
        # Publish
        if global_points is not None and len(global_points) > 0:
            points_msg = point_cloud2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
            self.tomogram_pub.publish(points_msg)
            self.get_logger().info(f"Published tomogram with {len(global_points)} points")
        else:
            self.get_logger().warn("Tomogram point cloud is empty!")

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, required=True, help='Name of the scene (e.g. Spiral, Building)')
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    
    visualizer = TomogramVisualizer(parsed_args.scene)
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
