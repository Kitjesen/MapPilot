#!/usr/bin/python3
import os
import sys
import time
import pickle
import numpy as np
import open3d as o3d
import os
os.environ['NUMEXPR_MAX_THREADS'] = '4'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from tomogram import Tomogram

sys.path.append('../')
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class Tomography(Node):
    def __init__(self, cfg, scene_cfg):
        super().__init__('pointcloud_tomography')
        self.export_dir = rsg_root + cfg.map.export_dir
        self.pcd_file = scene_cfg.pcd.file_name
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = Tomogram(scene_cfg)
        points = self.loadPCD(self.pcd_file)

        # Process
        self.process(points)

    def initROS(self, cfg):
        self.map_frame = cfg.ros.map_frame

        pointcloud_topic = cfg.ros.pointcloud_topic
        # ROS 2 uses QoS profile instead of latch
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, qos_profile)

        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        layer_G_topic = cfg.ros.layer_G_topic
        layer_C_topic = cfg.ros.layer_C_topic
        for i in range(self.n_slice):
            layer_G_pub = self.create_publisher(PointCloud2, layer_G_topic + str(i), qos_profile)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = self.create_publisher(PointCloud2, layer_C_topic + str(i), qos_profile)
            self.layer_C_pub_list.append(layer_C_pub)

        tomogram_topic = cfg.ros.tomogram_topic
        self.tomogram_pub = self.create_publisher(PointCloud2, tomogram_topic, qos_profile)

    def loadPCD(self, pcd_file):
        pcd = o3d.io.read_point_cloud(rsg_root + "/rsc/pcd/" + pcd_file)
        points = np.asarray(pcd.points).astype(np.float32)
        self.get_logger().info("PCD points: %d" % points.shape[0])

        if points.shape[1] > 3:
            points = points[:, :3]
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        self.get_logger().info("Map center: [%.2f, %.2f]" % (self.center[0], self.center[1]))
        self.get_logger().info("Dim_x: %d" % self.map_dim_x)
        self.get_logger().info("Dim_y: %d" % self.map_dim_y)
        self.get_logger().info("Num slices init: %d" % n_slice_init)

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = 10

        """ 
        CPU time benchmark.
        The function is repeatedly run for n_repeat times to calculate the average processing time of each modules.
        The time of the first warm-up run is excluded to reduce timing fluctuation and exclude the overhead in initial invocations.
        """
        for i in range(n_repeat + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_cpu = self.tomogram.point2map(points)

            if i > 0:
                t_map += t_cpu['t_map']
                t_trav += t_cpu['t_trav']
                t_simp += t_cpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        if n_repeat > 0:
            self.get_logger().info("Num slices simp: %d" % layers_g.shape[0])
            self.get_logger().info("Num repeats (for benchmarking only): %d" % n_repeat)
            self.get_logger().info(" -- avg t_map  (ms): %f" % (t_map / n_repeat))
            self.get_logger().info(" -- avg t_trav (ms): %f" % (t_trav / n_repeat))
            self.get_logger().info(" -- avg t_simp (ms): %f" % (t_simp / n_repeat))
            self.get_logger().info(" -- avg t_all  (ms): %f" % (t_all / n_repeat))

        self.n_slice = layers_g.shape[0]

        map_file = os.path.splitext(self.pcd_file)[0]
        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)), map_file)

        cfg = Config()
        self.initROS(cfg)
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)

    def exportTomogram(self, tomogram, map_file):        
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        with open(self.export_dir + file_name, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info("Tomogram exported: %s" % file_name)

    def publishPoints(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        point_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = point_cloud2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy() 
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
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

        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = np.concatenate((global_points, valid_points), axis=0)
        
        points_msg = point_cloud2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\', \'Room\',\'Stairs\',\'Common\',\'Floor\']')
    parsed_args, unknown = parser.parse_known_args()

    cfg = Config()
    scene_cfg = getattr(__import__('config'), 'Scene' + parsed_args.scene)

    # ROS 2 需要将 sys.argv 传递给 rclpy.init
    import sys
    rclpy.init(args=sys.argv if args is None else args)
    
    mapping = Tomography(cfg, scene_cfg)

    try:
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        pass
    finally:
        mapping.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
