#!/usr/bin/env python3
import sys
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

# 库路径（先加 planner，再导入 Config，避免被 tomography 的 config 覆盖）
current_dir = os.path.dirname(os.path.abspath(__file__))
_planner_root = os.path.join(current_dir, "..")
sys.path.insert(0, _planner_root)
sys.path.append(os.path.join(current_dir, "../lib"))

from config import Config
from utils import traj2ros
from planner_wrapper import TomogramPlanner


class GlobalPlanner(Node):
    """PCT 全局规划节点：加载 tomogram 地图，订阅目标点，规划路径并发布 /pct_path。"""
    def __init__(self):
        super().__init__('pct_global_planner')

        # 1. 并发控制
        self.callback_group = ReentrantCallbackGroup()
        self.plan_lock = threading.Lock()

        # 2. 参数：默认值来自 config/param.py，运行时可经 launch/ROS 再配置
        cfg = Config()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_file', cfg.node.map_file),
                ('map_frame', cfg.node.map_frame),
                ('robot_frame', cfg.node.robot_frame),
                ('min_plan_interval', cfg.node.min_plan_interval),
                ('default_goal_height', cfg.node.default_goal_height),
                ('tomogram_resolution', cfg.wrapper.tomogram_resolution),
                ('tomogram_slice_dh', cfg.wrapper.tomogram_slice_dh),
                ('tomogram_ground_h', cfg.wrapper.tomogram_ground_h),
                ('publish_map_pointcloud', cfg.node.publish_map_pointcloud),
                ('publish_tomogram', cfg.node.publish_tomogram),
                ('use_quintic', cfg.planner.use_quintic),
                ('max_heading_rate', cfg.planner.max_heading_rate),
                ('obstacle_thr', cfg.planner.obstacle_thr),
            ]
        )
        self.tomo_file = self.get_parameter('map_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_plan_interval = self.get_parameter('min_plan_interval').value
        self.default_goal_height = self.get_parameter('default_goal_height').value
        self.tomogram_resolution = self.get_parameter('tomogram_resolution').value
        self.tomogram_slice_dh = self.get_parameter('tomogram_slice_dh').value
        self.tomogram_ground_h = self.get_parameter('tomogram_ground_h').value
        self.publish_map_pointcloud = self.get_parameter('publish_map_pointcloud').value
        self.publish_tomogram = self.get_parameter('publish_tomogram').value
        self.use_quintic = self.get_parameter('use_quintic').value
        self.max_heading_rate = self.get_parameter('max_heading_rate').value
        self.obstacle_thr = self.get_parameter('obstacle_thr').value
        cfg.node.map_file = self.tomo_file
        cfg.node.map_frame = self.map_frame
        cfg.node.robot_frame = self.robot_frame
        cfg.node.min_plan_interval = self.min_plan_interval
        cfg.node.default_goal_height = self.default_goal_height
        cfg.node.publish_map_pointcloud = self.publish_map_pointcloud
        cfg.node.publish_tomogram = self.publish_tomogram
        cfg.wrapper.tomogram_resolution = self.tomogram_resolution
        cfg.wrapper.tomogram_slice_dh = self.tomogram_slice_dh
        cfg.wrapper.tomogram_ground_h = self.tomogram_ground_h
        cfg.planner.use_quintic = self.use_quintic
        cfg.planner.max_heading_rate = self.max_heading_rate
        cfg.planner.obstacle_thr = self.obstacle_thr

        self.planner = TomogramPlanner(cfg)
        try:
            self.planner.loadTomogram(
                self.tomo_file,
                resolution=cfg.wrapper.tomogram_resolution,
                slice_dh=cfg.wrapper.tomogram_slice_dh,
                ground_h=cfg.wrapper.tomogram_ground_h,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load map from {self.tomo_file}: {e}")
            self.get_logger().error("Navigation unavailable - please provide a valid map file")
            return
        
        # 4. 发布
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, "/pct_path", qos_latched)
        self.status_pub = self.create_publisher(String, '/pct_planner/status', 10)
        from sensor_msgs.msg import PointCloud2
        self.map_pointcloud_pub = self.create_publisher(PointCloud2, '/map_pointcloud', qos_latched)
        self.tomogram_pub = self.create_publisher(PointCloud2, '/tomogram', qos_latched)
        if self.publish_map_pointcloud:
            self._publish_map_pointcloud_once()
        if self.publish_tomogram:
            self._publish_tomogram_once()

        # 5. TF
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 6. 订阅
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10, 
            callback_group=self.callback_group)
        self.goal_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.goal_point_callback, 10, 
            callback_group=self.callback_group)
        
        self.last_plan_time = self.get_clock().now()

        self.get_logger().info(f'PCT Global Planner Ready | Map: {self.tomo_file}')
        self.publish_status("IDLE")

    def _publish_map_pointcloud_once(self):
        """将加载的地图以 PointCloud2 发布到 /map_pointcloud，RViz 固定帧为 map。"""
        tomo = self.tomo_file
        if os.path.isabs(tomo):
            base = os.path.splitext(os.path.basename(tomo))[0]
            pcd_path = os.path.join(os.path.dirname(tomo), base + '.pcd')
        else:
            try:
                from ament_index_python.packages import get_package_share_directory
                pct_share = get_package_share_directory('pct_planner')
                base = tomo[:-4] if tomo.endswith('.pcd') else tomo
                for sub in ('tomogram', 'pcd'):
                    cand = os.path.join(pct_share, 'rsc', sub, base + '.pcd')
                    if os.path.isfile(cand):
                        pcd_path = cand
                        break
                else:
                    pcd_path = os.path.join(pct_share, 'rsc', 'tomogram', base + '.pcd')
            except Exception:
                pcd_path = ''
        if not pcd_path or not os.path.isfile(pcd_path):
            return
        try:
            import open3d as o3d
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs_py import point_cloud2
            from std_msgs.msg import Header
            pcd = o3d.io.read_point_cloud(pcd_path)
            # 点数过多时 RViz2 易卡顿，超过 10 万点做体素下采样
            if len(pcd.points) > 100000:
                pcd = pcd.voxel_down_sample(0.15)
            points = np.asarray(pcd.points).astype(np.float32)
            if points.size == 0:
                return
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.map_frame
            msg = point_cloud2.create_cloud_xyz32(header, points)
            self.map_pointcloud_pub.publish(msg)
            self.get_logger().info(f'Published map point cloud: {pcd_path} ({len(points)} pts)')
        except Exception as e:
            self.get_logger().warn(f'Could not publish map point cloud: {e}')

    def _publish_tomogram_once(self):
        """将 tomogram（高程 + 可穿越性）以 PointCloud2 发布到 /tomogram 供 RViz 显示。"""
        if not hasattr(self.planner, 'layers_g') or not hasattr(self.planner, 'layers_t'):
            return
        try:
            import importlib.util
            _script_dir = os.path.dirname(os.path.abspath(__file__))
            _tomo_proto = None
            # 1) install 下: lib/pct_planner -> ../../share/pct_planner/tomography/config/prototype.py
            _share_proto = os.path.normpath(os.path.join(_script_dir, '..', '..', 'share', 'pct_planner', 'tomography', 'config', 'prototype.py'))
            if os.path.isfile(_share_proto):
                _tomo_proto = _share_proto
            if not _tomo_proto:
                try:
                    from ament_index_python.packages import get_package_share_directory
                    _pkg_share = get_package_share_directory('pct_planner')
                    _tomo_proto = os.path.join(_pkg_share, 'tomography', 'config', 'prototype.py')
                except Exception:
                    pass
            if not _tomo_proto or not os.path.isfile(_tomo_proto):
                _tomo_proto = os.path.join(_script_dir, '..', '..', 'tomography', 'config', 'prototype.py')
            if not os.path.isfile(_tomo_proto):
                raise FileNotFoundError(f"tomography config prototype not found: {_tomo_proto}")
            _spec = importlib.util.spec_from_file_location("tomo_prototype", _tomo_proto)
            _tomo_mod = importlib.util.module_from_spec(_spec)
            _spec.loader.exec_module(_tomo_mod)
            POINT_FIELDS_XYZI = _tomo_mod.POINT_FIELDS_XYZI
            GRID_POINTS_XYZI = _tomo_mod.GRID_POINTS_XYZI
            from std_msgs.msg import Header
            from sensor_msgs_py import point_cloud2
        except Exception as e:
            self.get_logger().warn(f'导入 tomography config 失败，无法发布 tomogram: {e}')
            return
        try:
            layers_g = self.planner.layers_g.copy()
            layers_t = self.planner.layers_t.copy()
            resolution = self.planner.resolution
            center = self.planner.center
            slice_dh = self.planner.slice_dh
            # map_dim is [x=W, y=H]; GRID_POINTS_XYZI expects (rows=H, cols=W) for layers_g indexing
            map_dim_x, map_dim_y = self.planner.map_dim[0], self.planner.map_dim[1]
            VISPROTO_I, VISPROTO_P = GRID_POINTS_XYZI(resolution, map_dim_y, map_dim_x)
            layer_points = VISPROTO_P.copy()
            layer_points[:, :2] += center
            n_slice = layers_g.shape[0]
            global_points = None
            for i in range(n_slice - 1):
                mask_h = (layers_g[i + 1] - layers_g[i]) < slice_dh
                layers_g[i, mask_h] = np.nan
                layers_t[i + 1, mask_h] = np.minimum(layers_t[i, mask_h], layers_t[i + 1, mask_h])
                layer_points[:, 2] = layers_g[i, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
                layer_points[:, 3] = layers_t[i, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
                valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
                global_points = valid_points if global_points is None else np.concatenate((global_points, valid_points), axis=0)
            layer_points[:, 2] = layers_g[-1, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
            layer_points[:, 3] = layers_t[-1, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            global_points = valid_points if global_points is None else np.concatenate((global_points, valid_points), axis=0)
            if len(global_points) > 0:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = self.map_frame
                msg = point_cloud2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
                self.tomogram_pub.publish(msg)
                self.get_logger().info(f'Published tomogram (traversability cost map): {len(global_points)} pts on /tomogram')
            else:
                self.get_logger().warn('Tomogram 点云为空')
        except Exception as e:
            self.get_logger().warn(f'发布 tomogram 失败: {e}')
        
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            pos = np.array([t.transform.translation.x, t.transform.translation.y], dtype=np.float32)
            return pos, t.transform.translation.z, True
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn(f"TF 等待: {e}", throttle_duration_sec=5.0)
            return None, None, False
        except Exception as e:
            self.get_logger().error(f"TF 错误: {e}")
            return None, None, False
        
    def goal_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        if z == 0.0:
            # 2D 工具 (RViz "2D Nav Goal", gRPC 未设 z) 发来 z=0
            # 自动从 tomogram 查找目标 XY 的地形表面高度，解决坡度地形规划失败问题
            terrain_z = self.planner.get_surface_height(np.array([x, y], dtype=np.float32))
            if terrain_z != 0.0:
                z = terrain_z
                self.get_logger().info(
                    f"2D goal auto-terrain: ({x:.1f}, {y:.1f}) → z={z:.2f}m"
                )
            else:
                z = self.default_goal_height
        self._handle_goal(x, y, z, "goal_pose")
    
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
        if self.plan_lock.locked():
            self.get_logger().warn("Planner is busy, dropping new goal")
            return
        start_pos, start_height, success = self.get_robot_pose()
        if not success:
            self.publish_status("NO_LOCALIZATION")
            return

        # 机器人物理 Z 可能低于 tomogram 最低切片 (slice_h0), 导致 plan() 找不到起点格栅
        # 例: building2_9 slice_h0=0.5, 但真实机器人地面 z≈0.0 → snap 到最低有效切片
        if self.tomogram_ground_h > 0.0 and start_height < self.tomogram_ground_h:
            self.get_logger().info(
                f"start_h {start_height:.2f} < tomogram_ground_h {self.tomogram_ground_h:.2f}, "
                f"snapping to ground slice"
            )
            start_height = self.tomogram_ground_h

        self.last_plan_time = current_time
        self.get_logger().info(
            f"Goal received ({source}): ({x:.2f}, {y:.2f}, {z:.2f}), "
            f"robot_pos=({start_pos[0]:.2f}, {start_pos[1]:.2f}, z={start_height:.2f})")
        self.pct_plan(start_pos, start_height, np.array([x, y], dtype=np.float32), z)

    def pct_plan(self, start_pos, start_h, end_pos, end_h):
        """在 tomogram 上规划路径并发布，线程安全。"""
        with self.plan_lock:
            self.publish_status("PLANNING")
            start_tick = time.time()
            
            try:
                traj_3d = self.planner.plan(start_pos, end_pos, start_h, end_h)
                
                duration_ms = (time.time() - start_tick) * 1000
                
                if traj_3d is not None and len(traj_3d) > 0:
                    path_msg = traj2ros(traj_3d)
                    path_msg.header.stamp = self.get_clock().now().to_msg()
                    path_msg.header.frame_id = self.map_frame
                    
                    self.path_pub.publish(path_msg)
                    self.get_logger().info(f"✓ Plan Success: {len(traj_3d)} pts in {duration_ms:.1f}ms")
                    self.publish_status("SUCCESS")
                else:
                    self.get_logger().error(f"✗ Plan Failed: No path found ({duration_ms:.1f}ms)")
                    # 发布空路径，通知适配器清除过期路径，停止追踪
                    empty_path = Path()
                    empty_path.header.stamp = self.get_clock().now().to_msg()
                    empty_path.header.frame_id = self.map_frame
                    self.path_pub.publish(empty_path)
                    self.publish_status("FAILED")

            except Exception as e:
                self.get_logger().error(f"Planner Core Internal Error: {e}")
                import traceback
                traceback.print_exc()
                # 同样发布空路径，确保适配器停止
                empty_path = Path()
                empty_path.header.stamp = self.get_clock().now().to_msg()
                empty_path.header.frame_id = self.map_frame
                self.path_pub.publish(empty_path)
                self.publish_status("ERROR")
    
    def publish_status(self, status: str):
        msg = String(data=status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPlanner()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(planner, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
