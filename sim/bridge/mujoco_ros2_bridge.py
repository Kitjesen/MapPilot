"""
MuJoCo ↔ ROS2 Bridge

订阅 /nav/cmd_vel → 自研底层控制器接口
发布 /mujoco/pos_w_pointcloud (PointCloud2)  — 与 mujoco_ray_caster ROS2 demo topic 保持一致
发布 /nav/odometry (Odometry, 50Hz)
发布 TF: map → odom → body

参考:
  mujoco_ray_caster ROS2 demo: d->sensordata + pos_w_data_point → PointCloud2
  https://github.com/Albusgive/mujoco_ray_caster/blob/main/demo/ROS2/colcon/src/ray_caster/src/sensor_data.cpp
"""
import numpy as np
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped, TransformStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import TransformBroadcaster
    import builtin_interfaces.msg
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print('[Bridge] ROS2 not available — running standalone')


def _pack_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> 'PointCloud2':
    """(N,3) float32 → sensor_msgs/PointCloud2"""
    msg             = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp    = stamp
    msg.height      = 1
    msg.width       = len(points)
    msg.is_dense    = False
    msg.is_bigendian = False
    msg.fields      = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step  = 12
    msg.row_step    = 12 * len(points)
    msg.data        = points.astype(np.float32).tobytes()
    return msg


class MuJoCoROS2Bridge:
    """
    MuJoCo 仿真与 ROS2 nav 栈的双向桥接。

    Topic 命名与 mujoco_ray_caster ROS2 demo 保持一致，
    方便直接换用 C++ demo 节点。
    """

    # 与 mujoco_ray_caster demo 一致的 topic 名称
    TOPIC_LIDAR = '/mujoco/pos_w_pointcloud'
    TOPIC_ODOM  = '/nav/odometry'
    TOPIC_CMD   = '/nav/cmd_vel'

    def __init__(self, model, data, lidar_sensor=None,
                 robot_body: str = 'base_link',
                 lidar_freq: float = 10.0,
                 odom_freq:  float = 50.0):
        self.model        = model
        self.data         = data
        self.lidar_sensor = lidar_sensor
        self.robot_body   = robot_body

        self._odom_dt     = 1.0 / odom_freq
        self._lidar_dt    = 1.0 / lidar_freq
        self._last_odom   = 0.0
        self._last_lidar  = 0.0

        self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
        self._cmd_ts = 0.0
        self.CMD_TIMEOUT = 0.5

        # terrain_analysis 订阅的是 /livox/lidar 或 /nav/map_cloud
        # 这里同时发布到两个 topic，兼容不同配置
        self.TOPIC_LIDAR_NAV = '/livox/lidar'

        if not HAS_ROS2:
            self._node = None
            return

        rclpy.init(args=None)
        self._node = rclpy.create_node('mujoco_sim')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers — 与 mujoco_ray_caster demo 同名 topic
        self._pub_lidar     = self._node.create_publisher(
            PointCloud2, self.TOPIC_LIDAR, qos)
        # 同时发到 nav_stack 期望的 topic
        self._pub_lidar_nav = self._node.create_publisher(
            PointCloud2, self.TOPIC_LIDAR_NAV, qos)
        self._pub_odom      = self._node.create_publisher(
            Odometry, self.TOPIC_ODOM, qos)

        # Subscriber
        self._sub_cmd = self._node.create_subscription(
            TwistStamped, self.TOPIC_CMD, self._cmd_cb, qos)

        self._tf_br = TransformBroadcaster(self._node)

        print(f'[Bridge] ROS2 node "mujoco_sim" ready')
        print(f'  Pub: {self.TOPIC_LIDAR}, {self.TOPIC_LIDAR_NAV}, {self.TOPIC_ODOM}')
        print(f'  Sub: {self.TOPIC_CMD}')

    def _cmd_cb(self, msg):
        self._cmd_vx = msg.twist.linear.x
        self._cmd_vy = msg.twist.linear.y
        self._cmd_wz = msg.twist.angular.z
        self._cmd_ts = time.time()

    def _get_robot_state(self):
        try:
            import mujoco
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.robot_body)
            pos  = self.data.xpos[bid].copy()
            quat = self.data.xquat[bid].copy()  # wxyz
            vel  = self.data.cvel[bid].copy() if hasattr(self.data, 'cvel') else np.zeros(6)
        except Exception:
            pos, quat, vel = np.zeros(3), np.array([1.,0.,0.,0.]), np.zeros(6)
        return pos, quat, vel

    def _apply_cmd(self):
        """透传速度到自研底层控制器。"""
        if time.time() - self._cmd_ts > self.CMD_TIMEOUT:
            self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
        # TODO: 填入自研底层控制器接口
        # e.g. self.data.ctrl[idx_vx] = self._cmd_vx
        pass

    def _sim_stamp(self):
        t = self.data.time
        s = builtin_interfaces.msg.Time()
        s.sec = int(t)
        s.nanosec = int((t - int(t)) * 1e9)
        return s

    def _pub_odom_tf(self):
        pos, quat, vel = self._get_robot_state()
        stamp = self._sim_stamp()

        # Odometry
        o = Odometry()
        o.header.stamp = stamp; o.header.frame_id = 'odom'; o.child_frame_id = 'body'
        o.pose.pose.position.x    = float(pos[0])
        o.pose.pose.position.y    = float(pos[1])
        o.pose.pose.position.z    = float(pos[2])
        o.pose.pose.orientation.w = float(quat[0])
        o.pose.pose.orientation.x = float(quat[1])
        o.pose.pose.orientation.y = float(quat[2])
        o.pose.pose.orientation.z = float(quat[3])
        o.twist.twist.linear.x  = float(self._cmd_vx)
        o.twist.twist.angular.z = float(self._cmd_wz)
        self._pub_odom.publish(o)

        # TF odom → body
        def _tf(parent, child, p, q):
            t = TransformStamped()
            t.header.stamp = stamp; t.header.frame_id = parent; t.child_frame_id = child
            t.transform.translation.x = float(p[0])
            t.transform.translation.y = float(p[1])
            t.transform.translation.z = float(p[2])
            t.transform.rotation.w = float(q[0])
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            return t

        self._tf_br.sendTransform(_tf('odom', 'body', pos, quat))
        self._tf_br.sendTransform(_tf('map', 'odom',
                                      np.zeros(3), np.array([1.,0.,0.,0.])))

    def _pub_cloud(self):
        if self.lidar_sensor is None:
            return
        pts = self.lidar_sensor.scan()   # (N, 3) float32, world frame
        if len(pts) == 0:
            return
        stamp = self._sim_stamp()
        msg = _pack_pointcloud2(pts, 'map', stamp)
        # 同时发到两个 topic
        self._pub_lidar.publish(msg)
        self._pub_lidar_nav.publish(msg)

    def spin_once(self):
        now = time.time()
        self._apply_cmd()

        if self._node is None:
            return

        if now - self._last_odom >= self._odom_dt:
            self._pub_odom_tf()
            self._last_odom = now

        if now - self._last_lidar >= self._lidar_dt:
            self._pub_cloud()
            self._last_lidar = now

        rclpy.spin_once(self._node, timeout_sec=0.0)

    def destroy(self):
        if self._node:
            self._node.destroy_node()
        if HAS_ROS2 and rclpy.ok():
            rclpy.shutdown()
