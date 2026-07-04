#!/usr/bin/env python3
"""
Building2_9 闂幆浠跨湡鑺傜偣 (ROS2 SITL)

鍏ㄦ爤娴嬭瘯: PCT A* 鍏ㄥ眬瑙勫垝 鈫?pct_path_adapter 鈫?localPlanner 鈫?pathFollower 鈫?cmd_vel 绉垎
鍦板浘:     building2_9.pickle (鐪熷疄寤虹瓚鐣寗鍥? 17脳18m, 97脳94 grid @ 0.2m/voxel)

璇ヨ妭鐐规壙鎷?"铏氭嫙鏈哄櫒浜? 瑙掕壊:
  - 鍙戝竷骞冲潶鍚堟垚鐐逛簯 鈫?/slam/map_cloud + /nav/terrain_map + /nav/terrain_map_ext
    (鏇夸唬鐪熷疄 LiDAR + terrain_analysis, 閬垮厤寤虹瓚澧欎綋琚鍒や负杩戝満闅滅鐗╄Е鍙?E-stop)
  - 鎺ユ敹 /nav/cmd_vel 鈫?绉垎浜岀淮杩愬姩瀛?鈫?鏇存柊浣嶅Э
  - 鎸佺画鍙戝竷 /nav/stop = 0 (娓呴櫎 pathFollower safetyStop_ 鏃楁爣)
  - 棰勭儹 WARMUP_S 绉掑悗鍙戦€?/nav/goal_pose 瑙﹀彂 PCT 瑙勫垝
  - 鍒拌揪鐩爣鎴栬秴鏃跺悗淇濆瓨 /tmp/sim_result.json + /tmp/sim_traj.png

閰嶇疆 (閫氳繃鐜鍙橀噺, 鐢?sim_navigation.launch.py 娉ㄥ叆):
  SIM_GOAL_X  鐩爣 X (m), 榛樿  5.0
  SIM_GOAL_Y  鐩爣 Y (m), 榛樿  7.3  (Corridor_E 鍦烘櫙)
  SIM_GOAL_Z  鐩爣 Z (m), 榛樿  0.0  (>0.1 瑙﹀彂 3D 璺ㄦゼ灞傝鍒?
  SIM_START_X 璧风偣 X (m), 榛樿 -5.5
  SIM_START_Y 璧风偣 Y (m), 榛樿  7.3

building2_9 涓栫晫鍧愭爣鑼冨洿: X鈭圼-7.95, 10.85] m, Y鈭圼-10.09, 9.31] m
"""
import math
import os
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Int8, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from runtime.runtime_interface import FRAMES, TOPICS

# 鈹€鈹€ 榛樿鍙傛暟 (琚幆澧冨彉閲忚鐩? 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_DEF_START_X   = float(os.environ.get('SIM_START_X',   '-5.5'))
_DEF_START_Y   = float(os.environ.get('SIM_START_Y',    '7.3'))
_DEF_START_Z   = float(os.environ.get('SIM_START_Z',    '0.0'))  # 鍦板浘鍒囩墖璧峰楂樺害 (building2_9: 0.5)
_DEF_START_YAW = 0.0   # rad
_DEF_GOAL_X    = float(os.environ.get('SIM_GOAL_X',    '5.0'))
_DEF_GOAL_Y    = float(os.environ.get('SIM_GOAL_Y',    '7.3'))
_DEF_GOAL_Z    = float(os.environ.get('SIM_GOAL_Z',    '0.0'))  # >0.1 鈫?3D璺ㄦゼ灞傝鍒?

# 鈹€鈹€ 灏忚溅 URDF 璺緞 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_URDF_PATH = os.path.join(os.path.dirname(__file__), 'simple_car.urdf')

# 鈹€鈹€ 宸€熼┍鍔ㄥ弬鏁?(涓?simple_car.urdf 涓€鑷? 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
WHEEL_RADIUS = 0.07   # m
WHEEL_BASE   = 0.34   # m (涓よ疆闂磋窛, left +0.17 right -0.17)

# 鈹€鈹€ 浠跨湡鍙傛暟 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
GOAL_THRESH = 0.5    # m, 鍒拌揪鍒ゅ畾璺濈
MAX_SECS    = 180.0  # s, 瓒呮椂鏃堕棿
DT          = 0.05   # s, 涓诲惊鐜懆鏈?(20 Hz)
TERRAIN_R   = 10.0   # m, 鍚堟垚骞冲潶鍦板舰鍗婂緞
TERRAIN_S   =  0.4   # m, 鏍兼爡闂磋窛
WARMUP_S    = float(os.environ.get('SIM_WARMUP_S', '6.0'))  # s, 棰勭儹 (鈮?joyToSpeedDelay=2.0s)
GOAL_RESEND_S = 3600.0 # s, 绂佺敤涓€旈噸鍙?(瑙勫垝鍣ㄥ凡鏈夎矾寰? 鏃犻渶閲嶅彂瀵艰嚧閲嶈鍒掗棯鐑?
LOOP_PAUSE_S  =  5.0 # s, 鍒拌揪鐩爣鍚庢殏鍋滃啀閲嶇疆 (婕旂ず寰幆)

# 涓栫晫鍧愭爣杈圭晫 (閫氳繃鐜鍙橀噺瑕嗙洊锛屾敮鎸佷笉鍚屽湴鍥?
# 宸ュ巶鍦烘櫙: SIM_MAP_X_MIN=-5 SIM_MAP_X_MAX=85 SIM_MAP_Y_MIN=-5 SIM_MAP_Y_MAX=60
MAP_X_MIN = float(os.environ.get('SIM_MAP_X_MIN', '-7.5'))
MAP_X_MAX = float(os.environ.get('SIM_MAP_X_MAX',  '10.5'))
MAP_Y_MIN = float(os.environ.get('SIM_MAP_Y_MIN', '-9.5'))
MAP_Y_MAX = float(os.environ.get('SIM_MAP_Y_MAX',   '9.0'))


# 鈹€鈹€ 寤虹瓚鐐逛簯 PCD 榛樿璺緞 (鎸変紭鍏堢骇鏌ユ壘) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
_OCTOPLANNER3D_BUILDING2_9_PCD = os.path.join(
    _REPO_ROOT,
    'src', 'nav', 'services', 'plan', 'global_planner', 'algorithm',
    'OctoPlanner3D', 'octomap', 'pcd_files', 'building2_9.pcd',
)

_PCD_CANDIDATES = [
    os.environ.get('SIM_PCD_PATH', ''),
    _OCTOPLANNER3D_BUILDING2_9_PCD,
    '/home/sunrise/data/SLAM/navigation/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/octomap/pcd_files/building2_9.pcd',
    '/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/rsc/pcd/building2_9.pcd',
    '/home/sunrise/data/SLAM/navigation/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/pcd/building2_9.pcd',
]


def _find_pcd() -> Optional[str]:
    for p in _PCD_CANDIDATES:
        if p and os.path.exists(p):
            return p
    # 灏濊瘯 ament 瀹夎璺緞
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('pct_planner')
        p = os.path.join(share, 'rsc', 'pcd', 'building2_9.pcd')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return None


def _load_pcd_binary(pcd_path: str, max_pts: int = 80000) -> Optional[np.ndarray]:
    """
    鍔犺浇 binary PCD 鏂囦欢 鈫?(N, 4) float32 [x, y, z, intensity=z].

    - 鏀寔 plain binary 鍜?binary 鏍煎紡 (鍚?x y z ... 瀛楁)
    - 瓒呰繃 max_pts 鏃堕殢鏈洪檷閲囨牱
    - intensity 鍒楄涓?z 楂樺害鍊? 渚?Foxglove 鎸夐珮搴︾潃鑹?
    """
    try:
        with open(pcd_path, 'rb') as f:
            raw = f.read()

        # 瑙ｆ瀽 ASCII 澶撮儴
        pos, meta = 0, {}
        while pos < len(raw):
            end = raw.find(b'\n', pos)
            if end == -1:
                break
            line = raw[pos:end].decode('ascii', errors='ignore').strip()
            parts = line.split()
            if parts:
                meta[parts[0].upper()] = parts[1:]
            pos = end + 1
            if parts and parts[0].upper() == 'DATA':
                break  # data_offset = pos

        data_offset = pos
        data_type = meta.get('DATA', ['ascii'])[0].lower()
        n_pts  = int(meta.get('POINTS', [0])[0])
        fields = meta.get('FIELDS', [])
        sizes  = [int(s) for s in meta.get('SIZE', [])]
        types  = meta.get('TYPE', [])

        if data_type != 'binary' or 'x' not in fields:
            return None

        # 鏋勫缓 numpy dtype (姣忎釜瀛楁 float32/int32/uint32)
        np_map = {'F': 'f4', 'I': 'i4', 'U': 'u4'}
        dt = np.dtype([(f, np_map.get(t, 'f4'))
                       for f, t in zip(fields, types)])

        arr = np.frombuffer(raw[data_offset: data_offset + n_pts * dt.itemsize],
                            dtype=dt)
        x = arr['x'].astype(np.float32)
        y = arr['y'].astype(np.float32)
        z = arr['z'].astype(np.float32)

        pts = np.column_stack([x, y, z, z.copy()])   # intensity = z for height colormap

        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]

        return pts
    except Exception as e:
        return None


def _make_xyzi_cloud(pts: np.ndarray, frame_id: str, stamp=None) -> PointCloud2:
    """灏?(N,4) float32 [x,y,z,i] 鏁扮粍鎵撳寘涓?PointCloud2 娑堟伅銆?""
    arr = pts.astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width  = len(arr)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step   = 16 * len(arr)
    msg.data       = arr.tobytes()
    return msg


def _flat_cloud(cx: float, cy: float, cz: float = 0.0, stamp=None) -> PointCloud2:
    """Build a flat XYZI terrain cloud around the simulated robot.

    The cloud is anchored at the current robot height so localPlanner does not
    treat other floors in a multi-floor scene as immediate obstacles.
    """
    r = int(TERRAIN_R / TERRAIN_S)
    pts = []
    for ix in range(-r, r + 1):
        for iy in range(-r, r + 1):
            pts.append([cx + ix * TERRAIN_S, cy + iy * TERRAIN_S, cz, cz])
    arr = np.array(pts, dtype=np.float32)

    msg = PointCloud2()
    msg.header.frame_id = FRAMES.odom
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step   = 16 * len(arr)
    msg.data       = arr.tobytes()
    return msg


class SimRobotNode(Node):
    def __init__(self):
        super().__init__('sim_robot_node')

        # 鈹€鈹€ 鏈哄櫒浜虹姸鎬?鈹€鈹€
        self.x   = _DEF_START_X
        self.y   = _DEF_START_Y
        self.yaw = _DEF_START_YAW
        self.gx  = _DEF_GOAL_X
        self.gy  = _DEF_GOAL_Y

        self.vx = self.vy = self.wz = 0.0
        self.z    = _DEF_START_Z  # 鏈哄櫒浜哄綋鍓峑楂樺害 (building2_9 鍦伴潰灞?0.5, 3D妯″紡涓嬫洿鏂?
        self.vz   = 0.0   # Z杞撮€熷害 (m/s)
        self._mode_3d    = False  # True=PCT 3D璺緞璺熻釜, False=PCT 2D鎺у埗
        self._3d_path    = []     # 3D璺偣鍒楄〃 [(x,y,z), ...]锛堟潵鑷狿CT 3D A*锛?
        self._gz         = 0.0    # 3D鐩爣Z楂樺害
        self._3d_waiting = False  # True=绛夊緟 PCT 3D A* 瑙勫垝缁撴灉锛堥伩鍏?tick 璇垽鍒拌揪锛?
        self.cmd_count = 0
        self.traj = []
        self.goal_reached = False
        self.t_start = None
        self.phase = 'warmup'   # warmup 鈫?running 鈫?pausing 鈫?(reset)
        self._user_goal_set = False   # 鐢ㄦ埛鎵嬪姩璁剧疆杩囩洰鏍囧悗, loop reset 涓嶈鐩栫洰鏍?

        # 鈹€鈹€ 杞﹁疆瑙掑害 (for JointState + RViz 杞﹁疆婊氬姩) 鈹€鈹€
        self._left_wheel_angle  = 0.0
        self._right_wheel_angle = 0.0

        # 鈹€鈹€ TF 鈹€鈹€
        self.static_br = StaticTransformBroadcaster(self)
        self.tf_br     = TransformBroadcaster(self)
        self._pub_static_tf()

        # 鈹€鈹€ Publishers 鈹€鈹€
        self.pub_odom    = self.create_publisher(Odometry,    TOPICS.odometry,        10)
        self.pub_cloud   = self.create_publisher(PointCloud2, TOPICS.map_cloud,       10)
        self.pub_terrain = self.create_publisher(PointCloud2, TOPICS.terrain_map,     10)
        self.pub_te      = self.create_publisher(PointCloud2, TOPICS.terrain_map_ext, 10)
        self.pub_goal    = self.create_publisher(PoseStamped, TOPICS.goal_pose,       10)
        self.pub_stop    = self.create_publisher(Int8,        TOPICS.stop,            10)

        # robot_state_publisher 闇€瑕?/joint_states (杞瓙鍏宠妭)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        # 鏈哄櫒浜轰綅缃ぇ鐞冩爣璁?(榛勮壊, Z=0.5m, 鐩村緞0.6m, 鍦ㄥ缓绛戠偣浜戜笂鏂规竻鏅板彲瑙?
        self.pub_marker = self.create_publisher(Marker, TOPICS.robot_marker, 10)

        # /robot_description latched (TRANSIENT_LOCAL): RViz/Foxglove 闅忔椂鍙鍙?URDF
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_desc  = self.create_publisher(String,       '/robot_description',  latched)
        # /nav/building_cloud: 鐪熷疄寤虹瓚 3D 鐐逛簯 (闈?latched, 姣?3s 閲嶅彂纭繚 RViz 濮嬬粓鍙)
        # 娉? latched 鐨勬椂闂存埑浼氳繃鏈熷鑷?RViz2 message filter 涓㈠純, 鏀逛负瀹氭椂閲嶅彂
        self.pub_bldg  = self.create_publisher(PointCloud2,  TOPICS.building_cloud, 10)
        self._bldg_last_pub  = 0.0   # 涓婃寤虹瓚鐐逛簯鍙戝竷鐨?wall-clock 鏃堕棿
        self._publish_robot_description()
        self._publish_building_cloud()

        # 鈹€鈹€ Subscribers 鈹€鈹€
        # pathFollower 浠?BEST_EFFORT 50 Hz 鍙戝竷 cmd_vel
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(TwistStamped, TOPICS.cmd_vel,        self._on_cmd,           be)
        # 鑸偣璺熻釜浜嬩欢 (JSON)
        self.create_subscription(String,       TOPICS.adapter_status, self._on_adapter,        10)
        # 鍏ㄥ眬瑙勫垝鍣ㄧ姸鎬?(pathFollower 鍙戝竷 GOAL_REACHED/STUCK 绛?
        self.create_subscription(String,       TOPICS.planner_status, self._on_planner,        10)
        # 鎵嬪姩鐩爣 鈥?PoseStamped (RViz2 "2D Nav Goal" 鈫?/nav/goal_pose)
        self.create_subscription(PoseStamped,  TOPICS.goal_pose,      self._on_goal_pose,      10)
        # 鎵嬪姩鐩爣 鈥?PointStamped (RViz2 "Publish Point" 鈫?/clicked_point, 鏀寔鐪?XYZ)
        self.create_subscription(PointStamped, '/clicked_point',      self._on_clicked_point,  10)
        # PCT 瑙勫垝鍣ㄨ緭鍑?(鍚?3D Z 鍧愭爣, TRANSIENT_LOCAL latched)
        _latch_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(Path, TOPICS.global_path, self._on_global_path, _latch_qos)

        # 鈹€鈹€ 20 Hz 涓诲惊鐜?鈹€鈹€
        self.create_timer(DT, self._tick)

        self.get_logger().info(
            f'[sim] 浠跨湡鑺傜偣鍚姩. '
            f'璧风偣=({self.x:.2f},{self.y:.2f}) '
            f'鐩爣=({self.gx:.2f},{self.gy:.2f}) '
            f'棰勭儹={WARMUP_S:.1f}s 瓒呮椂={MAX_SECS:.0f}s')

    # 鈹€鈹€ TF 鍙戝竷 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _publish_robot_description(self):
        """灏?simple_car.urdf 鍙戝竷鍒?/robot_description (latched)銆?""
        if os.path.exists(_URDF_PATH):
            with open(_URDF_PATH) as f:
                content = f.read()
        else:
            self.get_logger().warn(f'URDF not found: {_URDF_PATH}')
            content = ''
        msg = String()
        msg.data = content
        self.pub_desc.publish(msg)
        self.get_logger().info(f'[sim] /robot_description published ({len(content)} chars)')

    def _publish_building_cloud(self):
        """鍙戝竷鐪熷疄寤虹瓚 3D 鐐逛簯鍒?/nav/building_cloud (latched)銆?
        棣栨璋冪敤浠?PCD 鏂囦欢鍔犺浇骞剁紦瀛? 鍚庣画鐩存帴澶嶇敤缂撳瓨銆?""
        if not hasattr(self, '_bldg_pts_cache'):
            pcd_path = _find_pcd()
            if pcd_path is None:
                self.get_logger().warn('[sim] building2_9.pcd not found 鈥?3D cloud skipped')
                self._bldg_pts_cache = None
                return
            pts = _load_pcd_binary(pcd_path, max_pts=80000)
            if pts is None or len(pts) == 0:
                self.get_logger().warn(f'[sim] PCD load failed: {pcd_path}')
                self._bldg_pts_cache = None
                return
            self._bldg_pts_cache = pts
            self.get_logger().info(f'[sim] PCD cached: {len(pts)} pts from {pcd_path}')

        if self._bldg_pts_cache is None:
            return

        # 浣跨敤褰撳墠鏃堕挓鏃堕棿鎴?鈥?RViz2 message filter 闇€瑕佽兘鍦?TF cache 涓煡鍒拌鏃跺埢
        msg = _make_xyzi_cloud(self._bldg_pts_cache, frame_id=FRAMES.map,
                               stamp=self.get_clock().now().to_msg())
        self.pub_bldg.publish(msg)
        self._bldg_last_pub = time.time()
        self.get_logger().info(f'[sim] Building cloud republished: {len(self._bldg_pts_cache)} pts')

    def _pub_static_tf(self):
        """鍙戝竷闈欐€?TF:  map鈫抩dom  +  body鈫抌ase_link (URDF 鏍瑰潗鏍囩郴)銆?""
        tfs = []

        # map 鈫?odom (浠跨湡涓棤 SLAM 婕傜Щ, 鎭掔瓑鍙樻崲)
        t1 = TransformStamped()
        t1.header.stamp    = self.get_clock().now().to_msg()
        t1.header.frame_id = FRAMES.map
        t1.child_frame_id  = FRAMES.odom
        t1.transform.rotation.w = 1.0
        tfs.append(t1)

        # body 鈫?base_link (鎭掔瓑鍙樻崲, 璁?robot_state_publisher 鐨?URDF TF 閾炬帴鍒板鑸潗鏍囩郴)
        t2 = TransformStamped()
        t2.header.stamp    = t1.header.stamp
        t2.header.frame_id = FRAMES.body
        t2.child_frame_id  = FRAMES.model_base
        t2.transform.rotation.w = 1.0
        tfs.append(t2)

        self.static_br.sendTransform(tfs)

    def _pub_odom_tf(self, now):
        """鍙戝竷 odom 鈫?body TF + /slam/odometry銆?""
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = FRAMES.odom
        tf.child_frame_id  = FRAMES.body
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation.z    = math.sin(self.yaw / 2)
        tf.transform.rotation.w    = math.cos(self.yaw / 2)
        self.tf_br.sendTransform(tf)

        od = Odometry()
        od.header.stamp    = now
        od.header.frame_id = FRAMES.odom
        od.child_frame_id  = FRAMES.body
        od.pose.pose.position.x    = self.x
        od.pose.pose.position.y    = self.y
        od.pose.pose.position.z    = self.z
        od.pose.pose.orientation.z = math.sin(self.yaw / 2)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2)
        od.twist.twist.linear.x    = self.vx
        od.twist.twist.linear.y    = self.vy
        od.twist.twist.angular.z   = self.wz
        self.pub_odom.publish(od)

        # 鏈哄櫒浜轰綅缃ぇ鐞?(Z=0.5m 鎮┖, 鍦ㄥ缓绛戠偣浜戜笂鏂瑰彲瑙?
        self._pub_robot_marker(now)

    def _pub_robot_marker(self, now):
        """鍙戝竷榛勮壊澶х悆 Marker 鏍囪鏈哄櫒浜哄疄鏃朵綅缃?(寤虹瓚鐐逛簯涓婃柟娓呮櫚鍙)銆?""
        m = Marker()
        m.header.frame_id = FRAMES.odom
        m.header.stamp    = now
        m.ns              = 'robot'
        m.id              = 0
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = max(0.3, self.z + 0.6)   # 璺熼殢鏈哄櫒浜虹湡瀹瀂楂樺害
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.7   # 鐩村緞 0.7m, 杩滃涔熸竻鏅板彲瑙?
        m.color.r = 1.0; m.color.g = 0.9; m.color.b = 0.0; m.color.a = 1.0  # 浜粍鑹?
        self.pub_marker.publish(m)

        # 鏈濆悜绠ご (钃濊壊, 浠庣悆蹇冨欢浼?1m)
        arr = Marker()
        arr.header.frame_id = FRAMES.odom
        arr.header.stamp    = now
        arr.ns              = 'robot_dir'
        arr.id              = 1
        arr.type            = Marker.ARROW
        arr.action          = Marker.ADD
        arr.pose.position.x = self.x
        arr.pose.position.y = self.y
        arr.pose.position.z = max(0.3, self.z + 0.6)
        arr.pose.orientation.z = math.sin(self.yaw / 2)
        arr.pose.orientation.w = math.cos(self.yaw / 2)
        arr.scale.x = 1.2   # 绠ご闀垮害
        arr.scale.y = 0.15  # 绠ご瀹藉害
        arr.scale.z = 0.15
        arr.color.r = 0.0; arr.color.g = 0.5; arr.color.b = 1.0; arr.color.a = 1.0  # 钃濊壊
        self.pub_marker.publish(arr)

    # 鈹€鈹€ 鍥炶皟 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_cmd(self, msg: TwistStamped):
        if self._mode_3d:
            return   # 3D妯″紡: 鐢?_follow_3d() 鐩存帴鎺у埗閫熷害, 蹇界暐 pathFollower cmd_vel
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.wz = msg.twist.angular.z
        self.cmd_count += 1

    def _on_adapter(self, msg: String):
        self.get_logger().info(f'[adapter] {msg.data}')

    def _on_planner(self, msg: String):
        status = msg.data.strip()
        self.get_logger().info(f'[planner] {status}')
        if status == 'GOAL_REACHED' and self.phase == 'running':
            self.goal_reached = True
            self.get_logger().info('*** GOAL_REACHED (from pathFollower) ***')
            self.phase = 'done'

    def _on_goal_pose(self, msg: PoseStamped):
        """RViz2 '2D Nav Goal' 鎴?ros2 topic pub 鈫?/nav/goal_pose.
        娉ㄦ剰: sim_robot_node 鑷韩涔熷彂甯冩璇濋, 闇€杩囨护鑷韩娑堟伅."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        # 3D 妯″紡涓嬪拷鐣? _activate_3d_nav 宸茬洿鎺ュ彂甯冨甫 Z 鐨勭洰鏍囩粰 PCT, 涓嶉渶瑕?_set_new_goal
        if self._mode_3d:
            return
        # 蹇界暐鑷韩閲嶅彂 (鍧愭爣涓庡綋鍓嶇洰鏍囧嚑涔庝竴鑷?
        if math.hypot(gx - self.gx, gy - self.gy) < 0.3:
            return
        self._set_new_goal(gx, gy, source='goal_pose')

    def _on_clicked_point(self, msg: PointStamped):
        """RViz2 'Publish Point' 宸ュ叿 鈫?/clicked_point (鐪熷疄 3D XYZ).
        鍦?RViz2 涓夌淮瑙嗗浘閲岀偣鍑诲缓绛戠偣浜戜换鎰忎綅缃?鈫?瑙﹀彂鐪熷疄 3D A* 璺緞瑙勫垝銆?""
        gx = msg.point.x
        gy = msg.point.y
        gz = msg.point.z
        self.get_logger().info(
            f'[3D] 鏀跺埌鐐瑰嚮鐩爣: ({gx:.2f}, {gy:.2f}, {gz:.2f}m)')
        self._activate_3d_nav(gx, gy, gz)

    def _activate_3d_nav(self, gx: float, gy: float, gz: float):
        """婵€娲?PCT 3D A* 瀵艰埅妯″紡: 灏嗙洰鏍囷紙鍚?Z锛夊彂閫佺粰 pct_planner_astar銆?
        瑙勫垝鍣ㄦ娴?goal.pose.position.z > 0.1 鈫?璋冪敤 _plan_3d() 鈫?鍙戝竷鍚湡瀹?Z 鐨勫叏灞€璺緞銆?
        sim_robot 閫氳繃 /nav/global_path 璁㈤槄鎺ユ敹璺緞, 鍐嶇敱 _follow_3d() 璺熻釜銆?""
        if not (MAP_X_MIN <= gx <= MAP_X_MAX and MAP_Y_MIN <= gy <= MAP_Y_MAX):
            self.get_logger().warn(
                f'[3D] 鐩爣 ({gx:.2f},{gy:.2f}) 瓒呭嚭鍦板浘鑼冨洿 鈥?宸插拷鐣?)
            return

        self._user_goal_set = True
        self.goal_reached   = False
        self._gz        = gz
        self._mode_3d   = (gz > 0.1)   # Z>0.1m 鈫?3D璺熻釜; 鍚﹀垯鍥為€€ 2D
        self._3d_path   = []
        self._3d_waiting = True   # 绛夊緟 PCT 瑙勫垝鍣ㄥ搷搴? 閬垮厤 tick 璇垽 "璺緞涓虹┖=鍒拌揪"
        if self.t_start is None:
            self.t_start = time.time()
        self.phase = 'running'

        # 鍙戦€佸甫 Z 鐨勭洰鏍囩粰 PCT 瑙勫垝鍣?鈫?瑙﹀彂 _plan_3d()
        now = self.get_clock().now().to_msg()
        msg_goal = PoseStamped()
        msg_goal.header.stamp    = now
        msg_goal.header.frame_id = 'map'
        msg_goal.pose.position.x = gx
        msg_goal.pose.position.y = gy
        msg_goal.pose.position.z = gz    # PCT 瑙勫垝鍣ㄦ娴?Z>0.1 鈫?鎵ц 3D A*
        msg_goal.pose.orientation.w = 1.0
        self.pub_goal.publish(msg_goal)
        self.get_logger().info(
            f'[3D] 鐩爣宸插彂閫佺粰 PCT 瑙勫垝鍣? ({gx:.2f},{gy:.2f},{gz:.2f}m)')

        # 绾㈣壊鐩爣鐞?Marker (id=2)
        gm = Marker()
        gm.header.frame_id = 'map'; gm.header.stamp = now
        gm.ns   = 'goal_3d'; gm.id = 2
        gm.type = Marker.SPHERE; gm.action = Marker.ADD
        gm.pose.position.x = gx; gm.pose.position.y = gy
        gm.pose.position.z = gz + 0.3
        gm.pose.orientation.w = 1.0
        gm.scale.x = gm.scale.y = gm.scale.z = 0.6
        gm.color.r = 1.0; gm.color.g = 0.2; gm.color.b = 0.2; gm.color.a = 0.9
        self.pub_marker.publish(gm)

    def _on_global_path(self, msg: Path):
        """鎺ユ敹 PCT 瑙勫垝鍣ㄥ彂甯冪殑鍏ㄥ眬璺緞 (鍚湡瀹?Z).
        鑻ヨ矾寰勫瓨鍦?Z 鍙樺寲 (>0.2m) 鈫?鍒ゅ畾涓?3D 璺緞, 濉厖 _3d_path 渚?_follow_3d() 璺熻釜銆?""
        if not msg.poses:
            return
        z_vals = [ps.pose.position.z for ps in msg.poses]
        z_range = max(z_vals) - min(z_vals)
        n = len(msg.poses)
        if self._mode_3d and z_range > 0.2:
            self._3d_path = [
                [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                for ps in msg.poses
            ]
            self._3d_waiting = False   # 璺緞宸叉敹鍒? tick 鍙甯稿垽鏂埌杈?
            self.get_logger().info(
                f'[3D] 鏀跺埌 PCT 3D 璺緞: {n}鐐? Z鑼冨洿={z_range:.2f}m  '
                f'鈫?_follow_3d 婵€娲?)

    def _follow_3d(self):
        """3D 璺緞璺熻釜鎺у埗鍣ㄣ€?
        XY: 姣斾緥杞悜 + 閫熷害鎺у埗锛堟姂鍒跺ぇ鍋忚埅鏃跺墠杩涢€熷害锛?
        Z:  姣斾緥鎺у埗鍨傜洿閫熷害"""
        if not self._3d_path:
            self.vx = self.vy = self.wz = self.vz = 0.0
            self._mode_3d = False
            return

        wx, wy, wz = self._3d_path[0][0], self._3d_path[0][1], self._3d_path[0][2]
        dx = wx - self.x; dy = wy - self.y; dz = wz - self.z
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dist_xy * dist_xy + dz * dz)

        # 鍒拌揪褰撳墠璺偣
        if dist_3d < 0.4:
            self._3d_path.pop(0)
            return

        # XY 鎺у埗
        target_yaw = math.atan2(dy, dx)
        yaw_err = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.wz = max(-1.5, min(1.5, 3.0 * yaw_err))
        # 澶у亸鑸椂鍑忛€燂紝瀵瑰噯鏂瑰悜鍚庡叏閫?
        speed_factor = max(0.0, 1.0 - abs(yaw_err) / 1.2)
        self.vx = min(0.8, dist_xy) * speed_factor
        self.vy = 0.0

        # Z 鎺у埗锛堝瀭鐩撮€熷害锛?
        self.vz = max(-0.5, min(0.5, 2.0 * dz))

    def _set_new_goal(self, gx: float, gy: float, source: str = '?'):
        """楠岃瘉骞惰缃柊鐩爣, 绔嬪嵆瑙﹀彂 PCT 閲嶈鍒?"""
        # 鍦板浘鑼冨洿鏍￠獙
        if not (MAP_X_MIN <= gx <= MAP_X_MAX and MAP_Y_MIN <= gy <= MAP_Y_MAX):
            self.get_logger().warn(
                f'[goal] 鐩爣 ({gx:.2f},{gy:.2f}) 瓒呭嚭鍦板浘鑼冨洿 '
                f'X鈭圼{MAP_X_MIN},{MAP_X_MAX}] Y鈭圼{MAP_Y_MIN},{MAP_Y_MAX}] 鈥?宸插拷鐣?)
            return
        old_gx, old_gy = self.gx, self.gy
        self.gx = gx
        self.gy = gy
        self._user_goal_set = True   # 鐢ㄦ埛鎵嬪姩璁剧疆杩囩洰鏍? loop reset 涓嶈鐩?
        self.get_logger().info(
            f'[goal] 鏂扮洰鏍?[{source}]: ({gx:.2f},{gy:.2f})  '
            f'(鍘?{old_gx:.2f},{old_gy:.2f})')

        # 鏃犺褰撳墠澶勪簬鍝釜闃舵, 绔嬪嵆杩涘叆 running 骞惰Е鍙戦噸瑙勫垝
        self.goal_reached = False
        self.vx = self.vy = self.wz = 0.0    # 娓呴€熷害闃叉儻鎬ф紓绉?
        if self.t_start is None:
            self.t_start = time.time()
        self.phase = 'running'
        now = self.get_clock().now().to_msg()
        self._pub_goal(now)
        self.get_logger().info(f'[goal] 宸插彂閫佹柊鐩爣缁?PCT 瑙勫垝鍣? 閲嶆柊瑙勫垝涓?..')

    # 鈹€鈹€ 杩愬姩瀛︾Н鍒?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _integrate(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x   += (c * self.vx - s * self.vy) * DT
        self.y   += (s * self.vx + c * self.vy) * DT
        self.yaw += self.wz * DT
        self.yaw  = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.z   += self.vz * DT          # Z 杞寸Н鍒?(3D 妯″紡)

        # 宸€熼┍鍔ㄨ溅杞搴︾Н鍒?
        v_left  = self.vx - self.wz * WHEEL_BASE / 2.0
        v_right = self.vx + self.wz * WHEEL_BASE / 2.0
        self._left_wheel_angle  += v_left  / WHEEL_RADIUS * DT
        self._right_wheel_angle += v_right / WHEEL_RADIUS * DT

        # 鍙戝竷 JointState (robot_state_publisher 闇€瑕?
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self._left_wheel_angle, self._right_wheel_angle]
        js.velocity = [v_left / WHEEL_RADIUS, v_right / WHEEL_RADIUS]
        self.pub_joints.publish(js)

    # 鈹€鈹€ 涓诲惊鐜?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _tick(self):
        now = self.get_clock().now().to_msg()

        # 鈹€鈹€ 寤虹瓚鐐逛簯: 姣?3s 閲嶅彂 (鎵€鏈夐樁娈靛潎閫傜敤, 纭繚 RViz 闅忔椂杩炴帴閮借兘鐪嬪埌) 鈹€鈹€
        if time.time() - self._bldg_last_pub >= 3.0:
            self._publish_building_cloud()

        # 鈹€鈹€ pausing 闃舵: 鎸佺画鍙戝竷 TF + 鍦板舰 (涓?sleep, 閬垮厤闃诲 executor) 鈹€鈹€
        if self.phase == 'pausing':
            self._pub_odom_tf(now)
            terrain = _flat_cloud(self.x, self.y, self.z, stamp=now)
            self.pub_cloud.publish(terrain)
            self.pub_terrain.publish(terrain)
            self.pub_te.publish(terrain)
            stop_msg = Int8(); stop_msg.data = 0
            self.pub_stop.publish(stop_msg)
            if time.time() - self._pause_start >= LOOP_PAUSE_S:
                self._reset_for_loop()
            return

        if self.phase == 'done':
            self._finish()
            return

        if self.t_start is None:
            self.t_start = time.time()
        elapsed = time.time() - self.t_start

        # 鈹€鈹€ 濮嬬粓鍙戝竷: 閲岀▼璁?+ TF + 鍚堟垚骞冲潶鍦板舰 + stop=0 鈹€鈹€
        self._pub_odom_tf(now)

        terrain = _flat_cloud(self.x, self.y, self.z, stamp=now)
        self.pub_cloud.publish(terrain)
        self.pub_terrain.publish(terrain)
        self.pub_te.publish(terrain)

        stop_msg = Int8()
        stop_msg.data = 0
        self.pub_stop.publish(stop_msg)

        # 鈹€鈹€ 棰勭儹闃舵 鈹€鈹€
        if self.phase == 'warmup':
            if elapsed >= WARMUP_S:
                self.phase = 'running'
                if _DEF_GOAL_Z > 0.1:
                    # 3D 璺ㄦゼ灞? 瑙﹀彂 3D A* 瑙勫垝
                    self._activate_3d_nav(self.gx, self.gy, _DEF_GOAL_Z)
                    self.get_logger().info(
                        f'== RUNNING [3D]: 鐩爣=({self.gx:.2f},{self.gy:.2f},Z={_DEF_GOAL_Z:.1f}m) ==')
                else:
                    self._pub_goal(now)
                    self.get_logger().info(
                        f'== RUNNING: 鐩爣宸插彂閫? PCT 瑙勫垝鍣ㄥ紑濮嬭鍒?==')

        # 鈹€鈹€ 杩愯闃舵 鈹€鈹€
        elif self.phase == 'running':
            # 鈹€鈹€ 3D 妯″紡: 鐢?PCT 3D A* 鎻愪緵璺緞, _follow_3d 璺熻釜, 缁曡繃 pathFollower 鈹€鈹€
            if self._mode_3d:
                # 绛夊緟 PCT 瑙勫垝鍣ㄥ搷搴?(_3d_waiting=True 鏃朵笉鍒ゆ柇鍒拌揪)
                if self._3d_waiting:
                    self._integrate()
                    return
                self._follow_3d()
                self._integrate()
                # 浠ョ害 2 Hz 璁板綍 3D 杞ㄨ抗
                if int(elapsed * 2) > int((elapsed - DT) * 2):
                    dist_xy = math.hypot(self.x - self.gx, self.y - self.gy)
                    self.get_logger().info(
                        f'[3D] t={elapsed:6.1f}s  '
                        f'pos=({self.x:.2f},{self.y:.2f},Z={self.z:.2f})  '
                        f'dist_xy={dist_xy:.2f}m  wps={len(self._3d_path)}')
                    self.traj.append({
                        't':    round(elapsed, 2),
                        'x':    round(self.x, 3),
                        'y':    round(self.y, 3),
                        'z':    round(self.z, 3),
                        'yaw':  round(math.degrees(self.yaw), 1),
                        'vx':   round(self.vx, 3),
                        'vy':   round(self.vy, 3),
                        'wz':   round(self.wz, 3),
                        'dist': round(dist_xy, 3),
                    })
                if not self._3d_path:
                    self.get_logger().info(
                        f'[3D] *** 3D鐩爣鍒拌揪! 缁堢偣=({self.x:.2f},{self.y:.2f},{self.z:.2f}) ***')
                    self.goal_reached = True
                    self.phase = 'done'
                return
            # 姣?GOAL_RESEND_S 绉掗噸鏂板彂閫佺洰鏍?(闃茶鍒掑櫒閲嶅惎鍚庝涪澶?
            if elapsed > WARMUP_S:
                slot_now  = int((elapsed - WARMUP_S) / GOAL_RESEND_S)
                slot_prev = int((elapsed - WARMUP_S - DT) / GOAL_RESEND_S)
                if slot_now > slot_prev:
                    self._pub_goal(now)

            self._integrate()

            # 浠ョ害 2 Hz 璁板綍杞ㄨ抗鍜屾棩蹇?
            if int(elapsed * 2) > int((elapsed - DT) * 2):
                dist = math.hypot(self.x - self.gx, self.y - self.gy)
                self.get_logger().info(
                    f't={elapsed:6.1f}s  '
                    f'pos=({self.x:.3f},{self.y:.3f})  '
                    f'yaw={math.degrees(self.yaw):6.1f}掳  '
                    f'v=({self.vx:.2f},{self.vy:.2f})  蠅={self.wz:.3f}  '
                    f'dist={dist:.2f}m  cmds={self.cmd_count}')
                self.traj.append({
                    't':    round(elapsed, 2),
                    'x':    round(self.x, 3),
                    'y':    round(self.y, 3),
                    'yaw':  round(math.degrees(self.yaw), 1),
                    'vx':   round(self.vx, 3),
                    'vy':   round(self.vy, 3),
                    'wz':   round(self.wz, 3),
                    'dist': round(dist, 3),
                })

                if dist < GOAL_THRESH:
                    self.goal_reached = True
                    self.get_logger().info(
                        f'*** GOAL REACHED (璺濈)  t={elapsed:.1f}s  dist={dist:.3f}m ***')
                    self.phase = 'done'
                    return

            if elapsed > MAX_SECS:
                dist = math.hypot(self.x - self.gx, self.y - self.gy)
                self.get_logger().warn(
                    f'TIMEOUT {elapsed:.0f}s  final_dist={dist:.2f}m')
                self.phase = 'done'
                return

        if self.phase == 'done':
            self._finish()

    def _pub_goal(self, now):
        msg = PoseStamped()
        msg.header.stamp    = now
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.gx
        msg.pose.position.y = self.gy
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f'[goal] 鐩爣宸插彂甯? ({self.gx:.2f}, {self.gy:.2f})')

    # 鈹€鈹€ 淇濆瓨缁撴灉 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _finish(self):
        """淇濆瓨缁撴灉鍚庡垏鎹㈠埌 pausing 闃舵 (闈為樆濉? 涓?sleep)銆?""
        self.phase = 'pausing'
        self._pause_start = time.time()
        result = {
            'goal_reached':  self.goal_reached,
            'start':         [_DEF_START_X, _DEF_START_Y],
            'goal':          [self.gx, self.gy],
            'final_pos':     [round(self.x, 3), round(self.y, 3)],
            'final_yaw_deg': round(math.degrees(self.yaw), 1),
            'cmd_count':     self.cmd_count,
            'trajectory':    self.traj,
        }
        json_path = '/tmp/sim_result.json'
        with open(json_path, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'缁撴灉宸蹭繚瀛? {json_path}  ({LOOP_PAUSE_S:.0f}s 鍚庨噸缃?..)')
        self._plot(result)

    def _reset_for_loop(self):
        """閲嶇疆鏈哄櫒浜轰綅缃埌璧风偣锛屼繚鐣欑敤鎴锋墜鍔ㄨ缃殑鐩爣銆?""
        self.x   = _DEF_START_X
        self.y   = _DEF_START_Y
        self.yaw = _DEF_START_YAW
        self.vx = self.vy = self.wz = 0.0
        self.z = 0.0    # 閲嶇疆 Z 楂樺害鍒板湴闈?(3D 妯″紡缁撴潫鍚庡綊闆?
        self.vz = 0.0
        self._mode_3d    = False
        self._3d_path    = []
        self._3d_waiting = False
        self.cmd_count = 0
        self.traj = []
        self.goal_reached = False
        self.t_start = None
        self._left_wheel_angle  = 0.0
        self._right_wheel_angle = 0.0
        # 鍙湁鐢ㄦ埛娌℃湁鎵嬪姩璁剧疆杩囩洰鏍囨椂鎵嶅洖褰掗粯璁ょ洰鏍?
        if not self._user_goal_set:
            self.gx = _DEF_GOAL_X
            self.gy = _DEF_GOAL_Y
        self.phase = 'warmup'
        self.get_logger().info(
            f'[sim] == 寰幆閲嶇疆: 璧风偣=({self.x},{self.y}) 鐩爣=({self.gx:.2f},{self.gy:.2f}) ==')
        # 閲嶆柊鍙戝竷寤虹瓚鐐逛簯, 纭繚鍒氳繛鎺ョ殑 RViz 瀹炰緥涔熻兘鐪嬪埌
        self._publish_building_cloud()

    def _plot(self, result):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            traj = result['trajectory']
            if not traj:
                self.get_logger().warn('杞ㄨ抗涓虹┖, 璺宠繃缁樺浘')
                return

            ts  = [p['t']    for p in traj]
            xs  = [p['x']    for p in traj]
            ys  = [p['y']    for p in traj]
            ds  = [p['dist'] for p in traj]
            vxs = [p['vx']   for p in traj]
            wzs = [p['wz']   for p in traj]

            fig, axes = plt.subplots(1, 2, figsize=(14, 6))

            # 闈㈡澘 1: XY 杞ㄨ抗
            ax = axes[0]
            ax.plot(xs, ys, 'b-', lw=2, label='鏈哄櫒浜鸿建杩?)
            ax.plot(xs[0],  ys[0],  'go',  ms=12, label=f'璧风偣 ({xs[0]:.1f},{ys[0]:.1f})')
            ax.plot(self.gx, self.gy, 'r*', ms=18, label=f'鐩爣 ({self.gx:.1f},{self.gy:.1f})')
            ax.plot(xs[-1], ys[-1], 'bs', ms=10,
                    label=f'缁堢偣 ({xs[-1]:.2f},{ys[-1]:.2f})')
            status = '鉁?鍒拌揪鐩爣' if result['goal_reached'] else f'鉁?瓒呮椂 (dist={ds[-1]:.2f}m)'
            _scene = os.environ.get('SIM_SCENE_NAME', 'Building2_9')
            ax.set_title(f'{_scene} SITL 鈥?{status}', fontsize=11)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
            ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

            # 闈㈡澘 2: 鏃堕棿搴忓垪鎸囨爣
            ax2 = axes[1]
            ax2.plot(ts, ds,  'r-',  lw=2,   label='鍒扮洰鏍囪窛绂?(m)')
            ax2.plot(ts, vxs, 'b-',  lw=1.5, label='vx (m/s)')
            ax2.plot(ts, wzs, 'g--', lw=1.2, label='蠅z (rad/s)')
            ax2.axhline(GOAL_THRESH, color='r', ls=':', alpha=0.5,
                        label=f'鍒拌揪闃堝€?{GOAL_THRESH}m')
            ax2.set_xlabel('鏃堕棿 (s)'); ax2.set_title('瀵艰埅鎸囨爣')
            ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            png_path = '/tmp/sim_traj.png'
            plt.savefig(png_path, dpi=130, bbox_inches='tight')
            self.get_logger().info(f'杞ㄨ抗鍥惧凡淇濆瓨: {png_path}')
        except Exception as e:
            self.get_logger().error(f'缁樺浘澶辫触: {e}')


def main():
    rclpy.init()
    node = SimRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[sim] 鐢ㄦ埛涓柇')
    finally:
        if node.phase not in ('done', 'saved') and node.traj:
            node._finish()
        node.destroy_node()
        rclpy.shutdown()
    print('[sim] 瀹屾垚.')


if __name__ == '__main__':
    main()
