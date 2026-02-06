Mapping（建图）	PGO	回环检测 + 位姿图优化，校正 odom 漂移，建立全局一致地图
Navigation（导航）	Localizer	加载已保存地图，用 ICP 把当前点云配准到地图，得到在 map 下的位姿


""这是一个流程化：
Localizer: 加载 .pcd 地图，订阅 /cloud_registered + /Odometry→ ICP 配准到地图 → 发布 TF: map → odom
         
fastlio2: 继续发布 odom → body（实时里程计）
TF 树: map (Localizer) → odom → body (fastlio2)


建图阶段（mapping_launch）
    建图结束后调用保存服务
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/path/to/your_map.pcd'}"
这里的 /path/to/your_map.pcd 就是“保存地址”。


跑 PCT 全局规划前要预先做什么？
1. 准备输入点云
2. 跑 tomography 生成 tomogram
3. 再跑全局规划

跑 PCT 全局规划前，必须预先用 tomography 把 rsc/pcd/ 下的点云处理成 rsc/tomogram/ 下的 .pickle；没有这份 .pickle 就不能做 PCT 规划
PCT 只认 tomography 产出的 .pickle

第一次会从 my_map.pcd 建 tomogram 并生成 my_map.pickle；之后会自动用 my_map.pickle。

用"点击点"发到 "/clicked_point"      "geometry_msgs/PoseStamped"        位置 (x,y,z) + 朝向
用“设置目标”发到 "/goal_pose"         "geometry_msgs/PointStamped"       3D 点 (x,y,z)

"""
    def goal_pose_callback(self, msg):
        self._handle_goal(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, "goal_pose")
    
    def goal_point_callback(self, msg):
        self._handle_goal(msg.point.x, msg.point.y, msg.point.z, "clicked_point")


/cloud_map 是 fastlio2 实时构建的地图： 实时生成：每一帧 LiDAR 扫描都会更新 坐标系：odom（world_frame）
 
// fastlio2/src/lio_node.cpp
m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 10000);

// 每帧处理时发布（当前帧 + 历史累积）
publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, ...);


/stop 话题：
0	正常控制	默认状态
1	停止速度，保留转向	需要转向避让
2	完全停止	紧急制动