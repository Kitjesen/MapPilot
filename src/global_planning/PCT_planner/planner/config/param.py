class ConfigNode():
    """ROS 节点层：话题、帧、地图名、发布开关等。默认在此，运行时可配置。"""
    map_file = 'spiral0.3_2'
    # 必须与加载的 PCD/地图所在坐标系一致（如 PCD 在 odom 下则设 map_frame:=odom）
    map_frame = 'map'
    robot_frame = 'body'
    min_plan_interval = 1.0
    default_goal_height = 0.0
    publish_map_pointcloud = True
    publish_tomogram = True


class ConfigPlanner():
    """轨迹规划与优化。默认在此，运行时可配置。"""
    use_quintic = True
    max_heading_rate = 10
    obstacle_thr = 50


class ConfigWrapper():
    """地图与 tomogram 路径与建图。默认在此，运行时可配置。"""
    tomo_dir = '/rsc/tomogram/'
    pcd_dir = None
    tomogram_resolution = 0.2
    tomogram_slice_dh = 0.2
    tomogram_ground_h = 0.0


class Config():
    node = ConfigNode()
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()
