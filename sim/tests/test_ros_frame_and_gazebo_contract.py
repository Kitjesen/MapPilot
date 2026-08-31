from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("rclpy", reason="Needs ROS2 runtime (Gazebo)")
pytestmark = [pytest.mark.sim]


from runtime.runtime_interface import (
    ADAPTER_TOPIC_ALIASES,
    ALGORITHM_INTERFACES,
    ARTIFACT_FORMATS,
    CORE_ALGORITHM_ENTRY_TOPICS,
    CORE_REQUIRED_TOPICS,
    DATA_SOURCE_CONTRACTS,
    FRAMES,
    LIDAR_EXTRINSICS,
    MESSAGE_FORMATS,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
    REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS,
    RUNTIME_DATA_FLOW,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    TOPIC_ALLOWED_FRAME_IDS,
    TOPIC_FORMATS,
    TOPICS,
    adapter_remappings,
    expand_frame_id_aliases,
    frame_id_aliases,
    product_data_source,
    resolved_runtime_data_flow,
    runtime_contract_manifest,
    runtime_data_flow_topics,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_id,
    topic_default_frame_id,
    topic_formats,
    transform_xyz,
)
from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig
from sim.engine.bridge.gazebo_runtime_adapter import Pose3, _odom_xyz_to_body, _transform_xyz

REPO_ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="ignore")


def test_ros_frame_contract_documents_body_base_link_alias():
    doc = _read("docs/architecture/ros_frame_contract.md")

    assert "map -> odom -> body" in doc
    assert "base_link == body" in doc
    assert "`world`" in doc
    assert "/slam/map_cloud" in doc
    assert "never body-relative points" in doc
    assert "`/slam/odometry`" in doc
    assert "`map` or `odom`" in doc
    assert "odom` is allowed only when an adapter transforms it" not in doc


def _markdown_frame_list(frames: tuple[str, ...]) -> str:
    return ", ".join(f"`{frame}`" for frame in frames)


def test_ros_frame_contract_topic_frame_table_mirrors_runtime_contract():
    doc = _read("docs/architecture/ros_frame_contract.md")
    general_allowed = runtime_topic_allowed_frame_ids(None)
    real_allowed = runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)
    required = set(REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS)

    for topic in real_allowed:
        row = (
            f"| `{topic}` | "
            f"`{runtime_topic_default_frame_id(REAL_RUNTIME_CONTRACT, topic)}` | "
            f"{_markdown_frame_list(general_allowed[topic])} | "
            f"{'yes' if topic in required else 'no'} | "
            f"{_markdown_frame_list(real_allowed[topic])} |"
        )
        assert row in doc

    assert ("`/slam/map_cloud` and `/nav/global_path` are deliberately stricter on real") in doc
    assert "real S100P evidence must reject `/slam/map_cloud` outside `map`" in doc


def test_simulation_contract_documents_switch_dataflow_and_frame_boundary():
    doc = _read("docs/architecture/SIMULATION_INTEGRATION_CONTRACT.md")

    assert "`endpoint`" in doc
    assert "RobotConfig selector" in doc
    assert "runtime-contract --json" in doc
    assert "runtime-spec explore --adapter mujoco_live" in doc
    assert "/api/v1/navigation/status" in doc
    assert "`runtime.blockers`" in doc
    assert "`launcher` and `launcher_args`" in doc
    assert "`frame_links`" in doc
    assert "`resolved_runtime_data_flow`" in doc
    assert "mujoco_live -> real_s100p" in doc
    assert "sensor/log/simulator source" in doc
    assert "`runtime_data_flow`" in doc
    assert "`resolved_runtime_data_flow.<data_source>`" in doc
    assert "`mujoco_fastlio2_live` resolves to `/points_raw + /imu_raw" in doc
    assert "`slam_or_relayed_localization_map`" in doc
    assert "`command_boundary`" in doc
    assert "map -> odom -> body -> lidar_link" in doc
    assert "`odom->body` must be observed from live odometry" in doc


def test_runtime_interface_is_single_source_for_frames_topics_formats_and_algorithms():
    from runtime.config import LidarConfig

    assert (
        runtime_data_flow_topics.__doc__ == "Return unique canonical runtime stream tokens in one resolved data-flow."
    )
    assert FRAMES.body_alias_note == "base_link == body"
    assert FRAMES.axis_convention == "x_forward_y_left_z_up"
    assert FRAMES.lidar_frame == "lidar_link"
    assert FRAMES.real_lidar == "livox_frame"
    assert TOPICS.raw_lidar_points == "/points_raw"
    assert TOPICS.raw_imu == "/imu_raw"
    assert TOPICS.registered_cloud == "/slam/registered_cloud"
    assert TOPICS.map_cloud == "/slam/map_cloud"
    assert MESSAGE_FORMATS["raw_timed_pointcloud2"].required_fields == (
        "x",
        "y",
        "z",
        "intensity",
        "time",
        "ring",
    )
    assert MESSAGE_FORMATS["registered_cloud"].frame_role == FRAMES.body
    assert TOPIC_FORMATS[TOPICS.registered_cloud] == ("registered_cloud",)
    assert TOPIC_FORMATS[TOPICS.map_cloud] == ("map_cloud",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.odometry] == (FRAMES.odom, FRAMES.map)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.map_cloud] == (FRAMES.map, FRAMES.odom)
    assert frame_id_aliases(FRAMES.body) == (FRAMES.body, FRAMES.model_base)
    assert frame_id_aliases(FRAMES.lidar) == (FRAMES.lidar, FRAMES.real_lidar)
    assert expand_frame_id_aliases((FRAMES.body, FRAMES.map)) == (
        FRAMES.body,
        FRAMES.model_base,
        FRAMES.map,
    )
    assert REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS[TOPICS.odometry] == (
        FRAMES.odom,
        FRAMES.map,
    )
    assert REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS[TOPICS.map_cloud] == (FRAMES.map,)
    assert REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS[TOPICS.global_path] == (FRAMES.map,)
    assert set(TOPIC_FORMATS[TOPICS.raw_lidar_points]) == {
        "raw_livox_custom",
        "raw_timed_pointcloud2",
    }
    assert topic_formats(TOPICS.cmd_vel) == ("cmd_vel",)
    assert TOPICS.lidar_scan in ALGORITHM_INTERFACES["fastlio_mapping"].inputs
    assert TOPICS.raw_lidar_points in ALGORITHM_INTERFACES["fastlio_raw_validation"].inputs
    assert TOPICS.lidar_scan in DATA_SOURCE_CONTRACTS["field"].normalized_outputs
    assert TOPICS.exploration_way_point in ALGORITHM_INTERFACES["exploration_strategy"].outputs
    assert DATA_SOURCE_CONTRACTS["field"].lidar_extrinsic_profile is None
    assert DATA_SOURCE_CONTRACTS["gazebo_industrial"].lidar_extrinsic_profile == "gazebo_proxy"
    assert RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES["map_layers_and_exploration"] == (
        "exploration_strategy",
    )
    global_stage = next(stage for stage in RUNTIME_DATA_FLOW if stage.name == "global_planning")
    assert "octoplanner3d_uses_headless_octomap_or_point_cloud" in global_stage.map_dependency
    assert ARTIFACT_FORMATS["map_pcd"].path == "map.pcd"
    assert product_data_source("map").data_source == "field"
    real_lidar = LIDAR_EXTRINSICS["go2_mid360"]
    cfg_lidar = LidarConfig()
    assert cfg_lidar.frame_id == real_lidar.child
    assert cfg_lidar.offset_x == pytest.approx(real_lidar.x)
    assert cfg_lidar.offset_y == pytest.approx(real_lidar.y)
    assert cfg_lidar.offset_z == pytest.approx(real_lidar.z)


def test_runtime_contract_manifest_exports_topics_formats_algorithms_sources_and_aliases():
    manifest = runtime_contract_manifest()

    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert manifest["frames"]["body"] == FRAMES.body
    assert manifest["topics"]["registered_cloud"] == TOPICS.registered_cloud
    assert tuple(manifest["core_required_topics"]) == CORE_REQUIRED_TOPICS
    assert manifest["frame_links"]["map_to_odom"] == {
        "parent": FRAMES.map,
        "child": FRAMES.odom,
        "required": True,
    }
    assert manifest["frame_links"]["odom_to_body"]["child"] == FRAMES.body
    assert manifest["frame_links"]["body_to_lidar"]["child"] == FRAMES.lidar
    assert manifest["frame_links"]["body_to_camera"] == {
        "parent": FRAMES.body,
        "child": FRAMES.camera,
        "required": True,
    }
    flow = manifest["runtime_data_flow"]
    assert [stage["name"] for stage in flow] == [
        "endpoint_adapter",
        "slam_or_relayed_localization_map",
        "map_layers_and_exploration",
        "global_planning",
        "local_planning_and_following",
        "dynamic_obstacle_gate",
        "command_boundary",
    ]
    assert flow[0]["outputs"] == ("source:data_source.normalized_outputs",)
    assert flow[-1]["inputs"] == (TOPICS.cmd_vel,)
    assert flow[-1]["outputs"] == ("sink:data_source.command_sink",)
    resolved_flow = manifest["resolved_runtime_data_flow"]["mujoco_fastlio2_live"]
    assert resolved_flow[0]["inputs"] == (TOPICS.raw_lidar_points, TOPICS.raw_imu)
    assert resolved_flow[0]["outputs"] == (TOPICS.raw_lidar_points, TOPICS.raw_imu)
    assert resolved_flow[1]["inputs"] == (TOPICS.raw_lidar_points, TOPICS.raw_imu)
    assert resolved_flow[-1]["outputs"] == ("mujoco_velocity_adapter",)
    assert manifest["message_formats"]["registered_cloud"]["frame_role"] == FRAMES.body
    assert manifest["topic_formats"][TOPICS.registered_cloud] == ("registered_cloud",)
    assert manifest["topic_allowed_frame_ids"][TOPICS.map_cloud] == [
        FRAMES.map,
        FRAMES.odom,
    ]
    assert manifest["real_runtime_topic_allowed_frame_ids"][TOPICS.map_cloud] == [
        FRAMES.map,
    ]
    assert tuple(manifest["real_runtime_required_topic_frame_ids"]) == (REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS)
    assert tuple(manifest["runtime_data_flow_topics"]["real_s100p"]) == (runtime_data_flow_topics("real_s100p"))
    assert TOPICS.map_cloud in manifest["algorithm_interfaces"]["exploration_strategy"]["inputs"]
    assert TOPICS.lidar_scan in manifest["data_sources"]["real_s100p"]["normalized_outputs"]
    fastlio_aliases = {item["source"]: item["target"] for item in manifest["adapter_aliases"]["fastlio2"]}
    assert fastlio_aliases["/cloud_registered"] == TOPICS.registered_cloud
    assert fastlio_aliases["/cloud_map"] == TOPICS.map_cloud
    assert fastlio_aliases["/Odometry"] == TOPICS.odometry
    assert "cmu_unity" not in manifest["adapter_relays"]
    assert "cmu_unity_external" not in manifest["data_sources"]


def test_runtime_stream_contract_language_is_not_ros2_topic_only():
    sim_contract = _read("docs/architecture/SIMULATION_INTEGRATION_CONTRACT.md")

    assert "canonical runtime stream tokens" in sim_contract
    assert "not a ROS2 topic browser" in sim_contract
    assert "native streams or topics" in sim_contract


def test_runtime_contract_references_declared_topics_formats_and_artifacts():
    manifest = runtime_contract_manifest()
    declared_topics = {
        value for value in manifest["topics"].values() if isinstance(value, str) and value.startswith("/")
    }
    declared_artifacts = set(manifest["artifact_formats"])
    declared_formats = set(manifest["message_formats"])
    topic_format_map = {topic: tuple(formats) for topic, formats in manifest["topic_formats"].items()}

    def _format_is_declared(format_name: str) -> bool:
        return format_name in declared_formats or format_name == "service" or "/msg/" in format_name

    for name, interface in manifest["algorithm_interfaces"].items():
        for topic in tuple(interface["inputs"]) + tuple(interface["outputs"]):
            if topic.startswith("artifact:"):
                artifact_name = topic.split(":", 1)[1]
                assert artifact_name in declared_artifacts, name
            elif topic.startswith("module:"):
                module_name, _, port_name = topic.split(":", 1)[1].partition(".")
                assert module_name and port_name, name
            else:
                assert topic in declared_topics, name
                assert topic in topic_format_map, name
                assert all(_format_is_declared(fmt) for fmt in topic_format_map[topic]), name

    for name, source in manifest["data_sources"].items():
        for topic in (
            tuple(source["normalized_outputs"])
            + tuple(source["algorithm_entry_outputs"])
            + tuple(source["algorithm_context_outputs"])
        ):
            if topic.startswith("/nav/") or topic.startswith("/exploration/"):
                assert topic in declared_topics, name
            assert topic in topic_format_map, name
        for topic in tuple(source["source_outputs"]):
            assert topic in topic_format_map, name
        assert source["command_sink"], name
        if str(source["command_sink"]).startswith("/"):
            assert source["command_sink"] in topic_format_map, name
        assert source["algorithm_entry_outputs"], name
        profile = source["lidar_extrinsic_profile"]
        if profile is not None:
            assert profile in manifest["lidar_extrinsics"], name

    for name, aliases in manifest["adapter_aliases"].items():
        for alias in aliases:
            assert alias["target"] in declared_topics, name
            msg_format = alias["msg_format"]
            assert _format_is_declared(msg_format), name
            if str(alias["source"]).startswith("/"):
                assert alias["source"] in topic_format_map, name

    for name, aliases in manifest["adapter_relays"].items():
        for alias in aliases:
            for endpoint in (alias["source"], alias["target"]):
                if endpoint.startswith("/nav/"):
                    assert endpoint in declared_topics, name
                if endpoint.startswith("/"):
                    assert endpoint in topic_format_map, name


def test_all_product_data_sources_reach_same_navigation_algorithm_entry_topics():
    real_runtime_entry_topics = CORE_ALGORITHM_ENTRY_TOPICS + (
        TOPICS.localization_health,
        TOPICS.localization_quality,
    )

    for name, source in DATA_SOURCE_CONTRACTS.items():
        if name == REAL_RUNTIME_CONTRACT:
            assert source.algorithm_entry_outputs == real_runtime_entry_topics, name
        else:
            assert source.algorithm_entry_outputs == CORE_ALGORITHM_ENTRY_TOPICS, name

    real = DATA_SOURCE_CONTRACTS[REAL_RUNTIME_CONTRACT]
    gazebo = DATA_SOURCE_CONTRACTS["gazebo_industrial"]
    mujoco_fastlio = DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"]

    assert set(real.source_outputs) == {TOPICS.lidar_scan, TOPICS.imu}
    assert set(mujoco_fastlio.source_outputs) == {TOPICS.raw_lidar_points, TOPICS.raw_imu}
    assert TOPICS.exploration_grid in gazebo.algorithm_context_outputs
    assert "cmu_unity_external" not in DATA_SOURCE_CONTRACTS


def test_resolved_runtime_data_flow_expands_endpoint_boundaries_without_placeholders():
    for name in DATA_SOURCE_CONTRACTS:
        stages = resolved_runtime_data_flow(name)
        assert [stage.name for stage in stages] == [stage["name"] for stage in manifest_stages()]
        tokens = [token for stage in stages for token in (*stage.inputs, *stage.outputs)]
        assert not any(token.startswith("source:data_source.") for token in tokens), name
        assert not any(token.startswith("sink:data_source.") for token in tokens), name

    real = {stage.name: stage for stage in resolved_runtime_data_flow("real_s100p")}
    assert real["endpoint_adapter"].inputs == (TOPICS.lidar_scan, TOPICS.imu)
    assert real["endpoint_adapter"].outputs == (TOPICS.lidar_scan, TOPICS.imu)
    assert real["slam_or_relayed_localization_map"].inputs == (
        TOPICS.lidar_scan,
        TOPICS.imu,
    )
    assert real["command_boundary"].outputs == ("driver",)

    mujoco_live = {stage.name: stage for stage in resolved_runtime_data_flow("mujoco_fastlio2_live")}
    assert mujoco_live["endpoint_adapter"].inputs == (
        TOPICS.raw_lidar_points,
        TOPICS.raw_imu,
    )
    assert mujoco_live["slam_or_relayed_localization_map"].outputs == (
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
    )
    assert mujoco_live["command_boundary"].outputs == ("mujoco_velocity_adapter",)

    gazebo = {stage.name: stage for stage in resolved_runtime_data_flow("gazebo_industrial")}
    assert gazebo["endpoint_adapter"].inputs == (
        "/model/thunder/odometry",
        "/lingtu/gazebo/raw/lidar_points",
        "/lingtu/gazebo/raw/lidar_scan",
    )
    assert TOPICS.exploration_grid in gazebo["endpoint_adapter"].outputs
    assert gazebo["command_boundary"].outputs == ("/lingtu/gazebo/cmd_vel",)


def manifest_stages():
    return runtime_contract_manifest()["runtime_data_flow"]


def test_lidar_extrinsic_preserves_body_axis_direction_contract():
    extrinsic = LIDAR_EXTRINSICS["gazebo_proxy"]

    origin = transform_xyz((0.0, 0.0, 0.0), extrinsic)
    forward = transform_xyz((1.0, 0.0, 0.0), extrinsic)
    left = transform_xyz((0.0, 1.0, 0.0), extrinsic)
    up = transform_xyz((0.0, 0.0, 1.0), extrinsic)

    assert forward[0] > origin[0]
    assert left[1] > origin[1]
    assert up[2] > origin[2]


def test_lidar_module_uses_runtime_frame_contract():
    module_source = _read("src/drivers/real/lidar/lidar_module.py")
    driver_source = _read("src/drivers/real/lidar/lidar.py")

    assert "from runtime.runtime_interface import TOPICS, real_lidar_frame_id" in module_source
    assert "LIDAR_RAW_FRAME_ID = real_lidar_frame_id()" in module_source
    assert "scan_topic: str = TOPICS.lidar_scan" in module_source
    assert "imu_topic: str = TOPICS.imu" in module_source
    assert "frame_id=LIDAR_RAW_FRAME_ID" in module_source
    assert "from runtime.runtime_interface import TOPICS" in driver_source
    assert "scan_topic: str = TOPICS.lidar_scan" in driver_source
    assert "imu_topic: str = TOPICS.imu" in driver_source


def test_native_motion_driver_uses_the_body_frame_contract():
    source = _read("src/drivers/real/motion/core.cpp")

    assert not (REPO_ROOT / "src/drivers/real/thunder/han_dog_module.py").exists()
    assert not (REPO_ROOT / "src/drivers/real/thunder/connection.py").exists()
    assert 'return frame == "body" || frame == "base_link";' in source
    assert "ActionReason::InvalidFrame" in source


def test_adapter_aliases_use_runtime_contract_topics():
    assert not (REPO_ROOT / "src/localization/native_factories.py").exists()
    assert adapter_remappings("fastlio2")["/cloud_registered"] == TOPICS.registered_cloud
    assert adapter_remappings("fastlio2")["/cloud_map"] == TOPICS.map_cloud
    assert adapter_remappings("fastlio2")["/Odometry"] == TOPICS.odometry
    assert adapter_remappings("localizer")["map_cloud"] == TOPICS.saved_map_cloud
    assert ADAPTER_TOPIC_ALIASES["tare"][0].target == TOPICS.map_cloud


def test_slam_runtime_defaults_keep_genz_frame_contract_available():
    assert topic_default_frame_id(TOPICS.odometry) == "odom"
    assert topic_default_frame_id(TOPICS.cmd_vel) == "body"


def test_src_mujoco_bridges_use_runtime_contract_frames():
    sensor_bridge = _read("src/drivers/sim/mujoco/sensors.py")
    stack = _read("src/drivers/sim/mujoco/stack.py")
    driver = _read("src/drivers/sim/mujoco/driver.py")

    assert "from runtime.runtime_interface import FRAME_LINKS, TOPICS, topic_default_frame_id" in sensor_bridge
    assert "MUJOCO_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)" in sensor_bridge
    assert 'MUJOCO_BODY_FRAME_ID = FRAME_LINKS["odom_to_body"].child' in sensor_bridge
    assert "msg.header.frame_id = MUJOCO_ODOM_FRAME_ID" in sensor_bridge
    assert "msg.child_frame_id = MUJOCO_BODY_FRAME_ID" in sensor_bridge
    assert 'parent=FRAME_LINKS["odom_to_body"].parent' in sensor_bridge
    assert 'child=FRAME_LINKS["odom_to_body"].child' in sensor_bridge
    assert "FRAMES.odom" not in sensor_bridge
    assert "FRAMES.body" not in sensor_bridge

    assert "from runtime.runtime_interface import TOPICS, topic_default_frame_id" in stack
    assert "MUJOCO_LIVE_PLANNING_FRAME_ID = topic_default_frame_id(TOPICS.odometry)" in stack
    assert "planning_frame_id=MUJOCO_LIVE_PLANNING_FRAME_ID" in stack
    assert "occupancy_frame_id=MUJOCO_LIVE_OCCUPANCY_FRAME_ID" in stack
    assert "goal_frame_id=MUJOCO_LIVE_GOAL_FRAME_ID" in stack
    assert "FRAMES.odom" not in stack

    assert "MUJOCO_MODULE_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)" in driver
    assert "MUJOCO_MODULE_BODY_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)" in driver
    assert "MUJOCO_MODULE_MAP_CLOUD_FRAME_ID = MUJOCO_MODULE_ODOM_FRAME_ID" in driver
    assert "frame_id=MUJOCO_MODULE_BODY_FRAME_ID" in driver
    assert "frame_id=MUJOCO_MODULE_MAP_CLOUD_FRAME_ID" in driver
    assert "frame_id=MUJOCO_MODULE_CAMERA_FRAME_ID" in driver
    assert "FRAMES.odom" not in driver
    assert "FRAMES.body" not in driver


def test_gazebo_bridge_config_exposes_lingtu_runtime_topics():
    cfg = GazeboBridgeConfig(world_name="test_world", robot_name="thunder")

    assert cfg.frames.body_alias_note == "base_link == body"
    assert cfg.required_lingtu_topics() == {
        "cmd_vel": "/nav/cmd_vel",
        "gazebo_cmd_vel_ros_input": "/lingtu/gazebo/cmd_vel",
        "odometry": "/slam/odometry",
        "map_cloud": "/slam/map_cloud",
        "terrain_map": "/nav/terrain_map",
        "terrain_map_ext": "/nav/terrain_map_ext",
        "cumulative_map_cloud": "/slam/cumulative_map_cloud",
        "registered_cloud": "/slam/registered_cloud",
        "color_image": "/camera/color/image_raw",
        "depth_image": "/camera/depth/image_raw",
        "camera_info": "/camera/color/camera_info",
    }
    assert cfg.raw_ros_topics() == {
        "odometry": "/lingtu/gazebo/raw/odometry",
        "lidar_points": "/lingtu/gazebo/raw/lidar_points",
        "lidar_scan": "/lingtu/gazebo/raw/lidar_scan",
        "color_image": "/lingtu/gazebo/raw/color_image",
        "depth_image": "/lingtu/gazebo/raw/depth_image",
        "camera_info": "/lingtu/gazebo/raw/camera_info",
    }
    assert cfg.gazebo_odometry == "/model/thunder/odometry"
    assert "test_world" in cfg.gazebo_lidar_points
    assert "test_world" in cfg.gazebo_lidar_scan
    assert "lidar_link" in cfg.gazebo_lidar_points
    assert "lidar_link" in cfg.gazebo_lidar_scan
    assert any("/lingtu/gazebo/cmd_vel" in spec for spec in cfg.ros_gz_bridge_specs())
    assert any("LaserScan" in spec for spec in cfg.ros_gz_bridge_specs())
    assert any("CameraInfo" in spec for spec in cfg.ros_gz_bridge_specs())
    remaps = " ".join(cfg.ros_remap_args())
    assert "/lingtu/gazebo/raw/lidar_points" in remaps
    assert "/lingtu/gazebo/raw/camera_info" in remaps


def test_gazebo_launch_is_optional_but_ros_native():
    launch = _read("launch/gazebo_simulation.launch.py")
    normalized_launch = launch.replace("\r\n", "\n")

    assert "lingtu_gazebo_demo_room.sdf" in launch
    assert '"worlds",\n    "gazebo",' in normalized_launch
    assert "lingtu_gazebo_empty.sdf" not in launch
    assert "ros_gz_sim" in launch
    assert "ros_gz_bridge" in launch
    assert "parameter_bridge" in launch
    assert "robot_model" in launch
    assert "spawn_robot" in launch
    assert "thunder_gazebo_proxy.sdf" in launch
    assert '"ros_gz_sim"' in launch
    assert '"create"' in launch
    assert "lingtu_gazebo_spawn_robot" in launch
    assert "sys.path.insert" in launch
    assert 'os.path.join(repo_root, "src")' in launch
    assert "headless" in launch
    assert "-r -s" in launch
    assert 'else f"-r {world}"' in launch
    assert "sim.engine.bridge.gazebo_cmd_vel_adapter" in launch
    assert "sim.engine.bridge.gazebo_runtime_adapter" in launch
    assert "additional_env=adapter_env" in launch
    assert "cwd=repo_root" in launch
    assert "world -> map -> odom -> body" in launch
    assert "base_link aliased to body" in launch
    assert "period=0.0" in launch
    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch


def test_gazebo_industrial_park_scene_is_first_class_product_scene():
    world_path = REPO_ROOT / "sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf"
    tree = ET.parse(world_path)
    root = tree.getroot()
    text = world_path.read_text(encoding="utf-8")
    world = root.find("world")
    assert world is not None
    assert world.attrib.get("name") == "lingtu_gazebo_industrial_park"

    model_names = {model.attrib.get("name") for model in root.findall(".//model")}
    assert {
        "industrial_ground_plane",
        "main_asphalt_road",
        "cross_asphalt_road",
        "perimeter_fence_north",
        "perimeter_fence_south",
        "perimeter_fence_east",
        "perimeter_gate_west_left",
        "perimeter_gate_west_right",
        "warehouse_north_hall",
        "factory_south_hall",
        "loading_dock_wall",
        "container_stack_blue",
        "container_stack_yellow",
        "pipe_rack_left",
        "pipe_rack_right",
        "pallet_cluster",
        "forklift_silhouette",
        "tank_silo_a",
        "tank_silo_b",
    } <= model_names
    assert '<world name="lingtu_gazebo_industrial_park">' in text
    assert 'filename="gz-sim-sensors-system"' in text
    assert "<size>36 22</size>" in text
    assert "<size>28 5.0 0.03</size>" in text
    assert "<pose>0 0" not in text


def test_gazebo_proxy_model_matches_bridge_frame_and_topic_contract():
    model_path = REPO_ROOT / "sim/assets/sdf/thunder_gazebo_proxy.sdf"
    tree = ET.parse(model_path)
    root = tree.getroot()
    text = model_path.read_text(encoding="utf-8")
    link_names = {elem.attrib.get("name") for elem in root.findall(".//link")}

    assert {"base_link", "lidar_link", "camera_link"} <= link_names
    assert "front_marker_visual" in text
    assert 'name="gz::sim::systems::DiffDrive"' in text
    assert 'type="gpu_lidar"' in text
    assert "<samples>16</samples>" in text
    assert "<max>8.0</max>" in text
    assert text.count("<xyz>0 1 0</xyz>") >= 4
    assert "<topic>/lingtu/gazebo/cmd_vel</topic>" in text
    assert "<odom_topic>/model/thunder/odometry</odom_topic>" in text
    assert "<frame_id>odom</frame_id>" in text
    assert "<child_frame_id>body</child_frame_id>" in text
    assert "<damping>1.0</damping>" in text
    assert "<friction>0.4</friction>" in text
    assert "<mu>1.4</mu>" in text

    lidar_pose = root.find(".//link[@name='lidar_link']/pose")
    assert lidar_pose is not None
    xyz_rpy = [float(value) for value in lidar_pose.text.split()]
    gazebo_lidar = LIDAR_EXTRINSICS["gazebo_proxy"]
    assert xyz_rpy == pytest.approx(
        [
            gazebo_lidar.x,
            gazebo_lidar.y,
            gazebo_lidar.z,
            gazebo_lidar.roll,
            gazebo_lidar.pitch,
            gazebo_lidar.yaw,
        ]
    )

    assert "lingtu_gazebo_empty" not in text

    cfg = GazeboBridgeConfig(world_name="lingtu_gazebo_demo_room", robot_name="thunder")
    assert cfg.gazebo_lidar_scan not in text
    assert cfg.gazebo_color_image not in text
    assert cfg.gazebo_depth_image not in text
    assert "/world/lingtu_gazebo_demo_room" not in text
    assert "sensor/lidar/scan</topic>" not in text
    assert "sensor/camera/image</topic>" not in text


def test_gazebo_cmd_vel_adapter_preserves_lingtu_stamped_command_boundary():
    adapter = _read("sim/engine/bridge/gazebo_cmd_vel_adapter.py")

    assert '"/nav/cmd_vel"' in adapter
    assert '"/lingtu/gazebo/cmd_vel"' in adapter
    assert "TwistStamped" in adapter
    assert "Twist" in adapter
    assert "hold_publish_hz" in adapter
    assert "command_timeout_sec" in adapter
    assert "create_timer" in adapter
    assert "zero-hold braking" in adapter


def test_gazebo_runtime_adapter_normalizes_frames_and_avoids_control_publication():
    adapter = _read("sim/engine/bridge/gazebo_runtime_adapter.py")

    assert '"/nav/odometry"' not in adapter
    assert "lingtu_odometry" in adapter
    assert "lingtu_map_cloud" in adapter
    assert "lingtu_terrain_map" in adapter
    assert "lingtu_terrain_map_ext" in adapter
    assert "lingtu_cumulative_map_cloud" in adapter
    assert "lingtu_registered_cloud" in adapter
    assert "out.header.frame_id = FRAMES.odom" in adapter
    assert "out.child_frame_id = FRAMES.body" in adapter
    assert "map_to_odom.header.frame_id = FRAMES.map" in adapter
    assert "map_to_odom.child_frame_id = FRAMES.odom" in adapter
    assert "body_to_lidar.header.frame_id = FRAMES.body" in adapter
    assert "body_to_lidar.child_frame_id = self._cfg.frames.lidar_frame" in adapter
    assert "body_to_camera.header.frame_id = FRAMES.body" in adapter
    assert "body_to_camera.child_frame_id = self._cfg.frames.camera_frame" in adapter
    assert "tf.header.frame_id = FRAMES.odom" in adapter
    assert "tf.child_frame_id = FRAMES.body" in adapter
    assert "body_to_lidar_x" in adapter
    assert "body_to_lidar_roll" in adapter
    assert "body_to_lidar_pitch" in adapter
    assert "body_to_lidar_yaw" in adapter
    assert "legacy alias fallback" in adapter
    assert "rpy_to_quaternion_xyzw" in adapter
    assert "point_cloud2.read_points" in adapter
    assert "LaserScan" in adapter
    assert "create_cloud_xyz32" in adapter
    assert "_on_scan" in adapter
    assert "_body_cloud_to_odom" in adapter
    assert "prefer_point_cloud_over_scan" in adapter
    assert "cloud_min_range" in adapter
    assert "cloud_max_range" in adapter
    assert "cloud_min_body_z" in adapter
    assert "cloud_max_body_z" in adapter
    assert "cloud_self_filter_forward" in adapter
    assert "cloud_self_filter_lateral" in adapter
    assert "terrain_height_reference_z" in adapter
    assert "terrain_min_height" in adapter
    assert "terrain_max_height" in adapter
    assert "terrain_max_body_range" in adapter
    assert "terrain_map_ext_max_body_range" in adapter
    assert "terrain_self_filter_forward" in adapter
    assert "_odom_xyz_to_body" in adapter
    assert "_is_self_filter_point" in adapter
    assert "_is_body_self_filter_point" in adapter
    assert "_body_cloud_point_allowed" in adapter
    assert "_terrain_cloud_from_xyz" in adapter
    assert "_create_xyzi_cloud" in adapter
    assert 'PointField(name="intensity"' in adapter
    assert "_matched_stamp_msg" in adapter
    assert "_last_point_cloud_wall_time" in adapter
    assert "_pose_for_stamp(_stamp_sec(msg.header.stamp))" in adapter
    assert "_pose_history" in adapter
    assert "_cumulative_voxels" in adapter
    assert "cumulative_voxel_size" in adapter
    assert "cumulative_min_z" in adapter
    assert "cumulative_max_z" in adapter
    assert 'declare_parameter("cumulative_min_z", 0.08)' in adapter
    assert 'declare_parameter("cumulative_max_range", 6.0)' in adapter
    assert "px - pose.x" in adapter
    assert "_publish_cumulative_map(map_cloud)" in adapter
    assert "max_body_range=self._terrain_max_body_range" in adapter
    assert "max_body_range=self._terrain_map_ext_max_body_range" in adapter
    assert "height = max(0.0, pz - self._terrain_height_reference_z)" in adapter
    assert "points.append((px, py, pz, height))" in adapter
    assert "_point_count(registered_cloud) <= 0" in adapter
    assert "_point_count(map_cloud) <= 0" in adapter
    assert "does not do geometric point transforms" not in adapter
    assert "create_subscription" in adapter
    assert '"/nav/cmd_vel"' not in adapter
    assert "create_publisher(Twist" not in adapter


def test_gazebo_runtime_adapter_point_transform_math():
    out = _transform_xyz(
        (1.0, 0.0, 0.0),
        translation=(0.28, 0.0, 0.20),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    assert out == (1.28, 0.0, 0.20)

    qz_90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    rotated = _transform_xyz(
        (1.0, 0.0, 0.0),
        translation=(2.0, 3.0, 0.0),
        rotation_xyzw=qz_90,
    )
    assert rotated[0] == pytest.approx(2.0, abs=1e-6)
    assert rotated[1] == pytest.approx(4.0, abs=1e-6)
    assert rotated[2] == pytest.approx(0.0, abs=1e-6)

    pose = Pose3(
        x=2.0,
        y=3.0,
        z=0.5,
        qx=0.0,
        qy=0.0,
        qz=math.sin(math.pi / 4.0),
        qw=math.cos(math.pi / 4.0),
    )
    body = _odom_xyz_to_body((2.0, 4.0, 0.7), pose)
    assert body[0] == pytest.approx(1.0, abs=1e-6)
    assert body[1] == pytest.approx(0.0, abs=1e-6)
    assert body[2] == pytest.approx(0.2, abs=1e-6)


def test_tf_contract_smoke_is_read_only_and_checks_runtime_chain():
    smoke = _read("sim/scripts/tf_contract_smoke.py")

    assert "map->odom->body" in smoke
    assert '"/slam/odometry"' in smoke
    assert '"/slam/map_cloud"' in smoke
    assert '"/slam/registered_cloud"' in smoke
    assert '"/camera/color/image_raw"' in smoke
    assert '"lingtu.gazebo_runtime_smoke.v1"' in smoke
    assert "--require-sensors" in smoke
    assert "--require-camera" in smoke
    assert "--json-out" in smoke
    assert "required_observations_ready" in smoke
    assert 'lookup_transform("map", "odom"' in smoke
    assert 'lookup_transform("odom", "body"' in smoke
    assert 'lookup_transform("body", "lidar_link"' in smoke
    assert 'lookup_transform("body", "camera_link"' in smoke
    assert "create_publisher" not in smoke
    assert '"/nav/cmd_vel"' not in smoke
    assert '"/nav/goal_pose"' not in smoke
