from dataclasses import asdict, replace

import pytest

pytestmark = [pytest.mark.sim]

from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_PLANNING_FRAME_ID,
    RUNTIME_ODOM_FRAME_ID,
)
from runtime.profiles.endpoints import (
    RUNTIME_ENDPOINTS,
    RuntimeEndpointError,
    resolve_runtime_run_spec,
)
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    FRAME_LINKS,
    FRAMES,
    PROFILE_DATA_SOURCE_BINDINGS,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    TOPICS,
    resolved_runtime_data_flow,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_ids,
)
from runtime.runtime_profiles import PROFILES
from runtime.runtime_switch import (
    compare_runtime_switch,
    runtime_spec_summary,
    validate_runtime_switch,
)


@pytest.mark.sim
def test_sim_to_real_switch_changes_command_sink_and_simulation_only():
    sim_cfg = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    real_cfg = resolve_profile_config("explore")
    sim_spec = resolve_runtime_run_spec("explore", sim_cfg)
    real_spec = resolve_runtime_run_spec("explore", real_cfg)

    diff = compare_runtime_switch(sim_spec, real_spec)

    assert diff["from"]["data_source"] == "mujoco_fastlio2_live"
    assert diff["to"]["data_source"] == "thunder_field"
    assert diff["from"]["runtime_contract"] == "mujoco_fastlio2_live"
    assert diff["to"]["runtime_contract"] == "thunder_field"
    assert diff["from"]["module_transport"] == "local"
    assert diff["to"]["module_transport"] == "local"
    assert diff["from"]["endpoint_transport"] == "local"
    assert diff["to"]["endpoint_transport"] == "dds"
    assert diff["from"]["slam_source"] == "lingtu_fastlio2"
    assert diff["to"]["slam_source"] == "lingtu_fastlio_or_external_robot_slam"
    assert diff["from"]["mapping_source"] == "fastlio2_map_cloud"
    assert diff["to"]["mapping_source"] == "slam_map_cloud"
    assert diff["from"]["lidar_extrinsic_profile"] == "mujoco_thunder_v3"
    assert diff["to"]["lidar_extrinsic_profile"] == "real_mid360"
    assert diff["from"]["command_sink"] == "mujoco_velocity_adapter"
    assert diff["to"]["command_sink"] == "driver"
    assert diff["from"]["frame_links"]["map_to_odom"] == {
        "parent": "map",
        "child": "odom",
        "required": True,
    }
    assert diff["from"]["topic_allowed_frame_ids"][TOPICS.map_cloud] == [
        "map",
        "odom",
    ]
    assert diff["to"]["topic_allowed_frame_ids"][TOPICS.map_cloud] == ["map"]
    assert diff["to"]["topic_allowed_frame_ids"][TOPICS.global_path] == ["map"]
    assert diff["from"]["required_topic_frame_ids"] == []
    assert diff["to"]["required_topic_frame_ids"] == [
        TOPICS.lidar_scan,
        TOPICS.imu,
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
    ]
    assert TOPICS.raw_lidar_points in diff["from"]["runtime_data_flow_topics"]
    assert TOPICS.lidar_scan in diff["to"]["runtime_data_flow_topics"]
    assert TOPICS.cmd_vel in diff["to"]["runtime_data_flow_topics"]
    assert diff["from"]["resolved_runtime_data_flow"][0]["inputs"] == [
        TOPICS.raw_lidar_points,
        TOPICS.raw_imu,
    ]
    assert diff["from"]["resolved_runtime_data_flow"][-1]["outputs"] == [
        "mujoco_velocity_adapter",
    ]
    assert diff["to"]["resolved_runtime_data_flow"][0]["inputs"] == [
        TOPICS.lidar_scan,
        TOPICS.imu,
    ]
    assert diff["to"]["resolved_runtime_data_flow"][-1]["outputs"] == [
        "driver",
    ]
    assert diff["from"]["simulation_only"] is True
    assert diff["to"]["simulation_only"] is False
    assert "command_sink" in diff["changed"]
    assert "endpoint_transport" in diff["changed"]
    assert "resolved_runtime_data_flow" in diff["changed"]
    assert "simulation_only" in diff["changed"]
    assert "topic_allowed_frame_ids" in diff["changed"]
    assert diff["to"]["launcher_args"] == []


def test_compatibility_profile_switch_summary_names_runtime_endpoint():
    sim_cfg = resolve_profile_config("sim_mujoco_live")
    real_cfg = resolve_profile_config("explore")
    sim_spec = resolve_runtime_run_spec("sim_mujoco_live", sim_cfg)
    real_spec = resolve_runtime_run_spec("explore", real_cfg)

    diff = compare_runtime_switch(sim_spec, real_spec)

    assert diff["from"]["endpoint"] == "mujoco_live"
    assert diff["from"]["robot_preset"] == "sim_endpoint"
    assert diff["to"]["endpoint"] == "thunder_field"
    assert diff["to"]["robot_preset"] == "thunder"
    assert diff["to"]["launcher_args"] == []


def test_runtime_spec_summary_is_json_native():
    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    spec = resolve_runtime_run_spec("explore", config)

    summary = runtime_spec_summary(spec)

    assert summary["topic_allowed_frame_ids"][TOPICS.map_cloud] == ["map", "odom"]
    assert isinstance(summary["topic_allowed_frame_ids"][TOPICS.map_cloud], list)
    assert summary["frames"]["axis_convention"] == "x_forward_y_left_z_up"
    assert summary["topic_default_frame_ids"][TOPICS.map_cloud] == "map"
    assert summary["module_transport"] == "local"
    assert summary["endpoint_transport"] == "local"
    assert summary["endpoint_contract"] is None
    assert summary["route_contract"] == "sim"
    assert summary["required_topic_frame_ids"] == []
    assert TOPICS.raw_lidar_points in summary["runtime_data_flow_topics"]
    assert isinstance(summary["runtime_data_flow_topics"], list)
    assert summary["resolved_runtime_data_flow"][0]["inputs"] == [
        TOPICS.raw_lidar_points,
        TOPICS.raw_imu,
    ]
    assert isinstance(summary["resolved_runtime_data_flow"][0]["inputs"], list)
    assert summary["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] == [
        "global_planning",
        "octoplanner3d_global_planning",
    ]
    assert summary["launcher_args"] == ["explore"]
    assert isinstance(summary["launcher_args"], list)
    assert summary["validation"]["ok"] is True
    assert summary["validation"]["blockers"] == []
    assert "warnings" in summary["validation"]


def test_runtime_switch_validates_route_contract_env() -> None:
    config = resolve_profile_config("nav")
    spec = resolve_runtime_run_spec("nav", config)

    assert spec.route_contract == "robot"
    assert validate_runtime_switch(spec).ok is True

    broken = replace(spec, env={**dict(spec.env), "LINGTU_ROUTE_CONTRACT": "sim"})

    validation = validate_runtime_switch(broken)

    assert validation.ok is False
    assert "env route contract does not match run spec" in validation.blockers


@pytest.mark.sim
def test_runtime_summary_reports_mujoco_explore_product_semantic_overrides():
    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    spec = resolve_runtime_run_spec("explore", config)

    summary = runtime_spec_summary(spec)
    overrides = {item["field"]: item for item in summary["product_semantic_overrides"]}

    assert validate_runtime_switch(spec).ok is True
    assert overrides["llm"]["product_value"] == "qwen"
    assert overrides["llm"]["endpoint_value"] == "mock"
    assert overrides["enable_semantic"]["product_value"] is True
    assert overrides["enable_semantic"]["endpoint_value"] is False
    assert "planner" not in overrides
    assert "data_source" not in overrides
    assert "_runtime_endpoint" not in overrides


def test_runtime_summary_reports_tare_endpoint_frame_override():
    config = resolve_profile_config("tare_explore", runtime_endpoint="mujoco_live")
    spec = resolve_runtime_run_spec("tare_explore", config)

    summary = runtime_spec_summary(spec)
    overrides = {item["field"]: item for item in summary["product_semantic_overrides"]}

    assert validate_runtime_switch(spec).ok is True
    assert "planner" not in overrides
    assert overrides["planning_frame_id"] == {
        "field": "planning_frame_id",
        "override_scope": "compatibility_override",
        "product_value": DEFAULT_PLANNING_FRAME_ID,
        "endpoint_value": RUNTIME_ODOM_FRAME_ID,
    }
    assert "tare_scenario" not in overrides
    assert "gateway_port" not in overrides
    assert "runtime compatibility override: planning_frame_id map -> odom" in (summary["validation"]["warnings"])


@pytest.mark.parametrize(
    (
        "profile",
        "endpoint",
        "data_source",
        "runtime_contract",
        "launcher",
        "default_args",
        "record_args",
    ),
    (
        (
            "sim_mujoco_live",
            "mujoco_live",
            "mujoco_fastlio2_live",
            "mujoco_fastlio2_live",
            "sim/scripts/mujoco/launch_fastlio2_live.sh",
            ("gate",),
            ("video",),
        ),
        (
            "sim_cmu_tare",
            "cmu_unity",
            "cmu_unity_external",
            "cmu_unity_external",
            "sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
            ("gate",),
            ("start", "--gate", "--rviz"),
        ),
    ),
)
def test_compatibility_external_profile_uses_endpoint_action_contract(
    profile,
    endpoint,
    data_source,
    runtime_contract,
    launcher,
    default_args,
    record_args,
):
    config = dict(PROFILES[profile])
    gate_spec = resolve_runtime_run_spec(profile, config)
    record_spec = resolve_runtime_run_spec(profile, config, record=True)

    assert gate_spec.endpoint == endpoint
    assert gate_spec.data_source == data_source
    assert gate_spec.runtime_contract == runtime_contract
    assert gate_spec.launcher == launcher
    assert gate_spec.launcher_args == default_args
    assert gate_spec.env["LINGTU_PROFILE"] == profile
    assert gate_spec.env["LINGTU_ENDPOINT"] == endpoint
    assert gate_spec.env["LINGTU_DATA_SOURCE"] == data_source
    assert gate_spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert gate_spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert gate_spec.env["LINGTU_RUNTIME_CONTRACT"] == runtime_contract
    assert gate_spec.env["LINGTU_SIMULATION_ONLY"] == "1"
    assert validate_runtime_switch(gate_spec).ok is True
    assert record_spec.launcher_args == record_args


@pytest.mark.parametrize(
    ("profile", "data_source"),
    (
        ("sim", "mujoco_module_graph"),
        ("portable_mujoco", "mujoco_module_graph"),
        ("sim_nav", "in_process_stub"),
    ),
)
def test_in_process_sim_profiles_resolve_without_runtime_endpoint(
    profile,
    data_source,
):
    config = resolve_profile_config(profile)
    spec = resolve_runtime_run_spec(profile, config)
    source = DATA_SOURCE_CONTRACTS[data_source]

    assert spec.endpoint is None
    assert spec.data_source == data_source
    assert spec.runtime_contract is None
    assert spec.simulation_only is True
    assert spec.command_sink == source.command_sink
    assert spec.launcher is None
    assert spec.launcher_args == ()
    assert spec.env == {
        "LINGTU_PROFILE": profile,
        "LINGTU_DATA_SOURCE": data_source,
        "LINGTU_MODULE_TRANSPORT": "local",
        "LINGTU_ENDPOINT_TRANSPORT": "local",
        "LINGTU_SIMULATION_ONLY": "1",
        "LINGTU_COMMAND_SINK": source.command_sink,
    }
    assert validate_runtime_switch(spec).ok is True
    if data_source == "mujoco_module_graph":
        topics = {topic for stage in spec.resolved_runtime_data_flow for topic in stage["inputs"] + stage["outputs"]}
        assert TOPICS.height_rays in source.normalized_outputs
        assert TOPICS.height_rays in source.algorithm_context_outputs
        assert TOPICS.height_rays in topics


@pytest.mark.parametrize(
    (
        "profile",
        "endpoint",
        "data_source",
        "runtime_contract",
    ),
    (
        ("sim_mujoco_live", "mujoco_live", "mujoco_fastlio2_live", "mujoco_fastlio2_live"),
        ("sim_gazebo", "gazebo", "gazebo_industrial", "gazebo_industrial"),
        ("sim_industrial", "gazebo", "gazebo_industrial", "gazebo_industrial"),
        ("sim_cmu_tare", "cmu_unity", "cmu_unity_external", "cmu_unity_external"),
    ),
)
def test_first_class_sim_profiles_resolve_endpoint_contract_without_launcher(
    profile,
    endpoint,
    data_source,
    runtime_contract,
):
    config = resolve_profile_config(profile)
    spec = resolve_runtime_run_spec(profile, config)
    source = DATA_SOURCE_CONTRACTS[data_source]

    assert "_external_launcher" not in config
    assert "_runtime_contract" not in config
    assert spec.endpoint == endpoint
    assert spec.data_source == data_source
    assert spec.runtime_contract == runtime_contract
    assert spec.simulation_only is True
    assert spec.command_sink == source.command_sink
    assert spec.launcher is None
    assert spec.launcher_args == ()
    assert spec.env["LINGTU_PROFILE"] == profile
    assert spec.env["LINGTU_ENDPOINT"] == endpoint
    assert spec.env["LINGTU_DATA_SOURCE"] == data_source
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert spec.env["LINGTU_RUNTIME_CONTRACT"] == runtime_contract
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"
    assert validate_runtime_switch(spec).ok is True


def test_external_launcher_without_action_contract_is_rejected():
    config = resolve_profile_config("explore")
    config["_external_launcher"] = "sim/scripts/custom_launcher.sh"

    with pytest.raises(RuntimeEndpointError) as exc_info:
        resolve_runtime_run_spec("explore", config)

    assert "external launcher args missing for profile 'explore'" in str(exc_info.value)


def test_external_launcher_extra_args_are_explicit_override():
    config = resolve_profile_config("explore")
    config["_external_launcher"] = "sim/scripts/custom_launcher.sh"

    spec = resolve_runtime_run_spec("explore", config, extra_args=("custom",))

    assert spec.launcher_args == ("custom",)


def test_switch_guard_rejects_simulation_target_with_hardware_sink():
    sim_cfg = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    sim_spec = resolve_runtime_run_spec("explore", sim_cfg)
    bad_spec = sim_spec.__class__(
        **{
            **sim_spec.__dict__,
            "command_sink": "driver",
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "simulation endpoint uses hardware command sink" in result.blockers


def test_switch_guard_rejects_runtime_endpoint_without_contract():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "runtime_contract": None,
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "runtime endpoint has no runtime contract" in result.blockers


def test_switch_guard_rejects_runtime_contract_data_source_mismatch():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "runtime_contract": "mujoco_fastlio2_live",
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "runtime contract does not match data source" in result.blockers


def test_switch_guard_accepts_normal_sim_and_real_targets():
    sim_cfg = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    real_cfg = resolve_profile_config("explore")
    sim_spec = resolve_runtime_run_spec("explore", sim_cfg)
    real_spec = resolve_runtime_run_spec("explore", real_cfg)

    assert real_spec.runtime_contract == "thunder_field"
    assert real_spec.env["LINGTU_RUNTIME_CONTRACT"] == "thunder_field"
    assert real_spec.module_transport == "local"
    assert real_spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert real_spec.endpoint_transport == "dds"
    assert real_spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "dds"
    assert real_spec.endpoint_contract == "thunder_field_dds_v1"
    assert real_spec.env["LINGTU_ENDPOINT_CONTRACT"] == "thunder_field_dds_v1"
    assert real_spec.localization_adapter == "cpp_slam_status"
    assert real_spec.env["LINGTU_LOCALIZATION_ADAPTER"] == "cpp_slam_status"
    assert validate_runtime_switch(sim_spec).ok is True
    assert validate_runtime_switch(real_spec).ok is True


def test_switch_guard_rejects_endpoint_contract_env_mismatch():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "env": {
                **real_spec.env,
                "LINGTU_ENDPOINT_CONTRACT": "wrong_contract",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "env endpoint contract does not match run spec" in result.blockers


def test_switch_guard_rejects_localization_adapter_env_mismatch():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "env": {
                **real_spec.env,
                "LINGTU_LOCALIZATION_ADAPTER": "ros2_slam_bridge",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "env localization adapter does not match run spec" in result.blockers


def test_switch_guard_rejects_simulation_target_exporting_real_mode_flag():
    sim_cfg = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    sim_spec = resolve_runtime_run_spec("explore", sim_cfg)
    bad_spec = sim_spec.__class__(
        **{
            **sim_spec.__dict__,
            "env": {
                **sim_spec.env,
                "LINGTU_SIMULATION_ONLY": "0",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "simulation endpoint exports real-mode flag" in result.blockers


def test_switch_guard_rejects_real_target_exporting_simulation_mode_flag():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "env": {
                **real_spec.env,
                "LINGTU_SIMULATION_ONLY": "1",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "real endpoint exports simulation-mode flag" in result.blockers


def test_switch_guard_rejects_runtime_env_mismatches():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)

    cases = (
        ("LINGTU_ENDPOINT", "mujoco_live", "env endpoint does not match run spec"),
        ("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live", "env data source does not match run spec"),
        ("LINGTU_MODULE_TRANSPORT", "zmq", "env module transport does not match run spec"),
        ("LINGTU_ENDPOINT_TRANSPORT", "local", "env endpoint transport does not match run spec"),
        ("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live", "env runtime contract does not match run spec"),
        ("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter", "env command sink does not match run spec"),
    )
    for key, value, blocker in cases:
        bad_spec = real_spec.__class__(
            **{
                **real_spec.__dict__,
                "env": {
                    **real_spec.env,
                    key: value,
                },
            }
        )

        result = validate_runtime_switch(bad_spec)

        assert result.ok is False, key
        assert blocker in result.blockers


def test_switch_guard_rejects_unknown_module_transport():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "module_transport": "mqtt",
            "env": {
                **real_spec.env,
                "LINGTU_MODULE_TRANSPORT": "mqtt",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "module transport is not a known transport strategy" in result.blockers


def test_switch_guard_rejects_unknown_endpoint_transport():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "endpoint_transport": "mqtt",
            "env": {
                **real_spec.env,
                "LINGTU_ENDPOINT_TRANSPORT": "mqtt",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "endpoint transport is not a known transport strategy" in result.blockers


def test_switch_guard_rejects_frame_link_contract_mismatch():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "frame_links": {
                **real_spec.frame_links,
                "body_to_lidar": {
                    **real_spec.frame_links["body_to_lidar"],
                    "child": "base_link",
                },
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "frame links do not match runtime contract" in result.blockers


def test_switch_guard_rejects_topic_frame_contract_mismatch():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "topic_allowed_frame_ids": {
                **real_spec.topic_allowed_frame_ids,
                TOPICS.map_cloud: ("map", "odom"),
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "topic frame_id contract does not match runtime contract" in result.blockers


def test_switch_guard_rejects_resolved_flow_not_matching_data_source():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "resolved_runtime_data_flow": tuple(
                stage.__dict__ for stage in resolved_runtime_data_flow("mujoco_fastlio2_live")
            ),
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "resolved runtime data flow does not match data source" in result.blockers


def test_switch_guard_rejects_frame_contract_drift():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "frames": {
                **real_spec.frames,
                "axis_convention": "z_up_unspecified",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "frames do not match runtime contract" in result.blockers


def test_switch_guard_rejects_topic_default_frame_drift():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "topic_default_frame_ids": {
                **real_spec.topic_default_frame_ids,
                TOPICS.map_cloud: "odom",
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "topic default frame_id contract does not match runtime contract" in result.blockers


def test_switch_guard_rejects_stage_algorithm_interface_binding_drift():
    real_cfg = resolve_profile_config("explore")
    real_spec = resolve_runtime_run_spec("explore", real_cfg)
    bad_spec = real_spec.__class__(
        **{
            **real_spec.__dict__,
            "runtime_data_flow_stage_algorithm_interfaces": {
                **real_spec.runtime_data_flow_stage_algorithm_interfaces,
                "global_planning": ("local_planning_and_following",),
            },
        }
    )

    result = validate_runtime_switch(bad_spec)

    assert result.ok is False
    assert "runtime data flow stage algorithm interfaces do not match contract" in result.blockers


def test_all_runtime_endpoint_profiles_resolve_to_matching_contracts():
    for endpoint_name, endpoint in RUNTIME_ENDPOINTS.items():
        assert endpoint.runtime_contract == endpoint.data_source, endpoint_name
        for profile in endpoint.supported_profiles:
            config = resolve_profile_config(profile, runtime_endpoint=endpoint_name)
            spec = resolve_runtime_run_spec(profile, config)

            assert spec.endpoint == endpoint_name
            assert spec.data_source == endpoint.data_source
            assert spec.runtime_contract == endpoint.runtime_contract
            assert spec.robot_preset == endpoint.robot_preset
            assert spec.module_transport == endpoint.module_transport
            assert spec.endpoint_transport == endpoint.endpoint_transport
            assert spec.env["LINGTU_MODULE_TRANSPORT"] == endpoint.module_transport
            assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == endpoint.endpoint_transport
            assert spec.simulation_only is endpoint.simulation_only
            assert validate_runtime_switch(spec).ok is True, (endpoint_name, profile)


def test_all_cli_profiles_resolve_to_declared_runtime_contracts():
    assert set(PROFILES) == set(PROFILE_DATA_SOURCE_BINDINGS)

    expected_frame_links = {name: asdict(link) for name, link in FRAME_LINKS.items()}
    expected_frames = asdict(FRAMES)
    for profile in PROFILES:
        config = resolve_profile_config(profile)
        spec = resolve_runtime_run_spec(profile, config)
        binding = PROFILE_DATA_SOURCE_BINDINGS[profile]
        source = DATA_SOURCE_CONTRACTS[binding.data_source]

        assert spec.profile == profile
        assert spec.data_source == binding.data_source
        expected_endpoint_transport = (
            RUNTIME_ENDPOINTS[spec.endpoint].endpoint_transport if spec.endpoint in RUNTIME_ENDPOINTS else "local"
        )
        assert spec.module_transport == "local"
        assert spec.endpoint_transport == expected_endpoint_transport
        assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
        assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == expected_endpoint_transport
        assert spec.command_sink == source.command_sink
        assert spec.slam_source == source.slam_source
        assert spec.localization_source == source.localization_source
        assert spec.mapping_source == source.mapping_source
        assert spec.lidar_extrinsic_profile == source.lidar_extrinsic_profile
        assert spec.frames == expected_frames
        assert spec.frame_links == expected_frame_links
        assert spec.topic_allowed_frame_ids == runtime_topic_allowed_frame_ids(
            spec.runtime_contract or spec.data_source
        )
        assert spec.topic_default_frame_ids == runtime_topic_default_frame_ids(
            spec.runtime_contract or spec.data_source
        )
        assert spec.resolved_runtime_data_flow == tuple(
            asdict(stage) for stage in resolved_runtime_data_flow(binding.data_source)
        )
        assert spec.runtime_data_flow_stage_algorithm_interfaces == {
            stage: tuple(interfaces) for stage, interfaces in (RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items())
        }
        assert validate_runtime_switch(spec).ok is True, profile


def test_runtime_endpoint_profile_tables_only_reference_supported_profiles():
    for endpoint_name, endpoint in RUNTIME_ENDPOINTS.items():
        supported = set(endpoint.supported_profiles)
        assert supported <= set(PROFILES), endpoint_name
        assert endpoint.data_source in DATA_SOURCE_CONTRACTS, endpoint_name

        for table_name, table in (
            ("profile_overrides", endpoint.profile_overrides),
            ("default_actions", endpoint.default_actions),
            ("record_actions", endpoint.record_actions),
        ):
            assert set(table) <= supported, (endpoint_name, table_name)


def test_runtime_endpoint_config_rejects_missing_default_action_profile():
    endpoint = RUNTIME_ENDPOINTS["mujoco_live"]
    default_actions = dict(endpoint.default_actions)
    default_actions.pop("explore")
    broken_endpoint = replace(endpoint, default_actions=default_actions)

    with pytest.raises(RuntimeEndpointError) as exc_info:
        broken_endpoint.config_for_profile("explore")

    assert "endpoint 'mujoco_live' default_actions missing profile 'explore'" in str(exc_info.value)


def test_runtime_endpoint_config_rejects_missing_record_action_profile():
    endpoint = RUNTIME_ENDPOINTS["mujoco_live"]
    record_actions = dict(endpoint.record_actions)
    record_actions.pop("explore")
    broken_endpoint = replace(endpoint, record_actions=record_actions)

    with pytest.raises(RuntimeEndpointError) as exc_info:
        broken_endpoint.config_for_profile("explore")

    assert "endpoint 'mujoco_live' record_actions missing profile 'explore'" in str(exc_info.value)
