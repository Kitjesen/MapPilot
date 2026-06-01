import pytest

pytestmark = [pytest.mark.sim]

"""Tests for shared CLI runtime boundary formatting."""

from __future__ import annotations

from types import SimpleNamespace


def test_runtime_display_formats_run_spec_like_object() -> None:
    from cli.runtime_display import (
        format_frame_links,
        format_product_acceptance_commands,
        format_product_runtime_boundary,
        format_runtime_boundary,
        format_runtime_flow,
        format_runtime_flow_stages,
        format_runtime_frames,
        format_runtime_sources,
        format_runtime_topic_frames,
    )

    runtime = SimpleNamespace(
        endpoint=None,
        data_source="real_s100p",
        runtime_contract="real_s100p",
        command_sink="hardware_driver_after_cmd_vel_mux",
        simulation_only=False,
        slam_source="lingtu_fastlio_or_external_robot_slam",
        localization_source="slam_localizer",
        mapping_source="slam_map_cloud",
        lidar_extrinsic_profile=None,
        frames={
            "map": "map",
            "odom": "odom",
            "body": "body",
            "lidar": "lidar_link",
            "real_lidar": "livox_frame",
            "camera": "camera_link",
            "axis_convention": "x_forward_y_left_z_up",
        },
        frame_links={
            "map_to_odom": {"parent": "map", "child": "odom"},
            "odom_to_body": {"parent": "odom", "child": "body"},
        },
        topic_allowed_frame_ids={
            "/nav/local_path": ("map", "odom", "body"),
            "/nav/map_cloud": ("map",),
            "/nav/cmd_vel": ("body",),
        },
        resolved_runtime_data_flow=(
            {"name": "endpoint_adapter", "inputs": ("/nav/lidar_scan", "/nav/imu")},
            {
                "name": "slam_or_relayed_localization_map",
                "owner": "slam_or_source_adapter",
                "frame_role": "map_odom_body",
                "inputs": ("/nav/lidar_scan", "/nav/imu"),
                "outputs": ("/nav/odometry", "/nav/map_cloud"),
            },
            {
                "name": "global_planning",
                "owner": "lingtu_navigation_or_pct",
                "frame_role": "map",
                "inputs": ("/nav/odometry", "/nav/goal_pose"),
                "outputs": ("/nav/global_path",),
            },
            {
                "name": "command_boundary",
                "owner": "cmd_vel_mux_to_endpoint_sink",
                "frame_role": "body_twist",
                "inputs": ("/nav/cmd_vel",),
                "outputs": ("hardware_driver_after_cmd_vel_mux",),
            },
        ),
        runtime_data_flow_stage_algorithm_interfaces={
            "slam_or_relayed_localization_map": (
                "fastlio_mapping",
                "fastlio_raw_validation",
            ),
            "global_planning": (
                "global_planning",
                "astar_global_planning",
                "pct_global_planning",
            ),
        },
    )

    assert format_runtime_boundary(runtime) == (
        "endpoint=in_process data_source=real_s100p "
        "runtime_contract=real_s100p "
        "command_sink=hardware_driver_after_cmd_vel_mux simulation_only=false"
    )
    assert format_runtime_sources(runtime) == (
        "slam_source=lingtu_fastlio_or_external_robot_slam "
        "localization_source=slam_localizer "
        "mapping_source=slam_map_cloud lidar_extrinsic=none"
    )
    assert format_runtime_frames(runtime) == (
        "map=map odom=odom body=body lidar=lidar_link "
        "real_lidar=livox_frame camera=camera_link "
        "axis_convention=x_forward_y_left_z_up"
    )
    assert format_frame_links(runtime) == "map->odom, odom->body"
    assert format_runtime_topic_frames(runtime) == (
        "map_cloud=map local_path=map,odom,body cmd_vel=body"
    )
    assert format_runtime_flow(runtime) == (
        "sensors=/nav/lidar_scan,/nav/imu "
        "localization_map=/nav/odometry,/nav/map_cloud "
        "command=hardware_driver_after_cmd_vel_mux"
    )
    assert format_product_runtime_boundary(runtime) == (
        "mode=field runtime_contract=real_s100p primary=Gateway+ModulePorts "
        "adapter=endpoint_only ros2_topic_inspection_required=false"
    )
    assert format_product_acceptance_commands(runtime) == (
        "python lingtu.py runtime-audit | "
        "python lingtu.py real-runtime-evidence --duration-sec 20 "
        "--json-out artifacts/real_s100p_runtime/report.json | "
        "python lingtu.py gateway-runtime-acceptance --acceptance-mode field "
        "--gateway-url http://<robot>:5050"
    )
    assert format_runtime_flow_stages(runtime) == (
        "endpoint_adapter[unknown|unknown] /nav/lidar_scan,/nav/imu->none | "
        "slam_or_relayed_localization_map[slam_or_source_adapter|map_odom_body] "
        "interfaces=fastlio_mapping,fastlio_raw_validation "
        "/nav/lidar_scan,/nav/imu->/nav/odometry,/nav/map_cloud | "
        "global_planning[lingtu_navigation_or_pct|map] "
        "interfaces=global_planning,astar_global_planning,pct_global_planning "
        "/nav/odometry,/nav/goal_pose->/nav/global_path | "
        "command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist] "
        "/nav/cmd_vel->hardware_driver_after_cmd_vel_mux"
    )


def test_runtime_display_preserves_run_state_dict_shape() -> None:
    from cli.runtime_display import format_runtime_sources

    runtime = {
        "slam_source": "slam",
        "localization_source": "localizer",
        "mapping_source": "map_cloud",
    }

    assert format_runtime_sources(runtime) == (
        "slam_source=slam localization_source=localizer mapping_source=map_cloud"
    )


def test_runtime_display_topic_frame_summary_uses_real_required_frame_contract() -> None:
    from cli.runtime_display import (
        TOPIC_FRAME_SUMMARY_TOPICS,
        format_runtime_topic_frames,
    )
    from core.runtime_interface import (
        REAL_RUNTIME_CONTRACT,
        TOPICS,
        runtime_required_topic_frame_ids,
    )

    runtime = {
        "topic_allowed_frame_ids": {
            TOPICS.cmd_vel: ("body",),
            TOPICS.local_path: ("map", "odom", "body"),
            TOPICS.map_cloud: ("map",),
            TOPICS.global_path: ("map",),
            TOPICS.registered_cloud: ("body",),
            TOPICS.odometry: ("odom", "map"),
        }
    }

    assert TOPIC_FRAME_SUMMARY_TOPICS == runtime_required_topic_frame_ids(
        REAL_RUNTIME_CONTRACT
    )
    assert format_runtime_topic_frames(runtime) == (
        "odometry=odom,map registered_cloud=body map_cloud=map "
        "global_path=map local_path=map,odom,body cmd_vel=body"
    )


def test_runtime_contract_manifest_summary_exposes_interfaces_flow_and_frames() -> None:
    from cli.runtime_display import format_runtime_contract_manifest
    from core.runtime_interface import runtime_contract_manifest

    output = format_runtime_contract_manifest(runtime_contract_manifest())

    assert "Runtime contract: lingtu.runtime_interface.v1" in output
    assert (
        "Frames: map=map odom=odom body=body lidar=lidar_link "
        "real_lidar=livox_frame camera=camera_link "
        "axis_convention=x_forward_y_left_z_up"
    ) in output
    assert "Frame links: map->odom, odom->body, body->lidar_link" in output
    assert "Real topic frames:" in output
    assert "  odometry=odom,map" in output
    assert "  registered_cloud=body" in output
    assert "  map_cloud=map" in output
    assert "  global_path=map" in output
    assert "  local_path=map,odom,body" in output
    assert "  cmd_vel=body" in output
    assert "Real data flow:" in output
    assert "  endpoint_adapter" in output
    assert "Data sources:" in output
    assert (
        "  real_s100p[hardware] source=/nav/lidar_scan,/nav/imu "
        "normalized=/nav/lidar_scan,/nav/imu"
    ) in output
    assert "command=hardware_driver_after_cmd_vel_mux" in output
    assert (
        "  mujoco_fastlio2_live[mujoco] source=/points_raw,/imu_raw "
        "normalized=/points_raw,/imu_raw"
    ) in output
    assert "Profile bindings:" in output
    assert "  nav->real_s100p mode=real_robot_saved_map_navigation" in output
    assert (
        "  sim_mujoco_live->mujoco_fastlio2_live "
        "mode=mujoco_raw_mid360_fastlio_live"
    ) in output
    assert "Artifact formats:" in output
    assert (
        "  tomogram path=tomogram.pickle type=pct_tomogram frame_role=map "
        "metadata=source_map_sha256,source_profile,data_source,frame_id,shape"
    ) in output
    assert "Adapter aliases:" in output
    assert (
        "  fastlio2 /cloud_registered->/nav/registered_cloud(registered_cloud),"
        "/cloud_map->/nav/map_cloud(map_cloud),/Odometry->/nav/odometry(odometry)"
    ) in output
    assert "Adapter relays:" in output
    assert (
        "  cmu_unity /state_estimation->/nav/odometry(odometry),"
        "/state_estimation_at_scan->/nav/state_estimation_at_scan"
        "(state_estimation_at_scan)"
    ) in output
    assert "/nav/cmd_vel->/cmd_vel(geometry_msgs/msg/TwistStamped)" in output
    assert "Stage interfaces:" in output
    assert "  slam_or_relayed_localization_map=" in output
    assert "Algorithm interfaces:" in output
    assert "  fastlio_mapping[slam|" in output


def test_runtime_spec_payload_summary_exposes_profile_flow_and_env() -> None:
    from cli.runtime_display import format_runtime_spec_payload

    output = format_runtime_spec_payload(
        {
            "ok": True,
            "validation": {"ok": True, "blockers": []},
            "spec": {
                "profile": "explore",
                "endpoint": "mujoco_live",
                "robot_preset": "sim",
                "data_source": "mujoco_fastlio2_live",
                "runtime_contract": "mujoco_fastlio2_live",
                "command_sink": "mujoco_velocity_adapter",
                "simulation_only": True,
                "slam_source": "lingtu_fastlio2",
                "localization_source": "fastlio2_odometry",
                "mapping_source": "fastlio2_map_cloud",
                "lidar_extrinsic_profile": "mujoco_thunder_v3",
                "frames": {
                    "map": "map",
                    "odom": "odom",
                    "body": "body",
                    "lidar": "lidar_link",
                    "real_lidar": "livox_frame",
                    "camera": "camera_link",
                    "axis_convention": "x_forward_y_left_z_up",
                },
                "frame_links": {
                    "map_to_odom": {"parent": "map", "child": "odom"},
                    "odom_to_body": {"parent": "odom", "child": "body"},
                },
                "topic_allowed_frame_ids": {
                    "/nav/odometry": ["odom", "map"],
                    "/nav/map_cloud": ["map", "odom"],
                    "/nav/cmd_vel": ["body"],
                },
                "resolved_runtime_data_flow": [
                    {
                        "name": "endpoint_adapter",
                        "owner": "endpoint_adapter",
                        "frame_role": "native_to_canonical",
                        "inputs": ["/points_raw", "/imu_raw"],
                        "outputs": ["/nav/lidar_scan", "/nav/imu"],
                    },
                    {
                        "name": "command_boundary",
                        "owner": "cmd_vel_mux_to_endpoint_sink",
                        "frame_role": "body_twist",
                        "inputs": ["/nav/cmd_vel"],
                        "outputs": ["mujoco_velocity_adapter"],
                    },
                ],
                "runtime_data_flow_stage_algorithm_interfaces": {},
            },
            "env": {
                "LINGTU_ENDPOINT": "mujoco_live",
                "LINGTU_DATA_SOURCE": "mujoco_fastlio2_live",
                "LINGTU_RUNTIME_CONTRACT": "mujoco_fastlio2_live",
                "LINGTU_COMMAND_SINK": "mujoco_velocity_adapter",
                "LINGTU_SIMULATION_ONLY": "1",
            },
        }
    )

    assert "Runtime spec: PASS" in output
    assert "Profile: profile=explore endpoint=mujoco_live robot_preset=sim" in output
    assert (
        "Runtime: endpoint=mujoco_live data_source=mujoco_fastlio2_live "
        "runtime_contract=mujoco_fastlio2_live "
        "command_sink=mujoco_velocity_adapter simulation_only=true"
    ) in output
    assert (
        "Product boundary: mode=simulation runtime_contract=mujoco_fastlio2_live "
        "primary=Gateway+ModulePorts adapter=endpoint_only "
        "ros2_topic_inspection_required=false"
    ) in output
    assert "Product acceptance:" in output
    assert "  python lingtu.py runtime-audit" in output
    assert (
        "  python lingtu.py gateway-runtime-acceptance --acceptance-mode simulation"
        in output
    )
    assert "Topic frames:" in output
    assert "  odometry=odom,map" in output
    assert "Data flow:" in output
    assert "  endpoint_adapter[endpoint_adapter|native_to_canonical]" in output
    assert "Runtime env:" in output
    assert "  LINGTU_RUNTIME_CONTRACT=mujoco_fastlio2_live" in output


@pytest.mark.sim
def test_runtime_switch_plan_summary_exposes_sim_real_boundary() -> None:
    from cli.runtime_display import format_runtime_switch_plan

    base_frames = {
        "map": "map",
        "odom": "odom",
        "body": "body",
        "lidar": "lidar_link",
        "real_lidar": "livox_frame",
        "camera": "camera_link",
        "axis_convention": "x_forward_y_left_z_up",
    }
    base_links = {
        "map_to_odom": {"parent": "map", "child": "odom"},
        "odom_to_body": {"parent": "odom", "child": "body"},
    }
    output = format_runtime_switch_plan(
        {
            "ok": False,
            "from": {
                "profile": "sim_mujoco_live",
                "endpoint": "mujoco_live",
                "robot_preset": "sim_gazebo",
                "data_source": "mujoco_fastlio2_live",
                "runtime_contract": "mujoco_fastlio2_live",
                "command_sink": "mujoco_velocity_adapter",
                "simulation_only": True,
                "slam_source": "lingtu_fastlio2",
                "localization_source": "fastlio2_odometry",
                "mapping_source": "fastlio2_map_cloud",
                "lidar_extrinsic_profile": "mujoco_thunder_v3",
                "frames": base_frames,
                "frame_links": base_links,
                "topic_allowed_frame_ids": {
                    "/nav/map_cloud": ["map", "odom"],
                    "/nav/cmd_vel": ["body"],
                },
                "required_topic_frame_ids": [],
                "runtime_data_flow_topics": ["/points_raw", "/nav/cmd_vel"],
                "resolved_runtime_data_flow": [
                    {
                        "name": "endpoint_adapter",
                        "owner": "endpoint_adapter",
                        "frame_role": "native_to_canonical",
                        "inputs": ["/points_raw"],
                        "outputs": ["/nav/lidar_scan"],
                    },
                    {
                        "name": "command_boundary",
                        "owner": "cmd_vel_mux_to_endpoint_sink",
                        "frame_role": "body_twist",
                        "inputs": ["/nav/cmd_vel"],
                        "outputs": ["mujoco_velocity_adapter"],
                    },
                ],
            },
            "to": {
                "profile": "explore",
                "endpoint": "real_s100p",
                "robot_preset": "s100p",
                "data_source": "real_s100p",
                "runtime_contract": "real_s100p",
                "command_sink": "hardware_driver_after_cmd_vel_mux",
                "simulation_only": False,
                "slam_source": "lingtu_fastlio_or_external_robot_slam",
                "localization_source": "slam_localizer",
                "mapping_source": "slam_map_cloud",
                "lidar_extrinsic_profile": "real_mid360",
                "frames": base_frames,
                "frame_links": base_links,
                "topic_allowed_frame_ids": {
                    "/nav/map_cloud": ["map"],
                    "/nav/cmd_vel": ["body"],
                },
                "required_topic_frame_ids": ["/nav/odometry", "/nav/cmd_vel"],
                "runtime_data_flow_topics": ["/nav/lidar_scan", "/nav/cmd_vel"],
                "resolved_runtime_data_flow": [
                    {
                        "name": "endpoint_adapter",
                        "owner": "endpoint_adapter",
                        "frame_role": "native_to_canonical",
                        "inputs": ["/nav/lidar_scan"],
                        "outputs": ["/nav/lidar_scan"],
                    },
                    {
                        "name": "command_boundary",
                        "owner": "cmd_vel_mux_to_endpoint_sink",
                        "frame_role": "body_twist",
                        "inputs": ["/nav/cmd_vel"],
                        "outputs": ["hardware_driver_after_cmd_vel_mux"],
                    },
                ],
            },
            "changed": ["command_sink", "data_source", "simulation_only"],
            "current_validation": {"ok": True, "blockers": []},
            "target_validation": {
                "ok": False,
                "blockers": ["real endpoint does not use hardware command sink"],
            },
        }
    )

    assert "Runtime switch plan: FAIL" in output
    assert (
        "Switch lifecycle: dry-run/preflight; endpoint changes require a fresh launcher"
        in output
    )
    assert "Current profile: profile=sim_mujoco_live endpoint=mujoco_live robot_preset=sim_gazebo" in output
    assert "Target profile: profile=explore endpoint=real_s100p robot_preset=s100p" in output
    assert "Current runtime: endpoint=mujoco_live data_source=mujoco_fastlio2_live" in output
    assert "Target runtime: endpoint=real_s100p data_source=real_s100p" in output
    assert (
        "Current product boundary: mode=simulation "
        "runtime_contract=mujoco_fastlio2_live primary=Gateway+ModulePorts "
        "adapter=endpoint_only ros2_topic_inspection_required=false"
    ) in output
    assert (
        "Target product boundary: mode=field runtime_contract=real_s100p "
        "primary=Gateway+ModulePorts adapter=endpoint_only "
        "ros2_topic_inspection_required=false"
    ) in output
    assert "Target product acceptance:" in output
    assert "real-runtime-evidence --duration-sec 20" in output
    assert "Current topic frames:" in output
    assert "  map_cloud=map,odom cmd_vel=body" in output
    assert "Target required topic frames:" in output
    assert "  /nav/odometry,/nav/cmd_vel" in output
    assert "Current data-flow topics:" in output
    assert "  /points_raw,/nav/cmd_vel" in output
    assert "Target data flow:" in output
    assert "hardware_driver_after_cmd_vel_mux" in output
    assert "Changed fields:" in output
    assert "  command_sink" in output
    assert "Current validation: PASS" in output
    assert "Target validation: FAIL" in output
    assert "  - real endpoint does not use hardware command sink" in output


def test_runtime_audit_payload_summary_exposes_checks_and_gate_sequence() -> None:
    from cli.runtime_display import format_runtime_audit_payload

    output = format_runtime_audit_payload(
        {
            "schema_version": "lingtu.runtime_contract_audit.v1",
            "ok": True,
            "blockers": [],
            "validation_gate": {
                "acceptance_step": 1,
                "required_when": "before_any_runtime_contract_or_field_readiness_claim",
                "requires_prior_gates": [],
                "conditional_prior_gates": [],
                "proves": [
                    "canonical_runtime_manifest_matches_yaml",
                ],
                "operator_summary_sections": [
                    "Blockers",
                    "Checks",
                    "Validation gate sequence",
                    "Validation commands",
                ],
            },
            "checks": {
                "yaml_manifest": {"ok": True, "blockers": []},
                "runtime_validation_gates": {
                    "ok": True,
                    "blockers": [],
                    "commands": {
                        "runtime_audit": "python lingtu.py runtime-audit --json-out artifacts/runtime_contract_audit.json",
                        "real_runtime_evidence": "python lingtu.py real-runtime-evidence --duration-sec 20 --json-out artifacts/real_s100p_runtime/report.json",
                    },
                    "acceptance": {
                        "runtime_audit": {
                            "acceptance_step": 1,
                            "required_when": "before_any_runtime_contract_or_field_readiness_claim",
                            "requires_prior_gates": [],
                            "conditional_prior_gates": [],
                            "proves": [
                                "canonical_runtime_manifest_matches_yaml",
                            ],
                            "operator_summary_sections": [
                                "Blockers",
                                "Checks",
                                "Validation gate sequence",
                                "Validation commands",
                            ],
                        },
                        "saved_map_artifact_gate": {
                            "acceptance_step": 2,
                            "required_when": "saved_map_tomogram_occupancy_or_pct_artifact_is_used",
                            "requires_prior_gates": ["runtime_audit"],
                            "conditional_prior_gates": [],
                            "proves": [
                                "saved_map_metadata_exists",
                                "map_pcd_checksum_matches_metadata",
                            ],
                            "operator_summary_sections": [
                                "Expected",
                                "Required artifacts",
                                "Metadata",
                                "Artifacts",
                                "Blockers",
                            ],
                        },
                        "real_runtime_evidence": {
                            "acceptance_step": 3,
                            "required_when": "before_claiming_real_s100p_runtime_or_field_navigation",
                            "requires_prior_gates": ["runtime_audit"],
                            "conditional_prior_gates": [
                                "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
                            ],
                            "proves": [
                                "observed_real_s100p_runtime_contract",
                                "observed_resolved_runtime_data_flow",
                            ],
                            "operator_summary_sections": [
                                "Blockers",
                                "Topic frame evidence",
                                "Frame link evidence",
                                "Stage evidence matrix",
                                "Data-flow evidence",
                            ],
                        },
                    },
                },
            },
        }
    )

    assert "Runtime audit: PASS" in output
    assert "Schema: lingtu.runtime_contract_audit.v1" in output
    assert "Validation gate:" in output
    assert (
        "  step=1 "
        "required_when=before_any_runtime_contract_or_field_readiness_claim "
        "prior=none conditional_prior=none "
        "proves=canonical_runtime_manifest_matches_yaml "
        "summary_sections=Blockers,Checks,Validation gate sequence,"
        "Validation commands"
    ) in output
    assert "Checks:" in output
    assert "  yaml_manifest ok=true blockers=0" in output
    assert "  runtime_validation_gates ok=true blockers=0" in output
    assert "Validation gate sequence:" in output
    assert (
        "  step=1 runtime_audit "
        "required_when=before_any_runtime_contract_or_field_readiness_claim "
        "prior=none conditional_prior=none "
        "proves=canonical_runtime_manifest_matches_yaml "
        "summary_sections=Blockers,Checks,Validation gate sequence,"
        "Validation commands"
    ) in output
    assert (
        "  step=2 saved_map_artifact_gate "
        "required_when=saved_map_tomogram_occupancy_or_pct_artifact_is_used "
        "prior=runtime_audit conditional_prior=none "
        "proves=saved_map_metadata_exists,map_pcd_checksum_matches_metadata "
        "summary_sections=Expected,Required artifacts,Metadata,Artifacts,Blockers"
    ) in output
    assert (
        "  step=3 real_runtime_evidence "
        "required_when=before_claiming_real_s100p_runtime_or_field_navigation "
        "prior=runtime_audit"
    ) in output
    assert (
        "summary_sections=Blockers,Topic frame evidence,"
        "Frame link evidence,Stage evidence matrix,Data-flow evidence"
    ) in output
    assert "Validation commands:" in output
    assert "  runtime_audit=python lingtu.py runtime-audit" in output


def test_saved_map_artifact_gate_summary_exposes_frame_source_and_artifacts() -> None:
    from cli.runtime_display import format_saved_map_artifact_gate_payload

    output = format_saved_map_artifact_gate_payload(
        {
            "schema_version": "lingtu.saved_map_artifacts.gate.v1",
            "ok": False,
            "map_dir": "maps/large_terrain",
            "validation_gate": {
                "acceptance_step": 2,
                "required_when": "saved_map_tomogram_occupancy_or_pct_artifact_is_used",
                "requires_prior_gates": ["runtime_audit"],
                "conditional_prior_gates": [],
                "proves": [
                    "saved_map_metadata_exists",
                    "tomogram_and_occupancy_derive_from_same_map_pcd",
                ],
                "operator_summary_sections": [
                    "Expected",
                    "Required artifacts",
                    "Metadata",
                    "Artifacts",
                    "Blockers",
                ],
            },
            "checked_required_artifacts": ["map_pcd", "tomogram"],
            "checked_allowed_frame_ids": ["map", "odom"],
            "checked_frame_id": "odom",
            "checked_expected": {
                "data_source": "real_s100p",
                "source_profile": "nav",
                "frame_id": "map",
            },
            "metadata": {
                "path": "maps/large_terrain/metadata.json",
                "exists": True,
            },
            "metadata_validation": {"ok": True, "blockers": []},
            "artifacts": {
                "map_pcd": {
                    "path": "maps/large_terrain/map.pcd",
                    "exists": True,
                    "sha256_ok": True,
                },
                "tomogram": {
                    "path": "maps/large_terrain/tomogram.pickle",
                    "exists": False,
                    "sha256_ok": False,
                },
            },
            "blockers": ["metadata.frame_id does not match expected frame_id"],
        }
    )

    assert "Saved map artifact gate: FAIL" in output
    assert "Map dir: maps/large_terrain" in output
    assert "Validation gate:" in output
    assert (
        "  step=2 "
        "required_when=saved_map_tomogram_occupancy_or_pct_artifact_is_used "
        "prior=runtime_audit conditional_prior=none "
        "proves=saved_map_metadata_exists,"
        "tomogram_and_occupancy_derive_from_same_map_pcd "
        "summary_sections=Expected,Required artifacts,Metadata,Artifacts,Blockers"
    ) in output
    assert "Frame: observed=odom allowed=map,odom" in output
    assert "Expected:" in output
    assert "  data_source=real_s100p" in output
    assert "  source_profile=nav" in output
    assert "  frame_id=map" in output
    assert "Required artifacts:" in output
    assert "  map_pcd" in output
    assert "  tomogram" in output
    assert "Metadata:" in output
    assert "  path=maps/large_terrain/metadata.json" in output
    assert "  exists=true" in output
    assert "  ok=true" in output
    assert "Artifacts:" in output
    assert "  map_pcd exists=true sha256_ok=true path=maps/large_terrain/map.pcd" in output
    assert "  tomogram exists=false sha256_ok=false path=maps/large_terrain/tomogram.pickle" in output
    assert "Blockers:" in output
    assert "  metadata.frame_id does not match expected frame_id" in output


def test_real_runtime_evidence_summary_exposes_blockers_and_boundary() -> None:
    from cli.runtime_display import format_real_runtime_evidence_summary

    output = format_real_runtime_evidence_summary(
        {
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "motion": {"odom_delta_m": 0.0, "min_motion_m": 0.05},
            "outputs": {
                "global_path_count": 0,
                "local_path_count": 0,
                "nav_cmd_vel_nonzero": 0,
            },
            "hardware_boundary": {
                "command_sink": "hardware_driver_after_cmd_vel_mux",
            },
            "validation_gate": {
                "acceptance_step": 3,
                "required_when": "before_claiming_real_s100p_runtime_or_field_navigation",
                "requires_prior_gates": ["runtime_audit"],
                "conditional_prior_gates": [
                    "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
                ],
                "proves": [
                    "observed_real_s100p_runtime_contract",
                    "observed_resolved_runtime_data_flow",
                ],
                "operator_summary_sections": [
                    "Blockers",
                    "Topic frame evidence",
                    "Frame link evidence",
                    "Stage evidence matrix",
                    "Data-flow evidence",
                ],
            },
            "runtime_contract": {"name": "real_s100p", "ok": False},
            "runtime_evidence": {
                "ok": False,
                "blockers": [
                    "real robot motion evidence missing",
                    "cmd_vel did not reach hardware boundary",
                ],
                "checked_runtime_topics": ["/nav/odometry", "/nav/cmd_vel"],
                "checked_frame_links": ["map_to_odom", "odom_to_body"],
                "checked_data_flow_stages": ["command_boundary"],
                "checked_runtime_data_flow_stage_algorithm_interfaces": {
                    "command_boundary": ["cmd_vel_mux_command_boundary"],
                },
                "checked_required_topic_frame_report": {
                    "/nav/odometry": {
                        "default_frame_id": "odom",
                        "observed_frame_id": None,
                        "allowed_frame_ids": ["odom", "map"],
                        "ok": False,
                    },
                    "/nav/cmd_vel": {
                        "default_frame_id": "body",
                        "observed_frame_id": "body",
                        "allowed_frame_ids": ["body"],
                        "ok": True,
                    },
                },
                "checked_frame_link_evidence": {
                    "map_to_odom": {
                        "expected_parent": "map",
                        "expected_child": "odom",
                        "observed_parent": None,
                        "observed_child": None,
                        "samples": 0,
                        "static": False,
                        "published": False,
                        "error": "tf missing",
                        "ok": False,
                    }
                },
                "checked_runtime_data_flow_evidence": {
                    "command_boundary": {
                        "ok": False,
                        "required": True,
                        "observed_inputs": ["/nav/cmd_vel"],
                        "observed_outputs": [],
                        "missing_inputs": [],
                        "missing_outputs": ["hardware_driver_after_cmd_vel_mux"],
                        "missing_signals": ["hardware_command_route"],
                        "owner": "cmd_vel_mux_to_endpoint_sink",
                        "frame_role": "body_twist",
                        "map_dependency": "command_sink_boundary",
                        "reason": "hardware boundary missing",
                    }
                },
            },
        }
    )

    assert "Real runtime evidence: FAIL" in output
    assert "Runtime contract: name=real_s100p ok=false" in output
    assert "Validation gate:" in output
    assert (
        "  step=3 "
        "required_when=before_claiming_real_s100p_runtime_or_field_navigation "
        "prior=runtime_audit"
    ) in output
    assert (
        "proves=observed_real_s100p_runtime_contract,"
        "observed_resolved_runtime_data_flow"
    ) in output
    assert (
        "summary_sections=Blockers,Topic frame evidence,"
        "Frame link evidence,Stage evidence matrix,Data-flow evidence"
    ) in output
    assert "Motion: real_robot_motion=false odom_delta_m=0.0 min_motion_m=0.05" in output
    assert "Command boundary: cmd_vel_sent_to_hardware=false" in output
    assert "  real robot motion evidence missing" in output
    assert "  cmd_vel did not reach hardware boundary" in output
    assert "  /nav/odometry" in output
    assert "  map_to_odom" in output
    assert "Topic frame evidence:" in output
    assert (
        "  /nav/odometry default=odom observed=missing "
        "allowed=odom,map ok=false"
    ) in output
    assert "  /nav/cmd_vel default=body observed=body allowed=body ok=true" in output
    assert "Frame link evidence:" in output
    assert (
        "  map_to_odom expected=map->odom observed=missing samples=0 "
        "static=false published=false ok=false error=tf missing"
    ) in output
    assert "Stage evidence matrix:" in output
    assert (
        "  command_boundary ok=false owner=cmd_vel_mux_to_endpoint_sink "
        "frame=body_twist map=command_sink_boundary "
        "interfaces=cmd_vel_mux_command_boundary observed=/nav/cmd_vel "
        "missing=hardware_command_route"
    ) in output
    assert "Data-flow evidence:" in output
    assert (
        "  command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist] ok=false "
        "observed_inputs=/nav/cmd_vel observed_outputs=none missing_inputs=none "
        "missing_outputs=hardware_driver_after_cmd_vel_mux "
        "missing_signals=hardware_command_route reason=hardware boundary missing"
    ) in output
