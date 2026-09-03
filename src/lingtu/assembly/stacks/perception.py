"""Scene perception stack.

PerceptionModule is the default RGB-D scene-perception boundary. It owns the
detector, projection, and tracker capabilities needed to publish scene_graph
and detections_3d.
"""

from __future__ import annotations

from perception.backends import DetectorSpec
from perception.pipeline import PerceptionSettings
from runtime.blueprint import Blueprint
from runtime.config import get_config
from runtime.contracts import (
    CAMERA_BACKEND_DDS,
    CAMERA_BACKEND_ORBBEC,
    CAMERA_BACKEND_SIM,
    CAMERA_CONFIG_FORCE,
    CAMERA_ROLE,
)
from runtime.plugin_resolution import (
    optional_fallback_module,
    optional_stack_module,
    stack_module,
)


def _camera(config: dict, *, robot_config=None) -> Blueprint:
    """Build only the canonical camera source for non-semantic products."""
    bp = Blueprint()
    drv_name = config.get("_driver_cls_name", "")
    camera_enabled = bool(config.get("enable_camera", True))
    force_camera = bool(config.get(CAMERA_CONFIG_FORCE))
    use_driver_camera = bool(config.get("use_driver_camera", False))
    needs_camera = camera_enabled and (force_camera or not use_driver_camera)

    if needs_camera:
        default_backend = (
            CAMERA_BACKEND_SIM
            if drv_name == "MujocoDriverModule"
            else CAMERA_BACKEND_ORBBEC
        )
        backend = str(config.get("camera_backend", default_backend))
        fallback = {
            CAMERA_BACKEND_SIM: "drivers.sim.camera.module.MujocoCameraModule",
            CAMERA_BACKEND_ORBBEC: "drivers.real.camera.module.OrbbecNativeCameraModule",
            CAMERA_BACKEND_DDS: "drivers.real.camera.dds_module.DdsCameraModule",
        }.get(backend)
        if fallback is None:
            raise ValueError(f"unsupported camera backend: {backend}")
        CameraModule = stack_module(
            CAMERA_ROLE,
            backend,
            seed_group="camera_sim" if backend == CAMERA_BACKEND_SIM else "camera",
            fallback=fallback,
        )
        cam_rotate = config.get("camera_rotate", 0)
        if cam_rotate == 0:
            if robot_config is None:
                robot_config = get_config()
            cam_rotate = robot_config.camera.rotate
        bp.add(CameraModule, alias=CAMERA_ROLE, rotate=int(cam_rotate))

    return bp


def camera(**config) -> Blueprint:
    """Build only the canonical camera source for non-semantic products."""
    return _camera(config)


def _explicit(config: dict, key: str, default, *aliases: str):
    for candidate in (key, *aliases):
        if candidate in config and config[candidate] is not None:
            return config[candidate]
    return default


def _perception_runtime_config(robot_config, detector_name: str, config: dict):
    perception_config = robot_config.perception
    detector_config = perception_config.detector
    tracking_config = perception_config.tracking

    settings = PerceptionSettings(
        default_classes=str(
            _explicit(config, "default_classes", perception_config.default_classes)
        ),
        min_depth=float(_explicit(config, "min_depth", perception_config.min_depth)),
        max_depth=float(_explicit(config, "max_depth", perception_config.max_depth)),
        u16_depth_scale=float(robot_config.camera.depth_scale),
        laplacian_threshold=float(
            _explicit(
                config,
                "laplacian_threshold",
                perception_config.laplacian_threshold,
            )
        ),
        merge_distance=float(
            _explicit(config, "merge_distance", tracking_config.merge_distance)
        ),
        tracking_iou_threshold=float(
            _explicit(
                config,
                "tracking_iou_threshold",
                tracking_config.iou_threshold,
                "iou_threshold",
            )
        ),
        max_objects=int(_explicit(config, "max_objects", tracking_config.max_objects)),
    )
    detector = DetectorSpec(
        name=detector_name,
        model_size=str(_explicit(config, "model_size", detector_config.model_size)),
        confidence=float(
            _explicit(config, "confidence", detector_config.confidence_threshold)
        ),
        iou_threshold=float(
            _explicit(
                config,
                "detector_iou_threshold",
                detector_config.iou_threshold,
                "iou_threshold",
            )
        ),
        max_detections=int(
            _explicit(
                config,
                "detector_max_detections",
                detector_config.max_detections,
                "max_detections",
            )
        ),
        min_box_size_px=int(
            _explicit(
                config,
                "detector_min_box_size_px",
                detector_config.min_box_size_px,
                "min_box_size_px",
            )
        ),
        device=str(_explicit(config, "device", detector_config.device)),
        model_path=str(
            _explicit(
                config,
                "detector_model_path",
                detector_config.model_path,
                "model_path",
            )
        ),
        world=str(config.get("world", "")),
        scenario_entities=tuple(config.get("scenario_entities", ())),
    )
    frame_config = {
        "skip_frames": int(
            _explicit(
                config,
                "perception_skip_frames",
                perception_config.skip_frames,
            )
        ),
        "max_rgbd_skew_s": float(
            _explicit(
                config,
                "max_rgbd_skew_s",
                perception_config.max_rgbd_skew_s,
            )
        ),
        "max_odom_age_s": float(
            _explicit(config, "max_odom_age_s", perception_config.max_odom_age_s)
        ),
        "max_map_odom_age_s": float(
            _explicit(
                config,
                "max_map_odom_age_s",
                perception_config.max_map_odom_age_s,
            )
        ),
    }
    return settings, detector, frame_config


def perception(detector: str = "yoloe", **config) -> Blueprint:
    """RGB-D scene perception plus optional reconstruction and standalone tools."""
    if "encoder" in config:
        raise ValueError("PerceptionModule no longer owns an encoder")
    robot_config = get_config()
    bp = _camera(config, robot_config=robot_config)

    PerceptionModule = stack_module(
        "perception",
        "scene",
        seed_group="perception",
        fallback="perception.module.PerceptionModule",
    )

    settings, detector_spec, frame_config = _perception_runtime_config(
        robot_config,
        detector,
        config,
    )

    bp.add(
        PerceptionModule,
        alias="PerceptionModule",
        settings=settings,
        detector=detector_spec,
        camera_to_body=robot_config.camera.T_camera_body,
        **frame_config,
    )

    if config.get("enable_inspection_evidence", False):
        InspectionEvidenceModule = optional_stack_module(
            "inspection_evidence",
            "native_bridge",
            seed_group="perception",
            fallback="perception.inspection.bridge_module.InspectionEvidenceModule",
        )
        if InspectionEvidenceModule is not None:
            bp.add(
                InspectionEvidenceModule,
                alias="InspectionEvidenceModule",
                domain_id=int(config.get("dds_domain_id", config.get("domain_id", 0))),
                evidence_root=config.get("inspection_evidence_root"),
                status_file=config.get("inspection_evidence_status_file"),
                max_rgb_odom_skew_s=float(
                    config.get("inspection_evidence_max_rgb_odom_skew_s", 0.2)
                ),
            )
        else:
            raise ImportError("inspection evidence module not available")

    recon_save_dir = config.get("recon_save_dir", "")
    recon_server_url = config.get("recon_server_url", "")
    if recon_save_dir or recon_server_url:
        ReconstructionModule = optional_fallback_module(
            "reconstruction",
            "default",
            fallback="perception.reconstruction.reconstruction_module.ReconstructionModule",
        )
        if ReconstructionModule is not None:
            bp.add(ReconstructionModule, alias="ReconstructionModule")

    # Optional: record keyframes to disk for offline reconstruction
    # Enabled when recon_save_dir is provided in config
    if recon_save_dir:
        DatasetRecorderModule = optional_stack_module(
            "reconstruction",
            "dataset_recorder",
            seed_group="reconstruction",
            fallback=("perception.reconstruction.dataset_recorder_module.DatasetRecorderModule"),
        )
        if DatasetRecorderModule is not None:
            bp.add(
                DatasetRecorderModule,
                alias="DatasetRecorderModule",
                save_dir=recon_save_dir,
                keyframe_dist_m=float(config.get("recon_kf_dist_m", 0.15)),
                keyframe_rot_rad=float(config.get("recon_kf_rot_rad", 0.17)),
                keyframe_time_s=float(config.get("recon_kf_time_s", 1.0)),
                max_depth_m=float(config.get("recon_max_depth_m", 6.0)),
                jpeg_quality=int(config.get("recon_jpeg_quality", 90)),
                session_name=str(config.get("recon_session", "")),
            )

    # Optional: stream keyframes to a remote reconstruction server
    # Enabled when recon_server_url is provided in config
    if recon_server_url:
        ReconKeyframeExporterModule = optional_stack_module(
            "reconstruction",
            "keyframe_exporter",
            seed_group="reconstruction",
            fallback=("perception.reconstruction.keyframe_exporter_module.ReconKeyframeExporterModule"),
        )
        if ReconKeyframeExporterModule is not None:
            bp.add(
                ReconKeyframeExporterModule,
                alias="ReconKeyframeExporterModule",
                server_url=recon_server_url,
                keyframe_dist_m=float(config.get("recon_kf_dist_m", 0.3)),
                keyframe_rot_rad=float(config.get("recon_kf_rot_rad", 0.26)),
                keyframe_time_s=float(config.get("recon_kf_time_s", 2.0)),
                jpeg_quality=int(config.get("recon_jpeg_quality", 85)),
            )

    return bp
