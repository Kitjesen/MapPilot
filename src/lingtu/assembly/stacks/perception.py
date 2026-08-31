"""Scene perception stack.

PerceptionModule is the default RGB-D scene-perception boundary. It owns the
detector, encoder, projection, and tracker capabilities needed to publish
scene_graph and detections_3d.
"""

from __future__ import annotations

from runtime.blueprint import Blueprint
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


def camera(**config) -> Blueprint:
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
            from runtime.config import get_config

            cam_rotate = get_config().raw.get("camera", {}).get("rotate", 0)
        bp.add(CameraModule, alias=CAMERA_ROLE, rotate=int(cam_rotate))

    return bp


def perception(detector: str = "yoloe", encoder: str = "none", **config) -> Blueprint:
    """RGB-D scene perception plus optional reconstruction and standalone tools."""
    bp = camera(**config)

    PerceptionModule = stack_module(
        "perception",
        "scene",
        seed_group="perception",
        fallback="perception.perception_module.PerceptionModule",
    )

    bp.add(
        PerceptionModule,
        alias="PerceptionModule",
        detector_type=detector,
        encoder_type=encoder,
        confidence_threshold=config.get("confidence", 0.3),
        tracking_iou_threshold=config.get(
            "tracking_iou_threshold",
            config.get("iou_threshold", 0.3),
        ),
        detector_iou_threshold=config.get(
            "detector_iou_threshold",
            config.get("iou_threshold", 0.45),
        ),
        detector_max_detections=config.get(
            "detector_max_detections",
            config.get("max_detections", 64),
        ),
        detector_min_box_size_px=config.get(
            "detector_min_box_size_px",
            config.get("min_box_size_px", 12),
        ),
        detector_model_size=config.get("model_size", "l"),
        detector_device=config.get("device", ""),
        detector_model_path=config.get(
            "detector_model_path",
            config.get("model_path", ""),
        ),
        skip_frames=config.get("perception_skip_frames", 1),
        world=config.get("world", ""),
        scenario_entities=config.get("scenario_entities", ()),
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
