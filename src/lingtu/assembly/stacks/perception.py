"""Scene perception stack.

PerceptionModule is the default RGB-D scene-perception boundary. It owns the
detector, encoder, projection, and tracker capabilities needed to publish
scene_graph and detections_3d.
"""

from __future__ import annotations

import logging

from runtime.adapters.perception_gateway import camera_module
from runtime.blueprint import Blueprint
from runtime.plugin_resolution import (
    optional_fallback_module,
    optional_stack_module,
    stack_module,
)
from runtime.contracts import (
    CAMERA_BACKEND_ORBBEC,
    CAMERA_BACKEND_SIM,
    CAMERA_COMPAT_CONFIG_FORCE,
    CAMERA_CONFIG_FORCE,
    CAMERA_ROLE,
)

logger = logging.getLogger(__name__)


def camera(**config) -> Blueprint:
    """Build only the canonical camera source for non-semantic products."""
    bp = Blueprint()
    drv_name = config.get("_driver_cls_name", "")
    camera_enabled = bool(config.get("enable_camera", True))
    force_camera = bool(config.get(CAMERA_CONFIG_FORCE, config.get(CAMERA_COMPAT_CONFIG_FORCE)))
    use_driver_camera = bool(config.get("use_driver_camera", False))
    needs_camera = camera_enabled and (force_camera or not use_driver_camera)

    if needs_camera:
        try:
            default_backend = CAMERA_BACKEND_SIM if drv_name == "MujocoDriverModule" else CAMERA_BACKEND_ORBBEC
            CameraModule = camera_module(
                enable_ros2=bool(config.get("enable_ros2_camera_bridge", False)),
                backend=str(config.get("camera_backend", default_backend)),
            )
            if CameraModule is None:
                raise ImportError("no registered camera adapter")
            # Read camera rotation from robot_config.yaml
            cam_rotate = config.get("camera_rotate", 0)
            if cam_rotate == 0:
                try:
                    from runtime.config import get_config

                    cam_rotate = get_config().raw.get("camera", {}).get("rotate", 0)
                except Exception:
                    pass
            bp.add(CameraModule, alias=CAMERA_ROLE, rotate=int(cam_rotate))
        except ImportError:
            pass

    return bp


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """RGB-D scene perception plus optional reconstruction and standalone tools."""
    bp = camera(**config)
    if config.get("manage_services", True):
        logger.debug(
            "perception(manage_services=True) is ignored; external camera "
            "startup is handled by lingtu.assembly.stacks.system.external_services"
        )

    try:
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
        )
    except ImportError as e:
        logger.warning("Perception modules not available: %s", e)

    if config.get("enable_standalone_encoder", False):
        EncoderModule = optional_stack_module(
            "encoder",
            "pluggable",
            seed_group="perception",
            fallback="perception.encoding.encoder_module.EncoderModule",
        )
        if EncoderModule is not None:
            # Experimental tool module; the full-stack scene graph path uses
            # PerceptionModule's internal encoder capability.
            bp.add(EncoderModule, alias="EncoderModule", encoder=encoder)
        else:
            logger.warning("Standalone encoder module not available")

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
            )
        else:
            logger.warning("Inspection evidence module not available")

    ReconstructionModule = optional_fallback_module(
        "reconstruction",
        "default",
        fallback="perception.reconstruction.reconstruction_module.ReconstructionModule",
    )
    if ReconstructionModule is not None:
        bp.add(ReconstructionModule, alias="ReconstructionModule")

    # Optional: record keyframes to disk for offline reconstruction
    # Enabled when recon_save_dir is provided in config
    recon_save_dir = config.get("recon_save_dir", "")
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
    recon_server_url = config.get("recon_server_url", "")
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
