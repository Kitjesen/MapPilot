"""Scene perception stack.

PerceptionModule is the default RGB-D scene-perception boundary. It owns the
detector, encoder, projection, and tracker capabilities needed to publish
scene_graph and detections_3d.
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import (
    optional_fallback_module,
    optional_stack_module,
    stack_module,
)

logger = logging.getLogger(__name__)
_NATIVE_CAMERA_DRIVERS = {"MujocoDriverModule"}  # Only MuJoCo has built-in camera


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """RGB-D scene perception plus optional reconstruction and standalone tools."""
    bp = Blueprint()
    if config.get("manage_services", True):
        try:
            from core.service_manager import get_service_manager
            svc = get_service_manager()
            svc.ensure("camera")
        except Exception:
            pass

    drv_name = config.get("_driver_cls_name", "")
    needs_camera_bridge = bool(config.get("force_camera_bridge")) or (
        drv_name not in _NATIVE_CAMERA_DRIVERS
        and not bool(config.get("use_driver_camera", False))
    )

    if needs_camera_bridge:
        try:
            CameraBridgeModule = stack_module(
                "camera_bridge",
                "default",
                seed_group="camera",
                fallback="drivers.real.thunder.camera_bridge_module.CameraBridgeModule",
            )
            # Read camera rotation from robot_config.yaml
            cam_rotate = config.get("camera_rotate", 0)
            if cam_rotate == 0:
                try:
                    from core.config import get_config
                    cam_rotate = get_config().raw.get("camera", {}).get("rotate", 0)
                except Exception:
                    pass
            bp.add(CameraBridgeModule, alias="CameraBridgeModule", rotate=int(cam_rotate))
        except ImportError:
            pass

    try:
        PerceptionModule = stack_module(
            "perception",
            "scene",
            seed_group="perception",
            fallback="semantic.perception.perception_module.PerceptionModule",
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
            fallback="semantic.perception.encoder_module.EncoderModule",
        )
        if EncoderModule is not None:
            # Experimental tool module; the full-stack scene graph path uses
            # PerceptionModule's internal encoder capability.
            bp.add(EncoderModule, alias="EncoderModule", encoder=encoder)
        else:
            logger.warning("Standalone encoder module not available")

    ReconstructionModule = optional_fallback_module(
        "reconstruction",
        "default",
        fallback="semantic.reconstruction.reconstruction_module.ReconstructionModule",
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
            fallback=(
                "semantic.reconstruction.dataset_recorder_module."
                "DatasetRecorderModule"
            ),
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
            fallback=(
                "semantic.reconstruction.keyframe_exporter_module."
                "ReconKeyframeExporterModule"
            ),
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
