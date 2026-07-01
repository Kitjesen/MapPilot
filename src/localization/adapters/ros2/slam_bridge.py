"""SlamBridgeModule 锟?bridges DDS SLAM outputs into the Python Module pipeline.

Subscribes to DDS topics (published by C++ SLAM nodes):
  /slam/map_cloud       锟?PointCloud2 from SLAM
  /slam/odometry        锟?Odometry from SLAM

Uses lightweight cyclonedds (no rclpy dependency). Falls back to rclpy if needed.

Usage::

    bp.add(SlamBridgeModule)
    bp.add(SlamBridgeModule, cloud_topic="/my/cloud")
"""

from __future__ import annotations

import collections
import json
import logging
import math
import os
import subprocess
import threading
import time as _time
from typing import Any, Callable, Mapping

from runtime.service_manager import get_service_manager

from runtime.backend_status import BackendStatus
from runtime.tf import FrameTree, transform_from_stamped
from runtime.module import Module, skill
from runtime.msgs.numpy_compat import np
from runtime.msgs.geometry import Pose, Quaternion, Transform, Twist, Vector3
from runtime.msgs.gnss import GnssFixType, GnssOdom
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import (
    TOPICS,
    normalize_frame_id,
    topic_default_frame_id,
)
from runtime.runtime_policy import slam_backend_contract
from runtime.stream import In, Out
from runtime.utils.scene_mode_detector import SceneModeConfig, SceneModeDetector
from localization.adapters.ros2.relocalization_service import (
    RelocalizationResult,
    relocalize_saved_map,
    relocalize_saved_map_with_env,
    trigger_global_relocalize,
)

logger = logging.getLogger("localization.bridge")

# Localization health states
LOC_UNINIT = "UNINIT"        # No data received yet
LOC_TRACKING = "TRACKING"    # Receiving data, quality OK
LOC_DEGRADED = "DEGRADED"    # Receiving data, quality poor (cloud timeout or degeneracy)
# DEGRADED for too long with healthy GNSS available 锟?flag fallback so downstream
# consumers (Navigation) can run cautiously while we still refine the
# state. The actual GNSS+IMU dead-reckoning takeover lands in S2.5 once the
# fault-injection harness exists; this state is the wiring half.
LOC_FALLBACK_GNSS_ONLY = "FALLBACK_GNSS_ONLY"
LOC_LOST = "LOST"            # No odometry for > timeout
LOC_DIVERGED = "DIVERGED"    # Odometry values physically impossible 锟?auto-relocalize

# Degeneracy severity levels (from SLAM quality metrics)
DEGEN_NONE = "NONE"              # Normal operation
DEGEN_MILD = "MILD"              # Feature count low but usable
DEGEN_SEVERE = "SEVERE"          # Severe feature loss, high covariance
DEGEN_CRITICAL = "CRITICAL"      # Complete degeneracy (corridor/void)

LOCALIZER_ODOM_GRACE_HEALTH = {"LOCKED", "RECOVERED"}
LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL = "ODOM_MISSING"
LOCALIZER_ODOM_LOSS_RECOVERY_ACTION = "restart_localization_chain"
MAP_ODOM_TF_BACKENDS = {"localizer"}


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _normalize_quat(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    """Normalize a quaternion in-place. Returns (qx, qy, qz, qw).

    The rotation-matrix formulas downstream assume a unit quaternion; a
    non-unit input (numeric drift upstream) would scale the rotation matrix
    and corrupt subsequent composed transforms.
    """
    qn = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if qn > 1e-9:
        return qx / qn, qy / qn, qz / qn, qw / qn
    return qx, qy, qz, qw


@register("slam_bridge", "default", description="DDS SLAM 锟?Python Module bridge")
@register(
    "localization_adapter",
    "ros2_slam_bridge",
    description="ROS2/DDS SLAM localization adapter",
)
class SlamBridgeModule(Module, layer=1):
    """Bridges SLAM point cloud and odometry into Module ports via DDS.

    Tries cyclonedds first (lightweight), falls back to rclpy.
    On systems without either, starts silently in stub mode.
    """

    map_cloud:             Out[PointCloud2]
    saved_map:             Out[PointCloud2]  # refined static map from localizer (map frame)
    odometry:              Out[Odometry]
    localization_quality:  Out[float]
    alive:                 Out[bool]
    localization_status:   Out[dict]
    gnss_fusion_health:    Out[dict]
    map_odom_tf:           Out[dict]  # {tx,ty,tz,qx,qy,qz,qw,valid} 锟?localizer-emitted map閳姪dom
    # P4: TF jump events. Cartographer-style 锟?when the localizer's map閳姪dom
    # transform jumps (PGO optimisation, BBS3D relocalisation, manual reset),
    # downstream consumers (Navigation, OccupancyGrid, ESDF) need to
    # invalidate cached state and replan/re-accumulate. We do NOT smooth the
    # jump because the new value is the correct one; smoothing would erase
    # the very correction PGO just produced.
    map_frame_jump_event:  Out[dict]  # {ts, dt_m, dyaw_deg, prev: {...}, next: {...}}
    # N2: indoor / outdoor classification with hysteresis. Other modules
    # subscribe to gate behaviour (PGO GNSS factor, Kabsch yaw, speed limits).
    scene_mode:            Out[str]   # "indoor" | "outdoor" | "unknown"

    # Visual odometry input for selective DOF fusion during degeneracy
    visual_odom: In[Odometry]

    # GNSS ENU odometry for global drift anchoring (auto-wired from GnssModule)
    gnss_odom: In[GnssOdom]

    def __init__(
        self,
        cloud_topic: str = TOPICS.map_cloud,
        odom_topic: str = TOPICS.odometry,
        quality_topic: str = TOPICS.localization_quality,
        registered_cloud_topic: str = TOPICS.registered_cloud,
        saved_map_topic: str = TOPICS.saved_map_cloud,
        backend_profile: str = "bridge",
        **kw,
    ):
        super().__init__(**kw)
        self._cloud_topic = cloud_topic
        self._odom_topic = odom_topic
        self._quality_topic = quality_topic
        self._registered_cloud_topic = registered_cloud_topic
        self._saved_map_topic = saved_map_topic
        self._backend_profile = str(backend_profile or "bridge")
        self._backend_detect_cache: str = self._backend_profile
        self._backend_detect_mono: float = 0.0
        self._backend_detect_interval: float = kw.get("backend_detect_interval", 1.0)
        self._last_status_backend: str = self._backend_profile
        self._reader = None
        self._rclpy_node = None  # fallback
        self._rclpy_executor = None
        self._odom_recv_ts: collections.deque = collections.deque(maxlen=30)
        self._pointcloud_worker_lock = threading.Lock()
        self._pointcloud_worker_thread: threading.Thread | None = None
        self._pointcloud_worker_drops: int = 0
        self._saved_map_worker_lock = threading.Lock()
        self._saved_map_worker_thread: threading.Thread | None = None
        self._saved_map_worker_drops: int = 0
        self._odom_worker_lock = threading.Lock()
        self._odom_worker_thread: threading.Thread | None = None
        self._odom_worker_drops: int = 0

        # Localization health watchdog
        self._last_odom_time: float = 0.0
        self._last_cloud_time: float = 0.0
        self._last_odom_mono: float = 0.0
        self._last_cloud_mono: float = 0.0
        self._loc_state: str = LOC_UNINIT
        self._odom_timeout: float = kw.get("odom_timeout", 2.0)
        self._cloud_timeout: float = kw.get("cloud_timeout", 5.0)
        self._watchdog_hz: float = kw.get("watchdog_hz", 2.0)
        self._shutdown_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

        # Auto-recovery
        self._reconnect_timeout: float = kw.get("reconnect_timeout", 10.0)
        self._max_reconnects: int = kw.get("max_reconnects", 10)
        self._reconnect_count: int = 0
        self._relocalization_state: str = "idle"
        self._last_recovery_signal: str = "NONE"
        self._last_recovery_action: str = "none"
        self._last_recovery_ts: float = 0.0
        self._watchdog_start_mono: float = _time.monotonic()
        self._localizer_odom_loss_recovery_enabled: bool = bool(kw.get(
            "localizer_odom_loss_recovery",
            _env_bool("LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY", True),
        ))
        self._localizer_odom_loss_recovery_s: float = float(kw.get(
            "localizer_odom_loss_recovery_s",
            _env_float("LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_S", 20.0),
        ))
        self._localizer_odom_loss_recovery_cooldown_s: float = float(kw.get(
            "localizer_odom_loss_recovery_cooldown_s",
            _env_float("LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_COOLDOWN_S", 300.0),
        ))
        self._last_localizer_odom_loss_recovery_ts: float = 0.0
        self._localizer_odom_loss_recovery_inflight: bool = False

        # map->odom TF cache + freshness gate. The localizer emits a fresh
        # map->odom TF on every relocalize tick; if that stream stalls (node
        # crash, ICP divergence) the last TF goes stale and lifting odom with
        # it silently reports a frozen-but-confident map pose. Gate the apply
        # path on TF age so a stale TF is ignored rather than trusted.
        self._T_map_odom = None
        self._T_map_odom_mono: float = 0.0
        self._frame_tree = kw.get("frame_tree") or FrameTree.from_robot_config()
        self._map_odom_tf_ttl_s: float = float(kw.get(
            "map_odom_tf_ttl_s",
            _env_float("LINGTU_MAP_ODOM_TF_TTL_S", 5.0),
        ))
        self._restart_recovery_lock = threading.Lock()
        self._localizer_odom_loss_recovery_start_mono: float = 0.0
        self._localizer_odom_loss_recovery_start_wall: float = 0.0
        self._localizer_odom_loss_recovery_waiting_new_odom: bool = False

        # Drift guard 锟?detect odometry divergence and auto-relocalize
        self._drift_max_speed: float = kw.get("drift_max_speed", 5.0)       # m/s, quad max ~3
        self._drift_max_jump: float = kw.get("drift_max_jump", 2.0)         # m between frames
        self._drift_max_z_jump: float = kw.get("drift_max_z_jump", 2.0)     # m between frames
        self._drift_max_abs_z: float = kw.get("drift_max_abs_z", 20.0)      # m from odom plane
        self._drift_max_pos: float = kw.get("drift_max_pos", 500.0)         # m from origin
        self._drift_confirm_frames: int = kw.get("drift_confirm_frames", 5) # consecutive bad
        self._drift_bad_count: int = 0
        self._drift_last_good_pos: np.ndarray | None = None                 # for relocalize
        self._drift_last_good_yaw: float = 0.0
        self._drift_relocalize_cooldown: float = kw.get("drift_relocalize_cooldown", 30.0)
        self._drift_recovery_enabled: bool = bool(kw.get(
            "drift_recovery_enabled",
            _env_bool("LINGTU_SLAM_DRIFT_RECOVERY", True),
        ))
        self._drift_last_relocalize: float = 0.0
        self._prev_odom_pos: np.ndarray | None = None

        # SLAM degeneracy detection
        self._degeneracy_topic: str = kw.get("degeneracy_topic", "/slam/degeneracy")
        self._degeneracy_detail_topic: str = kw.get(
            "degeneracy_detail_topic", "/slam/degeneracy_detail")
        self._degen_level: str = DEGEN_NONE
        self._icp_fitness: float = 0.0
        self._effective_ratio: float = 1.0  # effective features / total features
        self._eigenvalue_ratio: float = 0.0  # condition number (max_eig / min_eig)
        self._condition_number: float = 0.0
        self._degenerate_dof_count: int = 0
        self._eigenvalues: np.ndarray | None = None  # 6-DOF eigenvalues
        self._dof_mask: np.ndarray | None = None  # 1.0=constrained, 0.0=degenerate
        self._max_pos_cov: float = 0.0        # max position covariance from Odometry
        # IEKF internals exposed via /slam/degeneracy_detail (extended layout v2):
        #   pos_cov_trace = trace of position covariance (m锟?; growing trace is the
        #     earliest signal of IEKF divergence 锟?leads pose blow-up by 30-60s.
        #   ieskf_iter_num = iterations the last update actually consumed.
        #   ieskf_converged = whether the loop exited via stop_func (vs hitting max_iter).
        self._pos_cov_trace: float = 0.0
        self._ieskf_iter_num: int = 0
        self._ieskf_converged: bool = True
        # Localizer-side multi-frame health (LOCKED / LOST / RECOVERED / UNKNOWN).
        # Set by the /slam/localization_health subscription (P3); reflects the
        # ICP-side N-of-M confirmed state, not the per-frame ICP success flag.
        self._localizer_health: str = "UNKNOWN"
        self._localizer_health_fitness: float = 0.0
        # R4: small_gicp now exposes iter + Hessian-derived position cov.
        # Both fields stay at -1 until the first /slam/localization_health
        # message arrives carrying them.
        self._localizer_health_iter: int = -1
        self._localizer_health_cov_trace: float = -1.0
        self._localizer_health_ts: float = 0.0
        self._localizer_health_timeout: float = float(
            kw.get("localizer_health_timeout", 2.0)
        )
        self._localizer_health_odom_grace_s: float = float(
            kw.get("localizer_health_odom_grace_s", 5.0)
        )

        # P4: TF jump thresholds. Translation gate (m) is the dominant trigger
        # for PGO loop closures (typically 0.5-3 m corrections); rotation gate
        # catches yaw-flip relocalisations from BBS3D global localisation.
        self._jump_t_threshold_m: float = float(kw.get("jump_t_threshold_m", 1.0))
        self._jump_r_threshold_deg: float = float(kw.get("jump_r_threshold_deg", 30.0))

        # N2: scene mode (indoor/outdoor) detector. Manual override flows
        # through env var LINGTU_SCENE_MODE; auto detection from GNSS health
        # is fed in _on_gnss_odom() and the watchdog loop.
        self._scene_mode_detector = SceneModeDetector(SceneModeConfig(
            hold_seconds=float(kw.get("scene_mode_hold_s", 5.0)),
            gnss_max_age_s=float(kw.get("gnss_max_age_for_fallback_s", 2.0)),
        ))
        self._last_published_scene_mode: str | None = None
        self._last_severe_warn: float = 0.0   # throttle SEVERE degeneracy warning
        self._last_degen_time: float = 0.0
        # Degeneracy thresholds (tunable)
        self._fitness_warn: float = kw.get("fitness_warn", 0.15)
        self._fitness_critical: float = kw.get("fitness_critical", 0.3)
        self._feature_ratio_warn: float = kw.get("feature_ratio_warn", 0.3)
        self._feature_ratio_critical: float = kw.get("feature_ratio_critical", 0.1)
        self._eigen_ratio_warn: float = kw.get("eigen_ratio_warn", 100.0)  # condition number

        # Selective DOF fusion 锟?visual odometry during degeneracy
        # Based on Selective KF (arXiv 2412.17235): only fuse degenerate DOFs
        self._visual_odom_fusion: bool = kw.get("visual_odom_fusion", True)
        self._visual_alpha: float = kw.get("visual_alpha", 0.6)  # visual blend weight during CRITICAL
        self._last_slam_odom: Odometry | None = None
        self._last_visual_odom: Odometry | None = None
        self._visual_anchor_T: np.ndarray | None = None  # SLAM pose when visual odom activated
        self._visual_fused_count: int = 0

        # GNSS fusion 锟?gently anchor long-term SLAM drift with RTK position.
        # Only XY is fused (Z / orientation stay with SLAM since GNSS altitude is
        # noisy and position alone gives no heading). Alignment is locked once,
        # then GNSS pulls SLAM toward global truth with weight alpha per frame.
        self._gnss_fusion: bool = kw.get("gnss_fusion", True)
        self._gnss_max_age_s: float = kw.get("gnss_max_age_s", 2.0)
        self._gnss_max_std_m: float = kw.get("gnss_max_std_m", 1.0)
        self._gnss_alpha_healthy: float = kw.get("gnss_alpha_healthy", 0.05)
        self._gnss_alpha_degraded: float = kw.get("gnss_alpha_degraded", 0.5)
        self._gnss_rtk_float_scale: float = kw.get("gnss_rtk_float_scale", 0.3)
        # Antenna position in body frame (metres). GNSS reports the antenna's
        # ENU position, so we compensate by rotating this offset into the map
        # frame (via SLAM orientation) and subtracting 锟?giving the body
        # origin position that can be fused with SLAM odometry.
        _ant = kw.get("gnss_antenna_offset", (0.0, 0.0, 0.0))
        try:
            antenna_offset = tuple(float(v) for v in _ant)
        except TypeError as exc:
            raise ValueError("gnss_antenna_offset must be an iterable of 3 numbers") from exc
        if len(antenna_offset) != 3:
            raise ValueError(
                f"gnss_antenna_offset must be length-3, got {len(antenna_offset)}"
            )
        self._gnss_antenna_offset = np.asarray(antenna_offset, dtype=float)
        self._last_gnss_odom: GnssOdom | None = None
        self._last_gnss_rx_ts: float = 0.0
        self._gnss_map_offset: np.ndarray | None = None  # shape (2,) XY, map = enu + offset
        self._gnss_fused_count: int = 0

        # Residual guard 锟?protects against locking on a bad first sample
        # (RTK multipath, init glitch). If aligned residual stays above the
        # warn threshold for too long, clear the offset to force re-lock on
        # the next healthy GNSS + SLAM sample.
        self._gnss_residual_warn_m: float = kw.get("gnss_residual_warn_m", 5.0)
        self._gnss_residual_warn_duration_s: float = kw.get(
            "gnss_residual_warn_duration_s", 10.0)
        self._gnss_residual_warn_ratio: float = kw.get(
            "gnss_residual_warn_ratio", 0.7)
        # Ring buffer of (ts, residual_m) for sliding-window ratio check
        self._gnss_residual_history: collections.deque = collections.deque()
        self._gnss_last_residual_m: float = 0.0
        self._gnss_relock_count: int = 0
        self._gnss_last_relock_ts: float = 0.0

        # FALLBACK_GNSS_ONLY transition tracking. Wall-clock time we first
        # entered DEGRADED in the current degraded streak; cleared on recovery.
        # Once we have been DEGRADED for `_fallback_after_degraded_s` AND GNSS
        # is healthy (RTK_FIXED/FLOAT, fresh) we promote to FALLBACK so
        # downstream Navigation can shed speed.
        self._degraded_since: float = 0.0
        self._fallback_after_degraded_s: float = float(
            kw.get("fallback_after_degraded_s", 10.0))
        self._gnss_max_age_for_fallback_s: float = float(
            kw.get("gnss_max_age_for_fallback_s", 2.0))

    def setup(self) -> None:
        # Visual odometry input for selective fusion
        if self._visual_odom_fusion:
            self.visual_odom.subscribe(self._on_visual_odom)

        # GNSS input for global position anchoring
        if self._gnss_fusion:
            self.gnss_odom.subscribe(self._on_gnss_odom)

        # Try cyclonedds first (lightweight, no ROS2 env needed)
        if self._try_cyclonedds():
            return
        # Fallback to rclpy
        if self._try_rclpy():
            return
        logger.info("SlamBridgeModule: no DDS backend available, stub mode")

    def _try_cyclonedds(self) -> bool:
        try:
            from runtime.dds import ROS2TopicReader, _HAS_CYCLONEDDS
            # Honour the LINGTU_DISABLE_DDS kill switch 锟?ROS2TopicReader
            # still imports (stub-safe) but its start() returns False;
            # without this check we'd silently construct a dead reader
            # instead of falling through to the rclpy path.
            if not _HAS_CYCLONEDDS:
                return False
            self._reader = ROS2TopicReader()
            self._reader.on_odometry(self._odom_topic, self._on_dds_odom)
            self._reader.on_pointcloud2(self._cloud_topic, self._on_dds_cloud)
            self._reader.on_pointcloud2(self._saved_map_topic, self._on_dds_saved_map)
            self._reader.on_float32(self._quality_topic, self._on_dds_quality)
            # Subscribe to /tf so we can relay map閳姪dom transform (from localizer ICP).
            # Downstream GatewayModule applies this to map_cloud points before SSE
            # so frontend sees both saved_map (already map frame) and map_cloud
            # (Fast-LIO2 odom frame) overlaid in the same reference frame.
            self._reader.on_tf("/tf", self._on_dds_tf)
            # Note: only subscribe to cloud_topic; use TOPICS.registered_cloud
            # for localizer mode (avoids duplicate accumulation when both topics fire)
            logger.info("SlamBridgeModule: using cyclonedds (lightweight)")
            return True
        except ImportError:
            return False

    def _try_rclpy(self) -> bool:
        node = None
        executor = None
        try:
            from nav_msgs.msg import Odometry as ROS2Odom
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import Float32  # noqa: F401
            try:
                from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            except Exception:
                MutuallyExclusiveCallbackGroup = None

            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            # Odometry is a latest-state stream for readiness/control gating.
            # Prefer dropping stale samples over letting a reliable queue stall
            # the Python bridge behind old poses.
            odom_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=5,
            )
            cloud_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=1,
            )
            saved_map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            status_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            node = Node("slam_bridge")
            executor = get_shared_executor()
            executor.add_node(node)
            odom_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup else None
            cloud_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup else None
            saved_map_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup else None
            status_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup else None
            self._create_subscription(
                node, PointCloud2, self._cloud_topic, self._on_rclpy_cloud,
                cloud_qos, callback_group=cloud_group)
            self._create_subscription(
                node, PointCloud2, self._saved_map_topic,
                self._on_rclpy_saved_map, saved_map_qos,
                callback_group=saved_map_group)
            # Note: only subscribe to cloud_topic; use TOPICS.registered_cloud
            # for localizer mode (avoids duplicate accumulation when both topics fire)
            self._create_subscription(
                node, ROS2Odom, self._odom_topic, self._on_rclpy_odom,
                odom_qos, callback_group=odom_group)
            # Also subscribe to /tf so map閳姪dom reaches downstream even when
            # cyclonedds is disabled (LINGTU_DISABLE_DDS=1). Without this,
            # Navigation plans from odom-frame (0,0) instead of the
            # localizer-aligned map-frame position 锟?PCT finds wrong start.
            try:
                from tf2_msgs.msg import TFMessage
                self._create_subscription(
                    node, TFMessage, "/tf", self._on_rclpy_tf,
                    status_qos, callback_group=status_group)
            except Exception as _e:
                logger.warning("SlamBridge: /tf rclpy sub failed: %s", _e)
            # Subscribe to degeneracy metrics if available
            self._subscribe_degeneracy_rclpy(
                node, status_qos, callback_group=status_group)
            self._rclpy_node = node
            self._rclpy_executor = executor
            logger.info("SlamBridgeModule: using rclpy (fallback)")
            return True
        except (ImportError, Exception) as e:
            self._cleanup_rclpy_node(node=node, executor=executor)
            logger.debug("SlamBridgeModule: rclpy unavailable: %s", e)
            return False

    @staticmethod
    def _create_subscription(node, msg_type, topic, callback, qos, *, callback_group=None):
        if callback_group is None:
            return node.create_subscription(msg_type, topic, callback, qos)
        try:
            return node.create_subscription(
                msg_type, topic, callback, qos, callback_group=callback_group)
        except TypeError:
            return node.create_subscription(msg_type, topic, callback, qos)

    def _cleanup_rclpy_node(self, node=None, executor=None) -> None:
        node = node if node is not None else self._rclpy_node
        executor = executor if executor is not None else self._rclpy_executor
        if node is None:
            return
        try:
            if executor is not None:
                executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if node is self._rclpy_node:
            self._rclpy_node = None
        if executor is self._rclpy_executor:
            self._rclpy_executor = None

    def _subscribe_degeneracy_rclpy(self, node, qos, *, callback_group=None) -> None:
        """Subscribe to SLAM degeneracy metrics via rclpy (best-effort).

        Topics:
          /slam/localization_quality 锟?Float32: ICP fitness score (lower=better)
          /slam/degeneracy          锟?Float32: effective_ratio (1.0=healthy)
          /slam/degeneracy_detail   锟?Float32MultiArray: 11-float detailed metrics
            [0]  effective_ratio
            [1]  condition_number
            [2]  min_eigenvalue
            [3]  max_eigenvalue
            [4]  degenerate_dof_count
            [5..10] eigenvalues (6 DOFs, ascending)
        """
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Float32, Float32MultiArray, String
            sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
            self._create_subscription(
                node, Float32, self._quality_topic, self._on_rclpy_quality,
                sensor_qos, callback_group=callback_group)
            self._create_subscription(
                node, Float32, self._degeneracy_topic,
                self._on_rclpy_degeneracy, sensor_qos,
                callback_group=callback_group)
            self._create_subscription(
                node, Float32MultiArray, self._degeneracy_detail_topic,
                self._on_rclpy_degeneracy_detail, sensor_qos,
                callback_group=callback_group)
            # Localizer-side multi-frame health: LOCKED / LOST / RECOVERED.
            # Reliability needs to be RELIABLE (not BEST_EFFORT) 锟?we cannot
            # afford to drop a LOST 锟?RECOVERED transition; navigation depends
            # on it.
            health_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            self._create_subscription(
                node, String, TOPICS.localization_health,
                self._on_rclpy_localization_health, health_qos,
                callback_group=callback_group)
        except Exception as e:
            logger.debug("Degeneracy subscription unavailable: %s", e)

    def start(self) -> None:
        super().start()
        if self._reader:
            self._reader.spin_background()
            self.alive.publish(True)
        elif self._rclpy_node:
            self.alive.publish(True)
        else:
            self.alive.publish(False)
        # Start localization health watchdog
        self._shutdown_event.clear()
        self._watchdog_start_mono = _time.monotonic()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True,
            name="slam-bridge-watchdog")
        self._watchdog_thread.start()

    def stop(self) -> None:
        self._shutdown_event.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        for worker in (
            self._pointcloud_worker_thread,
            self._saved_map_worker_thread,
            self._odom_worker_thread,
        ):
            if worker and worker.is_alive():
                worker.join(timeout=1.0)
        self._pointcloud_worker_thread = None
        self._saved_map_worker_thread = None
        self._odom_worker_thread = None
        if self._reader:
            self._reader.stop()
        self._cleanup_rclpy_node()
        super().stop()

    # 閳光偓閳光偓 cyclonedds callbacks (parsed CDR) 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

    def _on_dds_quality(self, msg) -> None:
        """DDS_Float32 锟?localization quality (lower = better)."""
        try:
            val = float(msg.data)
            self.localization_quality.publish(val)
            self._icp_fitness = val
            self._last_degen_time = _time.time()
            self._update_degeneracy_level()
        except Exception as e:
            logger.debug("SlamBridge dds quality error: %s", e)

    def _check_drift(self, odom: Odometry) -> bool:
        """Return True if odometry looks diverged (should NOT be published).

        Checks:
          1. |velocity| > max_speed (quad can't exceed ~3 m/s)
          2. |position jump| > max_jump between consecutive frames
          3. |Z jump| or |Z| impossible for a ground robot
          4. |position| > max_pos from origin (indoor environment bound)

        After drift_confirm_frames consecutive bad readings, triggers
        auto-relocalize to last known good position.
        """
        pos = np.array([odom.x, odom.y, odom.z])
        speed = float(
            math.sqrt(
                float(getattr(odom, "vx", 0.0)) ** 2
                + float(getattr(odom, "vy", 0.0)) ** 2
                + float(getattr(getattr(odom, "twist", None), "linear", Vector3()).z) ** 2
            )
        )
        bad = False

        # Check 1: speed
        if speed > self._drift_max_speed:
            bad = True

        # Check 2: position jump
        if self._prev_odom_pos is not None:
            jump = float(np.linalg.norm(pos - self._prev_odom_pos))
            if jump > self._drift_max_jump:
                bad = True
            z_jump = abs(float(pos[2] - self._prev_odom_pos[2]))
            if z_jump > self._drift_max_z_jump:
                bad = True

        # Check 3: ground-robot vertical bound
        if abs(float(pos[2])) > self._drift_max_abs_z:
            bad = True

        # Check 4: absolute XY position
        if np.linalg.norm(pos[:2]) > self._drift_max_pos:
            bad = True

        if bad:
            self._drift_bad_count += 1
        else:
            # Good frame 锟?save as fallback position for relocalize
            self._drift_bad_count = 0
            self._drift_last_good_pos = pos.copy()
            self._drift_last_good_yaw = getattr(odom, 'yaw', 0.0)
            self._prev_odom_pos = pos.copy()
            return False

        self._prev_odom_pos = pos.copy()

        if self._drift_bad_count >= self._drift_confirm_frames:
            now = _time.time()
            contract = self._backend_contract(self._current_backend_profile())
            if self._loc_state != LOC_DIVERGED:
                logger.error(
                    "Localization DIVERGED: pos=(%.1f,%.1f,%.1f) speed=%.1f "
                    "bad_frames=%d 锟?suppressing odometry",
                    pos[0], pos[1], pos[2], speed, self._drift_bad_count)
                self._loc_state = LOC_DIVERGED
            self._last_recovery_signal = LOC_DIVERGED
            self._last_recovery_action = str(contract["recovery_action"])
            self._last_recovery_ts = now
            self._relocalization_state = (
                "requested" if contract["relocalization_supported"] else "unsupported"
            )

            # Auto-relocalize (with cooldown)
            if (
                self._drift_recovery_enabled
                and now - self._drift_last_relocalize > self._drift_relocalize_cooldown
            ):
                self._drift_last_relocalize = now
                threading.Thread(
                    target=self._auto_relocalize,
                    daemon=True, name="drift-relocalize",
                ).start()
            return True  # suppress this frame

        return False

    def _restart_backend_for_recovery(self, profile: str) -> bool:
        """Restart the active localization backend after drift divergence."""
        backend = str(profile or "bridge").lower()
        try:
            svc = get_service_manager()
            if backend == "super_lio":
                svc.stop(
                    "slam",
                    "slam_pgo",
                    "localizer",
                    "super_lio",
                    "super_lio_relocation",
                )
                self._ensure_recovery_services(svc, "lidar", "super_lio")
                svc.wait_ready("lidar", "super_lio", timeout=15.0)
            elif backend == "super_lio_relocation":
                svc.stop(
                    "slam",
                    "slam_pgo",
                    "localizer",
                    "super_lio",
                    "super_lio_relocation",
                )
                self._ensure_recovery_services(svc, "lidar", "super_lio_relocation")
                svc.wait_ready("lidar", "super_lio_relocation", timeout=15.0)
            elif backend == "localizer":
                return self._restart_localization_chain_for_recovery()
            elif backend in {"fastlio2", "slam"}:
                svc.stop("slam", "slam_pgo")
                self._ensure_recovery_services(svc, "slam", "slam_pgo")
                svc.wait_ready("slam", timeout=15.0)
            else:
                svc.stop("slam")
                self._ensure_recovery_services(svc, "slam")
                svc.wait_ready("slam", timeout=15.0)
            return True
        except Exception as e:
            logger.warning("Drift guard: service_manager restart failed: %s", e)

        if backend == "localizer":
            return self._restart_localization_chain_for_recovery()

        service = (
            "robot-super-lio-relocation.service"
            if backend == "super_lio_relocation"
            else "super_lio" if backend == "super_lio" else "slam"
        )
        try:
            result = subprocess.run(
                ["sudo", "systemctl", "restart", service],
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=15,
                check=False,
            )
            if result.returncode != 0:
                logger.warning(
                    "Drift guard: %s restart returned %d: %s",
                    service, result.returncode,
                    (result.stderr or result.stdout or "").strip()[:200],
                )
            else:
                logger.warning("Drift guard: %s service restarted", service)
            return True
        except Exception as e:
            logger.error("Drift guard: %s restart failed: %s", service, e)
            return False

    def _wait_ros_topic_publishers(
        self,
        topic: str,
        *,
        min_publishers: int = 1,
        timeout: float = 30.0,
    ) -> bool:
        import shlex

        deadline = _time.monotonic() + timeout
        quoted_topic = shlex.quote(topic)
        cmd = (
            "set +u; "
            "[ -f /opt/ros/humble/setup.bash ] && "
            "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
            "[ -f /opt/lingtu/current/install/setup.bash ] && "
            "source /opt/lingtu/current/install/setup.bash >/dev/null 2>&1; "
            f"ros2 topic info {quoted_topic} 2>/dev/null | "
            "awk '/Publisher count:/ {print $3}' | tail -1"
        )
        while _time.monotonic() < deadline and not self._shutdown_event.is_set():
            try:
                result = subprocess.run(
                    ["bash", "-lc", cmd],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=5,
                    check=False,
                )
                publishers = int((result.stdout or "0").strip() or "0")
                if publishers >= min_publishers:
                    return True
            except Exception as e:
                logger.debug(
                    "Localization recovery: topic readiness check failed for %s: %s",
                    topic,
                    e,
                )
            self._shutdown_event.wait(timeout=1.0)
        return False

    def _wait_for_odom_sample_since(
        self,
        since_mono: float,
        *,
        timeout: float = 10.0,
    ) -> bool:
        deadline = _time.monotonic() + timeout
        while _time.monotonic() < deadline and not self._shutdown_event.is_set():
            if self._last_odom_mono >= since_mono:
                return True
            self._shutdown_event.wait(timeout=0.1)
        return False

    def _wait_for_localizer_health_since(
        self,
        since_wall: float,
        *,
        timeout: float = 10.0,
    ) -> bool:
        deadline = _time.monotonic() + timeout
        while _time.monotonic() < deadline and not self._shutdown_event.is_set():
            if self._localizer_health_ts >= since_wall:
                return True
            self._shutdown_event.wait(timeout=0.1)
        return False

    def _ensure_localizer_service_best_effort(self, svc: Any | None = None) -> None:
        """Avoid leaving the saved-map localizer stopped after failed recovery."""
        try:
            if svc is None:
                svc = get_service_manager()
            self._ensure_recovery_services(svc, "localizer")
            return
        except Exception as e:
            logger.warning(
                "Localization recovery: service_manager could not restart "
                "localizer after failed recovery: %s",
                e,
            )

        try:
            result = subprocess.run(
                ["sudo", "systemctl", "start", "robot-localizer.service"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=15,
                check=False,
            )
            if result.returncode != 0:
                logger.error(
                    "Localization recovery: best-effort localizer restart "
                    "failed: %s",
                    (result.stderr or result.stdout or "").strip(),
                )
        except Exception as e:
            logger.error(
                "Localization recovery: best-effort localizer restart failed: %s",
                e,
            )

    def _track_recovery_service_ownership(self) -> bool:
        configured = str(getattr(self, "_backend_profile", "bridge") or "bridge")
        return configured.strip().lower() != "bridge"

    def _ensure_recovery_services(self, svc: Any, *services: str) -> None:
        track_started = self._track_recovery_service_ownership()
        try:
            svc.ensure(*services, track_started=track_started)
        except TypeError:
            svc.ensure(*services)

    def _restart_localization_chain_for_recovery(self) -> bool:
        """Restart Fast-LIO2 and localizer when the odometry publisher vanishes."""
        if not self._restart_recovery_lock.acquire(blocking=False):
            logger.warning(
                "Localization recovery: restart already in progress; "
                "skipping duplicate chain restart"
            )
            return False
        try:
            return self._restart_localization_chain_for_recovery_locked()
        finally:
            self._restart_recovery_lock.release()

    def _restart_localization_chain_for_recovery_locked(self) -> bool:
        """Restart Fast-LIO2/localizer after caller acquired recovery lock."""
        try:
            restart_start_mono = _time.monotonic()
            restart_start_wall = _time.time()

            svc = get_service_manager()
            svc.stop("localizer")
            svc.stop("slam")
            self._ensure_recovery_services(svc, "slam")
            if not self._wait_ros_topic_publishers(self._odom_topic, timeout=30.0):
                logger.error(
                    "Localization recovery: %s did not regain a publisher",
                    self._odom_topic,
                )
                self._ensure_localizer_service_best_effort(svc)
                return False
            if not self._wait_for_odom_sample_since(
                restart_start_mono, timeout=10.0
            ):
                logger.error(
                    "Localization recovery: %s publisher returned but no fresh "
                    "odometry sample reached SlamBridge",
                    self._odom_topic,
                )
                self._ensure_localizer_service_best_effort(svc)
                return False
            self._ensure_recovery_services(svc, "localizer")
            if not self._wait_ros_topic_publishers(
                TOPICS.localization_health, timeout=30.0
            ):
                logger.error(
                    "Localization recovery: %s did not regain a publisher",
                    TOPICS.localization_health,
                )
                self._ensure_localizer_service_best_effort(svc)
                return False
            if not self._wait_for_localizer_health_since(
                restart_start_wall, timeout=10.0
            ):
                logger.error(
                    "Localization recovery: %s publisher returned but no fresh "
                    "health sample reached SlamBridge",
                    TOPICS.localization_health,
                )
                self._ensure_localizer_service_best_effort(svc)
                return False
            logger.warning(
                "Localization recovery: Fast-LIO2 + localizer chain restarted"
            )
            return True
        except Exception as e:
            logger.warning(
                "Localization recovery: service_manager chain restart failed: %s",
                e,
            )
            return self._restart_localization_chain_systemctl_fallback()

    def _restart_localization_chain_systemctl_fallback(self) -> bool:
        commands = (
            ["sudo", "systemctl", "stop", "robot-localizer.service"],
            ["sudo", "systemctl", "stop", "robot-fastlio2.service"],
            ["sudo", "systemctl", "start", "robot-fastlio2.service"],
        )
        try:
            restart_start_mono = _time.monotonic()
            restart_start_wall = _time.time()
            for cmd in commands:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=15,
                    check=False,
                )
                if result.returncode != 0:
                    logger.error(
                        "Localization recovery fallback command failed (%s): %s",
                        " ".join(cmd),
                        (result.stderr or result.stdout or "").strip(),
                    )
                    self._ensure_localizer_service_best_effort()
                    return False
            if not self._wait_ros_topic_publishers(self._odom_topic, timeout=30.0):
                logger.error(
                    "Localization recovery fallback: %s did not regain a publisher",
                    self._odom_topic,
                )
                self._ensure_localizer_service_best_effort()
                return False
            if not self._wait_for_odom_sample_since(
                restart_start_mono, timeout=10.0
            ):
                logger.error(
                    "Localization recovery fallback: %s publisher returned but no "
                    "fresh odometry sample reached SlamBridge",
                    self._odom_topic,
                )
                self._ensure_localizer_service_best_effort()
                return False
            result = subprocess.run(
                ["sudo", "systemctl", "start", "robot-localizer.service"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=15,
                check=False,
            )
            if result.returncode != 0:
                logger.error(
                    "Localization recovery fallback command failed "
                    "(start robot-localizer.service): %s",
                    (result.stderr or result.stdout or "").strip(),
                )
                return False
            if not self._wait_ros_topic_publishers(
                TOPICS.localization_health, timeout=30.0
            ):
                logger.error(
                    "Localization recovery fallback: %s did not regain a publisher",
                    TOPICS.localization_health,
                )
                self._ensure_localizer_service_best_effort()
                return False
            if not self._wait_for_localizer_health_since(
                restart_start_wall, timeout=10.0
            ):
                logger.error(
                    "Localization recovery fallback: %s publisher returned but no "
                    "fresh health sample reached SlamBridge",
                    TOPICS.localization_health,
                )
                self._ensure_localizer_service_best_effort()
                return False
            logger.warning(
                "Localization recovery fallback: Fast-LIO2 + localizer chain "
                "restarted"
            )
            return True
        except Exception as e:
            logger.error("Localization recovery fallback failed: %s", e)
            return False

    def _localizer_odom_loss_recovery_due(
        self,
        *,
        backend: str,
        localizer_topic_fresh: bool,
        odom_age: float,
        now: float,
        now_mono: float,
    ) -> bool:
        if not self._localizer_odom_loss_recovery_enabled:
            return False
        if backend != "localizer":
            return False
        if self._localizer_odom_loss_recovery_inflight:
            return False
        if not localizer_topic_fresh:
            return False
        if self._localizer_health not in LOCALIZER_ODOM_GRACE_HEALTH:
            return False
        if self._last_odom_time <= 0.0:
            return False
        startup_age = max(0.0, now_mono - self._watchdog_start_mono)
        if startup_age < self._localizer_odom_loss_recovery_s:
            return False
        if self._last_localizer_odom_loss_recovery_ts > 0.0:
            elapsed = now - self._last_localizer_odom_loss_recovery_ts
            if elapsed < self._localizer_odom_loss_recovery_cooldown_s:
                return False
        if self._last_odom_time > 0.0:
            loss_age = odom_age
        else:
            loss_age = max(0.0, now_mono - self._watchdog_start_mono)
        return loss_age >= self._localizer_odom_loss_recovery_s

    def _start_localizer_odom_loss_recovery(self, *, now: float) -> None:
        now_mono = _time.monotonic()
        self._last_localizer_odom_loss_recovery_ts = now
        self._localizer_odom_loss_recovery_inflight = True
        self._localizer_odom_loss_recovery_start_mono = now_mono
        self._localizer_odom_loss_recovery_start_wall = now
        self._localizer_odom_loss_recovery_waiting_new_odom = True
        self._last_recovery_signal = LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL
        self._last_recovery_action = LOCALIZER_ODOM_LOSS_RECOVERY_ACTION
        self._last_recovery_ts = now
        self._relocalization_state = "restarting"
        threading.Thread(
            target=self._run_localizer_odom_loss_recovery,
            daemon=True,
            name="localizer-odom-loss-recovery",
        ).start()

    def _run_localizer_odom_loss_recovery(self) -> None:
        try:
            ok = self._restart_localization_chain_for_recovery()
            self._relocalization_state = "restarted" if ok else "failed"
        finally:
            self._localizer_odom_loss_recovery_inflight = False

    def _auto_relocalize(self) -> None:
        """Trigger relocalize via ROS2 service using last known good position.

        If no good position was ever recorded (SLAM diverged from startup),
        falls back to origin (0, 0, 0).
        """
        profile = self._current_backend_profile()
        contract = self._backend_contract(profile)
        pos = self._drift_last_good_pos
        yaw = self._drift_last_good_yaw
        if pos is None:
            logger.warning("Drift guard: no good position recorded, relocalize to origin")
            pos = np.zeros(3)
            yaw = 0.0

        map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
        pcd_path = os.path.join(map_dir, "active", "map.pcd")
        if not contract["relocalization_supported"]:
            self._relocalization_state = "unsupported"
            self._last_recovery_action = str(contract["recovery_action"])
            self._last_recovery_ts = _time.time()
            logger.error(
                "Drift guard: %s cannot use relocalize (supported=%s); running %s",
                profile,
                contract["relocalization_supported"],
                contract["recovery_action"],
            )
            if self._restart_backend_for_recovery(str(contract["backend"])):
                self._drift_bad_count = 0
                self._drift_last_good_pos = None  # reset so first new frame becomes anchor
                self._relocalization_state = "restarted"
            return
        if not os.path.isfile(pcd_path):
            # No active map to relocalize against (mapping mode or never saved).
            # Fallback: restart the SLAM systemd service to clear the IEKF
            # state 锟?Fast-LIO2 starts fresh at the robot's current pose.
            # This costs ~3s of blank odometry but recovers from divergence.
            logger.error(
                "Drift guard: no active map PCD (%s) 锟?falling back to "
                "systemctl restart slam to clear IEKF state",
                pcd_path)
            try:
                slam_result = subprocess.run(
                    ["sudo", "systemctl", "restart", "slam"],
                    capture_output=True, text=True, encoding="utf-8",
                    errors="replace", timeout=15,
                    check=False,
                )
                if slam_result.returncode != 0:
                    logger.warning(
                        "Drift guard: slam restart returned %d: %s",
                        slam_result.returncode,
                        (slam_result.stderr or slam_result.stdout or "").strip()[:200],
                    )
                else:
                    logger.warning("Drift guard: slam service restarted")
                self._drift_bad_count = 0
                self._drift_last_good_pos = None  # reset so first new frame becomes anchor
            except Exception as e:
                logger.error("Drift guard: slam restart failed: %s", e)
            return

        self._relocalization_state = "requested"
        self._last_recovery_action = "relocalize_service"
        self._last_recovery_ts = _time.time()
        logger.warning(
            "Drift guard: auto-relocalize to (%.2f, %.2f, yaw=%.2f) using %s",
            pos[0], pos[1], yaw, pcd_path)
        try:
            result = relocalize_saved_map(
                pcd_path,
                float(pos[0]),
                float(pos[1]),
                float(yaw),
                timeout_s=30.0,
            )
            if result.timed_out:
                raise TimeoutError(result.message)
            if result.returncode != 0 or not result.success:
                logger.warning(
                    "Drift guard: relocalize failed result rc=%s success=%s: %s",
                    result.returncode,
                    result.success,
                    (result.message or result.stderr or result.stdout or "").strip()[:200],
                )
                self._relocalization_state = "failed"
                return
            else:
                logger.info("Drift guard: relocalize call completed")
            self._drift_bad_count = 0
            self._relocalization_state = "completed"
        except Exception as e:
            logger.warning("Drift guard: relocalize failed: %s", e)
            self._relocalization_state = "failed"

    def trigger_global_relocalize(
        self,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        return trigger_global_relocalize(timeout_s=timeout_s)

    def relocalize_saved_map(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 30.0,
    ) -> RelocalizationResult:
        return relocalize_saved_map(pcd_path, x, y, yaw, timeout_s=timeout_s)

    def relocalize_saved_map_with_env(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 20.0,
        base_env: Mapping[str, str] | None = None,
    ) -> RelocalizationResult:
        return relocalize_saved_map_with_env(
            pcd_path,
            x,
            y,
            yaw,
            timeout_s=timeout_s,
            base_env=base_env,
        )

    def _mark_odom_received(
        self,
        *,
        wall_now: float | None = None,
        mono_now: float | None = None,
    ) -> None:
        wall = _time.time() if wall_now is None else wall_now
        mono = _time.monotonic() if mono_now is None else mono_now
        self._last_odom_time = wall
        self._last_odom_mono = mono
        if (
            self._localizer_odom_loss_recovery_waiting_new_odom
            and self._localizer_odom_loss_recovery_start_mono > 0.0
            and mono >= self._localizer_odom_loss_recovery_start_mono
        ):
            self._localizer_odom_loss_recovery_waiting_new_odom = False
        self._odom_recv_ts.append(wall)

    def _mark_cloud_received(
        self,
        *,
        wall_now: float | None = None,
        mono_now: float | None = None,
    ) -> None:
        self._last_cloud_time = _time.time() if wall_now is None else wall_now
        self._last_cloud_mono = _time.monotonic() if mono_now is None else mono_now

    def _odom_age_s(self, *, mono_now: float | None = None) -> float:
        if self._last_odom_mono > 0.0:
            now = _time.monotonic() if mono_now is None else mono_now
            return max(0.0, now - self._last_odom_mono)
        if self._last_odom_time > 0.0:
            return max(0.0, _time.time() - self._last_odom_time)
        return float("inf")

    def _cloud_age_s(self, *, mono_now: float | None = None) -> float:
        if self._last_cloud_mono > 0.0:
            now = _time.monotonic() if mono_now is None else mono_now
            return max(0.0, now - self._last_cloud_mono)
        if self._last_cloud_time > 0.0:
            return max(0.0, _time.time() - self._last_cloud_time)
        return float("inf")

    def _backend_contract(self, profile: str | None = None) -> dict[str, Any]:
        """Expose backend capabilities for Gateway/session consumers."""
        return slam_backend_contract(profile or self._backend_profile or "bridge")

    def _backend_health_fields(
        self,
        *,
        profile: str | None = None,
        contract: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        configured = str(self._backend_profile or "bridge").strip().lower() or "bridge"
        if configured == "bridge":
            configured_backend = "bridge"
        else:
            configured_backend = str(self._backend_contract(configured)["backend"])
        effective_contract = contract or self._backend_contract(
            profile or self._current_backend_profile()
        )
        effective_backend = str(effective_contract["backend"])
        status = BackendStatus.configured_as(configured_backend)
        if configured_backend == "bridge":
            status.use(effective_backend, degraded=False)
        elif effective_backend != configured_backend:
            status.use(
                effective_backend,
                reason=(
                    f"effective backend '{effective_backend}' differs from "
                    f"configured backend '{configured_backend}'"
                ),
            )
        return status.as_health_fields()

    def _localizer_health_topic_fresh(self, now: float) -> bool:
        return (
            self._localizer_health_ts > 0.0
            and (now - self._localizer_health_ts) <= self._localizer_health_timeout
        )

    def _localizer_health_allows_odom_grace(
        self,
        backend: str,
        localizer_topic_fresh: bool,
    ) -> bool:
        return (
            backend == "localizer"
            and localizer_topic_fresh
            and self._localizer_health in LOCALIZER_ODOM_GRACE_HEALTH
        )

    def _effective_odom_timeout(self, *, localizer_health_grace: bool) -> float:
        if not localizer_health_grace:
            return self._odom_timeout
        return max(self._odom_timeout, self._localizer_health_odom_grace_s)

    @staticmethod
    def _synthesized_localizer_health(state: str) -> str:
        if state == LOC_TRACKING:
            return "LIO_TRACKING"
        if state in {LOC_DEGRADED, LOC_FALLBACK_GNSS_ONLY}:
            return "LIO_DEGRADED"
        if state == LOC_DIVERGED:
            return "LIO_DIVERGED"
        if state == LOC_LOST:
            return "LIO_LOST"
        return "UNKNOWN"

    def _current_backend_profile(self) -> str:
        """Best-effort running backend detection for status and recovery."""
        configured = str(self._backend_profile or "bridge").strip().lower()
        if configured not in {"", "bridge"}:
            return configured
        if os.name == "nt" or not os.path.isdir("/run/systemd/system"):
            return configured or "bridge"
        now = _time.monotonic()
        if (
            self._backend_detect_mono > 0.0
            and now - self._backend_detect_mono < self._backend_detect_interval
        ):
            return self._backend_detect_cache or configured or "bridge"
        self._backend_detect_mono = now
        try:
            services = get_service_manager().status(
                "super_lio_relocation",
                "super_lio",
                "slam_pgo",
                "localizer",
                "slam",
            )
            if services.get("super_lio_relocation") in ("running", "active"):
                self._backend_detect_cache = "super_lio_relocation"
                return "super_lio_relocation"
            if services.get("super_lio") in ("running", "active"):
                self._backend_detect_cache = "super_lio"
                return "super_lio"
            if services.get("localizer") in ("running", "active"):
                self._backend_detect_cache = "localizer"
                return "localizer"
            if services.get("slam_pgo") in ("running", "active"):
                self._backend_detect_cache = "fastlio2"
                return "fastlio2"
            if services.get("slam") in ("running", "active"):
                self._backend_detect_cache = "slam"
                return "slam"
        except Exception:
            pass
        self._backend_detect_cache = configured or "bridge"
        return configured or "bridge"

    def _on_dds_odom(self, msg) -> None:
        """DDS_Odometry 锟?Module Odometry (with selective visual fusion + drift guard)."""
        try:
            now = _time.time()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            slam_odom = Odometry(
                pose=Pose(
                    position=Vector3(x=p.x, y=p.y, z=p.z),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
                ),
                twist=Twist(
                    linear=Vector3(x=t.linear.x, y=t.linear.y, z=t.linear.z),
                    angular=Vector3(x=t.angular.x, y=t.angular.y, z=t.angular.z),
                ),
                ts=stamp,
                frame_id=self._frame_from_msg_header(
                    msg, default=topic_default_frame_id(TOPICS.odometry)),
                child_frame_id=self._child_frame_from_msg(
                    msg, default=topic_default_frame_id(TOPICS.cmd_vel)),
            )
            fused = self._fuse_odometry(slam_odom)

            # Drift guard: suppress diverged odometry, trigger auto-relocalize
            if self._check_drift(fused):
                return  # don't publish garbage to downstream modules

            # Localizer publishes odom-frame poses plus map->odom TF; Super-LIO
            # relocation already publishes its optimized world/map pose.
            fused = self._maybe_apply_map_odom_to_odometry(fused)
            # Mark receive freshness before downstream fan-out; slow
            # Gateway/SSE/App consumers must not look like an odom outage.
            self._mark_odom_received(wall_now=now)
            self._submit_odometry_worker(fused)
        except Exception as e:
            logger.debug("SlamBridge dds odom error: %s", e)

    def _on_dds_cloud(self, msg) -> None:
        self._submit_pointcloud_worker(lambda: self._process_dds_cloud(msg))

    def _process_dds_cloud(self, msg) -> None:
        """DDS_PointCloud2 锟?Module PointCloud2."""
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
            xyz_buf = raw[:, :12].copy()
            xyz = xyz_buf.view(np.float32).reshape(-1, 3)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                source_frame = self._cloud_frame_from_msg(
                    msg, default=topic_default_frame_id(TOPICS.odometry))
                xyz_map, frame_id = self._map_cloud_points_and_frame(xyz, source_frame)
                self._mark_cloud_received()
                self.map_cloud.publish(PointCloud2.from_numpy(xyz_map, frame_id=frame_id))
        except Exception as e:
            logger.debug("SlamBridge dds cloud error: %s", e)

    def _on_dds_saved_map(self, msg) -> None:
        self._submit_saved_map_worker(lambda: self._process_dds_saved_map(msg))

    def _process_dds_saved_map(self, msg) -> None:
        """DDS saved-map PointCloud2 锟?Module saved_map Out."""
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
            xyz_buf = raw[:, :12].copy()
            xyz = xyz_buf.view(np.float32).reshape(-1, 3)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                self.saved_map.publish(PointCloud2.from_numpy(
                    xyz, frame_id=topic_default_frame_id(TOPICS.saved_map_cloud)))
        except Exception as e:
            logger.debug("SlamBridge dds saved_map error: %s", e)

    def _submit_pointcloud_worker(self, task: Callable[[], None]) -> bool:
        """Run heavy point-cloud conversion outside the ROS executor.

        Odom freshness is control-critical; live point clouds are visualization
        and map-layer inputs that can tolerate dropped frames. Both the DDS
        reader loop and rclpy fallback can otherwise spend too long converting
        large PointCloud2 payloads inline and delay odom freshness updates.
        """
        if self._shutdown_event.is_set():
            return False
        if not self._pointcloud_worker_lock.acquire(blocking=False):
            self._pointcloud_worker_drops += 1
            return False

        def run() -> None:
            try:
                if not self._shutdown_event.is_set():
                    task()
            finally:
                self._pointcloud_worker_lock.release()

        self._pointcloud_worker_thread = threading.Thread(
            target=run,
            daemon=True,
            name="slam_bridge_pointcloud",
        )
        self._pointcloud_worker_thread.start()
        return True

    def _submit_saved_map_worker(self, task: Callable[[], None]) -> bool:
        if self._shutdown_event.is_set():
            return False
        if not self._saved_map_worker_lock.acquire(blocking=False):
            self._saved_map_worker_drops += 1
            return False

        def run() -> None:
            try:
                if not self._shutdown_event.is_set():
                    task()
            finally:
                self._saved_map_worker_lock.release()

        self._saved_map_worker_thread = threading.Thread(
            target=run,
            daemon=True,
            name="slam_bridge_saved_map",
        )
        self._saved_map_worker_thread.start()
        return True

    def _submit_odometry_worker(self, odom: Odometry) -> bool:
        """Publish odometry outside the ROS executor.

        Odometry freshness is marked before this method is called. Downstream
        consumers such as Gateway SSE or perception bookkeeping may be slower
        than the ROS receive cadence; in that case we keep the latest receive
        timestamp accurate and drop the downstream fan-out frame.
        """
        if self._shutdown_event.is_set():
            return False
        if not self._odom_worker_lock.acquire(blocking=False):
            self._odom_worker_drops += 1
            return False

        def run() -> None:
            try:
                if not self._shutdown_event.is_set():
                    self.odometry.publish(odom)
            finally:
                self._odom_worker_lock.release()

        self._odom_worker_thread = threading.Thread(
            target=run,
            daemon=True,
            name="slam_bridge_odometry",
        )
        self._odom_worker_thread.start()
        return True

    def _process_rclpy_cloud(self, msg) -> None:
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            if step == 16:
                pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                xyz = pts[:, :3].copy()
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
                xyz_buf = raw[:, :12].copy()
                xyz = xyz_buf.view(np.float32).reshape(-1, 3)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                source_frame = self._cloud_frame_from_msg(
                    msg, default=topic_default_frame_id(TOPICS.odometry))
                xyz_map, frame_id = self._map_cloud_points_and_frame(xyz, source_frame)
                self._mark_cloud_received()
                self.map_cloud.publish(PointCloud2.from_numpy(xyz_map, frame_id=frame_id))
        except Exception as e:
            logger.debug("SlamBridge rclpy cloud error: %s", e)

    def _process_rclpy_saved_map(self, msg) -> None:
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            if step == 16:
                pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                xyz = pts[:, :3].copy()
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
                xyz_buf = raw[:, :12].copy()
                xyz = xyz_buf.view(np.float32).reshape(-1, 3)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                self.saved_map.publish(PointCloud2.from_numpy(
                    xyz, frame_id=topic_default_frame_id(TOPICS.saved_map_cloud)))
        except Exception as e:
            logger.debug("SlamBridge rclpy saved_map error: %s", e)

    def _apply_map_odom_to_points(self, xyz: np.ndarray) -> np.ndarray:
        """Transform an (N, 3) point cloud from odom frame to map frame using
        the cached map閳姪dom TF. Returns the same array if TF not yet cached.
        Vectorized 锟?avoids a per-point loop for tens of thousands of points.
        """
        if xyz.shape[0] == 0:
            return xyz
        try:
            return self._ensure_frame_tree().transform_points(
                xyz,
                topic_default_frame_id(TOPICS.map_cloud),
                topic_default_frame_id(TOPICS.odometry),
            ).astype(xyz.dtype, copy=False)
        except Exception:
            pass
        T = getattr(self, "_T_map_odom", None)
        if T is None or xyz.shape[0] == 0:
            return xyz
        R = T[:3, :3].astype(np.float32)
        t = T[:3, 3].astype(np.float32)
        return (xyz @ R.T) + t

    def _cloud_frame_from_msg(self, msg, *, default: str) -> str:
        return self._frame_from_msg_header(msg, default=default)

    def _frame_from_msg_header(self, msg, *, default: str) -> str:
        header = getattr(msg, "header", None)
        frame_id = normalize_frame_id(getattr(header, "frame_id", None))
        return frame_id or normalize_frame_id(default) or topic_default_frame_id(
            TOPICS.odometry)

    def _child_frame_from_msg(self, msg, *, default: str) -> str:
        child_frame_id = normalize_frame_id(getattr(msg, "child_frame_id", None))
        return child_frame_id or normalize_frame_id(default) or topic_default_frame_id(
            TOPICS.cmd_vel)

    def _map_cloud_points_and_frame(
        self,
        xyz: np.ndarray,
        source_frame: str,
    ) -> tuple[np.ndarray, str]:
        """Return map-cloud points without relabeling odom data as map data."""

        normalized_source = (
            normalize_frame_id(source_frame)
            or topic_default_frame_id(TOPICS.odometry)
        )
        if (
            normalized_source == topic_default_frame_id(TOPICS.odometry)
            and getattr(self, "_T_map_odom", None) is not None
            and self._should_apply_map_odom_tf()
        ):
            return (
                self._apply_map_odom_to_points(xyz),
                topic_default_frame_id(TOPICS.map_cloud),
            )
        return xyz, normalized_source

    def _should_apply_map_odom_tf(self) -> bool:
        """Return whether incoming SLAM data needs localizer map->odom lift.

        The localizer publishes odometry in odom frame plus a map->odom TF. In
        contrast, Super-LIO relocation publishes its optimized world/map pose
        directly after ICP. Applying a cached localizer TF to Super-LIO outputs
        double-transforms the pose and shows up as a stationary jump.
        """
        configured = str(getattr(self, "_backend_profile", "bridge") or "bridge").strip().lower()
        if configured and configured != "bridge":
            backend = configured
        else:
            backend = str(
                getattr(self, "_backend_detect_cache", configured or "bridge") or "bridge"
            ).strip().lower()
        return backend in MAP_ODOM_TF_BACKENDS

    def _map_odom_tf_fresh(self) -> bool:
        """True if the cached map->odom TF is recent enough to trust.

        A TTL <= 0 disables the gate (always fresh). A cached TF older than
        the TTL is treated as stale: lifting odom with a frozen TF would
        report a confident-but-wrong map pose, so callers fall back to the
        untransformed odom-frame pose instead.
        """
        ttl = getattr(self, "_map_odom_tf_ttl_s", 0.0)
        if ttl <= 0.0:
            return True
        stamped = getattr(self, "_T_map_odom_mono", 0.0)
        if stamped <= 0.0:
            return False
        return (_time.monotonic() - stamped) <= ttl

    def _ensure_frame_tree(self) -> FrameTree:
        tree = getattr(self, "_frame_tree", None)
        if tree is None:
            tree = FrameTree()
            self._frame_tree = tree
        return tree

    def _maybe_apply_map_odom_to_points(self, xyz: np.ndarray) -> np.ndarray:
        if getattr(self, "_T_map_odom", None) is None:
            return xyz
        if not self._should_apply_map_odom_tf():
            return xyz
        if not self._map_odom_tf_fresh():
            return xyz
        return self._apply_map_odom_to_points(xyz)

    def _maybe_apply_map_odom_to_odometry(self, odom: Odometry) -> Odometry:
        odom.frame_id = (
            normalize_frame_id(getattr(odom, "frame_id", None))
            or topic_default_frame_id(TOPICS.odometry)
        )
        odom.child_frame_id = (
            normalize_frame_id(getattr(odom, "child_frame_id", None))
            or topic_default_frame_id(TOPICS.cmd_vel)
        )
        try:
            self._ensure_frame_tree().update_odometry(odom)
        except Exception as e:
            logger.debug("SlamBridge frame-tree odometry update failed: %s", e)
        if getattr(self, "_T_map_odom", None) is None:
            return odom
        if odom.frame_id != topic_default_frame_id(TOPICS.odometry):
            return odom
        if not self._should_apply_map_odom_tf():
            return odom
        if not self._map_odom_tf_fresh():
            return odom
        return self._apply_map_odom_to_odometry(odom)

    def _apply_map_odom_to_odometry(self, odom: Odometry) -> Odometry:
        """Transform odom from Fast-LIO2 odom frame into map frame using the
        localizer-emitted map閳姪dom TF. No-op when TF is not yet cached.
        Composes both translation and rotation 锟?forgetting the rotation
        makes map.yaw == odom.yaw which is wrong whenever the BBS3D align
        produced a non-zero yaw (very common, e.g. -104锟?here).
        """
        try:
            odom.pose = self._ensure_frame_tree().transform_pose(
                odom.pose,
                topic_default_frame_id(TOPICS.map_cloud),
                topic_default_frame_id(TOPICS.odometry),
            )
            odom.frame_id = topic_default_frame_id(TOPICS.map_cloud)
            return odom
        except Exception:
            pass
        T = getattr(self, "_T_map_odom", None)
        if T is None:
            return odom
        # Position: map_p = T_map_odom * odom_p
        p = odom.pose.position
        ox, oy, oz = float(p.x), float(p.y), float(p.z)
        odom.pose.position.x = float(T[0, 0] * ox + T[0, 1] * oy + T[0, 2] * oz + T[0, 3])
        odom.pose.position.y = float(T[1, 0] * ox + T[1, 1] * oy + T[1, 2] * oz + T[1, 3])
        odom.pose.position.z = float(T[2, 0] * ox + T[2, 1] * oy + T[2, 2] * oz + T[2, 3])
        # Rotation: map_R_body = map_R_odom * odom_R_body
        # Convert odom quat 锟?3x3, multiply, convert back to quat.
        q = odom.pose.orientation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wzz = qw * qx, qw * qy, qw * qz
        R_body = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wzz),    2 * (xz + wy)],
            [    2 * (xy + wzz), 1 - 2 * (xx + zz),    2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ], dtype=np.float64)
        R_map = T[:3, :3] @ R_body
        # 3x3 锟?quat (Shepperd-style, stable for any rotation)
        tr = R_map[0, 0] + R_map[1, 1] + R_map[2, 2]
        if tr > 0:
            S = 2.0 * np.sqrt(tr + 1.0)
            nw = 0.25 * S
            nx = (R_map[2, 1] - R_map[1, 2]) / S
            ny = (R_map[0, 2] - R_map[2, 0]) / S
            nz = (R_map[1, 0] - R_map[0, 1]) / S
        elif (R_map[0, 0] > R_map[1, 1]) and (R_map[0, 0] > R_map[2, 2]):
            S = 2.0 * np.sqrt(1.0 + R_map[0, 0] - R_map[1, 1] - R_map[2, 2])
            nw = (R_map[2, 1] - R_map[1, 2]) / S
            nx = 0.25 * S
            ny = (R_map[0, 1] + R_map[1, 0]) / S
            nz = (R_map[0, 2] + R_map[2, 0]) / S
        elif R_map[1, 1] > R_map[2, 2]:
            S = 2.0 * np.sqrt(1.0 + R_map[1, 1] - R_map[0, 0] - R_map[2, 2])
            nw = (R_map[0, 2] - R_map[2, 0]) / S
            nx = (R_map[0, 1] + R_map[1, 0]) / S
            ny = 0.25 * S
            nz = (R_map[1, 2] + R_map[2, 1]) / S
        else:
            S = 2.0 * np.sqrt(1.0 + R_map[2, 2] - R_map[0, 0] - R_map[1, 1])
            nw = (R_map[1, 0] - R_map[0, 1]) / S
            nx = (R_map[0, 2] + R_map[2, 0]) / S
            ny = (R_map[1, 2] + R_map[2, 1]) / S
            nz = 0.25 * S
        odom.pose.orientation.x = float(nx)
        odom.pose.orientation.y = float(ny)
        odom.pose.orientation.z = float(nz)
        odom.pose.orientation.w = float(nw)
        odom.frame_id = topic_default_frame_id(TOPICS.map_cloud)
        return odom

    def _cache_map_odom_tf(self, tx, ty, tz, qx, qy, qz, qw, ts: float | None = None):
        """Cache map閳姪dom as 4x4 for downstream variance-free transforms.

        Also detects discontinuities (PGO optimisation, BBS3D relocalisation)
        and emits map_frame_jump_event so Navigation + costmap modules
        can replan / clear cached state. Jump thresholds default to 1.0 m
        translation or 30锟?rotation; both configurable via kw.

        Why hand-rolled
        ---------------
        ROS2 / tf2_ros has no native "transform discontinuity" notification.
        The tf2 Buffer API only checks connectivity (canTransform) and
        timeouts (transform_tolerance). Cartographer ROS, LIO-SAM, and
        FAST-LIO-LOCALIZATION all push the responsibility of detecting and
        reacting to PGO/relocalize-induced map閳攼dom jumps onto downstream
        consumers 锟?that is the deliberate, industry-standard pattern. The
        Cartographer authors specifically reject smoothing the jump in the
        publisher, because the jump itself is the optimised correction.
        Hand-rolling this 60-line check at the consumer (here, SlamBridge)
        is therefore the *only* canonical option."""
        tf_ts = float(ts) if ts is not None else _time.time()
        qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        try:
            self._ensure_frame_tree().set_transform(
                Transform(
                    translation=Vector3(tx, ty, tz),
                    rotation=Quaternion(qx, qy, qz, qw),
                    frame_id=topic_default_frame_id(TOPICS.map_cloud),
                    child_frame_id=topic_default_frame_id(TOPICS.odometry),
                    ts=tf_ts,
                )
            )
        except Exception as e:
            logger.debug("SlamBridge frame-tree map->odom update failed: %s", e)

        # 閳光偓閳光偓 Jump detection 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓
        # Compare against the previous cached T. First call sets baseline only.
        prev_T = getattr(self, "_T_map_odom", None)
        if prev_T is not None:
            dt_vec = T[:3, 3] - prev_T[:3, 3]
            dt_norm = float(np.linalg.norm(dt_vec))
            # Rotation angle between two rotation matrices = arccos((tr(R_rel) - 1) / 2)
            R_rel = T[:3, :3] @ prev_T[:3, :3].T
            tr = float(np.clip((np.trace(R_rel) - 1.0) * 0.5, -1.0, 1.0))
            dyaw_rad = float(np.arccos(tr))
            dyaw_deg = dyaw_rad * 180.0 / np.pi
            jump_t_thresh = getattr(self, "_jump_t_threshold_m", 1.0)
            jump_r_thresh = getattr(self, "_jump_r_threshold_deg", 30.0)
            if dt_norm > jump_t_thresh or dyaw_deg > jump_r_thresh:
                logger.warning(
                    "map閳攼dom TF JUMPED: |铻杢|=%.2fm 铻杫aw=%.1f锟?"
                    "(prev=[%.2f,%.2f,%.2f] 锟?new=[%.2f,%.2f,%.2f])",
                    dt_norm, dyaw_deg,
                    prev_T[0, 3], prev_T[1, 3], prev_T[2, 3],
                    T[0, 3], T[1, 3], T[2, 3])
                try:
                    self.map_frame_jump_event.publish({
                        "ts": _time.time(),
                        "dt_m": round(dt_norm, 4),
                        "dyaw_deg": round(dyaw_deg, 2),
                        "prev_xyz": [float(prev_T[0, 3]), float(prev_T[1, 3]),
                                     float(prev_T[2, 3])],
                        "new_xyz": [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])],
                    })
                except Exception as e:
                    logger.debug("map_frame_jump_event publish failed: %s", e)

        self._T_map_odom = T
        self._T_map_odom_mono = _time.monotonic()
        configured = str(getattr(self, "_backend_profile", "bridge") or "bridge").strip().lower()
        cached = str(
            getattr(self, "_backend_detect_cache", configured or "bridge") or "bridge"
        ).strip().lower()
        if configured in {"", "bridge"} and cached in {"", "bridge"}:
            self._backend_detect_cache = "localizer"

    def _on_rclpy_tf(self, msg) -> None:
        try:
            for t in msg.transforms:
                parent = normalize_frame_id(t.header.frame_id)
                child = normalize_frame_id(t.child_frame_id)
                if (
                    parent == topic_default_frame_id(TOPICS.map_cloud)
                    and child == topic_default_frame_id(TOPICS.odometry)
                ):
                    transform = transform_from_stamped(t)
                    tr = transform.translation
                    q = transform.rotation
                    self._cache_map_odom_tf(
                        float(tr.x), float(tr.y), float(tr.z),
                        float(q.x), float(q.y), float(q.z), float(q.w),
                        ts=transform.ts)
                    self.map_odom_tf.publish({
                        "tx": float(tr.x), "ty": float(tr.y), "tz": float(tr.z),
                        "qx": float(q.x), "qy": float(q.y),
                        "qz": float(q.z), "qw": float(q.w),
                        "ts": transform.ts,
                        "valid": True,
                    })
                    return
        except Exception as e:
            logger.debug("SlamBridge rclpy tf error: %s", e)

    def _on_dds_tf(self, msg) -> None:
        """DDS /tf TFMessage 锟?extract map閳姪dom, relay as dict to map_odom_tf Out.

        /tf is fan-in from every broadcaster, so we walk transforms and pick
        the specific (parent=map, child=odom) pair the localizer publishes.
        Downstream GatewayModule applies this to odom-frame point clouds so
        the browser sees saved_map + map_cloud overlaid correctly.
        """
        try:
            transforms = getattr(msg, "transforms", None) or []
            for t in transforms:
                parent = normalize_frame_id(getattr(t.header, "frame_id", None))
                child = normalize_frame_id(getattr(t, "child_frame_id", None))
                if (
                    parent == topic_default_frame_id(TOPICS.map_cloud)
                    and child == topic_default_frame_id(TOPICS.odometry)
                ):
                    transform = transform_from_stamped(t)
                    trans = transform.translation
                    rot = transform.rotation
                    tf_msg = {
                        "tx": float(trans.x), "ty": float(trans.y), "tz": float(trans.z),
                        "qx": float(rot.x),   "qy": float(rot.y),
                        "qz": float(rot.z),   "qw": float(rot.w),
                        "ts": transform.ts,
                        "valid": True,
                    }
                    self._cache_map_odom_tf(
                        tf_msg["tx"], tf_msg["ty"], tf_msg["tz"],
                        tf_msg["qx"], tf_msg["qy"], tf_msg["qz"], tf_msg["qw"],
                        ts=tf_msg["ts"])
                    self.map_odom_tf.publish(tf_msg)
                    return
        except Exception as e:
            logger.debug("SlamBridge dds tf error: %s", e)

    # 閳光偓閳光偓 rclpy callbacks (fallback) 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

    def _on_rclpy_odom(self, msg) -> None:
        try:
            now = _time.time()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            slam_odom = Odometry(
                pose=Pose(
                    position=Vector3(x=float(p.x), y=float(p.y), z=float(p.z)),
                    orientation=Quaternion(x=float(q.x), y=float(q.y), z=float(q.z), w=float(q.w)),
                ),
                twist=Twist(
                    linear=Vector3(x=float(t.linear.x), y=float(t.linear.y), z=float(t.linear.z)),
                    angular=Vector3(x=float(t.angular.x), y=float(t.angular.y), z=float(t.angular.z)),
                ),
                frame_id=self._frame_from_msg_header(
                    msg, default=topic_default_frame_id(TOPICS.odometry)),
                child_frame_id=self._child_frame_from_msg(
                    msg, default=topic_default_frame_id(TOPICS.cmd_vel)),
            )
            # Track max position covariance from IESKF P matrix (filled by lio_node.cpp)
            cov = msg.pose.covariance  # 36-element row-major 6x6
            if len(cov) >= 15:
                self._max_pos_cov = max(float(cov[0]), float(cov[7]), float(cov[14]))

            fused = self._fuse_odometry(slam_odom)
            # Keep the rclpy fallback under the same drift guard as DDS. Live
            # sim and desktop setups often use rclpy, so a diverged LIO pose
            # must not reach navigation just because it bypassed CycloneDDS.
            if self._check_drift(fused):
                return
            # Localizer publishes odom-frame poses plus map->odom TF; Super-LIO
            # relocation already publishes its optimized world/map pose.
            fused = self._maybe_apply_map_odom_to_odometry(fused)
            self._mark_odom_received(wall_now=now)
            self._submit_odometry_worker(fused)
        except Exception as e:
            logger.warning("SlamBridge rclpy odom error: %s", e)

    def _on_rclpy_cloud(self, msg) -> None:
        self._submit_pointcloud_worker(lambda: self._process_rclpy_cloud(msg))

    def _on_rclpy_saved_map(self, msg) -> None:
        self._submit_saved_map_worker(lambda: self._process_rclpy_saved_map(msg))

    # 閳光偓閳光偓 Degeneracy metric callbacks 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

    def _on_rclpy_quality(self, msg) -> None:
        """ICP fitness score from localizer (lower = better)."""
        val = float(msg.data)
        self._icp_fitness = val
        self.localization_quality.publish(val)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _on_rclpy_degeneracy(self, msg) -> None:
        """Effective feature ratio from SLAM (1.0 = all features valid)."""
        self._effective_ratio = float(msg.data)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _on_rclpy_localization_health(self, msg) -> None:
        """Multi-frame confirmed localizer health from /slam/localization_health.

        Payload format (R4-extended, backward-compatible):
            "<state>|fitness=<v>|iter=<n>|cov=<v>"
        produced by LocalizerNode::updateAndPublishHealth(). The leading
        "<state>|fitness=..." prefix is the original P3 contract; iter and
        cov are R4 additions sourced from small_gicp's RegistrationResult
        and Hessian respectively. Unknown keys (future fields) are skipped
        without raising so older robots speaking the v1 payload still
        update state correctly.
        """
        try:
            payload = str(msg.data)
            parts = payload.split("|")
            self._localizer_health = (parts[0].strip().upper() if parts else "") or "UNKNOWN"
            for kv in parts[1:]:
                key, _, val = kv.partition("=")
                key = key.strip().lower()
                if not val:
                    continue
                if key == "fitness":
                    try:
                        self._localizer_health_fitness = float(val)
                    except ValueError:
                        pass
                elif key == "iter":
                    try:
                        self._localizer_health_iter = int(float(val))
                    except ValueError:
                        pass
                elif key == "cov":
                    try:
                        self._localizer_health_cov_trace = float(val)
                    except ValueError:
                        pass
                # Other keys silently ignored 锟?forward-compat for future
                # localizer payload extensions.
            self._localizer_health_ts = _time.time()
            logger.info(
                "Localizer health 锟?%s (fitness=%.4f iter=%d cov=%.4f)",
                self._localizer_health, self._localizer_health_fitness,
                self._localizer_health_iter, self._localizer_health_cov_trace)
        except Exception as e:
            logger.debug("localization_health parse failed: %s", e)

    def _on_rclpy_degeneracy_detail(self, msg) -> None:
        """Detailed degeneracy metrics from Hessian eigenvalue analysis.

        Float32MultiArray, original 11-float layout extended to 14:
          [0]  condition_number
          [1]  min_eigenvalue
          [2]  max_eigenvalue
          [3]  effective_ratio
          [4]  degenerate_dof_count
          [5..10] dof_mask (6 DOFs)
          [11] pos_cov_trace (m锟?        锟?IEKF position covariance trace (NEW)
          [12] iter_num                   锟?IEKF iterations actually used (NEW)
          [13] converged (1.0 / 0.0)      锟?whether stop_func fired (NEW)
        Older publishers send only 11 floats; the extra fields are read with len guard.
        """
        d = msg.data
        if len(d) < 11:
            return
        self._condition_number = float(d[0])
        self._eigenvalue_ratio = float(d[0])  # condition_number = max_eig / min_eig
        self._effective_ratio = float(d[3])
        self._degenerate_dof_count = int(d[4])
        self._dof_mask = np.array([float(d[5 + i]) for i in range(6)])
        if len(d) >= 14:
            self._pos_cov_trace = float(d[11])
            self._ieskf_iter_num = int(d[12])
            self._ieskf_converged = bool(d[13] >= 0.5)
        # Reconstruct eigenvalues from mask (approximate; mask gives 0/1 per DOF)
        max_eig = float(d[2])
        min_eig = float(d[1])
        self._eigenvalues = np.array([min_eig if self._dof_mask[i] < 0.5 else max_eig
                                      for i in range(6)])

        # SEVERE early warning 锟?throttled to once per 30s
        now = _time.time()
        if self._condition_number > 1e6 and (now - self._last_severe_warn) > 30.0:
            self._last_severe_warn = now
            logger.warning(
                "SEVERE SLAM degeneracy: cond=%.2e, effective_ratio=%.3f, degen_dofs=%d 锟?"
                "covariance growth likely; consider stopping or relocating robot",
                self._condition_number, self._effective_ratio, self._degenerate_dof_count)

        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _update_degeneracy_level(self) -> None:
        """Classify degeneracy severity from SLAM quality metrics.

        Uses multiple signals:
          - ICP fitness score (localizer quality)
          - effective_ratio (Hessian-based, from C++ eigenvalue analysis)
          - condition_number (max_eig / min_eig, from degeneracy_detail)
          - degenerate_dof_count (how many of 6 DOFs are degenerate)
        """
        prev = self._degen_level

        # Hessian-based criteria (from degeneracy_detail)
        hessian_critical = (self._degenerate_dof_count >= 3
                            or self._condition_number > 10000)
        hessian_severe = (self._degenerate_dof_count >= 1
                          or self._condition_number > self._eigen_ratio_warn)

        # Critical: ICP fitness very bad OR almost no features OR Hessian critical
        if (self._icp_fitness > self._fitness_critical
                or self._effective_ratio < self._feature_ratio_critical
                or hessian_critical):
            level = DEGEN_CRITICAL
        # Severe: ICP poor OR low features OR Hessian severe
        elif (self._icp_fitness > self._fitness_warn
              or self._effective_ratio < self._feature_ratio_warn
              or hessian_severe):
            level = DEGEN_SEVERE
        # Mild: borderline but still usable
        elif (self._icp_fitness > self._fitness_warn * 0.7
              or self._effective_ratio < self._feature_ratio_warn * 1.5):
            level = DEGEN_MILD
        else:
            level = DEGEN_NONE

        self._degen_level = level
        if level != prev and level != DEGEN_NONE:
            logger.warning(
                "SLAM degeneracy: %s -> %s (fitness=%.3f, feat_ratio=%.2f, "
                "cond=%.0f, degen_dofs=%d)",
                prev, level, self._icp_fitness, self._effective_ratio,
                self._condition_number, self._degenerate_dof_count)
        elif level == DEGEN_NONE and prev != DEGEN_NONE:
            logger.info("SLAM degeneracy cleared: %s -> NONE", prev)

    # 閳光偓閳光偓 Visual odometry selective DOF fusion 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

    def _on_visual_odom(self, odom: Odometry) -> None:
        """Receive visual odometry from DepthVisualOdomModule.

        Based on Selective KF (arXiv 2412.17235):
        Only fuse visual odometry for degenerate DOF directions.
        When SLAM is healthy, visual odom is ignored.
        """
        self._last_visual_odom = odom

    def _fuse_odometry(self, slam_odom: Odometry) -> Odometry:
        """Two-stage fusion: visual (DOF-selective) then GNSS (global anchor)."""
        after_visual = self._fuse_visual(slam_odom)
        return self._fuse_gnss_position(after_visual)

    def _fuse_visual(self, slam_odom: Odometry) -> Odometry:
        """Selectively blend visual odometry into degenerate DOF directions.

        When degeneracy is NONE/MILD: pass SLAM odom unchanged.
        When SEVERE: blend visual at alpha=0.3 for degenerate directions.
        When CRITICAL: blend visual at alpha=0.6 for degenerate directions.

        Degenerate direction detection (simplified):
        - Corridor 锟?X (along-corridor) is degenerate
        - Open field 锟?X,Y (horizontal plane) are degenerate
        - We use ICP fitness + effective_ratio as proxy:
          * High fitness + low ratio 锟?full translational degeneracy
          * High fitness + moderate ratio 锟?partial (along-corridor)
        """
        if not self._visual_odom_fusion:
            return slam_odom
        if self._degen_level not in (DEGEN_SEVERE, DEGEN_CRITICAL):
            self._last_slam_odom = slam_odom
            return slam_odom
        if self._last_visual_odom is None:
            return slam_odom

        # Determine blend alpha based on severity
        if self._degen_level == DEGEN_CRITICAL:
            alpha = self._visual_alpha  # 0.6
        else:
            alpha = self._visual_alpha * 0.5  # 0.3 for SEVERE

        # Anchor: record SLAM pose when fusion first activates
        slam_pos = np.array([
            slam_odom.pose.position.x,
            slam_odom.pose.position.y,
            slam_odom.pose.position.z,
        ])
        vis_pos = np.array([
            self._last_visual_odom.pose.position.x,
            self._last_visual_odom.pose.position.y,
            self._last_visual_odom.pose.position.z,
        ])

        # Detect degenerate translational axes.
        # If Hessian DOF mask is available (from degeneracy_detail), use it
        # directly: first 3 DOFs are translational (tx, ty, tz).
        # Otherwise fall back to heuristic based on fitness/ratio.
        if self._dof_mask is not None and len(self._dof_mask) >= 3:
            # W2-9: strict preference for Hessian eigenvalue-based mask when
            # available. DOF mask: 1.0=constrained, 0.0=degenerate.
            # Invert: degenerate axes get alpha, constrained get 0.
            logger.debug(
                "slam_bridge: fusing via eigenvalue DOF mask (trans=%s)",
                self._dof_mask[:3].tolist(),
            )
            trans_mask = self._dof_mask[:3]  # tx, ty, tz
            degen_mask = np.where(trans_mask < 0.5, alpha, 0.0)
            # Always reduce Z fusion weight (vertical usually stable from IMU)
            degen_mask[2] *= 0.3
        elif self._effective_ratio < self._feature_ratio_critical:
            # Full translational degeneracy 锟?blend all XYZ (heuristic fallback)
            logger.debug(
                "slam_bridge: heuristic full-XYZ path (effective_ratio=%.3f)",
                self._effective_ratio,
            )
            degen_mask = np.array([alpha, alpha, alpha * 0.3])
        elif self._icp_fitness > self._fitness_critical:
            # Corridor-like: blend XY, keep Z from SLAM (heuristic fallback)
            logger.debug(
                "slam_bridge: heuristic corridor path (icp_fitness=%.3f)",
                self._icp_fitness,
            )
            degen_mask = np.array([alpha, alpha, 0.0])
        else:
            # Partial: blend X (along-axis of highest uncertainty) 锟?heuristic
            logger.debug("slam_bridge: heuristic partial-X path")
            degen_mask = np.array([alpha, alpha * 0.3, 0.0])

        # Selective blend: fused = slam * (1 - mask) + visual * mask
        fused_pos = slam_pos * (1.0 - degen_mask) + vis_pos * degen_mask

        self._visual_fused_count += 1
        if self._visual_fused_count % 50 == 1:
            logger.info(
                "Visual-SLAM fusion active: alpha=%.2f, degen=%s, "
                "slam_xy=(%.2f,%.2f), vis_xy=(%.2f,%.2f), fused_xy=(%.2f,%.2f)",
                alpha, self._degen_level,
                slam_pos[0], slam_pos[1],
                vis_pos[0], vis_pos[1],
                fused_pos[0], fused_pos[1],
            )

        # Build fused odometry (keep SLAM orientation + twist)
        fused_odom = Odometry(
            pose=Pose(
                position=Vector3(
                    x=float(fused_pos[0]),
                    y=float(fused_pos[1]),
                    z=float(fused_pos[2]),
                ),
                orientation=slam_odom.pose.orientation,
            ),
            twist=slam_odom.twist,
            ts=slam_odom.ts,
            frame_id=slam_odom.frame_id,
            child_frame_id=slam_odom.child_frame_id,
        )
        self._last_slam_odom = fused_odom
        return fused_odom

    # 閳光偓閳光偓 GNSS global position anchoring 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

    def _on_gnss_odom(self, odom: GnssOdom) -> None:
        """Store latest GNSS ENU odometry; used by _fuse_gnss_position
        and SceneModeDetector for indoor/outdoor classification (N2)."""
        self._last_gnss_odom = odom
        self._last_gnss_rx_ts = _time.time()
        # Feed scene-mode detector 锟?fix_type names align with GnssFixType enum.
        try:
            self._scene_mode_detector.observe_gnss(
                fix_type=odom.fix_type.name, age_s=0.0, now=self._last_gnss_rx_ts)
            self._maybe_publish_scene_mode()
        except Exception as e:
            logger.debug("scene_mode observe_gnss failed: %s", e)

    def _maybe_publish_scene_mode(self) -> None:
        """Publish scene_mode whenever the effective mode changes."""
        cur = self._scene_mode_detector.mode
        if cur != self._last_published_scene_mode:
            self._last_published_scene_mode = cur
            try:
                self.scene_mode.publish(cur)
            except Exception as e:
                logger.debug("scene_mode publish failed: %s", e)

    def _fuse_gnss_position(self, odom: Odometry) -> Odometry:
        """Blend RTK GNSS ENU position into SLAM XY to anchor long-term drift.

        Strategy
        --------
        1. Gate on fix quality (RTK only), freshness, horizontal std.
        2. First qualifying sample while SLAM is healthy locks the
           ``map 锟?enu`` translation offset (one-shot alignment).
        3. Subsequent samples: weighted blend of SLAM XY with aligned GNSS XY.
           Weight rises when SLAM is degraded so GNSS carries more load.
        4. Z and orientation untouched 锟?GNSS altitude is too noisy and a
           single position doesn't define heading.
        """
        if not self._gnss_fusion or self._last_gnss_odom is None:
            return odom

        g = self._last_gnss_odom

        # Freshness: drop if the last received sample is stale
        age = _time.time() - self._last_gnss_rx_ts
        if age > self._gnss_max_age_s:
            return odom

        # Quality: only RTK (fixed or float) carries position-level authority.
        # SINGLE/DGPS are too coarse (> 1 m) to help a LiDAR SLAM anchor.
        if g.fix_type not in (GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT):
            return odom

        # Noise gate
        h_var = max(g.cov_e, 0.0) + max(g.cov_n, 0.0)
        h_std = (h_var * 0.5) ** 0.5
        if h_std > self._gnss_max_std_m:
            return odom

        slam_xy = np.array([odom.pose.position.x, odom.pose.position.y])

        # Lever-arm compensation: GNSS reports the antenna, SLAM reports the
        # body. Rotate the antenna offset from body to map frame using current
        # SLAM orientation, then subtract its XY from the GNSS ENU position
        # so both operands refer to the body origin.
        if any(self._gnss_antenna_offset):
            R_body2map = odom.pose.orientation.to_rotation_matrix()
            a_map = R_body2map @ np.asarray(self._gnss_antenna_offset, dtype=float)
            gnss_xy = np.array([g.east - a_map[0], g.north - a_map[1]])
        else:
            gnss_xy = np.array([g.east, g.north])

        # One-shot alignment: lock the map閳摗NU translation while SLAM healthy
        if self._gnss_map_offset is None:
            if self._loc_state != LOC_TRACKING:
                return odom
            if self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL):
                return odom
            self._gnss_map_offset = slam_xy - gnss_xy
            logger.info(
                "GNSS-SLAM alignment locked: map_offset=(%.3f, %.3f) m "
                "(fix=%s, std=%.2fm)",
                self._gnss_map_offset[0], self._gnss_map_offset[1],
                g.fix_type.name, h_std,
            )
            return odom

        # GNSS in SLAM map frame
        gnss_in_map = gnss_xy + self._gnss_map_offset

        # Residual guard: track misalignment, force re-lock on sustained drift
        residual = float(np.linalg.norm(slam_xy - gnss_in_map))
        self._gnss_last_residual_m = residual
        if self._check_residual_and_maybe_relock(residual):
            # Just cleared the offset 锟?skip blending this frame, next valid
            # GNSS sample will re-lock on a fresh pair.
            self._publish_gnss_health()
            return odom

        # Weight: degraded SLAM 锟?trust GNSS more
        if self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL):
            alpha = self._gnss_alpha_degraded
        else:
            alpha = self._gnss_alpha_healthy
        if g.fix_type == GnssFixType.RTK_FLOAT:
            alpha *= self._gnss_rtk_float_scale

        fused_xy = slam_xy * (1.0 - alpha) + gnss_in_map * alpha

        self._gnss_fused_count += 1
        if self._gnss_fused_count % 50 == 1:
            logger.info(
                "GNSS-SLAM fusion active: alpha=%.2f, fix=%s, std=%.2fm, "
                "residual=%.2fm, slam=(%.2f,%.2f) gnss=(%.2f,%.2f) "
                "fused=(%.2f,%.2f)",
                alpha, g.fix_type.name, h_std, residual,
                slam_xy[0], slam_xy[1],
                gnss_in_map[0], gnss_in_map[1],
                fused_xy[0], fused_xy[1],
            )
        self._publish_gnss_health()

        return Odometry(
            pose=Pose(
                position=Vector3(
                    x=float(fused_xy[0]),
                    y=float(fused_xy[1]),
                    z=odom.pose.position.z,
                ),
                orientation=odom.pose.orientation,
            ),
            twist=odom.twist,
            ts=odom.ts,
            frame_id=odom.frame_id,
            child_frame_id=odom.child_frame_id,
        )

    def _check_residual_and_maybe_relock(self, residual: float) -> bool:
        """Sliding-window check: if too many recent samples exceed threshold,
        clear the alignment offset so the next good sample re-locks it.

        Returns True iff the offset was just cleared.
        """
        now = _time.time()
        window = self._gnss_residual_warn_duration_s
        self._gnss_residual_history.append((now, residual))
        cutoff = now - window
        # Drop entries older than the window
        while self._gnss_residual_history and self._gnss_residual_history[0][0] < cutoff:
            self._gnss_residual_history.popleft()

        # Need a filled window + enough samples (锟?5) before firing
        if not self._gnss_residual_history:
            return False
        if self._gnss_residual_history[0][0] > now - window * 0.5:
            return False
        if len(self._gnss_residual_history) < 5:
            return False

        warn_count = sum(
            1 for _ts, r in self._gnss_residual_history
            if r > self._gnss_residual_warn_m
        )
        ratio = warn_count / len(self._gnss_residual_history)
        if ratio < self._gnss_residual_warn_ratio:
            return False

        # Fire: clear offset, bump counter, drop history
        logger.warning(
            "GNSS-SLAM residual sustained > %.1fm (%.0f%% of last %.1fs, "
            "n=%d); relocking alignment. Previous offset=%s",
            self._gnss_residual_warn_m, ratio * 100.0, window,
            len(self._gnss_residual_history),
            None if self._gnss_map_offset is None
            else f"({self._gnss_map_offset[0]:.2f}, {self._gnss_map_offset[1]:.2f})",
        )
        self._gnss_map_offset = None
        self._gnss_residual_history.clear()
        self._gnss_relock_count += 1
        self._gnss_last_relock_ts = now
        return True

    def _publish_gnss_health(self) -> None:
        """Publish a compact diagnostic snapshot for Dashboard / watchdog."""
        self.gnss_fusion_health.publish({
            "enabled": self._gnss_fusion,
            "alignment_locked": self._gnss_map_offset is not None,
            "map_offset": (
                None if self._gnss_map_offset is None
                else [float(self._gnss_map_offset[0]),
                      float(self._gnss_map_offset[1])]
            ),
            "last_residual_m": float(self._gnss_last_residual_m),
            "fused_count": int(self._gnss_fused_count),
            "relock_count": int(self._gnss_relock_count),
            "last_relock_ts": float(self._gnss_last_relock_ts),
            "last_fix_type": (
                self._last_gnss_odom.fix_type.name
                if self._last_gnss_odom is not None else "NONE"
            ),
            "last_gnss_age_s": (
                float(_time.time() - self._last_gnss_rx_ts)
                if self._last_gnss_rx_ts > 0 else float("inf")
            ),
        })

    def _is_gnss_healthy(self, now: float) -> bool:
        """True when GNSS is fresh and at RTK_FIXED/FLOAT precision.

        Used to gate the LOC_DEGRADED 锟?LOC_FALLBACK_GNSS_ONLY transition:
        promoting to fallback only makes sense if we have a globally-anchored
        position to fall back to.
        """
        if self._last_gnss_odom is None or self._last_gnss_rx_ts <= 0:
            return False
        age = now - self._last_gnss_rx_ts
        if age > self._gnss_max_age_for_fallback_s:
            return False
        return self._last_gnss_odom.fix_type in (
            GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT)

    # Localization health watchdog.

    def _watchdog_loop(self) -> None:
        """Periodic localization health check with automatic recovery.

        Runs at self._watchdog_hz Hz. Each cycle evaluates 11 phases:

        Phase  1 锟?Sample timestamps and compute effective timeouts
        Phase  2 锟?Localization state machine
                  UNINIT (no data) -> LOST (odom timeout) -> DEGRADED (cloud
                  timeout or CRITICAL degeneracy) -> TRACKING (healthy)
        Phase  3 锟?Promote sustained DEGRADED to FALLBACK_GNSS_ONLY when GNSS
                  is healthy (Navigation runs cautiously)
        Phase  4 锟?Degeneracy-based confidence adjustment (SEVERE/CRITICAL
                  degrades trust; visual odom fusion can boost it back)
        Phase  5 锟?Assemble localization_status dict fields
        Phase  6 锟?Log state transitions, track recovery, reset counters
                  on recovery to TRACKING
        Phase  7 锟?Detect completion of localizer odom-loss recovery when
                  fresh odometry arrives
        Phase  8 锟?Trigger localizer chain restart if odometry has been
                  absent while localizer health says LOCKED/RECOVERED
        Phase  9 锟?Auto-reconnect DDS/rclpy reader when LOST exceeds
                  reconnect_timeout (exponential backoff)
        Phase 10 锟?Gate pose_fresh and auto_resume_blocked flags while
                  recovery is in-flight
        Phase 11 锟?Publish localization_status dict and update scene_mode
                  detector (indoor/outdoor) when GNSS is silent

        Recovery triggers:
        - Drift divergence (LOC_DIVERGED): auto-relocalize via ROS2 service
          call or systemctl restart slam
        - Localizer odom loss: restart Fast-LIO2 + localizer chain via
          service_manager (preferred) or systemctl (fallback)
        - Prolonged LOST: stop + recreate DDS/rclpy reader with backoff
        """
        interval = 1.0 / self._watchdog_hz
        while not self._shutdown_event.is_set():
            # --- Phase 1: Sample timestamps and compute effective timeouts ---
            now = _time.time()
            now_mono = _time.monotonic()
            odom_age = self._odom_age_s(mono_now=now_mono)
            cloud_age = self._cloud_age_s(mono_now=now_mono)
            contract = self._backend_contract(self._current_backend_profile())
            localizer_topic_fresh = self._localizer_health_topic_fresh(now)
            localizer_health_grace = self._localizer_health_allows_odom_grace(
                str(contract["backend"]), localizer_topic_fresh
            )
            effective_odom_timeout = self._effective_odom_timeout(
                localizer_health_grace=localizer_health_grace
            )

            # --- Phase 2: Localization state machine ---
            if self._loc_state == LOC_DIVERGED:
                # Drift guard set this 锟?keep it until drift clears
                new_state = LOC_DIVERGED
                confidence = 0.0
            elif self._last_odom_time == 0.0:
                new_state = LOC_UNINIT
                confidence = 0.0
            elif odom_age > effective_odom_timeout:
                new_state = LOC_LOST
                confidence = 0.0
            elif cloud_age > self._cloud_timeout:
                new_state = LOC_DEGRADED
                confidence = 0.3
            elif self._degen_level == DEGEN_CRITICAL:
                new_state = LOC_DEGRADED
                confidence = 0.1
            else:
                new_state = LOC_TRACKING
                confidence = max(0.0, 1.0 - odom_age / effective_odom_timeout)

            # --- Phase 3: Promote sustained DEGRADED to FALLBACK_GNSS_ONLY ---
            # Today this only changes the published state so Navigation
            # can run cautiously; the GNSS+IMU dead-reckoning takeover lands
            # in S2.5 once fault-injection tooling exists.
            if new_state == LOC_DEGRADED:
                if self._degraded_since == 0.0:
                    self._degraded_since = now
                if (now - self._degraded_since) > self._fallback_after_degraded_s \
                        and self._is_gnss_healthy(now):
                    new_state = LOC_FALLBACK_GNSS_ONLY
                    confidence = max(confidence, 0.4)
            else:
                self._degraded_since = 0.0

            # --- Phase 4: Degeneracy-based confidence adjustment + GNSS backstop ---
            visual_active = (self._last_visual_odom is not None
                             and self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL))
            if self._degen_level == DEGEN_SEVERE:
                confidence = min(confidence, 0.6 if visual_active else 0.4)
            elif self._degen_level == DEGEN_MILD:
                confidence = min(confidence, 0.7)
            elif self._degen_level == DEGEN_CRITICAL:
                if visual_active:
                    # Visual fusion compensates for CRITICAL 锟?upgrade to DEGRADED not LOST
                    confidence = max(confidence, 0.3)

            # --- Phase 5: Assemble localization status fields ---
            health_source = (
                "localizer_health_topic"
                if localizer_topic_fresh
                else "odom_map_cloud"
            )
            effective_localizer_health = (
                self._localizer_health
                if localizer_topic_fresh
                else self._synthesized_localizer_health(new_state)
            )
            pose_fresh = (
                new_state not in {LOC_UNINIT, LOC_LOST, LOC_DIVERGED}
                and odom_age <= effective_odom_timeout
            )
            map_cloud_fresh = (
                self._last_cloud_time > 0.0
                and cloud_age <= self._cloud_timeout
            )
            if self._last_cloud_time == 0.0:
                map_state = "waiting_for_map_cloud"
            elif map_cloud_fresh:
                map_state = "live_map_cloud"
            else:
                map_state = "map_cloud_stale"
            relocalization_supported = bool(contract["relocalization_supported"])
            if contract["backend"] != self._last_status_backend:
                self._last_status_backend = str(contract["backend"])
                self._relocalization_state = (
                    "idle" if relocalization_supported else "unsupported"
                )
                if self._last_recovery_signal == "RECOVERED":
                    self._last_recovery_signal = "NONE"
                    self._last_recovery_action = "none"
                    self._last_recovery_ts = 0.0
            relocalization_state = self._relocalization_state
            if not relocalization_supported and relocalization_state == "idle":
                relocalization_state = "unsupported"
                self._relocalization_state = relocalization_state
            topic_age_ms = (
                round(max(0.0, now - self._localizer_health_ts) * 1000.0, 1)
                if self._localizer_health_ts > 0.0
                else None
            )

            # --- Phase 6: State transition logging and recovery tracking ---
            if new_state != self._loc_state:
                if new_state == LOC_LOST:
                    logger.warning("Localization LOST: no odometry for %.1fs", odom_age)
                elif new_state == LOC_DEGRADED and self._degen_level != DEGEN_NONE:
                    logger.warning("Localization DEGRADED: SLAM degeneracy %s",
                                   self._degen_level)
                elif new_state == LOC_FALLBACK_GNSS_ONLY:
                    logger.warning(
                        "Localization FALLBACK_GNSS_ONLY: DEGRADED for %.1fs with "
                        "healthy GNSS 锟?Navigation should run cautiously",
                        now - self._degraded_since)
                elif new_state == LOC_TRACKING and (
                        self._loc_state in (
                            LOC_LOST,
                            LOC_DEGRADED,
                            LOC_DIVERGED,
                            LOC_FALLBACK_GNSS_ONLY,
                        )
                        or (
                            self._last_recovery_signal
                            == LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL
                        )
                ):
                    logger.info("Localization recovered -> TRACKING from %s",
                                self._loc_state)
                    self._reconnect_count = 0
                    self._drift_bad_count = 0
                    self._last_recovery_signal = "RECOVERED"
                    self._last_recovery_action = "none"
                    self._last_recovery_ts = now
                    self._relocalization_state = (
                        "unsupported" if not relocalization_supported else "idle"
                    )
                self._loc_state = new_state

            # --- Phase 7: Check if recovery completed (fresh odometry back) ---
            if (
                self._last_recovery_signal
                == LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL
                and not self._localizer_odom_loss_recovery_inflight
                and not self._localizer_odom_loss_recovery_waiting_new_odom
                and new_state == LOC_TRACKING
                and odom_age <= effective_odom_timeout
            ):
                logger.info(
                    "Localization recovery completed with fresh odometry -> "
                    "TRACKING"
                )
                self._last_recovery_signal = "RECOVERED"
                self._last_recovery_action = "none"
                self._last_recovery_ts = now
                self._relocalization_state = (
                    "unsupported" if not relocalization_supported else "idle"
                )

            # --- Phase 8: Trigger localizer odom-loss recovery if due ---
            if self._localizer_odom_loss_recovery_due(
                backend=str(contract["backend"]),
                localizer_topic_fresh=localizer_topic_fresh,
                odom_age=odom_age,
                now=now,
                now_mono=now_mono,
            ):
                logger.warning(
                    "Localization recovery: localizer health is %s but odometry "
                    "has been absent for %.1fs; restarting localization chain",
                    self._localizer_health,
                    odom_age if self._last_odom_time > 0.0
                    else now_mono - self._watchdog_start_mono,
                )
                self._start_localizer_odom_loss_recovery(now=now)

            # --- Phase 9: Auto-reconnect DDS/rclpy for prolonged LOST ---
            if (new_state == LOC_LOST
                    and odom_age > self._reconnect_timeout
                    and not self._localizer_odom_loss_recovery_inflight
                    and self._reconnect_count < self._max_reconnects):
                self._attempt_reconnect()

            # --- Phase 10: Gate pose freshness and auto-resume ---
            odom_loss_recovery_waiting_new_odom = (
                self._localizer_odom_loss_recovery_waiting_new_odom
            )
            auto_resume_blocked = (
                self._localizer_odom_loss_recovery_inflight
                or odom_loss_recovery_waiting_new_odom
                or (
                    self._last_recovery_signal
                    == LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL
                )
            )
            pose_fresh = (
                pose_fresh
                and not self._localizer_odom_loss_recovery_inflight
                and not odom_loss_recovery_waiting_new_odom
            )

            # --- Phase 11: Publish status and update scene mode ---
            self.localization_status.publish({
                **self._backend_health_fields(contract=contract),
                "state": self._loc_state,
                "odom_age_ms": round(odom_age * 1000, 1),
                "odom_timeout_ms": round(effective_odom_timeout * 1000, 1),
                "localizer_health_grace": localizer_health_grace,
                "cloud_age_ms": round(cloud_age * 1000, 1),
                "confidence": round(confidence, 2),
                "ts": now,
                "mono_ts": now_mono,
                "health_source": health_source,
                "pose_fresh": pose_fresh,
                "map_cloud_fresh": map_cloud_fresh,
                "map_state": map_state,
                "map_save_supported": bool(contract["map_save_supported"]),
                "map_save_source": contract["map_save_source"],
                "relocalization_supported": relocalization_supported,
                "saved_map_relocalization_supported": bool(
                    contract["saved_map_relocalization_supported"]
                ),
                "restart_recovery_supported": bool(
                    contract["restart_recovery_supported"]
                ),
                "recovery_method": contract["recovery_method"],
                "relocalization_state": relocalization_state,
                "recovery_signal": self._last_recovery_signal,
                "recovery_action": self._last_recovery_action,
                "recovery_ts": self._last_recovery_ts,
                "odom_loss_recovery_supported": (
                    str(contract["backend"]) == "localizer"
                    and self._localizer_odom_loss_recovery_enabled
                ),
                "odom_loss_recovery_inflight": (
                    self._localizer_odom_loss_recovery_inflight
                ),
                "odom_loss_recovery_waiting_new_odom": (
                    odom_loss_recovery_waiting_new_odom
                ),
                "odom_loss_recovery_s": self._localizer_odom_loss_recovery_s,
                "odom_loss_recovery_cooldown_s": (
                    self._localizer_odom_loss_recovery_cooldown_s
                ),
                "motion_hold_required": auto_resume_blocked,
                "auto_resume_blocked": auto_resume_blocked,
                "degeneracy": self._degen_level,
                "icp_fitness": round(self._icp_fitness, 4),
                "effective_ratio": round(self._effective_ratio, 3),
                "condition_number": round(self._condition_number, 1),
                "degenerate_dof_count": self._degenerate_dof_count,
                "cov_warning": self._max_pos_cov > 100.0,
                "max_pos_cov": round(self._max_pos_cov, 3),
                "pos_cov_trace": round(self._pos_cov_trace, 4),
                "ieskf_iter_num": self._ieskf_iter_num,
                "ieskf_converged": self._ieskf_converged,
                "localizer_health": effective_localizer_health,
                "localizer_health_raw": self._localizer_health,
                "localizer_health_source": health_source,
                "localizer_health_topic_age_ms": topic_age_ms,
                "localizer_health_fitness": round(self._localizer_health_fitness, 4),
                "localizer_health_iter": self._localizer_health_iter,
                "localizer_health_cov_trace": round(self._localizer_health_cov_trace, 4),
                "scene_mode": self._scene_mode_detector.mode,
            })

            # N2: feed indoor evidence to detector when GNSS has been silent
            # for too long. Without this, the detector would never converge
            # to "indoor" on a robot that simply has no GNSS module wired.
            if self._last_gnss_rx_ts == 0.0 \
                    or (now - self._last_gnss_rx_ts) > self._gnss_max_age_for_fallback_s:
                self._scene_mode_detector.observe_no_gnss(now=now)
                self._maybe_publish_scene_mode()

            self._shutdown_event.wait(timeout=interval)

    def _attempt_reconnect(self) -> None:
        """Try to reconnect DDS/rclpy backend after data loss."""
        if self._localizer_odom_loss_recovery_inflight:
            logger.info(
                "SlamBridge: skipping backend reconnect while localization "
                "chain recovery is in progress"
            )
            return
        self._reconnect_count += 1
        backoff = min(2.0 ** self._reconnect_count, 30.0)
        logger.warning(
            "SlamBridge: attempting reconnect (%d/%d, backoff %.0fs)",
            self._reconnect_count, self._max_reconnects, backoff)

        # Destroy old connections
        if self._reader:
            try:
                self._reader.stop()
            except Exception as e:
                logger.debug("SlamBridge: reader.stop() cleanup error: %s", e)
            self._reader = None
        self._cleanup_rclpy_node()

        self._shutdown_event.wait(timeout=backoff)
        if self._shutdown_event.is_set():
            return

        # Try to reconnect
        if self._try_cyclonedds():
            self._reader.spin_background()
            logger.info("SlamBridge: cyclonedds reconnected")
            self.alive.publish(True)
        elif self._try_rclpy():
            logger.info("SlamBridge: rclpy reconnected")
            self.alive.publish(True)
        else:
            logger.warning("SlamBridge: reconnect failed")
            self.alive.publish(False)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        now_mono = _time.monotonic()
        profile = self._current_backend_profile()
        contract = self._backend_contract(profile)
        info["localization"] = {
            **self._backend_health_fields(profile=profile, contract=contract),
            "state": self._loc_state,
            "odom_age_ms": round(self._odom_age_s(mono_now=now_mono) * 1000, 1) if self._last_odom_time else -1,
            "cloud_age_ms": round(self._cloud_age_s(mono_now=now_mono) * 1000, 1) if self._last_cloud_time else -1,
            "degeneracy": self._degen_level,
            "icp_fitness": round(self._icp_fitness, 4),
            "effective_ratio": round(self._effective_ratio, 3),
            "condition_number": round(self._condition_number, 1),
            "degenerate_dof_count": self._degenerate_dof_count,
            "dof_mask": self._dof_mask.tolist() if self._dof_mask is not None else None,
            "visual_fusion_active": (
                self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL)
                and self._last_visual_odom is not None
            ),
            "visual_fused_count": self._visual_fused_count,
            "odom_worker_drops": self._odom_worker_drops,
            "pointcloud_worker_drops": self._pointcloud_worker_drops,
            "saved_map_worker_drops": self._saved_map_worker_drops,
        }
        info["gnss_fusion"] = self._gnss_health_snapshot()
        return info

    # -- AI-callable skills ------------------------------------------------

    def _gnss_health_snapshot(self) -> dict[str, Any]:
        """Shared payload for @skill and health() 锟?matches Out port schema."""
        return {
            "enabled": self._gnss_fusion,
            "alignment_locked": self._gnss_map_offset is not None,
            "map_offset": (
                None if self._gnss_map_offset is None
                else [float(self._gnss_map_offset[0]),
                      float(self._gnss_map_offset[1])]
            ),
            "antenna_offset_body": list(self._gnss_antenna_offset),
            "last_residual_m": float(self._gnss_last_residual_m),
            "fused_count": int(self._gnss_fused_count),
            "relock_count": int(self._gnss_relock_count),
            "last_relock_ts": float(self._gnss_last_relock_ts),
            "last_fix_type": (
                self._last_gnss_odom.fix_type.name
                if self._last_gnss_odom is not None else "NONE"
            ),
            "last_gnss_age_s": (
                float(_time.time() - self._last_gnss_rx_ts)
                if self._last_gnss_rx_ts > 0 else float("inf")
            ),
        }

    @skill
    def get_gnss_fusion_status(self) -> str:
        """Return GNSS-SLAM fusion health: alignment, residual, fix quality,
        relock count, and last GNSS sample age. Use this to diagnose outdoor
        drift or suspected GNSS signal loss."""
        return json.dumps(self._gnss_health_snapshot(), default=str)

    @skill
    def relock_gnss_alignment(self) -> str:
        """Force re-locking of the GNSS閳摬LAM alignment offset. The next
        valid RTK fix received while SLAM is healthy will establish a new
        offset. Use this after a known map switch or after diagnosing that
        the current alignment drifted."""
        prev = (
            None if self._gnss_map_offset is None
            else [float(self._gnss_map_offset[0]),
                  float(self._gnss_map_offset[1])]
        )
        self._gnss_map_offset = None
        self._gnss_residual_history.clear()
        self._gnss_relock_count += 1
        self._gnss_last_relock_ts = _time.time()
        logger.warning(
            "GNSS-SLAM alignment manually relocked (prev=%s)", prev)
        return json.dumps({
            "status": "relocked",
            "previous_offset": prev,
            "relock_count": self._gnss_relock_count,
        })

    @skill
    def set_gnss_fusion(self, enabled: bool) -> str:
        """Enable or disable the GNSS-SLAM fusion runtime switch. When
        disabled, SLAM odometry is published without any GNSS correction
        (equivalent to pure LiDAR-inertial SLAM). Persisted only in this
        process 锟?does not write robot_config.yaml."""
        self._gnss_fusion = bool(enabled)
        if not self._gnss_fusion:
            # Dropping the alignment is kinder than leaving a stale offset
            self._gnss_map_offset = None
            self._gnss_residual_history.clear()
        logger.warning("GNSS fusion runtime switch: %s",
                        "ON" if self._gnss_fusion else "OFF")
        return json.dumps({
            "status": "ok",
            "enabled": self._gnss_fusion,
        })
