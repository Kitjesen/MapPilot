"""TerrainModule — traversability analysis as a pluggable Module.

Analyzes point cloud + odometry to produce terrain maps for local planning.

Backends:
  "nanobind" — C++ terrain_core via nanobind (zero-copy, GIL-released, preferred)
  "cmu"      — alias for the legacy C++ NativeModule terrain stack
  "native"   — C++ NativeModule subprocess (legacy, S100P fallback)
  "simple"   — Python passthrough (testing, no analysis)

Usage::

    bp.add(TerrainModule, backend="nanobind")
    # In: odometry, map_cloud → Out: terrain_map, traversability, elevation_map
"""

from __future__ import annotations

import logging
import time
from typing import Any

from base_autonomy.modules._nav_core_loader import nav_core_build_hint, try_import_nav_core
from core.backend_status import BackendStatus, require_backend
from core.msgs.numpy_compat import np
from core.module import Module
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In, Out

logger = logging.getLogger(__name__)


_AVAILABLE_TERRAIN_BACKENDS = ("nanobind", "native", "cmu", "simple")


@register("terrain", "nanobind", description="C++ terrain analysis via nanobind (zero-copy)")
@register("terrain", "cmu", description="CMU-style C++ terrain analysis via NativeModule subprocess")
@register("terrain", "native", description="C++ terrain analysis via NativeModule subprocess")
@register("terrain", "simple", description="Passthrough for testing")
class TerrainModule(Module, layer=2):
    """Traversability analysis — ground estimation + obstacle detection.

    "nanobind": C++ TerrainAnalysisCore called directly from Python.
      Data flows through In/Out ports. Zero-copy numpy. GIL released during C++.
    "cmu": Alias for the legacy NativeModule subprocess (C++ process + DDS).
    "native": Legacy NativeModule subprocess (C++ process + DDS).
    "simple": Passthrough for testing.
    """

    # -- Inputs --
    odometry: In[Odometry]
    map_cloud: In[PointCloud2]

    # -- Outputs --
    terrain_map: Out[PointCloud2]
    traversability: Out[dict]
    elevation_map: Out[Any]
    alive: Out[bool]

    def __init__(self, backend: str = "nanobind", **kw):
        super().__init__(**kw)
        require_backend("terrain", backend, _AVAILABLE_TERRAIN_BACKENDS)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._core = None       # nanobind: TerrainAnalysisCore
        self._nodes: dict[str, Any] = {}  # native: NativeModule dict
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_z = 0.0
        self._odom_yaw = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)
        self.map_cloud.set_policy("latest")

        if self._backend == "nanobind":
            self._setup_nanobind()
        elif self._backend in {"native", "cmu"}:
            self._setup_native()
        else:
            logger.info("TerrainModule: simple backend (passthrough)")

    def _setup_nanobind(self):
        """Setup C++ terrain_core via nanobind binding."""
        _nav_core = try_import_nav_core(("TerrainParams", "TerrainAnalysisCore"))
        if _nav_core is None:
            raise RuntimeError(
                f"TerrainModule [nanobind]: compatible _nav_core not found. "
                f"Build C++ backend or explicitly choose backend='simple' for passthrough testing.\n"
                f"  To build: {nav_core_build_hint()}"
            )
        try:
            params = _nav_core.TerrainParams()
            # Load from robot config if available
            try:
                from core.config import get_config
                cfg = get_config()
                ta = cfg.raw.get("terrain_analysis", {})
                if ta:
                    for attr in ["scan_voxel_size", "decay_time", "no_decay_dis",
                                 "obstacle_height_thre", "vehicle_height",
                                 "min_rel_z", "max_rel_z"]:
                        if attr in ta:
                            setattr(params, attr, ta[attr])
            except ImportError:
                pass

            self._core = _nav_core.TerrainAnalysisCore(params)
            logger.info("TerrainModule [nanobind]: C++ terrain_core loaded")
        except RuntimeError:
            raise
        except Exception as e:
            raise RuntimeError(
                f"TerrainModule [nanobind]: _nav_core init failed: {e}. "
                f"Install _nav_core.so or explicitly choose backend='simple'."
            ) from e

    def _setup_native(self):
        """Setup C++ NativeModule backends (legacy)."""
        try:
            from base_autonomy.native_factories import terrain_analysis, terrain_analysis_ext
            from core.config import get_config
            cfg = get_config()
            self._nodes = {
                "terrain": terrain_analysis(cfg),
                "terrain_ext": terrain_analysis_ext(cfg),
            }
            for name, node in self._nodes.items():
                try:
                    node.setup()
                except (FileNotFoundError, PermissionError) as e:
                    logger.warning("TerrainModule: %s setup failed: %s", name, e)
                    self._backend_status.mark_degraded(f"{name} setup failed: {e}")
        except ImportError as e:
            logger.warning("TerrainModule: native backend not available: %s", e)
            self._backend_status.mark_degraded(f"native backend not available: {e}")

    def start(self):
        super().start()
        if self._backend in {"native", "cmu"}:
            started = 0
            for name, node in self._nodes.items():
                try:
                    node.start()
                    started += 1
                except Exception as e:
                    logger.error("TerrainModule: %s start failed: %s", name, e)
            self.alive.publish(started == len(self._nodes))
        else:
            self.alive.publish(True)
        logger.info("TerrainModule [%s]: started", self._backend)

    def stop(self):
        for _name, node in reversed(list(self._nodes.items())):
            try:
                node.stop()
            except Exception as e:
                logger.warning("TerrainModule stop %s: %s", _name, e)
        self._nodes.clear()
        if self._core:
            self._core.clear()
            self._core = None
        self.alive.publish(False)
        super().stop()

    def _on_odom(self, odom: Odometry):
        self._odom_x = odom.x
        self._odom_y = odom.y
        self._odom_z = getattr(odom, 'z', 0.0)
        self._odom_yaw = odom.yaw

        if self._core:
            # Feed odometry to C++ core (roll/pitch = 0 for ground robot)
            self._core.update_vehicle(
                odom.x, odom.y, self._odom_z,
                0.0, 0.0, odom.yaw)

        if self._backend == "simple":
            self.traversability.publish({"status": "passthrough"})

    def _on_cloud(self, cloud: PointCloud2):
        # GIL hot-path. Three layers of protection:
        # 1. Bypass entirely if env LINGTU_DISABLE_TERRAIN=1
        # 2. Throttle to 0.5 Hz (2 s interval)
        # 3. Cap point count in _process_nanobind below
        import os
        if os.environ.get("LINGTU_DISABLE_TERRAIN", "0") == "1":
            return
        now = time.time()
        if now - getattr(self, "_last_process_ts", 0.0) < 2.0:
            return
        self._last_process_ts = now

        if self._backend == "simple":
            self.terrain_map.publish(cloud)
            return

        if self._backend == "nanobind" and self._core is not None:
            self._process_nanobind(cloud)

    def _process_nanobind(self, cloud: PointCloud2):
        """Process point cloud through C++ terrain_core. GIL released during C++."""
        pts = cloud.points  # numpy Nx3 or Nx4
        if pts is None or len(pts) == 0:
            return

        # Ensure Nx4 float32 (x, y, z, intensity)
        if pts.ndim == 2 and pts.shape[1] == 3:
            pts4 = np.zeros((len(pts), 4), dtype=np.float32)
            pts4[:, :3] = pts
        elif pts.ndim == 2 and pts.shape[1] >= 4:
            pts4 = pts[:, :4].astype(np.float32, copy=False)
        else:
            return

        # W2-5: pass numpy array directly — no 30K truncation, no .tolist()
        # conversion. nanobind reads the buffer protocol, so this is close to
        # zero-copy without requiring the full nb::ndarray<float, ndim<1>>
        # binding refactor (follow-up C++ change).
        flat = np.ascontiguousarray(pts4, dtype=np.float32).ravel()
        ts = getattr(cloud, 'ts', time.time())

        result = self._core.process(flat, ts)

        # Publish terrain cloud
        if result.n_points > 0:
            arr = np.array(result.terrain_points, dtype=np.float32).reshape(-1, 4)
            self.terrain_map.publish(PointCloud2(
                points=arr,
                frame_id=topic_default_frame_id(TOPICS.terrain_map),
                ts=ts,
            ))
            self.traversability.publish({
                "n_obstacles": result.n_points,
                "map_width": result.map_width,
                "map_resolution": result.map_resolution,
            })

        # Publish elevation map
        if result.map_width > 0:
            elev = np.array(result.elevation_map, dtype=np.float32).reshape(
                result.map_width, result.map_width)
            self.elevation_map.publish(elev)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        node_health = {}
        for name, node in self._nodes.items():
            h = node.health()
            native = h.get("native", {})
            node_health[name] = {
                "running": native.get("running", False),
                "pid": native.get("pid"),
            }
        info["terrain"] = {
            **self._backend_status.as_health_fields(),
            "has_core": self._core is not None,
            "nodes": node_health,
        }
        return info
