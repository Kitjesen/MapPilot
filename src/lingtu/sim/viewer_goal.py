"""Submit MuJoCo picks through the current simulation Host's navigation API."""

from __future__ import annotations

import http.client
import json
import select
import socket
import threading
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import numpy as np

from runtime.msgs.geometry import Quaternion
from runtime.tf.frames import body_frame_id


class _GoalSocket(socket.socket):
    """Make HTTP reads cancellable even when Windows select ignores shutdown."""

    cancelled: threading.Event

    def recv_into(self, buffer: Any, nbytes: int = 0, flags: int = 0) -> int:
        deadline = time.monotonic() + self.gettimeout()
        while not self.cancelled.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Goal reply timed out")
            readable, _, _ = select.select([self], [], [], min(0.1, remaining))
            if readable and not self.cancelled.is_set():
                return super().recv_into(buffer, nbytes, flags)
        raise OSError("Viewer closed; goal reply cancelled")


def _rotation(pose: dict[str, Any]) -> np.ndarray:
    quaternion = np.array([pose[key] for key in ("qx", "qy", "qz", "qw")], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("Localization rotation is unavailable")
    return Quaternion(*(quaternion / norm)).to_rotation_matrix()


def world_point_to_map(
    point: np.ndarray, body_position: np.ndarray, body_rotation: np.ndarray,
    slam: dict[str, Any],
) -> np.ndarray:
    """Compose map<-odom<-body<-world; world and map need not share an origin."""
    transform = slam["map_odom_tf"]
    odometry = slam["odometry"]
    if (
        not isinstance(transform, dict)
        or not isinstance(odometry, dict)
        or transform.get("valid") is not True
        or transform.get("frame_id") != "map"
        or transform.get("child_frame_id") != "odom"
        or odometry.get("frame_id") != "odom"
        or odometry.get("child_frame_id") != body_frame_id()
    ):
        raise ValueError("Navigation map transform is not ready")
    pose = odometry["pose"]
    in_body = body_rotation.T @ (point - body_position)
    in_odom = _rotation(pose) @ in_body + np.array([pose[k] for k in ("x", "y", "z")])
    result = _rotation(transform) @ in_odom + np.array([transform[k] for k in ("tx", "ty", "tz")])
    if not np.isfinite(result).all():
        raise ValueError("Navigation target coordinates are invalid")
    return result


class ViewerGoal:
    """One HTTP request and the latest pending click, without blocking rendering."""

    def __init__(self, plan: Any, session_root: Path, product_session_id: str) -> None:
        config = plan.host_config
        self._enabled = bool(config.get("enable_goals"))
        self._base_url = f"http://127.0.0.1:{int(config['gateway_port'])}"
        self._socket: socket.socket | None = None
        self._connection_lock = threading.Lock()
        self._cancelled = threading.Event()
        self._closed = False
        self._session_id = product_session_id
        self._slam_path = session_root / "slam.status.json"
        slam = plan.process("slam") if plan.has_process("slam") else None
        self._truth_localization = (
            slam is not None and slam.command is not None
            and "--navigation-fixture" in getattr(slam.command, "argv", ())
        )
        nav = plan.process("nav") if plan.has_process("nav") else None
        environment = dict(nav.command.env) if nav is not None and nav.command is not None else {}
        self._max_age_s = float(environment.get("LINGTU_NAV_ODOM_MAX_AGE_S", 0.6))
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco-goal")
        self._future: Future[str] | None = None
        self._pending_target: list[float] | None = None
        self._poses: deque[tuple[float, np.ndarray, np.ndarray]] = deque(maxlen=300)
        self._poses_lock = threading.Lock()
        self.message = (
            "Ready: click ground (simulation truth)" if self._truth_localization
            else "Click ground (SLAM localization)"
        ) if self._enabled else "Point navigation requires the nav Product and a saved map"

    def observe_pose(self, stamp_s: float, position: np.ndarray, rotation: np.ndarray) -> None:
        with self._poses_lock:
            self._poses.append((stamp_s, position.copy(), rotation.copy()))

    def submit(self, point: np.ndarray) -> None:
        if self._closed:
            raise ValueError("Viewer is closed")
        if not self._enabled:
            raise ValueError("Point navigation requires the nav Product and a saved map")
        target = self._map_target(point).tolist()
        if self._future is not None:
            self._pending_target = target
            self.message = f"Latest goal queued ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})"
            return
        self.message = "Submitting goal..."
        self._future = self._executor.submit(self._send, target)

    def _map_target(self, point: np.ndarray) -> np.ndarray:
        rotation, translation = self.world_to_map_transform()
        return rotation @ point + translation

    def world_to_map_transform(self) -> tuple[np.ndarray, np.ndarray]:
        """Use the same matched pose for picking and inverse overlay rendering."""
        if self._truth_localization:
            # The resolved native fixture publishes world poses and identity map<-odom.
            return np.eye(3), np.zeros(3)
        try:
            slam = json.loads(self._slam_path.read_text(encoding="utf-8"))
            age = time.time() - float(slam["snapshot_written_at_s"])
            if (slam.get("native_product") or {}).get("product_session_id") != self._session_id or not 0 <= age <= self._max_age_s:
                raise ValueError("Localization is stale or belongs to another session")
            if not slam.get("has_odom"):
                state = slam.get("state") or "initializing"
                reason = slam.get("reason") or "odometry unavailable"
                raise ValueError(f"Localization unavailable: {state} / {reason}")
            if not 0 <= time.time() - float(slam["stamp_s"]) <= self._max_age_s:
                raise ValueError("Localization pose is stale")
            with self._poses_lock:
                poses = tuple(self._poses)
            if not poses:
                raise ValueError("Waiting for simulation pose history")
            stamp, body_position, body_rotation = min(
                poses, key=lambda pose: abs(pose[0] - float(slam["stamp_s"])),
            )
            # Match the delayed estimator pose, rather than the current moving body.
            if abs(stamp - float(slam["stamp_s"])) > 0.05:
                raise ValueError("No simulation pose close to the localization timestamp")
            translation = world_point_to_map(np.zeros(3), body_position, body_rotation, slam)
            rotation = _rotation(slam["map_odom_tf"]) @ _rotation(slam["odometry"]["pose"]) @ body_rotation.T
            return rotation, translation
        except (OSError, KeyError, TypeError) as exc:
            raise ValueError("Localization is not ready for point navigation") from exc

    def poll(self) -> str:
        if self._future is not None and self._future.done():
            try:
                self.message = self._future.result()
            except (http.client.HTTPException, json.JSONDecodeError, TimeoutError, OSError) as exc:
                self.message = f"Goal reply unavailable; check navigation state: {exc}"
            except Exception as exc:
                self.message = f"Goal rejected: {exc}"
            self._future = None
            if self._pending_target is not None and not self._closed:
                target, self._pending_target = self._pending_target, None
                self.message = "Submitting latest goal..."
                self._future = self._executor.submit(self._send, target)
        return self.message

    def _request(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        endpoint = urlsplit(self._base_url)
        connection = http.client.HTTPConnection(endpoint.hostname, endpoint.port, timeout=2.0)
        with self._connection_lock:
            if self._closed:
                raise OSError("Viewer closed; request cancelled")
        try:
            connection.connect()
            with self._connection_lock:
                if self._closed:
                    raise OSError("Viewer closed; request cancelled")
                connection.sock = _GoalSocket(fileno=connection.sock.detach())
                connection.sock.cancelled = self._cancelled
                self._socket = connection.sock
                connection.sock.settimeout(2.0 if payload is None else 12.0)
            connection.request(
                "GET" if payload is None else "POST", path,
                body=None if payload is None else json.dumps(payload).encode(),
                headers={"Content-Type": "application/json"},
            )
            response = connection.getresponse()
            detail = response.read().decode("utf-8")
            if response.status < 400:
                return json.loads(detail)
            if response.status >= 500:
                raise OSError(f"HTTP {response.status}: goal reply unavailable")
            try:
                result = json.loads(detail)
                error_detail = result.get("detail") or {}
                blockers = error_detail.get("blockers") or []
                detail = "; ".join(str(item) for item in blockers) or str(
                    result.get("message") or result.get("error") or detail
                )
            except (ValueError, AttributeError, TypeError):
                pass
            raise ValueError(f"HTTP {response.status}: {detail[:240]}")
        finally:
            with self._connection_lock:
                self._socket = None
            connection.close()

    def send_map_goal(self, target: list[float]) -> dict[str, Any]:
        """Submit a map-space pick and retain its Gateway task acknowledgement."""
        session = self._request("/api/v1/session")
        if session.get("env") != "sim" or session.get("product_session_id") != self._session_id:
            raise ValueError("Gateway is not the current simulation session")
        # A new click explicitly returns control to navigation. Native resume
        # rejects an active operator or emergency stop and never replays motion.
        resumed = self._request("/api/v1/navigation/resume", {"client_id": "mujoco-viewer"})
        if resumed.get("accepted") is not True:
            raise ValueError(str(resumed.get("message") or resumed.get("error") or "Release movement keys before clicking"))
        if resumed.get("resume_was_required") is True:
            for _ in range(20):
                status = self._request("/api/v1/navigation/status")
                control = status.get("control") or {}
                if control.get("resume_required") is False and control.get("authority") != "OPERATOR":
                    break
                if self._cancelled.wait(0.05):
                    raise OSError("Viewer closed; request cancelled")
            else:
                raise ValueError("Navigation remains held; release movement keys before clicking")
        result = self._request("/api/v1/navigate/click", {
            "x": target[0], "y": target[1], "z": target[2],
            "source": "map_click", "target_type": "map_point",
            "label": "mujoco_click", "client_id": "mujoco-viewer",
        })
        if result.get("ok") is False or result.get("accepted") is not True:
            raise ValueError(str(result.get("message") or result.get("error") or "Navigation is not ready"))
        return result

    def _send(self, target: list[float]) -> str:
        self.send_map_goal(target)
        return f"Goal submitted ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}); watch navigation status"

    def close(self) -> None:
        with self._connection_lock:
            self._closed = True
            self._pending_target = None
            self._cancelled.set()
            if self._socket is not None:
                try:
                    self._socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        # Interrupt an in-flight reply; a connection attempt has a short timeout.
        self._executor.shutdown(wait=True, cancel_futures=True)
