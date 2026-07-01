"""Shared rclpy context for LingTu ROS2 bridge nodes.

Prevents multiple nodes from each running their own executor, which causes
DDS callback starvation on ROS2 Humble.

Usage:
    from compat.ros2.context import ensure_rclpy, get_shared_executor

    ensure_rclpy()
    node = Node("my_node")
    get_shared_executor().add_node(node)
    # Executor spin thread is auto-started on first add_node.
"""

from __future__ import annotations

import logging
import os
import threading

logger = logging.getLogger(__name__)

_lock = threading.Lock()
_executor = None
_spin_thread: threading.Thread | None = None
_running = False


def ensure_rclpy() -> None:
    """Initialize rclpy if not already done."""

    try:
        import rclpy

        if not rclpy.ok():
            rclpy.init()
    except ImportError:
        pass


def _executor_thread_count() -> int:
    raw = os.environ.get("LINGTU_ROS2_EXECUTOR_THREADS", "4")
    try:
        return max(1, int(raw))
    except (TypeError, ValueError):
        logger.warning(
            "Invalid LINGTU_ROS2_EXECUTOR_THREADS=%r; using 4 threads", raw
        )
        return 4


def _create_executor():
    """Create the preferred shared executor, with single-thread fallback."""

    ensure_rclpy()
    try:
        from rclpy.executors import MultiThreadedExecutor

        threads = _executor_thread_count()
        executor = MultiThreadedExecutor(num_threads=threads)
        logger.info("Shared ROS2 MultiThreadedExecutor started (%d threads)", threads)
        return executor
    except (ImportError, TypeError) as e:
        logger.warning(
            "ROS2 MultiThreadedExecutor unavailable (%s); falling back to single thread",
            e,
        )
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        logger.info("Shared ROS2 SingleThreadedExecutor started")
        return executor


def get_shared_executor():
    """Get the singleton shared executor. Auto-starts spin thread."""

    global _executor, _spin_thread, _running
    with _lock:
        if _executor is None:
            _executor = _create_executor()
            _running = True
            _spin_thread = threading.Thread(
                target=_spin_loop, daemon=True, name="ros2_shared_spin"
            )
            _spin_thread.start()
    return _executor


def shutdown_shared_executor() -> None:
    """Shutdown the shared executor (call on system stop)."""

    global _executor, _spin_thread, _running
    with _lock:
        _running = False
        executor = _executor
        if _spin_thread:
            _spin_thread.join(timeout=3)
            _spin_thread = None
        if executor:
            executor.shutdown()
        _executor = None


def _spin_loop() -> None:
    import rclpy

    while _running and rclpy.ok():
        executor = _executor
        if executor is None:
            break
        try:
            executor.spin_once(timeout_sec=0.02)
        except Exception as e:
            if _is_stale_destroyed_entity_error(e):
                logger.debug("ROS2 spin_once ignored stale destroyed entity: %s", e)
                continue
            logger.warning("ROS2 spin_once error: %s", e)


def _is_stale_destroyed_entity_error(exc: Exception) -> bool:
    msg = str(exc)
    return "Destroyable" in msg and "destruction was requested" in msg
