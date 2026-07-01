"""Legacy ROS 2 camera snapshot helper.

This adapter is a compatibility fallback for old deployments where the Gateway
route has no in-process camera frame cache. Product runtime should prefer
CameraBridgeModule, TeleopModule, or another Module-port camera source.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path


def capture_compressed_camera_snapshot(
    *,
    topic: str = "/camera/color/image_raw/compressed",
    timeout_s: float = 6.0,
    spin_timeout_s: float = 2.0,
    ros_setup: str = "/opt/ros/humble/setup.bash",
) -> bytes | None:
    """Capture one compressed ROS camera frame as JPEG bytes."""

    out = Path(tempfile.gettempdir()) / "lingtu_cam_snap.jpg"
    script = Path(tempfile.gettempdir()) / "lingtu_cam_snap.py"
    script.write_text(
        "\n".join(
            [
                "import rclpy, time",
                "from sensor_msgs.msg import CompressedImage",
                "rclpy.init()",
                "node = rclpy.create_node('cam_snap')",
                "msg = [None]",
                (
                    "node.create_subscription("
                    f"CompressedImage, {topic!r}, lambda m: msg.__setitem__(0, m), 1)"
                ),
                "deadline = time.time() + " + repr(float(spin_timeout_s)),
                "while msg[0] is None and time.time() < deadline:",
                "    rclpy.spin_once(node, timeout_sec=0.1)",
                "node.destroy_node()",
                "rclpy.shutdown()",
                f"open({json.dumps(str(out))}, 'wb').write(msg[0].data) if msg[0] else None",
                "",
            ]
        ),
        encoding="utf-8",
    )

    env = os.environ.copy()
    # The real camera deployment uses Fast DDS; avoid inheriting a conflicting
    # RMW setting from the Gateway process.
    env.pop("RMW_IMPLEMENTATION", None)
    subprocess.run(
        ["bash", "-c", f"source {ros_setup} && python3 {script}"],
        capture_output=True,
        timeout=timeout_s,
        env=env,
    )
    if out.is_file() and out.stat().st_size > 100:
        return out.read_bytes()
    return None
