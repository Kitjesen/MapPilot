"""
VLA Navigation ROS2 Node — End-to-end embodied navigation.

Replaces the modular semantic_planner + semantic_perception pipeline
with a single VLA model that directly maps RGB + instruction → waypoints.

Architecture (VLingNav, arXiv 2601.08665):
  RGB stream → Dynamic FPS → Qwen2.5-VL-3B → AdaCoT trigger
  → [THINK: CoT reasoning → VLingMem store]
  → Action Head → waypoint trajectory → PoseStamped

Subscriptions:
  - RGB image           (sensor_msgs/Image)
  - Depth image         (sensor_msgs/Image)  — for obstacle awareness
  - Camera info         (sensor_msgs/CameraInfo)
  - Instruction         (std_msgs/String, JSON)
  - Cancel              (std_msgs/String)

Publications:
  - Goal waypoint       (geometry_msgs/PoseStamped)
  - Trajectory          (nav_msgs/Path)
  - Status              (std_msgs/String, JSON)

TF2:
  - Listens to camera_link → map transform for world-frame conversion
"""

import json
import logging
import math
import time
import traceback
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

import tf2_ros
from rclpy.duration import Duration

logger = logging.getLogger(__name__)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class VLANavNode(Node):
    """
    ROS2 node for VLA-based semantic navigation.

    This node runs the full VLingNav inference pipeline:
      1. Receives RGB frames and buffers them
      2. On receiving an instruction, starts navigation
      3. At control_hz, runs VLA inference → publishes waypoint
      4. Monitors progress and publishes status
    """

    def __init__(self):
        super().__init__("vla_nav_node")

        # ---- Declare parameters ----
        self._declare_all_params()

        # ---- Read parameters ----
        self._read_params()

        # ---- TF2 ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- CvBridge ----
        from cv_bridge import CvBridge
        self._bridge = CvBridge()

        # ---- Model (lazy load) ----
        self._model = None
        self._model_loaded = False
        self._loading = False

        # ---- State ----
        self._active = False
        self._instruction = ""
        self._robot_position = np.zeros(3)
        self._robot_heading = 0.0
        self._step_count = 0
        self._last_inference_time = 0.0

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscriptions ----
        self._sub_rgb = self.create_subscription(
            Image, self._topic_rgb, self._rgb_callback, sensor_qos,
        )
        self._sub_instruction = self.create_subscription(
            String, self._topic_instruction, self._instruction_callback, 10,
        )
        self._sub_cancel = self.create_subscription(
            String, self._topic_cancel, self._cancel_callback, 10,
        )

        # ---- Publishers ----
        self._pub_waypoint = self.create_publisher(
            PoseStamped, self._topic_waypoint, 10,
        )
        self._pub_trajectory = self.create_publisher(
            Path, self._topic_trajectory, 10,
        )
        self._pub_status = self.create_publisher(
            String, self._topic_status, 10,
        )

        # ---- Control timer ----
        control_period = 1.0 / max(self._control_hz, 0.1)
        self._control_timer = self.create_timer(control_period, self._control_callback)

        # ---- Status timer ----
        self._status_timer = self.create_timer(1.0, self._status_callback)

        self.get_logger().info(
            "VLA Nav node initialized (control_hz=%.1f, model=%s)",
            self._control_hz,
            self._backbone_name,
        )

        # Start model loading in background
        self._schedule_model_load()

    # ---- Parameter declaration -----------------------------------------------

    def _declare_all_params(self):
        self.declare_parameter("model.backbone", "Qwen/Qwen2.5-VL-3B-Instruct")
        self.declare_parameter("model.quantized_path", "")
        self.declare_parameter("model.device", "cuda")
        self.declare_parameter("model.dtype", "float16")

        self.declare_parameter("dynamic_fps.fs_max", 10.0)
        self.declare_parameter("dynamic_fps.stability", 5.0)
        self.declare_parameter("dynamic_fps.max_frames", 32)

        self.declare_parameter("adacot.enabled", True)
        self.declare_parameter("adacot.trigger_threshold", 0.5)
        self.declare_parameter("adacot.max_cot_tokens", 128)
        self.declare_parameter("adacot.temperature", 0.3)

        self.declare_parameter("vlingmem.enabled", True)
        self.declare_parameter("vlingmem.max_entries", 100)
        self.declare_parameter("vlingmem.top_k_retrieval", 5)

        self.declare_parameter("action_head.horizon", 5)
        self.declare_parameter("action_head.max_linear_step", 0.5)
        self.declare_parameter("action_head.max_angular_step", 0.785)

        self.declare_parameter("inference.control_hz", 3.0)
        self.declare_parameter("inference.warmup_frames", 5)

        self.declare_parameter("topics.rgb_image", "/camera/color/image_raw")
        self.declare_parameter("topics.instruction", "/nav/semantic/instruction")
        self.declare_parameter("topics.waypoint_out", "/nav/semantic/goal")
        self.declare_parameter("topics.trajectory_out", "/nav/semantic/trajectory")
        self.declare_parameter("topics.status_out", "/nav/semantic/status")
        self.declare_parameter("topics.cancel", "/nav/semantic/cancel")

        self.declare_parameter("tf.camera_frame", "camera_link")
        self.declare_parameter("tf.base_frame", "base_link")
        self.declare_parameter("tf.world_frame", "map")
        self.declare_parameter("tf.timeout_sec", 0.5)

    def _read_params(self):
        self._backbone_name = self.get_parameter("model.backbone").value
        self._quantized_path = self.get_parameter("model.quantized_path").value
        self._device = self.get_parameter("model.device").value

        self._control_hz = self.get_parameter("inference.control_hz").value
        self._warmup_frames = self.get_parameter("inference.warmup_frames").value

        self._topic_rgb = self.get_parameter("topics.rgb_image").value
        self._topic_instruction = self.get_parameter("topics.instruction").value
        self._topic_waypoint = self.get_parameter("topics.waypoint_out").value
        self._topic_trajectory = self.get_parameter("topics.trajectory_out").value
        self._topic_status = self.get_parameter("topics.status_out").value
        self._topic_cancel = self.get_parameter("topics.cancel").value

        self._camera_frame = self.get_parameter("tf.camera_frame").value
        self._base_frame = self.get_parameter("tf.base_frame").value
        self._world_frame = self.get_parameter("tf.world_frame").value
        self._tf_timeout = self.get_parameter("tf.timeout_sec").value

    # ---- Model loading -------------------------------------------------------

    def _schedule_model_load(self):
        """Load model in a one-shot timer to avoid blocking constructor."""
        self._load_timer = self.create_timer(0.1, self._load_model_once)

    def _load_model_once(self):
        """Load the VLA model (called once from timer)."""
        self._load_timer.cancel()

        if self._model_loaded or self._loading:
            return

        self._loading = True
        self.get_logger().info("Loading VLA model...")

        try:
            from vla_nav.model.vla_model import VLANavModel

            config = {
                "model": {
                    "backbone": self._backbone_name,
                    "device": self._device,
                    "dtype": self.get_parameter("model.dtype").value,
                },
                "dynamic_fps": {
                    "fs_max": self.get_parameter("dynamic_fps.fs_max").value,
                    "stability": self.get_parameter("dynamic_fps.stability").value,
                    "max_frames": self.get_parameter("dynamic_fps.max_frames").value,
                },
                "adacot": {
                    "enabled": self.get_parameter("adacot.enabled").value,
                    "trigger_threshold": self.get_parameter("adacot.trigger_threshold").value,
                    "max_cot_tokens": self.get_parameter("adacot.max_cot_tokens").value,
                    "temperature": self.get_parameter("adacot.temperature").value,
                },
                "vlingmem": {
                    "enabled": self.get_parameter("vlingmem.enabled").value,
                    "max_entries": self.get_parameter("vlingmem.max_entries").value,
                    "top_k_retrieval": self.get_parameter("vlingmem.top_k_retrieval").value,
                },
                "action_head": {
                    "horizon": self.get_parameter("action_head.horizon").value,
                    "max_linear_step": self.get_parameter("action_head.max_linear_step").value,
                    "max_angular_step": self.get_parameter("action_head.max_angular_step").value,
                },
            }

            self._model = VLANavModel.from_config(config)

            quantized = self._quantized_path
            if quantized:
                self._model.load(quantized_path=quantized)
            else:
                self._model.load()

            self._model_loaded = True
            self.get_logger().info("VLA model loaded successfully")

        except Exception as e:
            self.get_logger().error(
                "Failed to load VLA model: %s\n%s", e, traceback.format_exc()
            )
        finally:
            self._loading = False

    # ---- Callbacks -----------------------------------------------------------

    def _rgb_callback(self, msg: Image):
        """Receive RGB frame and push to model buffer."""
        if not self._model_loaded:
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("CvBridge error: %s", e)
            return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._model.push_frame(cv_image, timestamp=stamp_sec)

        # Update robot pose from TF
        self._update_robot_pose(msg.header.stamp)

    def _instruction_callback(self, msg: String):
        """Receive navigation instruction and start task."""
        if not self._model_loaded:
            self.get_logger().warn("Model not loaded yet — ignoring instruction")
            return

        try:
            data = json.loads(msg.data)
            instruction = data.get("instruction", data.get("text", msg.data))
        except (json.JSONDecodeError, TypeError):
            instruction = msg.data

        self.get_logger().info("New instruction: %s", instruction[:100])

        self._model.reset()
        self._model.set_instruction(instruction)
        self._instruction = instruction
        self._active = True
        self._step_count = 0

        self._publish_status("ACTIVE", f"Navigating: {instruction[:50]}")

    def _cancel_callback(self, msg: String):
        """Cancel current navigation task."""
        reason = msg.data if msg.data else "User cancelled"
        self.get_logger().info("Navigation cancelled: %s", reason)
        self._active = False
        self._model.reset()
        self._publish_status("CANCELLED", reason)

    # ---- Control loop --------------------------------------------------------

    def _control_callback(self):
        """Main control loop — runs at control_hz."""
        if not self._active or not self._model_loaded:
            return

        t_now = time.monotonic()

        try:
            # Run VLA inference
            output = self._model.step(current_time=t_now)
            self._step_count += 1

            # Publish first waypoint as PoseStamped
            if output.world_waypoints:
                wx, wy, wtheta = output.world_waypoints[0]
                self._publish_waypoint(wx, wy, wtheta)

                # Publish full trajectory as Path
                self._publish_trajectory(output.world_waypoints)

            # Log
            adacot = output.adacot_result
            mode = "THINK" if adacot.should_think else "NO_THINK"
            self.get_logger().debug(
                "Step %d: %s (p=%.2f), %.1f ms, wp=(%.2f, %.2f)",
                self._step_count,
                mode,
                adacot.trigger_prob,
                output.inference_time_ms,
                output.world_waypoints[0][0] if output.world_waypoints else 0,
                output.world_waypoints[0][1] if output.world_waypoints else 0,
            )

        except Exception as e:
            self.get_logger().error(
                "VLA inference error: %s\n%s", e, traceback.format_exc()
            )

    # ---- TF2 -----------------------------------------------------------------

    def _update_robot_pose(self, stamp):
        """Get robot pose from TF2 (base_link → map)."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._base_frame,
                stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            self._robot_position = np.array([t.x, t.y, t.z])
            # Extract yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._robot_heading = math.atan2(siny_cosp, cosy_cosp)

            self._model.update_robot_pose(
                t.x, t.y, t.z, self._robot_heading
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass  # TF not yet available

    # ---- Publishers ----------------------------------------------------------

    def _publish_waypoint(self, x: float, y: float, theta: float):
        """Publish a goal waypoint as PoseStamped."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation = _yaw_to_quaternion(theta)
        self._pub_waypoint.publish(msg)

    def _publish_trajectory(self, world_waypoints: list):
        """Publish the full predicted trajectory as a Path."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame

        for wx, wy, wtheta in world_waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quaternion(wtheta)
            msg.poses.append(pose)

        self._pub_trajectory.publish(msg)

    def _publish_status(self, state: str, detail: str = ""):
        """Publish navigation status as JSON."""
        status = {
            "state": state,
            "detail": detail,
            "step": self._step_count,
            "instruction": self._instruction[:50],
        }

        if self._model_loaded and self._model is not None:
            stats = self._model.get_statistics()
            status["adacot"] = stats.get("adacot", {})
            status["memory"] = stats.get("memory", {})

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    def _status_callback(self):
        """Periodic status publishing."""
        if self._active:
            self._publish_status("NAVIGATING")
        elif self._model_loaded:
            self._publish_status("IDLE")
        else:
            self._publish_status("LOADING")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VLANavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
