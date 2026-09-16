"""Thunder flat G2 model_4998's single-frame control contract (simulation only)."""

import numpy as np

STANDING_POSE = np.array(
    (-0.20, -0.93, 1.96, 0.20, 0.93, -1.96,
     0.20, 0.93, -1.96, -0.20, -0.93, 1.96, 0.0, 0.0, 0.0, 0.0),
    dtype=np.float64,
)
ACTION_SCALE = np.array((0.125, 0.25, 0.25) * 4 + (5.0,) * 4)
KP = np.array((90.0,) * 12 + (0.0,) * 4)
KD = np.array((6.93,) * 12 + (1.0,) * 4)


class VelocityFeedback:
    """Bounded PI tracking around the flat policy's velocity reference in MuJoCo."""

    _INTEGRAL_LIMIT = np.array((0.1, 0.2, 0.1))
    _CORRECTION_LIMIT = np.array((0.2, 0.25, 0.3))

    def __init__(self, command_limits):
        self._command_limits = np.asarray(command_limits, dtype=np.float64)
        self.integral = np.zeros(3)
        self._previous_reference = np.zeros(3)

    def reset(self):
        self.integral[:] = 0.0
        self._previous_reference[:] = 0.0

    def update(self, reference, measured, dt):
        reference = np.asarray(reference)
        if np.linalg.norm(reference) <= 1e-4:
            self.reset()
            return np.zeros(3)
        # An ended or reversed axis must not inherit its old drive bias. Keep
        # compensation on unchanged axes that are rejecting drift.
        changed_axis = (np.abs(self._previous_reference) > 1e-4) & (
            (np.abs(reference) <= 1e-4) | (reference * self._previous_reference < 0)
        )
        self.integral[changed_axis] = 0.0
        self._previous_reference = reference.copy()
        error = reference - measured
        candidate = np.clip(
            self.integral + error * dt, -self._INTEGRAL_LIMIT, self._INTEGRAL_LIMIT
        )
        raw_correction = error + candidate
        unlimited = reference + raw_correction
        correction = np.clip(raw_correction, -self._CORRECTION_LIMIT, self._CORRECTION_LIMIT)
        limited = np.clip(reference + correction, -self._command_limits, self._command_limits)
        # Integrate only when it does not push further into either saturation.
        # Error of the opposite sign can still unwind an existing bias.
        self.integral = np.where(error * (unlimited - limited) > 0, self.integral, candidate)
        correction = np.clip(error + self.integral, -self._CORRECTION_LIMIT, self._CORRECTION_LIMIT)
        return np.clip(reference + correction, -self._command_limits, self._command_limits)


def observation(gyro, gravity, command, joint_position, joint_velocity, previous_action):
    """Build the training observation from base-frame and Dart-ordered inputs."""
    return np.concatenate((
        np.clip(gyro, -100.0, 100.0) * 0.25,
        np.clip(gravity, -100.0, 100.0),
        np.clip(command, -100.0, 100.0),
        np.clip(np.asarray(joint_position)[:12] - STANDING_POSE[:12], -100.0, 100.0),
        np.clip(joint_velocity, -100.0, 100.0) * 0.05,
        np.clip(previous_action, -100.0, 100.0),
    )).astype(np.float32)


def action_targets(raw_action):
    """Map raw network actions to leg positions and wheel velocities."""
    return np.asarray(raw_action, dtype=np.float64) * ACTION_SCALE + STANDING_POSE
