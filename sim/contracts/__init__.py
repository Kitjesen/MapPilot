"""Backend-neutral simulation contracts shared by compilers and runtimes."""

from .timebase import (
    PhysicsSensorTimebaseViolation,
    physics_sensor_timebase_violation,
)

__all__ = [
    "PhysicsSensorTimebaseViolation",
    "physics_sensor_timebase_violation",
]
