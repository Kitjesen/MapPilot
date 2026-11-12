"""Brainstem observation values in public units; hardware layout stays here."""


def values(sensor, event):
    if sensor == "imu":
        if not event.HasField("gyroscope") or not event.HasField("quaternion"):
            raise ValueError("Brainstem IMU event is missing angular velocity or orientation")
        result = {"angular_velocity_rad_s": {axis: getattr(event.gyroscope, axis) for axis in "xyz"},
                  "orientation_world_to_body": {axis: getattr(event.quaternion, axis) for axis in "wxyz"}}
        if event.HasField("linear_acceleration"):
            result["linear_acceleration_m_s2"] = {axis: getattr(event.linear_acceleration, axis) for axis in "xyz"}
    elif sensor == "joints":
        kind = event.WhichOneof("data")
        if kind == "single_joint":
            joint = event.single_joint
            if joint.id >= 16:
                raise ValueError("Brainstem joint ID must be between 0 and 15")
            rows = [(joint.id, joint.position, joint.velocity, joint.torque, joint.status)]
        elif kind == "all_joints":
            arrays = [getattr(event.all_joints, field).values for field in ("position", "velocity", "torque", "status")]
            if any(len(items) != 16 for items in arrays):
                raise ValueError("Brainstem joint frame requires 16 positions, velocities, torques and statuses")
            rows = [(index, *row) for index, row in enumerate(zip(*arrays))]
        else:
            raise ValueError("Brainstem joint event is missing joint data")
        result = {"joints": [dict(zip(("id", "position_rad", "velocity_rad_s", "torque_nm", "status_code"), row)) for row in rows]}
    elif sensor == "motors":
        if len(event.motors) != 16 or {motor.id for motor in event.motors} != set(range(16)):
            raise ValueError("Brainstem motor status requires unique IDs 0 through 15")
        return {"motors": [{"id": motor.id, "online": motor.online, "status_code": motor.status_code,
                            "temperature_c": motor.temperature, "errors": list(motor.errors)} for motor in event.motors]}
    elif sensor == "voltages":
        if len(event.values) != 16:
            raise ValueError("Brainstem motor voltage requires 16 values")
        return {"volts": list(event.values)}
    else:
        raise ValueError("unsupported_sensor")
    if event.HasField("timestamp"):
        result["source_elapsed"] = {"seconds": event.timestamp.seconds, "nanos": event.timestamp.nanos}
    return result
