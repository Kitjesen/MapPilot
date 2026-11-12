"""Request-scoped Brainstem observations using the native driver's endpoint."""

from collections.abc import Mapping
from contextlib import contextmanager
from ipaddress import IPv4Address, IPv4Network
from pathlib import Path
from urllib.parse import urlsplit

import grpc
from google.protobuf.empty_pb2 import Empty
from grpc.experimental import ChannelOptions

from .generated.brainstem_api.cms_pb2 import (
    CmsState,
    ControlStatus,
    Imu,
    Joint,
    MotorStatusResponse,
    ProfileInfo,
    Voltage,
)


class Observations:
    def __init__(self, environment: Mapping[str, str]):
        self._environment = dict(environment)

    def capabilities(self):
        return {"provider": "brainstem", "sensors": {
            "imu": {"stream": True, "fields": ["angular_velocity_rad_s", "orientation_world_to_body", "linear_acceleration_m_s2"], "clock": "source_elapsed"},
            "joints": {"fields": ["position_rad", "velocity_rad_s", "torque_nm", "status_code"], "joint_count": 16, "clock": "source_elapsed"},
            "motors": {"fields": ["online", "temperature_c", "status_code", "errors"], "motor_count": 16},
            "voltages": {"fields": ["volts"], "motor_count": 16},
        }}

    def sample(self, sensor: str, timeout: float = 2):
        from .telemetry import values

        if sensor in {"imu", "joints"}:
            with self.watch(sensor, timeout=timeout) as call:
                return values(sensor, next(call))
        with self.read(sensor, timeout) as call:
            return values(sensor, call.result())

    @contextmanager
    def stream(self, sensor: str):
        from .telemetry import values

        with self.watch(sensor) as call:
            yield (map(lambda event: values(sensor, event), call), call.cancel)

    def _channel(self):
        config = self._environment
        target = config.get("LINGTU_DRIVER_TARGET", "").strip()
        if not target:
            raise ValueError("Brainstem observation target is not configured")
        transport = config.get("LINGTU_OBSERVATIONS_TRANSPORT", "mtls").strip()
        if transport not in {"mtls", "lan-dev"}:
            raise ValueError("Brainstem observation transport must be mtls or lan-dev")
        files = [config.get(f"LINGTU_DRIVER_TLS_{name}_FILE", "") for name in ("CA", "CERT", "KEY")]
        server_name = config.get("LINGTU_DRIVER_TLS_SERVER_NAME", "")
        options = [(ChannelOptions.SingleThreadedUnaryStream, 1)]
        if server_name:
            options.append(("grpc.ssl_target_name_override", server_name))
        if transport == "lan-dev" and (any(files) or server_name):
            raise ValueError("Brainstem lan-dev observations cannot use TLS configuration")
        if any(files):
            if not all(files):
                raise ValueError("Brainstem observations require all three mTLS certificate files")
            ca, certificate, key = [Path(path).read_bytes() for path in files]
            if not all((ca, certificate, key)):
                raise ValueError("Brainstem observation certificate files must not be empty")
            credentials = grpc.ssl_channel_credentials(ca, key, certificate)
            return grpc.secure_channel(target, credentials, options=options)
        if server_name:
            raise ValueError("Brainstem TLS server name requires mTLS certificate files")
        hostname = urlsplit(f"//{target}").hostname
        if hostname not in {"localhost", "localhost.localdomain", "127.0.0.1", "::1"}:
            if transport != "lan-dev":
                raise ValueError("Remote Brainstem observations require mTLS certificate files")
            try:
                address = IPv4Address(hostname)
            except ValueError:
                raise ValueError("Brainstem lan-dev observations require a literal RFC1918 IPv4 address") from None
            if not any(address in IPv4Network(network) for network in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")):
                raise ValueError("Brainstem lan-dev observations require a literal RFC1918 IPv4 address")
        return grpc.insecure_channel(target, options=options)

    @contextmanager
    def read(self, sensor: str, timeout: float):
        method, response = {
            "motors": ("GetMotorStatus", MotorStatusResponse), "voltages": ("GetVoltage", Voltage),
            "body": ("GetCmsState", CmsState),
            "control": ("GetControlStatus", ControlStatus),
            "profile": ("GetProfile", ProfileInfo),
        }[sensor]
        with self._channel() as channel:
            call = channel.unary_unary(
                f"/brainstem.api.v1.RobotControl/{method}",
                request_serializer=Empty.SerializeToString,
                response_deserializer=response.FromString,
            ).future(Empty(), timeout=timeout)
            try:
                yield call
            finally:
                call.cancel()

    @contextmanager
    def watch(self, sensor: str, timeout: float | None = None):
        method, response = {
            "imu": ("ListenImu", Imu), "joints": ("ListenJoint", Joint), "body": ("ListenCmsState", CmsState),
        }[sensor]
        with self._channel() as channel:
            call = channel.unary_stream(
                f"/brainstem.api.v1.RobotControl/{method}",
                request_serializer=Empty.SerializeToString,
                response_deserializer=response.FromString,
            )(Empty(), timeout=timeout)
            try:
                yield call
            finally:
                call.cancel()
