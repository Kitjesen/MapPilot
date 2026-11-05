"""Request-scoped Brainstem observations using the native driver's endpoint."""

from collections.abc import Mapping
from contextlib import contextmanager
from pathlib import Path
from urllib.parse import urlsplit

import grpc
from google.protobuf.empty_pb2 import Empty

from .generated.brainstem_api.cms_pb2 import CmsState, Imu, Joint, MotorStatusResponse, Voltage


class Observations:
    def __init__(self, environment: Mapping[str, str]):
        self._environment = dict(environment)

    def _channel(self):
        config = self._environment
        target = config.get("LINGTU_DRIVER_TARGET", "").strip()
        if not target:
            raise ValueError("Brainstem observation target is not configured")
        files = [config.get(f"LINGTU_DRIVER_TLS_{name}_FILE", "") for name in ("CA", "CERT", "KEY")]
        server_name = config.get("LINGTU_DRIVER_TLS_SERVER_NAME", "")
        if any(files):
            if not all(files):
                raise ValueError("Brainstem observations require all three mTLS certificate files")
            ca, certificate, key = [Path(path).read_bytes() for path in files]
            if not all((ca, certificate, key)):
                raise ValueError("Brainstem observation certificate files must not be empty")
            credentials = grpc.ssl_channel_credentials(ca, key, certificate)
            options = [("grpc.ssl_target_name_override", server_name)] if server_name else []
            return grpc.secure_channel(target, credentials, options=options)
        if server_name:
            raise ValueError("Brainstem TLS server name requires mTLS certificate files")
        if urlsplit(f"//{target}").hostname not in {"localhost", "localhost.localdomain", "127.0.0.1", "::1"}:
            raise ValueError("Remote Brainstem observations require mTLS certificate files")
        return grpc.insecure_channel(target)

    @contextmanager
    def read(self, sensor: str, timeout: float):
        method, response = {
            "motors": ("GetMotorStatus", MotorStatusResponse), "voltages": ("GetVoltage", Voltage),
            "body": ("GetCmsState", CmsState),
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
    def watch(self, sensor: str):
        method, response = {
            "imu": ("ListenImu", Imu), "joints": ("ListenJoint", Joint), "body": ("ListenCmsState", CmsState),
        }[sensor]
        with self._channel() as channel:
            call = channel.unary_stream(
                f"/brainstem.api.v1.RobotControl/{method}",
                request_serializer=Empty.SerializeToString,
                response_deserializer=response.FromString,
            )(Empty())
            try:
                yield call
            finally:
                call.cancel()
