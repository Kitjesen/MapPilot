"""Configured SDK credentials and TLS for the gRPC listener."""

import hmac
import os
from pathlib import Path

import grpc


class APIKeyInterceptor(grpc.ServerInterceptor):
    def __init__(self, key: str):
        self._key = key.encode("utf-8")

    def intercept_service(self, continuation, call_details):
        handler = continuation(call_details)
        if handler is None:
            return None
        provided = [value for key, value in call_details.invocation_metadata if key == "x-api-key"]
        if len(provided) == 1 and isinstance(provided[0], str) and hmac.compare_digest(
            provided[0].encode("utf-8"), self._key,
        ):
            return handler

        def reject(request, context):
            context.abort(grpc.StatusCode.UNAUTHENTICATED, "Invalid or missing SDK API key")

        factory = {
            (False, False): grpc.unary_unary_rpc_method_handler,
            (False, True): grpc.unary_stream_rpc_method_handler,
            (True, False): grpc.stream_unary_rpc_method_handler,
            (True, True): grpc.stream_stream_rpc_method_handler,
        }[(handler.request_streaming, handler.response_streaming)]
        return factory(
            reject,
            request_deserializer=handler.request_deserializer,
            response_serializer=handler.response_serializer,
        )


def api_key() -> str | None:
    key = os.environ.get("LINGTU_GRPC_API_KEY") or None
    if key is None and os.environ.get("LINGTU_GRPC_REQUIRE_API_KEY") == "1":
        raise ValueError("LINGTU_GRPC_API_KEY is required")
    if key is not None and (not key.isascii() or not key.strip() or any(ord(ch) < 32 or ord(ch) > 126 for ch in key)):
        raise ValueError("LINGTU_GRPC_API_KEY must contain printable ASCII characters")
    return key


def tls_credentials():
    certificate_file = os.environ.get("LINGTU_GRPC_TLS_CERT_FILE", "")
    key_file = os.environ.get("LINGTU_GRPC_TLS_KEY_FILE", "")
    if bool(certificate_file) != bool(key_file):
        raise ValueError("gRPC TLS certificate and key files must be configured together")
    if not certificate_file:
        return None
    try:
        certificate = Path(certificate_file).read_bytes()
        key = Path(key_file).read_bytes()
    except OSError:
        raise ValueError("Cannot read gRPC TLS certificate or key file") from None
    if not certificate or not key:
        raise ValueError("gRPC TLS certificate and key files must not be empty")
    return grpc.ssl_server_credentials(((key, certificate),))
