"""Expose only real IMU/joint observations and SDK handshake on loopback."""

import argparse
import os
from concurrent.futures import ThreadPoolExecutor
from threading import BoundedSemaphore
from types import SimpleNamespace

import grpc

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import observations_pb2 as observations_pb
from gateway.rpc.generated import system_pb2 as system_pb
from gateway.rpc.observations import Observations as ObservationService
from gateway.rpc.services import System
from runtime.adapters.brainstem.source import Observations


def start(target: str, *, port: int = 50051, lan_development: bool = False):
    environment = {**os.environ, "LINGTU_DRIVER_TARGET": target}
    if lan_development:
        environment["LINGTU_OBSERVATIONS_TRANSPORT"] = "lan-dev"
    source = Observations(environment)
    with source._channel():
        pass
    gateway = SimpleNamespace(_observations=source)
    slots = BoundedSemaphore(8)
    system = System(gateway, slots)
    observations = ObservationService(gateway, slots)
    server = grpc.server(ThreadPoolExecutor(max_workers=12))
    handlers = {
        "doso.v1.System": {"Handshake": grpc.unary_unary_rpc_method_handler(
            system.Handshake,
            request_deserializer=system_pb.Handshake.FromString,
            response_serializer=system_pb.HandshakeReply.SerializeToString,
        )},
        "doso.v1.Observations": {
            name: grpc.unary_stream_rpc_method_handler(
                getattr(observations, name),
                request_deserializer=common_pb.Empty.FromString,
                response_serializer=message.SerializeToString,
            ) for name, message in (("WatchImu", observations_pb.ImuSample),
                                   ("WatchJoints", observations_pb.JointSample))
        },
    }
    for service, methods in handlers.items():
        server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(service, methods),))
        server.add_registered_method_handlers(service, methods)
    bound_port = server.add_insecure_port(f"127.0.0.1:{port}")
    if not bound_port:
        raise RuntimeError(f"Cannot bind observation endpoint: 127.0.0.1:{port}")
    server.start()
    return server, bound_port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, help="Brainstem host:port")
    parser.add_argument("--port", type=int, default=50051, help="Local SDK listener port (default: 50051)")
    parser.add_argument("--lan-development", action="store_true", help="Explicit plaintext RFC1918 development transport")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        server, port = start(args.target, port=args.port, lan_development=args.lan_development)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f"Read-only observations listening on 127.0.0.1:{port}", flush=True)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop(0).wait(3)


if __name__ == "__main__":
    main()
