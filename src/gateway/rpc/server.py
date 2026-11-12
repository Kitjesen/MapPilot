"""gRPC listener owned by the Gateway lifecycle."""

from concurrent.futures import ThreadPoolExecutor
from threading import BoundedSemaphore

import grpc

from . import services
from .generated import localization_pb2_grpc as localization_rpc
from .generated import maps_pb2_grpc as maps_rpc
from .generated import motion_pb2_grpc as motion_rpc
from .generated import navigation_pb2_grpc as navigation_rpc
from .generated import observations_pb2_grpc as observations_rpc
from .generated import system_pb2_grpc as system_rpc
from .generated import tasks_pb2_grpc as tasks_rpc
from .security import APIKeyInterceptor, api_key, tls_credentials


def start(gateway, address: str):
    key = api_key()
    credentials = tls_credentials()
    interceptors = (APIKeyInterceptor(key),) if key else ()
    server = grpc.server(ThreadPoolExecutor(max_workers=16), interceptors=interceptors)
    # Long-lived synchronous streams share eight workers, leaving room for commands.
    watch_slots = BoundedSemaphore(8)
    system_rpc.add_SystemServicer_to_server(services.System(gateway, watch_slots), server)
    motion_rpc.add_MotionServicer_to_server(services.Motion(gateway, watch_slots), server)
    localization_rpc.add_LocalizationServicer_to_server(services.Localization(gateway, watch_slots), server)
    maps_rpc.add_MapsServicer_to_server(services.Maps(gateway, watch_slots), server)
    navigation_rpc.add_NavigationServicer_to_server(services.Navigation(gateway, watch_slots), server)
    observations_rpc.add_ObservationsServicer_to_server(services.Observations(gateway, watch_slots), server)
    tasks_rpc.add_TasksServicer_to_server(services.Tasks(gateway, watch_slots), server)
    port = server.add_secure_port(address, credentials) if credentials else server.add_insecure_port(address)
    if port == 0:
        raise RuntimeError(f"Cannot bind DOSO gRPC endpoint: {address}")
    server.start()
    return server
