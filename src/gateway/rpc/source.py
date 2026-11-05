"""Forward read-only source calls with request cancellation and shared stream slots."""

import threading

import grpc


class Source:
    def __init__(self, gateway, watch_slots=None):
        self.gw = gateway
        self._watch_slots = watch_slots if watch_slots is not None else threading.BoundedSemaphore(8)

    def _read(self, sensor, context):
        source = getattr(self.gw, "_observations", None)
        if source is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Robot observation source is not configured for this RunPlan")
        remaining = context.time_remaining()
        timeout = 10.0 if remaining is None else min(10.0, remaining)
        try:
            with source.read(sensor, timeout) as call:
                if not context.add_callback(call.cancel):
                    context.abort(grpc.StatusCode.CANCELLED, "Observation request cancelled")
                return call.result()
        except grpc.FutureCancelledError:
            context.abort(grpc.StatusCode.CANCELLED, "Observation request cancelled")
        except grpc.RpcError as error:
            context.set_trailing_metadata(error.trailing_metadata() or ())
            context.abort(error.code(), error.details())
        except (OSError, ValueError) as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))

    def _watch(self, sensor, convert, context):
        source = getattr(self.gw, "_observations", None)
        if source is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Robot observation source is not configured for this RunPlan")
        if not self._watch_slots.acquire(blocking=False):
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
        try:
            with source.watch(sensor) as call:
                if not context.add_callback(call.cancel):
                    return
                for event in call:
                    if not context.is_active():
                        return
                    yield convert(event)
        except grpc.RpcError as error:
            if context.is_active():
                context.set_trailing_metadata(error.trailing_metadata() or ())
                context.abort(error.code(), error.details())
        except (OSError, ValueError) as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))
        finally:
            self._watch_slots.release()
