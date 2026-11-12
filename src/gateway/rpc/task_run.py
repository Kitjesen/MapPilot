"""Observe the current task owner without resubmitting or cancelling work."""

import threading
import time

import grpc


def watch(context, slots, read, terminal=None):
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    try:
        stopped = threading.Event()
        if not context.add_callback(stopped.set):
            return
        next_tick = time.monotonic()
        while context.is_active():
            run = read()
            yield run
            if not run.found:
                return
            if terminal is not None:
                finished = terminal(run)
            else:
                state = run.details.fields.get("state")
                finished = state is not None and state.string_value in {"done", "failed", "cancelled"}
            if finished:
                return
            next_tick += 0.02
            now = time.monotonic()
            if now - next_tick > 0.02:
                next_tick = now
            if stopped.wait(max(0.0, next_tick - now)):
                return
    finally:
        slots.release()
