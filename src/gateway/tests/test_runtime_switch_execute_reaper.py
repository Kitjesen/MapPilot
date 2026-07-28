import threading
import time


def test_dev_process_reaper_clears_pending_after_wait():
    import gateway.services.runtime_switch_execute as switch_execute
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._runtime_switch_pending = {
        "command_id": "dev-switch-reap",
        "started_monotonic": time.monotonic(),
    }
    release = threading.Event()
    entered = threading.Event()

    class WaitablePopen:
        def wait(self):
            entered.set()
            assert release.wait(timeout=2.0)
            return 0

    switch_execute._reap_dev_switch_process(gateway, "dev-switch-reap", WaitablePopen())

    assert entered.wait(timeout=1.0)
    assert getattr(gateway, "_runtime_switch_pending", None) is not None
    release.set()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and getattr(gateway, "_runtime_switch_pending", None) is not None:
        time.sleep(0.01)

    assert getattr(gateway, "_runtime_switch_pending", None) is None


def test_dev_process_reaper_clears_pending_after_wait_error():
    import gateway.services.runtime_switch_execute as switch_execute
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._runtime_switch_pending = {
        "command_id": "dev-switch-error-reap",
        "started_monotonic": time.monotonic(),
    }

    class FailingPopen:
        def wait(self):
            raise RuntimeError("wait failed")

    switch_execute._reap_dev_switch_process(gateway, "dev-switch-error-reap", FailingPopen())

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and getattr(gateway, "_runtime_switch_pending", None) is not None:
        time.sleep(0.01)

    assert getattr(gateway, "_runtime_switch_pending", None) is None