"""runtime.worker — Worker process hosting Module instances.

Each Worker runs in a separate process and manages one or more Modules.
Communication with the main process is via multiprocessing.Queue (command/response).

IPC Protocol:
    Main → Worker: (cmd, *args)
    Worker → Main: ("OK", result) | ("ERROR", message) | ("RESULT", value)

Commands:
    DEPLOY:   (module_cls, module_id, args, kwargs) → instantiate module
    SETUP:    (module_id,) → call module.setup()
    START:    (module_id,) → call module.start()
    STOP:     (module_id,) → call module.stop()
    RPC_CALL: (module_id, method_name, args, kwargs) → call @rpc method
    HEALTH:   (module_id,) → return module.health() or port_summary()
    LIST:     () → return list of module_ids
    SHUTDOWN: () → stop all modules, exit
"""
import logging
import multiprocessing
import os
import sys
import traceback
from typing import Any

logger = logging.getLogger(__name__)


class Worker(multiprocessing.Process):
    """Subprocess hosting Module instances.

    Runs an event loop that processes IPC commands from the main process.
    Each command produces exactly one response on the resp_queue.
    """

    def __init__(self, worker_id: str, cmd_queue: multiprocessing.Queue,
                 resp_queue: multiprocessing.Queue,
                 extra_sys_path: "list[str] | None" = None):
        super().__init__(daemon=True, name=f"worker-{worker_id}")
        self.worker_id = worker_id
        self._cmd_q = cmd_queue
        self._resp_q = resp_queue
        self._modules: dict[str, Any] = {}  # module_id → instance
        # Capture parent sys.path so the subprocess can import the same modules.
        # On Windows (spawn start method) the child process does not inherit the
        # parent's sys.path, causing pickle failures for classes defined in
        # project source trees (e.g. test_runtime._WorkerTestModule).
        self._extra_sys_path: list[str] = list(extra_sys_path or sys.path)

    def run(self) -> None:
        """Worker event loop — process commands until SHUTDOWN."""
        # Restore parent's sys.path before any imports or pickle operations.
        for p in reversed(self._extra_sys_path):
            if p not in sys.path:
                sys.path.insert(0, p)
        logger.info("Worker %s started (pid=%d)", self.worker_id, os.getpid())
        while True:
            try:
                msg = self._cmd_q.get(timeout=1.0)
            except Exception as _get_exc:
                if not isinstance(_get_exc, __import__('queue').Empty):
                    self._resp_q.put(("ERROR", f"get failed: {_get_exc!r}"))
                continue

            if msg is None:
                break

            cmd = msg[0]
            try:
                if cmd == "DEPLOY":
                    _, cls, mod_id, args, kwargs = msg
                    instance = cls(*args, **kwargs)
                    self._modules[mod_id] = instance
                    self._resp_q.put(("OK", mod_id))

                elif cmd == "SETUP":
                    _, mod_id = msg
                    self._modules[mod_id].setup()
                    self._resp_q.put(("OK", mod_id))

                elif cmd == "START":
                    _, mod_id = msg
                    self._modules[mod_id].start()
                    self._resp_q.put(("OK", mod_id))

                elif cmd == "STOP":
                    _, mod_id = msg
                    self._modules[mod_id].stop()
                    self._resp_q.put(("OK", mod_id))

                elif cmd == "RPC_CALL":
                    _, mod_id, method, _rpc_args, rpc_kwargs = msg
                    mod = self._modules[mod_id]
                    result = mod.call_rpc(method, **rpc_kwargs)
                    self._resp_q.put(("RESULT", result))

                elif cmd == "HEALTH":
                    _, mod_id = msg
                    mod = self._modules[mod_id]
                    # Use health() if available (e.g. NativeModule), else port_summary()
                    h = mod.health() if hasattr(mod, "health") else mod.port_summary()
                    self._resp_q.put(("RESULT", h))

                elif cmd == "LIST":
                    self._resp_q.put(("RESULT", list(self._modules.keys())))

                elif cmd == "GET_SKILLS":
                    _, mod_id = msg
                    mod = self._modules[mod_id]
                    infos = mod.get_skill_infos()
                    # Serialize to plain dicts — Queue cannot carry dataclass instances
                    result = [
                        {
                            "func_name": i.func_name,
                            "class_name": i.class_name,
                            "args_schema": i.args_schema,
                        }
                        for i in infos
                    ]
                    self._resp_q.put(("RESULT", result))

                elif cmd == "BIND_PORT":
                    _, mod_id, port_name, direction, topic = msg
                    mod = self._modules[mod_id]
                    from runtime.transport.factory import create_transport_adapter
                    t = create_transport_adapter("shm")
                    if direction == "out":
                        port = mod.ports_out.get(port_name)
                        if port is not None:
                            port._bind_transport(t, topic)
                    elif direction == "in":
                        port = mod.ports_in.get(port_name)
                        if port is not None:
                            t.subscribe(topic, port._deliver)
                    self._resp_q.put(("OK", port_name))

                elif cmd == "SHUTDOWN":
                    for _mod_id, mod in list(self._modules.items()):
                        try:
                            mod.stop()
                        except RuntimeError:
                            pass
                    self._modules.clear()
                    self._resp_q.put(("OK", "shutdown"))
                    break

                else:
                    self._resp_q.put(("ERROR", f"unknown command: {cmd}"))

            except Exception as e:
                self._resp_q.put(
                    ("ERROR", f"{cmd} failed: {e}\n{traceback.format_exc()}")
                )

        logger.info("Worker %s exiting", self.worker_id)
