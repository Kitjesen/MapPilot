"""runtime.worker_manager — Pool of Worker subprocesses.

Usage::

    mgr = WorkerManager(n_workers=4)
    mgr.start()
    mgr.deploy(worker_id=0, module_cls=MyModule, module_id="my_mod")
    mgr.setup(worker_id=0, module_id="my_mod")
    mgr.start_module(worker_id=0, module_id="my_mod")
    result = mgr.rpc_call(worker_id=0, module_id="my_mod", method="get_value")
    mgr.shutdown()
"""
import logging
import multiprocessing
from typing import Any

from runtime.worker import Worker

logger = logging.getLogger(__name__)

_DEFAULT_TIMEOUT = 30.0


class WorkerManager:
    """Pool of Worker subprocesses.

    Each worker runs in an isolated process. Modules are deployed to workers
    by index. All IPC is synchronous request/response over multiprocessing.Queue.
    """

    def __init__(self, n_workers: int = 4):
        self._n_workers = n_workers
        self._workers: dict[int, Worker] = {}
        self._cmd_queues: dict[int, multiprocessing.Queue] = {}
        self._resp_queues: dict[int, multiprocessing.Queue] = {}
        self._started = False

    def start(self) -> None:
        """Start all worker processes."""
        if self._started:
            return
        for i in range(self._n_workers):
            cmd_q: multiprocessing.Queue = multiprocessing.Queue()
            resp_q: multiprocessing.Queue = multiprocessing.Queue()
            w = Worker(worker_id=str(i), cmd_queue=cmd_q, resp_queue=resp_q)
            w.start()
            self._workers[i] = w
            self._cmd_queues[i] = cmd_q
            self._resp_queues[i] = resp_q
        self._started = True
        logger.info("WorkerManager: %d workers started", self._n_workers)

    def _send(self, worker_id: int, msg: tuple,
              timeout: float = _DEFAULT_TIMEOUT) -> Any:
        """Send a command and block until the worker responds.

        Returns the raw (status, payload) tuple.
        Raises TimeoutError if the worker does not respond within timeout.
        Raises RuntimeError if the worker returns an ERROR response.
        """
        self._cmd_queues[worker_id].put(msg)
        try:
            resp = self._resp_queues[worker_id].get(timeout=timeout)
        except Exception:
            raise TimeoutError(
                f"Worker {worker_id} timed out after {timeout}s on command {msg[0]}"
            ) from None
        if resp[0] == "ERROR":
            raise RuntimeError(f"Worker {worker_id}: {resp[1]}")
        return resp

    def deploy(self, worker_id: int, module_cls: type, module_id: str,
               args: tuple = (), kwargs: dict | None = None) -> None:
        """Instantiate a Module class inside a worker process."""
        self._send(worker_id, ("DEPLOY", module_cls, module_id, args, kwargs or {}))

    def setup(self, worker_id: int, module_id: str) -> None:
        """Call module.setup() in the worker."""
        self._send(worker_id, ("SETUP", module_id))

    def start_module(self, worker_id: int, module_id: str) -> None:
        """Call module.start() in the worker."""
        self._send(worker_id, ("START", module_id))

    def stop_module(self, worker_id: int, module_id: str) -> None:
        """Call module.stop() in the worker."""
        self._send(worker_id, ("STOP", module_id))

    def rpc_call(self, worker_id: int, module_id: str, method: str,
                 kwargs: dict | None = None,
                 timeout: float = _DEFAULT_TIMEOUT) -> Any:
        """Call an @rpc method on a module running in a worker.

        Returns the method's return value, or None if the response has no payload.
        """
        resp = self._send(
            worker_id,
            ("RPC_CALL", module_id, method, (), kwargs or {}),
            timeout,
        )
        return resp[1] if resp[0] == "RESULT" else None

    def health(self, worker_id: int, module_id: str) -> dict:
        """Return health/port_summary dict for a module in a worker."""
        resp = self._send(worker_id, ("HEALTH", module_id))
        return resp[1] if resp[0] == "RESULT" else {}

    def list_modules(self, worker_id: int) -> list[str]:
        """Return list of module_ids deployed to a worker."""
        resp = self._send(worker_id, ("LIST",))
        return resp[1] if resp[0] == "RESULT" else []

    def get_skills(self, worker_id: int, module_id: str) -> list[dict]:
        """Return serialized SkillInfo list for a module in a worker."""
        resp = self._send(worker_id, ("GET_SKILLS", module_id))
        return resp[1] if resp[0] == "RESULT" else []

    def bind_port(self, worker_id: int, module_id: str,
                  port_name: str, direction: str, topic: str) -> None:
        """Bind a port inside a worker to SHM transport on the given topic.

        direction: "out" | "in"
        Sends BIND_PORT IPC command to the worker; the worker attaches the
        port to the standard SHM transport adapter using the given topic.
        """
        self._send(worker_id, ("BIND_PORT", module_id, port_name, direction, topic))

    def shutdown(self) -> None:
        """Shutdown all workers gracefully, then join their processes."""
        for wid in list(self._workers):
            try:
                self._send(wid, ("SHUTDOWN",), timeout=10.0)
            except (OSError, RuntimeError):
                pass
        for w in self._workers.values():
            w.join(timeout=5.0)
            if w.is_alive():
                w.terminate()
        self._workers.clear()
        self._cmd_queues.clear()
        self._resp_queues.clear()
        self._started = False
        logger.info("WorkerManager: all workers shut down")

    def get_pid(self, worker_id: int) -> int | None:
        """Return the OS PID of a worker process, or None if not started."""
        w = self._workers.get(worker_id)
        return w.pid if w is not None else None

    @property
    def n_workers(self) -> int:
        """Number of worker processes configured."""
        return self._n_workers

    @property
    def is_started(self) -> bool:
        """True if start() has been called and workers are running."""
        return self._started
