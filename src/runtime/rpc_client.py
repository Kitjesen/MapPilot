"""RPCClient — transparent proxy for remote Module method calls.

Acts as a stand-in for a Module deployed in a Worker process.
Method calls on @rpc-decorated methods are forwarded via IPC.
Port access returns RemoteOut/RemoteIn proxies.

Usage:
    proxy = RPCClient(worker_manager, worker_id=0, module_id="nav")
    result = proxy.navigate(x=5.0, y=3.0)  # RPC call
    proxy.start()  # lifecycle command
"""
from __future__ import annotations

import logging
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from runtime.module import SkillInfo

logger = logging.getLogger(__name__)


class RPCClient:
    """Transparent proxy for a Module in a Worker process.

    Intercepts attribute access and forwards @rpc method calls to the
    remote Worker via the WorkerManager IPC interface.
    """

    def __init__(
        self,
        worker_manager: Any,
        worker_id: int,
        module_id: str,
        rpc_methods: set[str] | None = None,
    ) -> None:
        self._mgr = worker_manager
        self._worker_id = worker_id
        self._module_id = module_id
        self._rpc_methods: set[str] = rpc_methods or set()
        self._timeout = 30.0

    def __getattr__(self, name: str) -> Any:
        # Prevent infinite recursion for private/dunder names.
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._rpc_methods:
            return self._make_rpc_caller(name)
        raise AttributeError(
            f"'{self._module_id}' has no RPC method '{name}'"
        )

    def _make_rpc_caller(self, method_name: str) -> Callable:
        """Return a callable that forwards keyword args to the remote method."""

        def caller(**kwargs: Any) -> Any:
            return self._mgr.rpc_call(
                self._worker_id,
                self._module_id,
                method_name,
                kwargs=kwargs,
                timeout=self._timeout,
            )

        caller.__name__ = method_name
        caller.__qualname__ = f"{self._module_id}.{method_name}"
        return caller

    # -- Lifecycle methods (non-RPC, direct worker commands) ------------------

    def setup(self) -> None:
        """Forward setup() to the remote Worker."""
        self._mgr.setup(self._worker_id, self._module_id)

    def start(self) -> None:
        """Forward start() to the remote Worker."""
        self._mgr.start_module(self._worker_id, self._module_id)

    def stop(self) -> None:
        """Forward stop() to the remote Worker."""
        self._mgr.stop_module(self._worker_id, self._module_id)

    def health(self) -> dict:
        """Return health dict from the remote Worker."""
        return self._mgr.health(self._worker_id, self._module_id)

    def get_skill_infos(self) -> list[SkillInfo]:
        """Fetch @skill metadata from the remote module via IPC GET_SKILLS.

        Returns a list of SkillInfo objects describing each @skill method.
        Used by MCPServerModule.on_system_modules() for dynamic tool discovery.
        """
        from runtime.module import SkillInfo
        raw = self._mgr.get_skills(self._worker_id, self._module_id)
        return [SkillInfo(**d) for d in raw]

    # -- Properties -----------------------------------------------------------

    @property
    def worker_id(self) -> int:
        return self._worker_id

    @property
    def module_id(self) -> str:
        return self._module_id

    @property
    def rpc_methods(self) -> set[str]:
        return self._rpc_methods

    def __repr__(self) -> str:
        return (
            f"RPCClient({self._module_id}@worker-{self._worker_id},"
            f" rpcs={self._rpc_methods})"
        )
