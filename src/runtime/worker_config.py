"""Worker deployment configuration for cross-host DDS and multi-domain routing.

Provides :class:`WorkerDeployment` — a per-module deployment descriptor that
controls transport selection, remote host targeting, and DDS domain assignment
for worker subprocesses.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class WorkerDeployment:
    """Configuration for deploying a module to a (possibly remote) worker.

    Attributes:
        module_name: Name of the module (must match the Blueprint entry name).
        host: Target host. ``"localhost"`` means the local machine and keeps
            SHM as the default transport.  Any other value implies a remote
            deployment where DDS is required.
        transport: Explicit transport override.

            * ``"shm"``  — always use shared memory (local only).
            * ``"dds"``  — always use DDS (works cross-host).
            * ``"auto"`` — pick DDS when *host* is remote, SHM otherwise.
            * ``None``   — same as ``"auto"``.

        domain_id: DDS domain ID override for this worker.  ``None`` means
            use the process-wide default (typically 0 or the value from
            ``config/qos_profiles.yaml``).
        qos_profile: Optional named QoS profile from
            ``config/qos_profiles.yaml``.  ``None`` keeps the default QoS
            behaviour.
    """

    module_name: str
    host: str = "localhost"
    transport: str | None = None
    domain_id: int | None = None
    qos_profile: str | None = None

    # ------------------------------------------------------------------
    # Derived helpers
    # ------------------------------------------------------------------

    @property
    def is_remote(self) -> bool:
        """``True`` when the target host is *not* the local machine."""
        return self.host not in ("localhost", "127.0.0.1", "::1", "")

    def resolve_transport(self) -> str:
        """Return the concrete transport backend name (``"shm"`` or ``"dds"``).

        The resolution rules are:

        1. Explicit ``transport="shm"`` or ``"dds"`` is honoured as-is.
        2. ``transport="auto"`` or ``None`` → DDS when :pyattr:`is_remote`,
           SHM otherwise.
        """
        if self.transport in ("shm", "dds"):
            return self.transport
        # auto / None
        return "dds" if self.is_remote else "shm"

    def to_metadata(self) -> dict[str, Any]:
        """Serialize to a flat dict suitable for connection metadata."""
        return {
            "host": self.host,
            "transport": self.resolve_transport(),
            "domain_id": self.domain_id,
            "qos_profile": self.qos_profile,
        }


# ---------------------------------------------------------------------------
# Registry helper — maps module names to their WorkerDeployment
# ---------------------------------------------------------------------------


class WorkerDeploymentRegistry:
    """Simple name → :class:`WorkerDeployment` mapping.

    Blueprint collects deployments during the builder phase and consults the
    registry during ``_build_worker_mode`` to decide transports.
    """

    def __init__(self) -> None:
        self._deployments: dict[str, WorkerDeployment] = {}

    def register(self, deployment: WorkerDeployment) -> None:
        self._deployments[deployment.module_name] = deployment

    def get(self, module_name: str) -> WorkerDeployment | None:
        return self._deployments.get(module_name)

    def resolve_transport(self, module_name: str, *, default: str = "shm") -> str:
        """Resolve the transport for *module_name*, falling back to *default*."""
        dep = self._deployments.get(module_name)
        if dep is None:
            return default
        return dep.resolve_transport()

    def resolve_domain_id(self, module_name: str) -> int | None:
        dep = self._deployments.get(module_name)
        if dep is None:
            return None
        return dep.domain_id

    def all(self) -> dict[str, WorkerDeployment]:
        return dict(self._deployments)
