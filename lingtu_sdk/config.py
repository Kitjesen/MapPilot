"""Configuration dataclass for the LingTu SDK."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LingTuConfig:
    """Connection settings for a LingTu robot Gateway instance.

    Attributes:
        host: Robot IP or hostname (default localhost for development).
        port: Gateway HTTP port (default 5050).
        mcp_port: MCP server port for AI-agent tool access (default 8090).
        api_key: Optional shared secret sent as X-API-Key header.
        timeout: Default HTTP request timeout in seconds (default 10.0).
    """

    host: str = "127.0.0.1"
    port: int = 5050
    mcp_port: int = 8090
    api_key: str | None = None
    timeout: float = 10.0

    @property
    def base_url(self) -> str:
        """Full HTTP base URL (``http://host:port``)."""
        return f"http://{self.host}:{self.port}"

    def merge(self, **overrides: object) -> LingTuConfig:
        """Return a new config with the given fields replaced."""
        d = {**self.__dict__, **overrides}
        return LingTuConfig(**d)
