"""Log filter that redacts secrets (API keys, tokens) from log output.

Usage::

    import logging
    from runtime.utils.redact import RedactSecretsFilter

    handler = logging.StreamHandler()
    handler.addFilter(RedactSecretsFilter())
    logging.getLogger().addHandler(handler)

Automatically detects environment variable values whose names match
``*_KEY``, ``*_TOKEN``, ``*_SECRET``, ``*_PASSWORD`` and replaces them
with ``***`` in log messages.
"""

from __future__ import annotations

import logging
import os
import re

# Env var name patterns that indicate secrets
_SECRET_PATTERNS = re.compile(
    r".*(_KEY|_TOKEN|_SECRET|_PASSWORD|_CREDENTIAL)$", re.IGNORECASE,
)

# Minimum length to avoid false positives on short values
_MIN_SECRET_LEN = 8


def _collect_secrets() -> list[tuple[str, str]]:
    """Scan environment for secret values to redact."""
    secrets = []
    for name, value in os.environ.items():
        if _SECRET_PATTERNS.match(name) and len(value) >= _MIN_SECRET_LEN:
            secrets.append((name, value))
    return secrets


class RedactSecretsFilter(logging.Filter):
    """Logging filter that replaces known secret values with '***'.

    Secrets are collected from environment variables at construction time.
    Call ``refresh()`` if new secrets are added at runtime.
    """

    def __init__(self, name: str = ""):
        super().__init__(name)
        self._secrets: list[tuple[str, str]] = []
        self.refresh()

    def refresh(self) -> None:
        """Re-scan environment for secrets."""
        self._secrets = _collect_secrets()

    def filter(self, record: logging.LogRecord) -> bool:
        if self._secrets:
            msg = record.getMessage()
            for env_name, value in self._secrets:
                if value in msg:
                    # Replace in the formatted message args
                    record.msg = str(record.msg)
                    record.args = None
                    record.msg = record.msg.replace(value, f"***({env_name})")
            # Also check exc_text
            if record.exc_text:
                for env_name, value in self._secrets:
                    if value in record.exc_text:
                        record.exc_text = record.exc_text.replace(
                            value, f"***({env_name})")
        return True
