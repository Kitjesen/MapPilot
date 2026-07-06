"""Runtime diagnostics, acceptance gates, and evidence tooling.

This package is intentionally outside the runtime kernel. Product code may use
these helpers to explain or validate a running system, but new module lifecycle,
transport, profile, or behavior logic should stay in the main runtime packages.
"""

