"""Runtime profile submodules.

Import the owning submodule directly, for example
``runtime.profiles.resolver`` or ``runtime.profiles.profile_adapters``. Keeping this
package initializer empty prevents an unrelated planner helper from loading
the complete field Product catalog as a side effect.
"""

from __future__ import annotations

__all__: list[str] = []
