"""Navigation algorithm-kernel loader helpers."""

from nav.kernel.paths import (
    nav_kernel_build_hint,
    nav_kernel_candidate_dirs,
    repo_root_for_nav_kernel,
)
from nav.kernel.loader import (
    NAV_KERNEL_EXTENSION_MODULE,
    PRODUCTION_NAV_KERNEL_SYMBOLS,
    ensure_nav_kernel_on_path,
    nav_kernel_available,
    require_nav_kernel,
    try_import_nav_kernel,
)

__all__ = [
    "NAV_KERNEL_EXTENSION_MODULE",
    "PRODUCTION_NAV_KERNEL_SYMBOLS",
    "ensure_nav_kernel_on_path",
    "nav_kernel_available",
    "nav_kernel_build_hint",
    "nav_kernel_candidate_dirs",
    "require_nav_kernel",
    "repo_root_for_nav_kernel",
    "try_import_nav_kernel",
]
