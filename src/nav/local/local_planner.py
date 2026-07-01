"""Compatibility import for the local planner Module."""

# setup_local_planner_backend lives in nav.services.plan.local_planner.runtime.
from importlib import import_module
import sys

_module = import_module("nav.services.plan.local_planner.service")
globals().update(_module.__dict__)
sys.modules[__name__] = _module
