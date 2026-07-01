"""Compatibility import for local planner backend helpers."""

from importlib import import_module
import sys

_module = import_module("nav.services.plan.local_planner.backend")
globals().update(_module.__dict__)
sys.modules[__name__] = _module
