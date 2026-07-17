"""Thin Python adapters for the native C++ inspection domain."""

from .native import InspectionNativeError, NativeInspectionStore

__all__ = ["InspectionNativeError", "NativeInspectionStore"]
