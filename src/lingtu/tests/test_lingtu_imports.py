"""Contract tests: verify the lingtu import-ready API is accessible."""

import importlib
import unittest


class TestLingTuImports(unittest.TestCase):
    def test_import_lingtu_package(self):
        """The lingtu package is importable."""
        self.assertIsNotNone(importlib.import_module("lingtu"))

    def test_import_lingtu_classes(self):
        """All public API classes import correctly."""
        from lingtu import Robot

        self.assertIsNotNone(Robot)

    def test_import_lingtu_all(self):
        """lingtu.__all__ matches the public API."""
        import lingtu

        expected = {"Robot"}
        self.assertEqual(set(lingtu.__all__), expected)

    def test_import_lingtu_local_entrypoints(self):
        """The local runtime API is importable as a submodule boundary."""
        from lingtu.runtime import build_system, resolve_runtime

        self.assertIsNotNone(build_system)
        self.assertIsNotNone(resolve_runtime)
