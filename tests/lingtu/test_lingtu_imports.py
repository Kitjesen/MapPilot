"""The package root is importable without hiding Product control."""

import importlib
import unittest


class TestLingTuImports(unittest.TestCase):
    def test_import_lingtu_package(self):
        """The lingtu package is importable."""
        self.assertIsNotNone(importlib.import_module("lingtu"))

    def test_import_product_control(self):
        from lingtu.control import ProductControl

        self.assertIsNotNone(ProductControl)
