from __future__ import annotations

from types import SimpleNamespace

from runtime.msgs import numpy_compat


def test_lazy_numpy_caches_resolved_attributes(monkeypatch):
    marker = object()
    fake_numpy = SimpleNamespace(isfinite=marker)
    imports: list[str] = []

    monkeypatch.setattr(numpy_compat, "numpy_import_is_safe", lambda: True)

    def fake_import(name: str):
        imports.append(name)
        return fake_numpy

    monkeypatch.setattr(numpy_compat.importlib, "import_module", fake_import)
    lazy = numpy_compat.LazyNumpy()

    assert lazy.isfinite is marker
    assert lazy.isfinite is marker
    assert imports == ["numpy"]
