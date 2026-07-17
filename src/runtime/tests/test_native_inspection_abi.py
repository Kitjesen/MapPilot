from pathlib import Path

import pytest

from runtime.adapters.native import inspection as native_inspection


class _FakeFunction:
    def __init__(self, result=None):
        self.result = result
        self.argtypes = None
        self.restype = None
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        return self.result


class _FakeInspectionLibrary:
    def __init__(self, *, abi_version=None, include_abi=True):
        if include_abi:
            self.lingtu_inspection_store_abi_version = _FakeFunction(abi_version)
        self.lingtu_inspection_store_create = _FakeFunction(1)
        self.lingtu_inspection_store_destroy = _FakeFunction(None)
        self.lingtu_inspection_store_put = _FakeFunction(0)
        self.lingtu_inspection_store_delete = _FakeFunction(0)
        self.lingtu_inspection_store_get_json = _FakeFunction(None)
        self.lingtu_inspection_store_list_json = _FakeFunction(None)
        self.lingtu_inspection_store_status_json = _FakeFunction(None)
        self.lingtu_inspection_store_last_error = _FakeFunction(b"fake error")
        self.lingtu_inspection_string_free = _FakeFunction(None)


def test_native_inspection_store_checks_abi_before_store_symbols(monkeypatch, tmp_path) -> None:
    library = _FakeInspectionLibrary(abi_version=native_inspection.NATIVE_INSPECTION_STORE_ABI_VERSION)
    monkeypatch.setattr(native_inspection, "_load_library", lambda: library)

    store = native_inspection.NativeInspectionStore(tmp_path)
    store.close()

    assert library.lingtu_inspection_store_abi_version.calls == [()]
    assert library.lingtu_inspection_store_create.calls == [(str(tmp_path).encode("utf-8"),)]


def test_native_inspection_store_fails_when_abi_metadata_missing(monkeypatch, tmp_path) -> None:
    library = _FakeInspectionLibrary(include_abi=False)
    monkeypatch.setattr(native_inspection, "_load_library", lambda: library)

    with pytest.raises(native_inspection.InspectionNativeError) as exc:
        native_inspection.NativeInspectionStore(tmp_path)

    assert "native inspection store ABI metadata is missing" in str(exc.value)
    assert library.lingtu_inspection_store_create.calls == []


def test_native_inspection_store_fails_when_abi_version_mismatches(monkeypatch, tmp_path) -> None:
    library = _FakeInspectionLibrary(abi_version=native_inspection.NATIVE_INSPECTION_STORE_ABI_VERSION + 1)
    monkeypatch.setattr(native_inspection, "_load_library", lambda: library)

    with pytest.raises(native_inspection.InspectionNativeError) as exc:
        native_inspection.NativeInspectionStore(tmp_path)

    assert "native inspection store ABI version mismatch" in str(exc.value)
    assert library.lingtu_inspection_store_create.calls == []


def test_native_inspection_c_api_exports_store_abi_version() -> None:
    header = Path("src/nav/inspection/c_api.h").read_text(encoding="utf-8")
    source = Path("src/nav/inspection/c_api.cpp").read_text(encoding="utf-8")

    assert "lingtu_inspection_store_abi_version" in header
    assert "lingtu_inspection_store_abi_version" in source
