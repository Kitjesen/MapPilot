from __future__ import annotations

from .common import *  # noqa: F401,F403

class RustGpmpOptimizerLibrary:
    """ctypes wrapper for the portable GPMP optimizer C ABI."""

    def __init__(self, path: str | os.PathLike[str]):
        self.path = Path(path).resolve()
        self._lib = ctypes.CDLL(str(self.path))

        abi_version = self._lib.lingtu_gpmp_optimizer_abi_version
        abi_version.argtypes = []
        abi_version.restype = ctypes.c_uint32
        version = int(abi_version())
        if version != PCT_RUST_OPTIMIZER_ABI_VERSION:
            raise RuntimeError(
                "Rust GPMP optimizer ABI version mismatch: "
                f"expected {PCT_RUST_OPTIMIZER_ABI_VERSION}, got {version}"
            )
        self.abi_version = version

        optimize = self._lib.lingtu_gpmp_optimizer_optimize_json
        optimize.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_size_t),
        ]
        optimize.restype = ctypes.c_int
        self._optimize = optimize

        free_json = self._lib.lingtu_gpmp_optimizer_free_json
        free_json.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        free_json.restype = None
        self._free_json = free_json

    def optimize_json(self, request_json: str) -> dict[str, Any]:
        payload = request_json.encode("utf-8")
        input_buffer = ctypes.create_string_buffer(payload)
        output_ptr = ctypes.c_void_p()
        output_len = ctypes.c_size_t()
        status = int(
            self._optimize(
                ctypes.cast(input_buffer, ctypes.c_void_p),
                len(payload),
                ctypes.byref(output_ptr),
                ctypes.byref(output_len),
            )
        )
        try:
            if not output_ptr.value:
                raise RuntimeError(f"Rust GPMP optimizer ABI returned no JSON output, status={status}")
            raw = ctypes.string_at(output_ptr, int(output_len.value)).decode("utf-8")
        finally:
            if output_ptr.value:
                self._free_json(output_ptr, output_len)
        return json.loads(raw)


