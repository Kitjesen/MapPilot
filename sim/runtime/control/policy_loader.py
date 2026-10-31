"""Load the policy selected by the simulation control plan."""

from pathlib import Path

from .thunder import legacy_policy as legacy
from .thunder.cts_policy import POLICY_CONTRACT, CtsPolicy


def open_session(path: str, cpu_threads: int = 1):
    import onnxruntime as ort

    threads = int(cpu_threads)
    if not 1 <= threads <= 8:
        raise ValueError("ONNX policy cpu_threads must be in [1, 8]")
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    return ort.InferenceSession(
        path,
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )


def load_policy(path: str, *, cpu_threads: int = 1, session=None):
    """Open one model and return its policy."""
    if Path(path).suffix.lower() in {".pt", ".pth", ".jit"}:
        return legacy.TorchPolicy(path)

    if session is None:
        session = open_session(path, cpu_threads)
    metadata = session.get_modelmeta().custom_metadata_map
    if metadata.get("policy_contract") == POLICY_CONTRACT:
        return CtsPolicy(path, session=session)
    return legacy.load(path, session=session)
