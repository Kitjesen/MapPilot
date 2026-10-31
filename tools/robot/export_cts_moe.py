"""Export a saved CTS MoE TorchScript student; conversion is not motion acceptance."""

import argparse
import json
from pathlib import Path

POLICY_CONTRACT = "thunder_cts_moe_v1"
EMBEDDED_CONTRACTS = ("actuator_contract", "training_context", "hip_contract")
BATCHES = (1, 4, 72)
SCALES = (0.0, 0.1, 1.0, 3.0)
TOLERANCE = 1e-4


def export_policy(source: Path, destination: Path, runtime_contract: Path) -> dict:
    """Keep the original two-input student and its saved deployment contracts."""
    destination.mkdir(parents=True, exist_ok=False)

    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch

    runtime_bytes = runtime_contract.read_bytes()
    json.loads(runtime_bytes)
    extras = {f"{name}.json": b"" for name in EMBEDDED_CONTRACTS}
    torch.set_num_threads(2)
    torch.manual_seed(34823)
    output = destination / "policy_cts_moe.onnx"

    # Disable autograd before JIT loading and its first execution. Cached
    # differentiable execution plans can otherwise break later ONNX conversion.
    with torch.no_grad():
        model = torch.jit.load(str(source), map_location="cpu", _extra_files=extras).eval()
        for parameter in model.parameters():
            parameter.requires_grad_(False)

        metadata = {"policy_contract": POLICY_CONTRACT}
        for name in EMBEDDED_CONTRACTS:
            value = extras[f"{name}.json"]
            if not value:
                raise ValueError(f"Missing embedded contract: {name}.json")
            if isinstance(value, str):
                value = value.encode("utf-8")
            json.loads(value)
            (destination / f"{name}.json").write_bytes(value)
            metadata[name] = value.decode("utf-8")
        (destination / "runtime_contract.json").write_bytes(runtime_bytes)
        metadata["runtime_contract"] = runtime_bytes.decode("utf-8")

        torch.onnx.export(
            model,
            (torch.zeros(1, 530), torch.zeros(1, 53)),
            str(output),
            input_names=["history", "single_obs"],
            output_names=["actions"],
            opset_version=17,
            dynamic_axes={name: {0: "batch"} for name in ("history", "single_obs", "actions")},
            dynamo=False,
        )
        graph = onnx.load(str(output))
        onnx.helper.set_model_props(graph, metadata)
        onnx.checker.check_model(graph)
        onnx.save(graph, str(output))

        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(
            str(output), sess_options=options, providers=["CPUExecutionProvider"]
        )
        cases = []
        for batch in BATCHES:
            for scale in SCALES:
                history = torch.randn(batch, 530) * scale
                single = torch.randn(batch, 53) * scale
                expected = model(history, single).numpy()
                actual = session.run(
                    ["actions"], {"history": history.numpy(), "single_obs": single.numpy()}
                )[0]
                if expected.shape != (batch, 16) or actual.shape != (batch, 16):
                    raise ValueError(f"Expected actions shape ({batch}, 16)")
                if not np.isfinite(expected).all() or not np.isfinite(actual).all():
                    raise ValueError(f"Non-finite actions for batch={batch}, scale={scale}")
                np.testing.assert_allclose(actual, expected, atol=TOLERANCE, rtol=TOLERANCE)
                cases.append({
                    "batch": batch,
                    "scale": scale,
                    "max_absolute_error": float(np.max(np.abs(actual - expected))),
                })

    report = {
        "conversion_passed": True,
        "behavior_verified": False,
        "source_path": str(source.resolve()),
        "source_bytes": source.stat().st_size,
        "onnx_path": str(output.resolve()),
        "onnx_bytes": output.stat().st_size,
        "runtime_contract_source": str(runtime_contract.resolve()),
        "policy_contract": POLICY_CONTRACT,
        "torch_version": torch.__version__,
        "numpy_version": np.__version__,
        "onnx_version": onnx.__version__,
        "onnxruntime_version": ort.__version__,
        "opset": 17,
        "inputs": [{"name": value.name, "shape": value.shape, "type": value.type}
                   for value in session.get_inputs()],
        "outputs": [{"name": value.name, "shape": value.shape, "type": value.type}
                    for value in session.get_outputs()],
        "atol": TOLERANCE,
        "rtol": TOLERANCE,
        "max_absolute_error": max(case["max_absolute_error"] for case in cases),
        "numerical_cases": cases,
    }
    (destination / "conversion.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Saved TorchScript student")
    parser.add_argument("destination", type=Path, help="New directory for ONNX and contracts")
    parser.add_argument("--runtime-contract", type=Path, required=True,
                        help="JSON containing the saved runtime settings for this student")
    args = parser.parse_args()
    print(json.dumps(export_policy(args.source, args.destination, args.runtime_contract)), flush=True)


if __name__ == "__main__":
    main()
