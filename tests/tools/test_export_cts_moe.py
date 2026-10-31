"""The exported student must keep both inputs, dynamic batches, and exact contracts."""

import json

import pytest
from tools.robot.export_cts_moe import EMBEDDED_CONTRACTS, POLICY_CONTRACT, export_policy


def test_existing_output_is_not_overwritten(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    existing = output / "policy_cts_moe.onnx"
    existing.write_bytes(b"existing policy")
    with pytest.raises(FileExistsError):
        export_policy(tmp_path / "source.pt", output, tmp_path / "runtime.json")
    assert existing.read_bytes() == b"existing policy"


def test_actual_export_keeps_two_inputs_and_embedded_contracts(tmp_path):
    torch = pytest.importorskip("torch")
    pytest.importorskip("onnx")
    ort = pytest.importorskip("onnxruntime")

    class Student(torch.nn.Module):
        def forward(self, history, single_obs):
            return history[:, :16] * 0.25 + single_obs[:, :16] * 0.5

    source = tmp_path / "student.pt"
    model = torch.jit.trace(Student(), (torch.zeros(1, 530), torch.zeros(1, 53)))
    contracts = {f"{name}.json": json.dumps({"saved": name}, indent=2).encode()
                 for name in EMBEDDED_CONTRACTS}
    torch.jit.save(model, str(source), _extra_files=contracts)
    runtime = tmp_path / "runtime.json"
    runtime.write_text('{\n  "dt": 0.02\n}\n', encoding="utf-8")
    destination = tmp_path / "output"

    report = export_policy(source, destination, runtime)
    assert report["conversion_passed"] and not report["behavior_verified"]
    assert len(report["numerical_cases"]) == 12
    assert report["inputs"] == [
        {"name": "history", "shape": ["batch", 530], "type": "tensor(float)"},
        {"name": "single_obs", "shape": ["batch", 53], "type": "tensor(float)"},
    ]
    assert report["outputs"] == [{"name": "actions", "shape": ["batch", 16], "type": "tensor(float)"}]
    session = ort.InferenceSession(str(destination / "policy_cts_moe.onnx"),
                                   providers=["CPUExecutionProvider"])
    metadata = session.get_modelmeta().custom_metadata_map
    assert metadata["policy_contract"] == POLICY_CONTRACT
    for filename, original in contracts.items():
        assert (destination / filename).read_bytes() == original
        assert metadata[filename.removesuffix(".json")] == original.decode()
    assert (destination / "runtime_contract.json").read_bytes() == runtime.read_bytes()
    assert metadata["runtime_contract"] == runtime.read_text(encoding="utf-8")
