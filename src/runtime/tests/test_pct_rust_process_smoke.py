from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_rust_process_smoke.py"
LM_GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_smoke.json"
)
GN_GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_gn_smoke.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_rust_process_smoke", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_golden_matches_selected_nonlinear_optimizer() -> None:
    module = _load_module()

    assert module.default_golden_for_optimizer("levenberg_marquardt") == LM_GOLDEN
    assert module.default_golden_for_optimizer("gauss_newton") == GN_GOLDEN
    assert module.default_golden_for_optimizer("unknown") == LM_GOLDEN


def test_gn_golden_fixture_requires_gauss_newton_runtime() -> None:
    module = _load_module()
    assert GN_GOLDEN.is_file()

    parser = module.build_parser()
    args = parser.parse_args(["--nonlinear-optimizer", "gauss_newton"])

    assert args.nonlinear_optimizer == "gauss_newton"
    assert args.golden is None
    assert module.default_golden_for_optimizer(args.nonlinear_optimizer) == GN_GOLDEN
