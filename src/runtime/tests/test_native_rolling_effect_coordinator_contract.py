from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
NAV_CPP = ROOT / "src" / "nav" / "cpp"
ROLLING_DIR = NAV_CPP / "endpoint" / "nav" / "runtime" / "rolling"
COORDINATOR_HEADER = ROLLING_DIR / "effects.hpp"
COORDINATOR_SOURCE = ROLLING_DIR / "effects.cpp"
COORDINATOR_TEST = NAV_CPP / "tests" / "endpoint" / "test_rolling_segment_effect_coordinator.cpp"
ENDPOINT = NAV_CPP / "endpoint" / "nav" / "main.cpp"
NAV_CPP_CMAKE = NAV_CPP / "CMakeLists.txt"
BUILD_SCRIPT = ROOT / "scripts" / "build" / "build_nav_endpoint.sh"
README = NAV_CPP / "README.md"


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8")


def test_declares_transport_free_rolling_segment_effect_coordinator_files():
    assert COORDINATOR_HEADER.exists()
    assert COORDINATOR_SOURCE.exists()


def test_adds_portable_cpp_test_for_rolling_segment_effect_coordinator():
    text = _read(COORDINATOR_TEST)
    assert "RollingSegmentEffectCoordinator" in text


def test_cmake_compiles_effect_coordinator_into_rolling_segment_library():
    cmake = _read(NAV_CPP_CMAKE)
    assert '"${_LINGTU_ROLLING_ENDPOINT_DIR}/effects.cpp"' in cmake
    assert re.search(
        r"add_library\s*\(\s*lingtu_nav_rolling_segment\b[\s\S]*"
        r"effects\.cpp",
        cmake,
    )


def test_cmake_registers_portable_effect_coordinator_ctest():
    cmake = _read(NAV_CPP_CMAKE)
    assert "test_rolling_segment_effect_coordinator.cpp" in cmake
    assert re.search(
        r"add_test\s*\(\s*NAME\s+test_rolling_segment_effect_coordinator\b",
        cmake,
    )


def test_native_endpoint_includes_and_constructs_named_effect_coordinator():
    source = _read(ENDPOINT)
    assert '#include "runtime/rolling/effects.hpp"' in source
    assert re.search(
        r"RollingSegmentEffectCoordinator\s+rolling_segment_effect_coordinator\b",
        source,
    )


def test_native_endpoint_removes_local_recursive_effect_lambda():
    source = _read(ENDPOINT)
    assert "std::function<bool(const RollingSegmentStepResult &)>" not in source
    assert "apply_rolling_segment_effects = [&]" not in source


def test_native_endpoint_delegates_rolling_segment_effects_through_coordinator():
    source = _read(ENDPOINT)
    assert "rolling_segment_effect_coordinator.apply" in source
    assert "apply_rolling_segment_effects(rolling_segment.step" not in source


def test_effect_coordinator_stays_transport_free():
    combined = "\n".join((_read(COORDINATOR_HEADER), _read(COORDINATOR_SOURCE)))
    forbidden_tokens = (
        "DdsRuntime",
        "Executor",
        "messages",
        "nav_dds_runtime",
        "navigation/executor.hpp",
    )
    for token in forbidden_tokens:
        assert token not in combined


def test_nav_endpoint_build_script_requires_effect_coordinator_ctest():
    script = _read(BUILD_SCRIPT)
    assert "test_rolling_segment_effect_coordinator" in script


def test_nav_cpp_readme_documents_effect_coordinator_ownership():
    readme = _read(README)
    assert "RollingSegmentEffectCoordinator" in readme
    assert "transport-free" in readme
