"""Atomic package-promotion contracts."""

# ruff: noqa: S101,S603

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from sim.catalog.composer import SessionComposer
from sim.catalog.resolver import CatalogError, CatalogResolver
from sim.importers import CatalogPromoter, ImportDraft, ImportFailure, RobotImporter

_PROCESS_BARRIER_TIMEOUT_S = 60


def _draft(root: Path, *, package_id: str = "field", version: str = "1.0.0", suffix: str = "") -> ImportDraft:
    draft_root = root / f"draft{suffix}"
    package_root = draft_root / "package"
    provenance_path = package_root / "provenance" / "world.provenance.json"
    qualification_root = draft_root / "qualification" / "world" / package_id
    evidence_root = qualification_root / "evidence" / version
    package_root.mkdir(parents=True)
    provenance_path.parent.mkdir(parents=True)
    evidence_root.mkdir(parents=True)
    (package_root / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" '
        'iterations="100" gravity="0 0 -9.81"/><worldbody/></mujoco>\n',
        encoding="utf-8",
    )
    (package_root / "world.package.yaml").write_text(
        """schema: lingtu.sim.world-package.v1
id: field
version: 1.0.0
kind: world
physics:
  mjcf: world.xml
  global_policy:
    timestep_s: 0.002
    integrator: rk4
    solver: newton
    iterations: 100
    gravity_mps2: [0.0, 0.0, -9.81]
visual:
  binding: WorldVisual:Field
  level: /Game/RobotSim/Maps/Field
entities: []
""".replace("id: field", f"id: {package_id}").replace("version: 1.0.0", f"version: {version}"),
        encoding="utf-8",
    )
    provenance_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.world-provenance.v1",
                "provenance": {
                    "owner": "LingTu tests",
                    "license": "LicenseRef-Test",
                    "source_uri": "file://test",
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    evidence_path = evidence_root / "qualification.json"
    evidence_path.write_text(json.dumps({"status": "passed"}, sort_keys=True), encoding="utf-8")
    qualification_path = qualification_root / f"{version}.qualification.json"
    qualification_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.qualification-record.v1",
                "package": {
                    "kind": "world",
                    "id": package_id,
                    "version": version,
                },
                "qualified_capabilities": {},
                "provenance": {
                    "path": "provenance/world.provenance.json",
                },
                "checks": [
                    {
                        "id": "content",
                        "status": "passed",
                        "evidence": [
                            {"path": f"evidence/{version}/qualification.json"}
                        ],
                    }
                ],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return ImportDraft(
        import_id=f"import-{package_id}-{version}{suffix}",
        kind="world",
        package_id=package_id,
        version=version,
        state="qualified",
        root=draft_root,
        package_root=package_root,
        manifest_path=package_root / "world.package.yaml",
        provenance_path=provenance_path,
        qualification_path=qualification_path,
    )


def _targets(root: Path, package_id: str = "field", version: str = "1.0.0") -> tuple[Path, Path, Path]:
    package = root / "sim" / "packages" / "worlds" / package_id / version
    qualification_parent = root / "sim" / "qualifications" / "world" / package_id
    return package, qualification_parent / f"{version}.qualification.json", qualification_parent / "evidence" / version


def _publish_race_worker(
    repo_root: str,
    source: str,
    target: str,
    ready_path: str,
    start_path: str,
    result_path: str,
    label: str,
) -> None:
    """Race an independent process through the real no-replace publisher."""

    ready = Path(ready_path)
    ready.write_text("ready\n", encoding="utf-8")
    start = Path(start_path)
    deadline = time.monotonic() + _PROCESS_BARRIER_TIMEOUT_S
    while not start.exists():
        if time.monotonic() > deadline:
            Path(result_path).write_text(
                json.dumps({"source": source, "status": "error", "message": "timed out waiting for start"}),
                encoding="utf-8",
            )
            return
        time.sleep(0.01)

    promoter = CatalogPromoter(Path(repo_root))
    try:
        promoter._publish_step(label, Path(source), Path(target))
    except ImportFailure as exc:
        result = {"source": source, "status": "conflict", "message": str(exc)}
    except BaseException as exc:  # pragma: no cover - surfaced as a test assertion
        result = {"source": source, "status": "error", "message": repr(exc)}
    else:
        result = {"source": source, "status": "published", "message": ""}
    Path(result_path).write_text(json.dumps(result, sort_keys=True), encoding="utf-8")


def _robot_draft(root: Path) -> ImportDraft:
    source = root / "robot-source"
    source.mkdir()
    (source / "LICENSE.txt").write_text("Test license\n", encoding="utf-8")
    (source / "robot.xml").write_text(
        """<mujoco model="promotion_bot">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base_link">
      <joint name="floating_base_joint" type="free"/>
      <geom name="body_visual" type="box" size="0.2 0.1 0.05" rgba="0.2 0.4 0.6 1"/>
      <site name="imu" pos="0 0 0.1" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    request = {
        "schema": "lingtu.sim.robot-import-request.v1",
        "id": "promotion_bot",
        "version": "1.0.0",
        "source": str(source),
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "units": {"length": "m", "angle": "radian"},
        "provenance": {
            "owner": "LingTu tests",
            "license": "Test-Only",
            "license_file": "LICENSE.txt",
            "source_uri": "file://test",
        },
        "physics": {"attach_root": "base_link", "root_joint": "floating_base_joint"},
        "visual": {"binding": "RobotVisual:PromotionBot"},
        "semantic": {"class": "test_robot"},
        "frames": [{"name": "base_link", "role": "body"}, {"name": "imu", "role": "sensor_mount"}],
        "interfaces": {"state": ["lingtu.sim.base-state.v1"], "command": ["lingtu.sim.base-velocity.v1"]},
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {"locomotion": ["drive"], "sensor_mounts": ["imu"]},
    }
    repo_root = Path(__file__).resolve().parents[2]
    return RobotImporter(repo_root, work_root=root / "imports").import_robot(request)


def test_full_artifact_digest_is_validated_before_publication(tmp_path: Path) -> None:
    draft = _draft(tmp_path)
    (draft.package_root / "world.xml").write_text("<mujoco><worldbody><geom/></worldbody></mujoco>\n", encoding="utf-8")
    promoter = CatalogPromoter(tmp_path)

    with pytest.raises(ImportFailure, match=r"content digest"):
        promoter.promote(draft)

    assert all(not path.exists() for path in _targets(tmp_path))


def test_robot_package_relative_projection_survives_staging(tmp_path: Path) -> None:
    draft = _robot_draft(tmp_path)
    projection = draft.package_root / "visual" / "robot.visual-projection.json"
    source_bytes = projection.read_bytes()
    assert json.loads(source_bytes)["mjcf"]["path"] == "source/robot.xml"

    result = CatalogPromoter(tmp_path).promote(draft)

    assert projection.read_bytes() == source_bytes
    assert (result.package_root / "visual" / projection.name).read_bytes() == source_bytes


def test_long_repo_root_keeps_package_and_evidence_staging_below_max_path(tmp_path: Path) -> None:
    repo_root = tmp_path
    while len(str(repo_root)) < 155:
        remaining = max(1, 155 - len(str(repo_root)) - 1)
        repo_root /= "r" * min(30, remaining)
    repo_root.mkdir(parents=True)
    draft = _draft(repo_root)

    old_evidence_path = (
        repo_root
        / "sim"
        / ".promotion-staging"
        / draft.kind
        / draft.package_id
        / draft.version
        / ("0" * 32)
        / "qualification"
        / draft.kind
        / draft.package_id
        / "evidence"
        / draft.version
        / "qualification.json"
    )
    assert len(str(old_evidence_path)) > 259

    result = CatalogPromoter(repo_root).promote(draft)

    assert (result.package_root / "world.xml").is_file()
    assert (result.qualification_path.parent / "evidence" / draft.version / "qualification.json").is_file()


def test_unknown_qualification_package_field_is_rejected(tmp_path: Path) -> None:
    draft = _draft(tmp_path)
    report = json.loads(draft.qualification_path.read_text(encoding="utf-8"))
    report["package"]["unused"] = True
    draft.qualification_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")

    with pytest.raises(ImportFailure, match=r"invalid fields"):
        CatalogPromoter(tmp_path).promote(draft)

    assert all(not path.exists() for path in _targets(tmp_path))


def test_package_is_the_final_commit_marker_and_orphaned_prerequisites_are_reused(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    observed = threading.Event()

    def fail_before_marker(layout, validation) -> None:
        assert not layout.package_target.exists()
        assert layout.evidence_target.is_dir()
        assert layout.qualification_target.is_file()
        assert not (draft.root / "promotion-result.json").exists()
        observed.set()
        raise RuntimeError("injected before final marker")

    monkeypatch.setattr(promoter, "_before_package_commit", fail_before_marker)

    with pytest.raises(ImportFailure, match=r"injected before final marker"):
        promoter.promote(draft)

    assert observed.is_set()
    package_target, qualification_target, evidence_target = _targets(tmp_path)
    assert not package_target.exists()
    assert qualification_target.is_file()
    assert evidence_target.is_dir()
    assert not (draft.root / "promotion-result.json").exists()

    result = CatalogPromoter(tmp_path).promote(draft)

    assert result.package_root == package_target
    assert (draft.root / "promotion-result.json").is_file()


def test_catalog_and_session_composer_cannot_see_package_before_final_commit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    entered = threading.Event()
    release = threading.Event()

    def block_before_package(layout, validation) -> None:
        assert not layout.package_target.exists()
        assert layout.evidence_target.is_dir()
        assert layout.qualification_target.is_file()
        assert not (draft.root / "promotion-result.json").exists()
        entered.set()
        assert release.wait(5)

    monkeypatch.setattr(promoter, "_before_package_commit", block_before_package)
    with ThreadPoolExecutor(max_workers=1) as pool:
        promotion = pool.submit(promoter.promote, draft)
        assert entered.wait(5)

        resolver = CatalogResolver(tmp_path, (tmp_path / "sim" / "packages",))
        with pytest.raises(CatalogError):
            resolver.find_package("field@1.0.0", kind="world")

        composer = SessionComposer(resolver, artifact_root=tmp_path / "artifacts")
        intent = {
            "schema": "lingtu.sim.session-intent.v1",
            "session": {
                "session_id": "promotion_visibility",
                "mujoco_version": "3.3.7",
                "seed": 1,
                "world": "field@1.0.0",
                "robots": [],
                "runtime": {"backend": "mujoco", "mode": "headless", "required_bindings": ["physics"]},
            },
        }
        with pytest.raises(CatalogError):
            composer.compose(intent, output_dir=Path("promotion_visibility"))

        release.set()
        result = promotion.result(timeout=5)

    assert result.package_root.is_dir()
    refreshed = CatalogResolver(tmp_path, (tmp_path / "sim" / "packages",))
    assert refreshed.find_package("field@1.0.0", kind="world").ref == "field@1.0.0"


def test_precommit_failure_does_not_delete_foreign_replacement_at_prerequisite_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    package_target, qualification_target, evidence_target = _targets(tmp_path)
    sentinel = b"foreign concurrent owner\n"

    def fail_after_foreign_write(layout, validation) -> None:
        replacement = qualification_target.with_suffix(".foreign")
        replacement.write_bytes(sentinel)
        replacement.replace(qualification_target)
        raise RuntimeError("injected after concurrent write")

    monkeypatch.setattr(promoter, "_before_package_commit", fail_after_foreign_write)

    with pytest.raises(ImportFailure, match=r"injected after concurrent write"):
        promoter.promote(draft)

    assert qualification_target.read_bytes() == sentinel
    assert not package_target.exists()
    assert evidence_target.is_dir()


def test_result_write_failure_keeps_catalog_commit_and_retry_repairs_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    original = promoter._write_promotion_result
    fail_once = True

    def fail_result_write(path, value) -> None:
        nonlocal fail_once
        if fail_once:
            fail_once = False
            raise OSError("injected result write failure")
        original(path, value)

    monkeypatch.setattr(promoter, "_write_promotion_result", fail_result_write)

    with pytest.raises(ImportFailure, match=r"injected result write failure") as raised:
        promoter.promote(draft)

    assert raised.value.details == {
        "committed": True,
        "package": {"kind": "world", "id": "field", "version": "1.0.0", "ref": "field@1.0.0"},
    }
    assert all(path.exists() for path in _targets(tmp_path))
    assert not (draft.root / "promotion-result.json").exists()
    resolver = CatalogResolver(tmp_path, (tmp_path / "sim" / "packages",))
    assert resolver.find_package("field@1.0.0", kind="world").ref == "field@1.0.0"

    repaired = promoter.promote(draft)

    receipt = json.loads((draft.root / "promotion-result.json").read_text(encoding="utf-8"))
    assert receipt == repaired.to_dict()


def test_top_level_provenance_is_required(tmp_path: Path) -> None:
    draft = _draft(tmp_path)
    report = json.loads(draft.qualification_path.read_text(encoding="utf-8"))
    del report["provenance"]
    draft.qualification_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")

    with pytest.raises(ImportFailure, match=r"invalid fields"):
        CatalogPromoter(tmp_path).promote(draft)

    assert all(not path.exists() for path in _targets(tmp_path))


def test_fault_during_publication_never_leaves_a_qualified_marker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    original = promoter._publish_step

    def fail_on_evidence(label, source, target) -> None:
        if label == "evidence":
            raise RuntimeError("injected evidence publication failure")
        original(label, source, target)

    monkeypatch.setattr(promoter, "_publish_step", fail_on_evidence)

    with pytest.raises(ImportFailure, match=r"injected evidence publication failure"):
        promoter.promote(draft)

    assert all(not path.exists() for path in _targets(tmp_path))


def test_package_publish_conflict_never_deletes_foreign_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    package_target, qualification_target, evidence_target = _targets(tmp_path)
    sentinel = b"foreign package owner\n"
    original = promoter._publish_step

    def inject_package_target(label, source, target) -> None:
        if label == "package":
            target.write_bytes(sentinel)
        original(label, source, target)

    monkeypatch.setattr(promoter, "_publish_step", inject_package_target)

    with pytest.raises(ImportFailure, match=r"target appeared"):
        promoter.promote(draft)

    assert package_target.read_bytes() == sentinel
    assert qualification_target.is_file()
    assert evidence_target.is_dir()


@pytest.mark.parametrize("foreign_label", ["evidence", "qualification"])
def test_different_prerequisite_conflicts_without_deleting_foreign_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, foreign_label: str
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    package_target, qualification_target, evidence_target = _targets(tmp_path)
    sentinel = b"foreign publication owner\n"
    original = promoter._publish_step
    foreign_target: Path | None = None
    sentinel_path: Path | None = None

    def inject_foreign_target(label, source, target) -> None:
        nonlocal foreign_target, sentinel_path
        if label == foreign_label:
            foreign_target = target
            if label == "evidence":
                target.mkdir()
                sentinel_path = target / "sentinel.bin"
            else:
                sentinel_path = target
            sentinel_path.write_bytes(sentinel)
        original(label, source, target)

    monkeypatch.setattr(promoter, "_publish_step", inject_foreign_target)

    with pytest.raises(ImportFailure, match=r"different content"):
        promoter.promote(draft)

    assert foreign_target is not None
    assert sentinel_path is not None
    assert sentinel_path.read_bytes() == sentinel
    assert not package_target.exists()
    if foreign_label == "evidence":
        assert not qualification_target.exists()
    else:
        assert evidence_target.is_dir()


def test_same_content_is_idempotent_and_different_content_conflicts(tmp_path: Path) -> None:
    first = _draft(tmp_path, suffix="-one")
    promoter = CatalogPromoter(tmp_path)
    result = promoter.promote(first)
    package_bytes = {
        path.relative_to(result.package_root).as_posix(): path.read_bytes()
        for path in result.package_root.rglob("*")
        if path.is_file()
    }
    qualification_bytes = result.qualification_path.read_bytes()

    repeated = promoter.promote(first)
    assert repeated == result
    assert package_bytes == {
        path.relative_to(result.package_root).as_posix(): path.read_bytes()
        for path in result.package_root.rglob("*")
        if path.is_file()
    }
    assert result.qualification_path.read_bytes() == qualification_bytes

    second = _draft(tmp_path, suffix="-two")
    (second.package_root / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" '
        'iterations="100" gravity="0 0 -9.81"/><worldbody><geom/></worldbody></mujoco>\n',
        encoding="utf-8",
    )
    with pytest.raises(ImportFailure, match=r"already|different|invalid"):
        promoter.promote(second)
    assert package_bytes == {
        path.relative_to(result.package_root).as_posix(): path.read_bytes()
        for path in result.package_root.rglob("*")
        if path.is_file()
    }


@pytest.mark.parametrize("source_kind", ["file", "directory"])
def test_independent_process_publish_race_never_replaces_the_winner(
    tmp_path: Path, source_kind: str
) -> None:
    """Two real processes must produce one winner and one preserved loser."""

    sources = [tmp_path / "source-a", tmp_path / "source-b"]
    target = tmp_path / "published"
    if source_kind == "file":
        sources[0].write_bytes(b"foreign-process-a\n")
        sources[1].write_bytes(b"foreign-process-b\n")
        label = "qualification"
    else:
        for source, payload in zip(sources, (b"foreign-process-a\n", b"foreign-process-b\n")):
            source.mkdir()
            (source / "payload.bin").write_bytes(payload)
        label = "package"

    barrier_root = tmp_path / "barriers"
    barrier_root.mkdir()
    start_path = barrier_root / "start"
    ready_paths = [barrier_root / f"ready-{index}.txt" for index in range(len(sources))]
    result_paths = [barrier_root / f"result-{index}.json" for index in range(len(sources))]
    worker_code = (
        "import sys\n"
        "from sim.tests.test_import_promotion import _publish_race_worker\n"
        "_publish_race_worker(*sys.argv[1:])\n"
    )
    processes = [
        subprocess.Popen(
            [
                sys.executable,
                "-c",
                worker_code,
                str(tmp_path),
                str(source),
                str(target),
                str(ready_path),
                str(start_path),
                str(result_path),
                label,
            ],
            cwd=Path(__file__).resolve().parents[2],
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        for source, ready_path, result_path in zip(sources, ready_paths, result_paths)
    ]
    try:
        ready_deadline = time.monotonic() + _PROCESS_BARRIER_TIMEOUT_S
        while not all(path.exists() for path in ready_paths):
            if time.monotonic() > ready_deadline:
                raise AssertionError("worker processes did not reach the filesystem barrier")
            time.sleep(0.01)
        start_path.write_text("start\n", encoding="utf-8")
        outputs = [process.communicate(timeout=15) for process in processes]
        for process, output in zip(processes, outputs):
            assert process.returncode == 0, output
        outcomes = [json.loads(path.read_text(encoding="utf-8")) for path in result_paths]
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.communicate(timeout=5)

    assert sorted(outcome["status"] for outcome in outcomes) == ["conflict", "published"]
    assert target.exists()
    if source_kind == "file":
        assert target.read_bytes() in {b"foreign-process-a\n", b"foreign-process-b\n"}
    else:
        assert (target / "payload.bin").read_bytes() in {b"foreign-process-a\n", b"foreign-process-b\n"}
    assert sum(source.exists() for source in sources) == 1


def test_per_package_lock_serializes_validation_and_publication(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    draft = _draft(tmp_path)
    promoter = CatalogPromoter(tmp_path)
    entered = threading.Event()
    release = threading.Event()
    original = promoter._copy_tree
    first_call = True

    def block_first_copy(source, target) -> None:
        nonlocal first_call
        if first_call:
            first_call = False
            entered.set()
            assert release.wait(5)
        original(source, target)

    monkeypatch.setattr(promoter, "_copy_tree", block_first_copy)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(promoter.promote, draft)
        assert entered.wait(5)
        second = pool.submit(promoter.promote, draft)
        assert not second.done()
        release.set()
        first_result = first.result(timeout=5)
        second_result = second.result(timeout=5)

    assert first_result == second_result
    assert _targets(tmp_path)[0].is_dir()


def test_existing_partial_target_is_a_conflict_and_is_not_overwritten(tmp_path: Path) -> None:
    draft = _draft(tmp_path)
    package_target, qualification_target, evidence_target = _targets(tmp_path)
    package_target.mkdir(parents=True)
    original = package_target / "world.package.yaml"
    original.write_text("different package\n", encoding="utf-8")

    with pytest.raises(ImportFailure, match=r"incomplete|overwrite|conflict|identity"):
        CatalogPromoter(tmp_path).promote(draft)

    assert original.read_text(encoding="utf-8") == "different package\n"
    assert not qualification_target.exists()
    assert not evidence_target.exists()
