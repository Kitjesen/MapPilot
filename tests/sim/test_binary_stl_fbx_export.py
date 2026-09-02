# ruff: noqa: S101
"""Contract tests for the deterministic Thunder STL-to-FBX asset pipeline."""

from __future__ import annotations

import hashlib
import json
import os
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.tools.assets import export_binary_stl_fbx
from sim.tools.assets.export_binary_stl_fbx import (
    StlAssetError,
    build_box_projected_uv_loops,
    build_conversion_plan,
    build_conversion_source_plan,
    build_mjcf_conversion_plan,
    build_mjcf_conversion_source_plan,
    discover_stl_sources,
    export_fbx_assets,
    read_binary_stl,
    runtime_decimation_ratio,
    validate_runtime_triangle_budget,
)


def _write_single_triangle_stl(path: Path) -> bytes:
    header = b"LingTu STL fixture".ljust(80, b"\0")
    triangle = struct.pack(
        "<12fH",
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0,
    )
    payload = header + struct.pack("<I", 1) + triangle
    path.write_bytes(payload)
    return payload


def test_reads_binary_stl_as_indexed_mesh(tmp_path: Path) -> None:
    source = tmp_path / "link.stl"
    payload = _write_single_triangle_stl(source)

    mesh = read_binary_stl(source)

    assert mesh.vertices == ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 2.0, 0.0))
    assert mesh.faces == ((0, 1, 2),)
    assert mesh.triangle_count == 1
    assert mesh.bounds_min == (0.0, 0.0, 0.0)
    assert mesh.bounds_max == (1.0, 2.0, 0.0)
    assert mesh.source_sha256 == hashlib.sha256(payload).hexdigest()


def test_stl_header_budget_is_checked_before_triangle_storage(tmp_path: Path) -> None:
    source = tmp_path / "declared-huge.stl"
    source.write_bytes(
        b"budget fixture".ljust(80, b"\0")
        + struct.pack("<I", export_binary_stl_fbx.MAX_STL_TRIANGLES + 1)
    )

    with pytest.raises(StlAssetError, match="triangle budget"):
        read_binary_stl(source)


def test_stl_reader_uses_one_open_handle_when_source_path_is_replaced(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "link.stl"
    original_payload = _write_single_triangle_stl(source)
    replacement = tmp_path / "replacement.stl"
    replacement.write_bytes(b"replacement")
    original_read = os.read
    replaced = False

    def replacing_read(descriptor: int, size: int) -> bytes:
        nonlocal replaced
        chunk = original_read(descriptor, size)
        if not replaced:
            replaced = True
            try:
                source.replace(tmp_path / "original-open-file.stl")
                replacement.replace(source)
            except PermissionError:
                # Windows denies delete-sharing for the protected source handle.
                pass
        return chunk

    monkeypatch.setattr(export_binary_stl_fbx.os, "read", replacing_read)

    mesh = read_binary_stl(source)

    assert mesh.triangle_count == 1
    assert mesh.source_sha256 == hashlib.sha256(original_payload).hexdigest()


def test_stl_reader_rejects_symbolic_links(tmp_path: Path) -> None:
    source = tmp_path / "source.stl"
    _write_single_triangle_stl(source)
    link = tmp_path / "link.stl"
    try:
        link.symlink_to(source)
    except OSError:
        pytest.skip("symbolic links are not available on this host")

    with pytest.raises(StlAssetError, match="regular link-free file"):
        read_binary_stl(link)


def _install_synthetic_private_export(
    monkeypatch: pytest.MonkeyPatch,
    *,
    fail_after_fbx: bool = False,
    index_blender_version: str = "test-blender",
    reported_blender_version: str | None = None,
) -> None:
    def build_private_export(
        _source_dir: Path | None,
        output_dir: Path,
        **_kwargs: object,
    ) -> export_binary_stl_fbx._PreparedFbxExport:
        output_dir.mkdir(parents=True, exist_ok=True)
        fbx = output_dir / "link.fbx"
        fbx.write_bytes(b"synthetic fbx")
        if fail_after_fbx:
            raise StlAssetError("synthetic Blender failure")
        script_path = Path(export_binary_stl_fbx.__file__)
        index = output_dir / "asset-index.json"
        index.write_text(
            json.dumps(
                {
                    "schema": "lingtu.sim.fbx-asset-index.v1",
                    "generator": {
                        "script": script_path.name,
                        "script_sha256": hashlib.sha256(script_path.read_bytes()).hexdigest(),
                        "blender_version": index_blender_version,
                    },
                    "assets": [
                        {
                            "fbx": fbx.name,
                            "fbx_bytes": fbx.stat().st_size,
                            "fbx_sha256": hashlib.sha256(fbx.read_bytes()).hexdigest(),
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        return export_binary_stl_fbx._PreparedFbxExport(
            index_path=index,
            blender_version=reported_blender_version or index_blender_version,
        )

    monkeypatch.setattr(
        export_binary_stl_fbx,
        "_export_fbx_assets_to_directory",
        build_private_export,
    )


def test_export_publishes_one_complete_randomly_staged_output_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch)
    output = tmp_path / "published"

    index = export_fbx_assets(tmp_path / "unused", output)

    assert index == output / "asset-index.json"
    assert {path.name for path in output.iterdir()} == {"asset-index.json", "link.fbx"}
    assert not list(tmp_path.glob(".published.staging-*"))


def test_export_failure_leaves_no_partial_formal_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch, fail_after_fbx=True)
    output = tmp_path / "published"

    with pytest.raises(StlAssetError, match="synthetic Blender failure"):
        export_fbx_assets(tmp_path / "unused", output)

    assert not output.exists()
    assert not list(tmp_path.glob(".published.staging-*"))


def test_export_never_replaces_an_existing_output_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch)
    output = tmp_path / "published"
    output.mkdir()
    marker = output / "winner.txt"
    marker.write_text("winner", encoding="utf-8")

    with pytest.raises(StlAssetError, match="already exists"):
        export_fbx_assets(tmp_path / "unused", output)

    assert marker.read_text(encoding="utf-8") == "winner"
    assert not list(tmp_path.glob(".published.staging-*"))


def test_export_rejects_an_index_with_a_tampered_blender_version(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(
        monkeypatch,
        index_blender_version="tampered-version",
        reported_blender_version="4.3.2-exact",
    )
    output = tmp_path / "published"

    with pytest.raises(StlAssetError, match="not bound to this exporter and Blender"):
        export_fbx_assets(tmp_path / "unused", output)

    assert not output.exists()
    assert not list(tmp_path.glob(".published.staging-*"))


def test_export_rejects_a_linked_output_parent(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch)
    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    try:
        linked_parent.symlink_to(real_parent, target_is_directory=True)
    except OSError:
        pytest.skip("symbolic links are not available on this host")

    with pytest.raises(StlAssetError, match="must not traverse a link"):
        export_fbx_assets(
            tmp_path / "unused",
            linked_parent / "new-parent" / "published",
        )

    assert not (real_parent / "new-parent").exists()


def test_atomic_rename_is_the_last_fallible_export_step(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch)
    committed = False
    original_rename = export_binary_stl_fbx._rename_directory_noreplace
    original_hash = export_binary_stl_fbx._hash_regular_bounded

    def committing_rename(
        anchor: export_binary_stl_fbx._DirectoryAnchor,
        source_name: str,
        target_name: str,
    ) -> None:
        nonlocal committed
        original_rename(anchor, source_name, target_name)
        committed = True

    def hash_only_before_commit(
        path: Path,
        *,
        limit: int,
        context: str,
    ) -> tuple[int, str]:
        if committed:
            raise AssertionError("hash validation ran after the commit point")
        return original_hash(path, limit=limit, context=context)

    monkeypatch.setattr(export_binary_stl_fbx, "_rename_directory_noreplace", committing_rename)
    monkeypatch.setattr(export_binary_stl_fbx, "_hash_regular_bounded", hash_only_before_commit)

    index = export_fbx_assets(tmp_path / "unused", tmp_path / "published")

    assert committed is True
    assert index.is_file()


def test_post_commit_anchor_close_error_does_not_report_export_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch)
    original_close = export_binary_stl_fbx._DirectoryAnchor.close

    def close_then_fail(anchor: export_binary_stl_fbx._DirectoryAnchor) -> None:
        original_close(anchor)
        raise OSError("synthetic post-commit close error")

    monkeypatch.setattr(export_binary_stl_fbx._DirectoryAnchor, "close", close_then_fail)

    index = export_fbx_assets(tmp_path / "unused", tmp_path / "published")

    assert index.is_file()


def test_cleanup_failure_reports_the_residual_private_staging(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_synthetic_private_export(monkeypatch, fail_after_fbx=True)
    original_rmtree = export_binary_stl_fbx.shutil.rmtree

    def failing_rmtree(_path: Path) -> None:
        raise PermissionError("synthetic cleanup denial")

    monkeypatch.setattr(export_binary_stl_fbx.shutil, "rmtree", failing_rmtree)

    with pytest.raises(StlAssetError, match="residual staging"):
        export_fbx_assets(tmp_path / "unused", tmp_path / "published")

    residual = list(tmp_path.glob(".published.staging-*"))
    assert len(residual) == 1
    original_rmtree(residual[0])


def test_private_staging_creation_self_cleans_after_post_create_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    anchor = export_binary_stl_fbx._open_directory_anchor(tmp_path)

    def fail_post_create_check() -> None:
        raise StlAssetError("synthetic post-create failure")

    try:
        monkeypatch.setattr(anchor, "assert_stable", fail_post_create_check)
        with pytest.raises(StlAssetError, match="synthetic post-create failure"):
            export_binary_stl_fbx._create_private_staging(anchor, "published")
        assert not list(tmp_path.glob(".published.staging-*"))
    finally:
        anchor.close()


def test_private_staging_creation_reports_residual_when_self_cleanup_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    anchor = export_binary_stl_fbx._open_directory_anchor(tmp_path)

    def fail_post_create_check() -> None:
        raise StlAssetError("synthetic post-create failure")

    def fail_cleanup(*_args: object, **_kwargs: object) -> None:
        raise PermissionError("synthetic cleanup denial")

    try:
        with monkeypatch.context() as context:
            context.setattr(anchor, "assert_stable", fail_post_create_check)
            context.setattr(export_binary_stl_fbx.os, "rmdir", fail_cleanup)
            with pytest.raises(StlAssetError, match="residual staging"):
                export_binary_stl_fbx._create_private_staging(anchor, "published")
        residual = list(tmp_path.glob(".published.staging-*"))
        assert len(residual) == 1
        residual[0].rmdir()
    finally:
        anchor.close()


def test_builds_non_degenerate_box_projected_uvs_for_stl_triangles(
    tmp_path: Path,
) -> None:
    source = tmp_path / "link.stl"
    _write_single_triangle_stl(source)
    mesh = read_binary_stl(source)

    uv_loops = build_box_projected_uv_loops(mesh)

    assert uv_loops == ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0))
    first, second, third = uv_loops
    signed_double_area = (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )
    assert abs(signed_double_area) > 1.0e-8


def test_discovers_stl_sources_in_stable_name_order(tmp_path: Path) -> None:
    _write_single_triangle_stl(tmp_path / "z_link.stl")
    _write_single_triangle_stl(tmp_path / "A_link.STL")
    (tmp_path / "ignore.txt").write_text("not a mesh", encoding="utf-8")

    sources = discover_stl_sources(tmp_path)

    assert [source.name for source in sources] == ["A_link.STL", "z_link.stl"]


def test_discovery_rejects_linked_stl_sources(tmp_path: Path) -> None:
    source = tmp_path / "source.stl"
    _write_single_triangle_stl(source)
    link = tmp_path / "linked.stl"
    try:
        link.symlink_to(source)
    except OSError:
        pytest.skip("symbolic links are not available on this host")

    with pytest.raises(StlAssetError, match="regular link-free file"):
        discover_stl_sources(tmp_path)


def test_builds_fbx_conversion_plan_from_source_content(tmp_path: Path) -> None:
    source_dir = tmp_path / "stl"
    output_dir = tmp_path / "fbx"
    source_dir.mkdir()
    payload = _write_single_triangle_stl(source_dir / "base_link.stl")

    plan = build_conversion_plan(source_dir, output_dir)

    assert len(plan) == 1
    assert plan[0].asset_name == "base_link"
    assert plan[0].source == (source_dir / "base_link.stl").resolve()
    assert plan[0].target == (output_dir / "base_link.fbx").resolve()
    assert plan[0].mesh.triangle_count == 1
    assert plan[0].mesh.source_sha256 == hashlib.sha256(payload).hexdigest()


def test_runtime_triangle_budget_selects_decimation_and_rejects_over_budget() -> None:
    assert runtime_decimation_ratio(40_000, 60_000) == 1.0
    assert runtime_decimation_ratio(120_000, 60_000) == 0.5
    assert (
        validate_runtime_triangle_budget(
            [60_000, 59_999, 2_000],
            max_triangles_per_asset=60_000,
            max_total_triangles=125_000,
        )
        == 121_999
    )

    with pytest.raises(StlAssetError, match="per-asset triangle budget"):
        validate_runtime_triangle_budget(
            [60_001],
            max_triangles_per_asset=60_000,
            max_total_triangles=125_000,
        )
    with pytest.raises(StlAssetError, match="total triangle budget"):
        validate_runtime_triangle_budget(
            [60_000, 60_000],
            max_triangles_per_asset=60_000,
            max_total_triangles=119_999,
        )


def test_builds_lightweight_source_plan_without_decoding_geometry(tmp_path: Path) -> None:
    source_dir = tmp_path / "stl"
    output_dir = tmp_path / "fbx"
    source_dir.mkdir()
    _write_single_triangle_stl(source_dir / "base_link.stl")

    plan = build_conversion_source_plan(source_dir, output_dir)

    assert len(plan) == 1
    assert plan[0].asset_name == "base_link"
    assert plan[0].source == (source_dir / "base_link.stl").resolve()
    assert plan[0].target == (output_dir / "base_link.fbx").resolve()
    assert not hasattr(plan[0], "mesh")


def test_source_plan_rejects_casefolded_fbx_target_collisions(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source_dir = tmp_path / "stl"
    source_dir.mkdir()
    monkeypatch.setattr(
        export_binary_stl_fbx,
        "discover_stl_sources",
        lambda _source_dir: (source_dir / "Link.stl", source_dir / "link.STL"),
    )

    with pytest.raises(StlAssetError, match="case-insensitive FBX target collision"):
        build_conversion_source_plan(source_dir, tmp_path / "fbx")


def test_mjcf_plan_exports_only_referenced_visual_meshes(tmp_path: Path) -> None:
    meshes = tmp_path / "meshes"
    model_dir = tmp_path / "mjcf"
    output_dir = tmp_path / "fbx"
    meshes.mkdir()
    model_dir.mkdir()
    _write_single_triangle_stl(meshes / "base.stl")
    _write_single_triangle_stl(meshes / "unused.stl")
    mjcf = model_dir / "robot.xml"
    mjcf.write_text(
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset>
    <mesh name="base_visual" file="base.stl" />
    <mesh name="unused_visual" file="unused.stl" />
  </asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    plan = build_mjcf_conversion_plan(mjcf, output_dir)

    assert [item.asset_name for item in plan] == ["base_visual"]
    assert [item.source.name for item in plan] == ["base.stl"]
    assert [item.target.name for item in plan] == ["base_visual.fbx"]

    source_plan = build_mjcf_conversion_source_plan(mjcf, output_dir)
    assert [item.asset_name for item in source_plan] == ["base_visual"]
    assert not hasattr(source_plan[0], "mesh")


def test_mjcf_plan_can_use_a_separate_visual_mesh_root(tmp_path: Path) -> None:
    physics_meshes = tmp_path / "physics"
    visual_meshes = tmp_path / "visual"
    model_dir = tmp_path / "mjcf"
    output_dir = tmp_path / "fbx"
    physics_meshes.mkdir()
    visual_meshes.mkdir()
    model_dir.mkdir()
    _write_single_triangle_stl(physics_meshes / "base.stl")
    visual_payload = _write_single_triangle_stl(visual_meshes / "base.stl") + b"detail"
    (visual_meshes / "base.stl").write_bytes(visual_payload)
    mjcf = model_dir / "robot.xml"
    mjcf.write_text(
        """\
<mujoco>
  <compiler meshdir="../physics" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    plan = build_mjcf_conversion_source_plan(
        mjcf,
        output_dir,
        source_mesh_root=visual_meshes,
    )

    assert plan[0].asset_name == "base_visual"
    assert plan[0].source == (visual_meshes / "base.stl").resolve()
    assert plan[0].target == (output_dir / "base_visual.fbx").resolve()


@pytest.mark.parametrize("unsafe_name", ["../outside", "folder/mesh", r"folder\mesh"])
def test_mjcf_plan_rejects_asset_names_that_escape_one_fbx_file(
    tmp_path: Path,
    unsafe_name: str,
) -> None:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    _write_single_triangle_stl(meshes / "base.stl")
    mjcf = tmp_path / "robot.xml"
    mjcf.write_text(
        f"""\
<mujoco>
  <compiler meshdir="meshes" />
  <asset><mesh name="{unsafe_name}" file="base.stl" /></asset>
  <worldbody><body><geom type="mesh" mesh="{unsafe_name}" /></body></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    with pytest.raises(StlAssetError, match="safe asset identifier"):
        build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")


def test_mjcf_plan_rejects_casefolded_fbx_target_collisions(tmp_path: Path) -> None:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    _write_single_triangle_stl(meshes / "first.stl")
    _write_single_triangle_stl(meshes / "second.stl")
    mjcf = tmp_path / "robot.xml"
    mjcf.write_text(
        """\
<mujoco>
  <compiler meshdir="meshes" />
  <asset>
    <mesh name="Link" file="first.stl" />
    <mesh name="link" file="second.stl" />
  </asset>
  <worldbody><body>
    <geom type="mesh" mesh="Link" />
    <geom type="mesh" mesh="link" />
  </body></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    with pytest.raises(StlAssetError, match="case-insensitive FBX target collision"):
        build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")


def test_mjcf_reader_rejects_dtds_and_entities(tmp_path: Path) -> None:
    mjcf = tmp_path / "robot.xml"
    mjcf.write_text(
        """\
<!DOCTYPE mujoco [<!ENTITY meshdir "meshes">]>
<mujoco><compiler meshdir="&meshdir;" /></mujoco>
""",
        encoding="utf-8",
    )

    with pytest.raises(StlAssetError, match="must not declare DTDs or entities"):
        build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")


@pytest.mark.parametrize(
    "payload",
    [
        "<mujoco />".encode("utf-16-le"),
        "<mujoco />".encode("utf-16-be"),
        "<mujoco />".encode("utf-16"),
        b"\xfe\xff" + "<mujoco />".encode("utf-16-be"),
    ],
    ids=["utf16-le", "utf16-be", "utf16-bom-native", "utf16-bom-be"],
)
def test_mjcf_reader_rejects_all_utf16_encodings(
    tmp_path: Path,
    payload: bytes,
) -> None:
    mjcf = tmp_path / "robot.xml"
    mjcf.write_bytes(payload)

    with pytest.raises(StlAssetError, match="must use UTF-8 or UTF-8-SIG"):
        build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")


def test_mjcf_reader_accepts_utf8_sig(tmp_path: Path) -> None:
    mjcf = tmp_path / "robot.xml"
    mjcf.write_bytes(b"\xef\xbb\xbf<mujoco />")

    plan = build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")

    assert plan == ()


def test_mjcf_reader_rejects_symbolic_links(tmp_path: Path) -> None:
    source = tmp_path / "source.xml"
    source.write_text("<mujoco />", encoding="utf-8")
    link = tmp_path / "robot.xml"
    try:
        link.symlink_to(source)
    except OSError:
        pytest.skip("symbolic links are not available on this host")

    with pytest.raises(StlAssetError, match="regular link-free file"):
        build_mjcf_conversion_source_plan(link, tmp_path / "fbx")


def test_mjcf_reader_uses_one_nofollow_handle_during_path_replacement(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    _write_single_triangle_stl(meshes / "base.stl")
    mjcf = tmp_path / "robot.xml"
    mjcf.write_text(
        """\
<mujoco>
  <compiler meshdir="meshes" />
  <asset><mesh name="base" file="base.stl" /></asset>
  <worldbody><geom type="mesh" mesh="base" /></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    replacement = tmp_path / "replacement.xml"
    replacement.write_text("<!DOCTYPE mujoco><mujoco />", encoding="utf-8")
    original_read = os.read
    replaced = False

    def replacing_read(descriptor: int, size: int) -> bytes:
        nonlocal replaced
        chunk = original_read(descriptor, size)
        if not replaced:
            replaced = True
            try:
                mjcf.replace(tmp_path / "original-open.xml")
                replacement.replace(mjcf)
            except PermissionError:
                pass
        return chunk

    monkeypatch.setattr(export_binary_stl_fbx.os, "read", replacing_read)

    plan = build_mjcf_conversion_source_plan(mjcf, tmp_path / "fbx")

    assert [item.asset_name for item in plan] == ["base"]


@pytest.mark.parametrize(
    ("budget_name", "budget_value", "message"),
    [
        ("MAX_TOTAL_STL_TRIANGLES", 1, "total triangle budget"),
        ("MAX_TOTAL_STL_BYTES", 200, "total physical byte budget"),
    ],
)
def test_selected_stl_set_is_preflighted_before_blender_initialization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    budget_name: str,
    budget_value: int,
    message: str,
) -> None:
    source_dir = tmp_path / "stl"
    output_dir = tmp_path / "private-output"
    source_dir.mkdir()
    output_dir.mkdir()
    _write_single_triangle_stl(source_dir / "first.stl")
    _write_single_triangle_stl(source_dir / "second.stl")
    read_factory_called = False

    def read_factory_settings(**_kwargs: object) -> None:
        nonlocal read_factory_called
        read_factory_called = True

    monkeypatch.setattr(export_binary_stl_fbx, budget_name, budget_value)
    monkeypatch.setitem(
        sys.modules,
        "bpy",
        SimpleNamespace(
            app=SimpleNamespace(version_string="4.3.2"),
            ops=SimpleNamespace(wm=SimpleNamespace(read_factory_settings=read_factory_settings)),
        ),
    )
    monkeypatch.setitem(sys.modules, "bmesh", SimpleNamespace())

    with pytest.raises(StlAssetError, match=message):
        export_binary_stl_fbx._export_fbx_assets_to_directory(source_dir, output_dir)

    assert read_factory_called is False


def test_unknown_blender_version_is_rejected_before_scene_reset(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source_dir = tmp_path / "stl"
    output_dir = tmp_path / "private-output"
    source_dir.mkdir()
    output_dir.mkdir()
    _write_single_triangle_stl(source_dir / "link.stl")
    read_factory_called = False

    def read_factory_settings(**_kwargs: object) -> None:
        nonlocal read_factory_called
        read_factory_called = True

    monkeypatch.setitem(
        sys.modules,
        "bpy",
        SimpleNamespace(
            app=SimpleNamespace(version_string="unknown"),
            ops=SimpleNamespace(wm=SimpleNamespace(read_factory_settings=read_factory_settings)),
        ),
    )
    monkeypatch.setitem(sys.modules, "bmesh", SimpleNamespace())

    with pytest.raises(StlAssetError, match="exact non-unknown version_string"):
        export_binary_stl_fbx._export_fbx_assets_to_directory(source_dir, output_dir)

    assert read_factory_called is False
