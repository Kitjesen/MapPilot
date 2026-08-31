# ruff: noqa: S101
"""Contracts for SimStudio's managed import source inbox."""

from __future__ import annotations

import asyncio
import io
import zipfile
from pathlib import Path

import pytest
from tools.simstudio.service.package_service import PackageImportService
from tools.simstudio.service.source_inbox import SourceInboxError, SourceInboxService
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[2]


async def _chunks(payload: bytes, size: int = 7):
    for offset in range(0, len(payload), size):
        yield payload[offset : offset + size]


def _robot_archive() -> bytes:
    model = """<mujoco model="studio_bot">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base_link">
      <joint name="floating_base_joint" type="free"/>
      <geom name="body_visual" type="box" size="0.2 0.1 0.05" rgba="0.2 0.4 0.6 1"/>
      <site name="imu" pos="0 0 0.1" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
"""
    payload = io.BytesIO()
    with zipfile.ZipFile(payload, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("LICENSE.txt", "SimStudio test license\n")
        archive.writestr("robot.xml", model)
    return payload.getvalue()


def _mixed_source_archive() -> bytes:
    payload = io.BytesIO()
    with zipfile.ZipFile(payload, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("LICENSE.txt", "SimStudio test license\n")
        archive.writestr("config.xml", "<config/>\n")
        archive.writestr("models/robot.xml", "<mujoco model='inspection_bot'><worldbody/></mujoco>\n")
        archive.writestr("models/robot.urdf", "<robot name='inspection_bot'/>\n")
        archive.writestr("meshes/body.stl", b"solid body\nendsolid body\n")
        archive.writestr("terrain/height.r16", b"\x00\x00\xff\xff")
        archive.writestr("textures/albedo.png", b"not-a-rendered-png")
        archive.writestr("README.md", "inspection fixture\n")
    return payload.getvalue()


def _robot_request() -> dict[str, object]:
    return {
        "schema": "lingtu.sim.robot-import-request.v1",
        "id": "studio_bot",
        "version": "1.0.0",
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "units": {"length": "m", "angle": "radian"},
        "provenance": {
            "owner": "LingTu Test",
            "license": "Test-Only",
            "license_file": "LICENSE.txt",
            "source_uri": "file://simstudio-upload",
        },
        "physics": {"attach_root": "base_link", "root_joint": "floating_base_joint"},
        "visual": {"binding": "RobotVisual:StudioBot"},
        "semantic": {"class": "wheeled_robot"},
        "frames": [
            {"name": "base_link", "role": "body"},
            {"name": "imu", "role": "sensor_mount"},
        ],
        "interfaces": {
            "state": ["lingtu.sim.base-state.v1"],
            "command": ["lingtu.sim.base-velocity.v1"],
        },
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {"locomotion": ["drive"], "sensor_mounts": ["imu"]},
    }


def test_upload_assigns_distinct_source_ids_and_lists_archives(tmp_path: Path) -> None:
    inbox = SourceInboxService(tmp_path / "inbox")
    payload = _robot_archive()

    first = asyncio.run(inbox.upload("studio-bot.zip", _chunks(payload)))
    second = asyncio.run(inbox.upload("renamed.zip", _chunks(payload, 11)))

    assert first["schema"] == "lingtu.sim.studio.inbox-source.v1"
    assert len(first["source_id"]) == 32
    assert first["entry"] == f"objects/{first['source_id']}.zip"
    assert first["bytes"] == len(payload)
    assert second["source_id"] != first["source_id"]
    assert second["entry"] == f"objects/{second['source_id']}.zip"
    assert (tmp_path / "inbox" / Path(*first["entry"].split("/"))).read_bytes() == payload
    listed = inbox.list_sources()
    assert listed["schema"] == "lingtu.sim.studio.inbox-source-list.v1"
    assert {source["source_id"] for source in listed["sources"]} == {
        first["source_id"],
        second["source_id"],
    }
    assert all(source["entry"] == f"objects/{source['source_id']}.zip" for source in listed["sources"])


def test_upload_rejects_unsafe_name_and_size_without_partial_publication(tmp_path: Path) -> None:
    inbox = SourceInboxService(tmp_path / "inbox", max_upload_bytes=4)

    with pytest.raises(SourceInboxError) as unsafe:
        asyncio.run(inbox.upload("../robot.zip", _chunks(b"1234")))
    assert unsafe.value.code == "SIMSTUDIO_INBOX_UNSAFE_NAME"

    with pytest.raises(SourceInboxError) as oversized:
        asyncio.run(inbox.upload("robot.zip", _chunks(b"12345")))
    assert oversized.value.code == "SIMSTUDIO_INBOX_SIZE_LIMIT"
    assert inbox.list_sources()["sources"] == []
    assert not list((tmp_path / "inbox").rglob("*.tmp"))


def test_source_inspection_classifies_import_candidates_and_recommends_defaults(tmp_path: Path) -> None:
    inbox = SourceInboxService(tmp_path / "inbox")
    uploaded = asyncio.run(inbox.upload("mixed-source.zip", _chunks(_mixed_source_archive())))

    inspection = inbox.inspect(uploaded["source_id"])

    assert inspection["schema"] == "lingtu.sim.studio.source-inspection.v1"
    assert inspection["source"] == {
        "entry": uploaded["entry"],
        "source_id": uploaded["source_id"],
        "bytes": len(_mixed_source_archive()),
        "archive_format": "zip",
    }
    assert inspection["summary"] == {"files": 8, "total_bytes": 183}
    assert [item["path"] for item in inspection["candidates"]["robot_models"]] == [
        "models/robot.xml",
        "models/robot.urdf",
    ]
    assert [item["format"] for item in inspection["candidates"]["robot_models"]] == ["mjcf", "urdf"]
    assert [item["path"] for item in inspection["candidates"]["licenses"]] == ["LICENSE.txt"]
    assert [item["path"] for item in inspection["candidates"]["heightmaps"]] == ["terrain/height.r16"]
    assert [item["path"] for item in inspection["candidates"]["meshes"]] == ["meshes/body.stl"]
    assert [item["path"] for item in inspection["candidates"]["textures"]] == ["textures/albedo.png"]
    assert inspection["recommendations"] == {
        "robot": {
            "source_format": "mjcf",
            "source_model": "models/robot.xml",
            "license_file": "LICENSE.txt",
        },
        "world": {
            "heightmap": "terrain/height.r16",
            "mesh": "meshes/body.stl",
            "license_file": "LICENSE.txt",
        },
    }
    assert sorted(item["path"] for item in inspection["files"]) == [
        "LICENSE.txt",
        "README.md",
        "config.xml",
        "meshes/body.stl",
        "models/robot.urdf",
        "models/robot.xml",
        "terrain/height.r16",
        "textures/albedo.png",
    ]
    assert not list((tmp_path / "inbox" / ".staging").iterdir())


def test_source_inspection_rejects_unsafe_archive_without_leaving_extracted_files(tmp_path: Path) -> None:
    payload = io.BytesIO()
    with zipfile.ZipFile(payload, "w") as archive:
        archive.writestr("safe/robot.xml", "<mujoco/>\n")
        archive.writestr("../escaped.xml", "<mujoco/>\n")
    inbox = SourceInboxService(tmp_path / "inbox")
    uploaded = asyncio.run(inbox.upload("unsafe.zip", _chunks(payload.getvalue())))

    with pytest.raises(SourceInboxError) as invalid_id:
        inbox.inspect("../../escape")
    assert invalid_id.value.code == "SIMSTUDIO_INBOX_INVALID_SOURCE_ID"

    with pytest.raises(SourceInboxError) as failure:
        inbox.inspect(uploaded["source_id"])

    assert failure.value.code == "SIMSTUDIO_INBOX_INSPECTION_FAILED"
    assert failure.value.details["diagnostic"]["code"] == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not (tmp_path / "escaped.xml").exists()
    assert not list((tmp_path / "inbox" / ".staging").iterdir())


def test_uploaded_robot_archive_flows_directly_into_a_ready_import_job(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    inbox = SourceInboxService(store.root / "inbox")
    uploaded = asyncio.run(inbox.upload("studio-bot.zip", _chunks(_robot_archive())))
    packages = PackageImportService.from_repository(repo_root=REPO_ROOT, store=store)

    job = packages.create_import_job(
        kind="robot",
        request=_robot_request(),
        source_entry=uploaded["entry"],
    )

    assert job["status"] == "READY"
    assert job["payload"]["source_path"] == uploaded["entry"]
    assert job["payload"]["source_identity"]["kind"] == "file"
    assert job["payload"]["source_identity"]["bytes"] == len(_robot_archive())
    assert job["payload"]["result"]["package"] == {
        "kind": "robot",
        "id": "studio_bot",
        "version": "1.0.0",
        "ref": "studio_bot@1.0.0",
    }


def test_import_contracts_supply_canonical_robot_and_world_templates(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    packages = PackageImportService.from_repository(repo_root=REPO_ROOT, store=store)

    robot = packages.import_contract("robot")
    world = packages.import_contract("world")

    assert robot["schema"] == "lingtu.sim.studio.import-contract.v1"
    assert robot["kind"] == "robot"
    assert robot["source_entry_owned_by"] == "simstudio"
    assert robot["request_schema"]["path"] == "schemas/simulation/robot-import.v1.json"
    assert "source" not in robot["request_template"]
    assert robot["request_template"]["schema"] == "lingtu.sim.robot-import-request.v1"
    assert robot["request_template"]["source_model"] == "robot.xml"
    assert world["kind"] == "world"
    assert world["request_schema"]["path"] == "schemas/simulation/world-import.v1.json"
    assert "path" not in world["request_template"]["source"]
    assert world["request_template"]["heightmap"]["path"] == "height.r16"

    with pytest.raises(Exception) as unsupported:
        packages.import_contract("scenario")
    assert getattr(unsupported.value, "code", "") == "PACKAGE_UNSUPPORTED_IMPORT_KIND"


def test_http_upload_and_source_listing_expose_only_managed_entries(tmp_path: Path) -> None:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from tools.simstudio.http.app import create_app
    from tools.simstudio.service.application import SimulationStudioService
    from tools.simstudio.service.http import API_PREFIX

    service = SimulationStudioService.from_repository(REPO_ROOT, artifact_root=tmp_path / "studio")
    payload = _robot_archive()

    with TestClient(create_app(service)) as client:
        uploaded_response = client.post(
            f"{API_PREFIX}/inbox/uploads",
            content=payload,
            headers={
                "Content-Type": "application/octet-stream",
                "X-SimStudio-Filename": "studio-bot.zip",
            },
        )
        uploaded_payload = uploaded_response.json()["result"]
        listed_response = client.get(f"{API_PREFIX}/inbox/sources")
        inspection_response = client.get(
            f"{API_PREFIX}/inbox/sources/{uploaded_payload['source_id']}/inspection"
        )
        contract_response = client.get(f"{API_PREFIX}/import-contracts/robot")
        unsafe_response = client.post(
            f"{API_PREFIX}/inbox/uploads",
            content=payload,
            headers={"X-SimStudio-Filename": "../escape.zip"},
        )

    assert uploaded_response.status_code == 201
    uploaded = uploaded_payload
    assert uploaded["entry"] == f"objects/{uploaded['source_id']}.zip"
    assert listed_response.status_code == 200
    assert listed_response.json()["result"]["sources"][0]["entry"] == uploaded["entry"]
    assert inspection_response.status_code == 200
    assert inspection_response.json()["result"]["recommendations"]["robot"] == {
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "license_file": "LICENSE.txt",
    }
    assert contract_response.status_code == 200
    assert contract_response.json()["result"]["request_template"]["source_model"] == "robot.xml"
    assert unsafe_response.status_code == 422
    assert unsafe_response.json()["error"]["code"] == "SIMSTUDIO_INBOX_UNSAFE_NAME"
