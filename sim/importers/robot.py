"""Deterministic MJCF-first RobotPackage importer."""

from __future__ import annotations

import json
import shutil
import uuid
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any, Protocol

import yaml

from sim.catalog.resolver import CatalogError, CatalogResolver
from sim.catalog.visual_binding import VisualBindingError, compile_robot_visual_manifest
from sim.catalog.visual_projection import (
    VisualProjectionError,
    compile_robot_visual_projection,
    validate_robot_visual_projection_matches_manifest,
)

from .contracts import (
    ImportCode,
    ImportDiagnostic,
    ImportDraft,
    ImportFailure,
    assert_no_reparse_components,
    require_identity,
    require_mapping,
    require_string,
    resolve_beneath,
    safe_relative_path,
    strict_keys,
    validate_provenance,
    write_json,
)
from .intake import SourceIntake
from .promotion import CatalogPromoter, _PackageLock


class UrdfToMjcfConverter(Protocol):
    """Injected boundary for URDF conversion; no converter is bundled here."""

    def convert(self, *, urdf: Path, output_dir: Path) -> Path:
        """Convert one URDF into an MJCF file under ``output_dir``."""


class RobotMeshProjectionConverter(Protocol):
    """Injected boundary for mapping source meshes to cooked visual assets."""

    def convert(self, visual_manifest: Mapping[str, Any]) -> Mapping[str, Any]:
        """Return bindings plus structured conversion evidence."""


class MujocoCompiler(Protocol):
    """Injectable MuJoCo compile boundary used by importer tests."""

    def __call__(self, mjcf: Path) -> Any:
        """Compile one MJCF path and return its compiled model."""


_REQUEST_SCHEMA = "lingtu.sim.robot-import-request.v1"
_PACKAGE_SCHEMA = "lingtu.sim.robot-package.v1"
_SUPPORTED_FORMATS = {"mjcf", "urdf"}
_NAMED_TAGS = {"body", "joint", "site", "camera", "geom"}
_ACTUATOR_TAGS = {"motor", "general", "position", "velocity", "cylinder", "muscle"}
_TEXTURE_FILE_ATTRS = ("file", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback")
_JOINT_TRANSMISSION = 0
_FREE_JOINT = 0
_COMPILE_SENTINEL = object()


class RobotImporter:
    """Import a robot source tree into a qualified or quarantined draft package."""

    def __init__(
        self,
        repo_root: Path,
        *,
        work_root: Path | None = None,
        intake: SourceIntake | None = None,
        urdf_converter: UrdfToMjcfConverter | None = None,
        mesh_converter: RobotMeshProjectionConverter | None = None,
        mujoco_compiler: MujocoCompiler | None = None,
    ) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.work_root = Path(work_root or self.repo_root / "sim" / ".imports").resolve()
        self.intake = intake or SourceIntake()
        self.urdf_converter = urdf_converter
        self.mesh_converter = mesh_converter
        self.mujoco_compiler = mujoco_compiler

    def import_robot(self, request: Mapping[str, Any]) -> ImportDraft:
        """Materialize an import draft and fail closed into quarantine."""

        package_id = "unknown"
        version = "unknown"
        import_id = "invalid"
        staging_root: Path | None = None
        try:
            spec = self._normalize_request(request)
            package_id = spec["id"]
            version = spec["version"]
            import_id = f"{package_id}-{version}"
            staging_root = self._new_staging_root()
            draft = self._import_normalized(spec, staging_root)
            import_id = draft.import_id
            return self._publish_draft(draft, staging_root)
        except ImportFailure as exc:
            if staging_root is not None:
                self._remove_staging_root(staging_root)
            return self._quarantine(import_id, package_id, version, exc.to_diagnostic())
        except (CatalogError, VisualBindingError, VisualProjectionError, ET.ParseError, OSError, ValueError) as exc:
            if staging_root is not None:
                self._remove_staging_root(staging_root)
            diagnostic = ImportDiagnostic(
                code=ImportCode.MODEL_INVALID,
                message=str(exc),
                context=package_id if package_id != "unknown" else None,
            )
            return self._quarantine(import_id, package_id, version, diagnostic)
        except Exception:
            if staging_root is not None:
                self._remove_staging_root(staging_root)
            raise

    def validate_package(
        self,
        package_root: Path,
        *,
        _compiled_model: Any = _COMPILE_SENTINEL,
    ) -> dict[str, Any]:
        """Validate an existing RobotPackage through the importer validator path."""

        package_root = Path(package_root).resolve()
        manifest_path = self._single_manifest(package_root)
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping):
            raise ImportFailure("robot package manifest must be an object", code=ImportCode.MODEL_INVALID)
        physics = require_mapping(manifest.get("physics"), "robot.package.physics")
        mjcf = self._resolve_manifest_mjcf(
            package_root,
            manifest_path,
            require_string(physics.get("mjcf"), "robot.package.physics.mjcf"),
        )
        if self._is_beneath(mjcf, package_root / "source"):
            asset_root = package_root / "source"
        elif self._is_beneath(mjcf, package_root):
            asset_root = package_root
        else:
            trusted_asset_root = (self.repo_root / "sim" / "robots").resolve()
            relative_asset = mjcf.relative_to(trusted_asset_root)
            asset_root = trusted_asset_root / relative_asset.parts[0]
        self._validate_mjcf(
            mjcf,
            source_root=asset_root,
            attach_root=require_string(physics.get("attach_root"), "robot.package.physics.attach_root"),
            root_joint=require_string(physics.get("root_joint"), "robot.package.physics.root_joint"),
            frames=manifest.get("frames"),
            commandable=True,
            compiled_model=_compiled_model,
        )
        resolver_root = package_root if self._is_beneath(mjcf, package_root) else self.repo_root
        resolver = CatalogResolver(resolver_root, (package_root,))
        record = resolver.find_package(
            f"{require_string(manifest.get('id'), 'robot.package.id')}@"
            f"{require_string(manifest.get('version'), 'robot.package.version')}",
            kind="robot",
        )
        return {
            "schema": "lingtu.sim.robot-import-validation.v1",
            "package": {"kind": record.kind, "id": record.id, "version": record.version, "ref": record.ref},
            "manifest_path": str(record.manifest_path),
        }

    def _import_normalized(self, spec: dict[str, Any], root: Path) -> ImportDraft:
        intake_root = root / "intake"
        source = self.intake.materialize(Path(spec["source"]), intake_root)
        provenance = validate_provenance(spec["provenance"], source_root=source.root)
        spec = {**spec, "provenance": provenance}
        source_model = resolve_beneath(source.root, spec["source_model"], "source_model")
        package_root = root / "package"
        package_source = package_root / "source"
        shutil.copytree(source.root, package_source, symlinks=False)

        package_model = resolve_beneath(package_source, spec["source_model"], "package source_model")
        conversion_evidence: dict[str, Any] = {
            "urdf": None,
            "mesh": None,
        }
        if spec["source_format"] == "urdf":
            if self.urdf_converter is None:
                raise ImportFailure(
                    "URDF import requires an injected converter",
                    code=ImportCode.CONVERTER_UNAVAILABLE,
                    context="source_format",
                )
            converted = self.urdf_converter.convert(
                urdf=source_model,
                output_dir=package_source / "_converted_mjcf",
            )
            converted_path = Path(converted)
            if not converted_path.is_absolute():
                converted_path = package_source / "_converted_mjcf" / converted_path
            package_model = converted_path.resolve()
            try:
                package_model.relative_to(package_source.resolve())
            except ValueError as exc:
                raise ImportFailure(
                    "URDF converter output must stay inside the package source copy",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=str(package_model),
                ) from exc
            if not package_model.is_file():
                raise ImportFailure(
                    "URDF converter output must identify a regular MJCF file",
                    code=ImportCode.CONVERTER_UNAVAILABLE,
                    context=str(package_model),
                )
            conversion_evidence["urdf"] = {
                "schema": "lingtu.sim.urdf-conversion-evidence.v1",
                "converter": self._converter_identity(self.urdf_converter),
                "source": {"path": source_model.relative_to(source.root).as_posix()},
                "output": {"path": package_model.relative_to(package_root).as_posix()},
            }

        relative_mjcf = package_model.relative_to(package_root).as_posix()
        compiled_model, compile_evidence = self._validate_mjcf(
            package_model,
            source_root=package_source,
            attach_root=spec["physics"]["attach_root"],
            root_joint=spec["physics"]["root_joint"],
            frames=spec["frames"],
            commandable=True,
        )
        manifest = self._manifest(spec, relative_mjcf)
        manifest_path = package_root / "robot.package.yaml"
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_bytes(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True).encode("utf-8"))
        provenance_path = write_json(
            package_root / "provenance" / "robot.provenance.json",
            {
                "schema": "lingtu.sim.robot-provenance.v1",
                "provenance": provenance,
            },
        )

        visual_manifest = compile_robot_visual_manifest(package_root, package_model).to_dict()
        asset_bindings, mesh_evidence = self._asset_bindings(visual_manifest, spec, package_root)
        conversion_evidence["mesh"] = mesh_evidence or None
        projection = compile_robot_visual_projection(visual_manifest, asset_bindings).to_dict()
        projection_path = write_json(package_root / "visual" / "robot.visual-projection.json", projection)
        validate_robot_visual_projection_matches_manifest(projection, visual_manifest)

        validation = self.validate_package(package_root, _compiled_model=compiled_model)
        import_id = f"{spec['id']}-{spec['version']}"
        identity_payload = {
            "schema": "lingtu.sim.robot-import-identity.v1",
            "request": spec,
            "source_content": source.to_dict(),
            "converters": conversion_evidence,
            "mujoco_compile": compile_evidence,
        }
        identity = {**identity_payload, "import_id": import_id}
        write_json(root / "import-identity.json", identity)
        qualification_path = self._write_qualification(
            root,
            spec,
            manifest_path,
            projection_path,
            source,
            validation,
            provenance,
            provenance_path,
            compile_evidence,
            conversion_evidence,
            identity,
        )
        return ImportDraft(
            import_id=import_id,
            kind="robot",
            package_id=spec["id"],
            version=spec["version"],
            state="qualified",
            root=root,
            package_root=package_root,
            manifest_path=manifest_path,
            provenance_path=provenance_path,
            qualification_path=qualification_path,
        )

    def _normalize_request(self, request: Mapping[str, Any]) -> dict[str, Any]:
        data = require_mapping(request, "request")
        strict_keys(
            data,
            required={
                "schema",
                "id",
                "version",
                "source",
                "source_format",
                "source_model",
                "units",
                "provenance",
                "physics",
                "visual",
                "semantic",
                "frames",
                "interfaces",
                "defaults",
                "declared_capabilities",
            },
            optional={"description", "compatibility", "visual_asset_index"},
            context="request",
        )
        if data["schema"] != _REQUEST_SCHEMA:
            raise ImportFailure("request.schema is unsupported", context="request.schema")
        units = require_mapping(data["units"], "request.units")
        strict_keys(units, required={"length", "angle"}, optional=set(), context="request.units")
        if units != {"length": "m", "angle": "radian"}:
            raise ImportFailure(
                "robot imports require explicit length=m and angle=radian",
                code=ImportCode.UNIT_AMBIGUOUS,
                context="request.units",
                details={"expected": {"length": "m", "angle": "radian"}, "actual": units},
            )
        source_format = require_string(data["source_format"], "request.source_format")
        if source_format not in _SUPPORTED_FORMATS:
            raise ImportFailure(
                f"unsupported robot source format {source_format!r}",
                code=ImportCode.SOURCE_FORMAT_UNSUPPORTED,
                context="request.source_format",
            )
        description = data.get("description")
        if description is not None:
            description = require_string(description, "request.description")
        return {
            "schema": _REQUEST_SCHEMA,
            "id": require_identity(data["id"], "request.id"),
            "version": require_identity(data["version"], "request.version", version=True),
            "description": description,
            "compatibility": dict(require_mapping(data.get("compatibility", {}), "request.compatibility")),
            "source": str(Path(require_string(data["source"], "request.source")).expanduser().resolve()),
            "source_format": source_format,
            "source_model": safe_relative_path(data["source_model"], "request.source_model"),
            "units": {"length": "m", "angle": "radian"},
            "provenance": require_mapping(data["provenance"], "request.provenance"),
            "physics": self._physics(data["physics"]),
            "visual": self._visual(data["visual"]),
            "semantic": self._semantic(data["semantic"]),
            "frames": self._frames(data["frames"]),
            "interfaces": self._interfaces(data["interfaces"]),
            "defaults": self._defaults(data["defaults"]),
            "declared_capabilities": require_mapping(data["declared_capabilities"], "request.declared_capabilities"),
            "visual_asset_index": self._visual_asset_index(data.get("visual_asset_index", {})),
        }

    def _physics(self, value: Any) -> dict[str, str]:
        physics = require_mapping(value, "request.physics")
        strict_keys(
            physics,
            required={"attach_root", "root_joint"},
            optional={"initial_keyframe"},
            context="request.physics",
        )
        result = {
            "attach_root": require_string(physics["attach_root"], "request.physics.attach_root"),
            "root_joint": require_string(physics["root_joint"], "request.physics.root_joint"),
        }
        if "initial_keyframe" in physics:
            result["initial_keyframe"] = require_string(physics["initial_keyframe"], "request.physics.initial_keyframe")
        return result

    def _visual(self, value: Any) -> dict[str, str]:
        visual = require_mapping(value, "request.visual")
        strict_keys(visual, required={"binding"}, optional=set(), context="request.visual")
        return {"binding": require_string(visual["binding"], "request.visual.binding")}

    def _semantic(self, value: Any) -> dict[str, str]:
        semantic = require_mapping(value, "request.semantic")
        strict_keys(semantic, required={"class"}, optional=set(), context="request.semantic")
        return {"class": require_string(semantic["class"], "request.semantic.class")}

    def _frames(self, value: Any) -> list[dict[str, Any]]:
        if not isinstance(value, list) or not value:
            raise ImportFailure("request.frames must be a non-empty array", context="request.frames")
        frames: list[dict[str, Any]] = []
        for index, item in enumerate(value):
            frame = require_mapping(item, f"request.frames[{index}]")
            strict_keys(
                frame,
                required={"name", "role"},
                optional={"parent_frame", "extrinsic"},
                context=f"request.frames[{index}]",
            )
            normalized = {
                "name": require_string(frame["name"], f"request.frames[{index}].name"),
                "role": require_string(frame["role"], f"request.frames[{index}].role"),
            }
            if "parent_frame" in frame:
                normalized["parent_frame"] = require_string(
                    frame["parent_frame"],
                    f"request.frames[{index}].parent_frame",
                )
            if "extrinsic" in frame:
                normalized["extrinsic"] = frame["extrinsic"]
            frames.append(normalized)
        return frames

    def _interfaces(self, value: Any) -> dict[str, list[str]]:
        interfaces = require_mapping(value, "request.interfaces")
        strict_keys(interfaces, required={"state", "command"}, optional=set(), context="request.interfaces")
        result: dict[str, list[str]] = {}
        for key in ("state", "command"):
            raw = interfaces[key]
            if not isinstance(raw, list) or not raw:
                raise ImportFailure(
                    f"request.interfaces.{key} must be a non-empty array",
                    context=f"request.interfaces.{key}",
                )
            result[key] = [require_string(item, f"request.interfaces.{key}[]") for item in raw]
        return result

    def _defaults(self, value: Any) -> dict[str, str | None]:
        defaults = require_mapping(value, "request.defaults")
        strict_keys(defaults, required={"controller", "sensor_rig"}, optional=set(), context="request.defaults")
        return {
            "controller": (
                None
                if defaults["controller"] is None
                else require_string(defaults["controller"], "request.defaults.controller")
            ),
            "sensor_rig": (
                None
                if defaults["sensor_rig"] is None
                else require_string(defaults["sensor_rig"], "request.defaults.sensor_rig")
            ),
        }

    def _visual_asset_index(self, value: Any) -> dict[str, str]:
        index = require_mapping(value, "request.visual_asset_index")
        return {
            require_string(key, "request.visual_asset_index key"): require_string(
                item,
                f"request.visual_asset_index.{key}",
            )
            for key, item in sorted(index.items())
        }

    def _manifest(self, spec: Mapping[str, Any], relative_mjcf: str) -> dict[str, Any]:
        physics = {
            "mjcf": relative_mjcf,
            "attach_root": spec["physics"]["attach_root"],
            "root_joint": spec["physics"]["root_joint"],
            "global_options": "inherit_session",
        }
        if "initial_keyframe" in spec["physics"]:
            physics["initial_keyframe"] = spec["physics"]["initial_keyframe"]
        manifest: dict[str, Any] = {
            "schema": _PACKAGE_SCHEMA,
            "id": spec["id"],
            "version": spec["version"],
            "kind": "robot",
            "physics": physics,
            "visual": {"binding": spec["visual"]["binding"], "projection": "visual/robot.visual-projection.json"},
            "semantic": spec["semantic"],
            "frames": spec["frames"],
            "interfaces": spec["interfaces"],
            "defaults": spec["defaults"],
            "declared_capabilities": spec["declared_capabilities"],
        }
        if spec.get("description") is not None:
            manifest["description"] = spec["description"]
        compatibility = dict(spec.get("compatibility") or {})
        if compatibility:
            manifest["compatibility"] = compatibility
        return manifest

    def _validate_mjcf(
        self,
        mjcf: Path,
        *,
        source_root: Path,
        attach_root: str,
        root_joint: str,
        frames: Any,
        commandable: bool = False,
        compiled_model: Any = _COMPILE_SENTINEL,
    ) -> tuple[Any, dict[str, Any]]:
        root = ET.parse(mjcf).getroot()  # noqa: S314 - importer rejects includes and unsafe asset references.
        if root.tag != "mujoco":
            raise ImportFailure("robot source must be MuJoCo MJCF", code=ImportCode.MODEL_INVALID, context=str(mjcf))
        compiler = root.find("compiler")
        if compiler is None or compiler.attrib.get("angle") != "radian":
            raise ImportFailure(
                "MJCF compiler must declare angle='radian'",
                code=ImportCode.UNIT_AMBIGUOUS,
                context=str(mjcf),
            )
        if root.findall(".//include"):
            raise ImportFailure("MJCF include elements are not allowed in robot imports", code=ImportCode.UNSAFE_SOURCE)
        option = root.find("option")
        if option is not None:
            fields = sorted(option.attrib)
            if fields:
                raise ImportFailure(
                    "attachable RobotPackage MJCF must inherit session-owned MuJoCo options",
                    code=ImportCode.GLOBAL_PHYSICS_OWNERSHIP,
                    context="robot.mjcf.option",
                    details={"fields": fields, "required_policy": "inherit_session"},
                )
        asset_closure = self._validate_mjcf_references(root, mjcf, source_root)
        names = self._named_symbols(root)
        if attach_root not in names["body"]:
            raise ImportFailure(
                f"attach_root {attach_root!r} is absent from MJCF bodies",
                code=ImportCode.MODEL_INVALID,
            )
        if root_joint not in names["joint"]:
            raise ImportFailure(f"root_joint {root_joint!r} is absent from MJCF joints", code=ImportCode.MODEL_INVALID)
        body_owners, body_parents, joint_owners = self._xml_ownership(root)
        if joint_owners.get(root_joint) != attach_root:
            raise ImportFailure(
                f"root_joint {root_joint!r} must be owned by attach_root {attach_root!r}",
                code=ImportCode.MODEL_INVALID,
            )
        if body_parents.get(attach_root) is not None:
            raise ImportFailure(
                f"attach_root {attach_root!r} must be a direct worldbody child",
                code=ImportCode.MODEL_INVALID,
            )
        root_joint_element = next(element for element in root.iter("joint") if element.attrib.get("name") == root_joint)
        if root_joint_element.attrib.get("type", "hinge") != "free":
            raise ImportFailure(
                f"root_joint {root_joint!r} must be a free joint",
                code=ImportCode.MODEL_INVALID,
            )
        frame_names = {require_mapping(item, "frame")["name"] for item in frames} if isinstance(frames, list) else set()
        valid_frames = names["body"] | names["site"] | names["camera"]
        missing_frames = sorted(frame_names - valid_frames)
        if missing_frames:
            raise ImportFailure(
                "RobotPackage frames must name MJCF body/site/camera symbols",
                code=ImportCode.MODEL_INVALID,
                details={"missing": missing_frames},
            )
        actuator = root.find("actuator")
        joints = names["joint"]
        if actuator is not None:
            for child in actuator:
                if child.tag not in _ACTUATOR_TAGS:
                    continue
                if commandable and not child.attrib.get("name"):
                    raise ImportFailure(
                        f"commandable robot actuator {child.tag!r} must have a stable name",
                        code=ImportCode.MODEL_INVALID,
                    )
                if commandable and "joint" not in child.attrib:
                    raise ImportFailure(
                        f"commandable robot actuator {child.attrib.get('name', child.tag)!r} must name a joint",
                        code=ImportCode.MODEL_INVALID,
                    )
                if "joint" in child.attrib and child.attrib["joint"] not in joints:
                    raise ImportFailure(
                        f"actuator {child.attrib.get('name', child.tag)!r} references unknown joint "
                        f"{child.attrib['joint']!r}",
                        code=ImportCode.MODEL_INVALID,
                    )
                if "joint" in child.attrib:
                    owner = joint_owners.get(child.attrib["joint"])
                    if owner is None or not self._xml_body_is_descendant(owner, attach_root, body_parents):
                        raise ImportFailure(
                            f"actuator {child.attrib.get('name', child.tag)!r} targets a joint outside attach_root",
                            code=ImportCode.MODEL_INVALID,
                        )

        if compiled_model is _COMPILE_SENTINEL:
            model, compile_evidence = self._compile_mjcf(mjcf)
        else:
            model = compiled_model
            compile_evidence = self._compile_evidence(model, mjcf, backend="injected")
        compiled_evidence = self._validate_compiled_layout(
            model,
            mjcf,
            root,
            attach_root=attach_root,
            root_joint=root_joint,
            commandable=commandable,
            xml_names=names,
            xml_body_owners=body_owners,
            xml_body_parents=body_parents,
            xml_joint_owners=joint_owners,
        )
        return model, {**compile_evidence, "layout": compiled_evidence, "asset_closure": asset_closure}

    def _named_symbols(self, root: ET.Element) -> dict[str, set[str]]:
        result: dict[str, set[str]] = {tag: set() for tag in (*_NAMED_TAGS, "actuator")}
        for tag in _NAMED_TAGS:
            seen: set[str] = set()
            for element in root.iter(tag):
                name = element.attrib.get("name")
                if not name:
                    continue
                if name in seen:
                    raise ImportFailure(
                        f"MJCF contains duplicate {tag} name {name!r}",
                        code=ImportCode.MODEL_INVALID,
                        context=tag,
                    )
                seen.add(name)
            result[tag] = seen
        actuator_seen: set[str] = set()
        actuator = root.find("actuator")
        if actuator is not None:
            for element in actuator:
                if element.tag not in _ACTUATOR_TAGS:
                    continue
                name = element.attrib.get("name")
                if not name:
                    continue
                if name in actuator_seen:
                    raise ImportFailure(
                        f"MJCF contains duplicate actuator name {name!r}",
                        code=ImportCode.MODEL_INVALID,
                        context="actuator",
                    )
                actuator_seen.add(name)
        result["actuator"] = actuator_seen
        return result

    def _xml_ownership(
        self,
        root: ET.Element,
    ) -> tuple[dict[str, ET.Element], dict[str, str | None], dict[str, str]]:
        bodies: dict[str, ET.Element] = {}
        parents: dict[str, str | None] = {}
        joints: dict[str, str] = {}

        def visit(parent: ET.Element, parent_name: str | None) -> None:
            for body in parent:
                if body.tag != "body":
                    continue
                name = body.attrib.get("name")
                if name:
                    bodies[name] = body
                    parents[name] = parent_name
                    for joint in body.findall("joint"):
                        joint_name = joint.attrib.get("name")
                        if joint_name:
                            joints[joint_name] = name
                    visit(body, name)
                else:
                    visit(body, parent_name)

        worldbody = root.find("worldbody")
        if worldbody is not None:
            visit(worldbody, None)
        return bodies, parents, joints

    @staticmethod
    def _xml_body_is_descendant(
        body_name: str,
        ancestor_name: str,
        parents: Mapping[str, str | None],
    ) -> bool:
        current: str | None = body_name
        while current is not None:
            if current == ancestor_name:
                return True
            current = parents.get(current)
        return False

    def _compile_mjcf(self, mjcf: Path) -> tuple[Any, dict[str, Any]]:
        if self.mujoco_compiler is not None:
            compiler_identity = self._converter_identity(self.mujoco_compiler)
            try:
                model = self.mujoco_compiler(mjcf)
            except (OSError, ValueError) as exc:
                raise ImportFailure(
                    f"MuJoCo compile failed: {exc}",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=str(mjcf),
                ) from exc
            if model is None:
                raise ImportFailure(
                    "MuJoCo compile gate returned no compiled model",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=str(mjcf),
                )
            return model, self._compile_evidence(
                model,
                mjcf,
                backend="injected",
                compiler_identity=compiler_identity,
            )
        try:
            import mujoco  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportFailure(
                "MuJoCo is unavailable; robot qualification cannot pass",
                code=ImportCode.QUALIFICATION_FAILED,
                context="mujoco",
            ) from exc
        compiler = getattr(getattr(mujoco, "MjModel", None), "from_xml_path", None)
        if compiler is None:
            raise ImportFailure(
                "MuJoCo MjModel.from_xml_path is unavailable; robot qualification cannot pass",
                code=ImportCode.QUALIFICATION_FAILED,
                context="mujoco.MjModel.from_xml_path",
            )
        try:
            model = compiler(str(mjcf))
        except (OSError, ValueError) as exc:
            raise ImportFailure(
                f"MuJoCo compile failed: {exc}",
                code=ImportCode.QUALIFICATION_FAILED,
                context=str(mjcf),
            ) from exc
        if model is None:
            raise ImportFailure(
                "MuJoCo compile gate returned no compiled model",
                code=ImportCode.QUALIFICATION_FAILED,
                context=str(mjcf),
            )
        return model, self._compile_evidence(
            model,
            mjcf,
            backend="mujoco",
            compiler_identity={
                "module": "mujoco",
                "version": str(getattr(mujoco, "__version__", "unknown")),
            },
        )

    def _compile_evidence(
        self,
        model: Any,
        mjcf: Path,
        *,
        backend: str,
        compiler_identity: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        stats: dict[str, int] = {}
        for name in ("nbody", "njnt", "nsite", "ncam", "nu", "nq", "nv"):
            value = getattr(model, name, None)
            if isinstance(value, int):
                stats[name] = value
            elif value is not None:
                try:
                    stats[name] = int(value)
                except (TypeError, ValueError):
                    continue
        return {
            "schema": "lingtu.sim.mujoco-compile-evidence.v1",
            "backend": backend,
            "compiler": dict(compiler_identity or {"class": f"{type(model).__module__}.{type(model).__qualname__}"}),
            "mjcf": {"path": mjcf.name},
            "model": stats,
        }

    def _validate_compiled_layout(
        self,
        model: Any,
        mjcf: Path,
        root: ET.Element,
        *,
        attach_root: str,
        root_joint: str,
        commandable: bool,
        xml_names: Mapping[str, set[str]],
        xml_body_owners: Mapping[str, ET.Element],
        xml_body_parents: Mapping[str, str | None],
        xml_joint_owners: Mapping[str, str],
    ) -> dict[str, Any]:
        compiled_bodies = self._compiled_symbol_names(model, "body")
        compiled_joints = self._compiled_symbol_names(model, "joint")
        compiled_actuators = self._compiled_symbol_names(model, "actuator")
        body_names = {name for name in compiled_bodies if name is not None} or set(xml_names["body"])
        joint_names = {name for name in compiled_joints if name is not None} or set(xml_names["joint"])
        if attach_root not in body_names:
            raise ImportFailure(
                f"compiled MuJoCo model is missing attach_root {attach_root!r}",
                code=ImportCode.MODEL_INVALID,
            )
        if root_joint not in joint_names:
            raise ImportFailure(
                f"compiled MuJoCo model is missing root_joint {root_joint!r}",
                code=ImportCode.MODEL_INVALID,
            )

        body_id = self._compiled_symbol_id(model, "body", attach_root, compiled_bodies)
        joint_id = self._compiled_symbol_id(model, "joint", root_joint, compiled_joints)
        parent_id = self._model_array_int(model, "body_parentid", body_id)
        joint_body_id = self._model_array_int(model, "jnt_bodyid", joint_id)
        if parent_id is not None and parent_id != 0:
            raise ImportFailure(
                f"compiled attach_root {attach_root!r} is not a direct worldbody child",
                code=ImportCode.MODEL_INVALID,
            )
        if body_id is not None and joint_body_id is not None and body_id != joint_body_id:
            raise ImportFailure(
                f"compiled root_joint {root_joint!r} is not owned by attach_root {attach_root!r}",
                code=ImportCode.MODEL_INVALID,
            )
        joint_type = self._model_array_int(model, "jnt_type", joint_id)
        if joint_type is not None and joint_type != _FREE_JOINT:
            raise ImportFailure(
                f"compiled root_joint {root_joint!r} is not a free joint",
                code=ImportCode.MODEL_INVALID,
            )

        layout: dict[str, Any] = {
            "attach_root": {
                "name": attach_root,
                "id": body_id,
                "parent_id": parent_id,
            },
            "root_joint": {
                "name": root_joint,
                "id": joint_id,
                "body_id": joint_body_id,
                "type": joint_type,
            },
            "actuators": [],
        }
        actuator_elements = [element for element in root.findall("./actuator/*") if element.tag in _ACTUATOR_TAGS]
        if commandable and actuator_elements and compiled_actuators:
            if any(name is None for name in compiled_actuators):
                raise ImportFailure(
                    "compiled commandable robot contains an unnamed actuator",
                    code=ImportCode.MODEL_INVALID,
                )
            compiled_actuator_names = [name for name in compiled_actuators if name is not None]
            for element in actuator_elements:
                name = element.attrib.get("name")
                if name is None or name not in compiled_actuator_names:
                    raise ImportFailure(
                        f"compiled MuJoCo model does not preserve actuator name {name or element.tag!r}",
                        code=ImportCode.MODEL_INVALID,
                    )
                actuator_id = compiled_actuator_names.index(name)
                target_joint = element.attrib.get("joint")
                target_joint_id = (
                    self._compiled_symbol_id(model, "joint", target_joint, compiled_joints) if target_joint else None
                )
                compiled_target_joint_id = self._model_array_int(model, "actuator_trnid", actuator_id, column=0)
                transmission_type = self._model_array_int(model, "actuator_trntype", actuator_id)
                if target_joint is not None and target_joint_id is not None and compiled_target_joint_id is not None:
                    if target_joint_id != compiled_target_joint_id:
                        raise ImportFailure(
                            f"compiled actuator {name!r} targets a different joint than MJCF",
                            code=ImportCode.MODEL_INVALID,
                        )
                if transmission_type is not None and transmission_type != _JOINT_TRANSMISSION:
                    raise ImportFailure(
                        f"commandable actuator {name!r} is not joint-backed",
                        code=ImportCode.MODEL_INVALID,
                    )
                target_body_id = self._model_array_int(model, "jnt_bodyid", compiled_target_joint_id)
                if body_id is not None and target_body_id is not None:
                    if not self._compiled_body_is_descendant(model, target_body_id, body_id):
                        raise ImportFailure(
                            f"compiled actuator {name!r} targets a joint outside attach_root",
                            code=ImportCode.MODEL_INVALID,
                        )
                layout["actuators"].append(
                    {
                        "name": name,
                        "id": actuator_id,
                        "joint": target_joint,
                        "joint_id": compiled_target_joint_id,
                        "transmission_type": transmission_type,
                    }
                )
        return layout

    def _compiled_symbol_names(self, model: Any, kind: str) -> list[str | None]:
        for attribute in (f"{kind}_names", f"{kind}Names"):
            values = getattr(model, attribute, None)
            if isinstance(values, Sequence) and not isinstance(values, (str, bytes)):
                return [None if value is None else str(value) for value in values]
        count = getattr(model, {"body": "nbody", "joint": "njnt", "actuator": "nu"}.get(kind, ""), None)
        if count is None:
            return []
        try:
            count = int(count)
        except (TypeError, ValueError):
            return []
        try:
            import mujoco  # type: ignore[import-not-found]

            object_type = {
                "body": mujoco.mjtObj.mjOBJ_BODY,
                "joint": mujoco.mjtObj.mjOBJ_JOINT,
                "actuator": mujoco.mjtObj.mjOBJ_ACTUATOR,
            }[kind]
            return [mujoco.mj_id2name(model, object_type, index) for index in range(count)]
        except (ImportError, KeyError, TypeError, ValueError):
            return []

    def _compiled_symbol_id(
        self,
        model: Any,
        kind: str,
        name: str | None,
        names: Sequence[str | None],
    ) -> int | None:
        if name is None:
            return None
        for index, item in enumerate(names):
            if item == name:
                return index
        lookup = getattr(model, kind, None)
        if callable(lookup):
            try:
                value = lookup(name)
                result = getattr(value, "id", value)
                return int(result)
            except (KeyError, TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _model_array_int(model: Any, attribute: str, index: int | None, *, column: int | None = None) -> int | None:
        if index is None:
            return None
        values = getattr(model, attribute, None)
        if values is None:
            return None
        try:
            value = values[index] if column is None else values[index][column]
            return int(value)
        except (IndexError, KeyError, TypeError, ValueError):
            return None

    def _compiled_body_is_descendant(self, model: Any, body_id: int, ancestor_id: int) -> bool:
        current: int | None = body_id
        while current is not None:
            if current == ancestor_id:
                return True
            current = self._model_array_int(model, "body_parentid", current)
            if current == 0 and ancestor_id != 0:
                return False
        return False

    def _validate_mjcf_references(
        self,
        root: ET.Element,
        mjcf: Path,
        source_root: Path,
    ) -> list[dict[str, str]]:
        """Resolve MuJoCo asset paths exactly and bind the files read by compilation."""

        compiler = root.find("compiler")
        attributes = compiler.attrib if compiler is not None else {}
        assetdir = attributes.get("assetdir", ".")
        meshdir = attributes.get("meshdir", assetdir)
        texturedir = attributes.get("texturedir", assetdir)
        strippath_value = attributes.get("strippath", "false")
        if strippath_value not in {"true", "false"}:
            raise ImportFailure(
                "MJCF compiler.strippath must be 'true' or 'false'",
                code=ImportCode.MODEL_INVALID,
                context="compiler.strippath",
            )
        strippath = strippath_value == "true"
        closure: set[str] = set()
        for element in root.iter():
            if element.tag == "texture":
                supported_file_attributes = set(_TEXTURE_FILE_ATTRS)
                references = tuple((name, element.attrib.get(name), texturedir) for name in _TEXTURE_FILE_ATTRS)
            elif element.tag == "mesh":
                supported_file_attributes = {"file"}
                references = (("file", element.attrib.get("file"), meshdir),)
            elif element.tag in {"hfield", "skin"}:
                supported_file_attributes = {"file"}
                references = (("file", element.attrib.get("file"), meshdir),)
            else:
                supported_file_attributes = set()
                references = ()
            file_attributes = {
                name for name, value in element.attrib.items() if value and (name == "file" or name.startswith("file"))
            }
            unsupported = sorted(file_attributes - supported_file_attributes)
            if unsupported:
                raise ImportFailure(
                    f"MJCF {element.tag} has unsupported file-bearing attributes",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=f"MJCF {element.tag}",
                    details={"attributes": unsupported},
                )
            for attribute, raw_reference, directory in references:
                if not raw_reference:
                    continue
                raw_path = PurePosixPath(raw_reference)
                if (
                    "://" in raw_reference
                    or "\\" in raw_reference
                    or raw_path.is_absolute()
                    or (raw_path.parts and ":" in raw_path.parts[0])
                ):
                    raise ImportFailure(
                        f"MJCF {element.tag}.{attribute} uses an unsafe path: {raw_reference!r}",
                        code=ImportCode.UNSAFE_SOURCE,
                        context=raw_reference,
                    )
                reference = PurePosixPath(raw_reference).name if strippath else raw_reference
                target = self._resolve_mjcf_asset(
                    mjcf,
                    source_root,
                    directory,
                    reference,
                    context=f"MJCF {element.tag}.{attribute}",
                )
                relative = target.relative_to(Path(source_root).resolve()).as_posix()
                closure.add(relative)
        return [{"path": path} for path in sorted(closure)]

    @staticmethod
    def _resolve_mjcf_asset(
        mjcf: Path,
        source_root: Path,
        directory: str,
        reference: str,
        *,
        context: str,
    ) -> Path:
        for value in (directory, reference):
            path = PurePosixPath(value)
            if (
                not value
                or "://" in value
                or "\\" in value
                or path.is_absolute()
                or (path.parts and ":" in path.parts[0])
            ):
                raise ImportFailure(
                    f"{context} uses an unsafe path: {value!r}",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=value,
                )
        owned_root = Path(source_root).resolve()
        target = (mjcf.parent / directory / reference).resolve()
        try:
            target.relative_to(owned_root)
        except ValueError as exc:
            raise ImportFailure(
                f"{context} escapes source root: {reference!r}",
                code=ImportCode.UNSAFE_SOURCE,
                context=reference,
            ) from exc
        assert_no_reparse_components(target, below=owned_root, context=context)
        if not target.is_file():
            raise ImportFailure(
                f"{context} is missing: {reference!r}",
                code=ImportCode.ASSET_MISSING,
                context=reference,
            )
        return target

    def _asset_bindings(
        self,
        visual_manifest: Mapping[str, Any],
        spec: Mapping[str, Any],
        package_root: Path,
    ) -> tuple[dict[str, str], list[dict[str, Any]]]:
        bindings = dict(spec["visual_asset_index"])
        visuals = [item for item in visual_manifest.get("visuals", []) if isinstance(item, Mapping)]
        mesh_visuals = [
            item for item in visuals if require_mapping(item.get("geometry"), "visual.geometry").get("kind") == "mesh"
        ]
        has_mesh = bool(mesh_visuals)
        if not has_mesh:
            return bindings, []
        if bindings:
            return bindings, []
        if self.mesh_converter is None:
            raise ImportFailure(
                "mesh robot visuals require visual_asset_index or an injected mesh projection converter",
                code=ImportCode.ASSET_MISSING,
                context="visual_asset_index",
            )
        result = self.mesh_converter.convert(visual_manifest)
        converted_bindings, raw_evidence = self._mesh_converter_result(result)
        if not converted_bindings:
            raise ImportFailure(
                "mesh projection converter returned no asset bindings",
                code=ImportCode.PROJECTION_INVALID,
                context="mesh_converter",
            )
        evidence = self._normalize_mesh_evidence(
            raw_evidence,
            visual_manifest=visual_manifest,
            bindings=converted_bindings,
            package_root=package_root,
        )
        return converted_bindings, evidence

    def _mesh_converter_result(self, result: Any) -> tuple[dict[str, str], Any]:
        converter_evidence = getattr(self.mesh_converter, "evidence", None)
        raw_bindings: Any
        if isinstance(result, Mapping):
            if "bindings" in result:
                raw_bindings = require_mapping(result["bindings"], "mesh_converter.bindings")
                raw_evidence = result.get("evidence", result.get("conversion_evidence", converter_evidence))
            elif "asset_bindings" in result:
                raw_bindings = require_mapping(result["asset_bindings"], "mesh_converter.asset_bindings")
                raw_evidence = result.get("evidence", result.get("conversion_evidence", converter_evidence))
            else:
                raw_bindings = {
                    key: value for key, value in result.items() if key not in {"evidence", "conversion_evidence"}
                }
                raw_evidence = result.get("evidence", result.get("conversion_evidence", converter_evidence))
        else:
            raw_bindings = getattr(result, "bindings", None)
            raw_evidence = getattr(result, "evidence", converter_evidence)
            if not isinstance(raw_bindings, Mapping):
                raise ImportFailure(
                    "mesh projection converter must return bindings and structured evidence",
                    code=ImportCode.PROJECTION_INVALID,
                    context="mesh_converter",
                )
        bindings: dict[str, str] = {}
        for key, value in require_mapping(raw_bindings, "mesh_converter.bindings").items():
            if not isinstance(key, str) or not isinstance(value, str):
                raise ImportFailure(
                    "mesh projection bindings must map stable IDs to cooked asset strings",
                    code=ImportCode.PROJECTION_INVALID,
                    context="mesh_converter.bindings",
                )
            bindings[key] = value
        return bindings, raw_evidence

    def _normalize_mesh_evidence(
        self,
        raw_evidence: Any,
        *,
        visual_manifest: Mapping[str, Any],
        bindings: Mapping[str, str],
        package_root: Path,
    ) -> list[dict[str, Any]]:
        if isinstance(raw_evidence, Mapping):
            entries = [raw_evidence]
        elif isinstance(raw_evidence, Sequence) and not isinstance(raw_evidence, (str, bytes)):
            entries = list(raw_evidence)
        else:
            raise ImportFailure(
                "mesh conversion requires structured evidence; a string mapping is insufficient",
                code=ImportCode.PROJECTION_INVALID,
                context="mesh_converter.evidence",
            )
        known_sources: set[str] = set()
        for raw_visual in visual_manifest.get("visuals", []):
            if not isinstance(raw_visual, Mapping):
                continue
            geometry = require_mapping(raw_visual.get("geometry"), "visual.geometry")
            if geometry.get("kind") == "mesh":
                source_mesh = require_string(
                    geometry.get("source_mesh"),
                    "visual.geometry.source_mesh",
                )
                known_sources.add(self._stable_evidence_path(source_mesh, package_root))
        normalized: list[dict[str, Any]] = []
        for index, raw in enumerate(entries):
            entry = require_mapping(raw, f"mesh_converter.evidence[{index}]")
            strict_keys(
                entry,
                required={"source", "output", "toolchain_version", "cooked_asset"},
                optional={"log_path"},
                context=f"mesh_converter.evidence[{index}]",
            )
            source = require_mapping(entry["source"], f"mesh_converter.evidence[{index}].source")
            output = require_mapping(entry["output"], f"mesh_converter.evidence[{index}].output")
            strict_keys(source, required={"path"}, optional=set(), context=f"mesh_converter.evidence[{index}].source")
            strict_keys(output, required={"path"}, optional=set(), context=f"mesh_converter.evidence[{index}].output")
            source_path = require_string(source.get("path"), f"mesh_converter.evidence[{index}].source.path")
            output_path = require_string(output.get("path"), f"mesh_converter.evidence[{index}].output.path")
            toolchain_version = require_string(
                entry.get("toolchain_version"),
                f"mesh_converter.evidence[{index}].toolchain_version",
            )
            cooked_asset = require_string(
                entry.get("cooked_asset"),
                f"mesh_converter.evidence[{index}].cooked_asset",
            )
            if (
                not cooked_asset.startswith("/Game/")
                or "\\" in cooked_asset
                or any(part in {"", ".", ".."} for part in cooked_asset.split("/")[2:])
            ):
                raise ImportFailure(
                    "mesh conversion cooked_asset must be a safe /Game/ path",
                    code=ImportCode.PROJECTION_INVALID,
                    context=cooked_asset,
                )
            stable_source = self._stable_evidence_path(source_path, package_root)
            if stable_source not in known_sources:
                raise ImportFailure(
                    "mesh conversion source does not match an MJCF mesh source",
                    code=ImportCode.PROJECTION_INVALID,
                    context=source_path,
                )
            source_file = self._evidence_file_path(source_path, package_root)
            if source_file is None or not source_file.is_file():
                raise ImportFailure(
                    "mesh conversion source path must identify a readable artifact",
                    code=ImportCode.ASSET_MISSING,
                    context=source_path,
                )
            output_file = self._evidence_file_path(output_path, package_root)
            if output_file is None or not output_file.is_file():
                raise ImportFailure(
                    "mesh conversion output path must identify a readable artifact",
                    code=ImportCode.ASSET_MISSING,
                    context=output_path,
                )
            log_path = entry.get("log_path")
            if log_path is not None:
                log_file = self._evidence_file_path(
                    require_string(log_path, "mesh_converter.evidence.log_path"),
                    package_root,
                )
                if log_file is None or not log_file.is_file():
                    raise ImportFailure(
                        "mesh conversion log path must identify a readable file",
                        code=ImportCode.ASSET_MISSING,
                        context=str(log_path),
                    )
            normalized.append(
                {
                    "schema": "lingtu.sim.mesh-conversion-evidence.v1",
                    "source": {"path": stable_source},
                    "output": {"path": self._stable_evidence_path(output_path, package_root)},
                    "toolchain_version": toolchain_version,
                    "cooked_asset": cooked_asset,
                    **(
                        {"log_path": self._stable_evidence_path(str(log_path), package_root)}
                        if log_path is not None
                        else {}
                    ),
                }
            )
        mesh_assets = {
            value
            for key, value in bindings.items()
            if any(
                key in {item.get("asset_key"), item.get("geometry", {}).get("mesh")}
                for item in visual_manifest.get("visuals", [])
                if isinstance(item, Mapping)
            )
        }
        evidenced_assets = {item["cooked_asset"] for item in normalized}
        if not mesh_assets.issubset(evidenced_assets):
            raise ImportFailure(
                "mesh conversion evidence must cover every cooked mesh asset",
                code=ImportCode.PROJECTION_INVALID,
                details={"missing": sorted(mesh_assets - evidenced_assets)},
            )
        return normalized

    @staticmethod
    def _evidence_file_path(value: str, package_root: Path) -> Path | None:
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = package_root / PurePosixPath(value)
        try:
            return candidate.resolve()
        except OSError:
            return None

    @staticmethod
    def _stable_evidence_path(value: str, package_root: Path) -> str:
        candidate = Path(value)
        if candidate.is_absolute():
            try:
                return candidate.resolve().relative_to(package_root.resolve()).as_posix()
            except ValueError:
                return str(candidate.resolve())
        return PurePosixPath(value).as_posix()

    def _write_qualification(
        self,
        root: Path,
        spec: Mapping[str, Any],
        manifest_path: Path,
        projection_path: Path,
        source: Any,
        validation: Mapping[str, Any],
        provenance: Mapping[str, Any],
        provenance_path: Path,
        compile_evidence: Mapping[str, Any],
        conversion_evidence: Mapping[str, Any],
        identity: Mapping[str, Any],
    ) -> Path:
        qualification_root = root / "qualification" / "robot" / spec["id"]
        evidence_root = qualification_root / "evidence" / spec["version"]
        provenance_relative = provenance_path.relative_to(manifest_path.parent).as_posix()
        evidence_files = {
            "source-intake.json": source.to_dict(),
            "catalog-validation.json": validation,
            "provenance.json": provenance,
            "mujoco-compile.json": compile_evidence,
            "conversion.json": conversion_evidence,
            "import-identity.json": identity,
            "visual-projection.json": json.loads(projection_path.read_text(encoding="utf-8")),
        }
        evidence: list[dict[str, str]] = []
        for name, document in sorted(evidence_files.items()):
            write_json(evidence_root / name, document)
            evidence.append(
                {
                    "path": f"evidence/{spec['version']}/{name}",
                }
            )
        evidence_by_name = {Path(item["path"]).name: item for item in evidence}

        def evidence_for(*names: str) -> list[dict[str, str]]:
            return [evidence_by_name[name] for name in names]

        record = {
            "schema": "lingtu.sim.qualification-record.v1",
            "package": {
                "kind": "robot",
                "id": spec["id"],
                "version": spec["version"],
            },
            "qualified_capabilities": {},
            "provenance": {"path": provenance_relative},
            "checks": [
                {"id": "source_intake", "status": "passed", "evidence": evidence_for("source-intake.json")},
                {
                    "id": "provenance",
                    "status": "passed",
                    "evidence": evidence_for("provenance.json"),
                },
                {
                    "id": "mujoco_compile",
                    "status": "passed",
                    "evidence": evidence_for("mujoco-compile.json", "conversion.json"),
                },
                {
                    "id": "catalog_resolver",
                    "status": "passed",
                    "evidence": evidence_for(
                        "catalog-validation.json",
                        "import-identity.json",
                    ),
                },
                {
                    "id": "visual_projection",
                    "status": "passed",
                    "evidence": evidence_for("visual-projection.json"),
                },
            ],
        }
        qualification_result: Path = write_json(
            qualification_root / f"{spec['version']}.qualification.json",
            record,
        )
        CatalogPromoter(manifest_path.parent)._validate_materialized(
            kind="robot",
            package_id=spec["id"],
            version=spec["version"],
            package_root=manifest_path.parent,
            manifest_path=manifest_path,
            provenance_path=provenance_path,
            qualification_path=qualification_result,
            evidence_root=evidence_root,
        )
        return qualification_result

    def _new_staging_root(self) -> Path:
        staging = self.work_root / "robot" / ".staging" / uuid.uuid4().hex
        staging.mkdir(parents=True, exist_ok=False)
        return staging

    def _remove_staging_root(self, root: Path) -> None:
        root = Path(root).resolve()
        staging_root = (self.work_root / "robot" / ".staging").resolve()
        try:
            root.relative_to(staging_root)
        except ValueError:
            return
        if root.exists():
            shutil.rmtree(root)

    def _published_root(self, package_id: str, version: str, import_id: str) -> Path:
        root = (self.work_root / "robot" / package_id / version / import_id).resolve()
        try:
            root.relative_to(self.work_root)
        except ValueError as exc:
            raise ImportFailure("published import root escapes work_root", code=ImportCode.UNSAFE_SOURCE) from exc
        return root

    def _publish_draft(self, draft: ImportDraft, staging_root: Path) -> ImportDraft:
        published_root = self._published_root(draft.package_id, draft.version, draft.import_id)
        published_draft = self._relocate_draft(draft, staging_root, published_root)
        write_json(staging_root / "import-draft.json", published_draft.to_dict())
        lock_path = self.work_root / "robot" / ".locks" / draft.package_id / draft.version / f"{draft.import_id}.lock"
        with _PackageLock(lock_path):
            if published_root.exists():
                try:
                    self._validate_published_draft(published_draft)
                except (CatalogError, ImportFailure, OSError, ValueError) as exc:
                    self._remove_staging_root(staging_root)
                    raise ImportFailure(
                        f"published robot import identity is stale or invalid: {draft.import_id}",
                        code=ImportCode.PROMOTION_CONFLICT,
                        context=draft.import_id,
                        details={"reason": str(exc)},
                    ) from exc
                self._remove_staging_root(staging_root)
                return published_draft

            published_root.parent.mkdir(parents=True, exist_ok=True)
            try:
                CatalogPromoter(self.work_root)._publish_step("robot import draft", staging_root, published_root)
            except ImportFailure:
                self._remove_staging_root(staging_root)
                raise
            return published_draft

    @staticmethod
    def _validate_published_draft(draft: ImportDraft) -> None:
        package_root = draft.package_root
        manifest_path = draft.manifest_path
        provenance_path = draft.provenance_path
        qualification_path = draft.qualification_path
        if (
            package_root is None
            or manifest_path is None
            or provenance_path is None
            or qualification_path is None
        ):
            raise ImportFailure(
                "published robot draft is missing required artifacts",
                code=ImportCode.PROMOTION_CONFLICT,
                context=draft.import_id,
            )
        evidence_root = qualification_path.parent / "evidence" / draft.version
        CatalogPromoter(draft.root)._validate_materialized(
            kind="robot",
            package_id=draft.package_id,
            version=draft.version,
            package_root=package_root,
            manifest_path=manifest_path,
            provenance_path=provenance_path,
            qualification_path=qualification_path,
            evidence_root=evidence_root,
        )

    @staticmethod
    def _relocate_draft(draft: ImportDraft, staging_root: Path, published_root: Path) -> ImportDraft:
        def relocate(value: Path | None) -> Path | None:
            if value is None:
                return None
            path = Path(value).resolve()
            relative = path.relative_to(Path(staging_root).resolve())
            return published_root / relative

        return ImportDraft(
            import_id=draft.import_id,
            kind=draft.kind,
            package_id=draft.package_id,
            version=draft.version,
            state=draft.state,
            root=published_root,
            package_root=relocate(draft.package_root),
            manifest_path=relocate(draft.manifest_path),
            provenance_path=relocate(draft.provenance_path),
            qualification_path=relocate(draft.qualification_path),
            diagnostics=draft.diagnostics,
        )

    def _quarantine(
        self,
        import_id: str,
        package_id: str,
        version: str,
        diagnostic: ImportDiagnostic,
    ) -> ImportDraft:
        safe_package = package_id if package_id != "unknown" else "unknown"
        safe_version = version if version != "unknown" else "unknown"
        root = (
            self.work_root / "robot" / "quarantine" / f"{safe_package}-{safe_version}-{import_id}-{uuid.uuid4().hex}"
        ).resolve()
        root.parent.mkdir(parents=True, exist_ok=True)
        root.mkdir(parents=True, exist_ok=True)
        quarantine = root / "quarantine"
        quarantine.mkdir(parents=True, exist_ok=True)
        write_json(quarantine / "diagnostics.json", {"diagnostics": [diagnostic.to_dict()]})
        draft = ImportDraft(
            import_id=import_id,
            kind="robot",
            package_id=package_id,
            version=version,
            state="quarantined",
            root=root,
            diagnostics=(diagnostic,),
        )
        write_json(root / "import-draft.json", draft.to_dict())
        return draft

    @staticmethod
    def _converter_identity(converter: Any) -> dict[str, Any]:
        if converter is None:
            return {"kind": "none"}
        identity = getattr(converter, "identity", None)
        if callable(identity):
            try:
                identity = identity()
            except TypeError:
                identity = None
        if isinstance(identity, Mapping):
            result: dict[str, Any] = dict(identity)
        elif identity is not None:
            result = {"id": require_string(identity, "converter.identity")}
        else:
            result = {"class": f"{type(converter).__module__}.{type(converter).__qualname__}"}
        for attribute in ("name", "version", "toolchain_version"):
            value = getattr(converter, attribute, None)
            if value is not None and attribute not in result:
                result[attribute] = require_string(value, f"converter.{attribute}")
        return result

    def _resolve_manifest_mjcf(self, package_root: Path, manifest_path: Path, value: str) -> Path:
        path = PurePosixPath(value)
        if (
            not value
            or "\\" in value
            or path.is_absolute()
            or not path.parts
            or ":" in path.parts[0]
            or any(part in {"", "."} for part in path.parts)
        ):
            raise ImportFailure(
                "robot.package.physics.mjcf must be a safe package-contained POSIX path",
                code=ImportCode.UNSAFE_SOURCE,
                context=value,
            )
        candidate = (manifest_path.parent / value).resolve()
        if self._is_beneath(candidate, package_root):
            return candidate
        trusted_asset_root = self.repo_root / "sim" / "robots"
        trusted_package_root = self.repo_root / "sim" / "packages" / "robots"
        if self._is_beneath(package_root, trusted_package_root) and self._is_beneath(candidate, trusted_asset_root):
            return candidate
        raise ImportFailure(
            "robot.package.physics.mjcf escapes the package-contained model root",
            code=ImportCode.UNSAFE_SOURCE,
            context=value,
        )

    def _single_manifest(self, package_root: Path) -> Path:
        candidates = sorted(package_root.glob("*.package.yaml")) + sorted(package_root.glob("*.package.yml"))
        if len(candidates) != 1:
            raise ImportFailure(
                "RobotPackage root must contain exactly one package manifest",
                code=ImportCode.MODEL_INVALID,
                context=str(package_root),
                details={"count": len(candidates)},
            )
        return candidates[0]

    @staticmethod
    def _is_beneath(path: Path, root: Path) -> bool:
        try:
            Path(path).resolve().relative_to(Path(root).resolve())
            return True
        except ValueError:
            return False


def validate_robot_package(package_root: Path, *, repo_root: Path | None = None) -> dict[str, Any]:
    """Validate a RobotPackage through ``RobotImporter`` without importing it."""

    root = Path(repo_root).resolve() if repo_root is not None else Path(package_root).resolve()
    return RobotImporter(root).validate_package(package_root)


__all__ = [
    "RobotImporter",
    "RobotMeshProjectionConverter",
    "UrdfToMjcfConverter",
    "validate_robot_package",
]
