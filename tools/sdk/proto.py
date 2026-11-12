"""Compile the public protocol and compare it with the gateway wire contract."""

from __future__ import annotations

import argparse
import ast
import difflib
import subprocess
import tempfile
from pathlib import Path

from google.protobuf import descriptor_pb2, text_format

ROOT = Path(__file__).resolve().parents[2]
PROTO = ROOT / "sdk" / "proto"
GENERATED = ROOT / "src" / "gateway" / "rpc" / "generated"


def gateway_descriptors() -> dict[str, descriptor_pb2.FileDescriptorProto]:
    """Read serialized descriptors without importing the Gateway or its dependencies."""
    descriptors = {}
    for path in sorted(GENERATED.rglob("*_pb2.py")):
        module = ast.parse(path.read_text(encoding="utf-8"))
        serialized = next(
            node.args[0].value
            for node in ast.walk(module)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "AddSerializedFile"
        )
        descriptor = descriptor_pb2.FileDescriptorProto.FromString(serialized)
        descriptors[descriptor.name] = descriptor
    return descriptors


def compile_descriptor(output: Path, *, protoc: str = "protoc") -> descriptor_pb2.FileDescriptorSet:
    sources = sorted(path.relative_to(PROTO).as_posix() for path in PROTO.rglob("*.proto"))
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [protoc, "-I", str(PROTO), "--include_imports", f"--descriptor_set_out={output}", *sources],
        cwd=PROTO,
        check=True,
    )
    return descriptor_pb2.FileDescriptorSet.FromString(output.read_bytes())


def canonical_descriptor(descriptor: descriptor_pb2.FileDescriptorProto) -> descriptor_pb2.FileDescriptorProto:
    canonical = descriptor_pb2.FileDescriptorProto()
    canonical.CopyFrom(descriptor)
    canonical.ClearField("source_code_info")
    # protoc emits derived JSON names that Python's serialized runtime descriptor omits.
    for message in canonical.message_type:
        for field in message.field:
            first, *parts = field.name.split("_")
            default_name = first + "".join(part[:1].upper() + part[1:] for part in parts)
            if field.json_name == default_name:
                field.ClearField("json_name")
    return canonical


def check_descriptor(compiled: descriptor_pb2.FileDescriptorSet) -> None:
    expected = gateway_descriptors()
    actual = {file.name: file for file in compiled.file if file.package == "doso.v1"}
    if actual.keys() != expected.keys():
        raise ValueError(f"Protocol files differ. Missing {expected.keys() - actual.keys()}, extra {actual.keys() - expected.keys()}")
    for name, original in expected.items():
        before = canonical_descriptor(original)
        after = canonical_descriptor(actual[name])
        if before != after:
            difference = "".join(
                difflib.unified_diff(
                    text_format.MessageToString(before).splitlines(keepends=True),
                    text_format.MessageToString(after).splitlines(keepends=True),
                    fromfile=f"gateway/{name}",
                    tofile=f"sdk/{name}",
                )
            )
            raise ValueError(f"Public protocol differs from gateway\n{difference}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Compare every public descriptor with the gateway contract")
    parser.add_argument("--descriptor-out", type=Path, help="Write a descriptor set including Google imports")
    parser.add_argument("--protoc", default="protoc", help="Protocol compiler executable")
    args = parser.parse_args()
    if not args.check and args.descriptor_out is None:
        parser.error("Specify --check or --descriptor-out")
    with tempfile.TemporaryDirectory(prefix="lingtu-proto-") as temporary:
        descriptor = compile_descriptor(Path(temporary) / "public.protoset", protoc=args.protoc)
        if args.check:
            check_descriptor(descriptor)
            print(f"Matched {len(gateway_descriptors())} public protocol files with the gateway")
        if args.descriptor_out:
            args.descriptor_out.parent.mkdir(parents=True, exist_ok=True)
            args.descriptor_out.write_bytes(descriptor.SerializeToString())
            print(f"Wrote {len(descriptor.file)} descriptors to {args.descriptor_out}")


if __name__ == "__main__":
    main()
