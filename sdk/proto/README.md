# Public gRPC protocol

The 19 `.proto` files in this directory define the Gateway's public `doso.v1`
wire protocol. They contain 73 messages, 13 enums, seven services, and 73 RPCs.
The `doso.v1` package and existing file names are part of the published contract.

These sources were restored from the serialized descriptors in
`src/gateway/rpc/generated` on 2026-11-10. The checked-in Gateway bindings provide
the compatibility reference. Source comments and original formatting were absent
from those descriptors. The restored sources preserve field names, numbers,
types, presence, oneofs, enum values, reserved ranges, public imports, service
names, method types, and streaming directions.

`error.proto` and `messages/error.proto` define the typed `ErrorDetail` carried
in the `doso-error-bin` gRPC trailer. Clients should generate them along with the
service sources. The older integration descriptor export omitted these two
files because service messages do not import the trailer payload.

## Verify and export

Run these commands from the repository root with Python, the installed
`protobuf` package, and `protoc` on `PATH`.

```sh
python tools/sdk/proto.py --check
python tools/sdk/proto.py --check --descriptor-out build/sdk/lingtu-public.protoset
python -m pytest tests/contracts/test_public_proto.py -q
```

The check compiles all sources and compares every public file descriptor with
the Gateway. It covers options and the import graph as well as messages and
services. Source locations and automatically derived JSON names are normalized.
Explicit JSON name changes remain visible. The exported descriptor set contains
all 19 public files and the three Google well-known dependencies. `--protoc`
accepts an explicit compiler executable path.

The contract test skips compilation when `protoc` is unavailable. Google
well-known `.proto` imports come from the compiler installation. Keep the
compiler's matching include directory available when using a custom toolchain.

## Generate client bindings

Choose protobuf and gRPC compiler/runtime versions supported by your language
toolchain. Generation requires that toolchain's installed gRPC compiler plugin.
`common.proto` and the `messages/` sources are also compiler inputs because
service stubs import their generated message modules.

For Python with the `grpcio-tools` development tool installed, run from this
directory after creating the output directory.

```sh
python -m grpc_tools.protoc -I . --python_out=../../build/sdk/grpc-python --grpc_python_out=../../build/sdk/grpc-python common.proto error.proto localization.proto maps.proto motion.proto navigation.proto observations.proto system.proto tasks.proto messages/common.proto messages/controller.proto messages/error.proto messages/localization.proto messages/maps.proto messages/motion.proto messages/navigation.proto messages/observations.proto messages/system.proto messages/tasks.proto
```

Add that output directory to Python's module search path. Standard generated
imports use the top-level `common_pb2`, `maps_pb2`, and `messages` module names.

For C++, run the following from this directory after creating the output
directory. Set `GRPC_CPP_PLUGIN` to the installed `grpc_cpp_plugin` executable.
Generate all 19 files, using the same list as the Python command.

```sh
protoc -I . --cpp_out=../../build/sdk/grpc-cpp --grpc_out=../../build/sdk/grpc-cpp --plugin=protoc-gen-grpc="$GRPC_CPP_PLUGIN" common.proto error.proto localization.proto maps.proto motion.proto navigation.proto observations.proto system.proto tasks.proto messages/common.proto messages/controller.proto messages/error.proto messages/localization.proto messages/maps.proto messages/motion.proto messages/navigation.proto messages/observations.proto messages/system.proto messages/tasks.proto
```

For Rust, configure the consumer project's `tonic`/`prost` build tooling to
compile the 19 sources with this directory as its include root. Its generated
protobuf module is `doso.v1`, and Google well-known message types use that
toolchain's configured well-known type mapping. Keep these build dependencies
in the consuming project.
