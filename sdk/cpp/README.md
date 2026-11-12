# LingTu C++ SDK

The C++17 SDK provides named clients for the Gateway's navigation, maps, tasks,
observations and system HTTP APIs, normally served on port 5050. It uses the
same Rust HTTP transport core as the Rust SDK through a private C ABI.

The public C++ interface uses standard-library types. JSON requests and owner
responses use UTF-8 `std::string` values. Binary responses retain their explicit
byte lengths. The Rust core owns JSON parsing, authentication, HTTP error
responses, command request identities, downloads, polling and SSE transport.

The installed SDK contains a C++ static facade and a Rust shared library. Its
build requires CMake 3.20 or newer, a C++17 compiler and Cargo with a compatible
Rust native target. Rust dependencies are pinned in `../rust/Cargo.lock`.

HTTP uses the system's proxy environment. Include `127.0.0.1,localhost,::1` in
`NO_PROXY` when connecting directly to a local Gateway on a machine using a proxy.

```cpp
#include <lingtu/sdk.hpp>
#include <iostream>

int main() {
    try {
        lingtu::sdk::Client robot("http://127.0.0.1:5050", "your-api-key");
        std::cout << robot.system().health() << '\n';
    } catch (const lingtu::sdk::Error& error) {
        std::cerr << error.status() << ' ' << error.body() << '\n';
        return 1;
    }
}
```

## Build

Run these commands from the LingTu repository root. On Windows, match the
MSVC Rust target with a Visual Studio x64 C++ build. The local build has been
checked with Visual Studio 2022 and `x86_64-pc-windows-msvc`.

```powershell
cmake -S sdk/cpp -B build/sdk/cpp -G "Visual Studio 17 2022" -A x64
cmake --build build/sdk/cpp --config Release
ctest --test-dir build/sdk/cpp -C Release --output-on-failure
```

On Linux, use the native C++ and Rust toolchains.

```sh
cmake -S sdk/cpp -B build/sdk/cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/sdk/cpp
ctest --test-dir build/sdk/cpp --output-on-failure
```

CMake builds the Rust core with `cargo build --release --locked`. Cargo may
download its locked dependencies when they are absent from its local cache.
For a machine whose Cargo cache already contains them, set
`CARGO_NET_OFFLINE=true` before building to keep dependency resolution offline.
All Cargo build outputs go inside the CMake build directory.

The standalone build includes a read-only health example and local HTTP and
WebSocket contract tests. The tests require Python 3 with `websockets>=15` and
use their own local servers. Set
`-DBUILD_TESTING=OFF` when preparing an SDK build without the test executable.

## Install and consume

```powershell
cmake --install build/sdk/cpp --config Release --prefix build/sdk/cpp-install
```

The installation contains `include/lingtu/sdk.hpp`, the C++ facade archive, the
Rust core library, and relocatable CMake package files. Consumers of an installed
SDK need their C++ compiler and the installed libraries. Cargo is used while
building the SDK itself. Consumers of the prebuilt installation use its shared
transport binary and can build their application without a Rust toolchain.

```cmake
cmake_minimum_required(VERSION 3.20)
project(MyLingTuClient LANGUAGES CXX)
find_package(LingTuSDK CONFIG REQUIRED)
add_executable(my_client main.cpp)
target_link_libraries(my_client PRIVATE LingTu::SDK)
if(WIN32)
    add_custom_command(TARGET my_client POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:LingtuSdkCore> $<TARGET_FILE_DIR:my_client>)
endif()
```

Configure that consumer with `-DCMAKE_PREFIX_PATH=/path/to/cpp-install`.
On Windows, place `lingtu_sdk.dll` beside the consuming executable, as in the
example above. On Linux, keep the installed shared library discoverable by the
application's runtime loader when deploying the executable.

For a source build inside another CMake project, keep `sdk/cpp` and `sdk/rust`
adjacent and use `add_subdirectory(path/to/sdk/cpp lingtu-sdk)`, then link
`LingTu::SDK`. Source inclusion builds the SDK libraries. The standalone build
provides the health example and tests.

## Binary feeds and operator input

```cpp
auto cloud = robot.watch_cloud();
if (auto frame = cloud.receive()) {
    // frame->data owns the complete message, including binary NUL bytes.
}
cloud.close();

auto camera = robot.watch_camera("viewer-token");
```

`receive()` returns `std::optional<Frame>`. `Frame::binary` distinguishes binary
and text messages. A clean peer close returns an empty optional. The feed owns
its connection and closes it when explicitly closed or destroyed.

For HTTP media access, pass the independent viewer token as the fourth client
constructor argument after the timeout. A token supplied to `watch_camera()`
overrides it for that feed.

```cpp
lingtu::sdk::Client robot("http://robot-host:5050", "gateway-key",
                          std::chrono::seconds(10), "viewer-token");
```

`robot.teleop(client_id)` opens an explicit operator session. Its `move(vx, vy,
yaw, manual_mode, request_id)` method requests a fresh input window and sends
one velocity sample. Keep the returned JSON `ingress_ack` or rejection with its
request identity. `hold()` releases the deadman. `close()` and destruction close
the socket so the Gateway can perform its existing disconnect hold and release.
Receipt fields continue to distinguish queue ingress, final output and motors.

## Product and saved data

`system().product_status()`, `switch_product(product, expected_session, options)`
and `product_operation(request_id)` preserve the ProductControl operation JSON.
Switch options carry the map name, request identity and optional initial pose.
The owning ProductControl executes the lifecycle transaction.

`stop_product(expected_session, options)` submits the same owner's durable stop
transaction. Supply `request_id` before submission. A Product stop can close its
Gateway connection. The independent control service retains the operation,
which can be queried once the Gateway is reachable again or through the local
control transport. A lost response leaves the result unconfirmed.

`navigation().goal_status(request_id)` reads a submitted goal by request identity.
`system().resume_motion(options)` releases native motion hold and retains its
original receipt. `observations().stop_recording(session_id)` supplies the
expected recording identity, while the existing zero-argument call stops the
active recording.

`observations()` includes recording catalog, stream and session lookup, summary,
series, scene and historical camera queries, validation and deletion. Repeated
`path_topic` query pairs select multiple saved paths. An unavailable historical
camera frame returns an empty string. `download_recording_file()` accepts a
declared relative artifact path, including nested folders, and
`download_recording_archive()` saves the session tar archive.

`tasks()` can save/read/delete inspection routes, list retained tasks, inspect
and review evidence, and download artifacts or individual sensor measurements.
Route JSON uses the public map, revision and point fields. Evidence review
records a finding through the existing evidence owner. All downloads retain
an existing destination if the transfer fails.

## Package

After building, create a platform archive from the installable SDK files.

```powershell
cpack --config build/sdk/cpp/CPackConfig.cmake -C Release -B build/sdk/dist
```

On Windows the output is
`build/sdk/dist/lingtu-sdk-cpp-2.3.0-Windows-AMD64.zip`. Other native platforms
use a `.tar.gz` archive with their CMake system and processor names. The archive
contains the header, facade archive, Rust shared core and its import library
where required, relocatable CMake package files, and this README. Extract the
archive and pass its top-level directory to the consumer's `CMAKE_PREFIX_PATH`.

## Contracts

Keep complete JSON responses when recording admission, execution, failure
reasons and task identities. Owner acceptance and physical completion have
separate evidence. A local polling timeout leaves the external task under its
existing owner.

Supply caller-selected `request_id` values when reconciling a submission after
a transport failure. Reusing the same identity preserves the Gateway's declared
idempotency contract. JSON options carry the current HTTP schema fields.

`Client::events` blocks while delivering each SSE message to a callback on the
calling thread. Return `false` from the callback to stop the subscription and
close its response. The configured timeout bounds an idle read. Select a timeout greater than the
Gateway's one-second heartbeat interval. Polling deadlines are checked between
requests, so an in-flight request can consume its configured timeout after the
polling deadline.

The shared schemas in `../proto` describe the separate gRPC API for consumers
who generate bindings with their selected protobuf and gRPC runtimes. A rejected
WebSocket handshake retains its HTTP status. The WebSocket library may omit the
optional rejection body. HTTP requests retain their full error response body.

Verification of this package establishes local SDK and HTTP contracts.
Simulation, field readiness and physical motion require their own evidence.
WSS currently uses WebPKI roots. A caller-supplied private CA is not yet configurable.
