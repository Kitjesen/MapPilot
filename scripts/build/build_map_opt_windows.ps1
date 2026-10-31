# Build the current saved-map tools with an isolated static GTSAM SDK.
param([ValidateRange(1, 16)][int]$Jobs = 2)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$version = (Get-Content "$PSScriptRoot/locks/gtsam.json" -Raw | ConvertFrom-Json).version
$sdk = "$repo/build/deps/gtsam-$version-windows-static-sdk"
$prefix = "$repo/third_party/install/slam-windows/x64-windows"
$build = "$repo/build/map_optimization"
if (!(Test-Path "$sdk/lib/libgtsam.lib")) {
    throw 'Run scripts/build/prepare_gtsam_windows_static.ps1 first.'
}
$previousCL = $env:_CL_
$previousReuse = $env:MSBUILDDISABLENODEREUSE
$previousOffline = $env:CARGO_NET_OFFLINE
try {
    $env:_CL_ = "/MP$Jobs"
    $env:MSBUILDDISABLENODEREUSE = '1'
    $env:CARGO_NET_OFFLINE = 'true'
    New-Item -ItemType Directory -Force -Path $build | Out-Null
    & cmake -S "$repo/src/localization/map_optimization" -B $build `
        -G 'Visual Studio 17 2022' -A x64 -U GTSAM_INCLUDE_DIR `
        "-DGTSAM_DIR=$sdk/CMake" "-DCMAKE_PREFIX_PATH=$sdk;$prefix" -DBUILD_TESTING=ON `
        *> "$build/windows-configure.log"
    if ($LASTEXITCODE) { throw "PGO configure failed: $build/windows-configure.log" }
    & cmake --build $build --config Release --target lt_pgo lt_loop_verify pgo_test `
        loop_information_test constraint_assembly_test online_graph_test --parallel $Jobs -- /nodeReuse:false `
        *> "$build/windows-build.log"
    if ($LASTEXITCODE) { throw "PGO build failed: $build/windows-build.log" }
    $linkerLine = Select-String -Path "$build/CMakeCache.txt" -Pattern '^CMAKE_LINKER:FILEPATH='
    $dumpbin = Join-Path (Split-Path ($linkerLine.Line -split '=', 2)[1]) 'dumpbin.exe'
    & cmake "-DTOOL_DIRECTORY=$build/Release" "-DDEPENDENCY_DIRECTORY=$prefix/bin" "-DDUMPBIN=$dumpbin" `
        -P "$PSScriptRoot/cmake/stage_map_opt_windows.cmake" *> "$build/windows-stage.log"
    if ($LASTEXITCODE) { throw "PGO DLL staging failed: $build/windows-stage.log" }
    & ctest --test-dir $build -C Release --output-on-failure `
        -R '^(pgo_core|loop_information|constraint_assembly|online_graph)$' `
        --output-junit "$build/windows-tests.xml" *> "$build/windows-tests.log"
    if ($LASTEXITCODE) { throw "PGO tests failed: $build/windows-tests.log" }
    Write-Output "Current saved-map tools and tests passed: $build/Release"
} finally {
    $env:_CL_ = $previousCL
    $env:MSBUILDDISABLENODEREUSE = $previousReuse
    $env:CARGO_NET_OFFLINE = $previousOffline
}
