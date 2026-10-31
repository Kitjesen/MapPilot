# Isolated static SDK for the short-lived Windows saved-map optimizer.
# Reuses local sources/dependencies; never replaces the shared SLAM SDK.
param(
    [string]$Source = '',
    [string]$BuildDirectory = '',
    [string]$InstallDirectory = '',
    [ValidateRange(1, 16)][int]$Jobs = 2
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$lock = Get-Content (Join-Path $PSScriptRoot 'locks/gtsam.json') -Raw | ConvertFrom-Json
if (!$Source) { $Source = "$repo/build/deps/gtsam-$($lock.version)" }
if (!$BuildDirectory) { $BuildDirectory = "$repo/build/deps/gtsam-$($lock.version)-windows-static-build" }
if (!$InstallDirectory) { $InstallDirectory = "$repo/build/deps/gtsam-$($lock.version)-windows-static-sdk" }
$sourceRoot = (Resolve-Path -LiteralPath $Source).Path
$prefix = "$repo/third_party/install/slam-windows/x64-windows"
if (!(Test-Path "$prefix/share/eigen3/Eigen3Config.cmake")) {
    throw 'The existing Windows Eigen dependency is required; no packages were installed.'
}
New-Item -ItemType Directory -Force -Path $BuildDirectory | Out-Null
$previousCL = $env:_CL_
$previousReuse = $env:MSBUILDDISABLENODEREUSE
try {
    $env:_CL_ = "/MP$Jobs"
    $env:MSBUILDDISABLENODEREUSE = '1'
    & cmake -S $sourceRoot -B $BuildDirectory -G 'Visual Studio 17 2022' -A x64 `
        "-DCMAKE_INSTALL_PREFIX=$InstallDirectory" "-DCMAKE_PREFIX_PATH=$prefix" `
        '-DCMAKE_CXX_FLAGS=/utf-8' -DBUILD_SHARED_LIBS=OFF `
        -DGTSAM_FORCE_SHARED_LIB=OFF -DGTSAM_FORCE_STATIC_LIB=ON `
        -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF `
        -DGTSAM_INSTALL_CPPUNITLITE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF `
        -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF `
        -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_ENABLE_CUDA=OFF `
        -DGTSAM_WITH_TBB=OFF -DGTSAM_WITH_EIGEN_MKL=OFF `
        -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF `
        *> "$BuildDirectory/configure.log"
    if ($LASTEXITCODE) { throw "GTSAM configure failed: $BuildDirectory/configure.log" }
    $config = Get-Content "$BuildDirectory/gtsam/config.h" -Raw
    if ($config -notmatch ('#define GTSAM_VERSION_STRING "' + [regex]::Escape($lock.version) + '"')) {
        throw 'Local GTSAM sources do not match the repository version lock.'
    }
    & cmake --build $BuildDirectory --config Release --target gtsam --parallel $Jobs -- /nodeReuse:false `
        *> "$BuildDirectory/build.log"
    if ($LASTEXITCODE) { throw "GTSAM build failed: $BuildDirectory/build.log" }
    & cmake --install $BuildDirectory --config Release *> "$BuildDirectory/install.log"
    if ($LASTEXITCODE) { throw "GTSAM install failed: $BuildDirectory/install.log" }
    $licenses = Join-Path $InstallDirectory 'share/lingtu/gtsam-licenses'
    New-Item -ItemType Directory -Force -Path $licenses | Out-Null
    Copy-Item -LiteralPath "$sourceRoot/LICENSE", "$sourceRoot/LICENSE.BSD" -Destination $licenses
    Get-ChildItem "$sourceRoot/gtsam/3rdparty" -Recurse -File |
        Where-Object { $_.Name -match '(?i)license|^COPYING' } | ForEach-Object {
            $relative = $_.FullName.Substring($sourceRoot.Length).TrimStart('\', '/')
            $target = Join-Path $licenses $relative
            New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
            Copy-Item -LiteralPath $_.FullName -Destination $target
        }
    Write-Output "Static GTSAM SDK: $InstallDirectory"
} finally {
    $env:_CL_ = $previousCL
    $env:MSBUILDDISABLENODEREUSE = $previousReuse
}
