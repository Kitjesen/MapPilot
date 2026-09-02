[CmdletBinding()]
param(
    [string]$DependencyPrefix,
    [string]$CycloneDDSPrefix,
    [string]$VcpkgRoot,
    [string]$VcpkgInstallRoot,
    [string]$VcpkgBinaryCache,
    [string]$GitExecutable = "git",
    [string]$BuildDir = "",
    [ValidateSet("Release")]
    [string]$Configuration = "Release",
    [switch]$PreflightOnly
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Source = Join-Path $Root "src\localization\slam\cpp"
$VcpkgManifestRoot = Join-Path $PSScriptRoot "vcpkg\slam-windows"
$CycloneDDSCanonicalLock = Join-Path $PSScriptRoot "locks\cyclonedds-windows-x64.json"
$CycloneDDSVerifier = Join-Path $PSScriptRoot "verify_cyclonedds_windows_sdk.ps1"
$CycloneLockData = Get-Content -Raw -LiteralPath $CycloneDDSCanonicalLock | ConvertFrom-Json
$ExpectedCycloneDDSVersion = [string]$CycloneLockData.tag

function Resolve-ExplicitPrefix {
    param([string]$Value, [string]$ParameterName)

    if ([string]::IsNullOrWhiteSpace($Value)) {
        throw "-$ParameterName is required and must name an absolute MSVC dependency prefix."
    }
    if (-not [System.IO.Path]::IsPathFullyQualified($Value)) {
        throw "-$ParameterName must be an explicit absolute path: $Value"
    }
    if (-not (Test-Path -LiteralPath $Value -PathType Container)) {
        throw "-$ParameterName does not exist or is not a directory: $Value"
    }
    $Resolved = (Resolve-Path -LiteralPath $Value).Path
    if ($Resolved -match '(?i)(^|[\\/])(mingw|msys|ucrt64|clang64)([\\/]|$)') {
        throw "-$ParameterName must be an MSVC x64 prefix, not a MinGW/MSYS prefix: $Resolved"
    }
    return $Resolved
}

function Resolve-PrefixFile {
    param([string]$Prefix, [string[]]$Candidates, [string]$Description)

    foreach ($RelativePath in $Candidates) {
        $Candidate = Join-Path $Prefix $RelativePath
        if (Test-Path -LiteralPath $Candidate -PathType Leaf) {
            $Resolved = (Resolve-Path -LiteralPath $Candidate).Path
            $PrefixBoundary = $Prefix.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
            if (-not $Resolved.StartsWith($PrefixBoundary, [System.StringComparison]::OrdinalIgnoreCase)) {
                throw "$Description resolves outside its explicit prefix: $Resolved"
            }
            return $Resolved
        }
    }
    throw "$Description is missing from explicit prefix '$Prefix'. Tried: $($Candidates -join ', ')"
}

function Get-CMakeCacheValue {
    param([string[]]$Lines, [string]$Name)

    $Match = $Lines | Where-Object { $_ -match "^$([regex]::Escape($Name)):[^=]+=" } | Select-Object -First 1
    if (-not $Match) { return $null }
    return ($Match -split '=', 2)[1]
}

function Assert-AuthoritativeCycloneDDSSdk {
    & $PowerShellExecutable -NoProfile -File $CycloneDDSVerifier -SdkRoot $CycloneDDSPrefix
    if ($LASTEXITCODE -ne 0) {
        throw "authoritative CycloneDDS SDK verification failed with exit code $LASTEXITCODE"
    }
}

function Assert-StagedExecutableLoads {
    param([string]$Path, [string]$ExpectedUsage)

    $OriginalPath = $env:PATH
    try {
        $env:PATH = [System.Environment]::SystemDirectory
        $HelpOutput = ((& $Path --help 2>&1) | Out-String)
        $ExitCode = $LASTEXITCODE
    }
    finally {
        $env:PATH = $OriginalPath
    }
    if ($ExitCode -lt 0) {
        throw "Windows loader failed for staged executable '$Path' with exit code $ExitCode"
    }
    if ($ExitCode -ne 2 -or -not $HelpOutput.Contains($ExpectedUsage)) {
        throw "Staged executable smoke failed for '$Path': expected --help exit 2 and '$ExpectedUsage', got exit $ExitCode and output '$($HelpOutput.Trim())'"
    }
}

$DependencyPrefix = Resolve-ExplicitPrefix $DependencyPrefix "DependencyPrefix"
$CycloneDDSPrefix = Resolve-ExplicitPrefix $CycloneDDSPrefix "CycloneDDSPrefix"
$VcpkgRoot = Resolve-ExplicitPrefix $VcpkgRoot "VcpkgRoot"
$VcpkgInstallRoot = Resolve-ExplicitPrefix $VcpkgInstallRoot "VcpkgInstallRoot"
$VcpkgBinaryCache = Resolve-ExplicitPrefix $VcpkgBinaryCache "VcpkgBinaryCache"
if ($VcpkgBinaryCache -match '[,;]') {
    throw "-VcpkgBinaryCache must not contain a comma or semicolon: $VcpkgBinaryCache"
}
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $Root "build\slam-core-windows-x64"
}
elseif (-not [System.IO.Path]::IsPathFullyQualified($BuildDir)) {
    throw "-BuildDir must be an absolute path: $BuildDir"
}

$VcpkgManifest = Join-Path $VcpkgManifestRoot "vcpkg.json"
if (-not (Test-Path -LiteralPath $VcpkgManifest -PathType Leaf)) {
    throw "Pinned vcpkg manifest is missing: $VcpkgManifest"
}
$Manifest = Get-Content -Raw -LiteralPath $VcpkgManifest | ConvertFrom-Json
$ManifestSha256 = (Get-FileHash -LiteralPath $VcpkgManifest -Algorithm SHA256).Hash.ToLowerInvariant()
$ExpectedVcpkgBaseline = [string]$Manifest.'builtin-baseline'
$VcpkgHead = Resolve-PrefixFile $VcpkgRoot @(".git\HEAD") "vcpkg detached HEAD"
if ((Get-Content -Raw -LiteralPath $VcpkgHead).Trim() -ne $ExpectedVcpkgBaseline) {
    throw "VcpkgRoot is not checked out at pinned baseline ${ExpectedVcpkgBaseline}: $VcpkgRoot"
}
$VcpkgToolchain = Resolve-PrefixFile $VcpkgRoot @("scripts\buildsystems\vcpkg.cmake") "vcpkg.cmake"
$GitCommand = Get-Command $GitExecutable -ErrorAction SilentlyContinue
if (-not $GitCommand) { throw "Git executable was not found: $GitExecutable" }
$ActualVcpkgHead = (& $GitCommand.Source -C $VcpkgRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $ActualVcpkgHead -ne $ExpectedVcpkgBaseline) {
    throw "VcpkgRoot HEAD drift: expected $ExpectedVcpkgBaseline, found $ActualVcpkgHead"
}
& $GitCommand.Source -C $VcpkgRoot symbolic-ref -q HEAD 2>$null | Out-Null
if ($LASTEXITCODE -eq 0) { throw "VcpkgRoot must be a detached checkout at the pinned baseline" }
$VcpkgDirty = ((@(& $GitCommand.Source -C $VcpkgRoot status --porcelain --untracked-files=normal)) -join "`n").Trim()
if ($LASTEXITCODE -ne 0 -or $VcpkgDirty) { throw "VcpkgRoot must be clean; git status reported: $VcpkgDirty" }
$ExpectedDependencyPrefix = Join-Path $VcpkgInstallRoot "x64-windows"
if (-not [System.IO.Path]::GetFullPath($DependencyPrefix).Equals(
        [System.IO.Path]::GetFullPath($ExpectedDependencyPrefix),
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "DependencyPrefix must be the pinned x64-windows install under VcpkgInstallRoot: expected $ExpectedDependencyPrefix, found $DependencyPrefix"
}

$EigenConfig = Resolve-PrefixFile $DependencyPrefix @(
    "share\eigen3\Eigen3Config.cmake", "lib\cmake\eigen3\Eigen3Config.cmake"
) "Eigen3Config.cmake"
$PclConfig = Resolve-PrefixFile $DependencyPrefix @(
    "share\pcl\PCLConfig.cmake", "lib\cmake\pcl\PCLConfig.cmake"
) "PCLConfig.cmake"
$YamlConfig = Resolve-PrefixFile $DependencyPrefix @(
    "share\yaml-cpp\yaml-cpp-config.cmake", "lib\cmake\yaml-cpp\yaml-cpp-config.cmake"
) "yaml-cpp config"
$CycloneDDSConfig = Resolve-PrefixFile $CycloneDDSPrefix @(
    "lib\cmake\CycloneDDS\CycloneDDSConfig.cmake",
    "share\CycloneDDS\CycloneDDSConfig.cmake"
) "CycloneDDSConfig.cmake"
$CycloneDDSConfigDir = Split-Path -Parent $CycloneDDSConfig
$CycloneDDSVersionConfig = Join-Path $CycloneDDSConfigDir "CycloneDDSConfigVersion.cmake"
if (-not (Test-Path -LiteralPath $CycloneDDSVersionConfig -PathType Leaf)) {
    throw "CycloneDDSConfigVersion.cmake is missing beside $CycloneDDSConfig"
}
$VersionText = Get-Content -Raw -LiteralPath $CycloneDDSVersionConfig
if ($VersionText -notmatch 'PACKAGE_VERSION\s+"([^\"]+)"') {
    throw "Cannot determine CycloneDDS SDK version from $CycloneDDSVersionConfig"
}
if ($Matches[1] -ne $ExpectedCycloneDDSVersion) {
    throw "CycloneDDS version drift: expected $ExpectedCycloneDDSVersion, found $($Matches[1]) in $CycloneDDSVersionConfig"
}
$Idlc = Resolve-PrefixFile $CycloneDDSPrefix @("bin\idlc.exe") "CycloneDDS idlc.exe"
$CycloneDDSReceipt = Resolve-PrefixFile $CycloneDDSPrefix @(
    "evidence\sdk-receipt.json"
) "CycloneDDS verified SDK receipt"
if (-not (Test-Path -LiteralPath $CycloneDDSVerifier -PathType Leaf)) {
    throw "Canonical CycloneDDS SDK verifier is missing: $CycloneDDSVerifier"
}
$VsWhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $VsWhere -PathType Leaf)) {
    throw "Visual Studio Installer vswhere.exe was not found; Visual Studio 2022 with MSVC x64 tools is required."
}
$VsArgs = @("-latest", "-products", "*", "-version", "[17.0,18.0)", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64")
$VsVersion = (& $VsWhere @VsArgs -property installationVersion).Trim()
if (-not $VsVersion -or -not $VsVersion.StartsWith("17.")) {
    throw "Visual Studio 2022 with MSVC x64 tools was not found."
}
$VsInstall = (& $VsWhere @VsArgs -property installationPath).Trim()
$Dumpbin = Get-ChildItem -LiteralPath (Join-Path $VsInstall "VC\Tools\MSVC") -Filter "dumpbin.exe" -Recurse |
    Where-Object { $_.FullName -match '[\\/]bin[\\/]Hostx64[\\/]x64[\\/]dumpbin\.exe$' } |
    Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
if (-not $Dumpbin) { throw "Visual Studio 2022 x64 dumpbin.exe was not found under $VsInstall" }
$CMakeCommand = Get-Command cmake -ErrorAction SilentlyContinue
if (-not $CMakeCommand) {
    throw "cmake is required."
}
$CMakeVersionText = (& $CMakeCommand.Source --version | Select-Object -First 1)
if ($LASTEXITCODE -ne 0 -or $CMakeVersionText -notmatch 'cmake version ([0-9]+\.[0-9]+(?:\.[0-9]+)?)') {
    throw "Cannot determine CMake version from '$($CMakeCommand.Source)'."
}
if ([version]$Matches[1] -lt [version]"3.27") {
    throw "CMake 3.27 or newer is required; found $($Matches[1])."
}

$CachePath = Join-Path $BuildDir "CMakeCache.txt"
if (Test-Path -LiteralPath $CachePath -PathType Leaf) {
    $CacheLines = Get-Content -LiteralPath $CachePath
    $CacheSystem = Get-CMakeCacheValue $CacheLines "CMAKE_SYSTEM_NAME"
    $CacheGenerator = Get-CMakeCacheValue $CacheLines "CMAKE_GENERATOR"
    $CachePlatform = Get-CMakeCacheValue $CacheLines "CMAKE_GENERATOR_PLATFORM"
    $CacheCycloneDir = Get-CMakeCacheValue $CacheLines "CycloneDDS_DIR"
    $CacheToolchain = Get-CMakeCacheValue $CacheLines "CMAKE_TOOLCHAIN_FILE"
    $CacheInstalled = Get-CMakeCacheValue $CacheLines "VCPKG_INSTALLED_DIR"
    $CacheTargetTriplet = Get-CMakeCacheValue $CacheLines "VCPKG_TARGET_TRIPLET"
    $CacheHostTriplet = Get-CMakeCacheValue $CacheLines "VCPKG_HOST_TRIPLET"
    $CacheCrt = Get-CMakeCacheValue $CacheLines "CMAKE_MSVC_RUNTIME_LIBRARY"
    $CacheIdlc = Get-CMakeCacheValue $CacheLines "CYCLONEDDS_IDLC_EXECUTABLE"
    $CacheEigen = Get-CMakeCacheValue $CacheLines "Eigen3_DIR"
    $CachePcl = Get-CMakeCacheValue $CacheLines "PCL_DIR"
    $CacheYaml = Get-CMakeCacheValue $CacheLines "yaml-cpp_DIR"
    $CacheSource = Get-CMakeCacheValue $CacheLines "CMAKE_HOME_DIRECTORY"
    $CacheManifestSha256 = Get-CMakeCacheValue $CacheLines "LINGTU_VCPKG_MANIFEST_SHA256"
    if (($CacheSystem -and $CacheSystem -ne "Windows") -or
        $CacheGenerator -ne "Visual Studio 17 2022" -or
        $CachePlatform -ne "x64") {
        throw "Refusing to reuse non-Windows/VS2022-x64 CMake cache: $CachePath"
    }
    if (-not $CacheCycloneDir -or -not [System.IO.Path]::GetFullPath($CacheCycloneDir).Equals(
            [System.IO.Path]::GetFullPath($CycloneDDSConfigDir),
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "CycloneDDS path drift in CMake cache '$CachePath': expected $CycloneDDSConfigDir, found $CacheCycloneDir"
    }
    if (-not $CacheToolchain -or -not [System.IO.Path]::GetFullPath($CacheToolchain).Equals(
            [System.IO.Path]::GetFullPath($VcpkgToolchain),
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "vcpkg toolchain drift in CMake cache '$CachePath': expected $VcpkgToolchain, found $CacheToolchain"
    }
    if (-not $CacheInstalled -or -not [System.IO.Path]::GetFullPath($CacheInstalled).Equals(
            [System.IO.Path]::GetFullPath($VcpkgInstallRoot),
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "vcpkg install-root drift in CMake cache '$CachePath': expected $VcpkgInstallRoot, found $CacheInstalled"
    }
    if ($CacheTargetTriplet -ne "x64-windows" -or $CacheHostTriplet -ne "x64-windows") {
        throw "vcpkg triplet drift in CMake cache '$CachePath': target=$CacheTargetTriplet host=$CacheHostTriplet"
    }
    if ($CacheCrt -ne "MultiThreadedDLL") { throw "MSVC CRT drift in CMake cache '$CachePath': $CacheCrt" }
    if (-not $CacheSource -or -not [System.IO.Path]::GetFullPath($CacheSource).Equals(
            [System.IO.Path]::GetFullPath($Source),
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "source directory drift in CMake cache '$CachePath': expected $Source, found $CacheSource"
    }
    if ($CacheManifestSha256 -ne $ManifestSha256) {
        throw "vcpkg manifest SHA256 drift in CMake cache '$CachePath': expected $ManifestSha256, found $CacheManifestSha256"
    }
    foreach ($FlagIdentity in @(
            @("LINGTU_SLAM_FASTLIO2_BACKEND", "ON"),
            @("LINGTU_SLAM_BUILD_DDS_RUNTIME", "ON"),
            @("LINGTU_SLAM_BUILD_TESTS", "ON"),
            @("LINGTU_ENABLE_SMALL_GICP", "OFF"))) {
        $CachedFlag = Get-CMakeCacheValue $CacheLines $FlagIdentity[0]
        if ($CachedFlag -ne $FlagIdentity[1]) {
            throw "$($FlagIdentity[0]) drift in CMake cache '$CachePath': expected $($FlagIdentity[1]), found $CachedFlag"
        }
    }
    foreach ($CacheIdentity in @(
            @("CYCLONEDDS_IDLC_EXECUTABLE", $CacheIdlc, $Idlc),
            @("Eigen3_DIR", $CacheEigen, (Split-Path -Parent $EigenConfig)),
            @("PCL_DIR", $CachePcl, (Split-Path -Parent $PclConfig)),
            @("yaml-cpp_DIR", $CacheYaml, (Split-Path -Parent $YamlConfig)))) {
        if (-not $CacheIdentity[1] -or -not [System.IO.Path]::GetFullPath($CacheIdentity[1]).Equals(
                [System.IO.Path]::GetFullPath($CacheIdentity[2]),
                [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "$($CacheIdentity[0]) drift in CMake cache '$CachePath': expected $($CacheIdentity[2]), found $($CacheIdentity[1])"
        }
    }
}

Write-Output "Preflight passed: Visual Studio $VsVersion, x64 dependencies, CycloneDDS $ExpectedCycloneDDSVersion."
if ($PreflightOnly) {
    return
}

$PowerShellExecutable = Join-Path $PSHOME "pwsh.exe"
if (-not (Test-Path -LiteralPath $PowerShellExecutable -PathType Leaf)) {
    throw "PowerShell executable for authoritative CycloneDDS SDK verification is missing: $PowerShellExecutable"
}
Assert-AuthoritativeCycloneDDSSdk

$ConfigureArgs = @(
    "-S", $Source,
    "-B", $BuildDir,
    "-G", "Visual Studio 17 2022",
    "-A", "x64",
    "-DCMAKE_TOOLCHAIN_FILE=$VcpkgToolchain",
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL",
    "-DVCPKG_MANIFEST_MODE=ON",
    "-DVCPKG_MANIFEST_DIR=$VcpkgManifestRoot",
    "-DVCPKG_INSTALLED_DIR=$VcpkgInstallRoot",
    "-DVCPKG_TARGET_TRIPLET=x64-windows",
    "-DVCPKG_HOST_TRIPLET=x64-windows",
    "-DVCPKG_BINARY_SOURCES=clear;files,$VcpkgBinaryCache,readwrite",
    "-DLINGTU_VCPKG_MANIFEST_SHA256=$ManifestSha256",
    "-DCMAKE_PREFIX_PATH=$DependencyPrefix;$CycloneDDSPrefix",
    "-DEigen3_DIR=$(Split-Path -Parent $EigenConfig)",
    "-DPCL_DIR=$(Split-Path -Parent $PclConfig)",
    "-Dyaml-cpp_DIR=$(Split-Path -Parent $YamlConfig)",
    "-DCycloneDDS_DIR=$CycloneDDSConfigDir",
    "-DCYCLONEDDS_IDLC_EXECUTABLE=$Idlc",
    "-DLINGTU_SLAM_FASTLIO2_BACKEND=ON",
    "-DLINGTU_SLAM_BUILD_DDS_RUNTIME=ON",
    "-DLINGTU_SLAM_BUILD_TESTS=ON",
    "-DLINGTU_ENABLE_SMALL_GICP=OFF"
)
$OriginalBinarySources = $env:VCPKG_BINARY_SOURCES
try {
    $env:VCPKG_BINARY_SOURCES = "clear;files,$VcpkgBinaryCache,readwrite"
    & cmake @ConfigureArgs
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed with exit code $LASTEXITCODE" }
}
finally {
    $env:VCPKG_BINARY_SOURCES = $OriginalBinarySources
}

& cmake --build $BuildDir --config $Configuration --target slamd slamctl --parallel
if ($LASTEXITCODE -ne 0) { throw "slamd/slamctl build failed with exit code $LASTEXITCODE" }
& cmake --build $BuildDir --config $Configuration --parallel
if ($LASTEXITCODE -ne 0) { throw "SLAM tests build failed with exit code $LASTEXITCODE" }

$OriginalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $DependencyPrefix 'bin');$(Join-Path $CycloneDDSPrefix 'bin');$OriginalPath"
    & ctest --test-dir $BuildDir -C $Configuration --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw "CTest failed with exit code $LASTEXITCODE" }
}
finally {
    $env:PATH = $OriginalPath
}

$StageScript = Join-Path $PSScriptRoot "cmake\stage_windows_runtime.cmake"
$RuntimeDir = Join-Path $BuildDir $Configuration
$StageRoot = Join-Path $BuildDir "stage"
& cmake `
    "-DSLAMD_EXECUTABLE=$(Join-Path $RuntimeDir 'slamd.exe')" `
    "-DSLAMCTL_EXECUTABLE=$(Join-Path $RuntimeDir 'slamctl.exe')" `
    "-DDEPENDENCY_PREFIX=$DependencyPrefix" `
    "-DCYCLONEDDS_PREFIX=$CycloneDDSPrefix" `
    "-DCYCLONEDDS_SDK_RECEIPT=$CycloneDDSReceipt" `
    "-DCYCLONEDDS_CANONICAL_LOCK=$CycloneDDSCanonicalLock" `
    "-DCYCLONEDDS_VERSION=$ExpectedCycloneDDSVersion" `
    "-DDEPENDENCY_TOOL=$Dumpbin" `
    "-DWINDOWS_SYSTEM_DIRECTORY=$([System.Environment]::SystemDirectory)" `
    "-DALLOWED_STAGE_PARENT=$BuildDir" `
    "-DSTAGE_ROOT=$StageRoot" `
    -P $StageScript
if ($LASTEXITCODE -ne 0) { throw "Windows runtime staging failed with exit code $LASTEXITCODE" }

$StageBin = Join-Path $StageRoot "bin"
Assert-StagedExecutableLoads (Join-Path $StageBin "slamd.exe") "usage: slamd "
Assert-StagedExecutableLoads (Join-Path $StageBin "slamctl.exe") "usage: slamctl "

Write-Output "Built, tested, and staged Windows x64 SLAM core in $StageRoot"
