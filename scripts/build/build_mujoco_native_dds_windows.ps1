param(
    [string]$CycloneDDSPrefix = $env:LINGTU_CYCLONEDDS_PREFIX,
    [string]$Configuration = "Release",
    [string]$BuildDirectory = "",
    [string]$Generator = "",
    [string]$Architecture = "",
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"

function Assert-WindowsVcRuntime {
    param(
        [Version]$MinimumVersion = [Version]"14.0.0.0"
    )

    if (-not [Environment]::Is64BitProcess) {
        throw "The Windows native DDS build requires 64-bit PowerShell."
    }
    $RuntimeRegistryPath = (
        "Registry::HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\VisualStudio\14.0\" +
        "VC\Runtimes\x64"
    )
    try {
        $RuntimeRegistration = Get-ItemProperty -LiteralPath $RuntimeRegistryPath
    }
    catch {
        throw "Microsoft Visual C++ Redistributable x64 is not registered."
    }
    if ([int]$RuntimeRegistration.Installed -ne 1) {
        throw "Microsoft Visual C++ Redistributable x64 is not installed."
    }
    $RuntimeVersionText = ([string]$RuntimeRegistration.Version).TrimStart("v")
    $RuntimeVersion = $null
    if (-not [Version]::TryParse($RuntimeVersionText, [ref]$RuntimeVersion)) {
        throw "Microsoft Visual C++ Redistributable x64 has an invalid registered version."
    }
    if ($RuntimeVersion -lt $MinimumVersion) {
        throw (
            "Microsoft Visual C++ Redistributable x64 is too old: " +
            "$RuntimeVersion; require at least $MinimumVersion for the selected MSVC toolset."
        )
    }

    $SystemDirectory = [Environment]::GetFolderPath(
        [Environment+SpecialFolder]::System
    )
    if ([string]::IsNullOrWhiteSpace($SystemDirectory)) {
        throw "Windows did not return its authoritative system directory."
    }
    $RequiredRuntimeDlls = @(
        "msvcp140.dll",
        "msvcp140_2.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
        "vcomp140.dll"
    )
    $MissingRuntimeDlls = @(
        $RequiredRuntimeDlls | Where-Object {
            -not (Test-Path -LiteralPath (Join-Path $SystemDirectory $_) -PathType Leaf)
        }
    )
    if ($MissingRuntimeDlls.Count -gt 0) {
        throw (
            "Microsoft Visual C++ Redistributable x64 is incomplete; missing runtime DLLs: " +
            ($MissingRuntimeDlls -join ", ")
        )
    }
    foreach ($RuntimeDll in $RequiredRuntimeDlls) {
        $RuntimePath = Join-Path $SystemDirectory $RuntimeDll
        $Stream = [System.IO.File]::Open(
            $RuntimePath,
            [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::Read,
            [System.IO.FileShare]::ReadWrite
        )
        try {
            $Reader = [System.IO.BinaryReader]::new($Stream)
            try {
                if ($Stream.Length -lt 64 -or $Reader.ReadUInt16() -ne 0x5A4D) {
                    throw "$RuntimePath is not a Windows PE file."
                }
                $Stream.Position = 0x3C
                $PeOffset = $Reader.ReadInt32()
                if ($PeOffset -lt 0 -or $PeOffset + 6 -gt $Stream.Length) {
                    throw "$RuntimePath has an invalid PE header offset."
                }
                $Stream.Position = $PeOffset
                if ($Reader.ReadUInt32() -ne 0x00004550) {
                    throw "$RuntimePath has an invalid PE signature."
                }
                if ($Reader.ReadUInt16() -ne 0x8664) {
                    throw "$RuntimePath is not an x64 PE runtime DLL."
                }
                $Stream.Position = $PeOffset + 20
                $OptionalHeaderSize = $Reader.ReadUInt16()
                if ($OptionalHeaderSize -lt 2 -or $PeOffset + 24 + $OptionalHeaderSize -gt $Stream.Length) {
                    throw "$RuntimePath has an invalid PE optional header size."
                }
                $Stream.Position = $PeOffset + 24
                if ($Reader.ReadUInt16() -ne 0x020B) {
                    throw "$RuntimePath is not a PE32+ runtime DLL."
                }
            }
            finally {
                $Reader.Dispose()
            }
        }
        finally {
            $Stream.Dispose()
        }
    }
}

function Get-ConfiguredMsvcToolsetVersion {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CachePath
    )

    $LinkerLine = Get-Content -LiteralPath $CachePath | Where-Object {
        $_ -match '^CMAKE_LINKER:FILEPATH='
    } | Select-Object -First 1
    if (-not $LinkerLine) {
        throw "Configured CMake cache does not declare CMAKE_LINKER: $CachePath"
    }
    $LinkerPath = ($LinkerLine -replace '^CMAKE_LINKER:FILEPATH=', '').Trim()
    if (
        $LinkerPath -notmatch (
            '(?i)[\\/]VC[\\/]Tools[\\/]MSVC[\\/](\d+\.\d+\.\d+)' +
            '[\\/]bin[\\/]Host(?:x64|x86)[\\/]x64[\\/]link\.exe$'
        )
    ) {
        throw "Configured linker is not from an MSVC x64 toolset: $LinkerPath"
    }
    return [Version]("$($Matches[1]).0")
}

Assert-WindowsVcRuntime

$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Source = Join-Path $Root "sim\adapters\dds"
if (-not $BuildDirectory) {
    $BuildDirectory = Join-Path $Root "build\windows-native-dds-adapter"
}

if ($PSBoundParameters.ContainsKey("Generator") -and [string]::IsNullOrWhiteSpace($Generator)) {
    throw "-Generator cannot be empty. Omit it for CMake's default Visual Studio generator, or pass a generator such as 'Ninja'."
}
if ($PSBoundParameters.ContainsKey("Architecture") -and [string]::IsNullOrWhiteSpace($Architecture)) {
    throw "-Architecture cannot be empty. Omit it to use x64 with Visual Studio or to avoid CMake's -A option with generators such as Ninja."
}

$RequestedGenerator = $Generator.Trim()
$EnvironmentGenerator = ([string]$env:CMAKE_GENERATOR).Trim()
$RequestedOrEnvironmentGenerator = $RequestedGenerator
if (-not $RequestedOrEnvironmentGenerator) {
    $RequestedOrEnvironmentGenerator = $EnvironmentGenerator
}

$CachePath = Join-Path $BuildDirectory "CMakeCache.txt"
$CachedGenerator = ""
$CachedConfigurationTypes = ""
if (Test-Path -LiteralPath $CachePath -PathType Leaf) {
    foreach ($CacheLine in Get-Content -LiteralPath $CachePath) {
        if ($CacheLine -match '^CMAKE_GENERATOR:INTERNAL=(.+)$') {
            $CachedGenerator = $Matches[1].Trim()
        }
        elseif ($CacheLine -match '^CMAKE_CONFIGURATION_TYPES:[^=]+=(.*)$') {
            $CachedConfigurationTypes = $Matches[1].Trim()
        }
    }
    if (-not $CachedGenerator) {
        throw "Existing CMake cache does not declare CMAKE_GENERATOR: $CachePath"
    }
    if (
        $RequestedOrEnvironmentGenerator -and
        $RequestedOrEnvironmentGenerator -cne $CachedGenerator
    ) {
        throw (
            "Requested CMake generator '$RequestedOrEnvironmentGenerator' conflicts with " +
            "the existing build directory generator '$CachedGenerator': $CachePath"
        )
    }
}
$EffectiveGenerator = if ($CachedGenerator) {
    $CachedGenerator
}
else {
    $RequestedOrEnvironmentGenerator
}

# With no explicit or environment override, CMake's Windows default is the
# Visual Studio generator. Preserve the helper's existing x64 default there.
$UsesDefaultVisualStudioGenerator = -not $EffectiveGenerator
$UsesVisualStudioGenerator =
    $UsesDefaultVisualStudioGenerator -or $EffectiveGenerator -match '^Visual Studio(?:\s|$)'
$GeneratorSupportsArchitecture =
    $UsesVisualStudioGenerator -or $EffectiveGenerator -eq "Green Hills MULTI"

$Architecture = $Architecture.Trim()
if ($Architecture -and -not $GeneratorSupportsArchitecture) {
    throw "CMake generator '$EffectiveGenerator' does not support the -A architecture option. Omit -Architecture when using this generator (for example: -Generator '$EffectiveGenerator'), and select its compiler through the generator's toolchain environment if needed."
}
if (-not $Architecture -and $UsesVisualStudioGenerator) {
    $Architecture = "x64"
}

$UsesMultiConfigGenerator = if ($CachedGenerator) {
    -not [string]::IsNullOrWhiteSpace($CachedConfigurationTypes)
}
else {
    $UsesVisualStudioGenerator -or
    $EffectiveGenerator -eq "Ninja Multi-Config" -or
    $EffectiveGenerator -eq "Xcode"
}

if (-not $CycloneDDSPrefix) {
    throw "Set LINGTU_CYCLONEDDS_PREFIX or pass -CycloneDDSPrefix with a CycloneDDS SDK install prefix."
}
$CycloneDDSPrefix = (Resolve-Path $CycloneDDSPrefix).Path
$CycloneDDSConfig = Join-Path $CycloneDDSPrefix "lib\cmake\CycloneDDS\CycloneDDSConfig.cmake"
if (-not (Test-Path -LiteralPath $CycloneDDSConfig -PathType Leaf)) {
    $CycloneDDSConfig = Join-Path $CycloneDDSPrefix "share\CycloneDDS\CycloneDDSConfig.cmake"
}
$Idlc = Join-Path $CycloneDDSPrefix "bin\idlc.exe"
$IdlcBackendDll = Join-Path $CycloneDDSPrefix "bin\cycloneddsidlc.dll"
$RuntimeDll = Join-Path $CycloneDDSPrefix "bin\ddsc.dll"
foreach ($RequiredPath in @($CycloneDDSConfig, $Idlc, $IdlcBackendDll, $RuntimeDll)) {
    if (-not (Test-Path -LiteralPath $RequiredPath -PathType Leaf)) {
        throw "CycloneDDS SDK is incomplete; required file is missing: $RequiredPath"
    }
}

$ConfigureArgs = @(
    "-S", $Source,
    "-B", $BuildDirectory,
    "-DCMAKE_PREFIX_PATH=$CycloneDDSPrefix",
    "-DLINGTU_MUJOCO_NATIVE_DDS_BUILD_RUNTIME=ON",
    "-DBUILD_TESTING=ON"
)
if ($EffectiveGenerator) {
    $ConfigureArgs += @("-G", $EffectiveGenerator)
}
if ($Architecture) {
    $ConfigureArgs += @("-A", $Architecture)
}
if (-not $UsesMultiConfigGenerator) {
    $ConfigureArgs += "-DCMAKE_BUILD_TYPE=$Configuration"
}
& cmake @ConfigureArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE"
}
$ConfiguredToolsetVersion = Get-ConfiguredMsvcToolsetVersion -CachePath $CachePath
Assert-WindowsVcRuntime -MinimumVersion $ConfiguredToolsetVersion

& cmake --build $BuildDirectory --config $Configuration --parallel
if ($LASTEXITCODE -ne 0) {
    throw "CMake build failed with exit code $LASTEXITCODE"
}

if (-not $SkipTests) {
    $CTest = (Get-Command ctest -ErrorAction Stop).Source
    $OriginalPath = $env:PATH
    $OriginalTestDdsDomainId = $env:LINGTU_TEST_DDS_DOMAIN_ID
    $OriginalTestDdsFailureDomainId = $env:LINGTU_TEST_DDS_FAILURE_DOMAIN_ID
    try {
        # Prove the app-local runtime closure. A developer SDK on PATH must not
        # make tests pass when ddsc.dll is missing beside an executable. CTest
        # owns every DDS test domain; developer-shell overrides must not leak
        # into tests that intentionally use their compiled default.
        $env:PATH = Join-Path $env:SystemRoot "System32"
        Remove-Item Env:LINGTU_TEST_DDS_DOMAIN_ID -ErrorAction SilentlyContinue
        Remove-Item Env:LINGTU_TEST_DDS_FAILURE_DOMAIN_ID -ErrorAction SilentlyContinue
        & $CTest --test-dir $BuildDirectory -C $Configuration --output-on-failure
        if ($LASTEXITCODE -ne 0) {
            throw "CTest failed with exit code $LASTEXITCODE"
        }
    }
    finally {
        $env:PATH = $OriginalPath
        if ($null -eq $OriginalTestDdsDomainId) {
            Remove-Item Env:LINGTU_TEST_DDS_DOMAIN_ID -ErrorAction SilentlyContinue
        }
        else {
            $env:LINGTU_TEST_DDS_DOMAIN_ID = $OriginalTestDdsDomainId
        }
        if ($null -eq $OriginalTestDdsFailureDomainId) {
            Remove-Item Env:LINGTU_TEST_DDS_FAILURE_DOMAIN_ID -ErrorAction SilentlyContinue
        }
        else {
            $env:LINGTU_TEST_DDS_FAILURE_DOMAIN_ID = $OriginalTestDdsFailureDomainId
        }
    }
}

$OutputDirectory = if ($UsesMultiConfigGenerator) {
    Join-Path $BuildDirectory $Configuration
}
else {
    $BuildDirectory
}
Write-Output "Built Windows native MuJoCo DDS adapter in $OutputDirectory"
Write-Output "The CycloneDDS runtime DLL is bundled beside the adapter executables."
