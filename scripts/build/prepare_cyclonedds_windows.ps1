param(
    [string]$SourceRoot = "",
    [string]$BuildRoot = "",
    [string]$InstallRoot = "",
    [string]$SdkRoot = "",
    [switch]$PreflightOnly
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$LockPath = Join-Path $PSScriptRoot "locks\cyclonedds-windows-x64.json"
$StageScript = Join-Path $PSScriptRoot "cmake\stage_cyclonedds_windows_sdk.cmake"
$VerifyScript = Join-Path $PSScriptRoot "verify_cyclonedds_windows_sdk.ps1"
$SmokeIdl = Join-Path $RepoRoot "tests\fixtures\idl\cyclonedds_smoke.idl"

function Resolve-AbsolutePathInput {
    param([string]$Value, [string]$Default, [string]$Name)
    if ([string]::IsNullOrWhiteSpace($Value)) { $Value = $Default }
    if (-not [System.IO.Path]::IsPathFullyQualified($Value)) { throw "-$Name must be an absolute path: $Value" }
    [System.IO.Path]::GetFullPath($Value)
}

function Invoke-Checked {
    param([string]$Program, [string[]]$Arguments, [string]$Failure)
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Failure (exit $LASTEXITCODE)" }
}

function Get-GitText {
    param([string[]]$Arguments)
    $result = (& git @Arguments 2>&1 | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw "git $($Arguments -join ' ') failed: $result" }
    $result
}

function Assert-CheckoutEolPolicy {
    param([string]$Path)
    $autocrlf = (& git -C $Path config --local --get core.autocrlf 2>$null | Out-String).Trim()
    $autocrlfExit = $LASTEXITCODE
    $eol = (& git -C $Path config --local --get core.eol 2>$null | Out-String).Trim()
    $eolExit = $LASTEXITCODE
    $safecrlf = (& git -C $Path config --local --get core.safecrlf 2>$null | Out-String).Trim()
    $safecrlfExit = $LASTEXITCODE
    if ($autocrlfExit -ne 0 -or $eolExit -ne 0 -or $safecrlfExit -ne 0 -or
        $autocrlf -ne "false" -or $eol -ne "lf" -or $safecrlf -ne "true") {
        throw "Managed CycloneDDS checkout EOL policy must be local core.autocrlf=false, core.eol=lf, core.safecrlf=true"
    }
}

function Remove-GeneratedDirectory {
    param([string]$Path, [string]$AllowedParent, [string]$ExpectedName)
    if (-not (Test-Path -LiteralPath $Path)) { return }
    $full = [IO.Path]::GetFullPath($Path)
    $parent = [IO.Path]::GetFullPath((Split-Path -Parent $full))
    if (-not $parent.Equals([IO.Path]::GetFullPath($AllowedParent), [StringComparison]::OrdinalIgnoreCase) -or
        (Split-Path -Leaf $full) -ne $ExpectedName) {
        throw "Refusing to remove generated directory outside its exact boundary: $full"
    }
    Remove-Item -LiteralPath $full -Recurse -Force
}

if (-not (Test-Path -LiteralPath $LockPath -PathType Leaf)) { throw "Missing source lock: $LockPath" }
$Lock = Get-Content -Raw -LiteralPath $LockPath | ConvertFrom-Json
$ExpectedRepository = "https://github.com/eclipse-cyclonedds/cyclonedds.git"
$ExpectedTag = "11.0.1"
$ExpectedCommit = "e54e991f75a3e67f8e628da3171122e36ea5b872"
$ExpectedTree = "56508d35826c362782fc8a388cad351a3d491f51"
if ($Lock.repository -ne $ExpectedRepository -or $Lock.tag -ne $ExpectedTag -or
    $Lock.commit -ne $ExpectedCommit -or $Lock.tree -ne $ExpectedTree -or
    $Lock.toolset -ne "v143" -or $Lock.msvc_runtime -ne "MultiThreadedDLL" -or
    $Lock.architecture -ne "x64" -or $Lock.configuration -ne "Release" -or
    -not $Lock.build_shared_libs -or $Lock.enable_security -or -not $Lock.build_idlc -or
    -not $Lock.install_system_runtime_libs_skip) {
    throw "CycloneDDS Windows source lock does not match the reviewed 11.0.1 build contract"
}

$SourceRoot = Resolve-AbsolutePathInput $SourceRoot (Join-Path $RepoRoot "third_party\toolchains\cyclonedds\$($Lock.commit)") "SourceRoot"
$BuildRoot = Resolve-AbsolutePathInput $BuildRoot (Join-Path $RepoRoot "third_party\build\cyclonedds-windows-x64-$($Lock.commit)") "BuildRoot"
$InstallRoot = Resolve-AbsolutePathInput $InstallRoot (Join-Path $RepoRoot "third_party\install\cyclonedds-windows-x64-$($Lock.commit)") "InstallRoot"
$SdkRoot = Resolve-AbsolutePathInput $SdkRoot (Join-Path $RepoRoot "third_party\sdk\cyclonedds-$($Lock.tag)-windows-x64") "SdkRoot"
$QuarantineRoot = "$SdkRoot.incoming"
$QuarantineStagingRoot = "$QuarantineRoot.staging"

function Assert-NoPathOverlap {
    param([hashtable]$Paths)
    function Get-CanonicalFuturePath {
        param([string]$Path)
        $full = [IO.Path]::GetFullPath($Path)
        $root = [IO.Path]::GetPathRoot($full)
        if (-not $root) { throw "Cannot determine preparation path root: $Path" }
        $relative = $full.Substring($root.Length)
        $segments = @($relative -split '[\\/]' | Where-Object { $_ })
        $current = $root
        for ($index = 0; $index -lt $segments.Count; $index++) {
            $candidate = Join-Path $current $segments[$index]
            if (-not (Test-Path -LiteralPath $candidate)) {
                for ($tailIndex = $index; $tailIndex -lt $segments.Count; $tailIndex++) {
                    $current = Join-Path $current $segments[$tailIndex]
                }
                break
            }
            $item = Get-Item -Force -LiteralPath $candidate
            if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                $resolved = $item.ResolveLinkTarget($true)
                if (-not $resolved) { throw "Cannot resolve preparation reparse point: $candidate" }
                $current = $resolved.FullName
            }
            else {
                $current = $item.FullName
            }
        }
        [IO.Path]::GetFullPath($current).TrimEnd('\', '/')
    }
    $names = @($Paths.Keys)
    for ($leftIndex = 0; $leftIndex -lt $names.Count; $leftIndex++) {
        for ($rightIndex = $leftIndex + 1; $rightIndex -lt $names.Count; $rightIndex++) {
            $leftName = $names[$leftIndex]
            $rightName = $names[$rightIndex]
            $left = Get-CanonicalFuturePath ([string]$Paths[$leftName])
            $right = Get-CanonicalFuturePath ([string]$Paths[$rightName])
            $leftPrefix = "$left$([IO.Path]::DirectorySeparatorChar)"
            $rightPrefix = "$right$([IO.Path]::DirectorySeparatorChar)"
            if ($left.Equals($right, [StringComparison]::OrdinalIgnoreCase) -or
                $leftPrefix.StartsWith($rightPrefix, [StringComparison]::OrdinalIgnoreCase) -or
                $rightPrefix.StartsWith($leftPrefix, [StringComparison]::OrdinalIgnoreCase)) {
                throw "Preparation paths must be distinct and non-overlapping: $leftName=$left, $rightName=$right"
            }
        }
    }
}
Assert-NoPathOverlap @{
    SourceRoot = $SourceRoot; BuildRoot = $BuildRoot; InstallRoot = $InstallRoot
    SdkRoot = $SdkRoot; QuarantineRoot = $QuarantineRoot; QuarantineStagingRoot = $QuarantineStagingRoot
}

foreach ($command in "git", "cmake", "pwsh", "tar") {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) { throw "$command is required" }
}
foreach ($file in $StageScript, $VerifyScript, $SmokeIdl) {
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Required build input is missing: $file" }
}

Write-Output "CycloneDDS source: $($Lock.repository) tag=$($Lock.tag) commit=$($Lock.commit) tree=$($Lock.tree)"
Write-Output "Toolchain: Visual Studio 17 2022, v143, x64, Release, /MD"
Write-Output "Build: shared core, security disabled, idlc enabled"
Write-Output "Source root: $SourceRoot"
Write-Output "Build root: $BuildRoot"
Write-Output "Install root: $InstallRoot"
Write-Output "SDK root: $SdkRoot"
if ($PreflightOnly) { return }

foreach ($unused in $SdkRoot, $QuarantineRoot, $QuarantineStagingRoot, $BuildRoot, $InstallRoot) {
    if (Test-Path -LiteralPath $unused) { throw "Refusing to overwrite existing preparation path: $unused" }
}
$normalizeRemote = { param($url) ($url.TrimEnd("/") -replace "\.git$", "").ToLowerInvariant() }

function Assert-ExactSourceCheckout {
    param([string]$Path)
    Assert-CheckoutEolPolicy $Path
    $remote = Get-GitText @("-C", $Path, "remote", "get-url", "origin")
    if ((& $normalizeRemote $remote) -ne (& $normalizeRemote $Lock.repository)) {
        throw "CycloneDDS origin mismatch: expected $($Lock.repository), found $remote"
    }
    $dirty = ((@(& git -C $Path status --porcelain --untracked-files=all)) -join "`n").Trim()
    if ($LASTEXITCODE -ne 0 -or $dirty) { throw "CycloneDDS checkout must be clean before any mutation: $dirty" }
    & git -C $Path symbolic-ref -q HEAD 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) { throw "CycloneDDS checkout must have a detached HEAD" }
    $commit = Get-GitText @("-C", $Path, "rev-parse", "HEAD")
    $tree = Get-GitText @("-C", $Path, "rev-parse", "HEAD^{tree}")
    $tagCommit = Get-GitText @("-C", $Path, "rev-parse", "refs/tags/$($Lock.tag)^{commit}")
    if ($commit -ne $Lock.commit -or $tree -ne $Lock.tree -or $tagCommit -ne $Lock.commit) {
        throw "CycloneDDS checkout identity mismatch: commit=$commit tree=$tree tag=$tagCommit"
    }
}

if (Test-Path -LiteralPath $SourceRoot) {
    if (-not (Test-Path -LiteralPath (Join-Path $SourceRoot ".git") -PathType Container)) {
        throw "SourceRoot exists but is not a managed git checkout: $SourceRoot"
    }
    # Existing checkouts are never fetched or checked out in place. They are accepted
    # only when already clean, detached and exactly content-addressed by the lock.
    Assert-ExactSourceCheckout $SourceRoot
}
else {
    $sourceParent = Split-Path -Parent $SourceRoot
    New-Item -ItemType Directory -Force -Path $sourceParent | Out-Null
    $cloneRoot = "$SourceRoot.clone-incoming-$([guid]::NewGuid().ToString('N'))"
    try {
        Invoke-Checked git @(
            "-c", "core.autocrlf=false", "-c", "core.eol=lf", "clone",
            "--filter=blob:none", "--no-checkout", $Lock.repository, $cloneRoot
        ) "CycloneDDS clone failed"
        Invoke-Checked git @("-C", $cloneRoot, "config", "--local", "core.autocrlf", "false") "Cannot pin checkout autocrlf policy"
        Invoke-Checked git @("-C", $cloneRoot, "config", "--local", "core.eol", "lf") "Cannot pin checkout EOL policy"
        Invoke-Checked git @("-C", $cloneRoot, "config", "--local", "core.safecrlf", "true") "Cannot pin checkout safecrlf policy"
        Invoke-Checked git @("-C", $cloneRoot, "fetch", "--depth", "1", "origin", "tag", $Lock.tag) "CycloneDDS tag fetch failed"
        Invoke-Checked git @("-C", $cloneRoot, "fetch", "--depth", "1", "origin", $Lock.commit) "CycloneDDS commit fetch failed"
        Invoke-Checked git @("-C", $cloneRoot, "checkout", "--detach", $Lock.commit) "CycloneDDS detached checkout failed"
        Assert-ExactSourceCheckout $cloneRoot
        [IO.Directory]::Move($cloneRoot, $SourceRoot)
    }
    finally {
        Remove-GeneratedDirectory $cloneRoot $sourceParent (Split-Path -Leaf $cloneRoot)
    }
}
New-Item -ItemType Directory -Force -Path $BuildRoot, $InstallRoot | Out-Null
$SnapshotRoot = Join-Path $BuildRoot "source-snapshot-$($Lock.tree)"
$ArchivePath = Join-Path $BuildRoot "source-$($Lock.tree).tar"
New-Item -ItemType Directory -Path $SnapshotRoot | Out-Null
try {
    Invoke-Checked git @(
        "-c", "core.autocrlf=false", "-c", "core.eol=lf", "-C", $SourceRoot,
        "archive", "--format=tar", "--output=$ArchivePath", $Lock.commit
    ) "CycloneDDS source archive failed"
    Invoke-Checked tar @("-xf", $ArchivePath, "-C", $SnapshotRoot) "CycloneDDS source snapshot extraction failed"
}
finally {
    if (Test-Path -LiteralPath $ArchivePath) { Remove-Item -LiteralPath $ArchivePath -Force }
}
foreach ($legalFile in "LICENSE", "NOTICE.md") {
    if (-not (Test-Path -LiteralPath (Join-Path $SnapshotRoot $legalFile) -PathType Leaf)) {
        throw "CycloneDDS source snapshot is missing $legalFile"
    }
}
Get-ChildItem -LiteralPath $SnapshotRoot -File -Recurse | ForEach-Object { $_.IsReadOnly = $true }

$configureArgs = @(
    "-S", $SnapshotRoot, "-B", $BuildRoot,
    "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "v143",
    "-DCMAKE_INSTALL_PREFIX=$InstallRoot", "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL",
    "-DCMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP=TRUE",
    "-DBUILD_SHARED_LIBS=ON", "-DENABLE_SECURITY=OFF", "-DBUILD_IDLC=ON",
    "-DBUILD_DDSPERF=OFF", "-DBUILD_EXAMPLES=OFF", "-DBUILD_TESTING=OFF", "-DINSTALL_PDB=OFF"
)
Invoke-Checked cmake $configureArgs "CycloneDDS configure failed"
$CachePath = Join-Path $BuildRoot "CMakeCache.txt"
$CacheText = Get-Content -Raw -LiteralPath $CachePath
function Assert-CMakeCacheValue {
    param([string]$Name, [string]$Expected)
    $match = [regex]::Match($CacheText, "(?m)^$([regex]::Escape($Name)):[^=]+=(.*)$")
    if (-not $match.Success -or $match.Groups[1].Value.Trim() -ne $Expected) {
        throw "CMake cache contract mismatch for $Name; expected '$Expected'"
    }
}
Assert-CMakeCacheValue "CMAKE_GENERATOR" "Visual Studio 17 2022"
Assert-CMakeCacheValue "CMAKE_GENERATOR_PLATFORM" "x64"
Assert-CMakeCacheValue "CMAKE_GENERATOR_TOOLSET" "v143"
Assert-CMakeCacheValue "CMAKE_MSVC_RUNTIME_LIBRARY" "MultiThreadedDLL"
Assert-CMakeCacheValue "CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP" "TRUE"
Assert-CMakeCacheValue "BUILD_SHARED_LIBS" "ON"
Assert-CMakeCacheValue "ENABLE_SECURITY" "OFF"
Assert-CMakeCacheValue "BUILD_IDLC" "ON"
Invoke-Checked cmake @("--build", $BuildRoot, "--config", "Release", "--parallel") "CycloneDDS build failed"
Invoke-Checked cmake @("--install", $BuildRoot, "--config", "Release") "CycloneDDS install failed"

$ReceiptRoot = Join-Path $BuildRoot "lingtu-receipts"
New-Item -ItemType Directory -Path $ReceiptRoot | Out-Null
$SourceReceipt = [ordered]@{
    schema_version = 1; repository = $Lock.repository; tag = $Lock.tag; tag_commit = $Lock.commit
    commit = $Lock.commit; tree = $Lock.tree; detached = $true; clean = $true
    checkout_eol_policy = "core.autocrlf=false;core.eol=lf;core.safecrlf=true"
}
$CmakeVersion = (& cmake --version | Select-Object -First 1).Trim()
$ToolchainReceipt = [ordered]@{
    schema_version = 1; generator = "Visual Studio 17 2022"; toolset = "v143"; architecture = "x64"
    configuration = "Release"; msvc_runtime = "/MD"; cmake = $CmakeVersion
    visual_studio_instance = ([regex]::Match($CacheText, "(?m)^CMAKE_GENERATOR_INSTANCE:[^=]+=(.*)$").Groups[1].Value.Trim())
    windows_sdk = ([regex]::Match($CacheText, "(?m)^CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION:[^=]+=(.*)$").Groups[1].Value.Trim())
}
$BuildReceipt = [ordered]@{
    schema_version = 1; build_shared_libs = $true; enable_security = $false; build_idlc = $true
    install_system_runtime_libs_skip = $true
    source_commit = $Lock.commit; source_tree = $Lock.tree
}
$SourceReceipt | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $ReceiptRoot "source.json") -Encoding utf8
$ToolchainReceipt | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $ReceiptRoot "toolchain.json") -Encoding utf8
$BuildReceipt | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $ReceiptRoot "build.json") -Encoding utf8

$StageParent = Split-Path -Parent $SdkRoot
New-Item -ItemType Directory -Force -Path $StageParent | Out-Null
try {
    Invoke-Checked cmake @(
        "-DINSTALL_ROOT=$InstallRoot", "-DSOURCE_ROOT=$SnapshotRoot", "-DRECEIPT_ROOT=$ReceiptRoot",
        "-DLOCK_PATH=$LockPath", "-DFINAL_SDK_ROOT=$SdkRoot", "-DEXPECTED_VERSION=$($Lock.tag)",
        "-P", $StageScript
    ) "CycloneDDS SDK staging failed"
    if (-not (Test-Path -LiteralPath $QuarantineRoot -PathType Container) -or (Test-Path -LiteralPath $SdkRoot)) {
        throw "SDK staging did not preserve the quarantine boundary"
    }
    Invoke-Checked pwsh @(
        "-NoProfile", "-File", $VerifyScript, "-SdkRoot", $QuarantineRoot,
        "-SmokeIdl", $SmokeIdl, "-CreateReceipt"
    ) "CycloneDDS SDK verification failed"
    $sdkReceiptPath = Join-Path $QuarantineRoot "evidence\sdk-receipt.json"
    if (-not (Test-Path -LiteralPath $sdkReceiptPath -PathType Leaf)) { throw "Verified SDK receipt was not produced" }
    $sdkReceipt = Get-Content -Raw -LiteralPath $sdkReceiptPath | ConvertFrom-Json
    if ($sdkReceipt.source.commit -ne $Lock.commit -or $sdkReceipt.source.tree -ne $Lock.tree -or
        $sdkReceipt.verification.result -ne "passed" -or $sdkReceipt.verification.dll_closure -ne "passed" -or
        $sdkReceipt.verification.consumer_compile_link -ne "passed") {
        throw "Verified SDK receipt does not bind the prepared artifact"
    }
    Assert-ExactSourceCheckout $SourceRoot
    [IO.Directory]::Move($QuarantineRoot, $SdkRoot)
}
catch {
    Remove-GeneratedDirectory $QuarantineRoot $StageParent (Split-Path -Leaf $QuarantineRoot)
    Remove-GeneratedDirectory $QuarantineStagingRoot $StageParent (Split-Path -Leaf $QuarantineStagingRoot)
    throw
}

Write-Output "Prepared verified CycloneDDS $($Lock.tag) Windows x64 SDK: $SdkRoot"
