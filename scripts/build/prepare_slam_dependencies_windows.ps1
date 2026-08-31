param(
    [string]$VcpkgRoot = "",
    [string]$InstallRoot = "",
    [string]$BinaryCache = "",
    [switch]$PreflightOnly
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$ManifestRoot = Join-Path $PSScriptRoot "vcpkg\slam-windows"
$ManifestPath = Join-Path $ManifestRoot "vcpkg.json"
$ExpectedBaseline = "9e593bb18ea69cc5095e012465dcd675a822ed0d"
$ExpectedRepository = "https://github.com/microsoft/vcpkg.git"
$Triplet = "x64-windows"

function Resolve-AbsoluteInputPath {
    param([string]$Value, [string]$Default, [string]$ParameterName)

    if ([string]::IsNullOrWhiteSpace($Value)) { $Value = $Default }
    if (-not [System.IO.Path]::IsPathFullyQualified($Value)) {
        throw "-$ParameterName must be an absolute path: $Value"
    }
    return [System.IO.Path]::GetFullPath($Value)
}

function Resolve-CanonicalPlannedPath {
    param([string]$Path)

    $FullPath = [System.IO.Path]::GetFullPath($Path)
    $RootPath = [System.IO.Path]::GetPathRoot($FullPath)
    if ([string]::IsNullOrWhiteSpace($RootPath)) { throw "Cannot resolve planned path safely: $Path" }
    $Canonical = $RootPath
    $Relative = $FullPath.Substring($RootPath.Length)
    foreach ($Part in @($Relative -split '[\\/]' | Where-Object { $_ })) {
        $Candidate = Join-Path $Canonical $Part
        if (Test-Path -LiteralPath $Candidate -PathType Container) {
            $Item = Get-Item -LiteralPath $Candidate -Force
            if (($Item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                $Target = $Item.ResolveLinkTarget($true)
                if (-not $Target) { throw "Cannot resolve reparse-point target safely: $Candidate" }
                $Canonical = $Target.FullName
            }
            else {
                $Canonical = (Resolve-Path -LiteralPath $Candidate).Path
            }
        }
        else {
            $Canonical = $Candidate
        }
    }
    return [System.IO.Path]::GetFullPath($Canonical)
}

function Test-PathOverlap {
    param([string]$First, [string]$Second)

    $FirstBoundary = $First.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    $SecondBoundary = $Second.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    return $First.Equals($Second, [System.StringComparison]::OrdinalIgnoreCase) -or
        $FirstBoundary.StartsWith($SecondBoundary, [System.StringComparison]::OrdinalIgnoreCase) -or
        $SecondBoundary.StartsWith($FirstBoundary, [System.StringComparison]::OrdinalIgnoreCase)
}

$VcpkgRoot = Resolve-AbsoluteInputPath $VcpkgRoot (Join-Path $Root "third_party\toolchains\vcpkg") "VcpkgRoot"
$InstallRoot = Resolve-AbsoluteInputPath $InstallRoot (Join-Path $Root "third_party\install\slam-windows") "InstallRoot"
$BinaryCache = Resolve-AbsoluteInputPath $BinaryCache (Join-Path $Root "third_party\cache\vcpkg") "BinaryCache"
if ($BinaryCache -match '[,;]') {
    throw "-BinaryCache must not contain a comma or semicolon: $BinaryCache"
}
$CanonicalVcpkgRoot = Resolve-CanonicalPlannedPath $VcpkgRoot
$CanonicalInstallRoot = Resolve-CanonicalPlannedPath $InstallRoot
$CanonicalBinaryCache = Resolve-CanonicalPlannedPath $BinaryCache
foreach ($Pair in @(
        @("VcpkgRoot", $CanonicalVcpkgRoot, "InstallRoot", $CanonicalInstallRoot),
        @("VcpkgRoot", $CanonicalVcpkgRoot, "BinaryCache", $CanonicalBinaryCache),
        @("InstallRoot", $CanonicalInstallRoot, "BinaryCache", $CanonicalBinaryCache))) {
    if (Test-PathOverlap $Pair[1] $Pair[3]) {
        throw "-$($Pair[0]) and -$($Pair[2]) must not overlap after canonical path resolution: $($Pair[1]) ; $($Pair[3])"
    }
}

if (-not (Test-Path -LiteralPath $ManifestPath -PathType Leaf)) {
    throw "Pinned vcpkg manifest is missing: $ManifestPath"
}
$Manifest = Get-Content -Raw -LiteralPath $ManifestPath | ConvertFrom-Json
if ($Manifest.'builtin-baseline' -ne $ExpectedBaseline) {
    throw "vcpkg baseline drift: expected $ExpectedBaseline, found $($Manifest.'builtin-baseline')"
}
$GitCommand = Get-Command git -ErrorAction SilentlyContinue
if (-not $GitCommand) {
    throw "git is required to prepare the pinned vcpkg toolchain."
}

function Assert-ManagedVcpkgCheckout {
    param([string]$CheckoutRoot)

    $ActualBaseline = (& $GitCommand.Source -C $CheckoutRoot rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0 -or $ActualBaseline -ne $ExpectedBaseline) {
        throw "vcpkg checkout drift: expected $ExpectedBaseline, found $ActualBaseline"
    }
    $ActualOrigin = (& $GitCommand.Source -C $CheckoutRoot remote get-url origin).Trim()
    if ($LASTEXITCODE -ne 0 -or $ActualOrigin -ne $ExpectedRepository) {
        throw "vcpkg origin drift: expected $ExpectedRepository, found $ActualOrigin"
    }
    & $GitCommand.Source -C $CheckoutRoot symbolic-ref -q HEAD 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) { throw "vcpkg checkout must be detached at $ExpectedBaseline" }
    $CheckoutDirty = ((@(& $GitCommand.Source -C $CheckoutRoot status --porcelain --untracked-files=normal)) -join "`n").Trim()
    if ($LASTEXITCODE -ne 0 -or $CheckoutDirty) { throw "vcpkg checkout must be clean: $CheckoutDirty" }
}

function Assert-VcpkgQuarantineBoundary {
    param([string]$QuarantineRoot)

    $ExpectedParent = [System.IO.Path]::GetFullPath((Split-Path -Parent $VcpkgRoot))
    $ActualParent = [System.IO.Path]::GetFullPath((Split-Path -Parent $QuarantineRoot))
    $ExpectedLeafPrefix = [System.IO.Path]::GetFileName($VcpkgRoot) + ".incoming.$PID."
    $ActualLeaf = [System.IO.Path]::GetFileName($QuarantineRoot)
    if (-not $ActualParent.Equals($ExpectedParent, [System.StringComparison]::OrdinalIgnoreCase) -or
        -not $ActualLeaf.StartsWith($ExpectedLeafPrefix, [System.StringComparison]::Ordinal) -or
        $ActualLeaf.Substring($ExpectedLeafPrefix.Length) -notmatch '^[0-9a-f]{32}$') {
        throw "Unsafe vcpkg checkout quarantine boundary: $QuarantineRoot"
    }
}

Write-Output "Dependency plan: vcpkg $ExpectedBaseline, target=$Triplet, host=$Triplet"
Write-Output "Manifest: $ManifestPath"
Write-Output "Toolchain: $VcpkgRoot"
Write-Output "Install root: $InstallRoot"
Write-Output "Local binary cache: $BinaryCache"
if ($PreflightOnly) { return }

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $VcpkgRoot) | Out-Null
New-Item -ItemType Directory -Force -Path $InstallRoot, $BinaryCache | Out-Null

if (Test-Path -LiteralPath (Join-Path $VcpkgRoot ".git") -PathType Container) {
    # Existing managed state is immutable input. Validate it before any network or checkout operation.
    Assert-ManagedVcpkgCheckout $VcpkgRoot
}
else {
    if (Test-Path -LiteralPath $VcpkgRoot) {
        throw "VcpkgRoot exists but is not a git checkout: $VcpkgRoot"
    }
    $QuarantineRoot = "$VcpkgRoot.incoming.$PID.$([Guid]::NewGuid().ToString('N'))"
    Assert-VcpkgQuarantineBoundary $QuarantineRoot
    $PublishedCheckout = $false
    try {
        & $GitCommand.Source clone --filter=blob:none --no-checkout $ExpectedRepository $QuarantineRoot
        if ($LASTEXITCODE -ne 0) { throw "vcpkg clone failed with exit code $LASTEXITCODE" }
        & $GitCommand.Source -C $QuarantineRoot fetch --depth 1 origin $ExpectedBaseline
        if ($LASTEXITCODE -ne 0) { throw "vcpkg baseline fetch failed with exit code $LASTEXITCODE" }
        & $GitCommand.Source -C $QuarantineRoot checkout --detach $ExpectedBaseline
        if ($LASTEXITCODE -ne 0) { throw "vcpkg baseline checkout failed with exit code $LASTEXITCODE" }
        Assert-ManagedVcpkgCheckout $QuarantineRoot
        [System.IO.Directory]::Move($QuarantineRoot, $VcpkgRoot)
        $PublishedCheckout = $true
    }
    finally {
        if (-not $PublishedCheckout -and (Test-Path -LiteralPath $QuarantineRoot)) {
            Assert-VcpkgQuarantineBoundary $QuarantineRoot
            Remove-Item -LiteralPath $QuarantineRoot -Recurse -Force
        }
    }
}

$Bootstrap = Join-Path $VcpkgRoot "bootstrap-vcpkg.bat"
if (-not (Test-Path -LiteralPath $Bootstrap -PathType Leaf)) {
    throw "Pinned vcpkg checkout is missing bootstrap-vcpkg.bat: $Bootstrap"
}
& $Bootstrap -disableMetrics
if ($LASTEXITCODE -ne 0) { throw "vcpkg bootstrap failed with exit code $LASTEXITCODE" }

$VcpkgExe = Join-Path $VcpkgRoot "vcpkg.exe"
if (-not (Test-Path -LiteralPath $VcpkgExe -PathType Leaf)) {
    throw "vcpkg bootstrap did not produce $VcpkgExe"
}
$OriginalBinarySources = $env:VCPKG_BINARY_SOURCES
$OriginalTargetTriplet = $env:VCPKG_DEFAULT_TRIPLET
$OriginalHostTriplet = $env:VCPKG_DEFAULT_HOST_TRIPLET
try {
    $env:VCPKG_BINARY_SOURCES = "clear;files,$BinaryCache,readwrite"
    $env:VCPKG_DEFAULT_TRIPLET = $Triplet
    $env:VCPKG_DEFAULT_HOST_TRIPLET = $Triplet
    & $VcpkgExe install `
        "--x-manifest-root=$ManifestRoot" `
        "--x-install-root=$InstallRoot" `
        "--triplet=$Triplet" `
        "--host-triplet=$Triplet" `
        --clean-after-build
    if ($LASTEXITCODE -ne 0) { throw "vcpkg install failed with exit code $LASTEXITCODE" }
}
finally {
    $env:VCPKG_BINARY_SOURCES = $OriginalBinarySources
    $env:VCPKG_DEFAULT_TRIPLET = $OriginalTargetTriplet
    $env:VCPKG_DEFAULT_HOST_TRIPLET = $OriginalHostTriplet
}

Assert-ManagedVcpkgCheckout $VcpkgRoot

Write-Output "Prepared Windows SLAM dependencies in $(Join-Path $InstallRoot $Triplet)"
