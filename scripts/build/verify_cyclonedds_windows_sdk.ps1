param(
    [Parameter(Mandatory = $true)][string]$SdkRoot,
    [string]$SmokeIdl = "",
    [switch]$CreateReceipt
)

$ErrorActionPreference = "Stop"
$ConvertFromJsonCommand = Get-Command ConvertFrom-Json -ErrorAction Stop
if (-not $ConvertFromJsonCommand.Parameters.ContainsKey("DateKind")) {
    throw "verify_cyclonedds_windows_sdk.ps1 requires pwsh with ConvertFrom-Json -DateKind support"
}
function ConvertFrom-LingTuJson {
    param([Parameter(Mandatory = $true)][string]$Json)
    $Json | ConvertFrom-Json -DateKind String
}
$CanonicalLockPath = Join-Path $PSScriptRoot "locks\cyclonedds-windows-x64.json"
$CanonicalLock = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath $CanonicalLockPath)
$ExpectedRepository = [string]$CanonicalLock.repository
$ExpectedVersion = [string]$CanonicalLock.tag
$ExpectedCommit = [string]$CanonicalLock.commit
$ExpectedTree = [string]$CanonicalLock.tree

if (-not [System.IO.Path]::IsPathFullyQualified($SdkRoot)) { throw "-SdkRoot must be absolute: $SdkRoot" }
$SdkRoot = [System.IO.Path]::GetFullPath($SdkRoot)
if (-not (Test-Path -LiteralPath $SdkRoot -PathType Container)) { throw "SDK root does not exist: $SdkRoot" }

$required = @(
    "include\dds\dds.h", "lib\ddsc.lib", "bin\ddsc.dll", "bin\idlc.exe",
    "lib\cmake\CycloneDDS\CycloneDDSConfig.cmake", "licenses\LICENSE", "licenses\NOTICE.md",
    "evidence\source.json", "evidence\toolchain.json", "evidence\build.json",
    "evidence\source-lock.json", "evidence\files.sha256", "evidence\sbom.spdx.json"
)
foreach ($relative in $required) {
    if (-not (Test-Path -LiteralPath (Join-Path $SdkRoot $relative) -PathType Leaf)) {
        throw "CycloneDDS SDK is missing required file: $relative"
    }
}
$receiptPath = Join-Path $SdkRoot "evidence\sdk-receipt.json"
if ($CreateReceipt -and (Test-Path -LiteralPath $receiptPath)) {
    throw "Receipt creation requires a fresh SDK quarantine"
}
if (-not $CreateReceipt -and -not (Test-Path -LiteralPath $receiptPath -PathType Leaf)) {
    throw "Published SDK is missing evidence/sdk-receipt.json"
}

function Assert-PeX64 {
    param([string]$Path)
    $bytes = [System.IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -lt 70 -or $bytes[0] -ne 0x4D -or $bytes[1] -ne 0x5A) { throw "Expected a PE executable: $Path" }
    $peOffset = [BitConverter]::ToInt32($bytes, 0x3C)
    if ($peOffset -lt 0 -or $peOffset + 6 -gt $bytes.Length -or
        $bytes[$peOffset] -ne 0x50 -or $bytes[$peOffset + 1] -ne 0x45 -or
        $bytes[$peOffset + 2] -ne 0 -or $bytes[$peOffset + 3] -ne 0) { throw "Invalid PE header: $Path" }
    $machine = [BitConverter]::ToUInt16($bytes, $peOffset + 4)
    if ($machine -ne 0x8664) { throw "Expected PE x64 (0x8664), found 0x$($machine.ToString('x4')): $Path" }
}

function Test-MicrosoftRuntimeName {
    param([string]$Name)
    $Name -match '^(?i:api-ms-|ext-ms-|ucrtbase|msvcp|msvcr|vcruntime|concrt|concret|vcomp|vcamp|atl|mfc|mfcm|openmp|pgort|pgosweep|clang_rt)[a-z0-9_.-]*\.dll$'
}

Assert-PeX64 (Join-Path $SdkRoot "bin\ddsc.dll")
Assert-PeX64 (Join-Path $SdkRoot "bin\idlc.exe")
foreach ($dll in Get-ChildItem -LiteralPath $SdkRoot -Filter "*.dll" -File -Recurse) {
    $signature = Get-AuthenticodeSignature -LiteralPath $dll.FullName
    $company = $dll.VersionInfo.CompanyName
    if ((Test-MicrosoftRuntimeName $dll.Name) -or $company -match '(?i)Microsoft' -or
        $signature.SignerCertificate.Subject -match '(?i)Microsoft') {
        throw "SDK must not copy Microsoft runtime DLLs; install VC Redist separately: $($dll.FullName)"
    }
}

# Verify the external SHA-256 manifest first. It covers the SPDX document and all
# payload/evidence files; only the manifest itself is intentionally self-excluded.
$manifestPath = Join-Path $SdkRoot "evidence\files.sha256"
$manifestFiles = [System.Collections.Generic.Dictionary[string, string]]::new([System.StringComparer]::Ordinal)
foreach ($line in Get-Content -LiteralPath $manifestPath) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    $parts = $line -split "  ", 2
    if ($parts.Count -ne 2 -or $parts[0] -notmatch '^[0-9a-f]{64}$') { throw "Invalid SHA-256 manifest line: $line" }
    $relative = $parts[1] -replace '\\', '/'
    if ([System.IO.Path]::IsPathFullyQualified($relative) -or $relative -match '(^|/)\.\.(/|$)' -or
        $manifestFiles.ContainsKey($relative)) { throw "Unsafe or duplicate SHA-256 manifest path: $relative" }
    $file = Join-Path $SdkRoot ($relative -replace '/', '\')
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Manifest file is missing: $relative" }
    $actual = (Get-FileHash -Algorithm SHA256 -LiteralPath $file).Hash.ToLowerInvariant()
    if ($actual -ne $parts[0]) { throw "SDK file hash mismatch: $relative" }
    $manifestFiles.Add($relative, $actual)
}
$actualFiles = @(Get-ChildItem -LiteralPath $SdkRoot -File -Recurse | ForEach-Object {
    [System.IO.Path]::GetRelativePath($SdkRoot, $_.FullName).Replace('\', '/')
} | Where-Object { $_ -notin "evidence/files.sha256", "evidence/sdk-receipt.json" })
foreach ($relative in $actualFiles) {
    if (-not $manifestFiles.ContainsKey($relative)) { throw "SDK file is not covered by SHA-256 evidence: $relative" }
}
if ($manifestFiles.Count -ne $actualFiles.Count -or -not $manifestFiles.ContainsKey("evidence/sbom.spdx.json") -or
    $manifestFiles.ContainsKey("evidence/sdk-receipt.json")) {
    throw "SHA-256 evidence does not exactly cover the SDK and its SPDX document"
}

$sourceReceipt = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath (Join-Path $SdkRoot "evidence\source.json"))
$sourceLockPath = Join-Path $SdkRoot "evidence\source-lock.json"
$sourceLock = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath $sourceLockPath)
if ($sourceLock.repository -ne $ExpectedRepository -or $sourceLock.tag -ne $ExpectedVersion -or
    $sourceLock.commit -ne $ExpectedCommit -or $sourceLock.tree -ne $ExpectedTree -or
    -not $sourceLock.install_system_runtime_libs_skip -or
    $sourceReceipt.repository -ne $sourceLock.repository -or $sourceReceipt.tag -ne $sourceLock.tag -or
    $sourceReceipt.commit -ne $sourceLock.commit -or $sourceReceipt.tag_commit -ne $sourceLock.commit -or
    $sourceReceipt.tree -ne $sourceLock.tree -or -not $sourceReceipt.detached -or -not $sourceReceipt.clean) {
    throw "Invalid or inconsistent CycloneDDS source lock/receipt"
}
if ($sourceReceipt.checkout_eol_policy -ne "core.autocrlf=false;core.eol=lf;core.safecrlf=true") {
    throw "Invalid CycloneDDS checkout EOL policy receipt"
}
$toolchainReceipt = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath (Join-Path $SdkRoot "evidence\toolchain.json"))
if ($toolchainReceipt.generator -ne "Visual Studio 17 2022" -or $toolchainReceipt.toolset -ne "v143" -or
    $toolchainReceipt.architecture -ne "x64" -or $toolchainReceipt.configuration -ne "Release" -or
    $toolchainReceipt.msvc_runtime -ne "/MD") {
    throw "Invalid CycloneDDS toolchain receipt"
}
$buildReceipt = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath (Join-Path $SdkRoot "evidence\build.json"))
if (-not $buildReceipt.build_shared_libs -or $buildReceipt.enable_security -or -not $buildReceipt.build_idlc -or
    -not $buildReceipt.install_system_runtime_libs_skip -or
    $buildReceipt.source_commit -ne $sourceReceipt.commit -or $buildReceipt.source_tree -ne $sourceReceipt.tree) {
    throw "Invalid CycloneDDS build receipt"
}

function Assert-SdkReceipt {
    $sdkReceipt = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath $receiptPath)
    if ($sdkReceipt.schema_version -ne 1 -or $sdkReceipt.sdk.name -ne "CycloneDDS" -or
        $sdkReceipt.sdk.version -ne $ExpectedVersion -or $sdkReceipt.source.repository -ne $ExpectedRepository -or
        $sdkReceipt.source.tag -ne $ExpectedVersion -or $sdkReceipt.source.commit -ne $ExpectedCommit -or
        $sdkReceipt.source.tree -ne $ExpectedTree -or
        $sdkReceipt.toolchain.generator -ne "Visual Studio 17 2022" -or $sdkReceipt.toolchain.toolset -ne "v143" -or
        $sdkReceipt.toolchain.architecture -ne "x64" -or $sdkReceipt.toolchain.configuration -ne "Release" -or
        $sdkReceipt.toolchain.msvc_runtime -ne "/MD" -or
        $sdkReceipt.paths.license -ne "licenses/LICENSE" -or $sdkReceipt.paths.notice -ne "licenses/NOTICE.md" -or
        $sdkReceipt.paths.cmake_config -ne "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake" -or
        $sdkReceipt.paths.idlc -ne "bin/idlc.exe" -or $sdkReceipt.paths.ddsc_dll -ne "bin/ddsc.dll" -or
        $sdkReceipt.paths.ddsc_import_library -ne "lib/ddsc.lib" -or $sdkReceipt.verification.result -ne "passed" -or
        -not $sdkReceipt.verification.pe_x64 -or $sdkReceipt.verification.dll_closure -ne "passed" -or
        $sdkReceipt.verification.idl_smoke -ne "passed" -or $sdkReceipt.verification.consumer_compile_link -ne "passed" -or
        -not $sdkReceipt.verification.sanitized_dll_search) {
        throw "Invalid CycloneDDS SDK verification receipt"
    }
}
if (-not $CreateReceipt) { Assert-SdkReceipt }

# Strictly validate the SPDX 2.3 JSON emitted for this one-package SDK.
$spdx = ConvertFrom-LingTuJson (Get-Content -Raw -LiteralPath (Join-Path $SdkRoot "evidence\sbom.spdx.json"))
if ($spdx.spdxVersion -ne "SPDX-2.3" -or $spdx.SPDXID -ne "SPDXRef-DOCUMENT" -or
    $spdx.dataLicense -ne "CC0-1.0" -or $spdx.name -ne "cyclonedds-windows-x64-sdk" -or
    $spdx.documentNamespace -notmatch "^https://inovxio\.example/spdx/cyclonedds/$([regex]::Escape($ExpectedVersion))/$ExpectedCommit/[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$" -or
    $spdx.creationInfo.created -notmatch '^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$' -or
    @($spdx.creationInfo.creators).Count -lt 1 -or @($spdx.packages).Count -ne 1) {
    throw "Invalid SPDX 2.3 document metadata"
}
$package = @($spdx.packages)[0]
if ($package.SPDXID -ne "SPDXRef-Package-CycloneDDS" -or $package.name -ne "Eclipse Cyclone DDS" -or
    $package.versionInfo -ne $ExpectedVersion -or $package.downloadLocation -ne $ExpectedRepository -or
    -not $package.filesAnalyzed -or $package.licenseConcluded -ne "EPL-2.0 OR BSD-3-Clause" -or
    $package.licenseDeclared -ne "EPL-2.0 OR BSD-3-Clause" -or -not $package.copyrightText) {
    throw "Invalid SPDX CycloneDDS package"
}
$spdxNames = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
$spdxIds = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
$sha1Values = [System.Collections.Generic.List[string]]::new()
foreach ($fileRecord in @($spdx.files)) {
    if ($fileRecord.SPDXID -notmatch '^SPDXRef-File-\d+$' -or -not $spdxIds.Add([string]$fileRecord.SPDXID) -or
        $fileRecord.fileName -notmatch '^\./.+' -or -not $spdxNames.Add(([string]$fileRecord.fileName).Substring(2)) -or
        $fileRecord.licenseConcluded -ne "NOASSERTION" -or @($fileRecord.licenseInfoInFiles) -notcontains "NOASSERTION" -or
        -not $fileRecord.copyrightText) { throw "Invalid SPDX file record" }
    $relative = ([string]$fileRecord.fileName).Substring(2)
    $path = Join-Path $SdkRoot ($relative -replace '/', '\')
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "SPDX names a missing file: $relative" }
    $checksums = @{}
    foreach ($checksum in @($fileRecord.checksums)) {
        if ($checksums.ContainsKey([string]$checksum.algorithm)) { throw "Duplicate SPDX checksum algorithm: $relative" }
        $checksums[[string]$checksum.algorithm] = [string]$checksum.checksumValue
    }
    $sha1 = (Get-FileHash -Algorithm SHA1 -LiteralPath $path).Hash.ToLowerInvariant()
    $sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant()
    if ($checksums.SHA1 -ne $sha1 -or $checksums.SHA256 -ne $sha256) { throw "SPDX file checksum mismatch: $relative" }
    $sha1Values.Add($sha1)
}
$expectedSpdxFiles = @(Get-ChildItem -LiteralPath $SdkRoot -File -Recurse | ForEach-Object {
    [System.IO.Path]::GetRelativePath($SdkRoot, $_.FullName).Replace('\', '/')
} | Where-Object { $_ -notin "evidence/files.sha256", "evidence/sbom.spdx.json", "evidence/sdk-receipt.json" })
if ($spdxNames.Count -ne $expectedSpdxFiles.Count) { throw "SPDX file set size mismatch" }
foreach ($relative in $expectedSpdxFiles) {
    if (-not $spdxNames.Contains($relative)) { throw "SPDX omits package file: $relative" }
}
$sortedSha1 = @($sha1Values | Sort-Object)
$verificationBytes = [Text.Encoding]::ASCII.GetBytes($sortedSha1 -join "")
$verificationCode = [Convert]::ToHexString([Security.Cryptography.SHA1]::HashData($verificationBytes)).ToLowerInvariant()
if ($package.packageVerificationCode.packageVerificationCodeValue -ne $verificationCode) {
    throw "SPDX package verification code mismatch"
}
$relationships = @($spdx.relationships)
$describes = @($relationships | Where-Object {
    $_.spdxElementId -eq "SPDXRef-DOCUMENT" -and $_.relationshipType -eq "DESCRIBES" -and
    $_.relatedSpdxElement -eq "SPDXRef-Package-CycloneDDS"
})
if ($describes.Count -ne 1) { throw "SPDX must contain one document DESCRIBES package relationship" }
foreach ($fileId in $spdxIds) {
    $contains = @($relationships | Where-Object {
        $_.spdxElementId -eq "SPDXRef-Package-CycloneDDS" -and $_.relationshipType -eq "CONTAINS" -and
        $_.relatedSpdxElement -eq $fileId
    })
    if ($contains.Count -ne 1) { throw "SPDX package containment is incomplete for $fileId" }
}
if ($relationships.Count -ne $spdxIds.Count + 1) { throw "SPDX contains unexpected relationships" }

$vsWhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vsWhere -PathType Leaf)) { throw "Visual Studio 2022 vswhere.exe is required" }
$vsArgs = @("-latest", "-products", "*", "-version", "[17.0,18.0)", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64")
$vsInstall = (& $vsWhere @vsArgs -property installationPath).Trim()
$vsVersion = (& $vsWhere @vsArgs -property installationVersion).Trim()
if (-not $vsInstall -or -not $vsVersion.StartsWith("17.")) { throw "Visual Studio 2022 v143 x64 tools were not found" }
$dumpbin = Get-ChildItem -LiteralPath (Join-Path $vsInstall "VC\Tools\MSVC") -Filter "dumpbin.exe" -File -Recurse |
    Where-Object { $_.FullName -match '[\\/]bin[\\/]Hostx64[\\/]x64[\\/]dumpbin\.exe$' } |
    Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
if (-not $dumpbin) { throw "Visual Studio 2022 x64 dumpbin.exe was not found" }
$cmake = (Get-Command cmake -ErrorAction Stop).Source
$system32 = [Environment]::SystemDirectory
$redistRoots = [System.Collections.Generic.List[string]]::new()
if ($env:VCToolsRedistDir -and (Test-Path -LiteralPath $env:VCToolsRedistDir -PathType Container)) {
    $redistRoots.Add([IO.Path]::GetFullPath($env:VCToolsRedistDir))
}
$redistBase = Join-Path $vsInstall "VC\Redist\MSVC"
if (Test-Path -LiteralPath $redistBase -PathType Container) {
    Get-ChildItem -LiteralPath $redistBase -Directory -Recurse | Where-Object {
        $_.FullName -match '[\\/]x64[\\/]Microsoft\.[^\\]+\.(CRT|OpenMP)$'
    } | ForEach-Object { $redistRoots.Add($_.FullName) }
}

function Resolve-ApprovedImport {
    param([string]$Name)
    $sdkCandidate = Join-Path (Join-Path $SdkRoot "bin") $Name
    if (Test-Path -LiteralPath $sdkCandidate -PathType Leaf) { return [pscustomobject]@{ Kind = "Sdk"; Path = $sdkCandidate } }
    $systemCandidate = Join-Path $system32 $Name
    if (Test-Path -LiteralPath $systemCandidate -PathType Leaf) { return [pscustomobject]@{ Kind = "System"; Path = $systemCandidate } }
    if ($Name -match '^(?i:api-ms-|ext-ms-)') { return [pscustomobject]@{ Kind = "ApiSet"; Path = "" } }
    foreach ($root in $redistRoots) {
        $candidate = Join-Path $root $Name
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { return [pscustomobject]@{ Kind = "VCRedist"; Path = $candidate } }
    }
    throw "Unresolved or unapproved DLL import: $Name"
}

function Assert-ApprovedDllClosure {
    param([string[]]$Roots)
    $queue = [Collections.Generic.Queue[string]]::new()
    $visited = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach ($root in $Roots) { $queue.Enqueue([IO.Path]::GetFullPath($root)) }
    while ($queue.Count -gt 0) {
        $binary = $queue.Dequeue()
        if (-not $visited.Add($binary)) { continue }
        Assert-PeX64 $binary
        $output = (& $dumpbin /nologo /dependents $binary 2>&1 | Out-String)
        if ($LASTEXITCODE -ne 0) { throw "dumpbin dependency inspection failed for $binary" }
        $imports = @([regex]::Matches($output, '(?im)^\s+([A-Za-z0-9._-]+\.dll)\s*$') | ForEach-Object { $_.Groups[1].Value })
        if ($imports.Count -eq 0) { throw "No PE imports could be parsed for $binary" }
        foreach ($import in $imports) {
            $resolved = Resolve-ApprovedImport $import
            if ($resolved.Kind -in "Sdk", "VCRedist") { $queue.Enqueue([string]$resolved.Path) }
        }
    }
}

$sdkBinaries = @(Get-ChildItem -LiteralPath (Join-Path $SdkRoot "bin") -File | Where-Object { $_.Extension -in ".dll", ".exe" } | Select-Object -ExpandProperty FullName)
Assert-ApprovedDllClosure $sdkBinaries

$temp = Join-Path ([IO.Path]::GetTempPath()) ("lingtu-cyclonedds-sdk-verify-" + [guid]::NewGuid().ToString("N"))
$originalPath = $env:PATH
$idlSmokePassed = $false
try {
    New-Item -ItemType Directory -Path $temp | Out-Null
    # Neither idlc nor the consumer may resolve a dependency from the caller's PATH.
    $env:PATH = "$(Join-Path $SdkRoot 'bin');$system32"
    if ($SmokeIdl) {
        if (-not [IO.Path]::IsPathFullyQualified($SmokeIdl) -or -not (Test-Path -LiteralPath $SmokeIdl -PathType Leaf)) {
            throw "-SmokeIdl must name an existing absolute file"
        }
        $idlOut = Join-Path $temp "idl"
        New-Item -ItemType Directory -Path $idlOut | Out-Null
        & (Join-Path $SdkRoot "bin\idlc.exe") -l c -o $idlOut $SmokeIdl
        if ($LASTEXITCODE -ne 0 -or -not (Get-ChildItem -LiteralPath $idlOut -File | Where-Object { $_.Extension -in ".c", ".h" })) {
            throw "idlc smoke compile failed in sanitized DLL environment"
        }
        $idlSmokePassed = $true
    }
    $consumerSource = Join-Path $temp "consumer"
    $consumerBuild = Join-Path $temp "build"
    New-Item -ItemType Directory -Path $consumerSource | Out-Null
    @'
#include <dds/dds.h>
int main(void) {
  dds_entity_t domain = dds_create_domain(DDS_DOMAIN_DEFAULT, NULL);
  if (domain < 0) return 1;
  return dds_delete(domain) < 0;
}
'@ | Set-Content -LiteralPath (Join-Path $consumerSource "main.c") -Encoding utf8
    $consumerCmake = @'
cmake_minimum_required(VERSION 3.27)
project(cyclonedds_sdk_consumer LANGUAGES C)
find_package(CycloneDDS __CYCLONEDDS_VERSION__ CONFIG REQUIRED)
add_executable(cyclonedds_sdk_consumer main.c)
target_link_libraries(cyclonedds_sdk_consumer PRIVATE CycloneDDS::ddsc)
set_property(TARGET cyclonedds_sdk_consumer PROPERTY MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
'@
    $consumerCmake.Replace("__CYCLONEDDS_VERSION__", $ExpectedVersion) |
        Set-Content -LiteralPath (Join-Path $consumerSource "CMakeLists.txt") -Encoding utf8
    $cycloneDir = Join-Path $SdkRoot "lib\cmake\CycloneDDS"
    & $cmake -S $consumerSource -B $consumerBuild -G "Visual Studio 17 2022" -A x64 -T v143 `
        "-DCycloneDDS_DIR=$cycloneDir" "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL"
    if ($LASTEXITCODE -ne 0) { throw "CycloneDDS SDK consumer configure failed" }
    & $cmake --build $consumerBuild --config Release --parallel
    if ($LASTEXITCODE -ne 0) { throw "CycloneDDS SDK consumer compile/link failed" }
    $consumer = Get-ChildItem -LiteralPath $consumerBuild -Filter "cyclonedds_sdk_consumer.exe" -File -Recurse |
        Select-Object -First 1 -ExpandProperty FullName
    if (-not $consumer) { throw "CycloneDDS SDK consumer executable was not produced" }
    Assert-ApprovedDllClosure @($consumer)
}
finally {
    $env:PATH = $originalPath
    if (Test-Path -LiteralPath $temp) {
        $tempParent = [IO.Path]::GetFullPath((Split-Path -Parent $temp))
        $expectedParent = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')
        if (-not $tempParent.TrimEnd('\').Equals($expectedParent, [StringComparison]::OrdinalIgnoreCase) -or
            (Split-Path -Leaf $temp) -notmatch '^lingtu-cyclonedds-sdk-verify-[0-9a-f]{32}$') {
            throw "Refusing to remove verifier directory outside the managed temp boundary: $temp"
        }
        Remove-Item -LiteralPath $temp -Recurse -Force
    }
}

if (-not (Test-Path -LiteralPath $receiptPath -PathType Leaf)) {
    if (-not $CreateReceipt -or -not $idlSmokePassed) { throw "A new SDK receipt requires explicit creation and a successful IDL smoke input" }
    $receipt = [ordered]@{
        schema_version = 1
        sdk = [ordered]@{ name = "CycloneDDS"; version = $ExpectedVersion }
        source = [ordered]@{
            repository = $ExpectedRepository; tag = $ExpectedVersion; commit = $ExpectedCommit; tree = $ExpectedTree
        }
        toolchain = [ordered]@{
            generator = "Visual Studio 17 2022"; toolset = "v143"; architecture = "x64"
            configuration = "Release"; msvc_runtime = "/MD"
        }
        paths = [ordered]@{
            license = "licenses/LICENSE"; notice = "licenses/NOTICE.md"
            cmake_config = "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake"; idlc = "bin/idlc.exe"
            ddsc_dll = "bin/ddsc.dll"; ddsc_import_library = "lib/ddsc.lib"
        }
        verification = [ordered]@{
            result = "passed"; pe_x64 = $true; dll_closure = "passed"; idl_smoke = "passed"
            consumer_compile_link = "passed"; sanitized_dll_search = $true
        }
    }
    $receipt | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $receiptPath -Encoding utf8
}
Assert-SdkReceipt

Write-Output "Verified usable CycloneDDS $ExpectedVersion Windows x64 SDK: $SdkRoot"
