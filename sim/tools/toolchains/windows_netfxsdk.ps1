[CmdletBinding()]
param(
    [string]$NetFxSdkRoot,
    [switch]$AllowMissing
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$headerRelativePath = 'Include\um\mscoree.h'
$minimumNetFxSdkVersion = [Version]'4.6'
$candidates = [System.Collections.Generic.List[string]]::new()

function Get-NetFxSdkVersion {
    param([string]$Path)

    $leaf = Split-Path -Leaf $Path.TrimEnd('\', '/')
    $match = [regex]::Match($leaf, '^v?(\d+(?:\.\d+){1,3})$', 'IgnoreCase')
    if (-not $match.Success) {
        return $null
    }
    return [Version]$match.Groups[1].Value
}

function Add-NetFxSdkCandidate {
    param([string]$Path)

    if ($Path) {
        $candidates.Add($Path)
    }
}

if ($NetFxSdkRoot) {
    Add-NetFxSdkCandidate -Path $NetFxSdkRoot
} elseif ($env:LINGTU_NETFXSDK_ROOT) {
    Add-NetFxSdkCandidate -Path $env:LINGTU_NETFXSDK_ROOT
} else {
    foreach ($registryPath in @(
        'HKLM:\SOFTWARE\Microsoft\Microsoft SDKs\NETFXSDK',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\NETFXSDK',
        'HKCU:\SOFTWARE\Microsoft\Microsoft SDKs\NETFXSDK',
        'HKCU:\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\NETFXSDK'
    )) {
        foreach ($versionKey in @(Get-ChildItem -LiteralPath $registryPath -ErrorAction SilentlyContinue)) {
            $properties = Get-ItemProperty -LiteralPath $versionKey.PSPath -ErrorAction SilentlyContinue
            Add-NetFxSdkCandidate -Path $properties.KitsInstallationFolder
            Add-NetFxSdkCandidate -Path $properties.InstallationFolder
        }
    }
    foreach ($drive in @(Get-PSDrive -PSProvider FileSystem)) {
        Add-NetFxSdkCandidate -Path (Join-Path $drive.Root 'Program Files (x86)\Windows Kits\NETFXSDK')
        Add-NetFxSdkCandidate -Path (Join-Path $drive.Root 'Program Files\Windows Kits\NETFXSDK')
        Add-NetFxSdkCandidate -Path (Join-Path $drive.Root 'Windows Kits\NETFXSDK')
    }
}

$resolvedNetFxSdkRoot = $null
$rejectedVersion = $null
foreach ($candidate in @($candidates | Select-Object -Unique)) {
    if (Test-Path -LiteralPath (Join-Path $candidate $headerRelativePath) -PathType Leaf) {
        $candidateVersion = Get-NetFxSdkVersion -Path $candidate
        if ($candidateVersion -and $candidateVersion -ge $minimumNetFxSdkVersion) {
            $resolvedNetFxSdkRoot = (Resolve-Path -LiteralPath $candidate).Path
            break
        }
        $rejectedVersion = $candidateVersion
    }
    if (-not (Test-Path -LiteralPath $candidate -PathType Container)) {
        continue
    }
    $versionRoot = Get-ChildItem -LiteralPath $candidate -Directory -ErrorAction SilentlyContinue |
        Where-Object {
            $version = Get-NetFxSdkVersion -Path $_.FullName
            $version -and
                $version -ge $minimumNetFxSdkVersion -and
                (Test-Path -LiteralPath (Join-Path $_.FullName $headerRelativePath) -PathType Leaf)
        } |
        Sort-Object @{ Expression = { Get-NetFxSdkVersion -Path $_.FullName }; Descending = $true } |
        Select-Object -First 1
    if ($versionRoot) {
        $resolvedNetFxSdkRoot = $versionRoot.FullName
        break
    }
}

if (-not $resolvedNetFxSdkRoot) {
    if ($AllowMissing) {
        return
    }
    $candidateExplanation = if ($NetFxSdkRoot -and $rejectedVersion) {
        "NETFXSDK $rejectedVersion is too old; $minimumNetFxSdkVersion or newer is required: $NetFxSdkRoot. "
    } elseif ($NetFxSdkRoot) {
        "The explicit NETFXSDK root does not contain $headerRelativePath`: $NetFxSdkRoot. "
    } else {
        'No registered or installed NETFXSDK root contains Include\um\mscoree.h. '
    }
    throw (
        'Unreal C++ build prerequisite is missing: .NET Framework SDK 4.6 or newer. ' +
        $candidateExplanation +
        'Install the Visual Studio .NET Framework 4.8 SDK + Targeting Pack components.'
    )
}

Write-Output $resolvedNetFxSdkRoot
