param(
    [Parameter(Mandatory = $true)]
    [string]$PluginDirectory,
    [Parameter(Mandatory = $true)]
    [string]$SampleDataDirectory
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$pluginSource = (Resolve-Path -LiteralPath $PluginDirectory).Path
$dataSource = (Resolve-Path -LiteralPath $SampleDataDirectory).Path
$pluginDescriptor = Join-Path $pluginSource 'LCC4Unreal.uplugin'
$sampleScene = Join-Path $dataSource 'lcc2\LCC2.lcc2'
$sampleMesh = Join-Path $dataSource 'lcc2\data\mesh'

if (-not (Test-Path -LiteralPath $pluginDescriptor -PathType Leaf)) {
    throw "LCC4Unreal.uplugin is missing from $pluginSource"
}
if (-not (Test-Path -LiteralPath $sampleScene -PathType Leaf)) {
    throw "The official LCC2 sample is missing from $dataSource"
}
if (-not (Test-Path -LiteralPath $sampleMesh -PathType Container)) {
    throw "The LCC2 collision mesh directory is missing from $dataSource"
}
if (-not (Get-ChildItem -LiteralPath $sampleMesh -Filter '*.ply' -File | Select-Object -First 1)) {
    throw "No LCC2 collision PLY files exist in $sampleMesh"
}

$pluginInfo = Get-Content -LiteralPath $pluginDescriptor -Raw -Encoding UTF8 | ConvertFrom-Json
if ($pluginInfo.VersionName -ne '3.4.0' -or $pluginInfo.EngineVersion -notlike '5.8*') {
    throw "Expected XGRIDS LCC4Unreal 3.4.0 for UE 5.8; got $($pluginInfo.VersionName) / $($pluginInfo.EngineVersion)"
}
$projectInfo = Get-Content -LiteralPath (Join-Path $projectRoot 'RobotSimUE.uproject') -Raw -Encoding UTF8 | ConvertFrom-Json
if (-not ($projectInfo.Plugins | Where-Object { $_.Name -eq 'LCC4Unreal' -and $_.Enabled })) {
    throw 'RobotSimUE.uproject does not enable LCC4Unreal'
}

$projectPrefix = $projectRoot.TrimEnd('\') + '\'
foreach ($source in @($pluginSource, $dataSource)) {
    if ($source.StartsWith($projectPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "A source must live outside RobotSimUE to avoid a recursive junction: $source"
    }
}

function Mount-Junction([string]$Destination, [string]$Source) {
    if (Test-Path -LiteralPath $Destination) {
        $existing = Get-Item -LiteralPath $Destination -Force
        if (-not ($existing.Attributes -band [System.IO.FileAttributes]::ReparsePoint)) {
            throw "Refusing to replace an existing non-junction directory: $Destination"
        }
        $target = [System.IO.Path]::GetFullPath([string]$existing.Target).TrimEnd('\')
        if (-not $target.Equals($Source.TrimEnd('\'), [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Existing junction points elsewhere: $Destination -> $target"
        }
        return
    }
    New-Item -ItemType Junction -Path $Destination -Target $Source | Out-Null
}

Mount-Junction (Join-Path $projectRoot 'Plugins\LCC4Unreal') $pluginSource
Mount-Junction (Join-Path $projectRoot 'Content\3DGSData') $dataSource

Write-Output "XGRIDS_PLUGIN=$(Join-Path $projectRoot 'Plugins\LCC4Unreal')"
Write-Output "XGRIDS_SAMPLE=$(Join-Path $projectRoot 'Content\3DGSData\lcc2\LCC2.lcc2')"
Write-Output 'XGRIDS_COLLISION_MESH=present'
