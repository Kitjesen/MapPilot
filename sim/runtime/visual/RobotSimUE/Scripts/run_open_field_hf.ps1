param(
    [string]$UnrealRoot = 'D:\Program Files\Epic Games\UE_5.8',
    [string]$NetFxSdkRoot,
    [string]$BlenderExe = 'D:\Development\Blender\5.2\blender.exe',
    [string]$EvidenceRoot,
    [string]$RobotRecipe,
    [string]$FbxDirectory,
    [switch]$SkipUnrealBuild,
    [switch]$Unattended,
    [ValidateRange(1, 1440)]
    [int]$UnrealBuildTimeoutMinutes = 90,
    [int]$TimeoutMinutes = 30,
    [int]$NoProgressMinutes = 8
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$errorSentinel = $null
$unrealLog = $null
$editorProcess = $null

function Write-LauncherErrorSentinel {
    param([string]$Message)

    if (-not $errorSentinel) {
        return
    }
    if (Test-Path -LiteralPath $errorSentinel -PathType Leaf) {
        return
    }
    $parent = Split-Path -Parent $errorSentinel
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
    $payload = [ordered]@{
        schema = 'lingtu.sim.unreal-open-field-hf-launcher-error.v1'
        mode = 'editor_high_fidelity_increment'
        message = $Message
        log = $unrealLog
    }
    $json = ($payload | ConvertTo-Json -Depth 4) + [Environment]::NewLine
    [System.IO.File]::WriteAllText(
        $errorSentinel,
        $json,
        [System.Text.UTF8Encoding]::new($false)
    )
}

trap {
    $failureMessage = $_.Exception.Message
    if (
        $Unattended -and
        $null -ne $editorProcess -and
        -not $editorProcess.HasExited
    ) {
        Stop-Process -Id $editorProcess.Id -Force -ErrorAction SilentlyContinue
    }
    Write-LauncherErrorSentinel -Message $failureMessage
    [Console]::Error.WriteLine($failureMessage)
    exit 1
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $projectRoot '..\..\..\..')).Path
$project = Join-Path $projectRoot 'RobotSimUE.uproject'
$editor = Join-Path $UnrealRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$buildBat = Join-Path $UnrealRoot 'Engine\Build\BatchFiles\Build.bat'
$python = Join-Path $repoRoot '.venv\Scripts\python.exe'
$netFxSdkPreflight = Join-Path $repoRoot 'sim\tools\toolchains\windows_netfxsdk.ps1'
$ueBuildRunner = Join-Path $repoRoot 'sim\tools\toolchains\ue_build.py'
if (-not $SkipUnrealBuild) {
    if (-not (Test-Path -LiteralPath $netFxSdkPreflight -PathType Leaf)) {
        throw "NETFXSDK preflight is missing: $netFxSdkPreflight"
    }
    if ($NetFxSdkRoot) {
        & $netFxSdkPreflight -NetFxSdkRoot $NetFxSdkRoot | Out-Null
    } else {
        & $netFxSdkPreflight | Out-Null
    }
}
$pythonScript = Join-Path $PSScriptRoot 'build_open_field_hf.py'
$worldRecipe = Join-Path $repoRoot 'sim\packages\worlds\open_field_hf\visual\ue_import.recipe.json'
$worldManifest = Join-Path $repoRoot 'sim\packages\worlds\open_field_hf\generated\asset-manifest.json'
$heightfield = Join-Path $repoRoot 'sim\packages\worlds\open_field_hf\generated\heightfield_r16.png'
$terrainObj = Join-Path $repoRoot 'sim\packages\worlds\open_field_hf\generated\terrain.obj'
$resolvedEvidenceRoot = if ($EvidenceRoot) {
    [System.IO.Path]::GetFullPath($EvidenceRoot)
} else {
    Join-Path $repoRoot 'build\unreal-open-field-hf'
}
$allowedEvidenceRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $repoRoot 'build\unreal-open-field-hf')
)
if (-not $resolvedEvidenceRoot.StartsWith(
    $allowedEvidenceRoot,
    [System.StringComparison]::OrdinalIgnoreCase
)) {
    throw "EvidenceRoot must stay under $allowedEvidenceRoot"
}
$resolvedRobotRecipe = if ($RobotRecipe) {
    (Resolve-Path -LiteralPath $RobotRecipe).Path
} else {
    Join-Path $repoRoot 'build\unreal-assets\thunderv4-runtime.recipe.json'
}
$resolvedFbxDirectory = if ($FbxDirectory) {
    (Resolve-Path -LiteralPath $FbxDirectory).Path
} else {
    Join-Path $repoRoot 'build\unreal-assets\thunderv4-mjcf-fbx'
}
$screenshot = Join-Path $resolvedEvidenceRoot 'OpenField_HF_1920x1080.png'
$successSentinel = Join-Path $resolvedEvidenceRoot 'OpenField_HF.success.json'
$errorSentinel = Join-Path $resolvedEvidenceRoot 'OpenField_HF.error.txt'
$unrealLog = Join-Path $resolvedEvidenceRoot 'UnrealEditor.log'

$requiredFiles = @(
    $editor,
    $BlenderExe,
    $project,
    $pythonScript,
    $worldRecipe,
    $worldManifest,
    $heightfield,
    $terrainObj,
    $resolvedRobotRecipe
)
if (-not $SkipUnrealBuild) {
    $requiredFiles += @($buildBat, $python, $ueBuildRunner)
}
foreach ($required in $requiredFiles) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required OpenField_HF tool or input is missing: $required"
    }
}
if (-not (Test-Path -LiteralPath $resolvedFbxDirectory -PathType Container)) {
    throw "ThunderV4 FBX staging directory is missing: $resolvedFbxDirectory"
}

$editorVersion = (Get-Item -LiteralPath $editor).VersionInfo.FileVersion
$blenderOutput = & $BlenderExe --version
$blenderExitCode = $LASTEXITCODE
$blenderVersion = ($blenderOutput | Select-Object -First 1)
if ($blenderExitCode -ne 0) {
    throw "Blender validation failed with exit code $blenderExitCode"
}
if (-not $editorVersion) {
    throw "Could not read Unreal Editor version from $editor"
}
if (-not $blenderVersion) {
    throw "Could not read Blender version from $BlenderExe"
}

$worldRecipeJson = Get-Content -Raw -LiteralPath $worldRecipe | ConvertFrom-Json
if ($worldRecipeJson.schema -ne 'lingtu.sim.unreal-world-import-recipe.v1') {
    throw "Unsupported OpenField_HF Unreal recipe schema: $($worldRecipeJson.schema)"
}
if ($worldRecipeJson.target_level -ne '/Game/RobotSim/Maps/OpenField_HF') {
    throw "OpenField_HF recipe targets the wrong map: $($worldRecipeJson.target_level)"
}
if ($worldRecipeJson.blender_mesh_import.import_scale -ne 1.0) {
    throw 'OpenField_HF terrain must be imported at exactly 1.0 scale'
}
$declaredTerrain = Join-Path $repoRoot (
    $worldRecipeJson.sources.terrain_obj.path -replace '/', '\'
)
if (
    [System.IO.Path]::GetFullPath($declaredTerrain) -ne
    [System.IO.Path]::GetFullPath($terrainObj)
) {
    throw "OpenField_HF recipe terrain path does not match $terrainObj"
}
$actualTerrainDigest = (
    Get-FileHash -LiteralPath $terrainObj -Algorithm SHA256
).Hash.ToLowerInvariant()
if ($actualTerrainDigest -ne $worldRecipeJson.sources.terrain_obj.sha256) {
    throw "OpenField_HF terrain SHA256 mismatch: $actualTerrainDigest"
}

$robotRecipeJson = Get-Content -Raw -LiteralPath $resolvedRobotRecipe | ConvertFrom-Json
if ($robotRecipeJson.schema -ne 'lingtu.sim.unreal-preview-recipe.v1') {
    throw "Unsupported ThunderV4 visual recipe: $resolvedRobotRecipe"
}
if (
    @($robotRecipeJson.bodies).Count -ne 21 -or
    @($robotRecipeJson.components).Count -ne 21
) {
    throw 'ThunderV4 visual recipe must contain 21 bodies and 21 components'
}
foreach ($component in $robotRecipeJson.components) {
    $fbx = Join-Path $resolvedFbxDirectory "$($component.asset_name).fbx"
    if (-not (Test-Path -LiteralPath $fbx -PathType Leaf)) {
        throw "ThunderV4 FBX staging is incomplete: $fbx"
    }
}

New-Item -ItemType Directory -Path $resolvedEvidenceRoot -Force | Out-Null
foreach ($staleOutput in @(
    $screenshot,
    $successSentinel,
    $errorSentinel,
    $unrealLog
)) {
    $fullStalePath = [System.IO.Path]::GetFullPath($staleOutput)
    if (-not $fullStalePath.StartsWith(
        $allowedEvidenceRoot,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Refusing to remove output outside $allowedEvidenceRoot"
    }
    if (Test-Path -LiteralPath $fullStalePath -PathType Leaf) {
        Remove-Item -LiteralPath $fullStalePath -Force
    }
}

if (-not $SkipUnrealBuild) {
    $buildTimeoutSeconds = [double]$UnrealBuildTimeoutMinutes * 60.0
    & $python $ueBuildRunner `
        --build-bat $buildBat `
        --uproject $project `
        --target RobotSimUEEditor `
        --platform Win64 `
        --configuration Development `
        --timeout-seconds $buildTimeoutSeconds
    if ($LASTEXITCODE -ne 0) {
        throw "RobotSimUEEditor build failed with exit code $LASTEXITCODE"
    }
}

$env:LINGTU_OPEN_FIELD_HF_RECIPE = $worldRecipe
$env:LINGTU_OPEN_FIELD_HF_ROBOT_RECIPE = $resolvedRobotRecipe
$env:LINGTU_OPEN_FIELD_HF_FBX_DIR = $resolvedFbxDirectory
$env:LINGTU_OPEN_FIELD_HF_EVIDENCE_DIR = $resolvedEvidenceRoot
$env:LINGTU_OPEN_FIELD_HF_SCREENSHOT = $screenshot
$env:LINGTU_OPEN_FIELD_HF_SUCCESS = $successSentinel
$env:LINGTU_OPEN_FIELD_HF_ERROR = $errorSentinel
$env:LINGTU_OPEN_FIELD_HF_UNATTENDED = if ($Unattended) { '1' } else { '0' }

$arguments = @(
    $project,
    "-ExecCmds=`"py $pythonScript`"",
    '-NoCompile',
    '-NoSplash',
    '-nop4',
    '-DDC-ForceMemoryCache',
    '-log',
    "-abslog=`"$unrealLog`""
)
if ($Unattended) {
    $arguments += '-unattended'
}
$windowStyle = if ($Unattended) { 'Hidden' } else { 'Normal' }
$editorProcess = Start-Process `
    -FilePath $editor `
    -ArgumentList $arguments `
    -WindowStyle $windowStyle `
    -PassThru

$deadline = (Get-Date).AddMinutes($TimeoutMinutes)
$lastProgressAt = Get-Date
$lastLogWriteTime = [DateTime]::MinValue
while (
    (Get-Date) -lt $deadline -and
    -not (Test-Path -LiteralPath $successSentinel -PathType Leaf)
) {
    if (Test-Path -LiteralPath $unrealLog -PathType Leaf) {
        $logItem = Get-Item -LiteralPath $unrealLog
        if ($logItem.LastWriteTime -gt $lastLogWriteTime) {
            $lastLogWriteTime = $logItem.LastWriteTime
            $lastProgressAt = Get-Date
        }
        $materialCompileFailure = Select-String `
            -LiteralPath $unrealLog `
            -Pattern 'M_OpenField_HF_Terrain.*Failed to compile Material|Failed to compile Material.*M_OpenField_HF_Terrain' `
            -ErrorAction SilentlyContinue
        if ($materialCompileFailure) {
            throw 'OpenField_HF terrain material failed to compile; see UnrealEditor.log'
        }
    }
    if (Test-Path -LiteralPath $errorSentinel -PathType Leaf) {
        $previewError = Get-Content -LiteralPath $errorSentinel -Raw
        throw "OpenField_HF Unreal Python failed: $previewError"
    }
    if ($editorProcess.HasExited) {
        throw (
            "Unreal Editor exited before OpenField_HF success " +
            "(exit $($editorProcess.ExitCode)); log=$unrealLog"
        )
    }
    if ((Get-Date) -gt $lastProgressAt.AddMinutes($NoProgressMinutes)) {
        throw (
            "Unreal Editor made no OpenField_HF log progress for " +
            "$NoProgressMinutes minutes; log=$unrealLog"
        )
    }
    Start-Sleep -Seconds 2
    $editorProcess.Refresh()
}

if (-not (Test-Path -LiteralPath $successSentinel -PathType Leaf)) {
    throw "Timed out waiting for OpenField_HF success sentinel: $successSentinel"
}
if (-not (Test-Path -LiteralPath $screenshot -PathType Leaf)) {
    throw "OpenField_HF reported success without screenshot: $screenshot"
}
if ($Unattended -and -not $editorProcess.HasExited) {
    $null = $editorProcess.WaitForExit(60000)
}

Write-Output "UnrealEditor=$editorVersion"
Write-Output "Blender=$blenderVersion"
Write-Output $screenshot
