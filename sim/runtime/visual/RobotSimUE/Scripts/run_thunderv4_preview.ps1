param(
    [string]$UnrealRoot = 'D:\Program Files\Epic Games\UE_5.8',
    [string]$UnrealAutoSdkRoot = $env:UE_SDKS_ROOT,
    [string]$NetFxSdkRoot,
    [string]$BlenderExe = 'D:\Development\Blender\5.2\blender.exe',
    [string]$MujocoHost,
    [string]$MuJoCoRoot,
    [string]$SnapshotPath,
    [string]$DerivedDataCacheRoot,
    [string]$RobotPackagePath,
    [string]$RobotMjcfPath,
    [string]$VisualMeshRoot,
    [string]$FbxOutputDir,
    [ValidateRange(1, 2000000)]
    [int]$MaxTrianglesPerAsset = 60000,
    [ValidateRange(1, 20000000)]
    [int]$MaxTotalTriangles = 1100000,
    [switch]$SkipUnrealBuild,
    [switch]$RebuildMaterials,
    [ValidateRange(1, 1440)]
    [int]$UnrealBuildTimeoutMinutes = 90,
    [switch]$Unattended
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $projectRoot '..\..\..\..')).Path
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot 'build'))
$derivedDataCache = [System.IO.Path]::GetFullPath(
    $(if ($DerivedDataCacheRoot) {
        $DerivedDataCacheRoot
    } else {
        Join-Path $buildRoot 'unreal-ddc\thunderv4-preview'
    })
)
$buildPrefix = $buildRoot.TrimEnd([System.IO.Path]::DirectorySeparatorChar) + [System.IO.Path]::DirectorySeparatorChar
if (-not ($derivedDataCache + [System.IO.Path]::DirectorySeparatorChar).StartsWith(
    $buildPrefix,
    [System.StringComparison]::OrdinalIgnoreCase
)) {
    throw 'DerivedDataCacheRoot must remain inside the repository build root'
}
New-Item -ItemType Directory -Path $derivedDataCache -Force | Out-Null
$ddcWriteProbe = Join-Path $derivedDataCache (
    '.lingtu-write-probe-{0}-{1}.tmp' -f $PID, [System.Guid]::NewGuid().ToString('N')
)
$probePayload = [System.Text.Encoding]::UTF8.GetBytes('LingTu DDC write probe')
$probeStream = $null
try {
    $probeStream = [System.IO.File]::Open(
        $ddcWriteProbe,
        [System.IO.FileMode]::CreateNew,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::None
    )
    $probeStream.Write($probePayload, 0, $probePayload.Length)
    $probeStream.Flush($true)
    $probeStream.Dispose()
    $probeStream = $null
    if ((Get-Item -LiteralPath $ddcWriteProbe).Length -ne $probePayload.Length) {
        throw 'DDC write probe length did not match its payload'
    }
}
catch {
    throw "DerivedDataCacheRoot is not writable: $derivedDataCache ($($_.Exception.Message))"
}
finally {
    if ($null -ne $probeStream) {
        $probeStream.Dispose()
    }
    if (Test-Path -LiteralPath $ddcWriteProbe -PathType Leaf) {
        Remove-Item -LiteralPath $ddcWriteProbe -Force
    }
}
${env:UE-LocalDataCachePath} = $derivedDataCache
${env:UE-SharedDataCachePath} = 'None'
$project = Join-Path $projectRoot 'RobotSimUE.uproject'
$python = Join-Path $repoRoot '.venv\Scripts\python.exe'
$assetExporter = Join-Path $repoRoot 'sim\tools\assets\export_binary_stl_fbx.py'
$mjcf = if ($RobotMjcfPath) {
    [System.IO.Path]::GetFullPath($RobotMjcfPath)
} else {
    Join-Path $repoRoot 'sim\packages\robots\doso\thunder_v4\mjcf\thunderv4.xml'
}
$robotPackage = if ($RobotPackagePath) {
    [System.IO.Path]::GetFullPath($RobotPackagePath)
} else {
    Join-Path $repoRoot 'sim\packages\robots\doso\thunder_v4'
}
$visualMeshRoot = if ($VisualMeshRoot) {
    [System.IO.Path]::GetFullPath($VisualMeshRoot)
} else {
    $null
}
$physicsSource = Join-Path $repoRoot 'sim\runtime\physics'
$physicsBuild = Join-Path $repoRoot 'build\mujoco-runtime-physics-win'
$sessionSpec = Join-Path $repoRoot 'sim\sessions\examples\thunderv4_controlled_headless\session.yaml'
$sessionBundle = Join-Path $repoRoot 'build\runtime-session-preview'
$runtimeRuns = Join-Path $repoRoot 'build\runtime-runs'
$fbxDir = if ($FbxOutputDir) {
    [System.IO.Path]::GetFullPath($FbxOutputDir)
} else {
    Join-Path $repoRoot 'build\unreal-assets\thunderv4-mjcf-fbx'
}
$assetIndex = Join-Path $fbxDir 'asset-index.json'
$canonicalMujocoHost = Join-Path $physicsBuild 'Release\lingtu_mujoco_headless.exe'
$headlessExe = if ($MujocoHost) { $MujocoHost } else { $canonicalMujocoHost }
$snapshot = if ($SnapshotPath) {
    (Resolve-Path -LiteralPath $SnapshotPath).Path
} else {
    Join-Path $repoRoot 'build\unreal-assets\thunderv4.runtime.snapshot.json'
}
$recipe = Join-Path $repoRoot 'build\unreal-assets\thunderv4-nominal-stand.recipe.json'
$previewScript = Join-Path $PSScriptRoot 'build_thunderv4_preview.py'
$screenshot = Join-Path $repoRoot 'build\unreal-preview\thunderv4-runtime.png'
$successSentinel = Join-Path $repoRoot 'build\unreal-preview\thunderv4-runtime.success.json'
$errorSentinel = Join-Path $repoRoot 'build\unreal-preview\thunderv4-runtime.error.txt'
$unrealLog = Join-Path $repoRoot 'build\unreal-preview\thunderv4-runtime.Unreal.log'
$buildBat = Join-Path $UnrealRoot 'Engine\Build\BatchFiles\Build.bat'
$editor = Join-Path $UnrealRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
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

if (-not $UnrealAutoSdkRoot) {
    $localAutoSdk = 'D:\Development\UnrealAutoSDK'
    if (Test-Path -LiteralPath $localAutoSdk -PathType Container) {
        $UnrealAutoSdkRoot = $localAutoSdk
    }
}
if ($UnrealAutoSdkRoot) {
    if (-not (Test-Path -LiteralPath $UnrealAutoSdkRoot -PathType Container)) {
        throw "Unreal AutoSDK root is missing: $UnrealAutoSdkRoot"
    }
    $env:UE_SDKS_ROOT = (Resolve-Path -LiteralPath $UnrealAutoSdkRoot).Path
}

foreach ($required in @(
    $python,
    $BlenderExe,
    $mjcf,
    $sessionSpec,
    $buildBat,
    $editor,
    $ueBuildRunner
)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required tool or input is missing: $required"
    }
}
if (-not (Test-Path -LiteralPath $physicsSource -PathType Container)) {
    throw "Physics Runtime source is missing: $physicsSource"
}
if (-not (Test-Path -LiteralPath $robotPackage -PathType Container)) {
    throw "RobotPackage is missing: $robotPackage"
}
if ($visualMeshRoot -and -not (Test-Path -LiteralPath $visualMeshRoot -PathType Container)) {
    throw "Visual mesh root is missing: $visualMeshRoot"
}

if ($MujocoHost) {
    if (-not (Test-Path -LiteralPath $headlessExe -PathType Leaf)) {
        throw "Explicit MuJoCo host is missing: $headlessExe"
    }
}
if (-not (Test-Path -LiteralPath $headlessExe -PathType Leaf)) {
    $configureArgs = @(
        '-S', $physicsSource,
        '-B', $physicsBuild,
        '-DLINGTU_MUJOCO_RUNTIME_BUILD_TESTS=OFF',
        '-DLINGTU_MUJOCO_RUNTIME_BUILD_CONTRACT_TESTS=OFF',
        '-DLINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS=ON'
    )
    if ($MuJoCoRoot) {
        if (-not (Test-Path -LiteralPath $MuJoCoRoot -PathType Container)) {
            throw "MuJoCo SDK root is missing: $MuJoCoRoot"
        }
        $configureArgs += "-DMUJOCO_ROOT=$MuJoCoRoot"
    }
    & cmake @configureArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Canonical MuJoCo Runtime configure failed with exit code $LASTEXITCODE"
    }
    & cmake --build $physicsBuild --config Release --target lingtu_mujoco_headless
    if ($LASTEXITCODE -ne 0) {
        throw "Canonical MuJoCo Runtime build failed with exit code $LASTEXITCODE"
    }
    if (-not (Test-Path -LiteralPath $headlessExe -PathType Leaf)) {
        throw "Canonical MuJoCo host was not produced: $headlessExe"
    }
}

if (-not $SnapshotPath) {
    & $python -m sim.catalog resolve $sessionSpec `
        --repo-root $repoRoot `
        --output-dir $sessionBundle
    if ($LASTEXITCODE -ne 0) {
        throw "SessionBundle resolution failed with exit code $LASTEXITCODE"
    }

    & $python -m sim.runtime.coordinator $sessionBundle `
        --repo-root $repoRoot `
        --run-root $runtimeRuns `
        --mujoco-host $headlessExe `
        --steps 1 `
        --snapshot-out $snapshot
    if ($LASTEXITCODE -ne 0) {
        throw "Physics Runtime snapshot failed with exit code $LASTEXITCODE"
    }
} elseif (-not (Test-Path -LiteralPath $snapshot -PathType Leaf)) {
    throw "Explicit SimulationSnapshot is missing: $snapshot"
}

$fbxExportArguments = @(
    '--background',
    '--factory-startup',
    '--python', $assetExporter,
    '--',
    '--mjcf', $mjcf,
    '--output-dir', $fbxDir,
    '--max-triangles-per-asset', $MaxTrianglesPerAsset,
    '--max-total-triangles', $MaxTotalTriangles
)
if ($visualMeshRoot) {
    $fbxExportArguments += @('--source-mesh-root', $visualMeshRoot)
}
& $BlenderExe @fbxExportArguments
if ($LASTEXITCODE -ne 0) {
    throw "Blender FBX staging failed with exit code $LASTEXITCODE"
}
if (-not (Test-Path -LiteralPath $assetIndex -PathType Leaf)) {
    throw "Blender did not create the expected asset index: $assetIndex"
}

& $python -m sim.tools.assets.build_unreal_preview_recipe `
    --robot-package $robotPackage `
    --asset-index $assetIndex `
    --snapshot $snapshot `
    --instance-id thunder_01 `
    --destination-path /Game/RobotSim/Robots/ThunderV4/Meshes `
    --output $recipe
if ($LASTEXITCODE -ne 0) {
    throw "Thunder preview recipe staging failed with exit code $LASTEXITCODE"
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

$env:LINGTU_THUNDER_FBX_DIR = $fbxDir
$env:LINGTU_THUNDER_PREVIEW_RECIPE = $recipe
$env:LINGTU_THUNDER_SCREENSHOT = $screenshot
$env:LINGTU_THUNDER_PREVIEW_SUCCESS = $successSentinel
$env:LINGTU_THUNDER_PREVIEW_ERROR = $errorSentinel
$env:LINGTU_THUNDER_PREVIEW_UNATTENDED = if ($Unattended) { '1' } else { '0' }
$env:LINGTU_THUNDER_REBUILD_MATERIALS = if ($RebuildMaterials) { '1' } else { '0' }

foreach ($staleOutput in @($screenshot, $successSentinel, $errorSentinel, $unrealLog)) {
    if (Test-Path -LiteralPath $staleOutput -PathType Leaf) {
        Remove-Item -LiteralPath $staleOutput -Force
    }
}

$arguments = @(
    ('"{0}"' -f $project),
    ('-ExecCmds="py {0}"' -f $previewScript),
    '-NoCompile',
    '-DDC=LingTuPreview',
    ('-LocalDataCachePath="{0}"' -f $derivedDataCache),
    '-log',
    ('-abslog="{0}"' -f $unrealLog)
)
if ($Unattended) {
    $arguments += @('-unattended', '-nop4')
}
$windowStyle = if ($Unattended) { 'Hidden' } else { 'Normal' }
$editorProcess = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle $windowStyle -PassThru
$deadline = (Get-Date).AddMinutes(20)
while ((Get-Date) -lt $deadline -and -not (Test-Path -LiteralPath $successSentinel -PathType Leaf)) {
    if (Test-Path -LiteralPath $errorSentinel -PathType Leaf) {
        $previewError = Get-Content -LiteralPath $errorSentinel -Raw
        throw "Unreal preview Python failed: $previewError"
    }
    if ($editorProcess.HasExited) {
        throw "Unreal Editor exited before reporting preview success (exit $($editorProcess.ExitCode))"
    }
    Start-Sleep -Seconds 2
    $editorProcess.Refresh()
}

if (-not (Test-Path -LiteralPath $successSentinel -PathType Leaf)) {
    throw "Timed out waiting for the Unreal offline preview success sentinel: $successSentinel"
}
if (-not (Test-Path -LiteralPath $screenshot -PathType Leaf)) {
    throw "Unreal offline preview reported success without the screenshot: $screenshot"
}
if (-not (Test-Path -LiteralPath $unrealLog -PathType Leaf)) {
    throw "Unreal offline preview reported success without its owned log: $unrealLog"
}
$unrealLogText = Get-Content -LiteralPath $unrealLog -Raw
$writableStoreMatches = [System.Text.RegularExpressions.Regex]::Matches(
    $unrealLogText,
    'Using data cache path (?<path>.+?): Writable'
)
$writableStorePaths = @(
    $writableStoreMatches | ForEach-Object {
        $_.Groups['path'].Value.Replace('/', '\').TrimEnd('\')
    }
)
$expectedWritableStore = $derivedDataCache.Replace('/', '\').TrimEnd('\')
if (
    $writableStorePaths.Count -ne 1 -or
    -not $writableStorePaths[0].Equals(
        $expectedWritableStore,
        [System.StringComparison]::OrdinalIgnoreCase
    )
) {
    throw "Unreal preview selected unexpected writable DDC stores: $($writableStorePaths -join ', ')"
}
foreach ($forbiddenCacheMarker in @(
    'ZenLocal: Using',
    'ZenShared: Using',
    'Shared: Using',
    'Cloud: Using'
)) {
    if ($unrealLogText.IndexOf(
        $forbiddenCacheMarker,
        [System.StringComparison]::Ordinal
    ) -ge 0) {
        throw "Unreal preview activated a forbidden non-local DDC store: $forbiddenCacheMarker"
    }
}

Write-Output $screenshot
