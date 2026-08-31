param(
    [string]$UnrealRoot = 'D:\Program Files\Epic Games\UE_5.8',
    [string]$EvidenceRoot,
    [string]$DerivedDataCacheRoot,
    [ValidateRange(1, 30)]
    [int]$TimeoutMinutes = 12
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $projectRoot '..\..\..\..')).Path
$project = Join-Path $projectRoot 'RobotSimUE.uproject'
$compiler = Join-Path $PSScriptRoot 'compile_factory_park_visual_instances.py'
$editor = Join-Path $UnrealRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$runId = 'factory-park-instances-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
$resolvedEvidenceRoot = [System.IO.Path]::GetFullPath($(if ($EvidenceRoot) {
    $EvidenceRoot
} else {
    Join-Path $repoRoot "build\factory-park-hf\unreal-instance-compiler\$runId"
}))
$resolvedDdcRoot = [System.IO.Path]::GetFullPath($(if ($DerivedDataCacheRoot) {
    $DerivedDataCacheRoot
} else {
    Join-Path $resolvedEvidenceRoot 'ddc'
}))
$success = Join-Path $resolvedEvidenceRoot 'FactoryPark_HF.instances.success.json'
$stageSentinel = Join-Path $resolvedEvidenceRoot 'FactoryPark_HF.instances.stage.json'
$launchRecord = Join-Path $resolvedEvidenceRoot 'launch.json'

foreach ($required in @($editor, $project, $compiler)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "required file is missing: $required"
    }
}
$preflightNames = @(
    'UnrealEditor',
    'UnrealBuildTool',
    'AutomationTool',
    'MuJoCo',
    'mujoco_headless',
    'dotnet',
    'cl',
    'link',
    'msbuild'
)
$preflightProcesses = @(
    Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $preflightNames -contains $_.ProcessName }
)
if ($preflightProcesses.Count -ne 0) {
    $summary = ($preflightProcesses | ForEach-Object { "$($_.ProcessName):$($_.Id)" }) -join ', '
    throw "FactoryPark instance compiler preflight is not clean: $summary"
}
New-Item -ItemType Directory -Path $resolvedEvidenceRoot -Force | Out-Null
New-Item -ItemType Directory -Path $resolvedDdcRoot -Force | Out-Null

${env:UE-LocalDataCachePath} = $resolvedDdcRoot
${env:UE-SharedDataCachePath} = 'None'
$env:UE_SKIP_UBT_SDK_SETUP = '1'
$env:LINGTU_FACTORY_PARK_INSTANCE_EVIDENCE_ROOT = $resolvedEvidenceRoot
$env:LINGTU_FACTORY_PARK_INSTANCE_SUCCESS = $success
$env:LINGTU_FACTORY_PARK_INSTANCE_STAGE = $stageSentinel
$env:LINGTU_FACTORY_PARK_INSTANCE_UNATTENDED = '1'

$phaseLaunches = @(
    foreach ($phase in @('stage', 'publish')) {
        $phaseLog = Join-Path $resolvedEvidenceRoot "Unreal-$phase.log"
        $phaseError = Join-Path $resolvedEvidenceRoot "FactoryPark_HF.instances.$phase.error.json"
        [ordered]@{
            phase = $phase
            log = $phaseLog
            error = $phaseError
            arguments = @(
                $project,
                "-ExecCmds=py $compiler",
                '-NoCompile',
                '-NoSplash',
                '-nop4',
                '-DDC-ForceMemoryCache',
                '-unattended',
                '-NoSound',
                '-log',
                "-abslog=$phaseLog"
            )
        }
    }
)
$launchPayload = [ordered]@{
    schema = 'lingtu.sim.factory-park-visual-instance-launch.v1'
    run_id = $runId
    editor = $editor
    project = $project
    phases = $phaseLaunches
    evidence_root = $resolvedEvidenceRoot
    ddc_root = $resolvedDdcRoot
    stage_evidence = $stageSentinel
    final_evidence = $success
    active_build_forbidden = $true
}
[System.IO.File]::WriteAllText(
    $launchRecord,
    (($launchPayload | ConvertTo-Json -Depth 5) + [Environment]::NewLine),
    [System.Text.UTF8Encoding]::new($false)
)

foreach ($phase in @('stage', 'publish')) {
    $phaseLaunch = $phaseLaunches | Where-Object { $_.phase -eq $phase }
    $phaseLog = [string]$phaseLaunch.log
    $phaseError = [string]$phaseLaunch.error
    $arguments = @($phaseLaunch.arguments)
    $env:LINGTU_FACTORY_PARK_INSTANCE_PHASE = $phase
    $env:LINGTU_FACTORY_PARK_INSTANCE_ERROR = $phaseError

    $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $editor
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    foreach ($argument in $arguments) {
        $startInfo.ArgumentList.Add($argument)
    }
    $process = [System.Diagnostics.Process]::new()
    $process.StartInfo = $startInfo
    $processStarted = $false
    try {
        $processStarted = $process.Start()
        if (-not $processStarted) {
            throw "FactoryPark $phase process did not start"
        }
        $deadline = (Get-Date).AddMinutes($TimeoutMinutes)
        while (-not $process.HasExited -and (Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 2
            $process.Refresh()
            $forbiddenBuildProcesses = @(
                Get-Process -ErrorAction SilentlyContinue |
                    Where-Object {
                        $_.ProcessName -in @(
                            'UnrealBuildTool',
                            'AutomationTool',
                            'dotnet',
                            'cl',
                            'link',
                            'msbuild'
                        )
                    }
            )
            if ($forbiddenBuildProcesses.Count -ne 0) {
                $summary = (
                    $forbiddenBuildProcesses |
                        ForEach-Object { "$($_.ProcessName):$($_.Id)" }
                ) -join ', '
                throw "FactoryPark instance compiler spawned or overlapped a build process: $summary"
            }
        }
        if (-not $process.HasExited) {
            throw "FactoryPark $phase exceeded $TimeoutMinutes minutes; log=$phaseLog"
        }
        if ($process.ExitCode -ne 0) {
            throw "FactoryPark $phase exited $($process.ExitCode); log=$phaseLog"
        }
        if (Test-Path -LiteralPath $phaseError -PathType Leaf) {
            throw "FactoryPark $phase wrote error evidence: $phaseError"
        }
        if ($phase -eq 'stage' -and -not (Test-Path -LiteralPath $stageSentinel -PathType Leaf)) {
            throw "FactoryPark stage did not write evidence: $stageSentinel"
        }
        if ($phase -eq 'publish' -and -not (Test-Path -LiteralPath $success -PathType Leaf)) {
            throw "FactoryPark publish did not write success evidence: $success"
        }
    } finally {
        if ($processStarted -and -not $process.HasExited) {
            $process.Kill($true)
            $process.WaitForExit()
        }
        if ($null -ne $process) {
            $process.Dispose()
        }
    }
}

$result = Get-Content -LiteralPath $success -Raw | ConvertFrom-Json
if (
    $result.schema -ne 'lingtu.sim.factory-park-visual-instance-evidence.v1' -or
    $result.result -ne 'success' -or
    $result.verification.stable_id_set_exact -ne $true -or
    $result.verification.collision_authority_verified -ne $true
) {
    throw "FactoryPark instance compiler evidence failed its public contract: $success"
}
Write-Output "run_id=$runId"
Write-Output "evidence=$success"
Write-Output "actor_reduction=$($result.contract.actor_reduction)"
Write-Output "instance_count=$($result.verification.observed_instance_count)"
