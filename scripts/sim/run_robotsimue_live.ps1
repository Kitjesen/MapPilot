param(
    [Parameter(Mandatory=$true)]
    [string]$Bundle,

    [Parameter(Mandatory=$true)]
    [string]$UnrealEditor,

    [string]$MujocoHost = "build\mujoco-runtime-physics-win\Release\lingtu_mujoco_headless.exe",
    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [string]$RunRoot = "runs\simulation",
    [string]$UProject = "sim\runtime\visual\RobotSimUE\RobotSimUE.uproject",
    [string]$Map = "/Game/RobotSim/Maps/ThunderV4_RuntimePreview",
    [ValidateSet("visual-applied", "session-ready")]
    [string]$Gate = "visual-applied",
    [int]$SnapshotPort = 25123,
    [int]$DdsDomain = 0,
    [int]$Frames = 300,
    [int]$StepsPerFrame = 8,
    [int]$WarmupSteps = 8,
    [double]$ReadyTimeoutS = 45.0,
    [string]$RunId,
    [string]$PythonExe = "python"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path $RepoRoot).Path
function Resolve-InputPath([string]$Value) {
    if ([System.IO.Path]::IsPathRooted($Value)) {
        return Resolve-Path $Value
    }
    return Resolve-Path (Join-Path $repo $Value)
}

$bundlePath = Resolve-InputPath $Bundle
$mujocoPath = Resolve-InputPath $MujocoHost
$uprojectPath = Resolve-InputPath $UProject
$editorPath = Resolve-InputPath $UnrealEditor

$argsList = @(
    "-m", "sim.runtime.coordinator.live_visual",
    $bundlePath.Path,
    "--repo-root", $repo,
    "--run-root", (Join-Path $repo $RunRoot),
    "--mujoco-host", $mujocoPath.Path,
    "--unreal-editor", $editorPath.Path,
    "--uproject", $uprojectPath.Path,
    "--map", $Map,
    "--snapshot-port", $SnapshotPort,
    "--dds-domain", $DdsDomain,
    "--gate", $Gate,
    "--frames", $Frames,
    "--steps-per-frame", $StepsPerFrame,
    "--warmup-steps", $WarmupSteps,
    "--ready-timeout-s", $ReadyTimeoutS
)
if ($RunId) {
    $argsList += @("--run-id", $RunId)
}

Push-Location $repo
try {
    & $PythonExe @argsList
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
