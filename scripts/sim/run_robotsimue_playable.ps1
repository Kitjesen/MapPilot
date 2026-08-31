param(
    [Parameter(Mandatory=$true)]
    [string]$Bundle,

    [Parameter(Mandatory=$true)]
    [string]$RobotSimUEExecutable,

    [Parameter(Mandatory=$true)]
    [string]$MujocoHost,

    [Parameter(Mandatory=$true)]
    [string]$ImuPublisher,

    [Parameter(Mandatory=$true)]
    [string]$TruthOdomPublisher,

    [Parameter(Mandatory=$true)]
    [string]$Mid360Publisher,

    [Parameter(Mandatory=$true)]
    [string]$Ffmpeg,

    [Parameter(Mandatory=$true)]
    [string]$Ffprobe,

    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [string]$RunRoot = "build\playable-runs",
    [ValidateRange(1, 65535)]
    [int]$VisualSnapshotPort = 25123,
    [ValidateRange(1, 65535)]
    [int]$ControlIntentPort = 25124,
    [ValidateRange(1, 65535)]
    [int]$ControlStatusPort = 25125,
    [ValidateRange(0, 232)]
    [int]$DdsDomain = 0,
    [string]$RunId,
    [string]$BootId,
    [string]$PythonExe = "python"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path -LiteralPath $RepoRoot).Path

function Resolve-PlayablePath([string]$Value) {
    $candidate = if ([System.IO.Path]::IsPathRooted($Value)) {
        $Value
    }
    else {
        Join-Path $repo $Value
    }
    return (Resolve-Path -LiteralPath $candidate).Path
}

$arguments = @(
    "-m", "sim.runtime.coordinator.playable_vertical_slice",
    (Resolve-PlayablePath $Bundle),
    "--repo-root", $repo,
    "--run-root", ([System.IO.Path]::GetFullPath((Join-Path $repo $RunRoot))),
    "--mujoco-host", (Resolve-PlayablePath $MujocoHost),
    "--robotsimue-executable", (Resolve-PlayablePath $RobotSimUEExecutable),
    "--imu-publisher", (Resolve-PlayablePath $ImuPublisher),
    "--truth-odom-publisher", (Resolve-PlayablePath $TruthOdomPublisher),
    "--mid360-publisher", (Resolve-PlayablePath $Mid360Publisher),
    "--ffmpeg", (Resolve-PlayablePath $Ffmpeg),
    "--ffprobe", (Resolve-PlayablePath $Ffprobe),
    "--visual-snapshot-port", $VisualSnapshotPort,
    "--control-intent-port", $ControlIntentPort,
    "--control-status-port", $ControlStatusPort,
    "--dds-domain", $DdsDomain
)
if ($RunId) {
    $arguments += @("--run-id", $RunId)
}
if ($BootId) {
    $arguments += @("--boot-id", $BootId)
}

Push-Location $repo
try {
    & $PythonExe @arguments
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
