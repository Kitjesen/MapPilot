[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [string]$Manifest,

    [Parameter(Mandatory)]
    [string]$Bundle,

    [string]$MujocoHost = 'build\mujoco-runtime-physics-win\Release\lingtu_mujoco_headless.exe',

    [string]$MotionCameraStableId,

    [int]$SnapshotPort = 25125
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$python = Join-Path $repoRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    throw "Repository Python runtime is missing: $python"
}

$arguments = @(
    '-m', 'sim.distribution.windows.smoke_cli',
    $Manifest,
    $Bundle,
    '--repo-root', $repoRoot,
    '--mujoco-host', $MujocoHost,
    '--snapshot-port', [string]$SnapshotPort
)
if ($MotionCameraStableId) {
    $arguments += @('--motion-camera-stable-id', $MotionCameraStableId)
}

Push-Location -LiteralPath $repoRoot
try {
    & $python @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Packaged RobotSimUE smoke failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
