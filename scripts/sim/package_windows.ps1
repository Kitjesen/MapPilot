[CmdletBinding(DefaultParameterSetName = 'Package')]
param(
    [Parameter(Mandatory, ParameterSetName = 'Preflight')]
    [switch]$Preflight,

    [Parameter(Mandatory, ParameterSetName = 'DryRun')]
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$python = Join-Path $repoRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    throw "Repository Python runtime is missing: $python"
}

$operation = if ($Preflight) {
    'preflight'
} elseif ($DryRun) {
    'dry-run'
} else {
    'package'
}

Push-Location -LiteralPath $repoRoot
try {
    & $python -m sim.distribution.windows $operation
    if ($LASTEXITCODE -ne 0) {
        throw "RobotSimUE Windows distribution failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
