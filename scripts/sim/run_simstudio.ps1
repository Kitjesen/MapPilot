[CmdletBinding()]
param(
    [switch]$Foreground,
    [ValidateRange(1, 65535)]
    [int]$Port = 8765
)

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")).Path

foreach ($requiredFile in @("pyproject.toml")) {
    if (-not (Test-Path -LiteralPath (Join-Path $repoRoot $requiredFile) -PathType Leaf)) {
        throw "SimStudio launcher resolved an uncontrolled repository root: $repoRoot"
    }
}
foreach ($requiredDirectory in @("sim", "config")) {
    if (-not (Test-Path -LiteralPath (Join-Path $repoRoot $requiredDirectory) -PathType Container)) {
        throw "SimStudio launcher resolved an uncontrolled repository root: $repoRoot"
    }
}
$pythonPath = Join-Path $repoRoot ".venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) {
    throw "SimStudio requires the repository virtual environment: $pythonPath"
}

$arguments = @("-m", "tools.simstudio", "--port", $Port.ToString())

if ($Foreground) {
    Push-Location $repoRoot
    try {
        & $pythonPath @arguments
        exit $LASTEXITCODE
    }
    finally {
        Pop-Location
    }
}

Start-Process `
    -FilePath $pythonPath `
    -ArgumentList $arguments `
    -WorkingDirectory $repoRoot `
    -WindowStyle Hidden
