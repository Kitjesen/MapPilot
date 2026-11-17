param(
  [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\")).Path,
  [string[]]$Scene,
  [switch]$GenerateOnly,
  [double]$Timeout = 45
)
$script = Join-Path $RepoRoot "build\terrain-exploration-20261117\explore.py"
$args = @($script, "--timeout", "$Timeout")
foreach ($id in $Scene) { $args += @("--scene", $id) }
if ($GenerateOnly) { $args += "--generate-only" }
uv run python @args
uv run python (Join-Path $RepoRoot "build\terrain-exploration-20261117\report.py")
uv run python (Join-Path $RepoRoot "docs\media\terrain-exploration-20261117\export_boundaries.py")

