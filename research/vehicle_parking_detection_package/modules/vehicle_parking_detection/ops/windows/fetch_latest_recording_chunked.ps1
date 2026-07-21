param(
  [string]$Board = "sunrise@192.168.66.65",
  [string]$RemoteSh = "~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh",
  [string]$RemoteTar = "~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_return.tgz",
  [string]$LocalReturn = "",
  [int]$Retries = 5
)

$ErrorActionPreference = "Stop"
$PackageRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
if (-not $LocalReturn) { $LocalReturn = Join-Path $PackageRoot "board_return\vehicle_parking_detection" }

function Run-Checked {
  param([string]$File, [string[]]$Arguments)
  & $File @Arguments
  if ($LASTEXITCODE -ne 0) {
    throw "$File failed with exit code $LASTEXITCODE"
  }
}

function Fetch-RemoteFile {
  param(
    [string]$RemotePath,
    [string]$LocalPath,
    [int]$Retries
  )
  $partial = "$LocalPath.part"
  if (Test-Path -LiteralPath $partial) {
    Remove-Item -LiteralPath $partial -Force
  }
  for ($attempt = 1; $attempt -le $Retries; $attempt++) {
    Write-Host "fetch $RemotePath -> $LocalPath attempt $attempt/$Retries"
    & scp "$Board`:$RemotePath" $partial
    if ($LASTEXITCODE -eq 0 -and (Test-Path -LiteralPath $partial) -and (Get-Item -LiteralPath $partial).Length -gt 0) {
      if (Test-Path -LiteralPath $LocalPath) {
        Remove-Item -LiteralPath $LocalPath -Force
      }
      Move-Item -LiteralPath $partial -Destination $LocalPath -Force
      return
    }
    if (Test-Path -LiteralPath $partial) {
      Remove-Item -LiteralPath $partial -Force
    }
    Start-Sleep -Seconds ([Math]::Min(10, 2 * $attempt))
  }
  throw "failed to fetch $RemotePath after $Retries attempts"
}

function Copy-FileIntoStream {
  param(
    [string]$Path,
    [System.IO.Stream]$OutputStream
  )
  $inputStream = [System.IO.File]::OpenRead($Path)
  try {
    $inputStream.CopyTo($OutputStream)
  } finally {
    $inputStream.Dispose()
  }
}

New-Item -ItemType Directory -Force -Path $LocalReturn | Out-Null
$localTar = Join-Path $LocalReturn "vehicle_parking_return.tgz"
$localTmp = Join-Path $LocalReturn "_extract_tmp"
$shaPath = "$localTar.sha256"

Write-Host "[chunked] Package latest recording into split files on board."
Run-Checked ssh @($Board, "ACTION=package_latest_split SPLIT_SIZE=256M bash $RemoteSh")

Write-Host "[chunked] Query remote parts."
$parts = & ssh $Board "ls -1 $RemoteTar.part.*"
if ($LASTEXITCODE -ne 0 -or -not $parts) {
  throw "failed to list remote split parts"
}
$parts = @($parts | Where-Object { $_ -and $_.Trim() } | ForEach-Object { $_.Trim() })

Write-Host "[chunked] Remote part count: $($parts.Count)"
Get-ChildItem -LiteralPath $LocalReturn -Filter "vehicle_parking_return.tgz.part.*" -ErrorAction SilentlyContinue | Remove-Item -Force
if (Test-Path -LiteralPath $localTar) {
  Remove-Item -LiteralPath $localTar -Force
}
if (Test-Path -LiteralPath $shaPath) {
  Remove-Item -LiteralPath $shaPath -Force
}

Fetch-RemoteFile "$RemoteTar.sha256" $shaPath $Retries

$localParts = @()
foreach ($remotePart in $parts) {
  $name = Split-Path -Leaf $remotePart
  $localPart = Join-Path $LocalReturn $name
  Fetch-RemoteFile $remotePart $localPart $Retries
  $localParts += $localPart
}

Write-Host "[chunked] Merge parts."
$outputStream = [System.IO.File]::Open($localTar, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write)
try {
  foreach ($part in ($localParts | Sort-Object)) {
    Copy-FileIntoStream $part $outputStream
  }
} finally {
  $outputStream.Dispose()
}

Write-Host "[chunked] Verify SHA256."
$expected = (Get-Content -LiteralPath $shaPath -TotalCount 1).Split(" ", [System.StringSplitOptions]::RemoveEmptyEntries)[0].ToLowerInvariant()
$actual = (Get-FileHash -LiteralPath $localTar -Algorithm SHA256).Hash.ToLowerInvariant()
if ($expected -ne $actual) {
  throw "sha256 mismatch: expected=$expected actual=$actual"
}

Write-Host "[chunked] Verify tar archive."
& tar -tzf $localTar *> $null
if ($LASTEXITCODE -ne 0) {
  throw "tar archive verification failed"
}

Write-Host "[chunked] Extract archive."
if (Test-Path -LiteralPath $localTmp) {
  Remove-Item -LiteralPath $localTmp -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $localTmp | Out-Null
Run-Checked tar @("-xzf", $localTar, "-C", $localTmp)

$outputPath = Join-Path $LocalReturn "output"
$logsPath = Join-Path $LocalReturn "logs"
if (Test-Path -LiteralPath $outputPath) {
  Remove-Item -LiteralPath $outputPath -Recurse -Force
}
if (Test-Path -LiteralPath $logsPath) {
  Remove-Item -LiteralPath $logsPath -Recurse -Force
}
Move-Item -LiteralPath (Join-Path $localTmp "output") -Destination $outputPath -Force
if (Test-Path -LiteralPath (Join-Path $localTmp "logs")) {
  Move-Item -LiteralPath (Join-Path $localTmp "logs") -Destination $logsPath -Force
}
Remove-Item -LiteralPath $localTmp -Recurse -Force

Write-Host "[chunked] Done: $LocalReturn"
