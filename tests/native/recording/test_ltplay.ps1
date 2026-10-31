param(
    [Parameter(Mandatory = $true)][string]$Recorder,
    [Parameter(Mandatory = $true)][string]$Publisher,
    [Parameter(Mandatory = $true)][string]$Player,
    [Parameter(Mandatory = $true)][string]$Ltplay
)
$ErrorActionPreference = 'Stop'
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('lingtu ltplay ' + [guid]::NewGuid())
[IO.Directory]::CreateDirectory($testRoot) | Out-Null
$oldRoot = $env:LINGTU_RECORDING_ROOT
$oldHome = $env:HOME
$session = $null
function Require($condition, [string]$message) { if (-not $condition) { throw $message } }
try {
    $started = (& $Recorder start --root $testRoot --prefix fixture --dds on --camera off --min-free-gib 0 --dds-domain 205 --dds-topic /imu/raw --dds-topic /slam/odometry --dds-topic /nav/command/ack --seconds 20) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $started.state -eq 'recording') 'fixture capture did not start'
    $session = $started.session
    & $Publisher 205 --with-odom --with-ack
    Require ($LASTEXITCODE -eq 0) 'fixture publisher failed'
    $stopped = (& $Recorder stop --root $testRoot --timeout-ms 5000) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $stopped.state -eq 'completed') 'fixture capture did not complete'
    $env:LINGTU_RECORDING_ROOT = $testRoot
    $info = (& $Ltplay --info $session.session_id --json) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $info.message_count -eq 60) 'ltplay named-session resolution lost messages'
    $env:HOME = Split-Path -Path $testRoot -Parent
    $env:LINGTU_RECORDING_ROOT = '~/' + (Split-Path -Path $testRoot -Leaf)
    $expanded = (& $Ltplay --info $session.session_id --json) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $expanded.message_count -eq 60) 'ltplay did not expand the Gateway-supported recording root'
    $env:LINGTU_RECORDING_ROOT = $testRoot
    $checked = (& $Ltplay $session.session_directory --dry-run --json) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $checked.validated_messages -eq 60) 'ltplay directory validation failed'
    $short = & $Ltplay $session.session_id -r 1000
    Require ($LASTEXITCODE -eq 0 -and $short -match '^replayed=20 domain=84 rate=1000') "ltplay default mixed old outputs with raw inputs: $short"
    $explicit = & $Ltplay $session.session_directory --topic /slam/odometry --domain 85 -r 1000
    Require ($LASTEXITCODE -eq 0 -and $explicit -match '^replayed=20 domain=85') 'explicit sensor selection did not override raw-input defaults'
    $legacy = & $Player (Join-Path $session.session_directory 'dds/sensors.mcap') --domain 86 --rate 1000
    Require ($LASTEXITCODE -eq 0 -and $legacy -match '^replayed=40 domain=86.*skipped_record_only=20') 'legacy player behavior changed or record-only ACK was published'
    & $Ltplay $session.session_id --topic /nav/cmd_vel 2>$null
    Require ($LASTEXITCODE -ne 0) 'ltplay accepted a motion command as replay input'
    & $Ltplay $session.session_id --domain 0 2>$null
    Require ($LASTEXITCODE -ne 0) 'ltplay accepted live domain without explicit opt-in'
    Write-Output 'ltplay session ID/directory/file, raw defaults, explicit override and command boundaries passed'
} finally {
    if ($session) { & $Recorder stop --root $testRoot --timeout-ms 5000 *> $null }
    $env:LINGTU_RECORDING_ROOT = $oldRoot
    $env:HOME = $oldHome
    $absolute = [IO.Path]::GetFullPath($testRoot)
    $temporary = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if (-not $absolute.StartsWith($temporary, [StringComparison]::OrdinalIgnoreCase)) { throw 'test cleanup escaped temporary directory' }
    Remove-Item -LiteralPath $absolute -Recurse -Force
}
