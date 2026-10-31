param(
    [Parameter(Mandatory = $true)][string]$Recorder,
    [Parameter(Mandatory = $true)][string]$Publisher,
    [Parameter(Mandatory = $true)][string]$Player
)
$ErrorActionPreference = "Stop"
$TestRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("lingtu recording control " + [guid]::NewGuid())
[System.IO.Directory]::CreateDirectory($TestRoot) | Out-Null
$Session = $null
$ManagerId = 0
function Require([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}
try {
    $TopicsText = & $Recorder topics --json
    Require ($LASTEXITCODE -eq 0) "native topic catalog failed"
    $Topics = $TopicsText | ConvertFrom-Json
    Require ($Topics.control_version -eq 1 -and $Topics.ok) "invalid native topic catalog"
    $Joint = @($Topics.topics | Where-Object topic -eq "/robot/joint_states")
    Require ($Joint.Count -eq 1 -and $Joint[0].replayable -and $Joint[0].type_name -eq "lingtu.dds.JointState") "joint telemetry catalog missing"
    $Motion = @($Topics.topics | Where-Object topic -eq "/nav/cmd_vel")
    Require ($Motion.Count -eq 1 -and -not $Motion[0].replayable) "motion evidence became replayable"
    $StartText = & $Recorder start --root $TestRoot --prefix control --camera off --dds on `
        --dds-domain 206 --dds-topic /imu/raw --dds-topic /nav/command/request --dds-topic /nav/command/ack --min-free-gib 0 `
        --product test-product --product-session-id test-session --robot-id test-robot
    Require ($LASTEXITCODE -eq 0) "native start failed: $StartText"
    $Started = $StartText | ConvertFrom-Json
    Require ($Started.healthy -and $Started.state -eq "recording") "manager did not become healthy"
    $Session = $Started.session
    $ManagerId = $Session.manager_pid
    Require ($Session.context.product -eq "test-product") "recording context was lost"

    $Mismatch = & $Recorder stop --root $TestRoot --expected-session-id wrong-session --timeout-ms 100
    Require ($LASTEXITCODE -ne 0) "stop accepted a different expected session"
    Require (($Mismatch | ConvertFrom-Json).error.code -eq "recording_session_mismatch") "stop mismatch code changed"
    $Live = & $Recorder status --root $TestRoot
    Require ($LASTEXITCODE -eq 0 -and ($Live | ConvertFrom-Json).healthy) "native live status failed"

    # Capture real typed DDS samples before asking the manager to flush its worker.
    Start-Sleep -Milliseconds 350
    & $Publisher 206 --with-ack
    Require ($LASTEXITCODE -eq 0) "typed DDS publisher failed"
    $StoppedText = & $Recorder stop --root $TestRoot --expected-session-id $Session.session_id --timeout-ms 5000
    Require ($LASTEXITCODE -eq 0) "native stop failed: $StoppedText"
    $Stopped = $StoppedText | ConvertFrom-Json
    Require ($Stopped.state -eq "completed") "recording did not complete"
    Require ($Stopped.session.children[0].exit_code -eq 0) "DDS worker did not exit cleanly"
    $Mcap = Join-Path $Session.session_directory "dds/sensors.mcap"
    Require ((Test-Path -LiteralPath $Mcap) -and -not (Test-Path -LiteralPath "$Mcap.tmp")) "MCAP was not promoted"
    $Info = & $Player --info $Mcap
    Require ($LASTEXITCODE -eq 0 -and ($Info -match "messages=[1-9][0-9]*")) "MCAP could not be read: $Info"
    $Summary = (& $Player --info $Mcap --json) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $Summary.ok -and $Summary.message_count -gt 0) "JSON MCAP summary failed"
    Require ($Summary.topics[0].topic -eq "/imu/raw" -and $Summary.topics[0].message_count -gt 0) "JSON MCAP topic count is wrong"
    $Quiet = @($Summary.topics | Where-Object topic -eq "/nav/command/request")
    Require ($Quiet.Count -eq 1 -and $Quiet[0].message_count -eq 0 -and -not $Quiet[0].replayable) "quiet optional topic was omitted or marked replayable"
    $Checked = (& $Player $Mcap --dry-run --json) | ConvertFrom-Json
    Require ($LASTEXITCODE -eq 0 -and $Checked.mode -eq "validate" -and $Checked.validated_messages -eq $Summary.message_count) "offline JSON validation disagrees with MCAP summary"
    $Acks = @($Summary.topics | Where-Object topic -eq "/nav/command/ack")
    Require ($Acks.Count -eq 1 -and $Acks[0].message_count -gt 0 -and $Checked.skipped_record_only -eq $Acks[0].message_count) "offline validation lost record-only message accounting"
    Write-Output $Info
    & $Player $Mcap --dry-run
    Require ($LASTEXITCODE -eq 0) "recorded CDR schema could not be validated"
    $ManagerId = 0

    $EmptyStart = & $Recorder start --root $TestRoot --prefix empty --camera off --dds on `
        --dds-domain 207 --dds-topic /imu/raw --min-free-gib 0
    Require ($LASTEXITCODE -eq 0) "empty session did not start"
    $Session = ($EmptyStart | ConvertFrom-Json).session
    $ManagerId = $Session.manager_pid
    $EmptyStop = & $Recorder stop --root $TestRoot --expected-session-id $Session.session_id --timeout-ms 5000
    Require ($LASTEXITCODE -ne 0 -and ($EmptyStop | ConvertFrom-Json).state -eq "failed") "empty evidence falsely completed"
    $EmptyMcap = Join-Path $Session.session_directory "dds/sensors.mcap"
    Require (-not (Test-Path -LiteralPath $EmptyMcap) -and (Test-Path -LiteralPath "$EmptyMcap.tmp")) "incomplete MCAP was promoted"
    $Failed = $EmptyStop | ConvertFrom-Json
    Require ($Failed.session.children[0].artifacts -contains "dds/sensors.mcap.tmp") "failed manifest hid retained partial MCAP"
    Require ($Failed.session.children[0].artifacts -contains "logs/dds.stderr.log") "failed manifest hid worker diagnostics"
    $ManagerId = 0

    $IncompleteStart = & $Recorder start --root $TestRoot --prefix incomplete-inspection `
        --camera off --dds on --dds-domain 208 --min-free-gib 0 `
        --dds-preset inspection-evidence-v1 --product inspection `
        --product-session-id product-incomplete-inspection `
        --inspection-task-id inspection-incomplete-task
    Require ($LASTEXITCODE -eq 0) "incomplete inspection session did not start"
    $Session = ($IncompleteStart | ConvertFrom-Json).session
    $ManagerId = $Session.manager_pid
    Start-Sleep -Milliseconds 350
    & $Publisher 208
    Require ($LASTEXITCODE -eq 0) "incomplete inspection DDS publisher failed"
    $IncompleteStop = & $Recorder stop --root $TestRoot --expected-session-id $Session.session_id --timeout-ms 5000
    Require ($LASTEXITCODE -ne 0 -and ($IncompleteStop | ConvertFrom-Json).state -eq "failed") "incomplete inspection evidence falsely completed"
    $IncompleteMcap = Join-Path $Session.session_directory "dds/sensors.mcap"
    Require (-not (Test-Path -LiteralPath $IncompleteMcap)) "incomplete inspection MCAP was promoted"
    $RetainedInfo = & $Player --info "$IncompleteMcap.tmp"
    Require ($LASTEXITCODE -eq 0 -and ($RetainedInfo -match "/imu/raw.*count=[1-9][0-9]*")) "failed inspection discarded its captured DDS facts: $RetainedInfo"
    Write-Output $RetainedInfo
    $ManagerId = 0

    $LockPath = Join-Path $TestRoot ".control.lock"
    $Lock = [System.IO.File]::Open($LockPath, "Open", "ReadWrite", "ReadWrite")
    try {
        $Lock.Lock(0, 1)
        $Locked = & $Recorder start --root $TestRoot --camera off --dds on --dds-topic /imu/raw
        Require ($LASTEXITCODE -ne 0 -and ($Locked | ConvertFrom-Json).error.code -eq "recording_control_busy") "controller ignored the root lock"
    } finally { $Lock.Dispose() }

    $ErrorActionPreference = "Continue"
    $CameraError = & $Recorder start --root $TestRoot --camera on --dds on 2>&1
    $ErrorActionPreference = "Stop"
    Require ($LASTEXITCODE -ne 0 -and "$CameraError" -match "requires Linux POSIX shared memory") "Windows camera request was silently accepted"
    Write-Output "Windows native start/status/stop completed with readable typed DDS MCAP"
} finally {
    if ($ManagerId -gt 0 -and $null -ne $Session) {
        & $Recorder stop --root $TestRoot --expected-session-id $Session.session_id --timeout-ms 5000 | Out-Null
    }
    $ResolvedTestRoot = [System.IO.Path]::GetFullPath($TestRoot)
    $TempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
    if ($ResolvedTestRoot.StartsWith($TempRoot) -and [System.IO.Path]::GetFileName($ResolvedTestRoot).StartsWith("lingtu recording control ")) {
        Remove-Item -LiteralPath $ResolvedTestRoot -Recurse -Force
    }
}
