param(
    [int]$ProcessId = 32200,
    [ValidateRange(100, 60000)]
    [int]$HoldMilliseconds = 1200,
    [ValidateRange(0, 10000)]
    [int]$ReleaseSettleMilliseconds = 400,
    [ValidateRange(1, 100)]
    [int]$MinimumHoldSamples = 2,
    [ValidateRange(0, 10000)]
    [int]$MinimumHoldWindowMilliseconds = 250,
    [ValidateSet("TargetedMessages", "ForegroundInput")]
    [string]$InputMethod = "TargetedMessages",
    [ValidateSet("W", "S", "A", "D", "Q", "E")]
    [string]$DirectionKey = "W",
    [string]$RunDirectory = "",
    [string]$ExpectedRunId = "",
    [string]$ProductSessionId = "",
    [string]$SnapshotPath = "",
    [string]$SnapshotStableId = "thunder_01/base_link",
    [double]$MinimumPositionDisplacementMeters = 0.02,
    [double]$MinimumYawDeltaRadians = 0.05,
    [double]$MaximumPostReleaseDriftMeters = 0.05,
    [double]$MaximumPostReleaseSpeedMps = 0.08,
    [ValidateRange(2, 100)]
    [int]$MinimumUprightHoldSamples = 3,
    [ValidateRange(2, 100)]
    [int]$MinimumUprightReleaseSamples = 3,
    [ValidateRange(-1.0, 1.0)]
    [double]$MinimumBodyUpDot = 0.75,
    [ValidateRange(0.0, 3.141592653589793)]
    [double]$MaximumAbsoluteRollRadians = 0.60,
    [ValidateRange(0.0, 3.141592653589793)]
    [double]$MaximumAbsolutePitchRadians = 0.60,
    [ValidateRange(0.0, 10.0)]
    [double]$MinimumBaseHeightMeters = 0.25,
    [ValidateRange(0.0, 10.0)]
    [double]$MaximumBaseHeightDropMeters = 0.20,
    [ValidateRange(0.0, 1.0)]
    [double]$QuaternionNormTolerance = 0.01,
    [switch]$RequireLocomotion,
    [switch]$AllowTargetedLocomotionDiagnostic
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
[Console]::InputEncoding = [System.Text.UTF8Encoding]::new($false)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public static class LingTuDriveProbeNative
{
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

    public sealed class TopLevelWindowCandidate
    {
        public long Handle { get; set; }
        public uint ProcessId { get; set; }
        public bool Visible { get; set; }
        public long OwnerHandle { get; set; }
        public string Title { get; set; }
        public int WindowWidth { get; set; }
        public int WindowHeight { get; set; }
        public int ClientWidth { get; set; }
        public int ClientHeight { get; set; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RECT
    {
        public int Left;
        public int Top;
        public int Right;
        public int Bottom;
    }

    [DllImport("user32.dll")]
    public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);

    [DllImport("user32.dll")]
    public static extern bool GetClientRect(IntPtr hWnd, out RECT rect);

    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int command);

    [DllImport("user32.dll")]
    public static extern IntPtr GetForegroundWindow();

    [DllImport("user32.dll")]
    public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdc, uint flags);

    [DllImport("user32.dll")]
    public static extern bool PostMessage(IntPtr hWnd, uint message, UIntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern void keybd_event(byte virtualKey, byte scanCode, uint flags, UIntPtr extraInfo);

    [DllImport("user32.dll")]
    public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint processId);

    [DllImport("user32.dll")]
    public static extern bool EnumWindows(EnumWindowsProc callback, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern bool IsWindowVisible(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern IntPtr GetWindow(IntPtr hWnd, uint command);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern int GetWindowTextLength(IntPtr hWnd);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern int GetWindowText(
        IntPtr hWnd,
        System.Text.StringBuilder text,
        int maximumCount
    );

    [DllImport("kernel32.dll")]
    public static extern uint GetCurrentThreadId();

    [DllImport("user32.dll")]
    public static extern bool AttachThreadInput(uint firstThreadId, uint secondThreadId, bool attach);

    [DllImport("user32.dll")]
    public static extern bool BringWindowToTop(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern IntPtr SetFocus(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool SetCursorPos(int x, int y);

    [DllImport("user32.dll")]
    public static extern void mouse_event(uint flags, uint dx, uint dy, uint data, UIntPtr extraInfo);

    public static TopLevelWindowCandidate[] EnumerateTopLevelWindows(uint requestedProcessId)
    {
        var candidates = new List<TopLevelWindowCandidate>();
        EnumWindows(delegate(IntPtr hWnd, IntPtr lParam)
        {
            uint processId;
            GetWindowThreadProcessId(hWnd, out processId);
            bool visible = IsWindowVisible(hWnd);
            IntPtr owner = GetWindow(hWnd, 4); // GW_OWNER
            if (processId != requestedProcessId || !visible || owner != IntPtr.Zero)
            {
                return true;
            }

            RECT windowRect;
            RECT clientRect;
            bool hasWindowRect = GetWindowRect(hWnd, out windowRect);
            bool hasClientRect = GetClientRect(hWnd, out clientRect);
            candidates.Add(new TopLevelWindowCandidate
            {
                Handle = hWnd.ToInt64(),
                ProcessId = processId,
                Visible = visible,
                OwnerHandle = owner.ToInt64(),
                Title = GetWindowTitle(hWnd),
                WindowWidth = hasWindowRect ? Math.Max(0, windowRect.Right - windowRect.Left) : 0,
                WindowHeight = hasWindowRect ? Math.Max(0, windowRect.Bottom - windowRect.Top) : 0,
                ClientWidth = hasClientRect ? Math.Max(0, clientRect.Right - clientRect.Left) : 0,
                ClientHeight = hasClientRect ? Math.Max(0, clientRect.Bottom - clientRect.Top) : 0
            });
            return true;
        }, IntPtr.Zero);
        return candidates.ToArray();
    }

    public static string GetWindowTitle(IntPtr hWnd)
    {
        int length = GetWindowTextLength(hWnd);
        if (length <= 0)
        {
            return "";
        }
        var text = new System.Text.StringBuilder(length + 1);
        GetWindowText(hWnd, text, text.Capacity);
        return text.ToString();
    }
}
'@

function Select-RobotSimWindowCandidate {
    param(
        [Parameter(Mandatory = $true)]
        [AllowNull()]
        [AllowEmptyCollection()]
        [object[]]$Candidates,
        [Parameter(Mandatory = $true)][uint32]$ExpectedProcessId
    )

    $eligible = @(
        $Candidates | Where-Object {
            $null -ne $_ -and
            [int64]$_.Handle -gt 0 -and
            [uint32]$_.ProcessId -eq $ExpectedProcessId -and
            [bool]$_.Visible -and
            [int64]$_.OwnerHandle -eq 0 -and
            [int64]$_.WindowWidth -gt 0 -and
            [int64]$_.WindowHeight -gt 0 -and
            [int64]$_.ClientWidth -gt 0 -and
            [int64]$_.ClientHeight -gt 0
        }
    )
    $preferred = @(
        $eligible | Where-Object {
            $title = [string]$_.Title
            $title -cmatch '^RobotSimUE($|[\s(\uFF08])' -and
            $title -cnotmatch '[\\/]' -and
            $title -notmatch '\.exe'
        }
    )
    if ($eligible.Count -eq 0) {
        throw "Process $ExpectedProcessId has no eligible visible unowned window with non-zero window and client area"
    }
    if ($preferred.Count -eq 0) {
        throw "Process $ExpectedProcessId has no eligible window whose title starts with RobotSimUE"
    }
    $selectionPool = $preferred
    $ranked = @(
        $selectionPool | ForEach-Object {
            [pscustomobject]@{
                Candidate = $_
                ClientArea = [int64]$_.ClientWidth * [int64]$_.ClientHeight
            }
        } | Sort-Object -Property ClientArea -Descending
    )
    if ($ranked.Count -gt 1 -and $ranked[0].ClientArea -eq $ranked[1].ClientArea) {
        throw "Process $ExpectedProcessId has multiple equally sized eligible RobotSimUE windows"
    }
    return $ranked[0].Candidate
}

function Get-RobotSimWindowCandidate {
    param(
        [Parameter(Mandatory = $true)][uint32]$ProcessId,
        [AllowNull()][scriptblock]$CandidateProvider = $null
    )

    $candidates = if ($null -eq $CandidateProvider) {
        @([LingTuDriveProbeNative]::EnumerateTopLevelWindows($ProcessId))
    }
    else {
        @(& $CandidateProvider)
    }
    $candidate = Select-RobotSimWindowCandidate `
        -Candidates $candidates `
        -ExpectedProcessId $ProcessId
    if ($null -eq $candidate) {
        throw "Process $ProcessId has no eligible visible unowned RobotSimUE window with non-zero window and client area"
    }
    return $candidate
}

function Get-ProbeForegroundWindowHandle {
    param(
        [switch]$Required,
        [scriptblock]$ForegroundWindowProvider = {
            [LingTuDriveProbeNative]::GetForegroundWindow()
        }
    )

    $foreground = & $ForegroundWindowProvider
    if ($null -eq $foreground) {
        if ($Required) {
            throw "RobotSimUE foreground window is unavailable"
        }
        return [int64]0
    }
    $foregroundPointer = [IntPtr]$foreground
    if ($foregroundPointer -eq [IntPtr]::Zero) {
        if ($Required) {
            throw "RobotSimUE foreground window is unavailable"
        }
        return [int64]0
    }
    return $foregroundPointer.ToInt64()
}

function Set-ProbeForegroundWindow {
    param([IntPtr]$WindowHandle)

    $foreground = [LingTuDriveProbeNative]::GetForegroundWindow()
    $foregroundProcessId = 0
    $targetProcessId = 0
    $foregroundThread = [LingTuDriveProbeNative]::GetWindowThreadProcessId(
        $foreground,
        [ref]$foregroundProcessId
    )
    $targetThread = [LingTuDriveProbeNative]::GetWindowThreadProcessId(
        $WindowHandle,
        [ref]$targetProcessId
    )
    $currentThread = [LingTuDriveProbeNative]::GetCurrentThreadId()

    $attachedForeground = $false
    $attachedTarget = $false
    try {
        if ($foregroundThread -ne 0 -and $foregroundThread -ne $currentThread) {
            $attachedForeground = [LingTuDriveProbeNative]::AttachThreadInput(
                $currentThread,
                $foregroundThread,
                $true
            )
        }
        if ($targetThread -ne 0 -and $targetThread -ne $currentThread) {
            $attachedTarget = [LingTuDriveProbeNative]::AttachThreadInput(
                $currentThread,
                $targetThread,
                $true
            )
        }
        [LingTuDriveProbeNative]::ShowWindow($WindowHandle, 9) | Out-Null
        [LingTuDriveProbeNative]::BringWindowToTop($WindowHandle) | Out-Null
        [LingTuDriveProbeNative]::SetForegroundWindow($WindowHandle) | Out-Null
        [LingTuDriveProbeNative]::SetFocus($WindowHandle) | Out-Null
    }
    finally {
        if ($attachedTarget) {
            [LingTuDriveProbeNative]::AttachThreadInput(
                $currentThread,
                $targetThread,
                $false
            ) | Out-Null
        }
        if ($attachedForeground) {
            [LingTuDriveProbeNative]::AttachThreadInput(
                $currentThread,
                $foregroundThread,
                $false
            ) | Out-Null
        }
    }
}

function Acquire-ProbeForegroundWindow {
    param(
        [IntPtr]$WindowHandle,
        [ValidateRange(100, 10000)]
        [int]$TimeoutMilliseconds = 5000
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMilliseconds)
    $observed = [int64]0
    do {
        Set-ProbeForegroundWindow -WindowHandle $WindowHandle
        Start-Sleep -Milliseconds 100
        $observed = Get-ProbeForegroundWindowHandle
        if ($observed -eq $WindowHandle.ToInt64()) {
            return $observed
        }
    } while ([DateTime]::UtcNow -lt $deadline)

    if ($observed -eq 0) {
        throw "RobotSimUE foreground window is unavailable after bounded retries"
    }
    throw "RobotSimUE could not acquire foreground input after bounded retries"
}

function Save-WindowCapture {
    param(
        [IntPtr]$WindowHandle,
        [string]$Path
    )

    $rect = New-Object LingTuDriveProbeNative+RECT
    if (-not [LingTuDriveProbeNative]::GetWindowRect($WindowHandle, [ref]$rect)) {
        throw "GetWindowRect failed"
    }

    $width = $rect.Right - $rect.Left
    $height = $rect.Bottom - $rect.Top
    $bitmap = New-Object System.Drawing.Bitmap $width, $height
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    try {
        $hdc = $graphics.GetHdc()
        try {
            if (-not [LingTuDriveProbeNative]::PrintWindow($WindowHandle, $hdc, 2)) {
                throw "PrintWindow failed"
            }
        }
        finally {
            $graphics.ReleaseHdc($hdc)
        }
        $bitmap.Save($Path, [System.Drawing.Imaging.ImageFormat]::Png)
    }
    finally {
        $graphics.Dispose()
        $bitmap.Dispose()
    }
}

function Measure-ImageChange {
    param(
        [string]$BeforePath,
        [string]$AfterPath
    )

    $before = [System.Drawing.Bitmap]::FromFile($BeforePath)
    $after = [System.Drawing.Bitmap]::FromFile($AfterPath)
    try {
        if ($before.Width -ne $after.Width -or $before.Height -ne $after.Height) {
            throw "Capture dimensions changed"
        }

        $changed = 0
        $samples = 0
        $absoluteDifference = 0.0
        for ($y = 48; $y -lt ($before.Height - 48); $y += 8) {
            for ($x = 48; $x -lt ($before.Width - 48); $x += 8) {
                $a = $before.GetPixel($x, $y)
                $b = $after.GetPixel($x, $y)
                $difference = [Math]::Abs([int]$a.R - [int]$b.R) +
                    [Math]::Abs([int]$a.G - [int]$b.G) +
                    [Math]::Abs([int]$a.B - [int]$b.B)
                if ($difference -gt 30) {
                    $changed += 1
                }
                $absoluteDifference += $difference
                $samples += 1
            }
        }

        return [ordered]@{
            changed_ratio = $changed / [double]$samples
            mean_rgb_absolute_difference = $absoluteDifference / [double]$samples
            samples = $samples
        }
    }
    finally {
        $before.Dispose()
        $after.Dispose()
    }
}

function Get-FileLineCount {
    param([string]$Path)

    if (-not $Path -or -not (Test-Path -LiteralPath $Path)) {
        return 0
    }
    return @(
        Get-Content -LiteralPath $Path -ErrorAction SilentlyContinue
    ).Count
}

function Read-JsonLines {
    param(
        [string]$Path,
        [int]$Skip = 0
    )

    if (-not $Path -or -not (Test-Path -LiteralPath $Path)) {
        return @()
    }

    $lines = @(Get-Content -LiteralPath $Path -ErrorAction SilentlyContinue)
    $start = [Math]::Max(0, $Skip)
    if ($start -ge $lines.Count) {
        return @()
    }

    $items = @()
    for ($index = $start; $index -lt $lines.Count; $index++) {
        $line = [string]$lines[$index]
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        try {
            $items += ($line | ConvertFrom-Json)
        }
        catch {
            # A concurrently appended final line may be incomplete. Ignore it;
            # the required phase evidence must be present in a complete record.
        }
    }
    return @($items)
}

function Read-OptionalJson {
    param([string]$Path)

    if (-not $Path -or -not (Test-Path -LiteralPath $Path)) {
        return $null
    }
    try {
        return Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    }
    catch {
        return $null
    }
}

function Get-NumberOrNull {
    param($Value)

    if ($null -eq $Value) {
        return $null
    }
    try {
        $number = [double]$Value
    }
    catch {
        return $null
    }
    if ([double]::IsNaN($number) -or [double]::IsInfinity($number)) {
        return $null
    }
    return $number
}

function Get-PropertyOrNull {
    param(
        $Object,
        [string]$Name
    )

    if ($null -eq $Object) {
        return $null
    }
    $property = $Object.PSObject.Properties[$Name]
    if ($null -eq $property) {
        return $null
    }
    return $property.Value
}

function Test-ControlOriginIdentity {
    param(
        [Parameter(Mandatory = $true)] [object]$Left,
        [Parameter(Mandatory = $true)] [object]$Right
    )

    $leftSourceId = [string](Get-PropertyOrNull $Left "source_id")
    $rightSourceId = [string](Get-PropertyOrNull $Right "source_id")
    $leftEpoch = Get-NumberOrNull (Get-PropertyOrNull $Left "source_epoch")
    $rightEpoch = Get-NumberOrNull (Get-PropertyOrNull $Right "source_epoch")
    return -not [string]::IsNullOrWhiteSpace($leftSourceId) -and
        $leftSourceId -eq $rightSourceId -and
        $null -ne $leftEpoch -and
        $leftEpoch -eq $rightEpoch
}

function Select-AcceptedControlClaims {
    param(
        [Parameter(Mandatory = $true)] [object[]]$Records,
        [Parameter(Mandatory = $true)] [string]$ExpectedRunId,
        [Parameter(Mandatory = $true)] [string]$ProductSessionId,
        [Parameter(Mandatory = $true)] [string]$ExpectedBootId,
        [Parameter(Mandatory = $true)] [object]$ExpectedModelGeneration,
        [Parameter(Mandatory = $true)] [object]$ExpectedResetGeneration
    )

    $modelGeneration = Get-NumberOrNull $ExpectedModelGeneration
    $resetGeneration = Get-NumberOrNull $ExpectedResetGeneration
    if ([string]::IsNullOrWhiteSpace($ExpectedRunId) -or
        [string]::IsNullOrWhiteSpace($ProductSessionId) -or
        [string]::IsNullOrWhiteSpace($ExpectedBootId) -or
        $null -eq $modelGeneration -or
        $null -eq $resetGeneration) {
        return @()
    }

    return @(
        $Records | Where-Object {
            [string](Get-PropertyOrNull $_ "event") -eq "runtime_request_accepted" -and
            [string](Get-PropertyOrNull $_ "request") -eq "control_claim" -and
            [string](Get-PropertyOrNull $_ "status") -eq "accepted" -and
            [string](Get-PropertyOrNull $_ "run_id") -eq $ExpectedRunId -and
            [string](Get-PropertyOrNull $_ "session_id") -eq $ProductSessionId -and
            [string](Get-PropertyOrNull $_ "boot_id") -eq $ExpectedBootId -and
            (Get-NumberOrNull (Get-PropertyOrNull $_ "model_generation")) -eq $modelGeneration -and
            (Get-NumberOrNull (Get-PropertyOrNull $_ "reset_generation")) -eq $resetGeneration
        }
    )
}

function Test-PressClaimCorrelation {
    param(
        [Parameter(Mandatory = $true)] [object]$PressRecord,
        [Parameter(Mandatory = $true)] [object]$ClaimRecord
    )

    if ([string](Get-PropertyOrNull $ClaimRecord "event") -ne "runtime_request_accepted" -or
        [string](Get-PropertyOrNull $ClaimRecord "request") -ne "control_claim" -or
        [string](Get-PropertyOrNull $ClaimRecord "status") -ne "accepted" -or
        -not (Test-ControlOriginIdentity -Left $PressRecord -Right $ClaimRecord)) {
        return $false
    }
    $pressSequence = Get-NumberOrNull (Get-PropertyOrNull $PressRecord "source_sequence")
    $claimSequence = Get-NumberOrNull (Get-PropertyOrNull $ClaimRecord "source_sequence")
    $pressTime = Get-NumberOrNull (Get-PropertyOrNull $PressRecord "source_monotonic_ns")
    $claimTime = Get-NumberOrNull (Get-PropertyOrNull $ClaimRecord "source_monotonic_ns")
    $timeGapNs = if ($null -ne $pressTime -and $null -ne $claimTime) {
        $pressTime - $claimTime
    }
    else {
        $null
    }
    return $null -ne $pressSequence -and
        $null -ne $claimSequence -and
        $pressSequence -gt $claimSequence -and
        $null -ne $pressTime -and
        $null -ne $claimTime -and
        $timeGapNs -ge 0 -and
        $timeGapNs -le 250000000
}

function Assert-LocomotionProbeContract {
    param(
        [bool]$RequireLocomotion,
        [string]$InputMethod,
        [bool]$AllowTargetedLocomotionDiagnostic,
        [string]$RunDirectory,
        [string]$ExpectedRunId,
        [string]$ProductSessionId
    )

    if (-not $RequireLocomotion) {
        return $true
    }
    if ($InputMethod -ne "ForegroundInput" -and
        -not ($InputMethod -eq "TargetedMessages" -and
            $AllowTargetedLocomotionDiagnostic)) {
        throw "RequireLocomotion with TargetedMessages requires AllowTargetedLocomotionDiagnostic"
    }
    if ([string]::IsNullOrWhiteSpace($RunDirectory)) {
        throw "RequireLocomotion requires an explicit RunDirectory"
    }
    if (-not [System.IO.Path]::IsPathRooted($RunDirectory)) {
        throw "RequireLocomotion requires an absolute RunDirectory"
    }
    if (-not (Test-Path -LiteralPath $RunDirectory -PathType Container)) {
        throw "RequireLocomotion RunDirectory does not exist: $RunDirectory"
    }
    if ([string]::IsNullOrWhiteSpace($ExpectedRunId)) {
        throw "RequireLocomotion requires ExpectedRunId"
    }
    if ([string]::IsNullOrWhiteSpace($ProductSessionId)) {
        throw "RequireLocomotion requires ProductSessionId"
    }
    return $true
}

function Get-RunIdentityQualification {
    param(
        [string]$RunDirectory,
        [string]$ExpectedRunId,
        [string]$ProductSessionId,
        $Allocation,
        $RuntimeBefore,
        $RuntimeAfter,
        $HealthBefore,
        $HealthAfter,
        [object[]]$ControlEvidenceRecords,
        [object[]]$SnapshotDocuments
    )

    $failureCodes = @()
    $runDirectoryFull = $null
    try {
        $runDirectoryFull = [System.IO.Path]::GetFullPath($RunDirectory)
    }
    catch {
        $failureCodes += "run_directory_invalid"
    }

    if ($null -eq $Allocation) {
        $failureCodes += "run_allocation_missing"
    }
    else {
        if ([string](Get-PropertyOrNull $Allocation "run_id") -cne $ExpectedRunId) {
            $failureCodes += "allocation_run_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $Allocation "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "allocation_session_id_mismatch"
        }
        $expectedLogDirectory = if ($null -ne $runDirectoryFull) {
            [System.IO.Path]::GetFullPath((Join-Path $runDirectoryFull "logs"))
        }
        else {
            $null
        }
        $observedLogDirectory = Get-PropertyOrNull $Allocation "log_dir"
        try {
            $observedLogDirectory = [System.IO.Path]::GetFullPath(
                [string]$observedLogDirectory
            )
        }
        catch {
            $observedLogDirectory = $null
        }
        if ($null -eq $expectedLogDirectory -or
            $null -eq $observedLogDirectory -or
            -not [System.StringComparer]::OrdinalIgnoreCase.Equals(
                $expectedLogDirectory,
                $observedLogDirectory
            )) {
            $failureCodes += "allocation_log_directory_mismatch"
        }
    }

    $runtimeModelGeneration = $null
    $runtimeResetGeneration = $null
    foreach ($entry in @(
            [pscustomobject]@{ label = "before"; value = $RuntimeBefore },
            [pscustomobject]@{ label = "after"; value = $RuntimeAfter }
        )) {
        $label = [string]$entry.label
        $runtime = $entry.value
        if ($null -eq $runtime) {
            $failureCodes += "runtime_${label}_missing"
            continue
        }
        if ([string](Get-PropertyOrNull $runtime "run_id") -cne $ExpectedRunId) {
            $failureCodes += "runtime_${label}_run_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $runtime "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "runtime_${label}_session_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $runtime "state") -cne "RUNNING") {
            $failureCodes += "runtime_${label}_not_running"
        }

        $runtimeAllocation = Get-PropertyOrNull $runtime "allocation"
        $observedRunDirectory = Get-PropertyOrNull $runtimeAllocation "run_dir"
        try {
            $observedRunDirectory = [System.IO.Path]::GetFullPath(
                [string]$observedRunDirectory
            )
        }
        catch {
            $observedRunDirectory = $null
        }
        if ($null -eq $runDirectoryFull -or
            $null -eq $observedRunDirectory -or
            -not [System.StringComparer]::OrdinalIgnoreCase.Equals(
                $runDirectoryFull,
                $observedRunDirectory
            )) {
            $failureCodes += "runtime_${label}_run_directory_mismatch"
        }

        $modelGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $runtime "model_generation"
        )
        $resetGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $runtime "reset_generation"
        )
        if ($null -eq $modelGeneration) {
            $failureCodes += "runtime_${label}_model_generation_missing"
        }
        if ($null -eq $resetGeneration) {
            $failureCodes += "runtime_${label}_reset_generation_missing"
        }
        if ($label -eq "before") {
            $runtimeModelGeneration = $modelGeneration
            $runtimeResetGeneration = $resetGeneration
        }
        else {
            if ($null -ne $runtimeModelGeneration -and
                $null -ne $modelGeneration -and
                [int64]$modelGeneration -ne [int64]$runtimeModelGeneration) {
                $failureCodes += "runtime_model_generation_changed"
            }
            if ($null -ne $runtimeResetGeneration -and
                $null -ne $resetGeneration -and
                [int64]$resetGeneration -ne [int64]$runtimeResetGeneration) {
                $failureCodes += "runtime_reset_generation_changed"
            }
        }
    }

    $healthSequences = @{}
    foreach ($entry in @(
            [pscustomobject]@{ label = "before"; value = $HealthBefore },
            [pscustomobject]@{ label = "after"; value = $HealthAfter }
        )) {
        $label = [string]$entry.label
        $health = $entry.value
        if ($null -eq $health) {
            $failureCodes += "health_${label}_missing"
            continue
        }
        if ([string](Get-PropertyOrNull $health "run_id") -cne $ExpectedRunId) {
            $failureCodes += "health_${label}_run_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $health "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "health_${label}_session_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $health "session_state") -cne "RUNNING") {
            $failureCodes += "health_${label}_not_running"
        }
        if ((Get-PropertyOrNull $health "owner_thread_alive") -ne $true) {
            $failureCodes += "health_${label}_owner_thread_not_alive"
        }
        $nonRunningTransitions = Get-NumberOrNull (
            Get-PropertyOrNull $health "non_running_transition_count"
        )
        if ($null -eq $nonRunningTransitions -or $nonRunningTransitions -ne 0.0) {
            $failureCodes += "health_non_running_transition_observed"
        }
        $ownerThreadStops = Get-NumberOrNull (
            Get-PropertyOrNull $health "owner_thread_stop_count"
        )
        if ($null -eq $ownerThreadStops -or $ownerThreadStops -ne 0.0) {
            $failureCodes += "health_owner_thread_stop_observed"
        }
        $healthModelGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $health "model_generation"
        )
        $healthResetGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $health "reset_generation"
        )
        if ($null -eq $runtimeModelGeneration -or
            $null -eq $healthModelGeneration -or
            [int64]$healthModelGeneration -ne [int64]$runtimeModelGeneration) {
            $failureCodes += "health_${label}_model_generation_mismatch"
        }
        if ($null -eq $runtimeResetGeneration -or
            $null -eq $healthResetGeneration -or
            [int64]$healthResetGeneration -ne [int64]$runtimeResetGeneration) {
            $failureCodes += "health_${label}_reset_generation_mismatch"
        }
        $healthSequences[$label] = [pscustomobject]@{
            monitor = Get-NumberOrNull (
                Get-PropertyOrNull $health "monitor_sequence"
            )
            event = Get-NumberOrNull (
                Get-PropertyOrNull $health "event_sequence"
            )
            truth = Get-NumberOrNull (
                Get-PropertyOrNull $health "last_truth_sequence"
            )
        }
    }
    if ($healthSequences.ContainsKey("before") -and
        $healthSequences.ContainsKey("after")) {
        $healthBeforeSequences = $healthSequences["before"]
        $healthAfterSequences = $healthSequences["after"]
        if ($null -eq $healthBeforeSequences.monitor -or
            $null -eq $healthAfterSequences.monitor -or
            $healthAfterSequences.monitor -le $healthBeforeSequences.monitor) {
            $failureCodes += "health_monitor_not_advanced"
        }
        if ($null -eq $healthBeforeSequences.event -or
            $null -eq $healthAfterSequences.event -or
            $healthAfterSequences.event -le $healthBeforeSequences.event) {
            $failureCodes += "health_event_not_advanced"
        }
        if ($null -eq $healthBeforeSequences.truth -or
            $null -eq $healthAfterSequences.truth -or
            $healthAfterSequences.truth -le $healthBeforeSequences.truth) {
            $failureCodes += "health_truth_not_advanced"
        }
    }

    $expectedBootId = Get-PropertyOrNull $Allocation "boot_id"
    $controlCount = 0
    foreach ($record in @($ControlEvidenceRecords)) {
        if ($null -eq $record) {
            continue
        }
        $controlCount += 1
        if ([string](Get-PropertyOrNull $record "run_id") -cne $ExpectedRunId) {
            $failureCodes += "control_run_id_mismatch"
        }
        if ([string](Get-PropertyOrNull $record "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "control_session_id_mismatch"
        }
        if ([string]::IsNullOrWhiteSpace([string]$expectedBootId) -or
            [string](Get-PropertyOrNull $record "boot_id") -cne
            [string]$expectedBootId) {
            $failureCodes += "control_boot_id_mismatch"
        }
        $modelGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $record "model_generation"
        )
        $resetGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $record "reset_generation"
        )
        if ($null -eq $runtimeModelGeneration -or
            $null -eq $modelGeneration -or
            [int64]$modelGeneration -ne [int64]$runtimeModelGeneration) {
            $failureCodes += "control_model_generation_mismatch"
        }
        if ($null -eq $runtimeResetGeneration -or
            $null -eq $resetGeneration -or
            [int64]$resetGeneration -ne [int64]$runtimeResetGeneration) {
            $failureCodes += "control_reset_generation_mismatch"
        }
    }
    if ($controlCount -eq 0) {
        $failureCodes += "control_evidence_missing"
    }

    $snapshotCount = 0
    foreach ($document in @($SnapshotDocuments)) {
        if ($null -eq $document) {
            continue
        }
        $snapshotCount += 1
        if ([string](Get-PropertyOrNull $document "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "snapshot_session_id_mismatch"
        }
        $modelGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $document "model_generation"
        )
        $resetGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $document "reset_generation"
        )
        if ($null -eq $runtimeModelGeneration -or
            $null -eq $modelGeneration -or
            [int64]$modelGeneration -ne [int64]$runtimeModelGeneration) {
            $failureCodes += "snapshot_model_generation_mismatch"
        }
        if ($null -eq $runtimeResetGeneration -or
            $null -eq $resetGeneration -or
            [int64]$resetGeneration -ne [int64]$runtimeResetGeneration) {
            $failureCodes += "snapshot_reset_generation_mismatch"
        }
    }
    if ($snapshotCount -eq 0) {
        $failureCodes += "snapshot_evidence_missing"
    }

    $failureCodes = @($failureCodes | Select-Object -Unique)
    $passed = $failureCodes.Count -eq 0
    return [ordered]@{
        state = if ($passed) { "passed" } else { "failed" }
        passed = $passed
        run_directory = $runDirectoryFull
        expected_run_id = $ExpectedRunId
        expected_session_id = $ProductSessionId
        boot_id = $expectedBootId
        model_generation = $runtimeModelGeneration
        reset_generation = $runtimeResetGeneration
        health_monitor_sequence_before = if ($healthSequences.ContainsKey("before")) {
            $healthSequences["before"].monitor
        }
        else {
            $null
        }
        health_monitor_sequence_after = if ($healthSequences.ContainsKey("after")) {
            $healthSequences["after"].monitor
        }
        else {
            $null
        }
        control_records = $controlCount
        snapshot_documents = $snapshotCount
        failure_codes = $failureCodes
        reason = if ($passed) {
            "all run-local control and truth evidence matched one live session generation"
        }
        else {
            "run identity, runtime state, or generation evidence did not remain exact"
        }
    }
}

function Get-RecordTimeNs {
    param($Record)

    foreach ($field in @(
            "apply_time_ns",
            "source_monotonic_ns",
            "server_monotonic_ns",
            "sim_time_ns"
        )) {
        $value = Get-NumberOrNull $Record.$field
        if ($null -ne $value -and $value -ge 0.0) {
            return [int64]$value
        }
    }
    return $null
}

function Test-NonZeroTwist {
    param($Twist)

    if ($null -eq $Twist) {
        return $false
    }
    foreach ($field in @("linear_x", "linear_y", "angular_z")) {
        $value = Get-NumberOrNull $Twist.$field
        if ($null -ne $value -and [Math]::Abs($value) -gt 0.01) {
            return $true
        }
    }
    return $false
}

function Test-ZeroTwist {
    param($Twist)

    if ($null -eq $Twist) {
        return $false
    }
    return -not (Test-NonZeroTwist $Twist)
}

function Test-ExpectedDirectionTwist {
    param(
        $Twist,
        [ValidateSet("W", "S", "A", "D", "Q", "E")]
        [string]$DirectionKey
    )

    if ($null -eq $Twist) {
        return $false
    }
    $linearX = Get-NumberOrNull $Twist.linear_x
    $linearY = Get-NumberOrNull $Twist.linear_y
    $angularZ = Get-NumberOrNull $Twist.angular_z
    if ($null -eq $linearX -or $null -eq $linearY -or $null -eq $angularZ) {
        return $false
    }
    switch ($DirectionKey) {
        "W" { return $linearX -gt 0.01 -and [Math]::Abs($linearY) -le 0.01 -and [Math]::Abs($angularZ) -le 0.01 }
        "S" { return $linearX -lt -0.01 -and [Math]::Abs($linearY) -le 0.01 -and [Math]::Abs($angularZ) -le 0.01 }
        "A" { return $linearY -gt 0.01 -and [Math]::Abs($linearX) -le 0.01 -and [Math]::Abs($angularZ) -le 0.01 }
        "D" { return $linearY -lt -0.01 -and [Math]::Abs($linearX) -le 0.01 -and [Math]::Abs($angularZ) -le 0.01 }
        "Q" { return $angularZ -gt 0.01 -and [Math]::Abs($linearX) -le 0.01 -and [Math]::Abs($linearY) -le 0.01 }
        "E" { return $angularZ -lt -0.01 -and [Math]::Abs($linearX) -le 0.01 -and [Math]::Abs($linearY) -le 0.01 }
    }
    return $false
}

function Get-PlanarSpeed {
    param($Velocity)

    if ($null -eq $Velocity) {
        return 0.0
    }
    $x = Get-NumberOrNull $Velocity.linear_x
    $y = Get-NumberOrNull $Velocity.linear_y
    if ($null -eq $x -or $null -eq $y) {
        return 0.0
    }
    return [Math]::Sqrt(($x * $x) + ($y * $y))
}

function Resolve-SnapshotPath {
    param(
        [string]$RequestedPath,
        [string]$RunDirectory
    )

    if (-not [string]::IsNullOrWhiteSpace($RequestedPath)) {
        try {
            return [System.IO.Path]::GetFullPath($RequestedPath)
        }
        catch {
            return $RequestedPath
        }
    }
    if ([string]::IsNullOrWhiteSpace($RunDirectory)) {
        return ""
    }

    $candidates = @(
        (Join-Path $RunDirectory "truth-snapshots.jsonl"),
        (Join-Path $RunDirectory "runtime-snapshots.jsonl"),
        (Join-Path $RunDirectory "snapshot.jsonl"),
        (Join-Path $RunDirectory "logs/truth-snapshots.jsonl"),
        (Join-Path $RunDirectory "logs/runtime-snapshots.jsonl"),
        (Join-Path $RunDirectory "truth-snapshot.json"),
        (Join-Path $RunDirectory "snapshot.json")
    )
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate) {
            return [System.IO.Path]::GetFullPath($candidate)
        }
    }
    return ""
}

function Test-SnapshotDocument {
    param($Document)

    if ($null -eq $Document) {
        return $false
    }
    return $null -ne $Document.bodies -or $null -ne $Document.entities
}

function Read-SnapshotDocuments {
    param(
        [string]$Path,
        [int]$SkipLines = 0
    )

    if (-not $Path -or -not (Test-Path -LiteralPath $Path)) {
        return @()
    }

    $raw = Get-Content -LiteralPath $Path -Raw -ErrorAction SilentlyContinue
    if ([string]::IsNullOrWhiteSpace($raw)) {
        return @()
    }

    if ($SkipLines -eq 0) {
        try {
            $parsed = $raw | ConvertFrom-Json
            $documents = @()
            if ($parsed -is [System.Array]) {
                foreach ($item in $parsed) {
                    if (Test-SnapshotDocument $item) {
                        $documents += $item
                    }
                }
            }
            elseif ($null -ne $parsed.snapshots -and $parsed.snapshots -is [System.Array]) {
                foreach ($item in $parsed.snapshots) {
                    if (Test-SnapshotDocument $item) {
                        $documents += $item
                    }
                }
            }
            elseif (Test-SnapshotDocument $parsed) {
                $documents += $parsed
            }
            if ($documents.Count -gt 0) {
                return @($documents)
            }
        }
        catch {
            # Fall through to JSONL parsing.
        }
    }

    return @(
        Read-JsonLines -Path $Path -Skip $SkipLines |
            Where-Object { Test-SnapshotDocument $_ }
    )
}

function Get-SnapshotPose {
    param(
        $Document,
        [string]$StableId
    )

    $items = $Document.bodies
    if ($null -eq $items) {
        $items = $Document.entities
    }
    foreach ($item in @($items)) {
        $itemId = [string]$item.stable_id
        if ([string]::IsNullOrWhiteSpace($itemId) -and $null -ne $item.id) {
            $itemId = [string]$item.id.stable_id
        }
        if ($itemId -ne $StableId) {
            continue
        }

        $transform = $item.transform
        $position = $item.position_m
        $quaternion = $item.quaternion_wxyz
        if ($null -eq $position -and $null -ne $transform) {
            $position = $transform.position_m
        }
        if ($null -eq $quaternion -and $null -ne $transform) {
            $quaternion = $transform.quaternion_wxyz
        }

        $positionValues = @($position)
        if ($positionValues.Count -ne 3) {
            return $null
        }
        $positionNumbers = @()
        foreach ($value in $positionValues) {
            $number = Get-NumberOrNull $value
            if ($null -eq $number) {
                return $null
            }
            $positionNumbers += $number
        }

        $quaternionNumbers = @()
        $quaternionValues = @($quaternion)
        if ($quaternionValues.Count -eq 4) {
            foreach ($value in $quaternionValues) {
                $number = Get-NumberOrNull $value
                if ($null -eq $number) {
                    $quaternionNumbers = @()
                    break
                }
                $quaternionNumbers += $number
            }
        }

        return [pscustomobject]@{
            position_m = $positionNumbers
            quaternion_wxyz = $quaternionNumbers
        }
    }
    return $null
}

function Get-SnapshotTimeNs {
    param($Document)

    foreach ($field in @("sim_time_ns", "timestamp_ns", "time_ns")) {
        $value = Get-NumberOrNull $Document.$field
        if ($null -ne $value -and $value -ge 0.0) {
            return [int64]$value
        }
    }
    return $null
}

function Get-UprightQualification {
    param(
        [string]$Path,
        [object[]]$Documents,
        [string]$StableId,
        [object]$HoldStartTimeNs,
        [object]$HoldEndTimeNs,
        [object]$ReleaseStartTimeNs,
        [string]$ProductSessionId,
        [object]$ExpectedModelGeneration,
        [object]$ExpectedResetGeneration,
        [int]$MinimumHoldSamples,
        [int]$MinimumReleaseSamples,
        [double]$MinimumBodyUpDot,
        [double]$MaximumAbsoluteRollRadians,
        [double]$MaximumAbsolutePitchRadians,
        [double]$MinimumBaseHeightMeters,
        [double]$MaximumBaseHeightDropMeters,
        [double]$QuaternionNormTolerance
    )

    $failureCodes = @()
    if ($null -eq $HoldStartTimeNs -or $null -eq $HoldEndTimeNs) {
        $failureCodes += "hold_time_bounds_missing"
    }
    if ($null -eq $ReleaseStartTimeNs) {
        $failureCodes += "release_time_boundary_missing"
    }

    $samples = @()
    foreach ($document in @($Documents)) {
        if ($null -eq $document) {
            continue
        }
        $timeNs = Get-SnapshotTimeNs $document
        $pose = Get-SnapshotPose -Document $document -StableId $StableId
        if ($null -eq $timeNs -or $null -eq $pose) {
            continue
        }

        $identityPassed = $true
        if ([string](Get-PropertyOrNull $document "session_id") -cne
            $ProductSessionId) {
            $failureCodes += "snapshot_session_id_mismatch"
            $identityPassed = $false
        }
        $modelGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $document "model_generation"
        )
        $resetGeneration = Get-NumberOrNull (
            Get-PropertyOrNull $document "reset_generation"
        )
        if ($null -eq $ExpectedModelGeneration -or
            $null -eq $modelGeneration -or
            [int64]$modelGeneration -ne [int64]$ExpectedModelGeneration) {
            $failureCodes += "snapshot_model_generation_mismatch"
            $identityPassed = $false
        }
        if ($null -eq $ExpectedResetGeneration -or
            $null -eq $resetGeneration -or
            [int64]$resetGeneration -ne [int64]$ExpectedResetGeneration) {
            $failureCodes += "snapshot_reset_generation_mismatch"
            $identityPassed = $false
        }

        $samples += [pscustomobject]@{
            time_ns = [int64]$timeNs
            pose = $pose
            identity_passed = $identityPassed
        }
    }
    $samples = @($samples | Sort-Object time_ns)

    $baselineCandidates = if ($null -ne $HoldStartTimeNs) {
        @($samples | Where-Object { $_.time_ns -le [int64]$HoldStartTimeNs })
    }
    else {
        @()
    }
    $baseline = if ($baselineCandidates.Count -gt 0) {
        $baselineCandidates[$baselineCandidates.Count - 1]
    }
    else {
        $null
    }
    if ($null -eq $baseline) {
        $failureCodes += "baseline_snapshot_missing"
    }

    $holdSamples = if ($null -ne $HoldStartTimeNs -and $null -ne $HoldEndTimeNs) {
        @(
            $samples | Where-Object {
                $_.time_ns -ge [int64]$HoldStartTimeNs -and
                $_.time_ns -le [int64]$HoldEndTimeNs
            }
        )
    }
    else {
        @()
    }
    if ($holdSamples.Count -lt $MinimumHoldSamples) {
        $failureCodes += "insufficient_hold_samples"
    }

    $releaseSamples = if ($null -ne $ReleaseStartTimeNs) {
        @(
            $samples | Where-Object {
                $_.time_ns -ge [int64]$ReleaseStartTimeNs
            }
        )
    }
    else {
        @()
    }
    if ($releaseSamples.Count -lt $MinimumReleaseSamples) {
        $failureCodes += "insufficient_release_samples"
    }

    $qualifiedSamples = @()
    if ($null -ne $baseline) {
        $qualifiedSamples += $baseline
    }
    $qualifiedSamples += $holdSamples
    $qualifiedSamples += $releaseSamples
    $qualifiedSamples = @($qualifiedSamples | Sort-Object time_ns -Unique)

    $minimumObservedUpDot = $null
    $maximumObservedRoll = $null
    $maximumObservedPitch = $null
    $minimumObservedHeight = $null
    foreach ($sample in $qualifiedSamples) {
        if (-not $sample.identity_passed) {
            continue
        }
        $height = Get-NumberOrNull $sample.pose.position_m[2]
        if ($null -eq $height) {
            $failureCodes += "invalid_base_height"
        }
        elseif ($null -eq $minimumObservedHeight -or $height -lt $minimumObservedHeight) {
            $minimumObservedHeight = $height
        }

        $quaternion = @($sample.pose.quaternion_wxyz)
        if ($quaternion.Count -ne 4) {
            $failureCodes += "invalid_quaternion"
            continue
        }
        $numbers = @()
        foreach ($value in $quaternion) {
            $number = Get-NumberOrNull $value
            if ($null -eq $number) {
                $numbers = @()
                break
            }
            $numbers += $number
        }
        if ($numbers.Count -ne 4) {
            $failureCodes += "invalid_quaternion"
            continue
        }
        $norm = [Math]::Sqrt(
            ($numbers | ForEach-Object { [double]$_ * [double]$_ } |
                Measure-Object -Sum).Sum
        )
        if ($norm -le 0.0 -or
            [Math]::Abs($norm - 1.0) -gt $QuaternionNormTolerance) {
            $failureCodes += "invalid_quaternion"
            continue
        }

        $w = [double]$numbers[0] / $norm
        $x = [double]$numbers[1] / $norm
        $y = [double]$numbers[2] / $norm
        $z = [double]$numbers[3] / $norm
        $upDot = 1.0 - (2.0 * (($x * $x) + ($y * $y)))
        $roll = [Math]::Atan2(
            2.0 * (($w * $x) + ($y * $z)),
            1.0 - (2.0 * (($x * $x) + ($y * $y)))
        )
        $pitchSin = 2.0 * (($w * $y) - ($z * $x))
        $pitchSin = [Math]::Max(-1.0, [Math]::Min(1.0, $pitchSin))
        $pitch = [Math]::Asin($pitchSin)
        $absoluteRoll = [Math]::Abs($roll)
        $absolutePitch = [Math]::Abs($pitch)

        if ($null -eq $minimumObservedUpDot -or $upDot -lt $minimumObservedUpDot) {
            $minimumObservedUpDot = $upDot
        }
        if ($null -eq $maximumObservedRoll -or $absoluteRoll -gt $maximumObservedRoll) {
            $maximumObservedRoll = $absoluteRoll
        }
        if ($null -eq $maximumObservedPitch -or $absolutePitch -gt $maximumObservedPitch) {
            $maximumObservedPitch = $absolutePitch
        }
    }

    $baselineHeight = if ($null -ne $baseline) {
        Get-NumberOrNull $baseline.pose.position_m[2]
    }
    else {
        $null
    }
    $maximumObservedHeightDrop = if ($null -ne $baselineHeight -and
        $null -ne $minimumObservedHeight) {
        [Math]::Max(0.0, $baselineHeight - $minimumObservedHeight)
    }
    else {
        $null
    }

    if ($null -eq $minimumObservedUpDot -or
        $minimumObservedUpDot -lt $MinimumBodyUpDot) {
        $failureCodes += "minimum_body_up_dot"
    }
    if ($null -eq $maximumObservedRoll -or
        $maximumObservedRoll -gt $MaximumAbsoluteRollRadians) {
        $failureCodes += "maximum_absolute_roll"
    }
    if ($null -eq $maximumObservedPitch -or
        $maximumObservedPitch -gt $MaximumAbsolutePitchRadians) {
        $failureCodes += "maximum_absolute_pitch"
    }
    if ($null -eq $minimumObservedHeight -or
        $minimumObservedHeight -lt $MinimumBaseHeightMeters) {
        $failureCodes += "minimum_base_height"
    }
    if ($null -eq $maximumObservedHeightDrop -or
        $maximumObservedHeightDrop -gt $MaximumBaseHeightDropMeters) {
        $failureCodes += "maximum_base_height_drop"
    }

    $failureCodes = @($failureCodes | Select-Object -Unique)
    $passed = $failureCodes.Count -eq 0
    return [ordered]@{
        state = if ($passed) { "passed" } else { "failed" }
        passed = $passed
        source_path = if ($Path) { $Path } else { $null }
        stable_id = $StableId
        samples = $samples.Count
        qualified_samples = $qualifiedSamples.Count
        hold_samples = $holdSamples.Count
        minimum_hold_samples = $MinimumHoldSamples
        release_samples = $releaseSamples.Count
        minimum_release_samples = $MinimumReleaseSamples
        baseline_height_m = $baselineHeight
        minimum_base_height_m = $minimumObservedHeight
        minimum_base_height_m_allowed = $MinimumBaseHeightMeters
        maximum_base_height_drop_m = $maximumObservedHeightDrop
        maximum_base_height_drop_m_allowed = $MaximumBaseHeightDropMeters
        minimum_body_up_dot = $minimumObservedUpDot
        minimum_body_up_dot_allowed = $MinimumBodyUpDot
        maximum_absolute_roll_rad = $maximumObservedRoll
        maximum_absolute_roll_rad_allowed = $MaximumAbsoluteRollRadians
        maximum_absolute_pitch_rad = $maximumObservedPitch
        maximum_absolute_pitch_rad_allowed = $MaximumAbsolutePitchRadians
        quaternion_norm_tolerance = $QuaternionNormTolerance
        expected_session_id = $ProductSessionId
        expected_model_generation = $ExpectedModelGeneration
        expected_reset_generation = $ExpectedResetGeneration
        hold_start_time_ns = $HoldStartTimeNs
        hold_end_time_ns = $HoldEndTimeNs
        release_start_time_ns = $ReleaseStartTimeNs
        failure_codes = $failureCodes
        reason = if ($passed) {
            "base truth remained upright and above the height gates through hold and release"
        }
        else {
            "base truth did not remain upright, elevated, and generation-consistent"
        }
    }
}

function Get-SnapshotQualification {
    param(
        [string]$Path,
        [object[]]$Documents,
        [string]$StableId,
        [object]$HoldStartTimeNs,
        [object]$HoldEndTimeNs,
        [double]$MinimumPositionDisplacementMeters,
        [double]$MinimumYawDeltaRadians,
        [bool]$RequirePositionDisplacement = $false,
        [ValidateSet("W", "S", "A", "D", "Q", "E")]
        [string]$DirectionKey = "W"
    )

    $poses = @()
    foreach ($document in @($Documents)) {
        if ($null -eq $document) {
            continue
        }
        $timeNs = Get-SnapshotTimeNs $document
        $pose = Get-SnapshotPose -Document $document -StableId $StableId
        if ($null -ne $timeNs -and $null -ne $pose) {
            $poses += [pscustomobject]@{
                time_ns = $timeNs
                pose = $pose
            }
        }
    }
    $poses = @($poses | Sort-Object time_ns)
    $baseResult = [ordered]@{
        state = "not_evaluated"
        passed = $null
        source_path = if ($Path) { $Path } else { $null }
        stable_id = $StableId
        samples = $poses.Count
        hold_samples = 0
        reason = "no timed runtime snapshot pose was available"
    }
    if ($poses.Count -lt 2) {
        return $baseResult
    }
    if ($null -eq $HoldStartTimeNs -or $null -eq $HoldEndTimeNs) {
        $baseResult.reason = "hold phase has no simulation-time bounds for snapshot correlation"
        return $baseResult
    }

    $holdPoses = @(
        $poses | Where-Object {
            $_.time_ns -ge [int64]$HoldStartTimeNs -and
            $_.time_ns -le [int64]$HoldEndTimeNs
        }
    )
    $baseResult.hold_samples = $holdPoses.Count
    if ($holdPoses.Count -lt 2) {
        $baseResult.reason = "runtime snapshot poses did not cover the accepted hold interval"
        return $baseResult
    }

    $first = $holdPoses[0].pose
    $last = $holdPoses[$holdPoses.Count - 1].pose
    $firstPosition = @($first.position_m)
    $lastPosition = @($last.position_m)
    $dx = [double]$lastPosition[0] - [double]$firstPosition[0]
    $dy = [double]$lastPosition[1] - [double]$firstPosition[1]
    $dz = [double]$lastPosition[2] - [double]$firstPosition[2]
    $positionDisplacement = [Math]::Sqrt(($dx * $dx) + ($dy * $dy) + ($dz * $dz))

    $orientationDelta = $null
    $yawDelta = $null
    $firstQuaternion = @($first.quaternion_wxyz)
    $lastQuaternion = @($last.quaternion_wxyz)
    if ($firstQuaternion.Count -eq 4 -and $lastQuaternion.Count -eq 4) {
        $firstNorm = [Math]::Sqrt(($firstQuaternion | ForEach-Object { [double]$_ * [double]$_ } | Measure-Object -Sum).Sum)
        $lastNorm = [Math]::Sqrt(($lastQuaternion | ForEach-Object { [double]$_ * [double]$_ } | Measure-Object -Sum).Sum)
        if ($firstNorm -gt 0.0 -and $lastNorm -gt 0.0) {
            $dot = 0.0
            for ($index = 0; $index -lt 4; $index++) {
                $dot += ([double]$firstQuaternion[$index] * [double]$lastQuaternion[$index])
            }
            $dot = [Math]::Min(1.0, [Math]::Abs($dot / ($firstNorm * $lastNorm)))
            $orientationDelta = 2.0 * [Math]::Acos($dot)
            $firstYaw = [Math]::Atan2(2.0 * ($firstQuaternion[0] * $firstQuaternion[3] + $firstQuaternion[1] * $firstQuaternion[2]),
                $firstNorm * $firstNorm - 2.0 * ($firstQuaternion[2] * $firstQuaternion[2] + $firstQuaternion[3] * $firstQuaternion[3]))
            $lastYaw = [Math]::Atan2(2.0 * ($lastQuaternion[0] * $lastQuaternion[3] + $lastQuaternion[1] * $lastQuaternion[2]),
                $lastNorm * $lastNorm - 2.0 * ($lastQuaternion[2] * $lastQuaternion[2] + $lastQuaternion[3] * $lastQuaternion[3]))
            $yawDelta = [Math]::Atan2([Math]::Sin($lastYaw - $firstYaw), [Math]::Cos($lastYaw - $firstYaw))
        }
    }

    $positionPassed = $positionDisplacement -ge $MinimumPositionDisplacementMeters
    $orientationPassed = $null -ne $yawDelta -and
        [Math]::Abs($yawDelta) -ge $MinimumYawDeltaRadians
    $movementPassed = if ($DirectionKey -in @("Q", "E")) {
        $directedYaw = if ($DirectionKey -eq "E") { -$yawDelta } else { $yawDelta }
        $null -ne $yawDelta -and $directedYaw -ge $MinimumYawDeltaRadians
    } elseif ($RequirePositionDisplacement) {
        $positionPassed
    }
    else {
        $positionPassed -or $orientationPassed
    }
    $baseResult.state = if ($movementPassed) { "passed" } else { "failed" }
    $baseResult.passed = [bool]$movementPassed
    $baseResult.reason = if ($baseResult.passed) {
        "runtime snapshot pose changed during the accepted hold interval"
    }
    else {
        "runtime snapshot pose did not exceed the configured movement thresholds"
    }
    $baseResult.position_displacement_m = $positionDisplacement
    $baseResult.minimum_position_displacement_m = $MinimumPositionDisplacementMeters
    $baseResult.position_displacement_required = $RequirePositionDisplacement
    $baseResult.orientation_delta_rad = $orientationDelta
    $baseResult.yaw_delta_rad = $yawDelta
    $baseResult.minimum_orientation_delta_rad = $MinimumYawDeltaRadians
    $baseResult.hold_start_time_ns = [int64]$HoldStartTimeNs
    $baseResult.hold_end_time_ns = [int64]$HoldEndTimeNs
    return $baseResult
}

function Get-PostReleaseQualification {
    param(
        [string]$Path,
        [object[]]$Documents,
        [string]$StableId,
        [object]$ReleaseStartTimeNs,
        [double]$MaximumDriftMeters,
        [double]$MaximumSpeedMps
    )

    $result = [ordered]@{
        state = "not_evaluated"
        passed = $null
        source_path = if ($Path) { $Path } else { $null }
        stable_id = $StableId
        samples = 0
        duration_seconds = 0.0
        reason = "no timed runtime snapshot pose covered the release interval"
    }
    if ($null -eq $ReleaseStartTimeNs) {
        $result.reason = "release phase has no simulation-time boundary"
        return $result
    }

    $poses = @()
    foreach ($document in @($Documents)) {
        if ($null -eq $document) {
            continue
        }
        $timeNs = Get-SnapshotTimeNs $document
        $pose = Get-SnapshotPose -Document $document -StableId $StableId
        if ($null -ne $timeNs -and
            $timeNs -ge [int64]$ReleaseStartTimeNs -and
            $null -ne $pose) {
            $poses += [pscustomobject]@{
                time_ns = [int64]$timeNs
                pose = $pose
            }
        }
    }
    $poses = @($poses | Sort-Object time_ns)
    $result.samples = $poses.Count
    if ($poses.Count -lt 3) {
        return $result
    }

    $first = $poses[0]
    $maximumDrift = 0.0
    foreach ($sample in $poses) {
        $dx = [double]$sample.pose.position_m[0] - [double]$first.pose.position_m[0]
        $dy = [double]$sample.pose.position_m[1] - [double]$first.pose.position_m[1]
        $maximumDrift = [Math]::Max(
            $maximumDrift,
            [Math]::Sqrt(($dx * $dx) + ($dy * $dy))
        )
    }

    $previous = $poses[$poses.Count - 2]
    $last = $poses[$poses.Count - 1]
    $finalDeltaSeconds = ($last.time_ns - $previous.time_ns) / 1000000000.0
    $finalSpeed = [double]::PositiveInfinity
    if ($finalDeltaSeconds -gt 0.0) {
        $finalDx = [double]$last.pose.position_m[0] - [double]$previous.pose.position_m[0]
        $finalDy = [double]$last.pose.position_m[1] - [double]$previous.pose.position_m[1]
        $finalSpeed = [Math]::Sqrt(($finalDx * $finalDx) + ($finalDy * $finalDy)) /
            $finalDeltaSeconds
    }

    $durationSeconds = ($last.time_ns - $first.time_ns) / 1000000000.0
    $durationPassed = $durationSeconds -ge 0.5
    $driftPassed = $maximumDrift -le $MaximumDriftMeters
    $speedPassed = $finalSpeed -le $MaximumSpeedMps
    $result.duration_seconds = $durationSeconds
    $result.maximum_planar_drift_m = $maximumDrift
    $result.maximum_planar_drift_m_allowed = $MaximumDriftMeters
    $result.final_planar_speed_mps = $finalSpeed
    $result.maximum_final_planar_speed_mps = $MaximumSpeedMps
    $result.release_start_time_ns = [int64]$ReleaseStartTimeNs
    $result.state = if ($durationPassed -and $driftPassed -and $speedPassed) {
        "passed"
    }
    else {
        "failed"
    }
    $result.passed = $result.state -eq "passed"
    $result.reason = if (-not $durationPassed) {
        "release snapshots covered less than 0.5 seconds"
    }
    elseif (-not $driftPassed) {
        "robot continued drifting beyond the configured release threshold"
    }
    elseif (-not $speedPassed) {
        "robot retained excessive speed after release"
    }
    else {
        "robot settled within the configured drift and speed thresholds"
    }
    return $result
}

function Get-DirectionKeyDefinition {
    param(
        [ValidateSet("W", "S", "A", "D", "Q", "E")]
        [string]$DirectionKey
    )

    $definitions = @{
        W = [pscustomobject]@{ VirtualKey = 0x57; ScanCode = 0x11 }
        S = [pscustomobject]@{ VirtualKey = 0x53; ScanCode = 0x1F }
        A = [pscustomobject]@{ VirtualKey = 0x41; ScanCode = 0x1E }
        D = [pscustomobject]@{ VirtualKey = 0x44; ScanCode = 0x20 }
        Q = [pscustomobject]@{ VirtualKey = 0x51; ScanCode = 0x10 }
        E = [pscustomobject]@{ VirtualKey = 0x45; ScanCode = 0x12 }
    }
    return $definitions[$DirectionKey]
}

function Send-ForegroundDriveInput {
    param(
        [int]$HoldMilliseconds,
        [int]$ReleaseSettleMilliseconds,
        [ValidateSet("W", "S", "A", "D", "Q", "E")]
        [string]$DirectionKey
    )

    $key = Get-DirectionKeyDefinition -DirectionKey $DirectionKey
    $keyDown = $false
    try {
        [LingTuDriveProbeNative]::keybd_event(
            $key.VirtualKey, $key.ScanCode, 0, [UIntPtr]::Zero)
        $keyDown = $true
        Start-Sleep -Milliseconds $HoldMilliseconds

        [LingTuDriveProbeNative]::keybd_event(
            $key.VirtualKey, $key.ScanCode, 0x0002, [UIntPtr]::Zero)
        $keyDown = $false
        Start-Sleep -Milliseconds $ReleaseSettleMilliseconds
    }
    finally {
        if ($keyDown) {
            [LingTuDriveProbeNative]::keybd_event(
                $key.VirtualKey, $key.ScanCode, 0x0002, [UIntPtr]::Zero)
        }
    }
}

function ConvertTo-ProbeUIntPtr {
    param([uint64]$Value)

    return [UIntPtr]::new($Value)
}

function Send-TargetedDriveInput {
    param(
        [IntPtr]$WindowHandle,
        [int]$HoldMilliseconds,
        [int]$ReleaseSettleMilliseconds,
        [ValidateSet("W", "S", "A", "D", "Q", "E")]
        [string]$DirectionKey
    )

    $wmKeyDown = 0x0100
    $wmKeyUp = 0x0101
    $keyDefinition = Get-DirectionKeyDefinition -DirectionKey $DirectionKey
    $key = ConvertTo-ProbeUIntPtr -Value $keyDefinition.VirtualKey
    $keyDown = $false
    try {
        if (-not [LingTuDriveProbeNative]::PostMessage(
                $WindowHandle, $wmKeyDown, $key, [IntPtr]1)) {
            throw "Targeted $DirectionKey keydown could not be posted"
        }
        $keyDown = $true

        $deadline = [DateTime]::UtcNow.AddMilliseconds($HoldMilliseconds)
        while ([DateTime]::UtcNow -lt $deadline) {
            # Bit 30 marks an autorepeat, so the input processor sees one press
            # followed by a hold rather than a stream of fresh presses.
            if (-not [LingTuDriveProbeNative]::PostMessage(
                    $WindowHandle, $wmKeyDown, $key, [IntPtr]0x40000001)) {
                throw "Targeted $DirectionKey repeat could not be posted"
            }
            Start-Sleep -Milliseconds 40
        }

        if (-not [LingTuDriveProbeNative]::PostMessage(
                $WindowHandle, $wmKeyUp, $key, [IntPtr]0xC0000001)) {
            throw "Targeted $DirectionKey keyup could not be posted"
        }
        $keyDown = $false
        Start-Sleep -Milliseconds $ReleaseSettleMilliseconds
    }
    finally {
        if ($keyDown) {
            [LingTuDriveProbeNative]::PostMessage(
                $WindowHandle, $wmKeyUp, $key, [IntPtr]0xC0000001) | Out-Null
        }
    }
}

$script:LingTuDriveProbeStage = "contract"
trap {
    $message = [string]$_.Exception.Message
    [Console]::Error.WriteLine(
        "LINGTU_DRIVE_PROBE_FAILED stage={0} error={1}",
        $script:LingTuDriveProbeStage,
        $message
    )
    exit 1
}

Assert-LocomotionProbeContract `
    -RequireLocomotion ([bool]$RequireLocomotion) `
    -InputMethod $InputMethod `
    -AllowTargetedLocomotionDiagnostic ([bool]$AllowTargetedLocomotionDiagnostic) `
    -RunDirectory $RunDirectory `
    -ExpectedRunId $ExpectedRunId `
    -ProductSessionId $ProductSessionId | Out-Null

$script:LingTuDriveProbeStage = "window_selection"
$process = Get-Process -Id $ProcessId -ErrorAction Stop
$windowCandidate = Get-RobotSimWindowCandidate -ProcessId ([uint32]$ProcessId)
$window = [IntPtr]([int64]$windowCandidate.Handle)
$windowTitle = [string]$windowCandidate.Title

[LingTuDriveProbeNative]::ShowWindow($window, 9) | Out-Null
Start-Sleep -Milliseconds 300

if ([string]::IsNullOrWhiteSpace($RunDirectory)) {
    $latestPath = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "..\manual-playable-runs\latest.json"
    $latestPath = [System.IO.Path]::GetFullPath($latestPath)
    if (Test-Path -LiteralPath $latestPath) {
        $RunDirectory = (Get-Content -LiteralPath $latestPath -Raw | ConvertFrom-Json).run_dir
    }
}
if (-not [string]::IsNullOrWhiteSpace($RunDirectory)) {
    $RunDirectory = [System.IO.Path]::GetFullPath($RunDirectory)
}
$acceptedPath = if ($RunDirectory) { Join-Path $RunDirectory "control-intent-accepted.jsonl" } else { "" }
$authorityPath = if ($RunDirectory) { Join-Path $RunDirectory "control-status-authority.jsonl" } else { "" }
$receivedPath = if ($RunDirectory) { Join-Path $RunDirectory "control-intent-received.jsonl" } else { "" }
$zeroAuditPath = if ($RunDirectory) { Join-Path $RunDirectory "control-command-zero-audit.jsonl" } else { "" }
$originPath = if ($RunDirectory) { Join-Path $RunDirectory "logs/ue-control-origin.jsonl" } else { "" }
$runtimeRequestTracePath = if ($RunDirectory) { Join-Path $RunDirectory "runtime-request-trace.jsonl" } else { "" }
$runtimePath = if ($RunDirectory) { Join-Path $RunDirectory "session.runtime.json" } else { "" }
$allocationPath = if ($RunDirectory) { Join-Path $RunDirectory "run-allocation.json" } else { "" }
$healthPath = if ($RunDirectory) { Join-Path $RunDirectory "manual-runtime-health.json" } else { "" }
$runAllocation = Read-OptionalJson $allocationPath
$runtimeBefore = Read-OptionalJson $runtimePath
$healthBefore = Read-OptionalJson $healthPath
$sessionStateBefore = if ($null -ne $runtimeBefore) { [string]$runtimeBefore.state } else { "unavailable" }

$acceptedBefore = Get-FileLineCount $acceptedPath
$authorityBefore = Get-FileLineCount $authorityPath
$receivedBefore = Get-FileLineCount $receivedPath
$zeroAuditBefore = Get-FileLineCount $zeroAuditPath
$originBefore = Get-FileLineCount $originPath
$runtimeRequestBefore = Get-FileLineCount $runtimeRequestTracePath
$snapshotPathResolved = Resolve-SnapshotPath -RequestedPath $SnapshotPath -RunDirectory $RunDirectory
$snapshotBefore = if ($snapshotPathResolved -and
    [System.IO.Path]::GetExtension($snapshotPathResolved) -eq ".jsonl") {
    Get-FileLineCount $snapshotPathResolved
}
else {
    0
}

$outputDirectory = if ($RunDirectory) {
    Join-Path $RunDirectory "drive-probe"
}
else {
    Split-Path -Parent $MyInvocation.MyCommand.Path
}
if (-not (Test-Path -LiteralPath $outputDirectory -PathType Container)) {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
}
$baselineA = Join-Path $outputDirectory "probe-baseline-a.png"
$baselineB = Join-Path $outputDirectory "probe-baseline-b.png"
$driven = Join-Path $outputDirectory "probe-driven.png"

$script:LingTuDriveProbeStage = "baseline_capture"
Save-WindowCapture -WindowHandle $window -Path $baselineA
Start-Sleep -Milliseconds 1000
Save-WindowCapture -WindowHandle $window -Path $baselineB

$foregroundBeforeInput = Get-ProbeForegroundWindowHandle
if ($InputMethod -eq "ForegroundInput") {
    $script:LingTuDriveProbeStage = "foreground_acquisition"
    $foregroundBeforeInput = Acquire-ProbeForegroundWindow -WindowHandle $window
    $focusRect = New-Object LingTuDriveProbeNative+RECT
    [LingTuDriveProbeNative]::GetWindowRect($window, [ref]$focusRect) | Out-Null
    $focusX = [int](($focusRect.Left + $focusRect.Right) / 2)
    $focusY = [int](($focusRect.Top + $focusRect.Bottom) / 2)
    [LingTuDriveProbeNative]::SetCursorPos($focusX, $focusY) | Out-Null
    [LingTuDriveProbeNative]::mouse_event(0x0002, 0, 0, 0, [UIntPtr]::Zero)
    [LingTuDriveProbeNative]::mouse_event(0x0004, 0, 0, 0, [UIntPtr]::Zero)
    Start-Sleep -Milliseconds 300
    $script:LingTuDriveProbeStage = "input_delivery"
    Send-ForegroundDriveInput -HoldMilliseconds $HoldMilliseconds `
        -ReleaseSettleMilliseconds $ReleaseSettleMilliseconds `
        -DirectionKey $DirectionKey
}
else {
    # Targeted messages avoid sending the diagnostic input to another desktop
    # window while preserving distinct press, repeat/hold, and release phases.
    $script:LingTuDriveProbeStage = "input_delivery"
    Send-TargetedDriveInput -WindowHandle $window -HoldMilliseconds $HoldMilliseconds `
        -ReleaseSettleMilliseconds $ReleaseSettleMilliseconds `
        -DirectionKey $DirectionKey
}
Start-Sleep -Milliseconds 700
Save-WindowCapture -WindowHandle $window -Path $driven

$script:LingTuDriveProbeStage = "evidence_validation"
$newAccepted = Read-JsonLines -Path $acceptedPath -Skip $acceptedBefore
$newAuthority = Read-JsonLines -Path $authorityPath -Skip $authorityBefore
$newReceived = Read-JsonLines -Path $receivedPath -Skip $receivedBefore
$newZeroAudit = Read-JsonLines -Path $zeroAuditPath -Skip $zeroAuditBefore
$newOrigin = Read-JsonLines -Path $originPath -Skip $originBefore
$runtimeRequestRecords = Read-JsonLines -Path $runtimeRequestTracePath -Skip $runtimeRequestBefore
$snapshotDocuments = Read-SnapshotDocuments -Path $snapshotPathResolved -SkipLines $snapshotBefore

$authorityByHash = @{}
foreach ($status in $newAuthority) {
    $hash = [string]$status.intent_datagram_sha256
    if ([string]::IsNullOrWhiteSpace($hash)) {
        continue
    }
    if (-not $authorityByHash.ContainsKey($hash)) {
        $authorityByHash[$hash] = @()
    }
    $authorityByHash[$hash] += $status
}

$originByHash = @{}
foreach ($origin in $newOrigin) {
    $hash = [string]$origin.datagram_sha256
    if ([string]::IsNullOrWhiteSpace($hash)) {
        continue
    }
    if (-not $originByHash.ContainsKey($hash)) {
        $originByHash[$hash] = @()
    }
    $originByHash[$hash] += $origin
}
$acceptedControlClaims = @(Select-AcceptedControlClaims `
    -Records $runtimeRequestRecords `
    -ExpectedRunId $ExpectedRunId `
    -ProductSessionId $ProductSessionId `
    -ExpectedBootId ([string]$runAllocation.boot_id) `
    -ExpectedModelGeneration (Get-PropertyOrNull $runtimeBefore "model_generation") `
    -ExpectedResetGeneration (Get-PropertyOrNull $runtimeBefore "reset_generation"))

$acceptedEvidence = @()
$acceptedIndex = 0
foreach ($record in $newAccepted) {
    $hash = [string]$record.datagram_sha256
    $authority = if ($authorityByHash.ContainsKey($hash)) {
        @($authorityByHash[$hash])
    }
    else {
        @()
    }
    $heldAuthority = @(
        $authority | Where-Object {
            $_.status -eq "accepted" -and
            $_.runtime.deadman -eq $true
        }
    )
    $origin = if ($originByHash.ContainsKey($hash)) {
        @($originByHash[$hash])
    }
    else {
        @()
    }
    $acceptedEvidence += [pscustomobject]@{
        index = $acceptedIndex
        record = $record
        hash = $hash
        time_ns = Get-RecordTimeNs $record
        is_nonzero = Test-NonZeroTwist $record.admitted_twist
        is_expected_direction = Test-ExpectedDirectionTwist `
            -Twist $record.admitted_twist `
            -DirectionKey $DirectionKey
        authority = $authority
        held_authority_count = $heldAuthority.Count
        origin_count = @($origin).Count
    }
    $acceptedIndex += 1
}

$nonzeroAccepted = @($acceptedEvidence | Where-Object { $_.is_nonzero })
$unexpectedNonzeroAccepted = @(
    $nonzeroAccepted | Where-Object { -not $_.is_expected_direction }
)

$holdCandidates = @(
    $acceptedEvidence | Where-Object {
        $_.is_expected_direction -and $_.held_authority_count -gt 0
    }
)
$holdTimedCandidates = @(
    $holdCandidates |
        Where-Object { $null -ne $_.time_ns } |
        Sort-Object time_ns
)
$holdStartTimeNs = if ($holdTimedCandidates.Count -gt 0) {
    [int64]$holdTimedCandidates[0].time_ns
}
else {
    $null
}
$holdEndTimeNs = if ($holdTimedCandidates.Count -gt 0) {
    [int64]$holdTimedCandidates[$holdTimedCandidates.Count - 1].time_ns
}
else {
    $null
}
$holdWindowMilliseconds = if ($null -ne $holdStartTimeNs -and $null -ne $holdEndTimeNs) {
    ($holdEndTimeNs - $holdStartTimeNs) / 1000000.0
}
else {
    0.0
}
$holdSamplesPassed = $holdCandidates.Count -ge $MinimumHoldSamples
$holdWindowPassed = $MinimumHoldWindowMilliseconds -eq 0 -or
    ($holdTimedCandidates.Count -ge 2 -and
        $holdWindowMilliseconds -ge $MinimumHoldWindowMilliseconds)
$holdOriginMatches = @(
    $holdCandidates | Where-Object {
        $_.origin_count -gt 0
    }
)
$holdOriginPassed = $holdCandidates.Count -gt 0 -and
    $holdOriginMatches.Count -eq $holdCandidates.Count
$pressAnchor = if ($holdCandidates.Count -gt 0) {
    $holdCandidates[0]
}
else {
    $null
}
$pressClaimMatches = @()
$pressOriginMatches = @()
if ($null -ne $pressAnchor) {
    $pressClaimMatches = @(
        $acceptedControlClaims | Where-Object {
            Test-PressClaimCorrelation -PressRecord $pressAnchor.record -ClaimRecord $_
        }
    )
    if ($pressAnchor.origin_count -gt 0) {
        $pressOriginMatches = @($pressAnchor)
    }
}

$lastHoldSourceSequence = $null
if ($holdCandidates.Count -gt 0) {
    $lastHoldRecord = $holdCandidates[$holdCandidates.Count - 1].record
    $lastHoldSourceSequence = Get-NumberOrNull $lastHoldRecord.source_sequence
}

$releaseRequests = @(
    $runtimeRequestRecords | Where-Object {
        $sourceSequence = Get-NumberOrNull (Get-PropertyOrNull $_ "source_sequence")
        [string](Get-PropertyOrNull $_ "event") -eq "runtime_request_accepted" -and
        [string](Get-PropertyOrNull $_ "request") -eq "control_release" -and
        [string](Get-PropertyOrNull $_ "status") -eq "pending" -and
        [string](Get-PropertyOrNull $_ "reason") -eq "safe_zero_pending:control_release" -and
        [string](Get-PropertyOrNull $_ "run_id") -eq $ExpectedRunId -and
        [string](Get-PropertyOrNull $_ "session_id") -eq $ProductSessionId -and
        [string](Get-PropertyOrNull $_ "boot_id") -eq [string]$runAllocation.boot_id -and
        (Get-NumberOrNull (Get-PropertyOrNull $_ "model_generation")) -eq
            (Get-NumberOrNull (Get-PropertyOrNull $runtimeBefore "model_generation")) -and
        (Get-NumberOrNull (Get-PropertyOrNull $_ "reset_generation")) -eq
            (Get-NumberOrNull (Get-PropertyOrNull $runtimeBefore "reset_generation")) -and
        $null -ne $lastHoldSourceSequence -and
        $null -ne $sourceSequence -and
        $sourceSequence -gt $lastHoldSourceSequence
    }
)
$releaseHashes = @{}
foreach ($releaseRequest in $releaseRequests) {
    $hash = [string]$releaseRequest.datagram_sha256
    if (-not [string]::IsNullOrWhiteSpace($hash)) {
        $releaseHashes[$hash] = $releaseRequest
    }
}
$releaseAuthority = @(
    $newAuthority | Where-Object {
        $hash = [string]$_.intent_datagram_sha256
        $releaseHashes.ContainsKey($hash) -and
        $_.runtime.deadman -eq $false -and
        $_.status -eq "released" -and
        [string]$_.reason -eq "runtime_request:control_release" -and
        (Test-ZeroTwist $_.motion.admitted_twist_mps_radps)
    }
)
$zeroAuditAfterHold = @()
foreach ($zero in $newZeroAudit) {
    $hash = [string]$zero.datagram_sha256
    if ($releaseHashes.ContainsKey($hash) -and
        [string]$zero.reason -eq "runtime_request:control_release" -and
        [string]$zero.submit_result -eq "accepted" -and
        (Test-ZeroTwist $zero.admitted_twist)) {
        $zeroAuditAfterHold += $zero
    }
}
$releaseOriginMatches = @()
foreach ($zero in $zeroAuditAfterHold) {
    $hash = [string]$zero.datagram_sha256
    if ($originByHash.ContainsKey($hash)) {
        $releaseOriginMatches += $zero
    }
}
$releaseStartCandidates = @(
    $zeroAuditAfterHold |
        ForEach-Object { Get-NumberOrNull $_.apply_time_ns } |
        Where-Object { $null -ne $_ } |
        Sort-Object
)
$releaseStartTimeNs = if ($releaseStartCandidates.Count -gt 0) {
    [int64]$releaseStartCandidates[0]
}
else {
    $authorityReleaseTimes = @(
        $releaseAuthority |
            ForEach-Object { Get-NumberOrNull $_.sim_time_ns } |
            Where-Object { $null -ne $_ } |
            Sort-Object
    )
    if ($authorityReleaseTimes.Count -gt 0) {
        [int64]$authorityReleaseTimes[0]
    }
    else {
        $null
    }
}

$inputReceived = $newReceived.Count -gt 0
$pressPassed = $null -ne $pressAnchor -and
    $pressAnchor.held_authority_count -gt 0 -and
    $pressClaimMatches.Count -eq 1 -and
    $pressOriginMatches.Count -eq 1
$holdPassed = $holdSamplesPassed -and $holdWindowPassed -and $holdOriginPassed -and
    $unexpectedNonzeroAccepted.Count -eq 0
$releasePassed = $releaseRequests.Count -eq 1 -and
    $releaseAuthority.Count -gt 0 -and
    $zeroAuditAfterHold.Count -gt 0 -and
    $releaseOriginMatches.Count -gt 0

$processStillRunning = $false
try {
    $process.Refresh()
    $processStillRunning = -not $process.HasExited
}
catch {
    $processStillRunning = $false
}

$holdAuthorityStatuses = @()
foreach ($candidate in $holdCandidates) {
    $holdAuthorityStatuses += @($candidate.authority)
}
$maxObservedLinearSpeed = 0.0
$maxObservedAngularSpeed = 0.0
foreach ($status in $holdAuthorityStatuses) {
    $speed = Get-PlanarSpeed $status.motion.observed_base_velocity_mps_radps
    $maxObservedLinearSpeed = [Math]::Max($maxObservedLinearSpeed, $speed)
    $angularSpeed = Get-NumberOrNull $status.motion.observed_base_velocity_mps_radps.angular_z
    if ($null -ne $angularSpeed) {
        $maxObservedAngularSpeed = [Math]::Max($maxObservedAngularSpeed, [Math]::Abs($angularSpeed))
    }
}

$runtimeAfter = Read-OptionalJson $runtimePath
$healthAfter = Read-OptionalJson $healthPath
$sessionStateAfter = if ($null -ne $runtimeAfter) { [string]$runtimeAfter.state } else { "unavailable" }
$runIdentityQualification = [ordered]@{
    state = "not_evaluated"
    passed = $null
    failure_codes = @()
    reason = "exact run identity is required only for locomotion qualification"
}
$uprightQualification = [ordered]@{
    state = "not_evaluated"
    passed = $null
    failure_codes = @()
    reason = "upright truth is required only for locomotion qualification"
}
if ($RequireLocomotion) {
    $controlEvidenceRecords = @()
    $controlEvidenceRecords += @($newAccepted)
    $controlEvidenceRecords += @($newAuthority)
    $controlEvidenceRecords += @($newReceived)
    $controlEvidenceRecords += @($newZeroAudit)
    $controlEvidenceRecords += @($newOrigin)
    $runIdentityQualification = Get-RunIdentityQualification `
        -RunDirectory $RunDirectory `
        -ExpectedRunId $ExpectedRunId `
        -ProductSessionId $ProductSessionId `
        -Allocation $runAllocation `
        -RuntimeBefore $runtimeBefore `
        -RuntimeAfter $runtimeAfter `
        -HealthBefore $healthBefore `
        -HealthAfter $healthAfter `
        -ControlEvidenceRecords $controlEvidenceRecords `
        -SnapshotDocuments $snapshotDocuments

    $expectedModelGeneration = Get-NumberOrNull (
        Get-PropertyOrNull $runtimeBefore "model_generation"
    )
    $expectedResetGeneration = Get-NumberOrNull (
        Get-PropertyOrNull $runtimeBefore "reset_generation"
    )
    $uprightQualification = Get-UprightQualification `
        -Path $snapshotPathResolved `
        -Documents $snapshotDocuments `
        -StableId $SnapshotStableId `
        -HoldStartTimeNs $holdStartTimeNs `
        -HoldEndTimeNs $holdEndTimeNs `
        -ReleaseStartTimeNs $releaseStartTimeNs `
        -ProductSessionId $ProductSessionId `
        -ExpectedModelGeneration $expectedModelGeneration `
        -ExpectedResetGeneration $expectedResetGeneration `
        -MinimumHoldSamples $MinimumUprightHoldSamples `
        -MinimumReleaseSamples $MinimumUprightReleaseSamples `
        -MinimumBodyUpDot $MinimumBodyUpDot `
        -MaximumAbsoluteRollRadians $MaximumAbsoluteRollRadians `
        -MaximumAbsolutePitchRadians $MaximumAbsolutePitchRadians `
        -MinimumBaseHeightMeters $MinimumBaseHeightMeters `
        -MaximumBaseHeightDropMeters $MaximumBaseHeightDropMeters `
        -QuaternionNormTolerance $QuaternionNormTolerance
}
$snapshotQualification = Get-SnapshotQualification `
    -Path $snapshotPathResolved `
    -Documents $snapshotDocuments `
    -StableId $SnapshotStableId `
    -HoldStartTimeNs $holdStartTimeNs `
    -HoldEndTimeNs $holdEndTimeNs `
    -MinimumPositionDisplacementMeters $MinimumPositionDisplacementMeters `
    -MinimumYawDeltaRadians $MinimumYawDeltaRadians `
    -RequirePositionDisplacement ([bool]$RequireLocomotion -and $DirectionKey -notin @("Q", "E")) `
    -DirectionKey $DirectionKey
$releaseSnapshotQualification = Get-PostReleaseQualification `
    -Path $snapshotPathResolved `
    -Documents $snapshotDocuments `
    -StableId $SnapshotStableId `
    -ReleaseStartTimeNs $releaseStartTimeNs `
    -MaximumDriftMeters $MaximumPostReleaseDriftMeters `
    -MaximumSpeedMps $MaximumPostReleaseSpeedMps
if (-not $snapshotPathResolved) {
    $snapshotQualification.reason =
        "no persisted runtime snapshot path was found; control authority exposes velocity only"
    $releaseSnapshotQualification.reason =
        "no persisted runtime snapshot path was found; physical release settling cannot be measured"
}

$baselineChange = Measure-ImageChange -BeforePath $baselineA -AfterPath $baselineB
$drivenChange = Measure-ImageChange -BeforePath $baselineB -AfterPath $driven
$minimumMotionRatio = [Math]::Max(0.01, $baselineChange.changed_ratio * 3.0)
$visualChangeObserved = $drivenChange.changed_ratio -gt $minimumMotionRatio
$commandAccepted = $holdCandidates.Count -gt 0
$physicsMotionObserved = if ($DirectionKey -in @("Q", "E")) {
    $maxObservedAngularSpeed -gt 0.01
} else { $maxObservedLinearSpeed -gt 0.01 }
$motionObserved = if ($snapshotQualification.state -eq "passed") {
    $true
}
elseif ($snapshotQualification.state -eq "failed") {
    $false
}
else {
    $null
}

$originCorrelationPassed = $pressPassed -and $holdOriginPassed -and
    $releaseOriginMatches.Count -gt 0
$transportProbePassed = $inputReceived -and
    $processStillRunning -and
    $pressPassed -and
    $holdPassed -and
    $releasePassed -and
    $originCorrelationPassed
$passed = $transportProbePassed
if ($RequireLocomotion) {
    $passed = $passed -and
        $runIdentityQualification.state -eq "passed" -and
        $snapshotQualification.state -eq "passed" -and
        $releaseSnapshotQualification.state -eq "passed" -and
        $uprightQualification.state -eq "passed"
}

$result = [ordered]@{
    schema = "lingtu.sim.debug.drive-input-probe.v3"
    process_id = $ProcessId
    input_method = $InputMethod
    direction_key = $DirectionKey
    run_directory = $RunDirectory
    expected_run_id = if ($ExpectedRunId) { $ExpectedRunId } else { $null }
    expected_session_id = if ($ProductSessionId) {
        $ProductSessionId
    }
    else {
        $null
    }
    physical_foreground_input = $InputMethod -eq "ForegroundInput"
    session_state_before = $sessionStateBefore
    session_state_after = $sessionStateAfter
    window_handle = [long]$window
    window_title = $windowTitle
    foreground_before_input = $foregroundBeforeInput
    foreground_handle = Get-ProbeForegroundWindowHandle
    process_still_running = $processStillRunning
    new_received_intents = $newReceived.Count
    new_accepted_commands = $newAccepted.Count
    new_nonzero_accepted_commands = $nonzeroAccepted.Count
    unexpected_nonzero_commands = $unexpectedNonzeroAccepted.Count
    max_observed_linear_speed_mps = $maxObservedLinearSpeed
    max_observed_angular_speed_radps = $maxObservedAngularSpeed
    command_accepted = $commandAccepted
    physics_motion_observed = $physicsMotionObserved
    press = [ordered]@{
        passed = $pressPassed
        accepted_nonzero_commands = if ($null -ne $pressAnchor) { 1 } else { 0 }
        accepted_control_claims = $pressClaimMatches.Count
        origin_matches = $pressOriginMatches.Count
    }
    hold = [ordered]@{
        passed = $holdPassed
        samples = $holdCandidates.Count
        minimum_samples = $MinimumHoldSamples
        window_milliseconds = $holdWindowMilliseconds
        minimum_window_milliseconds = $MinimumHoldWindowMilliseconds
        origin_matches = $holdOriginMatches.Count
    }
    release = [ordered]@{
        passed = $releasePassed
        control_release_requests = $releaseRequests.Count
        released_authority_statuses = $releaseAuthority.Count
        zero_audit_records = $zeroAuditAfterHold.Count
        origin_matches = $releaseOriginMatches.Count
    }
    transport_probe = [ordered]@{
        passed = $transportProbePassed
        input_received = $inputReceived
        press = $pressPassed
        hold = $holdPassed
        release = $releasePassed
        origin_correlation = $originCorrelationPassed
    }
    transport_probe_passed = $transportProbePassed
    run_identity_qualification = $runIdentityQualification
    locomotion_qualification = $snapshotQualification
    release_stop_qualification = $releaseSnapshotQualification
    upright_qualification = $uprightQualification
    locomotion_qualification_required = [bool]$RequireLocomotion
    baseline = $baselineChange
    driven = $drivenChange
    minimum_motion_ratio = $minimumMotionRatio
    visual_change_observed = $visualChangeObserved
    visual_motion_observed = $visualChangeObserved
    motion_observed = $motionObserved
    captures = [ordered]@{
        baseline_a = $baselineA
        baseline_b = $baselineB
        driven = $driven
    }
    passed = $passed
}

$result | ConvertTo-Json -Depth 8
if (-not $passed) {
    exit 1
}
