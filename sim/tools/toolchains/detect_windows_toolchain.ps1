[CmdletBinding()]
param(
    [string]$RepoRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)),
    [string]$LockFile,
    [string]$UnrealRoot,
    [string]$WindowsSdkRoot,
    [string]$MuJoCoRoot,
    [string]$BlenderExecutable
)
$ErrorActionPreference = "SilentlyContinue"
$NetFxSdkProbe = Join-Path $PSScriptRoot "windows_netfxsdk.ps1"

function New-Component([string]$Id) {
    [ordered]@{ id = $Id; status = [ordered]@{ state = "not_found"; reasons = @("No installation detected.") }; version = [ordered]@{ exact = $null; source = $null }; source = [ordered]@{ type = "unknown"; location = $null; evidence = @() }; architecture = "unknown"; sha256 = [ordered]@{ value = $null; path = $null; algorithm = "sha256"; evidence = @() }; installation = [ordered]@{ root_path = $null; executable_path = $null; evidence = @() }; validation = [ordered]@{ checks = @(); validated_at_utc = $null }; lock = $null }
}
function Add-Check($C, [string]$Id, [bool]$Passed, [string]$Evidence) { $C.validation.checks += [ordered]@{ id = $Id; passed = $Passed; evidence = $Evidence } }
function Set-Component($C, [string]$Version, [string]$VersionSource, [string]$SourceType, [string]$Location, [string]$Architecture, [string]$Root, [string]$Executable, [string]$Evidence) {
    $C.version.exact = $Version; $C.version.source = $VersionSource; $C.source.type = $SourceType; $C.source.location = $Location; $C.source.evidence = @($Evidence); $C.architecture = $Architecture; $C.installation.root_path = $Root; $C.installation.executable_path = $Executable; $C.installation.evidence = @($Evidence); $C.status.state = "detected"; $C.status.reasons = @("A candidate installation was detected.")
}
function Add-Hash($C, [string]$Path) {
    if ($Path -and (Test-Path -LiteralPath $Path -PathType Leaf)) { $H = Get-FileHash -LiteralPath $Path -Algorithm SHA256; $C.sha256.value = $H.Hash.ToLowerInvariant(); $C.sha256.path = $Path; $C.sha256.evidence = @("Get-FileHash SHA256: $Path") }
}
function Validate-Component($C, [string]$VersionEvidence, [bool]$HasExactVersion) {
    $hasPath = [bool]($C.installation.executable_path -and (Test-Path -LiteralPath $C.installation.executable_path -PathType Leaf)); $hasHash = [bool]($C.sha256.value -and $C.sha256.value -match "^[A-Fa-f0-9]{64}$")
    Add-Check $C "exact_version" $HasExactVersion $VersionEvidence; Add-Check $C "installation_path" $hasPath $C.installation.executable_path; Add-Check $C "sha256" $hasHash $C.sha256.path
    if ($HasExactVersion -and $hasPath -and $hasHash) { $C.status.state = "validated"; $C.status.reasons = @("Exact version, executable and SHA-256 evidence are present."); $C.validation.validated_at_utc = [DateTime]::UtcNow.ToString("o") } else { $C.status.state = "detected"; $C.status.reasons = @("Candidate detected, but exact version or validation evidence is incomplete.") }
}
function Get-UeRoots {
    $Roots = @()
    if ($UnrealRoot) { $Roots += $UnrealRoot }
    foreach ($Name in @("UE_ROOT", "UNREAL_ENGINE_ROOT")) {
        $Value = [Environment]::GetEnvironmentVariable($Name)
        if ($Value) { $Roots += $Value }
    }
    $Launcher = Join-Path $env:ProgramData "Epic\UnrealEngineLauncher\LauncherInstalled.dat"
    if (Test-Path -LiteralPath $Launcher -PathType Leaf) { try { $J = Get-Content -LiteralPath $Launcher -Raw | ConvertFrom-Json; foreach ($E in @($J.InstallationList)) { if ($E.InstallLocation) { $Roots += [string]$E.InstallLocation } } } catch {} }
    $ProgramRoots = @($env:ProgramFiles, ${env:ProgramFiles(x86)})
    foreach ($Drive in @(Get-PSDrive -PSProvider FileSystem)) {
        $ProgramRoots += (Join-Path $Drive.Root "Program Files")
        $ProgramRoots += (Join-Path $Drive.Root "Program Files (x86)")
    }
    foreach ($Base in @($ProgramRoots | Where-Object { $_ } | Sort-Object -Unique)) { if ($Base) { $Epic = Join-Path $Base "Epic Games"; if (Test-Path -LiteralPath $Epic -PathType Container) { $Roots += @(Get-ChildItem -LiteralPath $Epic -Directory -Filter "UE_*" | ForEach-Object FullName) } } }
    $Roots | Where-Object { $_ } | Sort-Object -Unique
}
function Detect-Unreal {
    $C = New-Component "unreal_engine"
    foreach ($Root in Get-UeRoots) { $Build = Join-Path $Root "Engine\Build\Build.version"; $Editor = Join-Path $Root "Engine\Binaries\Win64\UnrealEditor.exe"; if (-not (Test-Path -LiteralPath $Build -PathType Leaf)) { continue }; try { $V = Get-Content -LiteralPath $Build -Raw | ConvertFrom-Json; $Exact = "$($V.MajorVersion).$($V.MinorVersion).$($V.PatchVersion)"; Set-Component $C $Exact "Engine/Build/Build.version" "installation" $Root "x64" $Root $Editor $Build; Add-Hash $C $Editor; Validate-Component $C "Build.version: $Build" ([bool]($V.MajorVersion -ne $null -and $V.MinorVersion -ne $null -and $V.PatchVersion -ne $null)); break } catch {} }
    $C
}
function Detect-VisualStudio {
    $C = New-Component "visual_studio_msvc"; $Candidates = @((Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"), (Join-Path $env:ProgramFiles "Microsoft Visual Studio\Installer\vswhere.exe")); $VsWhere = $Candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
    if ($VsWhere) { try { $Instances = (& $VsWhere -products * -format json) | ConvertFrom-Json; $I = @($Instances) | Where-Object { $_.isComplete -ne $false } | Select-Object -First 1; if ($I) { $Root = [string]$I.installationPath; $Cl = Get-ChildItem -LiteralPath (Join-Path $Root "VC\Tools\MSVC") -Filter cl.exe -File -Recurse | Where-Object { $_.FullName -match "Hostx64\\x64\\cl\.exe$" } | Sort-Object FullName -Descending | Select-Object -First 1; $Exact = [string]$I.installationVersion; $Exe = if ($Cl) { $Cl.FullName } else { $null }; Set-Component $C $Exact "vswhere installationVersion" "installation" $Root "x64" $Root $Exe $VsWhere; if ($Cl) { Add-Hash $C $Cl.FullName }; Validate-Component $C "vswhere: $VsWhere" ([bool]($Exact -and $Exact -match "^\d+\.\d+")); $NetFxSdk = if (Test-Path -LiteralPath $NetFxSdkProbe -PathType Leaf) { & $NetFxSdkProbe -AllowMissing }; $HasNetFxSdk = [bool]$NetFxSdk; Add-Check $C "netfx_sdk" $HasNetFxSdk $(if ($NetFxSdk) { "NETFXSDK: $NetFxSdk" } else { "No .NET Framework SDK 4.6+ registration with Include/um/mscoree.h was found." }); if (-not $HasNetFxSdk) { $C.status.state = "detected"; $C.status.reasons = @("MSVC is present, but Unreal Editor builds require the .NET Framework SDK 4.6 or newer.") } } } catch {} }
    $C
}
function Detect-WindowsSdk {
    $C = New-Component "windows_sdk"
    $KitCandidates = @()
    if ($WindowsSdkRoot) { $KitCandidates += $WindowsSdkRoot }
    if ($env:WindowsSdkDir) { $KitCandidates += $env:WindowsSdkDir }
    foreach ($RegistryPath in @("HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots", "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots")) {
        try { $RegisteredRoot = (Get-ItemProperty -LiteralPath $RegistryPath -Name KitsRoot10).KitsRoot10; if ($RegisteredRoot) { $KitCandidates += $RegisteredRoot } } catch {}
    }
    foreach ($Drive in @(Get-PSDrive -PSProvider FileSystem)) {
        $KitCandidates += (Join-Path $Drive.Root "Program Files (x86)\Windows Kits\10")
        $KitCandidates += (Join-Path $Drive.Root "Program Files\Windows Kits\10")
    }
    $Kits = $KitCandidates | Where-Object { $_ -and (Test-Path -LiteralPath $_ -PathType Container) } | Sort-Object -Unique | Select-Object -First 1
    if ($Kits) { $Sdk = Get-ChildItem -LiteralPath (Join-Path $Kits "Lib") -Directory | Where-Object Name -match "^10\.0\.\d+\.0$" | Sort-Object Name -Descending | Select-Object -First 1; if ($Sdk) { $Rc = Get-ChildItem -LiteralPath (Join-Path $Kits "bin") -Filter rc.exe -File -Recurse | Where-Object FullName -match "\\x64\\rc\.exe$" | Select-Object -First 1; $Exact = $Sdk.Name; $Exe = if ($Rc) { $Rc.FullName } else { $null }; Set-Component $C $Exact "Windows Kits/Lib directory name" "installation" $Kits "x64" $Kits $Exe $Sdk.FullName; if ($Rc) { Add-Hash $C $Rc.FullName }; Validate-Component $C "Windows SDK Lib directory: $($Sdk.FullName)" ([bool]($Exact -match "^10\.0\.\d+\.0$")) } }
    $C
}
function Find-MuJoCoRoot {
    $Candidates = @()
    if ($MuJoCoRoot) { $Candidates += $MuJoCoRoot }
    foreach ($Name in @("MUJOCO_ROOT", "MUJOCO_HOME", "MUJOCO_DIR")) { $Value = [Environment]::GetEnvironmentVariable($Name); if ($Value) { $Candidates += $Value } }
    $Candidates += @((Join-Path $RepoRoot "sim\third_party\mujoco"), (Join-Path $RepoRoot "third_party\mujoco"), (Join-Path $RepoRoot "vendor\mujoco"))
    foreach ($Drive in @(Get-PSDrive -PSProvider FileSystem)) {
        foreach ($Base in @((Join-Path $Drive.Root "Development\MuJoCo"), (Join-Path $Drive.Root "SDKs\MuJoCo"))) {
            if (Test-Path -LiteralPath $Base -PathType Container) { $Candidates += @(Get-ChildItem -LiteralPath $Base -Directory | Sort-Object Name -Descending | ForEach-Object FullName) }
        }
    }
    $Candidates | Where-Object { $_ -and (Test-Path -LiteralPath (Join-Path $_ "include\mujoco\mujoco.h") -PathType Leaf) } | Sort-Object -Unique | Select-Object -First 1
}
function Detect-MuJoCo {
    $C = New-Component "mujoco_sdk"; $Root = Find-MuJoCoRoot
    if ($Root) { $Headers = @((Join-Path $Root "include\mujoco\mjversion.h"), (Join-Path $Root "include\mujoco\mjmodel.h"), (Join-Path $Root "include\mujoco\mujoco.h")) | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf }; $Library = Get-ChildItem -LiteralPath $Root -Filter "mujoco.dll" -File -Recurse | Select-Object -First 1; $Version = $null; foreach ($Header in $Headers) { $Text = Get-Content -LiteralPath $Header -Raw; $M = [regex]::Match($Text, '(?m)^\s*#define\s+(?:MJ_VERSION|MUJOCO_VERSION)\s+"([0-9]+(?:\.[0-9]+)+)"'); if ($M.Success) { $Version = $M.Groups[1].Value; break }; $N = [regex]::Match($Text, '(?m)^\s*#define\s+mjVERSION_HEADER\s+(\d+)'); if ($N.Success) { $Encoded = [int64]$N.Groups[1].Value; if ($Encoded -ge 1000000) { $Major = [math]::Floor($Encoded / 1000000); $Minor = [math]::Floor(($Encoded % 1000000) / 1000); $Patch = $Encoded % 1000; $Version = "$Major.$Minor.$Patch" } elseif ($N.Groups[1].Value.Length -eq 3) { $Digits = $N.Groups[1].Value; $Version = "$($Digits.Substring(0,1)).$($Digits.Substring(1,1)).$($Digits.Substring(2,1))" }; if ($Version) { break } } }; $Exe = if ($Library) { $Library.FullName } else { $null }; $MuJoCoSource = if (@("MUJOCO_ROOT", "MUJOCO_HOME", "MUJOCO_DIR") | ForEach-Object { [Environment]::GetEnvironmentVariable($_) } | Where-Object { $_ -and ((Resolve-Path -LiteralPath $_).Path -eq (Resolve-Path -LiteralPath $Root).Path) }) { "environment" } else { "installation" }; Set-Component $C $Version "MuJoCo header version macro" $MuJoCoSource $Root "x64" $Root $Exe ("candidate root: $Root"); if ($Library) { Add-Hash $C $Library.FullName }; Validate-Component $C "MuJoCo header exact version" ([bool]($Version -and $Version -match "^\d+\.\d+\.\d+$")) }
    $C
}
function Detect-Blender {
    $C = New-Component "blender"; $Candidates = @()
    if ($BlenderExecutable) { $Candidates += $BlenderExecutable }
    if ($env:BLENDER_EXE) { $Candidates += $env:BLENDER_EXE }
    $SearchRoots = @($env:ProgramFiles, ${env:ProgramFiles(x86)}, (Join-Path $env:LOCALAPPDATA "Programs"))
    foreach ($Drive in @(Get-PSDrive -PSProvider FileSystem)) {
        $SearchRoots += (Join-Path $Drive.Root "Program Files")
        $SearchRoots += (Join-Path $Drive.Root "Program Files (x86)")
        $DevelopmentRoot = Join-Path $Drive.Root "Development\Blender"
        if (Test-Path -LiteralPath $DevelopmentRoot -PathType Container) { $Candidates += @(Get-ChildItem -LiteralPath $DevelopmentRoot -Filter blender.exe -File -Recurse | ForEach-Object FullName) }
    }
    foreach ($Base in @($SearchRoots | Where-Object { $_ } | Sort-Object -Unique)) { $Foundation = Join-Path $Base "Blender Foundation"; if (Test-Path -LiteralPath $Foundation -PathType Container) { $Candidates += @(Get-ChildItem -LiteralPath $Foundation -Filter blender.exe -File -Recurse | ForEach-Object FullName) } }; $Command = Get-Command blender.exe -ErrorAction SilentlyContinue; if ($Command) { $Candidates += $Command.Source }; $Exe = $Candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
    if ($Exe) { $FileVersion = [Diagnostics.FileVersionInfo]::GetVersionInfo($Exe).FileVersion; try { $VersionOutput = & $Exe --version 2>$null | Select-Object -First 1; $VersionMatch = [regex]::Match([string]$VersionOutput, '^Blender\s+(\d+\.\d+\.\d+)'); if ($VersionMatch.Success) { $FileVersion = $VersionMatch.Groups[1].Value } } catch {}; $Root = Split-Path -Parent $Exe; Set-Component $C $FileVersion "blender --version" "installation" $Root "x64" $Root $Exe $Exe; Add-Hash $C $Exe; Validate-Component $C "blender --version: $Exe" ([bool]($FileVersion -and $FileVersion -match "^\d+\.\d+\.\d+")) }
    $C
}
function Apply-LockEvidence($Components, [string]$Path) {
    if (-not $Path -or -not (Test-Path -LiteralPath $Path -PathType Leaf)) { return }; try { $Locked = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json } catch { return }
    foreach ($Name in @("unreal_engine", "visual_studio_msvc", "windows_sdk", "mujoco_sdk", "blender")) { $Candidate = $Components[$Name]; $Approved = $Locked.components.$Name; if ($Approved -and $Approved.status.state -eq "locked" -and $Candidate.status.state -eq "validated" -and $Approved.version.exact -eq $Candidate.version.exact -and $Approved.sha256.value -eq $Candidate.sha256.value -and $Approved.architecture -eq $Candidate.architecture -and $Approved.source.location -eq $Candidate.source.location) { $Candidate.status.state = "locked"; $Candidate.status.reasons = @("Detected evidence matches the supplied lock file."); $Candidate.lock = $Approved.lock } }
}
$Components = [ordered]@{ unreal_engine = Detect-Unreal; visual_studio_msvc = Detect-VisualStudio; windows_sdk = Detect-WindowsSdk; mujoco_sdk = Detect-MuJoCo; blender = Detect-Blender }; Apply-LockEvidence $Components $LockFile; $LockedCount = @($Components.Values | Where-Object { $_.status.state -eq "locked" }).Count; $SummaryState = if ($LockedCount -eq 5) { "locked" } elseif ($LockedCount -gt 0) { "partially_locked" } else { "unlocked" }
[ordered]@{ schema = "lingtu.sim.windows-toolchain-lock"; schema_version = 1; generated_at_utc = [DateTime]::UtcNow.ToString("o"); host = [ordered]@{ computer_name = $env:COMPUTERNAME; os_build = [Environment]::OSVersion.Version.ToString(); architecture = if ([Environment]::Is64BitOperatingSystem) { "x64" } else { "x86" } }; policy = [ordered]@{ lock_requires_exact_version = $true; lock_requires_source = $true; lock_requires_sha256 = $true; lock_requires_installation_evidence = $true; notes = "Detection is read-only; locking requires reviewed evidence and -LockFile matching." }; summary = [ordered]@{ state = $SummaryState; locked_count = $LockedCount; component_count = 5 }; components = $Components } | ConvertTo-Json -Depth 16
