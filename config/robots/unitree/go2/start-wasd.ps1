<#
.SYNOPSIS
Open a dedicated Go2 WASD control window using the native operator-motion stream.
.DESCRIPTION
The robot must already have a ready Product that accepts operator motion.
teleop_avoid provides assisted avoidance; map and teleop use direct control.
This launcher does not start or switch Products. The keyboard client starts with
zero input and requires released keys before movement. Its exit/error output
stays visible until Enter.
.PARAMETER DomainId
Required DDS domain from the active RunPlan. Do not infer it from an old session.
.EXAMPLE
.\config\robots\unitree\go2\start-wasd.ps1 -DomainId 0
Use this example only after confirming that the active RunPlan uses domain 0.
#>
[CmdletBinding()]
param(
    [string]$SshConfig = (Join-Path $env:USERPROFILE '.ssh\lingtu-go2.conf'),
    [string]$SshTarget = 'lingtu-go2-nx',
    [Parameter(Mandatory = $true)]
    [ValidateRange(0, 232)]
    [int]$DomainId,
    [ValidateScript({ $_ -gt 0 -and -not [double]::IsInfinity($_) })]
    [double]$Speed = 0.2,
    [ValidateScript({ $_ -gt 0 -and -not [double]::IsInfinity($_) })]
    [double]$TurnRate = 0.35
)

$ErrorActionPreference = 'Stop'
if ($env:OS -ne 'Windows_NT') {
    throw 'The Go2 WASD launcher requires Windows and classic conhost.'
}
$repositoryPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..\..'))
$pythonPath = Join-Path $repositoryPath '.venv\Scripts\python.exe'
$sourcePath = Join-Path $repositoryPath 'src'
$sshConfigPath = [System.IO.Path]::GetFullPath($SshConfig)
foreach ($requiredPath in @($pythonPath, $sshConfigPath)) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "Required file is missing: $requiredPath"
    }
}

# Quote PowerShell string literals, then encode the child script so paths with
# spaces or apostrophes survive both Start-Process and conhost argument parsing.
$literalArguments = @($repositoryPath, $sourcePath, $pythonPath, $sshConfigPath, $SshTarget) |
    ForEach-Object { "'" + $_.Replace("'", "''") + "'" }
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$childScript = @'
$ErrorActionPreference = 'Stop'
$keyboardExitCode = 1
try {{
    Set-Location -LiteralPath {0}
    $env:PYTHONPATH = {1} + [System.IO.Path]::PathSeparator + $env:PYTHONPATH
    & {2} -m lingtu.operator_keyboard --ssh-config {3} --ssh-target {4} --domain-id {5} --speed {6} --turn-rate {7}
    $keyboardExitCode = $LASTEXITCODE
}} catch {{
    Write-Host ('Keyboard launcher failed: ' + $_.Exception.Message) -ForegroundColor Red
}} finally {{
    Write-Host ('Keyboard client exited with code ' + $keyboardExitCode)
    Read-Host 'Press Enter to close this window' | Out-Null
}}
exit $keyboardExitCode
'@ -f ($literalArguments + @(
    $DomainId.ToString($culture), $Speed.ToString($culture), $TurnRate.ToString($culture)
))
$encodedScript = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($childScript))
$consoleHostPath = Join-Path $env:SystemRoot 'System32\conhost.exe'
$shellPath = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
$consoleArguments = '"' + $shellPath + '" -NoLogo -NoProfile -ExecutionPolicy Bypass -EncodedCommand ' + $encodedScript
Start-Process -FilePath $consoleHostPath -ArgumentList $consoleArguments `
    -WorkingDirectory $repositoryPath -WindowStyle Normal -PassThru
