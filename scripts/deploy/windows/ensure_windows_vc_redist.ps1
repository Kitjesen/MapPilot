<#
.SYNOPSIS
Checks or installs the central Microsoft Visual C++ Redistributable x64 runtime.

.DESCRIPTION
This deployment entry point owns only the host prerequisite. CMake and vcpkg
remain responsible for application DLL staging, and RunPlan remains responsible
for authenticating the final application artifacts.

.PARAMETER MinimumVersion
The minimum four-part VC++ Redistributable version required by the build.

.PARAMETER InstallerPath
Path to the official Microsoft vc_redist.x64.exe. It is required only when the
runtime is missing or too old and CheckOnly is not set.

.PARAMETER CheckOnly
Reports whether installation is required without starting an installer. Exit
code 2 means installation is required; exit code 0 means the runtime is ready.

.PARAMETER Quiet
Uses the installer's /quiet mode. The default installation UI is /passive.

.PARAMETER LogPath
Optional path for the Microsoft installer log.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [version]$MinimumVersion,

    [IO.FileInfo]$InstallerPath,

    [switch]$CheckOnly,

    [switch]$Quiet,

    [string]$LogPath
)

$ErrorActionPreference = "Stop"
$modulePath = Join-Path $PSScriptRoot "windows_vc_redist.psm1"
Import-Module -Force $modulePath

try {
    $parameters = @{
        MinimumVersion = $MinimumVersion
        CheckOnly = $CheckOnly
        Quiet = $Quiet
    }
    if ($null -ne $InstallerPath) {
        $parameters.InstallerPath = $InstallerPath
    }
    if ($LogPath) {
        $parameters.LogPath = $LogPath
    }
    $result = Invoke-LingTuVcRedistBootstrap @parameters
    $result | ConvertTo-Json -Compress
    if ($CheckOnly -and $result.Status -eq "InstallRequired") {
        exit 2
    }
    exit 0
}
catch {
    Write-Error -Message $_.Exception.Message
    exit 1
}
