Set-StrictMode -Version Latest

function ConvertFrom-LingTuVcRuntimeVersion {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Value
    )

    $text = ([string]$Value).Trim()
    if ($text.StartsWith("v", [StringComparison]::OrdinalIgnoreCase)) {
        $text = $text.Substring(1)
    }
    $version = $null
    if (-not [version]::TryParse($text, [ref]$version)) {
        throw "Microsoft Visual C++ Redistributable x64 registered an invalid version: '$Value'."
    }
    return $version
}

function Get-LingTuVcRedistRegistration {
    $baseKey = [Microsoft.Win32.RegistryKey]::OpenBaseKey(
        [Microsoft.Win32.RegistryHive]::LocalMachine,
        [Microsoft.Win32.RegistryView]::Registry64
    )
    try {
        $runtimeKey = $baseKey.OpenSubKey(
            "SOFTWARE\Microsoft\VisualStudio\14.0\VC\Runtimes\x64"
        )
        if ($null -eq $runtimeKey) {
            return [pscustomobject]@{ Installed = $false; Version = $null }
        }
        try {
            return [pscustomobject]@{
                Installed = ([int]$runtimeKey.GetValue("Installed", 0) -eq 1)
                Version = $runtimeKey.GetValue("Version", $null)
            }
        }
        finally {
            $runtimeKey.Dispose()
        }
    }
    finally {
        $baseKey.Dispose()
    }
}

function Assert-LingTuVcRedistInstaller {
    param(
        [Parameter(Mandatory = $true)]
        [IO.FileInfo]$InstallerPath
    )

    if (-not $InstallerPath.Exists) {
        throw "Microsoft Visual C++ Redistributable installer does not exist: $InstallerPath"
    }
    $signature = Get-AuthenticodeSignature -LiteralPath $InstallerPath.FullName
    if ($signature.Status -ne [System.Management.Automation.SignatureStatus]::Valid -or
        $null -eq $signature.SignerCertificate -or
        $signature.SignerCertificate.Subject -notmatch "(^|,\s*)O=Microsoft Corporation(,|$)") {
        throw "Microsoft Visual C++ Redistributable installer must have a valid Microsoft signature."
    }
    $versionInfo = $InstallerPath.VersionInfo
    if ($versionInfo.CompanyName -ne "Microsoft Corporation" -or
        $versionInfo.OriginalFilename -ine "VC_redist.x64.exe" -or
        $versionInfo.ProductName -notmatch
            "^Microsoft Visual C\+\+ .+ Redistributable \(x64\)(?: - .+)?$") {
        throw "'$($InstallerPath.FullName)' is not the Microsoft Visual C++ Redistributable x64 installer."
    }
}

function Invoke-LingTuVcRedistInstaller {
    param(
        [Parameter(Mandatory = $true)]
        [IO.FileInfo]$InstallerPath,

        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    Assert-LingTuVcRedistInstaller -InstallerPath $InstallerPath
    $startProcessParameters = @{
        FilePath = $InstallerPath.FullName
        ArgumentList = $Arguments
        Wait = $true
        PassThru = $true
    }
    $process = Start-Process @startProcessParameters
    return $process.ExitCode
}

function Invoke-LingTuVcRedistBootstrap {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [version]$MinimumVersion,

        [IO.FileInfo]$InstallerPath,

        [switch]$CheckOnly,

        [switch]$Quiet,

        [string]$LogPath,

        [scriptblock]$RegistrationReader = { Get-LingTuVcRedistRegistration },

        [scriptblock]$InstallerInvoker
    )

    $registration = & $RegistrationReader
    $installedVersion = $null
    if ($null -ne $registration -and [bool]$registration.Installed) {
        $installedVersion = ConvertFrom-LingTuVcRuntimeVersion $registration.Version
    }
    if ($null -ne $installedVersion -and $installedVersion -ge $MinimumVersion) {
        return [pscustomobject][ordered]@{
            Status = "Satisfied"
            InstalledVersion = $installedVersion.ToString()
            RequiredVersion = $MinimumVersion.ToString()
            InstallerExitCode = $null
        }
    }

    if ($CheckOnly) {
        $installedVersionText = if ($null -eq $installedVersion) {
            $null
        }
        else {
            $installedVersion.ToString()
        }
        return [pscustomobject][ordered]@{
            Status = "InstallRequired"
            InstalledVersion = $installedVersionText
            RequiredVersion = $MinimumVersion.ToString()
            InstallerExitCode = $null
        }
    }

    if ($null -eq $InstallerPath -or -not $InstallerPath.Exists) {
        throw "Microsoft Visual C++ Redistributable x64 installation requires an existing installer path."
    }
    $installerArguments = @("/install")
    if ($Quiet) {
        $installerArguments += "/quiet"
    }
    else {
        $installerArguments += "/passive"
    }
    $installerArguments += "/norestart"
    if ($LogPath) {
        $resolvedLogPath = [IO.Path]::GetFullPath($LogPath)
        if ($resolvedLogPath.IndexOfAny([char[]]@('"', "`r", "`n")) -ge 0) {
            throw "Microsoft Visual C++ Redistributable log path contains invalid characters."
        }
        $installerArguments += @("/log", ('"{0}"' -f $resolvedLogPath))
    }
    if ($null -eq $InstallerInvoker) {
        $InstallerInvoker = {
            param([IO.FileInfo]$Path, [string[]]$Arguments)
            Invoke-LingTuVcRedistInstaller -InstallerPath $Path -Arguments $Arguments
        }
    }
    $installerExitCode = [int](& $InstallerInvoker $InstallerPath $installerArguments)
    if ($installerExitCode -notin @(0, 3010)) {
        throw "Microsoft Visual C++ Redistributable x64 installer failed with exit code $installerExitCode."
    }

    $postInstallRegistration = & $RegistrationReader
    $postInstallVersion = $null
    if ($null -ne $postInstallRegistration -and [bool]$postInstallRegistration.Installed) {
        $postInstallVersion = ConvertFrom-LingTuVcRuntimeVersion $postInstallRegistration.Version
    }
    if ($null -eq $postInstallVersion -or $postInstallVersion -lt $MinimumVersion) {
        throw (
            "Microsoft Visual C++ Redistributable x64 remains below " +
            "$MinimumVersion after installer exit code $installerExitCode."
        )
    }

    return [pscustomobject][ordered]@{
        Status = if ($installerExitCode -eq 3010) { "RestartRequired" } else { "Installed" }
        InstalledVersion = $postInstallVersion.ToString()
        RequiredVersion = $MinimumVersion.ToString()
        InstallerExitCode = $installerExitCode
    }
}

Export-ModuleMember -Function Invoke-LingTuVcRedistBootstrap
