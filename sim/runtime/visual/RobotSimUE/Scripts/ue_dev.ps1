[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet("status", "build", "editor", "mcp", "terminal")]
    [string] $Action = "status",

    [ValidateRange(1, 65535)]
    [int] $McpPort = 8000,

    [switch] $Wait
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ProjectFile = Join-Path $ProjectRoot "RobotSimUE.uproject"
$WorkspaceRoot = (Resolve-Path (Join-Path $ProjectRoot "..\..\..\..")).Path
$CodexConfig = Join-Path $WorkspaceRoot ".codex\config.toml"
$DevelopmentPlugins = @("ModelContextProtocol", "AllToolsets", "Terminal")

function Resolve-UnrealEngineRoot {
    if ($env:UE_5_8_ROOT) {
        $candidate = [System.IO.Path]::GetFullPath($env:UE_5_8_ROOT)
        if (Test-Path (Join-Path $candidate "Engine\Binaries\Win64\UnrealEditor.exe")) {
            return $candidate
        }
        throw "UE_5_8_ROOT does not contain UnrealEditor.exe: $candidate"
    }

    $manifestRoot = "C:\ProgramData\Epic\EpicGamesLauncher\Data\Manifests"
    if (Test-Path $manifestRoot) {
        foreach ($manifest in Get-ChildItem $manifestRoot -Filter "*.item") {
            try {
                $metadata = Get-Content -Raw $manifest.FullName | ConvertFrom-Json
                if ($metadata.AppName -eq "UE_5.8" -and $metadata.InstallLocation) {
                    $candidate = [System.IO.Path]::GetFullPath([string] $metadata.InstallLocation)
                    if (Test-Path (Join-Path $candidate "Engine\Binaries\Win64\UnrealEditor.exe")) {
                        return $candidate
                    }
                }
            }
            catch {
                continue
            }
        }
    }

    $fallback = "D:\Program Files\Epic Games\UE_5.8"
    if (Test-Path (Join-Path $fallback "Engine\Binaries\Win64\UnrealEditor.exe")) {
        return $fallback
    }
    throw "Unreal Engine 5.8 was not found. Set UE_5_8_ROOT to the engine directory."
}

function Resolve-PowerShellExecutable {
    $command = Get-Command "pwsh.exe" -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($command) {
        return $command.Source
    }

    $portable = Join-Path $env:LOCALAPPDATA "Programs\PowerShell\7.6.3-portable\pwsh.exe"
    if (Test-Path $portable) {
        return $portable
    }
    throw "PowerShell 7 was not found. Install pwsh or add it to PATH."
}

function Test-TcpPort {
    param([int] $Port)

    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $task = $client.ConnectAsync("127.0.0.1", $Port)
        return $task.Wait([TimeSpan]::FromMilliseconds(500)) -and $client.Connected
    }
    catch {
        return $false
    }
    finally {
        $client.Dispose()
    }
}

function ConvertFrom-McpBody {
    param([Parameter(Mandatory = $true)][string] $Body)

    $trimmed = $Body.Trim()
    if ($trimmed.StartsWith("{")) {
        return $trimmed | ConvertFrom-Json
    }

    foreach ($line in ($trimmed -split "`r?`n")) {
        if ($line.StartsWith("data:")) {
            $payload = $line.Substring(5).Trim()
            if ($payload -and $payload -ne "[DONE]") {
                return $payload | ConvertFrom-Json
            }
        }
    }
    throw "Unreal MCP returned an unsupported response body."
}

function Invoke-McpJsonRpc {
    param(
        [Parameter(Mandatory = $true)][string] $Url,
        [Parameter(Mandatory = $true)][hashtable] $Payload,
        [string] $SessionId = ""
    )

    $headers = @{ Accept = "application/json, text/event-stream" }
    if ($SessionId) {
        $headers["Mcp-Session-Id"] = $SessionId
    }
    $response = Invoke-WebRequest `
        -Uri $Url `
        -Method Post `
        -Headers $headers `
        -ContentType "application/json" `
        -Body ($Payload | ConvertTo-Json -Depth 16 -Compress)

    $result = $null
    if ($response.Content) {
        $result = ConvertFrom-McpBody -Body $response.Content
    }
    $responseSession = [string] $response.Headers["Mcp-Session-Id"]
    return [PSCustomObject]@{
        Payload = $result
        SessionId = if ($responseSession) { $responseSession } else { $SessionId }
        StatusCode = [int] $response.StatusCode
    }
}

function Test-UnrealMcp {
    param([int] $Port)

    $url = "http://127.0.0.1:$Port/mcp"
    $initialize = Invoke-McpJsonRpc -Url $url -Payload @{
        jsonrpc = "2.0"
        id = 1
        method = "initialize"
        params = @{
            protocolVersion = "2025-11-25"
            capabilities = @{}
            clientInfo = @{ name = "lingtu-ue-check"; version = "1.0" }
        }
    }
    if (-not $initialize.Payload.result -or $initialize.Payload.result.protocolVersion -ne "2025-11-25") {
        throw "The endpoint did not negotiate the Unreal MCP protocol version."
    }

    [void] (Invoke-McpJsonRpc -Url $url -SessionId $initialize.SessionId -Payload @{
        jsonrpc = "2.0"
        method = "notifications/initialized"
    })
    $tools = Invoke-McpJsonRpc -Url $url -SessionId $initialize.SessionId -Payload @{
        jsonrpc = "2.0"
        id = 2
        method = "tools/list"
        params = @{}
    }
    if (-not $tools.Payload.result -or -not $tools.Payload.result.tools) {
        throw "Unreal MCP initialized but advertised no tools. Check that AllToolsets is enabled."
    }
    $toolNames = @($tools.Payload.result.tools | ForEach-Object { [string] $_.name })
    foreach ($requiredTool in @("list_toolsets", "describe_toolset", "call_tool")) {
        if ($requiredTool -notin $toolNames) {
            throw "Unreal MCP is missing required discovery tool: $requiredTool"
        }
    }

    $toolsets = Invoke-McpJsonRpc -Url $url -SessionId $initialize.SessionId -Payload @{
        jsonrpc = "2.0"
        id = 3
        method = "tools/call"
        params = @{ name = "list_toolsets"; arguments = @{} }
    }
    $toolsetText = [string] $toolsets.Payload.result.content[0].text
    $toolsetCount = @(($toolsetText -split "`r?`n") | Where-Object { $_.StartsWith("- ") }).Count
    if ($toolsetCount -lt 1) {
        throw "Unreal MCP discovery works, but AllToolsets exposed no toolsets."
    }

    return [PSCustomObject]@{
        url = $url
        server_info_name = [string] $initialize.Payload.result.serverInfo.name
        server_info_version = [string] $initialize.Payload.result.serverInfo.version
        protocol_version = [string] $initialize.Payload.result.protocolVersion
        session_id = $initialize.SessionId
        discovery_tools = $toolNames
        toolset_count = $toolsetCount
    }
}

$EngineRoot = Resolve-UnrealEngineRoot
$EditorExe = Join-Path $EngineRoot "Engine\Binaries\Win64\UnrealEditor.exe"
$BuildBat = Join-Path $EngineRoot "Engine\Build\BatchFiles\Build.bat"
$BuildVersion = Join-Path $EngineRoot "Engine\Build\Build.version"
$PowerShellExe = Resolve-PowerShellExecutable
$process = $null

switch ($Action) {
    "status" {
        $version = if (Test-Path $BuildVersion) { Get-Content -Raw $BuildVersion | ConvertFrom-Json } else { $null }
        $plugins = (Get-Content -Raw $ProjectFile | ConvertFrom-Json).Plugins
        $enabledPluginNames = @($plugins | Where-Object Enabled | ForEach-Object { [string] $_.Name })
        [PSCustomObject]@{
            engine_root = $EngineRoot
            engine_version = if ($version) { "$($version.MajorVersion).$($version.MinorVersion).$($version.PatchVersion)" } else { "unknown" }
            project = $ProjectFile
            editor_running = [bool] (Get-Process UnrealEditor -ErrorAction SilentlyContinue)
            mcp_listening = Test-TcpPort -Port $McpPort
            mcp_url = "http://127.0.0.1:$McpPort/mcp"
            codex_config = Test-Path $CodexConfig
            terminal_shell = $PowerShellExe
            development_plugins = $DevelopmentPlugins
            development_plugins_enabled_by_default = @(
                $DevelopmentPlugins | Where-Object { $_ -notin $enabledPluginNames }
            ).Count -eq 0
        } | ConvertTo-Json -Depth 4
    }
    "build" {
        & $BuildBat "RobotSimUEEditor" "Win64" "Development" $ProjectFile "-WaitMutex" "-NoHotReloadFromIDE"
        if ($LASTEXITCODE -ne 0) {
            throw "RobotSimUEEditor build failed with exit code $LASTEXITCODE."
        }
    }
    "editor" {
        $existing = @()
        try {
            $existing = @(Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" -ErrorAction Stop |
                Where-Object { $_.CommandLine -and $_.CommandLine.Contains($ProjectFile, [System.StringComparison]::OrdinalIgnoreCase) })
        }
        catch {
            Write-Warning "Unable to inspect UnrealEditor command lines; MCP state will determine whether this project is already running."
            if (Test-TcpPort -Port $McpPort) {
                $existing = @(Get-Process UnrealEditor -ErrorAction SilentlyContinue)
            }
        }
        if ($existing) {
            Write-Host "RobotSimUE is already running (PID $($existing[0].ProcessId))."
        }
        else {
            $env:LINGTU_WORKSPACE_ROOT = $WorkspaceRoot
            $env:LINGTU_UE_PROJECT = $ProjectFile
            $editorUserDir = Join-Path $ProjectRoot "Saved\DeveloperUser"
            $derivedDataDir = Join-Path $ProjectRoot "Saved\DerivedDataCache"
            [void] (New-Item -ItemType Directory -Force -Path $editorUserDir)
            [void] (New-Item -ItemType Directory -Force -Path $derivedDataDir)
            $arguments = @(
                ('"{0}"' -f $ProjectFile),
                "-EnablePlugins=$($DevelopmentPlugins -join ',')",
                "-UserDir=$editorUserDir",
                "-ddc=NoZenLocalFallback",
                "-LocalDataCachePath=$derivedDataDir",
                "-ModelContextProtocolStartServer",
                "-ModelContextProtocolPort=$McpPort",
                "-LogCmds=`"LogModelContextProtocol Verbose`""
            )
            $process = Start-Process -FilePath $EditorExe -ArgumentList $arguments -PassThru
            Write-Host "Started RobotSimUE (PID $($process.Id))."
        }

        $deadline = [DateTime]::UtcNow.AddMinutes(3)
        while ([DateTime]::UtcNow -lt $deadline -and -not (Test-TcpPort -Port $McpPort)) {
            Start-Sleep -Milliseconds 500
        }
        if (-not (Test-TcpPort -Port $McpPort)) {
            throw "Unreal Editor started, but MCP did not listen on port $McpPort within 3 minutes."
        }
        Test-UnrealMcp -Port $McpPort | ConvertTo-Json -Depth 5

        if ($Wait -and $process) {
            $process.WaitForExit()
        }
    }
    "mcp" {
        if (-not (Test-TcpPort -Port $McpPort)) {
            throw "Unreal MCP is not listening on port $McpPort. Run: .\ue_dev.ps1 editor"
        }
        Test-UnrealMcp -Port $McpPort | ConvertTo-Json -Depth 5
    }
    "terminal" {
        $env:LINGTU_WORKSPACE_ROOT = $WorkspaceRoot
        $env:LINGTU_UE_PROJECT = $ProjectFile
        $command = "`$env:TERM='xterm-256color'; Set-Location -LiteralPath '$WorkspaceRoot'; codex"
        Start-Process -FilePath $PowerShellExe -WorkingDirectory $WorkspaceRoot -ArgumentList @("-NoExit", "-Command", $command)
    }
}
