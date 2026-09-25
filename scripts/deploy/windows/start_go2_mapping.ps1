param([string]$NxAddress = '192.168.123.18')

$ErrorActionPreference = 'Stop'
Write-Host 'Starting mapping on NX. Keep the robot stationary during startup.'
Write-Host 'Enter the NX login / sudo password if prompted.'

# Resolve and start through ProductControl on NX; no local service graph or tunnel.
$nxCommand = "sudo /bin/bash -lc 'set -a; . /opt/lingtu/config/go2-native.env; set +a; exec /bin/bash /opt/lingtu/current/scripts/lingtu --robot unitree/go2 --env real switch map --variant camera'"
& ssh -t -o ConnectTimeout=6 "unitree@$NxAddress" $nxCommand
if ($LASTEXITCODE -ne 0) {
    Write-Host 'Mapping did not start. Keep this output for diagnosis.'
    Read-Host 'Press Enter to close'
    exit 1
}

# The browser connects directly to NX over Ethernet, including when Wi-Fi is off.
Start-Process "http://${NxAddress}:5050/"
