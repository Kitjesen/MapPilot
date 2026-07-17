param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$Root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$Source = Join-Path $Root "src\kernels\gateway\pointcloud_codec"
$Build = Join-Path $Source "build"

cmake -S $Source -B $Build
cmake --build $Build --config $Configuration -j

Write-Output "Built pointcloud codec in $Build"
