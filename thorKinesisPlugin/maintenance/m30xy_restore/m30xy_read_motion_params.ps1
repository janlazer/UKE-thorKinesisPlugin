<#
Read-only wrapper for m30xy_restore_motion_defaults.ps1.

This script only queries BDC/M30XY parameters. It does not write settings and
does not move, home, enable, or disable the stage.
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$Serial,
    [int[]]$Channels = @(1, 2),
    [string]$KinesisDir
)

$ErrorActionPreference = "Stop"

$restoreScript = Join-Path $PSScriptRoot "m30xy_restore_motion_defaults.ps1"
$params = @{
    Serial = $Serial
    Channels = $Channels
    Action = "Query"
}

if ($KinesisDir) {
    $params.KinesisDir = $KinesisDir
}

& $restoreScript @params
exit $LASTEXITCODE
