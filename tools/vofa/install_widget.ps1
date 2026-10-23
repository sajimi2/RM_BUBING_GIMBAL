param([string]$VofaDirectory)
$ErrorActionPreference = 'Stop'
if (-not $VofaDirectory) {
    $vofaExe = Get-Process -Name 'vofa+' -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Path
    if (-not $vofaExe) {
        throw 'Open VOFA first, or provide -VofaDirectory.'
    }
    $VofaDirectory = Split-Path -Parent $vofaExe
}
$widgetSource = Join-Path $PSScriptRoot 'yaw_sweep'
$pluginRoot = Join-Path $VofaDirectory 'plugins\widgets'
if (-not (Test-Path -LiteralPath $pluginRoot -PathType Container)) {
    throw 'plugins/widgets not found. Check -VofaDirectory.'
}
$widgetTarget = Join-Path $pluginRoot 'yaw_sweep'
New-Item -ItemType Directory -Path $widgetTarget -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $widgetSource 'yaw_sweep.qml') -Destination $widgetTarget
Copy-Item -LiteralPath (Join-Path $widgetSource 'motion.js') -Destination $widgetTarget
Write-Output "Installed: $widgetTarget"
Write-Output 'Exit auto aim, save data, disconnect serial, then restart VOFA manually to load the widget.'
