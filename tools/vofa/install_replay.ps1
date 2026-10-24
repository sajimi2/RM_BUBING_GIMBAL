param([string]$VofaDirectory)
$ErrorActionPreference = 'Stop'
if (-not $VofaDirectory) {
    $vofaExe = Get-Process -Name 'vofa+' -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Path
    if (-not $vofaExe) {
        throw 'Open VOFA first, or provide -VofaDirectory.'
    }
    $VofaDirectory = Split-Path -Parent $vofaExe
}
$pluginRoot = Join-Path $VofaDirectory 'plugins\widgets'
if (-not (Test-Path -LiteralPath $pluginRoot -PathType Container)) {
    throw 'plugins/widgets not found. Check -VofaDirectory.'
}
$widgetSource = Join-Path $PSScriptRoot 'vision_replay'
$widgetTarget = Join-Path $pluginRoot 'vision_replay'
$names = @('vision_replay.qml', 'replay.js', 'recording.js')
foreach ($name in $names) {
    if (-not (Test-Path -LiteralPath (Join-Path $widgetSource $name) -PathType Leaf)) {
        throw "Source file missing: $name"
    }
}
# 更新已有重放控件时，先留一份恢复副本；不触碰原正弦控件。
if (Test-Path -LiteralPath $widgetTarget -PathType Container) {
    $backupRoot = Join-Path $VofaDirectory 'widget-backups'
    $backup = Join-Path $backupRoot ('vision_replay-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $backup -Force | Out-Null
    foreach ($name in $names) {
        $old = Join-Path $widgetTarget $name
        if (Test-Path -LiteralPath $old -PathType Leaf) {
            Copy-Item -LiteralPath $old -Destination $backup
        }
    }
    Write-Output "Backup: $backup"
}
New-Item -ItemType Directory -Path $widgetTarget -Force | Out-Null
foreach ($name in $names) {
    $sourceFile = Join-Path $widgetSource $name
    $targetFile = Join-Path $widgetTarget $name
    Copy-Item -LiteralPath $sourceFile -Destination $targetFile
    if ((Get-FileHash -LiteralPath $sourceFile).Hash -ne (Get-FileHash -LiteralPath $targetFile).Hash) {
        throw "Installed file differs: $name"
    }
}
Write-Output "Installed and verified: $widgetTarget"
Write-Output 'Save CSV, exit auto aim, disconnect serial, then restart VOFA manually. This script does not send commands or restart the app.'
