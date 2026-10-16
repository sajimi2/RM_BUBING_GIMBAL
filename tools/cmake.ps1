param(
    [ValidateSet('Configure', 'Build', 'Rebuild', 'Flash', 'CheckFlash')]
    [string]$Action = 'Build',
    [ValidateSet('Debug', 'Release')][string]$Preset = 'Debug'
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding
$projectRoot = Split-Path -Parent $PSScriptRoot
try {
    Push-Location -LiteralPath $projectRoot
    & cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { throw 'CMake 配置失败。' }
    if ($Action -eq 'Configure') { exit 0 }
    $arguments = @('--build', '--preset', $Preset, '--parallel')
    if ($Action -eq 'Rebuild') { $arguments += '--clean-first' }
    # Flash 目标依赖固件，编译失败时 CMake 不会执行下载命令。
    if ($Action -eq 'Flash') { $arguments += @('--target', 'flash') }
    & cmake @arguments
    if ($LASTEXITCODE -ne 0) { throw '构建或烧录失败，请查看上方输出。' }
    if ($Action -eq 'CheckFlash') {
        & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'flash.ps1') `
            -Firmware (Join-Path $projectRoot "build\$Preset\standard_robot.elf") -DryRun
        if ($LASTEXITCODE -ne 0) { throw '烧录配置检查失败。' }
    }
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
} finally {
    Pop-Location
}
