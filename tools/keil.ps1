param(
    [ValidateSet('Build', 'Rebuild', 'Flash')]
    [string]$Action = 'Build',
    [string]$Uv4Path = 'D:\STM32 Related\keil5\UV4\UV4.exe'
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding
$projectRoot = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $projectRoot 'MDK-ARM\standard_robot.uvprojx'
$outputPath = Join-Path $projectRoot 'MDK-ARM\standard_robot'
$logDirectory = Join-Path $projectRoot '.vscode'

function Invoke-Keil([string]$Operation, [string]$LogName) {
    $logPath = Join-Path $logDirectory $LogName
    # µVision 是 GUI 程序，必须等待进程退出，才能判断构建是否成功。
    if (Test-Path -LiteralPath $logPath) {
        Remove-Item -LiteralPath $logPath
    }
    $arguments = '{0} "{1}" -t "standard_robot" -j0 -sg -o "{2}"' -f $Operation, $projectPath, $logPath
    $process = Start-Process -FilePath $Uv4Path -ArgumentList $arguments `
        -WorkingDirectory (Split-Path -Parent $projectPath) `
        -WindowStyle Hidden -Wait -PassThru
    if (Test-Path -LiteralPath $logPath) {
        $logText = [System.IO.File]::ReadAllText($logPath, [System.Text.Encoding]::GetEncoding(936))
        Write-Host $logText
    } else {
        throw "Keil 未生成日志，请检查工程和工具路径：$logPath"
    }
    # Keil 返回 1 表示仅有警告；2 及以上才是失败。
    if ($process.ExitCode -notin @(0, 1)) {
        throw "Keil 执行失败，退出码 $($process.ExitCode)。日志：$logPath"
    }
}

try {
    if (-not (Test-Path -LiteralPath $Uv4Path -PathType Leaf)) {
        throw "未找到 Keil：$Uv4Path。可通过 -Uv4Path 指定 UV4.exe。"
    }
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    $buildOperation = if ($Action -eq 'Rebuild') { '-r' } else { '-b' }
    Write-Host "正在构建 standard_robot（$Action）..."
    Invoke-Keil $buildOperation 'keil-build.log'
    foreach ($artifact in @('standard_robot.axf', 'standard_robot.hex')) {
        if (-not (Test-Path -LiteralPath (Join-Path $outputPath $artifact) -PathType Leaf)) {
            throw "构建产物缺失：$artifact"
        }
    }
    Write-Host "构建完成：$(Join-Path $outputPath 'standard_robot.hex')"

    # 只有显式选择 Flash 且构建成功，才使用工程配置的下载器写入板子。
    if ($Action -eq 'Flash') {
        Write-Host '正在烧录，请保持板子供电和下载器连接...'
        Invoke-Keil '-f' 'keil-flash.log'
        Write-Host 'Keil 下载命令已完成，请检查上方校验结果。'
    }
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
