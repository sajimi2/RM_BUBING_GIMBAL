param(
    [Parameter(Mandatory = $true)][string]$Firmware,
    [string]$ProgrammerPath = $env:STM32_PROGRAMMER_CLI,
    [string]$Serial = $env:STM32_STLINK_SERIAL,
    [ValidateSet('NORMAL', 'UR', 'HOTPLUG')][string]$ConnectMode = 'NORMAL',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding
try {
    $firmwarePath = (Resolve-Path -LiteralPath $Firmware).Path
    if ([IO.Path]::GetExtension($firmwarePath) -notin @('.elf', '.hex')) {
        throw '请使用带有地址信息的 ELF 或 HEX 固件。'
    }
    if (-not $ProgrammerPath) {
        $command = Get-Command STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue
        if ($command) { $ProgrammerPath = $command.Source }
    }
    if (-not $ProgrammerPath) {
        # STM32 VS Code 扩展将下载器安装在版本目录中，按版本选择已安装工具。
        $bundleRoot = Join-Path $env:LOCALAPPDATA 'STM32Cube\bundles\programmer'
        $candidates = @(Get-ChildItem -LiteralPath $bundleRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '^\d+\.\d+\.\d+$' } |
            Sort-Object { [version]$_.Name } -Descending)
        foreach ($candidate in $candidates) {
            $exe = Join-Path $candidate.FullName 'bin\STM32_Programmer_CLI.exe'
            if (Test-Path -LiteralPath $exe) { $ProgrammerPath = $exe; break }
        }
    }
    if (-not $ProgrammerPath) {
        $installed = Join-Path $env:ProgramFiles 'STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
        if (Test-Path -LiteralPath $installed) { $ProgrammerPath = $installed }
    }
    if (-not $ProgrammerPath -or -not (Test-Path -LiteralPath $ProgrammerPath -PathType Leaf)) {
        throw '未找到 STM32CubeProgrammer。请设置 STM32_PROGRAMMER_CLI 环境变量为 STM32_Programmer_CLI.exe 的完整路径。'
    }
    $arguments = @('-c', 'port=SWD', 'freq=4000', "mode=$ConnectMode")
    if ($Serial) { $arguments += "sn=$Serial" }
    $arguments += @('-w', $firmwarePath, '-v', '-rst')
    Write-Host "烧录工具：$ProgrammerPath"
    Write-Host "固件：$firmwarePath"
    if ($DryRun) {
        Write-Host ('仅检查命令，不连接板子：' + ($arguments -join ' '))
        exit 0
    }
    # 不执行全片擦除；CubeProgrammer 按固件覆盖范围写入，然后校验并复位运行。
    $logPath = Join-Path (Split-Path -Parent $firmwarePath) 'flash.log'
    & $ProgrammerPath @arguments 2>&1 | Tee-Object -FilePath $logPath
    if ($LASTEXITCODE -ne 0) { throw "烧录失败，退出码 $LASTEXITCODE。日志：$logPath" }
    Write-Host "烧录命令完成，请核对上方校验结果。日志：$logPath"
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
