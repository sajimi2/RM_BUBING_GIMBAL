param(
    [ValidateSet('Configure', 'Build', 'Rebuild', 'Flash')]
    [string]$Action = 'Build',
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug'
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding
$projectRoot = Split-Path -Parent $PSScriptRoot

function Find-CubeTool([string]$Name, [string]$Extension, [string]$RelativePath) {
    $available = Get-Command $Name -ErrorAction SilentlyContinue
    if ($available) { return $available.Source }
    # 普通终端未继承扩展 PATH 时，查找本机已安装的官方扩展包装器。
    $extensionsRoot = Join-Path $env:USERPROFILE '.vscode\extensions'
    $candidates = Get-ChildItem -LiteralPath $extensionsRoot -Directory |
        Where-Object { $_.Name -like "$Extension-*" } | Sort-Object Name -Descending
    foreach ($extensionDirectory in $candidates) {
        $candidate = Join-Path $extensionDirectory.FullName $RelativePath
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
    }
    throw "未找到 $Name，请先安装 STM32CubeIDE for VS Code 扩展及工具包。"
}

try {
    $cube = Find-CubeTool 'cube.exe' 'stmicroelectronics.stm32cube-ide-core' 'resources\binaries\win32\x86_64\cube.exe'
    $cubeCmake = Find-CubeTool 'cube-cmake.exe' 'stmicroelectronics.stm32cube-ide-build-cmake' 'resources\cube-cmake\win32\x86_64\cube-cmake.exe'
    # 仅修改当前任务进程环境，工具版本由 STM32Cube Bundle Manager 选择。
    $env:PATH = "$(Split-Path -Parent $cube);$(Split-Path -Parent $cubeCmake);$env:PATH"
    Push-Location -LiteralPath $projectRoot
    try {
        & $cubeCmake --preset $Configuration
        if ($LASTEXITCODE -ne 0) { throw 'CMake 配置失败，请查看上方输出。' }
        if ($Action -eq 'Configure') { exit 0 }

        $buildArguments = @('--build', '--preset', $Configuration, '--parallel')
        if ($Action -eq 'Rebuild') { $buildArguments += '--clean-first' }
        & $cubeCmake @buildArguments
        if ($LASTEXITCODE -ne 0) { throw '构建失败，未执行烧录。' }
        $elf = Join-Path $projectRoot "build\$Configuration\standard_robot.elf"
        if (-not (Test-Path -LiteralPath $elf -PathType Leaf)) { throw "缺少固件：$elf" }
        Write-Host "构建完成：$elf"

        if ($Action -eq 'Flash') {
            Write-Host '通过 ST-Link/SWD 烧录并校验，完成后复位运行...'
            & $cube programmer -c port=SWD -w $elf -v -rst
            if ($LASTEXITCODE -ne 0) { throw '烧录失败，请查看 CubeProgrammer 输出。' }
        }
    } finally {
        Pop-Location
    }
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
