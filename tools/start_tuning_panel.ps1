$ErrorActionPreference = 'Stop'
try {
    $panelRoot = Split-Path -Parent $PSScriptRoot
    $pythonCommand = Get-Command python.exe -ErrorAction Stop
    $pythonWindowless = Join-Path (Split-Path -Parent $pythonCommand.Source) 'pythonw.exe'
    if (-not (Test-Path -LiteralPath $pythonWindowless)) {
        throw 'pythonw.exe was not found next to python.exe. Install the standard Python distribution.'
    }
    $panelScript = Join-Path $PSScriptRoot 'tuning_panel.py'
    Start-Process -FilePath $pythonWindowless -ArgumentList @('"' + $panelScript + '"') `
        -WorkingDirectory $panelRoot -WindowStyle Hidden
} catch {
    Write-Error $_.Exception.Message
    exit 1
}
