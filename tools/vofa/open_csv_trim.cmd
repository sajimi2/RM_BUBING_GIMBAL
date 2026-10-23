@echo off
where pythonw >nul 2>nul
if errorlevel 1 (
    python "%~dp0..\vofa_csv_trim.py"
) else (
    start "" pythonw "%~dp0..\vofa_csv_trim.py"
)
