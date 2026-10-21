@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\start_tuning_panel.ps1"
if errorlevel 1 pause
