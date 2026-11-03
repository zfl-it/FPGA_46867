@echo off
setlocal
title ¹æÔò×Ô²â
set "SCRIPT=%~dp0traffic_rules_sim.py"
set "PY=%~dp0.venv\Scripts\python.exe"

if not exist "%PY%" set "PY=python.exe"

set PYTHONUTF8=1
"%PY%" "%SCRIPT%" --selftest
echo.
pause
