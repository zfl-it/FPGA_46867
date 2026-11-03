@echo off
setlocal
title ΩÁ√Ê√∞—Ã≤‚ ‘
set "SCRIPT=%~dp0smoke_test.py"
set "PY=%~dp0.venv\Scripts\python.exe"

if not exist "%PY%" set "PY=python.exe"

set PYTHONUTF8=1
"%PY%" "%SCRIPT%"
echo.
pause
