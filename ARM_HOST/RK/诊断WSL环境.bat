@echo off
setlocal
title WSL diag for BL-3 conversion
set DISTRO=Ubuntu-22.04
set PROJ=/mnt/c/Users/Lenovo/Desktop/RK
set LOG=%~dp0wsl_diag.txt
echo Running diagnostics, please wait ...
wsl -d %DISTRO% -u root bash -lc "cd %PROJ% && bash wsl_diag.sh" > "%LOG%" 2>&1
echo.
type "%LOG%"
echo.
echo Log saved to: %LOG%
pause
