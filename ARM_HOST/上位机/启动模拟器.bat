@echo off
setlocal
set "SCRIPT=%~dp0traffic_rules_sim.py"
set "PYW=%~dp0.venv\Scripts\pythonw.exe"
set "PY=%~dp0.venv\Scripts\python.exe"

if not exist "%PYW%" set "PYW=pythonw.exe"
if not exist "%PY%" set "PY=python.exe"

start "" "%PYW%" "%SCRIPT%"

timeout /t 2 /nobreak >nul
tasklist /FI "IMAGENAME eq pythonw.exe" 2>nul | find /I "pythonw.exe" >nul
if not errorlevel 1 goto :eof

echo.
echo   界面没有成功启动，正在以控制台模式重试，请查看下方的错误信息：
echo.
"%PY%" "%SCRIPT%"
echo.
echo   退出代码 = %ERRORLEVEL%
echo.
pause
