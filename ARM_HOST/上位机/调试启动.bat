@echo off
setlocal
title 调试启动 - 交规决策模拟器
set "SCRIPT=%~dp0traffic_rules_sim.py"
set "PY=%~dp0.venv\Scripts\python.exe"

if not exist "%PY%" set "PY=python.exe"

echo ============================================
echo   调试启动（控制台模式）
echo   解释器 : %PY%
echo   脚本   : %SCRIPT%
echo ============================================
echo.

"%PY%" "%SCRIPT%"

echo.
echo   退出代码 = %ERRORLEVEL%
echo   如果上面有 Python 报错，请把它截图发出来。
echo   （如果图形界面已经正常打开，直接关掉这个黑框即可。）
echo.
pause
