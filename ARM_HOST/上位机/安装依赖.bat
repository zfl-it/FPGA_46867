@echo off
setlocal
title 安装依赖
echo.
echo   正在准备运行环境（需要联网）...
echo.

rem 依次尝试候选解释器：图形界面依赖 tkinter，精简版 Python 没有它，必须挑一个带 tkinter 的
set "BASE="
for %%P in ("D:/python/python.exe" "C:/Users/Lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe") do (
  if not defined BASE if exist %%P (
    %%P -c "import tkinter" >nul 2>nul
    if not errorlevel 1 set "BASE=%%~P"
  )
)
if not defined BASE (
  echo   没有找到带 tkinter 的 Python，请安装 Python 3.10 及以上版本（勾选 tcl/tk）后重试。
  echo.
  pause
  exit /b 1
)
echo   使用解释器：%BASE%
echo.

if not exist "%~dp0.venv\Scripts\python.exe" (
  echo   正在创建虚拟环境 .venv ...
  "%BASE%" -m venv "%~dp0.venv"
  echo.
)

echo   安装 / 更新：pyserial pillow opencv-python pygame edge-tts ...
"%~dp0.venv\Scripts\python.exe" -m pip install --upgrade pyserial pillow opencv-python pygame edge-tts

echo.
echo   依赖安装完成，双击「启动模拟器.bat」即可运行程序。
echo.
pause
