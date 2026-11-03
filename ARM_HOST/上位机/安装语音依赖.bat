@echo off
setlocal
title 安装人声语音依赖
echo.
echo   正在为「交规决策模拟器」安装人声提示依赖（需要联网）：
echo       edge-tts   在线神经网络语音（阿/中/英/法/俄/西 六种语言）
echo       pygame     播放合成出来的 mp3
echo.
echo   说明：不安装也能运行，程序会自动降级到 Windows 系统语音，
echo         再不行就用蜂鸣兜底；但六种语言里只有中文/英文有离线语音。
echo.

if not exist "%~dp0.venv\Scripts\python.exe" (
  echo   未找到 .venv，请先双击「安装依赖.bat」创建虚拟环境。
  echo.
  pause
  exit /b 1
)

echo   安装 / 更新 edge-tts 与 pygame ...
"%~dp0.venv\Scripts\python.exe" -m pip install --upgrade edge-tts pygame

echo.
echo   安装完成。重新启动「交规决策模拟器」后，
echo   在「决策助手」卡片里选择语言并点「试听」即可验证。
echo.
pause
