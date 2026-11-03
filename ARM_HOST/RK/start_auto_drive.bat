@echo off
setlocal

set "RK_DIR=%~dp0."
set "SIM_DIR=%~dp0..\上位机"
set "PY=D:\python\python.exe"

echo ============================================
echo   自动驾驶链路一键启动
echo   [1/3] RK3568: ADAS 感知 + 串口下发
echo   [2/3] RK3568: HDMI 本地显示
echo   [3/3] PC   : 交规决策模拟器
echo ============================================
echo.

if not exist "%PY%" (
    echo [ERROR] 找不到 %PY%
    pause
    exit /b 1
)

echo [1/3] 启动 RK3568 端（自动检查并加载 PCIe 驱动）...
pushd "%RK_DIR%"
"%PY%" v8_start.py start
popd
echo.

echo [2/3] 启动 HDMI 本地显示（杀桌面占用 + 锁定 HDMI-A-1 + NV12 直送）...
pushd "%RK_DIR%"
"%PY%" v8_hdmi_display.py start --fast --connector-id 175 --kill-desktop
popd
echo.

echo [3/3] 启动上位机交规决策模拟器...
if exist "%SIM_DIR%\启动模拟器.bat" (
    start "" cmd /c ""%SIM_DIR%\启动模拟器.bat""
) else if exist "%SIM_DIR%\TrafficRuleSim.exe" (
    start "" "%SIM_DIR%\TrafficRuleSim.exe"
) else (
    echo [WARN] 未找到上位机启动入口，请手动打开 %SIM_DIR%
)
echo.

echo ============================================
echo   启动完成。验收清单：
echo   1. HDMI 屏出现推理画面（左上角 ADAS 位串）
echo   2. 模拟器界面打开 A 路串口（115200-8-N-1，无流控）
echo   3. A 路应以 20Hz 刷新 "xxxxxx N" 帧
echo      位序: 左实线 右实线 直行绿 左转绿 右转绿 斑马线
echo   4. 视频流: rtsp://192.168.137.100:8558/live/plate
echo ============================================
echo   注意: 不要再用 v8_ctrl.py 启动 —— 那是旧的车牌模式，
echo         会把 ADAS 进程顶掉，屏幕左上角会变成 "No plate"。
echo ============================================
echo.
pause
