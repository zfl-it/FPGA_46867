@echo off
setlocal
title BL-3 to RKNN model conversion
set DISTRO=Ubuntu-22.04
set PROJ=/mnt/c/Users/Lenovo/Desktop/RK
set LOG=%~dp0convert_log.txt

echo ================================================================
echo   BL-3 (12 classes) -^> RK3568 int8 RKNN
echo   step1 export  : /root/lane_work/onnx_env/bin/python (ultralytics)
echo   step2/3 quant : system python3 (rknn-toolkit2 2.3.2)
echo   output        : model\adas12_736x1280_i8_scaled.rknn
echo ================================================================
echo.
echo Running ... this may take 3-10 minutes (ONNX export + int8 quant).
echo Log is also written to: %LOG%
echo.

wsl -d %DISTRO% -u root bash -lc "cd %PROJ% && python3 -u convert_bl3.py" > "%LOG%" 2>&1
set RC=%ERRORLEVEL%

echo ---------------- LOG ----------------
type "%LOG%"
echo -------------------------------------
echo.
if not "%RC%"=="0" (
  echo [FAIL] exit code %RC% - see log above / in %LOG%
) else (
  if exist "%~dp0model\adas12_736x1280_i8_scaled.rknn" (
    echo [DONE] model\adas12_736x1280_i8_scaled.rknn generated
    echo        next: python push_and_build.py  then  python v8_start.py restart
  ) else (
    echo [FAIL] script returned 0 but model file is missing - check log
  )
)
echo.
pause
