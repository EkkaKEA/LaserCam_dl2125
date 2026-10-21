@echo off
rem ---------------------------------------------------------------------------
rem MV-DL2125-04H-R: commands needed for testing dl2125_scan.py
rem Usage: run.cmd N
rem   0 - syntax check            1 - list cameras
rem   2 - dump all camera params  3 - scan PointCloud3D (PLY/PNG/RAW + measure)
rem   4 - scan Range3D            5 - chunk Thickness + Image test
rem   6 - working run (+ PLC/SCADA if MODBUS_HOST / JSON_HOST are set)
rem   7 - save current camera params to file
rem   8..13 - 3D PointCloud3D diagnostics (timeout / free-run / retriggers / burst / reopen / chunks)
rem   14 - brightness check: Origin, free-run, exposure 5000 us, save PNG
rem   15 - 3D scan: PointCloud3D + chunks, save PLY/CSV, measure height/volume
rem   16 - 3D scan: chunks + burst 100 (many frames from one trigger)
rem   17 - 3D scan: chunks + 101 software triggers (line-by-line assembly)
rem   18 - 3D scan with CALIBRATION hypothesis: z-scale 10 (0.1 mm/unit), x 0.3125 mm/pt
rem   19 - PNG with normal brightness: Origin, free-run, exposure 20000 us
rem   20 - POINTS only: PointCloud3D + chunks, save PLY/CSV/RAW
rem   21 - SIZES: PointCloud3D + chunks + measure (x=0.295 by checker cell, z=10 hypothesis)
rem Overridable vars: PY (python), SN (camera serial)
rem ---------------------------------------------------------------------------
chcp 65001 >nul
setlocal

if not defined PY set "PY=python"
if not defined SN set "SN=02DB1432164"

set "HERE=%~dp0"
set "SCRIPT=%HERE%dl2125_scan.py"
set "OUT=%HERE%out"
set "PARAMS=%HERE%dl2125_params_current.txt"

rem NOTE: do not use ( ) blocks around lines that expand paths — paths like
rem "C:\Program Files (x86)\..." break cmd parsing inside if (...) blocks.

if not exist "%SCRIPT%" goto no_script

if "%~1"=="" goto menu
if "%~1"=="0" goto c0
if "%~1"=="1" goto c1
if "%~1"=="2" goto c2
if "%~1"=="3" goto c3
if "%~1"=="4" goto c4
if "%~1"=="5" goto c5
if "%~1"=="6" goto c6
if "%~1"=="7" goto c7
if "%~1"=="8" goto c8
if "%~1"=="9" goto c9
if "%~1"=="10" goto c10
if "%~1"=="11" goto c11
if "%~1"=="12" goto c12
if "%~1"=="13" goto c13
if "%~1"=="14" goto c14
if "%~1"=="15" goto c15
if "%~1"=="16" goto c16
if "%~1"=="17" goto c17
if "%~1"=="18" goto c18
if "%~1"=="19" goto c19
if "%~1"=="20" goto c20
if "%~1"=="21" goto c21
echo [ERROR] unknown command "%~1", use: run.cmd 0..21
exit /b 1

:menu
echo Usage: run.cmd N
echo   0 - syntax check            1 - list cameras
echo   2 - dump all camera params  3 - scan PointCloud3D (PLY/PNG/RAW + measure)
echo   4 - scan Range3D            5 - chunk Thickness + Image test
echo   6 - working run (+ PLC/SCADA if MODBUS_HOST / JSON_HOST are set)
echo   7 - save current camera params to file
echo   8 - 3D scan, one trigger, long timeout 30 s
echo   9 - 3D scan in free-run (no trigger, does ANY frame arrive?)
echo  10 - 3D scan, 201 software triggers then wait 30 s
echo  11 - 3D scan, burst of 100 frames per one trigger
echo  12 - 3D scan, reopen camera after mode change (stream re-registration)
echo  13 - 3D scan with chunk Image + Thickness enabled
echo  14 - brightness check: Origin, free-run, exposure 5000 us, save PNG
echo  15 - 3D scan: PointCloud3D + chunks, save PLY/CSV, measure height/volume
echo  16 - 3D scan: chunks + burst 100 (many frames from one trigger)
echo  17 - 3D scan: chunks + 101 software triggers (line-by-line assembly)
echo  18 - 3D scan with calibration hypothesis: z-scale 10, x-mm-per-px 0.3125
echo  19 - PNG with normal brightness (Origin, exposure 20000 us, free-run)
echo  20 - points only: PointCloud3D + chunks, save PLY/CSV/RAW
echo  21 - sizes: measure with calibrated scales (x 0.295 by checker, z-scale 10)
exit /b 0

:c0
"%PY%" -m py_compile "%SCRIPT%"
if errorlevel 1 (echo [FAIL] py_compile) else (echo [OK] py_compile)
exit /b %errorlevel%

:c1
"%PY%" "%SCRIPT%" --list
exit /b %errorlevel%

:c2
"%PY%" "%SCRIPT%" --serial "%SN%" --dump-params --dump-all
exit /b %errorlevel%

:c3
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --ply --png --raw --measure --out "%OUT%"
exit /b %errorlevel%

:c4
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode Range3D --ply --png --raw --measure --out "%OUT%"
exit /b %errorlevel%

:c5
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --chunk Thickness --chunk Image --png --raw --out "%OUT%"
exit /b %errorlevel%

:c6
set "PUB="
if defined MODBUS_HOST set "PUB=%PUB% --modbus-host %MODBUS_HOST%"
if defined MODBUS_PORT set "PUB=%PUB% --modbus-port %MODBUS_PORT%"
if defined MODBUS_ADDR set "PUB=%PUB% --modbus-addr %MODBUS_ADDR%"
if defined JSON_HOST set "PUB=%PUB% --json-host %JSON_HOST%"
if defined JSON_PORT set "PUB=%PUB% --json-port %JSON_PORT%"
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --measure --out "%OUT%" %PUB%
exit /b %errorlevel%

:c7
"%PY%" "%SCRIPT%" --serial "%SN%" --params-save "%PARAMS%"
exit /b %errorlevel%

:c8
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --timeout 30000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c9
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --free-run --timeout 10000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c10
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --retriggers 200 --timeout 30000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c11
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --burst 100 --timeout 30000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c12
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --reopen --timeout 10000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c13
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --timeout 10000 --ply --png --raw --out "%OUT%"
exit /b %errorlevel%

:c14
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode Origin --reopen --free-run --exposure 5000 --png --raw --out "%OUT%"
exit /b %errorlevel%

:c15
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --ply --csv --measure --timeout 10000 --out "%OUT%"
exit /b %errorlevel%

:c16
"%PY%" "%SCRIPT%" --serial "%SN%" --count 3 --image-mode PointCloud3D --chunk Image --chunk Thickness --burst 100 --ply --csv --measure --timeout 30000 --out "%OUT%"
exit /b %errorlevel%

:c17
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --retriggers 100 --ply --csv --measure --timeout 30000 --out "%OUT%"
exit /b %errorlevel%

:c18
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --z-scale 10 --x-mm-per-px 0.3125 --ply --csv --measure --timeout 10000 --out "%OUT%"
exit /b %errorlevel%

:c19
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode Origin --reopen --free-run --exposure 20000 --png --raw --out "%OUT%"
exit /b %errorlevel%

:c20
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --ply --csv --raw --timeout 10000 --out "%OUT%"
exit /b %errorlevel%

:c21
"%PY%" "%SCRIPT%" --serial "%SN%" --count 1 --image-mode PointCloud3D --chunk Image --chunk Thickness --z-scale 10 --x-mm-per-px 0.295 --ply --csv --measure --timeout 10000 --out "%OUT%"
exit /b %errorlevel%

:no_script
echo [ERROR] script not found: %SCRIPT%
exit /b 1
