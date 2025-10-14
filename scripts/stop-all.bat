@echo off
REM scripts\stop-all.bat - Stop all L.A.D services (Windows)

echo ========================================
echo    L.A.D Platform - Stopping Services
echo ========================================
echo.

REM Stop Node/React processes
echo [1/3] Stopping React frontend...
taskkill /FI "WINDOWTITLE eq React Frontend*" /T /F 2>nul
if %ERRORLEVEL%==0 (
    echo [OK] React stopped
) else (
    echo [INFO] React not running
)
echo.

REM Stop Python/Django processes
echo [2/3] Stopping Django backend...
taskkill /FI "WINDOWTITLE eq Django Backend*" /T /F 2>nul
if %ERRORLEVEL%==0 (
    echo [OK] Django stopped
) else (
    echo [INFO] Django not running
)
echo.

REM Stop Docker
echo [3/3] Stopping ROS Docker...
cd qcar_docker
docker compose down
cd ..
echo [OK] Docker stopped
echo.

echo ========================================
echo    All services stopped successfully!
echo ========================================
echo.

pause
