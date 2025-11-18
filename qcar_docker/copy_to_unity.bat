@echo off
REM Script to copy QCar URDF and meshes to Unity project
REM Usage: copy_to_unity.bat "C:\path\to\your\UnityProject"

echo ========================================
echo QCar Unity Import - File Copy Script
echo ========================================
echo.

if "%~1"=="" (
    echo ERROR: No Unity project path provided!
    echo.
    echo Usage: copy_to_unity.bat "C:\path\to\your\UnityProject"
    echo.
    echo Example: copy_to_unity.bat "C:\Users\nicom\Documents\MyUnityProject"
    echo.
    pause
    exit /b 1
)

set UNITY_PROJECT=%~1
set DEST_FOLDER=%UNITY_PROJECT%\Assets\QCar

echo Unity Project: %UNITY_PROJECT%
echo Destination: %DEST_FOLDER%
echo.

REM Check if Unity project exists
if not exist "%UNITY_PROJECT%\Assets" (
    echo ERROR: Invalid Unity project path!
    echo The folder "%UNITY_PROJECT%\Assets" does not exist.
    echo.
    pause
    exit /b 1
)

REM Create QCar folder in Assets
echo Creating QCar folder in Unity Assets...
if not exist "%DEST_FOLDER%" mkdir "%DEST_FOLDER%"

REM Create Materials folder
echo Creating Materials folder...
if not exist "%DEST_FOLDER%\Materials" mkdir "%DEST_FOLDER%\Materials"

REM Create meshes folder
echo Creating meshes folder...
if not exist "%DEST_FOLDER%\meshes" mkdir "%DEST_FOLDER%\meshes"

REM Copy URDF file
echo Copying URDF file...
copy /Y "qcar_unity.urdf" "%DEST_FOLDER%\qcar_unity.urdf"

REM Copy mesh files
echo Copying mesh files...
copy /Y "qcar\rosws\src\qcar_description\meshes\*.stl" "%DEST_FOLDER%\meshes\"

echo.
echo ========================================
echo SUCCESS! Files copied to Unity project
echo ========================================
echo.
echo Location: %DEST_FOLDER%
echo.
echo Next steps:
echo 1. Open your Unity project (if not already open)
echo 2. Wait for Unity to detect the new files
echo 3. In Unity Project window, navigate to Assets/QCar/
echo 4. Right-click on qcar_unity.urdf
echo 5. Select "Import Robot from URDF"
echo.
echo IMPORTANT: The URDF file MUST be inside Assets folder!
echo Unity cannot import URDF files from outside the project.
echo.
pause
