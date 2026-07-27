@echo off
setlocal
set "TOOL_DIR=%~dp0"
cd /d "%TOOL_DIR%"
if errorlevel 1 (
    echo Failed to enter tool directory:
    echo "%TOOL_DIR%"
    pause
    exit /b 1
)

where python >nul 2>nul
if errorlevel 1 (
    set "PY_CMD=py -3"
) else (
    set "PY_CMD=python"
)

%PY_CMD% -c "import serial; import PySide6; import pyqtgraph; import OpenGL" >nul 2>nul
if errorlevel 1 (
    echo Installing Python dependencies...
    %PY_CMD% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo.
        echo Failed to install Python dependencies.
        echo Please try manually:
        echo %PY_CMD% -m pip install -r requirements.txt
        echo.
        pause
        exit /b 1
    )
)

%PY_CMD% imu_visualizer.py
if errorlevel 1 (
    echo.
    echo Failed to start IMU visualizer.
    echo You can try manually:
    echo %PY_CMD% imu_visualizer.py
    echo.
    pause
)
