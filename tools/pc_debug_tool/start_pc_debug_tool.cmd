@echo off
setlocal
set "TOOL_DIR=%~dp0"
cd /d "%TOOL_DIR%"
if errorlevel 1 (
    echo Failed to enter the tool directory:
    echo "%TOOL_DIR%"
    pause
    exit /b 1
)

rem Run the interpreter instead of relying on where.exe. Windows app aliases
rem can make python runnable even when where.exe cannot find it.
set "PY_CMD="
python -c "import sys; raise SystemExit(0 if sys.version_info.major == 3 else 1)" >nul 2>nul
if not errorlevel 1 set "PY_CMD=python"
if defined PY_CMD goto python_found

py -3 -c "import sys; raise SystemExit(0 if sys.version_info.major == 3 else 1)" >nul 2>nul
if not errorlevel 1 set "PY_CMD=py -3"
if defined PY_CMD goto python_found

echo Python 3 was not found.
echo Install Python 3 and enable "Add Python to PATH", then try again.
pause
exit /b 1

:python_found
%PY_CMD% -c "import serial; import PySide6; import pyqtgraph" >nul 2>nul
if errorlevel 1 (
    echo Installing Python dependencies...
    %PY_CMD% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo.
        echo Failed to install Python dependencies.
        echo Try this command manually:
        echo %PY_CMD% -m pip install -r requirements.txt
        echo.
        pause
        exit /b 1
    )
)

if /i "%~1"=="--check" (
    echo Launcher check passed using: %PY_CMD%
    exit /b 0
)

%PY_CMD% pc_debug_tool.py
if errorlevel 1 (
    echo.
    echo Failed to start the Chieftain MK10 debug tool.
    echo Try this command manually:
    echo %PY_CMD% pc_debug_tool.py
    echo.
    pause
)
