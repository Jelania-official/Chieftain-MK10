@echo off
set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=python"
set "SCRIPT_PATH=%~dp0pc_debug_controller.py"
set "REQUIREMENTS_PATH=%~dp0requirements-pc-debug.txt"
cd /d "%~dp0\.."
"%PYTHON_EXE%" -c "import PySide6, bleak" >nul 2>&1
if errorlevel 1 (
    echo PC debug dependencies are missing.
    echo Run:
    echo "%PYTHON_EXE%" -m pip install -r "%REQUIREMENTS_PATH%"
    pause
    exit /b 1
)
"%PYTHON_EXE%" "%SCRIPT_PATH%"
if errorlevel 1 pause
