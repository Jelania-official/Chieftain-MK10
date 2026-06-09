@echo off
cd /d "%~dp0\.."
set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=python"
"%PYTHON_EXE%" tools\pc_debug_controller.py
if errorlevel 1 pause
