@echo off
chcp 65001 >nul
setlocal
set "TOOL_DIR=%~dp0"
cd /d "%TOOL_DIR%"
if errorlevel 1 (
    echo 无法进入工具目录：
    echo "%TOOL_DIR%"
    pause
    exit /b 1
)

where python >nul 2>nul
if not errorlevel 1 (
    set "PY_CMD=python"
) else (
    where py >nul 2>nul
    if errorlevel 1 (
        echo 未找到 Python 3。
        echo 请安装 Python 3，并勾选“Add Python to PATH”，然后重试。
        pause
        exit /b 1
    )
    set "PY_CMD=py -3"
)

%PY_CMD% -c "import serial; import PySide6; import pyqtgraph" >nul 2>nul
if errorlevel 1 (
    echo 正在安装 Python 依赖，请稍候……
    %PY_CMD% -m pip install -r pc_debug_tool_requirements.txt
    if errorlevel 1 (
        echo.
        echo Python 依赖安装失败。
        echo 请尝试手动执行：
        echo %PY_CMD% -m pip install -r pc_debug_tool_requirements.txt
        echo.
        pause
        exit /b 1
    )
)

%PY_CMD% pc_debug_tool.py
if errorlevel 1 (
    echo.
    echo Chieftain MK10 整车调试工具启动失败。
    echo 请尝试手动执行：
    echo %PY_CMD% pc_debug_tool.py
    echo.
    pause
)
