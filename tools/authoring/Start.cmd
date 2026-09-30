@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import sys, tkinter; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
    if not errorlevel 1 (
        py -3 "%~dp0app.py"
        if errorlevel 1 pause
        exit /b
    )
)
where python >nul 2>nul
if not errorlevel 1 (
    python -c "import sys, tkinter; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
    if not errorlevel 1 (
        python "%~dp0app.py"
        if errorlevel 1 pause
        exit /b
    )
)
echo Python 3.10 or newer with Tkinter is required.
echo Install Python for Windows with the Tcl/Tk component, then run Start.cmd again.
pause
exit /b 1
