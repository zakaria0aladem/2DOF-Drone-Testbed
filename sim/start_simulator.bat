@echo off
rem Testbed simulator launcher (Windows).
rem   Double-click            -> the simulator window with the demo firmware
rem   Drop a .bin/.elf on it  -> the simulator window with that firmware
rem The first time, it installs the Python packages the simulator needs.

cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo Python was not found. Install Python 3.10 or newer from python.org
    echo and tick "Add python.exe to PATH" in the installer.
    pause
    exit /b 1
)

python -c "import unicorn, serial" >nul 2>nul
if errorlevel 1 (
    echo Installing the simulator's Python packages, only needed the first time...
    python -m pip install -r requirements.txt
    if errorlevel 1 (
        echo.
        echo Installing failed. Check the internet connection and try again.
        pause
        exit /b 1
    )
)

python -m testbed_sim --gui %*
if errorlevel 1 pause
