@echo off
echo Starting ElectricShock Racing Car Control System...
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.7+ and try again
    pause
    exit /b 1
)

REM Check if virtual environment exists
if not exist "venv\" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
echo Activating virtual environment...
call venv\Scripts\activate.bat

REM Install dependencies
echo Installing dependencies...
pip install -r requirements.txt

REM Change to webcontroller directory
cd webcontroller

REM Start the Flask application
echo.
echo Starting Flask application...
echo Web interface will be available at: http://localhost:5000
echo Press Ctrl+C to stop the server
echo.
python uart_car_control.py

REM Deactivate virtual environment when done
deactivate

echo.
echo Server stopped.
pause
