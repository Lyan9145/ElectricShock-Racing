#!/bin/bash
echo "=== ElectricShock Racing Launch Control System ==="
echo "Checking requirements..."

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: Python3 is not installed"
    exit 1
fi

# Check if pip is installed
if ! command -v pip3 &> /dev/null; then
    echo "Error: pip3 is not installed"
    exit 1
fi

# Install requirements if needed
echo "Installing/checking Python dependencies..."
pip3 install -r requirements.txt

# Check camera permissions
echo "Checking camera permissions..."
if [ ! -r /dev/video0 ]; then
    echo "Warning: Cannot access /dev/video0. You may need to:"
    echo "  1. Connect a USB camera"
    echo "  2. Add your user to video group: sudo usermod -a -G video $USER"
    echo "  3. Logout and login again"
fi

# Check serial port permissions
echo "Checking serial port permissions..."
if [ ! -w /dev/ttyUSB0 ]; then
    echo "Warning: Cannot access /dev/ttyUSB0. You may need to:"
    echo "  1. Connect the car control device"
    echo "  2. Add your user to dialout group: sudo usermod -a -G dialout $USER"
    echo "  3. Logout and login again"
fi
# 在screen中启动launchcontrol/app.py
screen -dmS launchctl python3 launchcontrol/app.py