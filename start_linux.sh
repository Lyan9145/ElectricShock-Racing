#!/bin/bash

# ElectricShock Racing - USB Camera Car Control System
# Linux startup script

echo "=== ElectricShock Racing Car Control System ==="
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

echo ""
echo "Starting car control system in a detached screen session..."
echo "Access the web interface at: http://localhost:5000"
echo "Or from other devices at: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "To view the running session: screen -r car_control"
echo "To detach: Ctrl+A then D"
echo ""

# Start the server in a detached screen session named 'car_control'
screen -dmS car_control python3 webcontroller/uart_car_control.py
