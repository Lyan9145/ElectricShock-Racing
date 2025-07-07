import serial
import struct
import time
from flask import Flask, render_template, request, jsonify, Response, send_file
import atexit
import cv2
import threading
import base64
import json
from datetime import datetime
import os
import platform
import subprocess
import shutil
from pathlib import Path
from uart_proxy import UartProxy
import glob
import re

# USB通信帧
USB_FRAME_HEAD = 0x42
USB_ADDR_CARCTRL = 1

# --- Configuration ---
# 移除Windows支持和配置文件
# CONFIG_FILE = 'serial_config.json'
def find_linux_serial_port():
    usb_ports = sorted(glob.glob('/dev/ttyUSB*'))
    if usb_ports:
        return usb_ports[0]
    else:
        return None

SERIAL_PORT = find_linux_serial_port()
BAUD_RATE = 115200

# Camera Configuration
CAMERA_DEFAULT_INDEX = 0
CAMERA_DEFAULT_WIDTH = 1024
CAMERA_DEFAULT_HEIGHT = 768
CAMERA_DEFAULT_FPS = 30
CAMERA_DEFAULT_QUALITY = 80  # JPEG quality (0-100)

# Default values
SPEED_DEFAULT = 0.0
SERVO_MID_DEFAULT = 5000
CONTINUOUS_SEND_INTERVAL_DEFAULT = 200 # milliseconds

# Servo range configuration
# Actual device hardware limits (for final clamping before sending)
DEVICE_SERVO_MIN_HW = 0
DEVICE_SERVO_MAX_HW = 65536
# UI Slider default configuration range
SERVO_SLIDER_MIN_UI_DEFAULT = 4000
SERVO_SLIDER_MAX_UI_DEFAULT = 6000
# Absolute uint16 limits for UI configuration inputs
UINT16_MIN = 0
UINT16_MAX = 65535

# Global serial object
ser = None

# Camera globals
camera = None
camera_thread = None
camera_running = False
camera_lock = threading.Lock()
current_frame = None

# Camera settings
current_camera_index = CAMERA_DEFAULT_INDEX
current_camera_width = CAMERA_DEFAULT_WIDTH
current_camera_height = CAMERA_DEFAULT_HEIGHT
current_camera_fps = CAMERA_DEFAULT_FPS
current_camera_quality = CAMERA_DEFAULT_QUALITY

# Initialize UART proxy
uart_proxy = None

app = Flask(__name__)

# --- Application State (for UI rendering) ---
current_speed_val = SPEED_DEFAULT
current_servo_val = SERVO_MID_DEFAULT
current_servo_min_slider_val = SERVO_SLIDER_MIN_UI_DEFAULT # For the main slider's range
current_servo_max_slider_val = SERVO_SLIDER_MAX_UI_DEFAULT # For the main slider's range
current_continuous_send_interval = CONTINUOUS_SEND_INTERVAL_DEFAULT


def init_serial(port_name):
    global ser
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        print(f"Serial port {port_name} opened successfully at {BAUD_RATE} baud.")
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {port_name}: {e}")
        ser = None
        return False

def close_serial_port():
    global ser
    if ser and ser.is_open:
        print("Sending stop command before closing port.")
        send_car_control_command(0, 0)
        time.sleep(0.1)
        ser.close()
        print("Serial port closed.")
        ser = None

atexit.register(close_serial_port)


def send_car_control_command(speed, servo):
    if not ser or not ser.is_open:
        return False

    speed = float(speed)
    servo = int(servo)
    # Final safety clamp to actual hardware limits, regardless of UI settings
    servo = max(DEVICE_SERVO_MIN_HW, min(DEVICE_SERVO_MAX_HW, servo))

    frame_len = 10
    command_buffer = bytearray(frame_len)
    command_buffer[0] = USB_FRAME_HEAD
    command_buffer[1] = USB_ADDR_CARCTRL
    command_buffer[2] = frame_len
    struct.pack_into("<f", command_buffer, 3, speed)
    struct.pack_into("<H", command_buffer, 7, servo) # Servo is uint16
    checksum = sum(command_buffer[:frame_len-1]) % 256
    command_buffer[frame_len - 1] = checksum

    try:
        ser.write(command_buffer)
        return True
    except Exception as e:
        print(f"Error sending command: {e}")
        return False

# --- Camera Functions ---
def init_camera(camera_index=0, width=640, height=480, fps=30, quality=80):
    """Initialize camera for MJPEG streaming"""
    global camera, current_camera_index, current_camera_width
    global current_camera_height, current_camera_fps, current_camera_quality
    
    try:
        # Stop existing camera
        stop_camera()
        
        # Set platform-specific camera backend
        if platform.system() == "Windows":
            try:
                camera = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
                if not camera.isOpened():
                    camera = cv2.VideoCapture(camera_index)
            except:
                camera = cv2.VideoCapture(camera_index)
        elif platform.system() == "Linux":
            try:
                camera = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
                if not camera.isOpened():
                    camera = cv2.VideoCapture(camera_index)
            except:
                camera = cv2.VideoCapture(camera_index)
        else:
            camera = cv2.VideoCapture(camera_index)
        
        if not camera.isOpened():
            print(f"Error: Cannot open camera {camera_index}")
            return False
        
        # Set camera properties
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer for low latency
        
        # Try to set MJPEG format for better performance
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Get actual camera settings
        actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = camera.get(cv2.CAP_PROP_FPS)
        
        # Update current settings
        current_camera_index = camera_index
        current_camera_width = actual_width
        current_camera_height = actual_height
        current_camera_fps = actual_fps
        current_camera_quality = quality
        
        print(f"Camera {camera_index} initialized: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS for MJPEG streaming")
        return True
        
    except Exception as e:
        print(f"Error initializing camera: {e}")
        camera = None
        return False

def camera_thread_func():
    """Camera capture thread for MJPEG streaming"""
    global camera, camera_running, current_frame
    
    while camera_running and camera is not None:
        try:
            ret, frame = camera.read()
            if ret:
                with camera_lock:
                    current_frame = frame.copy()
                time.sleep(1.0 / current_camera_fps)
            else:
                print("Failed to read frame from camera")
                time.sleep(0.1)
        except Exception as e:
            print(f"Camera thread error: {e}")
            time.sleep(0.5)
    
    print("Camera thread exiting")

def start_camera():
    """Start camera streaming"""
    global camera_thread, camera_running
    
    if camera is None:
        print("Camera not initialized")
        return False
    
    if camera_running:
        print("Camera streaming already running")
        return True
    
    print("Starting camera streaming...")
    
    # Start camera thread
    camera_running = True
    camera_thread = threading.Thread(target=camera_thread_func, daemon=True)
    camera_thread.start()
    
    print("Camera streaming started successfully")
    return True

def stop_camera():
    """Stop camera streaming"""
    global camera, camera_thread, camera_running, current_frame
    
    print("Stopping camera streaming...")
    camera_running = False
    
    # Stop camera thread
    if camera_thread is not None:
        camera_thread.join(timeout=5)
        if camera_thread.is_alive():
            print("Warning: Camera thread did not stop gracefully")
        camera_thread = None
    
    # Clear current frame
    with camera_lock:
        current_frame = None
    
    # Release camera
    if camera is not None:
        try:
            camera.release()
        except:
            pass
        camera = None
    
    print("Camera streaming stopped")

def generate_mjpeg():
    """Generate MJPEG stream"""
    global current_frame
    
    while True:
        with camera_lock:
            if current_frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', current_frame, 
                                         [cv2.IMWRITE_JPEG_QUALITY, current_camera_quality])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(1.0 / current_camera_fps)

def update_camera_settings(camera_index=None, width=None, height=None, fps=None, quality=None):
    """Update camera settings"""
    global current_camera_index, current_camera_width, current_camera_height
    global current_camera_fps, current_camera_quality
    
    with camera_lock:
        restart_needed = False
        
        # Check if restart is needed
        if camera_index is not None and camera_index != current_camera_index:
            restart_needed = True
        if width is not None and width != current_camera_width:
            restart_needed = True
        if height is not None and height != current_camera_height:
            restart_needed = True
        if fps is not None and fps != current_camera_fps:
            restart_needed = True
        if quality is not None and quality != current_camera_quality:
            current_camera_quality = quality  # Quality can be updated without restart
        
        if restart_needed:
            was_running = camera_running
            if was_running:
                stop_camera()
            
            # Update settings
            new_index = camera_index if camera_index is not None else current_camera_index
            new_width = width if width is not None else current_camera_width
            new_height = height if height is not None else current_camera_height
            new_fps = fps if fps is not None else current_camera_fps
            new_quality = quality if quality is not None else current_camera_quality
            
            success = init_camera(new_index, new_width, new_height, new_fps, new_quality)
            
            if success and was_running:
                start_camera()
            
            return success
        
        return True

# Register camera cleanup
def cleanup_camera():
    stop_camera()

atexit.register(cleanup_camera)

# --- Configuration Functions ---
def get_default_serial_port():
    """Get default serial port based on operating system"""
    if platform.system() == "Windows":
        return "COM3"  # Common Windows serial port
    else:
        return "/dev/ttyUSB0"  # Linux/Unix serial port

# --- Serial Data Streaming (Distance, Voltage, Speed) ---
serial_data_pattern = re.compile(r'D:(-?\d+\.\d+) , V:(-?\d+\.\d+) , S:(-?\d+\.\d+)')
latest_serial_data = {
    'distance': None,
    'voltage': None,
    'speed': None,
    'timestamp': None
}
serial_data_lock = threading.Lock()
serial_data_thread = None
serial_data_thread_running = False

def serial_data_reader():
    global ser, serial_data_thread_running, latest_serial_data
    serial_data_thread_running = True
    while serial_data_thread_running:
        try:
            if ser and ser.is_open:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    m = serial_data_pattern.match(line)
                    if m:
                        with serial_data_lock:
                            latest_serial_data['distance'] = float(m.group(1))
                            latest_serial_data['voltage'] = float(m.group(2))
                            latest_serial_data['speed'] = float(m.group(3))
                            latest_serial_data['timestamp'] = datetime.now().isoformat()
            else:
                time.sleep(0.2)
        except Exception as e:
            time.sleep(0.2)

def start_serial_data_thread():
    global serial_data_thread
    if serial_data_thread is None or not serial_data_thread.is_alive():
        t = threading.Thread(target=serial_data_reader, daemon=True)
        t.start()
        serial_data_thread = t

def stop_serial_data_thread():
    global serial_data_thread_running
    serial_data_thread_running = False

atexit.register(stop_serial_data_thread)

@app.route('/serial/stream')
def serial_data_stream():
    def event_stream():
        last_sent = None
        while True:
            with serial_data_lock:
                data = latest_serial_data.copy()
            if data['timestamp'] != last_sent and data['distance'] is not None:
                yield f"data: {json.dumps(data)}\n\n"
                last_sent = data['timestamp']
            time.sleep(0.1)
    return Response(event_stream(), mimetype='text/event-stream')

# --- Flask Web Interface ---

@app.route('/')
def index():
    global current_speed_val, current_servo_val, current_servo_min_slider_val, \
           current_servo_max_slider_val, current_continuous_send_interval
    return render_template('index.html',
                          current_speed=current_speed_val,
                          current_servo=current_servo_val,
                          current_servo_min_slider=current_servo_min_slider_val,
                          current_servo_max_slider=current_servo_max_slider_val,
                          current_continuous_interval=current_continuous_send_interval,
                          servo_mid_default=SERVO_MID_DEFAULT,
                          device_servo_min_hw=DEVICE_SERVO_MIN_HW,
                          device_servo_max_hw=DEVICE_SERVO_MAX_HW,
                          uint16_min=UINT16_MIN,
                          uint16_max=UINT16_MAX
                          )

@app.route('/control', methods=['POST'])
def control_car():
    global current_speed_val, current_servo_val
    data = request.get_json()
    speed = data.get('speed', SPEED_DEFAULT)
    servo = data.get('servo', SERVO_MID_DEFAULT) # Servo value received is already within UI slider's range

    current_speed_val = float(speed)
    current_servo_val = int(servo)

    if send_car_control_command(current_speed_val, current_servo_val):
        return jsonify({'success': True, 'message': 'Command sent.'})
    else:
        return jsonify({'success': False, 'message': 'Failed to send command (check console).'}), 200

# --- Flask Routes ---
@app.route('/video_feed')
def video_feed():
    """MJPEG video stream endpoint"""
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera/start', methods=['POST'])
def start_camera_route():
    """Start camera streaming"""
    if camera is None:
        return jsonify({'success': False, 'message': 'Camera not initialized'})
    
    success = start_camera()
    if success:
        return jsonify({
            'success': True, 
            'message': 'Camera streaming started',
            'stream_url': '/video_feed'
        })
    else:
        return jsonify({'success': False, 'message': 'Failed to start camera streaming'})

@app.route('/camera/stop', methods=['POST'])
def stop_camera_route():
    """Stop camera streaming"""
    stop_camera()
    return jsonify({'success': True, 'message': 'Camera streaming stopped'})

@app.route('/camera/settings', methods=['GET', 'POST'])
def camera_settings_route():
    """Get or update camera settings"""
    global current_camera_index, current_camera_width, current_camera_height
    global current_camera_fps, current_camera_quality, camera_running
    
    if request.method == 'GET':
        return jsonify({
            'camera_index': current_camera_index,
            'width': current_camera_width,
            'height': current_camera_height,
            'fps': current_camera_fps,
            'quality': current_camera_quality,
            'running': camera_running,
            'streaming_type': 'MJPEG',
            'stream_url': '/video_feed'
        })
    
    elif request.method == 'POST':
        data = request.get_json()
        
        camera_index = data.get('camera_index')
        width = data.get('width')
        height = data.get('height')
        fps = data.get('fps')
        quality = data.get('quality')
        
        success = update_camera_settings(camera_index, width, height, fps, quality)
        
        if success:
            return jsonify({
                'success': True,
                'message': 'Camera settings updated',
                'settings': {
                    'camera_index': current_camera_index,
                    'width': current_camera_width,
                    'height': current_camera_height,
                    'fps': current_camera_fps,
                    'quality': current_camera_quality,
                    'running': camera_running,
                    'streaming_type': 'MJPEG',
                    'stream_url': '/video_feed'
                }
            })
        else:
            return jsonify({'success': False, 'message': 'Failed to update camera settings'})

@app.route('/camera/init', methods=['POST'])
def init_camera_route():
    """Initialize camera"""
    data = request.get_json()
    
    camera_index = data.get('camera_index', CAMERA_DEFAULT_INDEX)
    width = data.get('width', CAMERA_DEFAULT_WIDTH)
    height = data.get('height', CAMERA_DEFAULT_HEIGHT)
    fps = data.get('fps', CAMERA_DEFAULT_FPS)
    quality = data.get('quality', CAMERA_DEFAULT_QUALITY)
    
    success = init_camera(camera_index, width, height, fps, quality)
    
    if success:
        return jsonify({
            'success': True, 
            'message': 'Camera initialized for MJPEG streaming',
            'streaming_type': 'MJPEG',
            'stream_url': '/video_feed'
        })
    else:
        return jsonify({'success': False, 'message': 'Failed to initialize camera'})

@app.route('/serial/settings', methods=['GET', 'POST'])
def serial_settings_route():
    """Get or update serial port settings (Linux only, auto-detect)"""
    global SERIAL_PORT, BAUD_RATE, ser

    if request.method == 'GET':
        return jsonify({
            'serial_port': SERIAL_PORT,
            'baud_rate': BAUD_RATE,
            'connected': ser is not None and ser.is_open if ser else False
        })

    elif request.method == 'POST':
        # 仅允许刷新串口（重新自动检测）
        SERIAL_PORT_NEW = find_linux_serial_port()
        if SERIAL_PORT_NEW is None:
            return jsonify({'success': False, 'message': 'No /dev/ttyUSB* device found'})
        close_serial_port()
        SERIAL_PORT = SERIAL_PORT_NEW
        if init_serial(SERIAL_PORT):
            return jsonify({
                'success': True,
                'message': 'Serial port auto-detected and connected',
                'settings': {
                    'serial_port': SERIAL_PORT,
                    'baud_rate': BAUD_RATE,
                    'connected': True
                }
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Failed to connect to detected serial port',
                'settings': {
                    'serial_port': SERIAL_PORT,
                    'baud_rate': BAUD_RATE,
                    'connected': False
                }
            })

@app.route('/serial/connect', methods=['POST'])
def serial_connect_route():
    """Connect to serial port with current settings (auto-detect on Linux)"""
    global SERIAL_PORT
    SERIAL_PORT = find_linux_serial_port()
    if SERIAL_PORT is None:
        return jsonify({'success': False, 'message': 'No /dev/ttyUSB* device found'})
    if init_serial(SERIAL_PORT):
        return jsonify({'success': True, 'message': 'Serial port connected'})
    else:
        return jsonify({'success': False, 'message': 'Failed to connect to serial port'})

@app.route('/serial/disconnect', methods=['POST'])
def serial_disconnect_route():
    """Disconnect from serial port"""
    close_serial_port()
    return jsonify({'success': True, 'message': 'Serial port disconnected'})

# Add MJPEG status endpoint
@app.route('/camera/status')
def camera_status():
    """Get camera streaming status"""
    global camera_running, camera 
    
    status = {
        'running': camera_running,
        'camera_available': camera is not None,
        'stream_url': '/video_feed' if camera_running else None,
        'streaming_type': 'MJPEG',
        'camera_settings': {
            'index': current_camera_index,
            'width': current_camera_width,
            'height': current_camera_height,
            'fps': current_camera_fps,
            'quality': current_camera_quality
        }
    }
    
    return jsonify(status)

@app.route('/proxy/init', methods=['POST'])
def init_uart_proxy():
    """Initialize UART proxy."""
    global uart_proxy
    data = request.get_json()
    physical_port = data.get('physical_port', SERIAL_PORT)
    baud_rate = data.get('baud_rate', BAUD_RATE)
    num_ports = data.get('num_ports', 2)
    prefix = data.get('prefix', 'ttyPX')

    if uart_proxy is None:
        uart_proxy = UartProxy(physical_port, baud_rate, num_ports, prefix)
        threading.Thread(target=uart_proxy.start, daemon=True).start()
        return jsonify({'success': True, 'message': 'UART proxy initialized'})
    else:
        return jsonify({'success': False, 'message': 'UART proxy already running'})

@app.route('/proxy/toggle_forwarding', methods=['POST'])
def toggle_forwarding():
    """Toggle forwarding for a virtual port."""
    global uart_proxy
    if uart_proxy is None:
        return jsonify({'success': False, 'message': 'UART proxy not initialized'})

    data = request.get_json()
    virtual_port_index = data.get('virtual_port_index')
    enable = data.get('enable', True)

    if virtual_port_index is None:
        return jsonify({'success': False, 'message': 'Missing virtual_port_index'})

    uart_proxy.toggle_forwarding(virtual_port_index, enable)
    return jsonify({'success': True, 'message': f'Forwarding {"enabled" if enable else "disabled"} for virtual port {virtual_port_index}'})

if __name__ == '__main__':
    print(f"Starting application on {platform.system()}")
    # 移除串口配置文件加载和输入
    print("Auto-detecting serial port on Linux...")
    SERIAL_PORT = find_linux_serial_port()
    if SERIAL_PORT is None:
        print("No /dev/ttyUSB* device found. Exiting.")
        exit(1)
    print(f"Using serial port: {SERIAL_PORT}")

    # Initialize camera for MJPEG streaming
    print(f"Initializing camera on {platform.system()} for MJPEG streaming...")
    if init_camera(CAMERA_DEFAULT_INDEX, CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_FPS, CAMERA_DEFAULT_QUALITY):
        print("Camera initialized successfully for MJPEG streaming")
    else:
        print("Warning: Camera initialization failed, camera features will be unavailable")

    if init_serial(SERIAL_PORT):
        # Start serial data background thread
        start_serial_data_thread()
        print("Flask server starting on http://0.0.0.0:5000")
        print("Open this address in your web browser.")
        print("MJPEG video streaming available at: http://0.0.0.0:5000/video_feed")
        try:
            app.run(host='0.0.0.0', port=5000, debug=False)
        except KeyboardInterrupt:
            print("\nFlask server stopping.")
    else:
        print(f"Could not initialize serial port {SERIAL_PORT}. Exiting.")
        return jsonify({'success': True, 'message': 'UART proxy initialized'})
    else:
        return jsonify({'success': False, 'message': 'UART proxy already running'})

@app.route('/proxy/toggle_forwarding', methods=['POST'])
def toggle_forwarding():
    """Toggle forwarding for a virtual port."""
    global uart_proxy
    if uart_proxy is None:
        return jsonify({'success': False, 'message': 'UART proxy not initialized'})

    data = request.get_json()
    virtual_port_index = data.get('virtual_port_index')
    enable = data.get('enable', True)

    if virtual_port_index is None:
        return jsonify({'success': False, 'message': 'Missing virtual_port_index'})

    uart_proxy.toggle_forwarding(virtual_port_index, enable)
    return jsonify({'success': True, 'message': f'Forwarding {"enabled" if enable else "disabled"} for virtual port {virtual_port_index}'})

if __name__ == '__main__':
    print(f"Starting application on {platform.system()}")
    
    # Load serial configuration from file
    print("Loading serial configuration...")
    if not load_serial_config():
        default_port = get_default_serial_port()
        port_input = input(f"Enter serial port (default for {platform.system()}: {default_port}, press Enter to use default): ")
        if port_input.strip():
            SERIAL_PORT = port_input.strip()
        else:
            SERIAL_PORT = default_port
        save_serial_config()
    
    # Initialize camera for MJPEG streaming
    print(f"Initializing camera on {platform.system()} for MJPEG streaming...")
    if init_camera(CAMERA_DEFAULT_INDEX, CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_FPS, CAMERA_DEFAULT_QUALITY):
        print("Camera initialized successfully for MJPEG streaming")
    else:
        print("Warning: Camera initialization failed, camera features will be unavailable")

    if init_serial(SERIAL_PORT):
        # Start serial data background thread
        start_serial_data_thread()
        print("Flask server starting on http://0.0.0.0:5000")
        print("Open this address in your web browser.")
        print("MJPEG video streaming available at: http://0.0.0.0:5000/video_feed")
        try:
            app.run(host='0.0.0.0', port=5000, debug=False)
        except KeyboardInterrupt:
            print("\nFlask server stopping.")
    else:
        print(f"Could not initialize serial port {SERIAL_PORT}. Exiting.")
