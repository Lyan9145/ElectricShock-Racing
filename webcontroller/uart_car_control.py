import serial
import struct
import time
from flask import Flask, render_template, request, jsonify, Response, send_file
import atexit
import cv2
import threading
import queue
import base64
import json
from datetime import datetime
import os
import platform
import subprocess
import shutil
from pathlib import Path

# USB通信帧
USB_FRAME_HEAD = 0x42
USB_ADDR_CARCTRL = 1

# --- Configuration ---
CONFIG_FILE = 'serial_config.json'
# Set OS-specific default serial port
if platform.system() == "Windows":
    SERIAL_PORT = 'COM3'
else:
    SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Camera Configuration
CAMERA_DEFAULT_INDEX = 0
CAMERA_DEFAULT_WIDTH = 640
CAMERA_DEFAULT_HEIGHT = 480
CAMERA_DEFAULT_FPS = 30
CAMERA_DEFAULT_BITRATE = 1000000  # 1Mbps for RTMP

# RTMP Configuration
RTMP_PORT = 1935
RTMP_STREAM_KEY = "live"
RTMP_SERVER_URL = f"rtmp://localhost:{RTMP_PORT}/live/{RTMP_STREAM_KEY}"
RTMP_PLAYBACK_URL = f"rtmp://localhost:{RTMP_PORT}/live/{RTMP_STREAM_KEY}"

# Default values
SPEED_DEFAULT = 0.0
SERVO_MID_DEFAULT = 4000
CONTINUOUS_SEND_INTERVAL_DEFAULT = 200 # milliseconds

# Servo range configuration
# Actual device hardware limits (for final clamping before sending)
DEVICE_SERVO_MIN_HW = 0
DEVICE_SERVO_MAX_HW = 65536
# UI Slider default configuration range
SERVO_SLIDER_MIN_UI_DEFAULT = 3000
SERVO_SLIDER_MAX_UI_DEFAULT = 6000
# Absolute uint16 limits for UI configuration inputs
UINT16_MIN = 0
UINT16_MAX = 65535

# Global serial object
ser = None

# Camera and RTMP globals
camera = None
rtmp_server_process = None
rtmp_stream_process = None
rtmp_thread = None
rtmp_running = False
rtmp_lock = threading.Lock()

# Camera settings
current_camera_index = CAMERA_DEFAULT_INDEX
current_camera_width = CAMERA_DEFAULT_WIDTH
current_camera_height = CAMERA_DEFAULT_HEIGHT
current_camera_fps = CAMERA_DEFAULT_FPS
current_camera_bitrate = CAMERA_DEFAULT_BITRATE

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
def check_ffmpeg():
    """Check if ffmpeg is available"""
    try:
        subprocess.run(['ffmpeg', '-version'], 
                      stdout=subprocess.DEVNULL, 
                      stderr=subprocess.DEVNULL, 
                      check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def start_rtmp_server():
    """Start FFmpeg RTMP server"""
    global rtmp_server_process
    
    try:
        # Kill any existing RTMP server process
        stop_rtmp_server()
        
        # FFmpeg RTMP server command
        rtmp_server_cmd = [
            'ffmpeg',
            '-listen', '1',
            '-f', 'flv',
            '-i', f'rtmp://localhost:{RTMP_PORT}/live/{RTMP_STREAM_KEY}',
            '-c', 'copy',
            '-f', 'flv',
            f'rtmp://localhost:{RTMP_PORT}/live/{RTMP_STREAM_KEY}_output'
        ]
        
        # Start RTMP server process
        rtmp_server_process = subprocess.Popen(
            rtmp_server_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        print(f"RTMP server started on port {RTMP_PORT}")
        time.sleep(1)  # Give server time to start
        return True
        
    except Exception as e:
        print(f"Error starting RTMP server: {e}")
        return False

def stop_rtmp_server():
    """Stop RTMP server"""
    global rtmp_server_process
    
    if rtmp_server_process is not None:
        try:
            rtmp_server_process.terminate()
            rtmp_server_process.wait(timeout=5)
        except:
            try:
                rtmp_server_process.kill()
            except:
                pass
        rtmp_server_process = None
        print("RTMP server stopped")

def init_camera(camera_index=0, width=640, height=480, fps=30, bitrate=1000000):
    """Initialize camera for RTMP streaming with H.264"""
    global camera, rtmp_stream_process, current_camera_index, current_camera_width
    global current_camera_height, current_camera_fps, current_camera_bitrate
    
    try:
        # Check if ffmpeg is available
        if not check_ffmpeg():
            print("Error: ffmpeg not found. Please install ffmpeg for H.264/RTMP streaming")
            return False
        
        # Stop existing processes
        stop_camera()
        
        # Set platform-specific camera backend
        backend = cv2.CAP_ANY
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
        
        # Set camera properties for optimal performance
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer for low latency
        
        # Try to set hardware acceleration if available
        if platform.system() == "Windows":
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
        current_camera_bitrate = bitrate
        
        print(f"Camera {camera_index} initialized: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS for H.264/RTMP")
        return True
        
    except Exception as e:
        print(f"Error initializing camera: {e}")
        camera = None
        return False

def start_rtmp_ffmpeg():
    """Start ffmpeg process for RTMP streaming"""
    global rtmp_stream_process, current_camera_width, current_camera_height, current_camera_fps, current_camera_bitrate
    
    try:
        # Stop any existing process first
        if rtmp_stream_process is not None:
            try:
                rtmp_stream_process.stdin.close()
                rtmp_stream_process.terminate()
                rtmp_stream_process.wait(timeout=3)
            except:
                try:
                    rtmp_stream_process.kill()
                except:
                    pass
            rtmp_stream_process = None
        
        # High-performance RTMP streaming command
        rtmp_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{current_camera_width}x{current_camera_height}',
            '-r', str(current_camera_fps),
            '-i', '-',  # Input from stdin
            '-c:v', 'libx264',
            '-preset', 'ultrafast',  # Fastest encoding
            '-tune', 'zerolatency',  # Zero latency tuning
            '-b:v', str(current_camera_bitrate),
            '-maxrate', str(current_camera_bitrate),
            '-bufsize', str(current_camera_bitrate // 2),  # Smaller buffer for lower latency
            '-g', '15',  # Small GOP size for lower latency
            '-keyint_min', '15',
            '-sc_threshold', '0',
            '-profile:v', 'baseline',  # Baseline profile for better compatibility
            '-level', '3.0',
            '-pix_fmt', 'yuv420p',
            '-flags', '+global_header',
            '-bsf:v', 'dump_extra',
            '-f', 'flv',
            '-reconnect', '1',
            '-reconnect_delay_max', '2',
            RTMP_SERVER_URL
        ]
        
        # Start ffmpeg RTMP streaming process
        rtmp_stream_process = subprocess.Popen(
            rtmp_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0  # Unbuffered for real-time streaming
        )
        
        # Verify process started correctly
        time.sleep(0.5)
        if rtmp_stream_process.poll() is not None:
            print("FFmpeg process failed to start or exited immediately")
            return False
        
        print("FFmpeg RTMP streaming process started")
        return True
        
    except Exception as e:
        print(f"Error starting FFmpeg RTMP process: {e}")
        return False

def rtmp_thread_func():
    """RTMP capture and streaming thread"""
    global camera, rtmp_running, rtmp_stream_process
    
    frame_count = 0
    consecutive_errors = 0
    max_consecutive_errors = 10
    
    while rtmp_running and camera is not None:
        try:
            # Check if FFmpeg process is still alive
            if rtmp_stream_process is None or rtmp_stream_process.poll() is not None:
                print("FFmpeg process not running, attempting restart...")
                if not start_rtmp_ffmpeg():
                    print("Failed to restart FFmpeg process")
                    time.sleep(1)
                    continue
            
            # Check if stdin is available
            if rtmp_stream_process.stdin is None:
                print("FFmpeg stdin not available")
                time.sleep(0.1)
                continue
            
            ret, frame = camera.read()
            if ret:
                try:
                    # Write frame data to FFmpeg stdin
                    rtmp_stream_process.stdin.write(frame.tobytes())
                    rtmp_stream_process.stdin.flush()
                    frame_count += 1
                    consecutive_errors = 0  # Reset error counter on success
                    
                    # Minimal delay for performance
                    time.sleep(0.001)
                    
                except (BrokenPipeError, OSError) as e:
                    print(f"Pipe error: {e}")
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors:
                        print("Too many consecutive pipe errors, restarting FFmpeg...")
                        if not start_rtmp_ffmpeg():
                            print("Failed to restart FFmpeg after pipe errors")
                            time.sleep(1)
                        consecutive_errors = 0
                    else:
                        time.sleep(0.1)
                
            else:
                print("Failed to read frame from camera")
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    print("Too many consecutive camera read errors")
                    break
                time.sleep(0.01)
                
        except Exception as e:
            print(f"RTMP thread error: {e}")
            consecutive_errors += 1
            if consecutive_errors >= max_consecutive_errors:
                print("Too many consecutive errors in RTMP thread")
                break
            time.sleep(0.1)
    
    print("RTMP thread exiting")

def start_camera():
    """Start RTMP camera streaming"""
    global rtmp_thread, rtmp_running
    
    if camera is None:
        return False
    
    if rtmp_running:
        return True
    
    # Start RTMP server first
    if not start_rtmp_server():
        print("Failed to start RTMP server")
        return False
    
    # Wait a moment for server to be ready
    time.sleep(2)
    
    # Start FFmpeg RTMP streaming process
    if not start_rtmp_ffmpeg():
        stop_rtmp_server()
        return False
    
    rtmp_running = True
    rtmp_thread = threading.Thread(target=rtmp_thread_func, daemon=True)
    rtmp_thread.start()
    
    print("RTMP streaming started")
    return True

def stop_camera():
    """Stop RTMP camera streaming"""
    global camera, rtmp_thread, rtmp_running, rtmp_stream_process
    
    print("Stopping RTMP streaming...")
    rtmp_running = False
    
    # Stop RTMP thread
    if rtmp_thread is not None:
        rtmp_thread.join(timeout=5)
        if rtmp_thread.is_alive():
            print("Warning: RTMP thread did not stop gracefully")
        rtmp_thread = None
    
    # Stop FFmpeg streaming process
    if rtmp_stream_process is not None:
        try:
            if rtmp_stream_process.stdin:
                rtmp_stream_process.stdin.close()
        except:
            pass
        
        try:
            rtmp_stream_process.terminate()
            rtmp_stream_process.wait(timeout=3)
        except:
            try:
                rtmp_stream_process.kill()
                rtmp_stream_process.wait(timeout=2)
            except:
                pass
        rtmp_stream_process = None
    
    # Stop RTMP server
    stop_rtmp_server()
    
    # Release camera
    if camera is not None:
        try:
            camera.release()
        except:
            pass
        camera = None
    
    print("RTMP streaming stopped")

def update_camera_settings(camera_index=None, width=None, height=None, fps=None, bitrate=None):
    """Update camera settings for RTMP"""
    global current_camera_index, current_camera_width, current_camera_height
    global current_camera_fps, current_camera_bitrate
    
    with rtmp_lock:
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
        if bitrate is not None and bitrate != current_camera_bitrate:
            restart_needed = True
        
        if restart_needed:
            was_running = rtmp_running
            if was_running:
                stop_camera()
            
            # Update settings
            new_index = camera_index if camera_index is not None else current_camera_index
            new_width = width if width is not None else current_camera_width
            new_height = height if height is not None else current_camera_height
            new_fps = fps if fps is not None else current_camera_fps
            new_bitrate = bitrate if bitrate is not None else current_camera_bitrate
            
            success = init_camera(new_index, new_width, new_height, new_fps, new_bitrate)
            
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

def load_serial_config():
    """Load serial configuration from JSON file with OS-specific defaults"""
    global SERIAL_PORT, BAUD_RATE
    
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
                SERIAL_PORT = config.get('serial_port', SERIAL_PORT)
                BAUD_RATE = config.get('baud_rate', BAUD_RATE)
                print(f"Loaded serial config: Port={SERIAL_PORT}, Baud={BAUD_RATE}")
                return True
    except Exception as e:
        print(f"Error loading serial config: {e}")
    
    # Set OS-specific default if no config found
    if SERIAL_PORT == '/dev/ttyUSB0' and platform.system() == "Windows":
        SERIAL_PORT = get_default_serial_port()
    
    return False

def save_serial_config():
    """Save serial configuration to JSON file"""
    try:
        config = {
            'serial_port': SERIAL_PORT,
            'baud_rate': BAUD_RATE,
            'last_updated': datetime.now().isoformat()
        }
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Saved serial config: Port={SERIAL_PORT}, Baud={BAUD_RATE}")
        return True
    except Exception as e:
        print(f"Error saving serial config: {e}")
        return False

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

# --- Flask RTMP Routes ---
@app.route('/rtmp/stream')
def rtmp_stream_url():
    """Get RTMP stream URL"""
    return jsonify({
        'rtmp_url': RTMP_PLAYBACK_URL,
        'port': RTMP_PORT,
        'stream_key': RTMP_STREAM_KEY
    })

@app.route('/video_feed')
def video_feed():
    """Redirect to RTMP stream for compatibility"""
    return Response(
        f"RTMP streaming active. Use {RTMP_PLAYBACK_URL} for video stream",
        mimetype='text/plain'
    )

@app.route('/camera/start', methods=['POST'])
def start_camera_route():
    """Start RTMP camera streaming"""
    if camera is None:
        return jsonify({'success': False, 'message': 'Camera not initialized'})
    
    success = start_camera()
    if success:
        return jsonify({
            'success': True, 
            'message': 'RTMP streaming started',
            'rtmp_url': RTMP_PLAYBACK_URL
        })
    else:
        return jsonify({'success': False, 'message': 'Failed to start RTMP streaming'})

@app.route('/camera/stop', methods=['POST'])
def stop_camera_route():
    """Stop RTMP camera streaming"""
    stop_camera()
    return jsonify({'success': True, 'message': 'RTMP streaming stopped'})

@app.route('/camera/settings', methods=['GET', 'POST'])
def camera_settings_route():
    """Get or update camera settings for RTMP"""
    global current_camera_index, current_camera_width, current_camera_height
    global current_camera_fps, current_camera_bitrate, rtmp_running
    
    if request.method == 'GET':
        return jsonify({
            'camera_index': current_camera_index,
            'width': current_camera_width,
            'height': current_camera_height,
            'fps': current_camera_fps,
            'bitrate': current_camera_bitrate,
            'running': rtmp_running,
            'streaming_type': 'RTMP/H.264',
            'rtmp_url': RTMP_PLAYBACK_URL
        })
    
    elif request.method == 'POST':
        data = request.get_json()
        
        camera_index = data.get('camera_index')
        width = data.get('width')
        height = data.get('height')
        fps = data.get('fps')
        bitrate = data.get('bitrate')
        
        success = update_camera_settings(camera_index, width, height, fps, bitrate)
        
        if success:
            return jsonify({
                'success': True,
                'message': 'RTMP camera settings updated',
                'settings': {
                    'camera_index': current_camera_index,
                    'width': current_camera_width,
                    'height': current_camera_height,
                    'fps': current_camera_fps,
                    'bitrate': current_camera_bitrate,
                    'running': rtmp_running,
                    'streaming_type': 'RTMP/H.264',
                    'rtmp_url': RTMP_PLAYBACK_URL
                }
            })
        else:
            return jsonify({'success': False, 'message': 'Failed to update RTMP camera settings'})

@app.route('/camera/init', methods=['POST'])
def init_camera_route():
    """Initialize camera for RTMP streaming"""
    data = request.get_json()
    
    camera_index = data.get('camera_index', CAMERA_DEFAULT_INDEX)
    width = data.get('width', CAMERA_DEFAULT_WIDTH)
    height = data.get('height', CAMERA_DEFAULT_HEIGHT)
    fps = data.get('fps', CAMERA_DEFAULT_FPS)
    bitrate = data.get('bitrate', CAMERA_DEFAULT_BITRATE)
    
    success = init_camera(camera_index, width, height, fps, bitrate)
    
    if success:
        return jsonify({
            'success': True, 
            'message': 'Camera initialized for RTMP streaming',
            'streaming_type': 'RTMP/H.264',
            'rtmp_url': RTMP_PLAYBACK_URL
        })
    else:
        return jsonify({'success': False, 'message': 'Failed to initialize camera for RTMP'})

@app.route('/serial/settings', methods=['GET', 'POST'])
def serial_settings_route():
    """Get or update serial port settings"""
    global SERIAL_PORT, BAUD_RATE, ser
    
    if request.method == 'GET':
        return jsonify({
            'serial_port': SERIAL_PORT,
            'baud_rate': BAUD_RATE,
            'connected': ser is not None and ser.is_open if ser else False
        })
    
    elif request.method == 'POST':
        data = request.get_json()
        
        new_port = data.get('serial_port')
        new_baud = data.get('baud_rate')
        
        # Validate inputs
        if new_port is None or new_baud is None:
            return jsonify({'success': False, 'message': 'Missing serial_port or baud_rate'})
        
        try:
            new_baud = int(new_baud)
        except ValueError:
            return jsonify({'success': False, 'message': 'Invalid baud rate'})
        
        # Close current connection if open
        close_serial_port()
        
        # Update settings
        SERIAL_PORT = new_port
        BAUD_RATE = new_baud
        
        # Save to config file
        if save_serial_config():
            # Try to reconnect with new settings
            if init_serial(SERIAL_PORT):
                return jsonify({
                    'success': True,
                    'message': 'Serial settings updated and connected',
                    'settings': {
                        'serial_port': SERIAL_PORT,
                        'baud_rate': BAUD_RATE,
                        'connected': True
                    }
                })
            else:
                return jsonify({
                    'success': True,
                    'message': 'Serial settings updated but connection failed',
                    'settings': {
                        'serial_port': SERIAL_PORT,
                        'baud_rate': BAUD_RATE,
                        'connected': False
                    }
                })
        else:
            return jsonify({'success': False, 'message': 'Failed to save serial settings'})

@app.route('/serial/connect', methods=['POST'])
def serial_connect_route():
    """Connect to serial port with current settings"""
    if init_serial(SERIAL_PORT):
        return jsonify({'success': True, 'message': 'Serial port connected'})
    else:
        return jsonify({'success': False, 'message': 'Failed to connect to serial port'})

@app.route('/serial/disconnect', methods=['POST'])
def serial_disconnect_route():
    """Disconnect from serial port"""
    close_serial_port()
    return jsonify({'success': True, 'message': 'Serial port disconnected'})

# Add RTMP status endpoint
@app.route('/rtmp/status')
def rtmp_status():
    """Get RTMP streaming status"""
    global rtmp_running, rtmp_stream_process, rtmp_server_process, camera
    
    status = {
        'running': rtmp_running,
        'camera_available': camera is not None,
        'rtmp_server_active': rtmp_server_process is not None and rtmp_server_process.poll() is None,
        'rtmp_stream_active': rtmp_stream_process is not None and rtmp_stream_process.poll() is None,
        'rtmp_url': RTMP_PLAYBACK_URL if rtmp_running else None,
        'port': RTMP_PORT,
        'stream_key': RTMP_STREAM_KEY,
        'camera_settings': {
            'index': current_camera_index,
            'width': current_camera_width,
            'height': current_camera_height,
            'fps': current_camera_fps,
            'bitrate': current_camera_bitrate
        }
    }
    
    return jsonify(status)

if __name__ == '__main__':
    print(f"Starting application on {platform.system()}")
    
    # Check for ffmpeg availability
    if not check_ffmpeg():
        print("Warning: ffmpeg not found. Please install ffmpeg for H.264/RTMP streaming")
        print("On Windows: Download from https://ffmpeg.org/download.html")
        print("On Linux: sudo apt install ffmpeg (Ubuntu/Debian) or equivalent")
        print("On macOS: brew install ffmpeg")
    
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
    
    # Initialize camera for RTMP streaming
    print(f"Initializing camera on {platform.system()} for RTMP/H.264 streaming...")
    if init_camera(CAMERA_DEFAULT_INDEX, CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_FPS, CAMERA_DEFAULT_BITRATE):
        print(f"Camera initialized successfully for RTMP streaming")
        
        # Auto-start RTMP streaming
        if start_camera():
            print("RTMP streaming started automatically")
            print(f"RTMP stream URL: {RTMP_PLAYBACK_URL}")
        else:
            print("Warning: Failed to start RTMP streaming automatically")
    else:
        print("Warning: Camera initialization failed, camera features will be unavailable")

    if init_serial(SERIAL_PORT):
        print(f"Flask server starting on http://0.0.0.0:5000")
        print("Open this address in your web browser.")
        print(f"RTMP video streaming available at: {RTMP_PLAYBACK_URL}")
        try:
            app.run(host='0.0.0.0', port=5000, debug=False)
        except KeyboardInterrupt:
            print("\nFlask server stopping.")
    else:
        print(f"Could not initialize serial port {SERIAL_PORT}. Exiting.")
