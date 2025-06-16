import serial
import struct
import time
from flask import Flask, render_template_string, request, jsonify, Response
import atexit
import cv2
import threading
import queue
import base64
import json
from datetime import datetime

# USB通信帧
USB_FRAME_HEAD = 0x42
USB_ADDR_CARCTRL = 1

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Camera Configuration
CAMERA_DEFAULT_INDEX = 0
CAMERA_DEFAULT_WIDTH = 640
CAMERA_DEFAULT_HEIGHT = 480
CAMERA_DEFAULT_FPS = 30
CAMERA_DEFAULT_QUALITY = 80  # JPEG quality (1-100)

# Default values
SPEED_DEFAULT = 0.0
SERVO_MID_DEFAULT = 1500
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

# Camera globals
camera = None
camera_thread = None
camera_frame_queue = queue.Queue(maxsize=2)
camera_running = False
camera_lock = threading.Lock()

# Camera settings
current_camera_index = CAMERA_DEFAULT_INDEX
current_camera_width = CAMERA_DEFAULT_WIDTH
current_camera_height = CAMERA_DEFAULT_HEIGHT
current_camera_fps = CAMERA_DEFAULT_FPS
current_camera_quality = CAMERA_DEFAULT_QUALITY

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
        send_car_control_command(SPEED_DEFAULT, SERVO_MID_DEFAULT)
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

# --- Flask Web Interface ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>UART Car Control</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; -webkit-tap-highlight-color: transparent; }
        .container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); max-width: 600px; margin: auto; }
        h1 { text-align: center; color: #333; }
        .control-group { margin-bottom: 20px; }
        .control-group-inline { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px;}
        .control-group-inline label { flex-basis: 40%; white-space: nowrap; margin-right: 5px;}
        .control-group-inline input[type="number"], .control-group-inline input[type="checkbox"] { flex-basis: 55%; }
        .control-group-inline input[type="checkbox"] { transform: scale(1.2); margin-left: 5px;}
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input[type="range"] { width: 100%; margin-top: 5px; margin-bottom: 5px; box-sizing: border-box;}
        input[type="number"] { width: 100px; padding: 8px; box-sizing: border-box; }
        .slider-container { display: flex; align-items: center; }
        .slider-container input[type="range"] { flex-grow: 1; margin-right: 10px; }
        .slider-container input[type="number"] { width: 80px; }
        button { padding: 10px 15px; background-color: #007bff; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }
        button:hover { background-color: #0056b3; }
        #continuousSendToggle { background-color: #28a745; }
        #continuousSendToggle.active { background-color: #dc3545; }
        .status { margin-top:20px; padding:10px; background-color:#e0e0e0; border-radius:4px; text-align:center; word-wrap: break-word; min-height: 20px;}
        .info-text { font-size: 0.9em; color: #555; margin-top: -8px; margin-bottom: 10px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>UART Car Control</h1>
        
        <div class="control-group">
            <label for="speedSlider">Speed (-1.0 to 1.0 m/s): <span id="speedValueDisplay">{{ "%.2f"|format(current_speed) }}</span></label>
            <div class="slider-container">
                <input type="range" id="speedSlider" min="-1.0" max="1.0" step="0.05" value="{{ current_speed }}">
                <input type="number" id="speedNumber" min="-1.0" max="1.0" step="0.05" value="{{ current_speed }}">
            </div>
        </div>
        
        <fieldset style="margin-bottom: 20px; border: 1px solid #ccc; padding: 10px;">
            <legend>Servo Slider Range Configuration</legend>
            <p class="info-text">Define the min/max for the servo slider below. Values are uint16 ({{ uint16_min }}-{{ uint16_max }}). Final output to device is clamped to {{ device_servo_min_hw }}-{{ device_servo_max_hw }}.</p>
            <div class="control-group-inline">
                <label for="servoMinPwmInput">Slider Min PWM:</label>
                <input type="number" id="servoMinPwmInput" value="{{ current_servo_min_slider }}" min="{{ uint16_min }}" max="{{ uint16_max }}" step="10">
            </div>
            <div class="control-group-inline">
                <label for="servoMaxPwmInput">Slider Max PWM:</label>
                <input type="number" id="servoMaxPwmInput" value="{{ current_servo_max_slider }}" min="{{ uint16_min }}" max="{{ uint16_max }}" step="10">
            </div>
        </fieldset>
        
        <div class="control-group">
            <label for="servoSlider">Servo Control (<span id="servoMinDisplay">{{ current_servo_min_slider }}</span> to <span id="servoMaxDisplay">{{ current_servo_max_slider }}</span> PWM): <span id="servoValueDisplay">{{ current_servo }}</span></label>
            <div class="slider-container">
                <input type="range" id="servoSlider" min="{{ current_servo_min_slider }}" max="{{ current_servo_max_slider }}" step="10" value="{{ current_servo }}">
                <input type="number" id="servoNumber" min="{{ current_servo_min_slider }}" max="{{ current_servo_max_slider }}" step="10" value="{{ current_servo }}">
            </div>
        </div>        <fieldset style="margin-bottom: 20px; border: 1px solid #ccc; padding: 10px;">
            <legend>Continuous Send</legend>
            <div class="control-group-inline">
                <label for="continuousSendIntervalInput">Interval (ms):</label>
                <input type="number" id="continuousSendIntervalInput" value="{{ current_continuous_interval }}" min="50" max="5000" step="10">
                <button id="continuousSendToggle" style="margin-left:10px;">Start Continuous</button>
            </div>
        </fieldset>

        <fieldset style="margin-bottom: 20px; border: 1px solid #ccc; padding: 10px;">
            <legend>Camera Control</legend>
            <div style="display: flex; justify-content: space-between; margin-bottom: 10px;">
                <button id="cameraInitButton" style="flex: 1; margin-right: 5px;">Initialize Camera</button>
                <button id="cameraStartButton" style="flex: 1; margin: 0 2px;">Start</button>
                <button id="cameraStopButton" style="flex: 1; margin-left: 5px;">Stop</button>
            </div>
            <div class="control-group-inline">
                <label for="cameraIndexInput">Camera Index:</label>
                <input type="number" id="cameraIndexInput" value="0" min="0" max="10" step="1">
            </div>
            <div class="control-group-inline">
                <label for="cameraWidthInput">Width:</label>
                <input type="number" id="cameraWidthInput" value="640" min="320" max="1920" step="10">
            </div>
            <div class="control-group-inline">
                <label for="cameraHeightInput">Height:</label>
                <input type="number" id="cameraHeightInput" value="480" min="240" max="1080" step="10">
            </div>
            <div class="control-group-inline">
                <label for="cameraFpsInput">FPS:</label>
                <input type="number" id="cameraFpsInput" value="30" min="5" max="60" step="5">
            </div>
            <div class="control-group-inline">
                <label for="cameraQualityInput">JPEG Quality:</label>
                <input type="number" id="cameraQualityInput" value="80" min="10" max="100" step="5">
            </div>
            <button id="cameraUpdateButton" style="width: 100%; margin-top: 10px;">Update Settings</button>
        </fieldset>

        <div id="cameraContainer" style="margin-bottom: 20px; text-align: center; display: none;">
            <h3>Live Video Feed</h3>
            <img id="cameraFeed" src="/video_feed" style="max-width: 100%; height: auto; border: 1px solid #ccc;">
        </div>

        <button id="resetButton" style="width:100%; margin-bottom:10px;">Reset to Center/Stop (Speed 0, Servo {{ servo_mid_default }})</button>
        <div id="status" class="status">Connect to serial and use controls.</div>
    </div>

    <script>
        const speedSlider = document.getElementById('speedSlider');
        const speedNumber = document.getElementById('speedNumber');
        const speedValueDisplay = document.getElementById('speedValueDisplay');
        
        const servoSlider = document.getElementById('servoSlider');
        const servoNumber = document.getElementById('servoNumber');
        const servoValueDisplay = document.getElementById('servoValueDisplay');

        const servoMinPwmInput = document.getElementById('servoMinPwmInput');
        const servoMaxPwmInput = document.getElementById('servoMaxPwmInput');
        const servoMinDisplay = document.getElementById('servoMinDisplay');
        const servoMaxDisplay = document.getElementById('servoMaxDisplay');
        
        const resetButton = document.getElementById('resetButton');
        const statusDiv = document.getElementById('status');

        const continuousSendIntervalInput = document.getElementById('continuousSendIntervalInput');
        const continuousSendToggle = document.getElementById('continuousSendToggle');

        const UINT16_MIN = {{ uint16_min }};
        const UINT16_MAX = {{ uint16_max }};
        const SERVO_MID_DEFAULT = {{ servo_mid_default }};
        // Device HW limits are mostly for information in UI, final clamping is server-side
        // const DEVICE_SERVO_MIN_HW = {{ device_servo_min_hw }}; 
        // const DEVICE_SERVO_MAX_HW = {{ device_servo_max_hw }};

        let currentServoMinSlider = parseInt(servoMinPwmInput.value); // Range for the main servo slider
        let currentServoMaxSlider = parseInt(servoMaxPwmInput.value); // Range for the main servo slider

        let manualSendLastTime = 0;
        const manualSendInterval = 100; 

        let continuousSendTimerId = null;
        let currentContinuousInterval = parseInt(continuousSendIntervalInput.value);

        function updateSpeedControls(value) {
            const val = parseFloat(value).toFixed(2);
            speedSlider.value = val;
            speedNumber.value = val;
            speedValueDisplay.textContent = val;
        }

        function updateServoControls(value) {
            const val = parseInt(value);
            // Clamp to the current slider's configured range
            const clampedVal = Math.max(currentServoMinSlider, Math.min(val, currentServoMaxSlider));
            servoSlider.value = clampedVal; 
            servoNumber.value = clampedVal;
            servoValueDisplay.textContent = clampedVal;
            return clampedVal; // Return the possibly clamped value
        }
        
        function handleSpeedChange() {
            updateSpeedControls(this.value);
            sendControlCommand(false, true); 
        }

        function handleServoChange() {
            updateServoControls(this.value); // updateServoControls now clamps and updates display
            sendControlCommand(false, true); 
        }
        
        speedSlider.addEventListener('input', handleSpeedChange);
        speedNumber.addEventListener('change', handleSpeedChange);
        
        servoSlider.addEventListener('input', handleServoChange);
        servoNumber.addEventListener('change', handleServoChange);

        function applyServoRangeChanges() {
            let newMin = parseInt(servoMinPwmInput.value);
            let newMax = parseInt(servoMaxPwmInput.value);

            // Validate and clamp newMin and newMax against uint16 and ensure min < max
            newMin = Math.max(UINT16_MIN, Math.min(newMin, UINT16_MAX - 1)); // Ensure min < max possible
            newMax = Math.max(UINT16_MIN + 1, Math.min(newMax, UINT16_MAX)); // Ensure max > min possible
            
            if (newMin >= newMax) {
                // If still invalid (e.g., user typed bad values), revert or set to a small valid range
                statusDiv.textContent = "Error: Slider Min PWM must be less than Slider Max PWM.";
                servoMinPwmInput.value = currentServoMinSlider; // Revert to previous valid
                servoMaxPwmInput.value = currentServoMaxSlider; // Revert to previous valid
                return; // Don't apply invalid range
            }

            currentServoMinSlider = newMin;
            currentServoMaxSlider = newMax;

            servoSlider.min = currentServoMinSlider;
            servoSlider.max = currentServoMaxSlider;
            // Step might need adjustment if range is huge, but 10 is usually fine
            // servoSlider.step = Math.max(1, Math.round((currentServoMaxSlider - currentServoMinSlider) / 100));


            servoNumber.min = currentServoMinSlider;
            servoNumber.max = currentServoMaxSlider;

            servoMinDisplay.textContent = currentServoMinSlider;
            servoMaxDisplay.textContent = currentServoMaxSlider;

            // Re-clamp current servo value to be within the new slider range
            let currentServoValue = parseInt(servoSlider.value);
            currentServoValue = updateServoControls(currentServoValue); // update and get clamped
            // No automatic send on range change, user will adjust slider after if needed
        }

        servoMinPwmInput.addEventListener('change', applyServoRangeChanges);
        servoMaxPwmInput.addEventListener('change', applyServoRangeChanges);

        resetButton.onclick = () => {
            if (continuousSendTimerId) toggleContinuousSend(); 
            updateSpeedControls(0.0);
            let resetServoValue = SERVO_MID_DEFAULT;
            // Clamp reset value to the current slider's configured range
            resetServoValue = Math.max(currentServoMinSlider, Math.min(resetServoValue, currentServoMaxSlider));
            updateServoControls(resetServoValue);
            sendControlCommand(true, false); 
        };

        continuousSendIntervalInput.addEventListener('change', () => {
            let newInterval = parseInt(continuousSendIntervalInput.value);
            if (newInterval < 50) newInterval = 50; 
            if (newInterval > 5000) newInterval = 5000;
            continuousSendIntervalInput.value = newInterval;
            currentContinuousInterval = newInterval;
            if (continuousSendTimerId) { 
                toggleContinuousSend(); 
                toggleContinuousSend(); 
            }
        });

        function toggleContinuousSend() {
            if (continuousSendTimerId) { 
                clearInterval(continuousSendTimerId);
                continuousSendTimerId = null;
                continuousSendToggle.textContent = 'Start Continuous';
                continuousSendToggle.classList.remove('active');
                statusDiv.textContent = 'Continuous send stopped.';
            } else { 
                continuousSendTimerId = setInterval(() => {
                    sendControlCommand(false, false); 
                }, currentContinuousInterval);
                continuousSendToggle.textContent = 'Stop Continuous';
                continuousSendToggle.classList.add('active');
                statusDiv.textContent = `Continuous send started (Interval: ${currentContinuousInterval}ms).`;
            }
        }
        continuousSendToggle.addEventListener('click', toggleContinuousSend);


        async function sendControlCommand(forceSend = false, isManualChange = false) {
            const currentTime = Date.now();
            if (isManualChange && !forceSend && (currentTime - manualSendLastTime < manualSendInterval)) {
                return; 
            }
            if (isManualChange) {
                manualSendLastTime = currentTime;
            }

            const speed = parseFloat(speedSlider.value);
            const servo = parseInt(servoSlider.value); // Value is already clamped by UI to currentServoMinSlider/MaxSlider
            
            let actionMsg = "Sending";
            if (continuousSendTimerId && !isManualChange && !forceSend) actionMsg = "Auto-Sending";

            // Only update status text for manual/forced to reduce noise
            if (isManualChange || forceSend || !continuousSendTimerId) {
                statusDiv.textContent = `${actionMsg}: Speed=${speed.toFixed(2)}, Servo=${servo}`;
            }
            
            try {
                const response = await fetch('/control', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ speed: speed, servo: servo }),
                });
                const result = await response.json();
                if (result.success) {
                    if (isManualChange || forceSend) {
                        statusDiv.textContent = `Sent: Speed=${speed.toFixed(2)}, Servo=${servo}. Response: ${result.message}`;
                    }
                } else {
                    statusDiv.textContent = `Error: ${result.message}`;
                    if (continuousSendTimerId) toggleContinuousSend(); 
                }
            } catch (error) {
                statusDiv.textContent = `Fetch error: ${error}`;
                console.error('Error sending command:', error);
                if (continuousSendTimerId) toggleContinuousSend(); 
            }
        }        document.addEventListener('DOMContentLoaded', () => {
            updateSpeedControls(speedSlider.value); // Init speed display
            // Initialize JS servo slider range vars from HTML (which got them from Python)
            currentServoMinSlider = parseInt(servoMinPwmInput.value); 
            currentServoMaxSlider = parseInt(servoMaxPwmInput.value);
            
            // Ensure initial main servo value is within the initial slider range
            let initialServo = parseInt(servoSlider.value);
            initialServo = updateServoControls(initialServo); // This also updates display

            // Set initial label text for servo slider range
            servoMinDisplay.textContent = currentServoMinSlider;
            servoMaxDisplay.textContent = currentServoMaxSlider;

            // Camera controls initialization
            initCameraControls();
        });

        // Camera control functions
        function initCameraControls() {
            const cameraInitButton = document.getElementById('cameraInitButton');
            const cameraStartButton = document.getElementById('cameraStartButton');
            const cameraStopButton = document.getElementById('cameraStopButton');
            const cameraUpdateButton = document.getElementById('cameraUpdateButton');
            const cameraContainer = document.getElementById('cameraContainer');

            cameraInitButton.addEventListener('click', initCamera);
            cameraStartButton.addEventListener('click', startCameraStream);
            cameraStopButton.addEventListener('click', stopCameraStream);
            cameraUpdateButton.addEventListener('click', updateCameraSettings);

            // Load current camera settings
            loadCameraSettings();
        }

        async function initCamera() {
            const cameraIndex = parseInt(document.getElementById('cameraIndexInput').value);
            const width = parseInt(document.getElementById('cameraWidthInput').value);
            const height = parseInt(document.getElementById('cameraHeightInput').value);
            const fps = parseInt(document.getElementById('cameraFpsInput').value);

            try {
                const response = await fetch('/camera/init', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        camera_index: cameraIndex, 
                        width: width, 
                        height: height, 
                        fps: fps 
                    }),
                });
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.textContent = `Camera initialized: ${result.message}`;
                } else {
                    statusDiv.textContent = `Camera init failed: ${result.message}`;
                }
            } catch (error) {
                statusDiv.textContent = `Camera init error: ${error}`;
                console.error('Camera init error:', error);
            }
        }

        async function startCameraStream() {
            try {
                const response = await fetch('/camera/start', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.textContent = `Camera started: ${result.message}`;
                    document.getElementById('cameraContainer').style.display = 'block';
                    // Add timestamp to prevent caching
                    document.getElementById('cameraFeed').src = '/video_feed?' + new Date().getTime();
                } else {
                    statusDiv.textContent = `Camera start failed: ${result.message}`;
                }
            } catch (error) {
                statusDiv.textContent = `Camera start error: ${error}`;
                console.error('Camera start error:', error);
            }
        }

        async function stopCameraStream() {
            try {
                const response = await fetch('/camera/stop', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.textContent = `Camera stopped: ${result.message}`;
                    document.getElementById('cameraContainer').style.display = 'none';
                } else {
                    statusDiv.textContent = `Camera stop failed: ${result.message}`;
                }
            } catch (error) {
                statusDiv.textContent = `Camera stop error: ${error}`;
                console.error('Camera stop error:', error);
            }
        }

        async function updateCameraSettings() {
            const cameraIndex = parseInt(document.getElementById('cameraIndexInput').value);
            const width = parseInt(document.getElementById('cameraWidthInput').value);
            const height = parseInt(document.getElementById('cameraHeightInput').value);
            const fps = parseInt(document.getElementById('cameraFpsInput').value);
            const quality = parseInt(document.getElementById('cameraQualityInput').value);

            try {
                const response = await fetch('/camera/settings', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        camera_index: cameraIndex, 
                        width: width, 
                        height: height, 
                        fps: fps,
                        quality: quality
                    }),
                });
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.textContent = `Camera settings updated: ${result.message}`;
                    // Update input fields with actual values
                    const settings = result.settings;
                    document.getElementById('cameraIndexInput').value = settings.camera_index;
                    document.getElementById('cameraWidthInput').value = settings.width;
                    document.getElementById('cameraHeightInput').value = settings.height;
                    document.getElementById('cameraFpsInput').value = settings.fps;
                    document.getElementById('cameraQualityInput').value = settings.quality;
                } else {
                    statusDiv.textContent = `Camera settings update failed: ${result.message}`;
                }
            } catch (error) {
                statusDiv.textContent = `Camera settings update error: ${error}`;
                console.error('Camera settings update error:', error);
            }
        }

        async function loadCameraSettings() {
            try {
                const response = await fetch('/camera/settings', {
                    method: 'GET',
                    headers: { 'Content-Type': 'application/json' }
                });
                const settings = await response.json();
                
                document.getElementById('cameraIndexInput').value = settings.camera_index;
                document.getElementById('cameraWidthInput').value = settings.width;
                document.getElementById('cameraHeightInput').value = settings.height;
                document.getElementById('cameraFpsInput').value = settings.fps;
                document.getElementById('cameraQualityInput').value = settings.quality;
                
                if (settings.running) {
                    document.getElementById('cameraContainer').style.display = 'block';
                    document.getElementById('cameraFeed').src = '/video_feed?' + new Date().getTime();
                }
            } catch (error) {
                console.error('Failed to load camera settings:', error);
            }
        }

    </script>
</body>
</html>
"""

@app.route('/')
def index():
    global current_speed_val, current_servo_val, current_servo_min_slider_val, \
           current_servo_max_slider_val, current_continuous_send_interval
    return render_template_string(HTML_TEMPLATE,
                                  current_speed=current_speed_val,
                                  current_servo=current_servo_val,
                                  current_servo_min_slider=current_servo_min_slider_val,
                                  current_servo_max_slider=current_servo_max_slider_val,
                                  current_continuous_interval=current_continuous_send_interval,
                                  servo_mid_default=SERVO_MID_DEFAULT,
                                  device_servo_min_hw=DEVICE_SERVO_MIN_HW, # For info text
                                  device_servo_max_hw=DEVICE_SERVO_MAX_HW, # For info text
                                  uint16_min=UINT16_MIN, # For config input limits
                                  uint16_max=UINT16_MAX  # For config input limits
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

# --- Camera Functions ---
def init_camera(camera_index=0, width=640, height=480, fps=30):
    """Initialize camera with specified parameters"""
    global camera, current_camera_index, current_camera_width, current_camera_height, current_camera_fps
    
    try:
        if camera is not None:
            camera.release()
        
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            print(f"Error: Cannot open camera {camera_index}")
            return False
        
        # Set camera properties
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        
        # Verify actual settings
        actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = camera.get(cv2.CAP_PROP_FPS)
        
        current_camera_index = camera_index
        current_camera_width = actual_width
        current_camera_height = actual_height
        current_camera_fps = actual_fps
        
        print(f"Camera {camera_index} initialized: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")
        return True
        
    except Exception as e:
        print(f"Error initializing camera: {e}")
        camera = None
        return False

def camera_thread_func():
    """Camera capture thread function"""
    global camera, camera_running, camera_frame_queue
    
    while camera_running and camera is not None:
        try:
            ret, frame = camera.read()
            if ret:
                # Clear old frames from queue to prevent lag
                try:
                    while not camera_frame_queue.empty():
                        camera_frame_queue.get_nowait()
                except queue.Empty:
                    pass
                
                # Add new frame
                try:
                    camera_frame_queue.put_nowait(frame)
                except queue.Full:
                    pass
            else:
                print("Failed to read frame from camera")
                time.sleep(0.1)
        except Exception as e:
            print(f"Camera thread error: {e}")
            time.sleep(0.1)


def start_camera():
    """Start camera capture thread"""
    global camera_thread, camera_running
    
    if camera is None:
        return False
    
    if camera_running:
        return True
    
    camera_running = True
    camera_thread = threading.Thread(target=camera_thread_func, daemon=True)
    camera_thread.start()
    print("Camera capture started")
    return True

def stop_camera():
    """Stop camera capture"""
    global camera, camera_thread, camera_running
    
    camera_running = False
    
    if camera_thread is not None:
        camera_thread.join(timeout=2)
        camera_thread = None
    
    if camera is not None:
        camera.release()
        camera = None
    
    print("Camera capture stopped")

def get_camera_frame():
    """Get the latest camera frame as JPEG bytes"""
    global camera_frame_queue, current_camera_quality
    
    try:
        frame = camera_frame_queue.get_nowait()
        # Encode frame as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), current_camera_quality]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        if result:
            return encimg.tobytes()
    except queue.Empty:
        pass
    except Exception as e:
        print(f"Error encoding frame: {e}")    
    return None

def update_camera_settings(camera_index=None, width=None, height=None, fps=None, quality=None):
    """Update camera settings"""
    global current_camera_index, current_camera_width, current_camera_height
    global current_camera_fps, current_camera_quality
    
    with camera_lock:
        restart_needed = False
        
        # Update quality without restart
        if quality is not None and 1 <= quality <= 100:
            current_camera_quality = quality
        
        # Check if restart is needed
        if camera_index is not None and camera_index != current_camera_index:
            restart_needed = True
        if width is not None and width != current_camera_width:
            restart_needed = True
        if height is not None and height != current_camera_height:
            restart_needed = True
        if fps is not None and fps != current_camera_fps:
            restart_needed = True
        
        if restart_needed:
            was_running = camera_running
            if was_running:
                stop_camera()
            
            # Update settings
            new_index = camera_index if camera_index is not None else current_camera_index
            new_width = width if width is not None else current_camera_width
            new_height = height if height is not None else current_camera_height
            new_fps = fps if fps is not None else current_camera_fps
            
            success = init_camera(new_index, new_width, new_height, new_fps)
            
            if success and was_running:
                start_camera()
            
            return success
        
        return True

# Register camera cleanup
def cleanup_camera():
    stop_camera()

atexit.register(cleanup_camera)

# --- Flask Camera Routes ---
@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            frame = get_camera_frame()
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                # Send a small delay if no frame available
                time.sleep(0.1)
    
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera/start', methods=['POST'])
def start_camera_route():
    """Start camera capture"""
    if camera is None:
        return jsonify({'success': False, 'message': 'Camera not initialized'})
    
    success = start_camera()
    if success:
        return jsonify({'success': True, 'message': 'Camera started'})
    else:
        return jsonify({'success': False, 'message': 'Failed to start camera'})

@app.route('/camera/stop', methods=['POST'])
def stop_camera_route():
    """Stop camera capture"""
    stop_camera()
    return jsonify({'success': True, 'message': 'Camera stopped'})

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
            'running': camera_running
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
                    'running': camera_running
                }
            })
        else:
            return jsonify({'success': False, 'message': 'Failed to update camera settings'})

@app.route('/camera/init', methods=['POST'])
def init_camera_route():
    """Initialize camera with settings"""
    data = request.get_json()
    
    camera_index = data.get('camera_index', CAMERA_DEFAULT_INDEX)
    width = data.get('width', CAMERA_DEFAULT_WIDTH)
    height = data.get('height', CAMERA_DEFAULT_HEIGHT)
    fps = data.get('fps', CAMERA_DEFAULT_FPS)
    
    success = init_camera(camera_index, width, height, fps)
    
    if success:
        return jsonify({'success': True, 'message': 'Camera initialized successfully'})
    else:
        return jsonify({'success': False, 'message': 'Failed to initialize camera'})

if __name__ == '__main__':
    port_input = input(f"Enter serial port (default: {SERIAL_PORT}): ")
    if port_input.strip():
        SERIAL_PORT = port_input.strip()

    # Initialize camera
    print("Initializing camera...")
    if init_camera(CAMERA_DEFAULT_INDEX, CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_FPS):
        print("Camera initialized successfully")
    else:
        print("Warning: Camera initialization failed, camera features will be unavailable")

    if init_serial(SERIAL_PORT):
        print(f"Flask server starting on http://0.0.0.0:5000")
        print("Open this address in your web browser.")
        print("Camera control available in the web interface.")
        try:
            app.run(host='0.0.0.0', port=5000, debug=False)
        except KeyboardInterrupt:
            print("\nFlask server stopping.")
    else:
        print(f"Could not initialize serial port {SERIAL_PORT}. Exiting.")