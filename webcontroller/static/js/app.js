// ElectricShock Racing Car Control System
// Main JavaScript Application

class CarControlSystem {
    constructor() {
        this.initializeElements();
        this.initializeState();        this.bindEvents();
        this.loadCameraSettings();
        // Simple MJPEG player
        this.mjpegStream = null;
    }

    initializeElements() {
        // Speed controls
        this.speedSlider = document.getElementById('speedSlider');
        this.speedNumber = document.getElementById('speedNumber');
        this.speedValueDisplay = document.getElementById('speedValueDisplay');
        
        // Servo controls
        this.servoSlider = document.getElementById('servoSlider');
        this.servoNumber = document.getElementById('servoNumber');
        this.servoValueDisplay = document.getElementById('servoValueDisplay');
        this.servoMinPwmInput = document.getElementById('servoMinPwmInput');
        this.servoMaxPwmInput = document.getElementById('servoMaxPwmInput');
        this.servoMinDisplay = document.getElementById('servoMinDisplay');
        this.servoMaxDisplay = document.getElementById('servoMaxDisplay');
          // Control buttons
        this.resetButton = document.getElementById('resetButton');
        this.emergencyStopButton = document.getElementById('emergencyStopButton');
        this.statusDiv = document.getElementById('status');
        
        // Continuous send
        this.continuousSendIntervalInput = document.getElementById('continuousSendIntervalInput');
        this.continuousSendToggle = document.getElementById('continuousSendToggle');
        
        // Camera controls
        this.cameraInitButton = document.getElementById('cameraInitButton');
        this.cameraStartButton = document.getElementById('cameraStartButton');
        this.cameraStopButton = document.getElementById('cameraStopButton');
        this.cameraUpdateButton = document.getElementById('cameraUpdateButton');
        this.cameraContainer = document.getElementById('cameraContainer');
        // IMPORTANT: Ensure 'cameraFeed' in your HTML is a <video> element, not <img> or <div>.
        // Example: <video id="cameraFeed" class="video-js vjs-default-skin w-full h-full object-contain"></video>
        this.cameraFeed = document.getElementById('cameraFeed');
          // Camera settings
        this.cameraIndexInput = document.getElementById('cameraIndexInput');
        this.cameraWidthInput = document.getElementById('cameraWidthInput');
        this.cameraHeightInput = document.getElementById('cameraHeightInput');
        this.cameraFpsInput = document.getElementById('cameraFpsInput');
        this.cameraQualityInput = document.getElementById('cameraQualityInput');
    }

    initializeState() {
        // Constants from template
        this.UINT16_MIN = parseInt(document.querySelector('[data-uint16-min]')?.dataset.uint16Min) || 0;
        this.UINT16_MAX = parseInt(document.querySelector('[data-uint16-max]')?.dataset.uint16Max) || 65535;
        this.SERVO_MID_DEFAULT = parseInt(document.querySelector('[data-servo-mid]')?.dataset.servoMid) || 1500;
        
        // Current state
        this.currentServoMinSlider = parseInt(this.servoMinPwmInput.value);
        this.currentServoMaxSlider = parseInt(this.servoMaxPwmInput.value);
        this.manualSendLastTime = 0;
        this.manualSendInterval = 100;
        this.continuousSendTimerId = null;
        this.currentContinuousInterval = parseInt(this.continuousSendIntervalInput.value);
        
        // Initialize displays
        this.updateSpeedControls(this.speedSlider.value);
        this.updateServoControls(this.servoSlider.value);
        this.updateServoRangeDisplay();
    }

    bindEvents() {
        // Speed controls
        this.speedSlider.addEventListener('input', (e) => this.handleSpeedChange(e));
        this.speedNumber.addEventListener('change', (e) => this.handleSpeedChange(e));
        
        // Servo controls
        this.servoSlider.addEventListener('input', (e) => this.handleServoChange(e));
        this.servoNumber.addEventListener('change', (e) => this.handleServoChange(e));
        
        // Servo range configuration
        this.servoMinPwmInput.addEventListener('change', () => this.applyServoRangeChanges());
        this.servoMaxPwmInput.addEventListener('change', () => this.applyServoRangeChanges());
          // Control buttons
        this.resetButton.addEventListener('click', () => this.handleReset());
        this.emergencyStopButton.addEventListener('click', () => this.handleEmergencyStop());
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => this.handleKeyboard(e));
        
        // Continuous send
        this.continuousSendIntervalInput.addEventListener('change', () => this.handleIntervalChange());
        this.continuousSendToggle.addEventListener('click', () => this.toggleContinuousSend());
        
        // Camera controls
        this.cameraInitButton.addEventListener('click', () => this.initCamera());
        this.cameraStartButton.addEventListener('click', () => this.startCameraStream());
        this.cameraStopButton.addEventListener('click', () => this.stopCameraStream());
        this.cameraUpdateButton.addEventListener('click', () => this.updateCameraSettings());
    }

    // Speed Control Methods
    updateSpeedControls(value) {
        const val = parseFloat(value).toFixed(2);
        this.speedSlider.value = val;
        this.speedNumber.value = val;
        this.speedValueDisplay.textContent = val;
        this.speedValueDisplay.className = this.getSpeedDisplayClass(parseFloat(val));
    }

    getSpeedDisplayClass(speed) {
        const baseClass = 'px-3 py-1 rounded-full text-sm font-bold text-white';
        if (speed > 0.1) return `${baseClass} bg-green-500`;
        if (speed < -0.1) return `${baseClass} bg-red-500`;
        return `${baseClass} bg-racing-orange`;
    }

    handleSpeedChange(event) {
        this.updateSpeedControls(event.target.value);
        this.sendControlCommand(false, true);
    }

    // Servo Control Methods
    updateServoControls(value) {
        const val = parseInt(value);
        const clampedVal = Math.max(this.currentServoMinSlider, Math.min(val, this.currentServoMaxSlider));
        this.servoSlider.value = clampedVal;
        this.servoNumber.value = clampedVal;
        this.servoValueDisplay.textContent = clampedVal;
        return clampedVal;
    }

    handleServoChange(event) {
        this.updateServoControls(event.target.value);
        this.sendControlCommand(false, true);
    }

    applyServoRangeChanges() {
        let newMin = parseInt(this.servoMinPwmInput.value);
        let newMax = parseInt(this.servoMaxPwmInput.value);

        newMin = Math.max(this.UINT16_MIN, Math.min(newMin, this.UINT16_MAX - 1));
        newMax = Math.max(this.UINT16_MIN + 1, Math.min(newMax, this.UINT16_MAX));
        
        if (newMin >= newMax) {
            this.showStatus('Error: Slider Min PWM must be less than Slider Max PWM.', 'error');
            this.servoMinPwmInput.value = this.currentServoMinSlider;
            this.servoMaxPwmInput.value = this.currentServoMaxSlider;
            return;
        }

        this.currentServoMinSlider = newMin;
        this.currentServoMaxSlider = newMax;

        this.servoSlider.min = this.currentServoMinSlider;
        this.servoSlider.max = this.currentServoMaxSlider;
        this.servoNumber.min = this.currentServoMinSlider;
        this.servoNumber.max = this.currentServoMaxSlider;

        this.updateServoRangeDisplay();
        
        let currentServoValue = parseInt(this.servoSlider.value);
        this.updateServoControls(currentServoValue);
    }

    updateServoRangeDisplay() {
        this.servoMinDisplay.textContent = this.currentServoMinSlider;
        this.servoMaxDisplay.textContent = this.currentServoMaxSlider;
    }

    // Control Commands
    async sendControlCommand(forceSend = false, isManualChange = false) {
        const currentTime = Date.now();
        if (isManualChange && !forceSend && (currentTime - this.manualSendLastTime < this.manualSendInterval)) {
            return;
        }
        if (isManualChange) {
            this.manualSendLastTime = currentTime;
        }

        const speed = parseFloat(this.speedSlider.value);
        const servo = parseInt(this.servoSlider.value);
        
        let actionMsg = "Sending";
        if (this.continuousSendTimerId && !isManualChange && !forceSend) actionMsg = "Auto-Sending";

        if (isManualChange || forceSend || !this.continuousSendTimerId) {
            this.showStatus(`${actionMsg}: Speed=${speed.toFixed(2)}, Servo=${servo}`, 'info');
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
                    this.showStatus(`✅ Sent: Speed=${speed.toFixed(2)}, Servo=${servo}`, 'success');
                }
            } else {
                this.showStatus(`❌ Error: ${result.message}`, 'error');
                if (this.continuousSendTimerId) this.toggleContinuousSend();
            }
        } catch (error) {
            this.showStatus(`🔌 Connection error: ${error}`, 'error');
            console.error('Error sending command:', error);
            if (this.continuousSendTimerId) this.toggleContinuousSend();
        }
    }    handleReset() {
        if (this.continuousSendTimerId) this.toggleContinuousSend();
        this.updateSpeedControls(0.0);
        let resetServoValue = this.SERVO_MID_DEFAULT;
        resetServoValue = Math.max(this.currentServoMinSlider, Math.min(resetServoValue, this.currentServoMaxSlider));
        this.updateServoControls(resetServoValue);
        this.sendControlCommand(true, false);
        this.showStatus('🔄 系统已重置: 速度=0, 舵机=' + resetServoValue, 'warning');
    }

    handleEmergencyStop() {
        // Immediately stop any continuous sending
        if (this.continuousSendTimerId) this.toggleContinuousSend();
        
        // Set speed to 0 immediately
        this.updateSpeedControls(0.0);
        
        // Send emergency stop command
        this.sendControlCommand(true, false);
        
        // Show emergency stop message
        this.showStatus('🚨 紧急停车已激活！车辆已停止。', 'error');
        
        // Add visual feedback
        this.emergencyStopButton.classList.add('animate-pulse');
        setTimeout(() => {
            this.emergencyStopButton.classList.remove('animate-pulse');
        }, 2000);
    }

    handleKeyboard(event) {
        // Emergency stop with ESC key
        if (event.key === 'Escape') {
            event.preventDefault();
            this.handleEmergencyStop();
            return;
        }
        
        // Space bar for quick stop (speed to 0, keep servo position)
        if (event.key === ' ' || event.code === 'Space') {
            event.preventDefault();
            this.updateSpeedControls(0.0);
            this.sendControlCommand(true, false);
            this.showStatus('⚡ 快速停车: 速度设为0', 'warning');
            return;
        }
    }

    // Continuous Send Methods
    handleIntervalChange() {
        let newInterval = parseInt(this.continuousSendIntervalInput.value);
        if (newInterval < 50) newInterval = 50;
        if (newInterval > 5000) newInterval = 5000;
        this.continuousSendIntervalInput.value = newInterval;
        this.currentContinuousInterval = newInterval;
        
        if (this.continuousSendTimerId) {
            this.toggleContinuousSend();
            this.toggleContinuousSend();
        }
    }

    toggleContinuousSend() {
        if (this.continuousSendTimerId) {
            clearInterval(this.continuousSendTimerId);
            this.continuousSendTimerId = null;
            this.continuousSendToggle.textContent = 'Start Continuous';
            this.continuousSendToggle.classList.remove('active', 'bg-red-600');
            this.continuousSendToggle.classList.add('bg-purple-600');
            this.showStatus('⏹️ Continuous send stopped.', 'warning');
        } else {
            this.continuousSendTimerId = setInterval(() => {
                this.sendControlCommand(false, false);
            }, this.currentContinuousInterval);
            this.continuousSendToggle.textContent = 'Stop Continuous';
            this.continuousSendToggle.classList.add('active', 'bg-red-600');
            this.continuousSendToggle.classList.remove('bg-purple-600');
            this.showStatus(`▶️ Continuous send started (${this.currentContinuousInterval}ms).`, 'success');
        }
    }    // Camera Methods
    async initCamera() {
        const cameraIndex = parseInt(this.cameraIndexInput.value);
        const width = parseInt(this.cameraWidthInput.value);
        const height = parseInt(this.cameraHeightInput.value);
        const fps = parseInt(this.cameraFpsInput.value);
        const quality = parseInt(this.cameraQualityInput.value);

        this.showStatus('📷 Initializing camera for MJPEG streaming...', 'info');

        try {
            const response = await fetch('/camera/init', {
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
                this.showStatus(`✅ Camera initialized for MJPEG: ${result.message}`, 'success');
            } else {
                this.showStatus(`❌ Camera init failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 Camera init error: ${error}`, 'error');
            console.error('Camera init error:', error);
        }
    }    async startCameraStream() {
        this.showStatus('📹 Starting MJPEG camera stream...', 'info');

        try {
            const response = await fetch('/camera/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`✅ MJPEG streaming started: ${result.message}`, 'success');
                this.cameraContainer.classList.remove('hidden');
                
                // Initialize MJPEG player
                this.initMJPEGPlayer();
            } else {
                this.showStatus(`❌ MJPEG start failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 MJPEG start error: ${error}`, 'error');
            console.error('MJPEG start error:', error);
        }
    }    async stopCameraStream() {
        this.showStatus('⏹️ Stopping MJPEG camera stream...', 'info');

        try {
            // Stop MJPEG player first
            this.stopMJPEGPlayer();
            
            const response = await fetch('/camera/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`⏹️ MJPEG streaming stopped: ${result.message}`, 'warning');
                this.cameraContainer.classList.add('hidden');
            } else {
                this.showStatus(`❌ MJPEG stop failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 MJPEG stop error: ${error}`, 'error');
            console.error('MJPEG stop error:', error);
        }
    }    async updateCameraSettings() {
        const cameraIndex = parseInt(this.cameraIndexInput.value);
        const width = parseInt(this.cameraWidthInput.value);
        const height = parseInt(this.cameraHeightInput.value);
        const fps = parseInt(this.cameraFpsInput.value);
        const quality = parseInt(this.cameraQualityInput.value);

        this.showStatus('⚙️ Updating MJPEG camera settings...', 'info');

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
                this.showStatus(`✅ MJPEG camera settings updated: ${result.message}`, 'success');
                const settings = result.settings;
                this.cameraIndexInput.value = settings.camera_index;
                this.cameraWidthInput.value = settings.width;
                this.cameraHeightInput.value = settings.height;
                this.cameraFpsInput.value = settings.fps;
                this.cameraQualityInput.value = settings.quality;
                
                // Restart MJPEG player if running
                if (settings.running) {
                    this.initMJPEGPlayer();
                }
            } else {
                this.showStatus(`❌ MJPEG settings update failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 MJPEG settings update error: ${error}`, 'error');
            console.error('MJPEG settings update error:', error);
        }
    }    async loadCameraSettings() {
        try {
            const response = await fetch('/camera/settings', {
                method: 'GET',
                headers: { 'Content-Type': 'application/json' }
            });
            const settings = await response.json();
            
            this.cameraIndexInput.value = settings.camera_index;
            this.cameraWidthInput.value = settings.width;
            this.cameraHeightInput.value = settings.height;
            this.cameraFpsInput.value = settings.fps;
            this.cameraQualityInput.value = settings.quality;
            
            if (settings.running) {
                this.cameraContainer.classList.remove('hidden');
                this.initMJPEGPlayer();
                this.showStatus('📹 MJPEG streaming is already running', 'success');
            } else {
                // Ensure player is stopped if not running
                this.stopMJPEGPlayer();
            }
        } catch (error) {
            console.error('Failed to load camera settings:', error);
            this.showStatus('⚠️ Failed to load MJPEG camera settings', 'warning');
        }
    }

    // MJPEG Player Management
    initMJPEGPlayer() {
        this.stopMJPEGPlayer(); // Stop any existing stream first
        
        if (this.cameraFeed) {
            // Simple MJPEG streaming - just set the src to the MJPEG endpoint
            this.cameraFeed.src = '/video_feed';
            this.mjpegStream = this.cameraFeed;
            
            this.cameraFeed.onload = () => {
                console.log('MJPEG stream loaded successfully');
                this.showStatus('📺 MJPEG player initialized and streaming', 'success');
            };
            
            this.cameraFeed.onerror = (e) => {
                console.error('MJPEG stream error:', e);
                this.showStatus('❌ MJPEG stream error. Check camera connection.', 'error');
            };
        }
    }

    stopMJPEGPlayer() {
        if (this.mjpegStream && this.cameraFeed) {
            this.cameraFeed.src = '';
            this.mjpegStream = null;
        }
    }

    // Utility Methods
    showStatus(message, type = 'info') {
        this.statusDiv.textContent = message;
        this.statusDiv.className = `bg-gray-50 rounded-lg p-4 text-sm font-mono break-words min-h-16 flex items-center status-${type}`;
        
        // Auto-clear non-error messages after 5 seconds
        if (type !== 'error') {
            setTimeout(() => {
                if (this.statusDiv.textContent === message) {
                    this.statusDiv.textContent = 'System ready. Connect serial and use controls.';
                    this.statusDiv.className = 'bg-gray-50 rounded-lg p-4 text-sm font-mono break-words min-h-16 flex items-center';
                }
            }, 5000);
        }
    }
}

// Initialize the application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new CarControlSystem();
});
