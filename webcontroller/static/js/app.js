// ElectricShock Racing Car Control System
// Main JavaScript Application

class CarControlSystem {
    constructor() {
        this.initializeElements();
        this.initializeState();
        this.bindEvents();
        this.loadCameraSettings();
        // Initialize RTMP player
        this.videoPlayer = null;
        this.rtmpUrl = null;
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
        this.cameraBitrateInput = document.getElementById('cameraBitrateInput');
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
    }

    // Camera Methods
    checkRTMPSupport() {
        // Check if RTMP streaming is supported
        // Check if Video.js is available for RTMP support
        return typeof videojs !== 'undefined';
    }

    async initCamera() {
        const cameraIndex = parseInt(this.cameraIndexInput.value);
        const width = parseInt(this.cameraWidthInput.value);
        const height = parseInt(this.cameraHeightInput.value);
        const fps = parseInt(this.cameraFpsInput.value);
        const bitrate = parseInt(this.cameraBitrateInput.value) * 1000; // Convert kbps to bps

        this.showStatus('📷 Initializing camera for RTMP/H.264 streaming...', 'info');

        try {
            const response = await fetch('/camera/init', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ 
                    camera_index: cameraIndex, 
                    width: width, 
                    height: height, 
                    fps: fps,
                    bitrate: bitrate
                }),
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`✅ Camera initialized for RTMP: ${result.message}`, 'success');
                this.rtmpUrl = result.rtmp_url;
            } else {
                this.showStatus(`❌ Camera init failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 Camera init error: ${error}`, 'error');
            console.error('Camera init error:', error);
        }
    }

    async startCameraStream() {
        this.showStatus('📹 Starting RTMP camera stream...', 'info');

        try {
            const response = await fetch('/camera/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`✅ RTMP streaming started: ${result.message}`, 'success');
                this.cameraContainer.classList.remove('hidden');
                this.rtmpUrl = result.rtmp_url;
                
                // Initialize RTMP player
                await this.initRTMPPlayer();
            } else {
                this.showStatus(`❌ RTMP start failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 RTMP start error: ${error}`, 'error');
            console.error('RTMP start error:', error);
        }
    }

    async stopCameraStream() {
        this.showStatus('⏹️ Stopping RTMP camera stream...', 'info');

        try {
            // Destroy RTMP player first
            this.destroyRTMPPlayer();
            
            const response = await fetch('/camera/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`⏹️ RTMP streaming stopped: ${result.message}`, 'warning');
                this.cameraContainer.classList.add('hidden');
            } else {
                this.showStatus(`❌ RTMP stop failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 RTMP stop error: ${error}`, 'error');
            console.error('RTMP stop error:', error);
        }
    }

    async updateCameraSettings() {
        const cameraIndex = parseInt(this.cameraIndexInput.value);
        const width = parseInt(this.cameraWidthInput.value);
        const height = parseInt(this.cameraHeightInput.value);
        const fps = parseInt(this.cameraFpsInput.value);
        const bitrate = parseInt(this.cameraBitrateInput.value) * 1000; // Convert kbps to bps

        this.showStatus('⚙️ Updating RTMP camera settings...', 'info');

        try {
            const response = await fetch('/camera/settings', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ 
                    camera_index: cameraIndex, 
                    width: width, 
                    height: height, 
                    fps: fps,
                    bitrate: bitrate
                }),
            });
            const result = await response.json();
            
            if (result.success) {
                this.showStatus(`✅ RTMP camera settings updated: ${result.message}`, 'success');
                const settings = result.settings;
                this.cameraIndexInput.value = settings.camera_index;
                this.cameraWidthInput.value = settings.width;
                this.cameraHeightInput.value = settings.height;
                this.cameraFpsInput.value = settings.fps;
                this.cameraBitrateInput.value = Math.round(settings.bitrate / 1000); // Convert bps to kbps
                this.rtmpUrl = settings.rtmp_url;
                
                // Restart RTMP player if running
                if (settings.running) {
                    await this.initRTMPPlayer();
                }
            } else {
                this.showStatus(`❌ RTMP settings update failed: ${result.message}`, 'error');
            }
        } catch (error) {
            this.showStatus(`🔌 RTMP settings update error: ${error}`, 'error');
            console.error('RTMP settings update error:', error);
        }
    }

    async loadCameraSettings() {
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
            this.cameraBitrateInput.value = Math.round(settings.bitrate / 1000); // Convert bps to kbps
            this.rtmpUrl = settings.rtmp_url;
            
            if (settings.running) {
                this.cameraContainer.classList.remove('hidden');
                await this.initRTMPPlayer();
                this.showStatus('📹 RTMP streaming is already running with H.264 optimization', 'success');
            } else {
                // Ensure player is destroyed if not running
                this.destroyRTMPPlayer();
            }
        } catch (error) {
            console.error('Failed to load camera settings:', error);
            this.showStatus('⚠️ Failed to load RTMP camera settings', 'warning');
        }
    }

    // RTMP Player Management
    async initRTMPPlayer() {
        this.destroyRTMPPlayer(); // Ensure any existing player is disposed of first
        
        if (!this.rtmpUrl) {
            this.showStatus('❌ No RTMP URL available', 'error');
            return;
        }

        // IMPORTANT: For RTMP playback, ensure you have included the videojs-flash plugin
        // and that the video-js.swf file is accessible.
        // Example: <script src="https.unpkg.com/videojs-flash/dist/videojs-flash.min.js"></script>
        // The SWF path is configured below.
        const videoJsOptions = {
            controls: true,
            autoplay: true,
            muted: true, // Muting helps with autoplay policies
            preload: 'auto',
            fluid: true, // Makes the player responsive
            responsive: true,
            playsinline: true,
            // Prioritize Flash for RTMP, then HTML5 fallback (though HTML5 won't play RTMP directly)
            techOrder: ['flash', 'html5'],
            flash: {
                // Path to the video-js.swf file.
                // You might need to host this file locally or use a reliable CDN.
                // Example using unpkg CDN:
                swf: 'https://unpkg.com/videojs-swf/dist/video-js.swf',
                // Other Flash-specific options can go here
                // See https://github.com/videojs/videojs-flash
            },
            sources: [{
                src: this.rtmpUrl,
                type: 'rtmp/flv' // Standard type for RTMP with FLV container
            }]
        };

        try {
            if (typeof videojs === 'undefined') {
                this.showStatus('⚠️ Video.js library not found. RTMP playback will likely fail.', 'error');
                // Basic fallback (won't work for RTMP in most modern browsers)
                if (this.cameraFeed instanceof HTMLMediaElement) {
                    this.cameraFeed.src = this.rtmpUrl;
                    this.cameraFeed.load();
                }
                return;
            }

            // Ensure cameraFeed is a video element and not already a Video.js player
            if (!(this.cameraFeed instanceof HTMLVideoElement)) {
                this.showStatus('❌ Camera feed element is not a <video> tag.', 'error');
                console.error('Camera feed element is not a <video> tag:', this.cameraFeed);
                return;
            }
            
            // Initialize Video.js player
            this.videoPlayer = videojs(this.cameraFeed, videoJsOptions);

            this.videoPlayer.ready(() => {
                console.log('RTMP player initialized with low latency settings');
                this.showStatus('📺 RTMP player initialized. Attempting to play...', 'success');
                this.videoPlayer.play().catch(e => {
                    console.warn('Autoplay was prevented:', e);
                    this.showStatus('⚠️ Autoplay prevented. Click play on video.', 'warning');
                });
            });

            this.videoPlayer.on('error', (e) => {
                const error = this.videoPlayer.error();
                console.error('RTMP player error:', error);
                let errorMsg = 'Unknown playback error';
                if (error) {
                    errorMsg = `CODE:${error.code} ${error.message}`;
                    if (error.code === 4) { // MEDIA_ERR_SRC_NOT_SUPPORTED
                        errorMsg += " (Ensure Flash is enabled and videojs-flash plugin is loaded with correct SWF path if using RTMP)";
                    }
                }
                this.showStatus(`❌ RTMP playback error: ${errorMsg}`, 'error');
            });

            this.videoPlayer.on('loadstart', () => console.log('RTMP stream loading started'));
            this.videoPlayer.on('canplay', () => console.log('RTMP stream ready to play'));
            
        } catch (error) {
            console.error('Error initializing RTMP player:', error);
            this.showStatus(`❌ RTMP player initialization error: ${error.message}`, 'error');
        }
    }

    destroyRTMPPlayer() {
        if (this.videoPlayer) {
            try {
                this.videoPlayer.dispose(); // Properly disposes of the Video.js player
            } catch (e) {
                console.warn('Error disposing video player:', e);
            }
            this.videoPlayer = null;
        }
        
        // Reset the original video element if it's a media element
        // This helps ensure it's clean for potential re-initialization
        if (this.cameraFeed && typeof this.cameraFeed.src !== 'undefined') {
            this.cameraFeed.src = ''; 
            if (typeof this.cameraFeed.load === 'function') {
                // Only call load if it's a native video element and not managed by Video.js anymore
                // However, after dispose(), the element should be back to its original state.
                // this.cameraFeed.load(); // Usually not needed after dispose
            }
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

// RTMP player configuration for optimal low latency - This global variable is not used in the class.
// Consider integrating these options into the initRTMPPlayer method if specific settings are desired beyond the defaults.
// const videoConfig = {
//     autoplay: true,
//     muted: true,
//     controls: true,
//     preload: 'auto',
//     playsinline: true,
//     // RTMP specific low latency settings
//     rtmpConfig: {
//         bufferTime: 0.1,      // Minimal buffer
//         liveStreamMode: true,
//         lowLatencyMode: true,
//         maxRetries: 5,
//         retryDelay: 1000
//     }
// };
