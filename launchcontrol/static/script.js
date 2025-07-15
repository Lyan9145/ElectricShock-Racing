document.addEventListener('DOMContentLoaded', () => {
    // --- 全局状态 ---
    let carRunning = false;
    let logEventSource = null;
    const FRAME_RATE = 30; // 假设视频帧率为30fps
    const FRAME_STEP = 1 / FRAME_RATE;

    // --- DOM 元素获取 ---
    const startBtn = document.getElementById('start-btn');
    const stopBtn = document.getElementById('stop-btn');
    const statusLight = document.getElementById('status-light');
    const statusText = document.getElementById('status-text');
    const logOutput = document.getElementById('log-output');
    
    const navTabs = document.querySelectorAll('.nav-tab');
    const tabContents = document.querySelectorAll('.tab-content');

    const videoPlayer = document.getElementById('video-player');
    const videoList = document.getElementById('video-list');
    const videoTitle = document.getElementById('video-title');
    const prevFrameBtn = document.getElementById('prev-frame-btn');
    const nextFrameBtn = document.getElementById('next-frame-btn');

    const configForm = document.getElementById('config-form');
    const saveConfigBtn = document.getElementById('save-config-btn');
    const presetSelect = document.getElementById('preset-select');
    const loadPresetBtn = document.getElementById('load-preset-btn');
    const savePresetInput = document.getElementById('save-preset-input');
    const savePresetBtn = document.getElementById('save-preset-btn');

    // --- 工具函数 ---
    const showToast = (message, type = 'success') => {
        const container = document.getElementById('toast-container');
        const toast = document.createElement('div');
        const bgColor = type === 'success' ? 'bg-green-500' : (type === 'error' ? 'bg-red-500' : 'bg-yellow-500');
        toast.className = `p-4 rounded-lg text-white shadow-lg mb-2 ${bgColor} animate-pulse`;
        toast.textContent = message;
        container.appendChild(toast);
        setTimeout(() => {
            toast.remove();
        }, 3000);
    };

    // --- 导航逻辑 ---
    navTabs.forEach(tab => {
        tab.addEventListener('click', () => {
            const target = tab.dataset.tab;

            navTabs.forEach(t => {
                t.classList.remove('text-blue-600', 'border-blue-600');
                t.classList.add('text-gray-500');
            });
            tab.classList.add('text-blue-600', 'border-blue-600');
            tab.classList.remove('text-gray-500');

            tabContents.forEach(content => {
                content.classList.add('hidden');
            });
            document.getElementById(`${target}-page`).classList.remove('hidden');
            
            // 切换到特定页面时加载数据
            if (target === 'recorder') loadVideos();
            if (target === 'config') {
                loadConfig();
                loadPresets();
            }
        });
    });

    // --- 车辆控制逻辑 ---
    const updateControlUI = (running) => {
        carRunning = running;
        if (running) {
            statusLight.classList.remove('bg-gray-400', 'bg-red-500');
            statusLight.classList.add('bg-green-500', 'animate-pulse');
            statusText.textContent = '车辆状态: 运行中';
            startBtn.disabled = true;
            stopBtn.disabled = false;
            // 运行时禁用其他导航
            navTabs.forEach(tab => { if(tab.dataset.tab !== 'control') tab.disabled = true; });
        } else {
            statusLight.classList.remove('bg-green-500', 'animate-pulse');
            statusLight.classList.add('bg-red-500');
            statusText.textContent = '车辆状态: 已停止';
            startBtn.disabled = false;
            stopBtn.disabled = true;
            // 停止时启用其他导航
            navTabs.forEach(tab => tab.disabled = false);
        }
    };

    const checkCarStatus = async () => {
        try {
            const response = await fetch('/api/car_status');
            const data = await response.json();
            updateControlUI(data.running);
            if(data.running && !logEventSource) {
                startLogStream();
            }
        } catch (error) {
            console.error('无法检查车辆状态:', error);
            updateControlUI(false);
        }
    };

    const startLogStream = () => {
        if (logEventSource) {
            logEventSource.close();
        }
        logOutput.textContent = '';
        logEventSource = new EventSource('/api/log_stream');
        logEventSource.onmessage = (event) => {
            // 过滤心跳
            if (event.data === "❤" || event.data === "\u2764") return;
            logOutput.textContent += event.data + '\n';
            logOutput.scrollTop = logOutput.scrollHeight;
        };
        logEventSource.onerror = (e) => {
            console.error('日志流连接错误，正在关闭。', e);
            logOutput.textContent += '[日志流连接错误]\n';
            logEventSource.close();
            logEventSource = null;
        };
    };

    startBtn.addEventListener('click', async () => {
        try {
            const response = await fetch('/api/start_car', { method: 'POST' });
            const data = await response.json();
            if (response.ok) {
                showToast(data.message, 'success');
                updateControlUI(true);
                startLogStream();
            } else {
                showToast(data.message, 'error');
            }
        } catch (error) {
            showToast('启动请求失败', 'error');
        }
    });

    stopBtn.addEventListener('click', () => {
        if (confirm('确定要紧急停止车辆吗？')) {
            fetch('/api/stop_car', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    if (data.status.includes('success') || data.status.includes('warning')) {
                        showToast(data.message, data.status);
                        updateControlUI(false);
                        if(logEventSource) logEventSource.close();
                    } else {
                        showToast(data.message, 'error');
                    }
                })
                .catch(error => showToast('停止请求失败', 'error'));
        }
    });

    // --- 行车记录逻辑 ---
    const loadVideos = async () => {
        try {
            const response = await fetch('/api/videos');
            const files = await response.json();
            videoList.innerHTML = '';
            if (files.length === 0) {
                videoList.innerHTML = '<p class="text-gray-500">没有找到录制的视频。</p>';
                return;
            }
            files.forEach(file => {
                const li = document.createElement('a');
                li.href = '#';
                li.textContent = file;
                li.className = 'block p-2 rounded-md hover:bg-gray-200 cursor-pointer';
                li.addEventListener('click', (e) => {
                    e.preventDefault();
                    videoPlayer.src = `/videos/${file}`;
                    videoTitle.textContent = file;
                    document.querySelectorAll('#video-list a').forEach(a => a.classList.remove('bg-blue-100', 'font-bold'));
                    li.classList.add('bg-blue-100', 'font-bold');
                });
                videoList.appendChild(li);
            });
        } catch (error) {
            videoList.innerHTML = '<p class="text-red-500">加载视频列表失败。</p>';
        }
    };
    
    prevFrameBtn.addEventListener('click', () => {
        if (!videoPlayer.paused) videoPlayer.pause();
        videoPlayer.currentTime = Math.max(0, videoPlayer.currentTime - FRAME_STEP);
    });

    nextFrameBtn.addEventListener('click', () => {
        if (!videoPlayer.paused) videoPlayer.pause();
        videoPlayer.currentTime += FRAME_STEP;
    });

    // --- 参数配置逻辑 ---
    const populateConfigForm = (config) => {
        configForm.innerHTML = '';
        const descriptions = config.record && config.record[0] ? config.record[0] : {};

        for (const key in config) {
            if (key === 'record') continue;

            const value = config[key];
            const description = descriptions[`#${key}`] || '没有提供描述信息';

            const group = document.createElement('div');
            group.className = 'grid grid-cols-1 md:grid-cols-3 gap-2 items-center';
            
            const label = document.createElement('label');
            label.htmlFor = `config-${key}`;
            label.textContent = key;
            label.className = 'font-semibold text-gray-700';
            
            const inputContainer = document.createElement('div');
            inputContainer.className = 'md:col-span-2 flex items-center';

            let input;
            if (typeof value === 'boolean') {
                input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = value;
                input.className = 'h-5 w-5 text-blue-600 border-gray-300 rounded focus:ring-blue-500';
            } else {
                input = document.createElement('input');
                input.type = typeof value === 'number' ? 'number' : 'text';
                input.value = value;
                input.className = 'block w-full shadow-sm sm:text-sm border-gray-300 rounded-md';
                if(typeof value === 'number') input.step = 'any';
            }
            input.id = `config-${key}`;
            input.name = key;

            const descElement = document.createElement('p');
            descElement.textContent = description;
            descElement.className = 'text-xs text-gray-500 mt-1 md:col-start-2 md:col-span-2';

            inputContainer.appendChild(input);
            group.appendChild(label);
            group.appendChild(inputContainer);
            configForm.appendChild(group);
            configForm.appendChild(descElement);
        }
    };

    const loadConfig = async () => {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();
            populateConfigForm(config);
        } catch (error) {
            showToast('加载配置文件失败', 'error');
        }
    };

    saveConfigBtn.addEventListener('click', async () => {
        const formData = new FormData(configForm);
        const configData = {};
        
        // 需要先从原始配置加载，以获取正确的类型
        const originalConfigResponse = await fetch('/api/config');
        const originalConfig = await originalConfigResponse.json();

        for (const key in originalConfig) {
            if (key === 'record') continue;

            const value = formData.get(key);
            if (typeof originalConfig[key] === 'boolean') {
                configData[key] = formData.has(key); // Checkbox value
            } else if (typeof originalConfig[key] === 'number') {
                configData[key] = parseFloat(value);
            } else {
                configData[key] = value;
            }
        }
        configData.record = originalConfig.record; // 保留原始的record部分

        try {
            const response = await fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(configData)
            });
            const result = await response.json();
            if(response.ok) {
                showToast(result.message, 'success');
            } else {
                showToast(result.message, 'error');
            }
        } catch (error) {
            showToast('保存配置请求失败', 'error');
        }
    });

    const loadPresets = async () => {
        try {
            const response = await fetch('/api/presets');
            const presets = await response.json();
            presetSelect.innerHTML = '<option value="">选择一个预设...</option>';
            presets.forEach(p => {
                const option = document.createElement('option');
                option.value = p;
                option.textContent = p;
                presetSelect.appendChild(option);
            });
        } catch (error) {
            showToast('加载预设列表失败', 'error');
        }
    };

    savePresetBtn.addEventListener('click', async () => {
        const presetName = savePresetInput.value.trim();
        if (!presetName) {
            showToast('请输入预设名称', 'error');
            return;
        }

        // 构造当前表单的配置数据
        const formData = new FormData(configForm);
        const originalConfigResponse = await fetch('/api/config');
        const originalConfig = await originalConfigResponse.json();
        const configData = {};
        for (const key in originalConfig) {
            if (key === 'record') continue;
            const value = formData.get(key);
            if (typeof originalConfig[key] === 'boolean') configData[key] = formData.has(key);
            else if (typeof originalConfig[key] === 'number') configData[key] = parseFloat(value);
            else configData[key] = value;
        }
        configData.record = originalConfig.record;
        
        try {
             const response = await fetch('/api/presets', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: presetName, config: configData })
            });
            const result = await response.json();
            if(response.ok) {
                showToast(result.message, 'success');
                savePresetInput.value = '';
                loadPresets(); // 刷新列表
            } else {
                showToast(result.message, 'error');
            }
        } catch (error) {
            showToast('保存预设请求失败', 'error');
        }
    });

    loadPresetBtn.addEventListener('click', async () => {
        const presetName = presetSelect.value;
        if (!presetName) {
            showToast('请先选择一个预设', 'error');
            return;
        }
        try {
            const response = await fetch('/api/presets/load', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: presetName })
            });
            const result = await response.json();
            if(response.ok) {
                showToast(result.message, 'success');
                populateConfigForm(result.config); // 使用返回的配置更新表单
            } else {
                showToast(result.message, 'error');
            }
        } catch (error) {
            showToast('加载预设请求失败', 'error');
        }
    });

    // --- 初始化 ---
    const initializeApp = () => {
        checkCarStatus();
    };

    initializeApp();
});