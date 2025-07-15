import os
import subprocess
import signal
import json
import time
from flask import Flask, render_template, jsonify, request, Response, send_from_directory

# --- 配置 ---
# 确定项目根目录，无论从哪里运行app.py都能找到正确路径
APP_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(APP_DIR, '..'))
ICAR_SRC_DIR = os.path.join(PROJECT_ROOT, 'sasu_icar2025_demo', 'src')
CONFIG_PATH = os.path.join(ICAR_SRC_DIR, 'config', 'config.json')
RECORDER_PATH = os.path.join(ICAR_SRC_DIR, 'recorder')
LOG_FILE_PATH = os.path.join(APP_DIR, 'icar_run.log')
PRESETS_DIR = os.path.join(APP_DIR, 'presets')
ICAR_EXECUTABLE = './icar' # icar可执行文件的名称

# 确保预设和日志目录存在
os.makedirs(PRESETS_DIR, exist_ok=True)
if not os.path.exists(LOG_FILE_PATH):
    open(LOG_FILE_PATH, 'w').close() # 创建空的日志文件

# --- 全局变量 ---
# 用于存储正在运行的icar进程
car_process = None

# --- Flask应用初始化 ---
app = Flask(__name__, template_folder='templates', static_folder='static')


# --- 页面路由 ---
@app.route('/')
def index():
    """渲染主页面"""
    return render_template('index.html')

# --- API 路由 ---

# 1. 车辆控制
@app.route('/api/start_car', methods=['POST'])
def start_car():
    """启动icar可执行文件"""
    global car_process
    if car_process and car_process.poll() is None:
        return jsonify({'status': 'error', 'message': '车辆已在运行中'}), 400

    if not os.path.exists(os.path.join(ICAR_SRC_DIR, ICAR_EXECUTABLE)):
         return jsonify({'status': 'error', 'message': f'错误: 未找到可执行文件 {ICAR_EXECUTABLE}'}), 500

    try:
        # 清空旧的日志文件
        open(LOG_FILE_PATH, 'w').close()
        # 打开日志文件以追加模式写入
        log_file = open(LOG_FILE_PATH, 'a')
        # 在icar的src目录下执行，并将输出重定向到日志文件
        car_process = subprocess.Popen(
            [ICAR_EXECUTABLE],
            cwd=ICAR_SRC_DIR,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1 # 行缓冲
        )
        return jsonify({'status': 'success', 'message': '车辆启动成功'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/stop_car', methods=['POST'])
def stop_car():
    """发送SIGINT (Ctrl+C) 信号停止车辆"""
    global car_process
    if car_process and car_process.poll() is None:
        try:
            car_process.send_signal(signal.SIGINT)
            car_process.wait(timeout=20) # 等待进程终止
            car_process = None
            return jsonify({'status': 'success', 'message': '停车指令已发送'})
        except subprocess.TimeoutExpired:
            car_process.kill() # 如果20秒后仍在运行，则强制终止
            car_process = None
            return jsonify({'status': 'warning', 'message': '车辆未在5秒内响应，已强制终止'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500
    else:
        car_process = None # 确保状态正确
        return jsonify({'status': 'error', 'message': '车辆未在运行或进程已丢失'}), 400


@app.route('/api/car_status')
def car_status():
    """检查车辆运行状态"""
    global car_process
    if car_process and car_process.poll() is None:
        return jsonify({'running': True})
    else:
        # 如果进程句柄存在但已结束，或句柄不存在，都视为未运行
        car_process = None
        return jsonify({'running': False})

@app.route('/api/log_stream')
def log_stream():
    """使用Server-Sent Events实时流式传输日志"""
    def generate():
        with open(LOG_FILE_PATH, 'r') as f:
            # 移动到文件末尾
            f.seek(0, 2)
            while True:
                line = f.readline()
                if not line:
                    time.sleep(0.1)
                    continue
                # SSE格式: data: ...\n\n，去除原始行末的换行符
                yield f"data: {line.rstrip()}\n\n"
    return Response(generate(), mimetype='text/event-stream')


# 2. 视频播放
@app.route('/api/videos')
def get_videos():
    """获取录制的视频列表"""
    try:
        files = [f for f in os.listdir(RECORDER_PATH) if f.endswith('.mp4')]
        # 按修改时间降序排序，最新的在前面
        files.sort(key=lambda x: os.path.getmtime(os.path.join(RECORDER_PATH, x)), reverse=True)
        return jsonify(files)
    except FileNotFoundError:
        return jsonify([])

@app.route('/videos/<path:filename>')
def serve_video(filename):
    """提供视频文件流"""
    return send_from_directory(RECORDER_PATH, filename)


# 3. 配置编辑
@app.route('/api/config', methods=['GET'])
def get_config():
    """获取当前配置"""
    try:
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            return jsonify(json.load(f))
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/config', methods=['POST'])
def save_config():
    """保存配置到config.json"""
    try:
        data = request.json
        with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        return jsonify({'status': 'success', 'message': '配置已成功保存'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/presets', methods=['GET'])
def get_presets():
    """获取所有预设"""
    presets = [f.replace('.json', '') for f in os.listdir(PRESETS_DIR) if f.endswith('.json')]
    return jsonify(presets)

@app.route('/api/presets', methods=['POST'])
def save_preset():
    """保存当前配置为新预设"""
    try:
        data = request.json
        preset_name = data.get('name')
        config_data = data.get('config')
        if not preset_name:
            return jsonify({'status': 'error', 'message': '预设名称不能为空'}), 400

        preset_path = os.path.join(PRESETS_DIR, f"{preset_name}.json")
        with open(preset_path, 'w', encoding='utf-8') as f:
            json.dump(config_data, f, indent=4, ensure_ascii=False)
        return jsonify({'status': 'success', 'message': f'预设 "{preset_name}" 已保存'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/presets/load', methods=['POST'])
def load_preset():
    """加载一个预设并覆盖当前config.json"""
    try:
        data = request.json
        preset_name = data.get('name')
        if not preset_name:
            return jsonify({'status': 'error', 'message': '预设名称不能为空'}), 400

        preset_path = os.path.join(PRESETS_DIR, f"{preset_name}.json")
        if not os.path.exists(preset_path):
            return jsonify({'status': 'error', 'message': f'预设 "{preset_name}" 不存在'}), 404

        with open(preset_path, 'r', encoding='utf-8') as f:
            preset_data = json.load(f)

        with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(preset_data, f, indent=4, ensure_ascii=False)

        return jsonify({'status': 'success', 'message': f'预设 "{preset_name}" 已加载并应用', 'config': preset_data})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

if __name__ == '__main__':
    # 允许局域网访问
    app.run(host='0.0.0.0', port=5000, debug=True)