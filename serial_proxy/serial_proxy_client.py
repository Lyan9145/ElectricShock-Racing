#!/usr/bin/env python3
"""
串口代理客户端 - Python版本
Serial Proxy Client for Python

用于连接到串口代理服务器的Python客户端库
可以替代直接的串口连接，通过TCP与代理服务器通信
"""

import socket
import threading
import time
import queue
import logging
from typing import Optional, Callable
import struct

logger = logging.getLogger(__name__)

class SerialProxyClient:
    """串口代理客户端类"""
    
    def __init__(self, proxy_host: str = 'localhost', proxy_port: int = 8899):
        """
        初始化串口代理客户端
        
        Args:
            proxy_host: 代理服务器主机地址
            proxy_port: 代理服务器端口
        """
        self.proxy_host = proxy_host
        self.proxy_port = proxy_port
        
        # 网络连接
        self.socket: Optional[socket.socket] = None
        self.connected = False
        
        # 线程控制
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        
        # 接收队列和回调
        self.receive_queue = queue.Queue()
        self.receive_callback: Optional[Callable[[bytes], None]] = None
        
        # 重连设置
        self.auto_reconnect = True
        self.reconnect_interval = 5.0
        
        # 统计信息
        self.bytes_sent = 0
        self.bytes_received = 0

    def connect(self, timeout: float = 5.0) -> bool:
        """
        连接到代理服务器
        
        Args:
            timeout: 连接超时时间(秒)
            
        Returns:
            bool: 连接是否成功
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.proxy_host, self.proxy_port))
            
            self.connected = True
            self.running = True
            
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_thread, daemon=True)
            self.receive_thread.start()
            
            logger.info(f"已连接到串口代理服务器 {self.proxy_host}:{self.proxy_port}")
            return True
            
        except socket.error as e:
            logger.error(f"连接代理服务器失败: {e}")
            self.connected = False
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    def disconnect(self):
        """断开与代理服务器的连接"""
        self.running = False
        self.connected = False
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        
        logger.info("已断开与串口代理服务器的连接")

    def write(self, data: bytes) -> bool:
        """
        发送数据到串口（通过代理服务器）
        
        Args:
            data: 要发送的字节数据
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected or not self.socket:
            logger.warning("未连接到代理服务器，无法发送数据")
            return False
        
        try:
            self.socket.send(data)
            self.bytes_sent += len(data)
            return True
        except socket.error as e:
            logger.error(f"发送数据失败: {e}")
            self._handle_disconnect()
            return False

    def read(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        从接收队列读取数据
        
        Args:
            timeout: 读取超时时间(秒)
            
        Returns:
            bytes or None: 接收到的数据，超时则返回None
        """
        try:
            return self.receive_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def set_receive_callback(self, callback: Callable[[bytes], None]):
        """
        设置数据接收回调函数
        
        Args:
            callback: 回调函数，参数为接收到的字节数据
        """
        self.receive_callback = callback

    def _receive_thread(self):
        """接收数据的线程"""
        logger.info("串口代理客户端接收线程已启动")
        
        while self.running:
            try:
                if not self.connected:
                    if self.auto_reconnect:
                        logger.info("尝试重新连接到代理服务器...")
                        if self.connect():
                            continue
                        else:
                            time.sleep(self.reconnect_interval)
                            continue
                    else:
                        break
                
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024)
                
                if not data:
                    logger.warning("代理服务器连接已断开")
                    self._handle_disconnect()
                    continue
                
                self.bytes_received += len(data)
                
                # 将数据放入队列
                self.receive_queue.put(data)
                
                # 调用回调函数
                if self.receive_callback:
                    try:
                        self.receive_callback(data)
                    except Exception as e:
                        logger.error(f"接收回调函数异常: {e}")
                
            except socket.timeout:
                continue
            except socket.error as e:
                logger.error(f"接收数据时发生错误: {e}")
                self._handle_disconnect()
            except Exception as e:
                logger.error(f"接收线程异常: {e}")
                break
        
        logger.info("串口代理客户端接收线程已退出")

    def _handle_disconnect(self):
        """处理连接断开"""
        self.connected = False
        if self.socket:
            self.socket.close()
            self.socket = None

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.connected

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            'connected': self.connected,
            'proxy_host': self.proxy_host,
            'proxy_port': self.proxy_port,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'auto_reconnect': self.auto_reconnect
        }


class SerialProxyAdapter:
    """
    串口代理适配器
    提供与标准serial.Serial类似的接口
    """
    
    def __init__(self, proxy_host: str = 'localhost', proxy_port: int = 8899):
        """
        初始化串口代理适配器
        
        Args:
            proxy_host: 代理服务器主机地址
            proxy_port: 代理服务器端口
        """
        self.client = SerialProxyClient(proxy_host, proxy_port)
        self._is_open = False

    def open(self) -> bool:
        """打开连接"""
        if self.client.connect():
            self._is_open = True
            return True
        return False

    def close(self):
        """关闭连接"""
        self.client.disconnect()
        self._is_open = False

    def write(self, data: bytes) -> int:
        """写入数据"""
        if self.client.write(data):
            return len(data)
        return 0

    def read(self, size: int = 1, timeout: float = 1.0) -> bytes:
        """读取数据"""
        data = self.client.read(timeout)
        if data is None:
            return b''
        return data[:size] if len(data) > size else data

    @property
    def is_open(self) -> bool:
        """检查是否打开"""
        return self._is_open and self.client.is_connected()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# 用于替换原有串口通信的兼容性函数
def create_car_control_adapter(proxy_host: str = 'localhost', proxy_port: int = 8899):
    """
    创建一个可以直接替换原有串口通信的适配器
    
    Args:
        proxy_host: 代理服务器主机地址
        proxy_port: 代理服务器端口
        
    Returns:
        SerialProxyAdapter: 串口代理适配器实例
    """
    return SerialProxyAdapter(proxy_host, proxy_port)


# USB通信帧常量（与原代码保持一致）
USB_FRAME_HEAD = 0x42
USB_ADDR_CARCTRL = 1

def send_car_control_command_via_proxy(proxy_adapter: SerialProxyAdapter, 
                                     speed: float, servo: int) -> bool:
    """
    通过代理发送车辆控制指令
    
    Args:
        proxy_adapter: 串口代理适配器
        speed: 速度值
        servo: 舵机值
        
    Returns:
        bool: 发送是否成功
    """
    if not proxy_adapter.is_open:
        return False

    speed = float(speed)
    servo = int(servo)
    servo = max(0, min(65535, servo))  # 限制到uint16范围

    frame_len = 10
    command_buffer = bytearray(frame_len)
    command_buffer[0] = USB_FRAME_HEAD
    command_buffer[1] = USB_ADDR_CARCTRL
    command_buffer[2] = frame_len
    struct.pack_into("<f", command_buffer, 3, speed)
    struct.pack_into("<H", command_buffer, 7, servo)
    checksum = sum(command_buffer[:frame_len-1]) % 256
    command_buffer[frame_len - 1] = checksum

    return proxy_adapter.write(command_buffer) == frame_len
