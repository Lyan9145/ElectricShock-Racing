#!/usr/bin/env python3
"""
串口代理服务器
Serial Proxy Server

允许多个客户端通过TCP连接共享同一个串口设备
支持同时为C++项目和Python Web控制器提供串口访问
"""

import serial
import socket
import threading
import time
import json
import struct
import queue
import signal
import sys
from datetime import datetime
import logging
from typing import Dict, List, Optional
import argparse

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SerialProxyServer:
    def __init__(self, serial_port: str, serial_baud: int = 115200, 
                 tcp_port: int = 8899, max_clients: int = 10):
        """
        初始化串口代理服务器
        
        Args:
            serial_port: 串口设备路径
            serial_baud: 串口波特率
            tcp_port: TCP服务端口
            max_clients: 最大客户端连接数
        """
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.tcp_port = tcp_port
        self.max_clients = max_clients
        
        # 串口相关
        self.serial_conn: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        
        # TCP服务相关
        self.server_socket: Optional[socket.socket] = None
        self.clients: Dict[str, socket.socket] = {}
        self.client_threads: Dict[str, threading.Thread] = {}
        self.clients_lock = threading.Lock()
        
        # 控制标志
        self.running = False
        self.serial_thread: Optional[threading.Thread] = None
        
        # 消息队列（用于发送到串口）
        self.serial_send_queue = queue.Queue()
        
        # 统计信息
        self.stats = {
            'bytes_sent': 0,
            'bytes_received': 0,
            'clients_connected': 0,
            'start_time': None
        }

    def init_serial(self) -> bool:
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port, 
                self.serial_baud, 
                timeout=0.1
            )
            logger.info(f"串口 {self.serial_port} 已打开 (波特率: {self.serial_baud})")
            return True
        except serial.SerialException as e:
            logger.error(f"无法打开串口 {self.serial_port}: {e}")
            return False

    def init_tcp_server(self) -> bool:
        """初始化TCP服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(self.max_clients)
            logger.info(f"TCP服务器已启动，监听端口 {self.tcp_port}")
            return True
        except socket.error as e:
            logger.error(f"无法启动TCP服务器: {e}")
            return False

    def serial_reader_thread(self):
        """串口读取线程 - 将串口数据广播给所有客户端"""
        logger.info("串口读取线程已启动")
        
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        self.stats['bytes_received'] += len(data)
                        self.broadcast_to_clients(data)
                else:
                    time.sleep(0.001)  # 1ms延迟
                    
            except serial.SerialException as e:
                logger.error(f"串口读取错误: {e}")
                break
            except Exception as e:
                logger.error(f"串口读取线程异常: {e}")
                break
        
        logger.info("串口读取线程已退出")

    def serial_writer_thread(self):
        """串口写入线程 - 处理发送队列中的数据"""
        logger.info("串口写入线程已启动")
        
        while self.running:
            try:
                # 从队列获取数据，超时1秒
                data = self.serial_send_queue.get(timeout=1.0)
                
                if data is None:  # 退出信号
                    break
                
                if self.serial_conn and self.serial_conn.is_open:
                    with self.serial_lock:
                        self.serial_conn.write(data)
                        self.stats['bytes_sent'] += len(data)
                
                self.serial_send_queue.task_done()
                
            except queue.Empty:
                continue
            except serial.SerialException as e:
                logger.error(f"串口写入错误: {e}")
            except Exception as e:
                logger.error(f"串口写入线程异常: {e}")
        
        logger.info("串口写入线程已退出")

    def broadcast_to_clients(self, data: bytes):
        """将数据广播给所有连接的客户端"""
        with self.clients_lock:
            dead_clients = []
            
            for client_id, client_socket in self.clients.items():
                try:
                    client_socket.send(data)
                except (socket.error, ConnectionResetError):
                    logger.warning(f"客户端 {client_id} 连接已断开")
                    dead_clients.append(client_id)
            
            # 清理断开的客户端
            for client_id in dead_clients:
                self.remove_client(client_id)

    def handle_client(self, client_socket: socket.socket, client_address: tuple):
        """处理单个客户端连接"""
        client_id = f"{client_address[0]}:{client_address[1]}"
        logger.info(f"新客户端连接: {client_id}")
        
        try:
            while self.running:
                data = client_socket.recv(1024)
                if not data:
                    break
                
                # 将客户端数据加入串口发送队列
                self.serial_send_queue.put(data)
                
        except (socket.error, ConnectionResetError):
            logger.info(f"客户端 {client_id} 断开连接")
        except Exception as e:
            logger.error(f"处理客户端 {client_id} 时发生异常: {e}")
        finally:
            self.remove_client(client_id)
            client_socket.close()

    def remove_client(self, client_id: str):
        """移除客户端连接"""
        with self.clients_lock:
            if client_id in self.clients:
                del self.clients[client_id]
                self.stats['clients_connected'] = len(self.clients)
                logger.info(f"已移除客户端 {client_id}，当前连接数: {self.stats['clients_connected']}")

    def accept_clients_thread(self):
        """接受客户端连接的线程"""
        logger.info("客户端接受线程已启动")
        
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                client_socket, client_address = self.server_socket.accept()
                
                client_id = f"{client_address[0]}:{client_address[1]}"
                
                with self.clients_lock:
                    if len(self.clients) >= self.max_clients:
                        logger.warning(f"已达到最大客户端数量，拒绝连接: {client_id}")
                        client_socket.close()
                        continue
                    
                    self.clients[client_id] = client_socket
                    self.stats['clients_connected'] = len(self.clients)
                
                # 为每个客户端创建处理线程
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address),
                    daemon=True
                )
                client_thread.start()
                self.client_threads[client_id] = client_thread
                
            except socket.timeout:
                continue
            except socket.error as e:
                if self.running:
                    logger.error(f"接受客户端连接时发生错误: {e}")
                break
            except Exception as e:
                logger.error(f"客户端接受线程异常: {e}")
                break
        
        logger.info("客户端接受线程已退出")

    def start(self) -> bool:
        """启动代理服务器"""
        logger.info("正在启动串口代理服务器...")
        
        # 初始化串口
        if not self.init_serial():
            return False
        
        # 初始化TCP服务器
        if not self.init_tcp_server():
            self.serial_conn.close()
            return False
        
        self.running = True
        self.stats['start_time'] = datetime.now()
        
        # 启动串口处理线程
        serial_read_thread = threading.Thread(target=self.serial_reader_thread, daemon=True)
        serial_write_thread = threading.Thread(target=self.serial_writer_thread, daemon=True)
        
        serial_read_thread.start()
        serial_write_thread.start()
        
        # 启动客户端接受线程
        accept_thread = threading.Thread(target=self.accept_clients_thread, daemon=True)
        accept_thread.start()
        
        logger.info(f"串口代理服务器已启动")
        logger.info(f"串口设备: {self.serial_port} @ {self.serial_baud} bps")
        logger.info(f"TCP服务端口: {self.tcp_port}")
        logger.info(f"最大客户端数: {self.max_clients}")
        
        return True

    def stop(self):
        """停止代理服务器"""
        logger.info("正在停止串口代理服务器...")
        
        self.running = False
        
        # 发送退出信号给串口写入线程
        self.serial_send_queue.put(None)
        
        # 关闭所有客户端连接
        with self.clients_lock:
            for client_socket in self.clients.values():
                client_socket.close()
            self.clients.clear()
        
        # 关闭TCP服务器
        if self.server_socket:
            self.server_socket.close()
        
        # 关闭串口
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        logger.info("串口代理服务器已停止")

    def get_stats(self) -> dict:
        """获取统计信息"""
        uptime = None
        if self.stats['start_time']:
            uptime = datetime.now() - self.stats['start_time']
        
        return {
            'running': self.running,
            'serial_port': self.serial_port,
            'serial_baud': self.serial_baud,
            'tcp_port': self.tcp_port,
            'clients_connected': self.stats['clients_connected'],
            'bytes_sent': self.stats['bytes_sent'],
            'bytes_received': self.stats['bytes_received'],
            'uptime_seconds': uptime.total_seconds() if uptime else 0
        }

    def print_stats(self):
        """打印统计信息"""
        stats = self.get_stats()
        print("\n" + "="*50)
        print("串口代理服务器统计信息")
        print("="*50)
        print(f"运行状态: {'运行中' if stats['running'] else '已停止'}")
        print(f"串口设备: {stats['serial_port']} @ {stats['serial_baud']} bps")
        print(f"TCP端口: {stats['tcp_port']}")
        print(f"连接客户端数: {stats['clients_connected']}")
        print(f"发送字节数: {stats['bytes_sent']}")
        print(f"接收字节数: {stats['bytes_received']}")
        print(f"运行时间: {stats['uptime_seconds']:.1f} 秒")
        print("="*50)


def signal_handler(signum, frame):
    """信号处理器"""
    logger.info("收到退出信号，正在停止服务器...")
    if 'server' in globals():
        server.stop()
    sys.exit(0)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='串口代理服务器')
    parser.add_argument('--serial-port', default='/dev/ttyUSB0',
                       help='串口设备路径 (默认: /dev/ttyUSB0)')
    parser.add_argument('--serial-baud', type=int, default=115200,
                       help='串口波特率 (默认: 115200)')
    parser.add_argument('--tcp-port', type=int, default=8899,
                       help='TCP服务端口 (默认: 8899)')
    parser.add_argument('--max-clients', type=int, default=10,
                       help='最大客户端连接数 (默认: 10)')
    parser.add_argument('--stats-interval', type=int, default=30,
                       help='统计信息显示间隔(秒) (默认: 30, 0=关闭)')
    
    args = parser.parse_args()
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 创建服务器实例
    global server
    server = SerialProxyServer(
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        tcp_port=args.tcp_port,
        max_clients=args.max_clients
    )
    
    # 启动服务器
    if not server.start():
        logger.error("无法启动串口代理服务器")
        return 1
    
    try:
        # 主循环 - 定期显示统计信息
        last_stats_time = time.time()
        
        while True:
            time.sleep(1)
            
            if args.stats_interval > 0:
                current_time = time.time()
                if current_time - last_stats_time >= args.stats_interval:
                    server.print_stats()
                    last_stats_time = current_time
                    
    except KeyboardInterrupt:
        logger.info("收到键盘中断信号")
    finally:
        server.stop()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
