#!/usr/bin/env python3
import os
import pty
import serial
import threading
import queue
import time
import select
import argparse
import logging
import signal

# --- 配置 ---
# 设置日志格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class UartProxy:
    """
    UART代理主类，用于管理物理和虚拟串口。
    """
    def __init__(self, physical_device, baudrate, num_virtual_ports, virtual_port_prefix):
        self.physical_device = physical_device
        self.baudrate = baudrate
        self.num_virtual_ports = num_virtual_ports
        self.virtual_port_prefix = virtual_port_prefix

        # 线程安全的发送队列
        self.send_queue = queue.Queue()
        # 用于通知所有线程退出的事件
        self.shutdown_event = threading.Event()
        # 存储所有虚拟端口处理器的列表
        self.virtual_clients = []
        # 存储所有线程的列表，以便于管理
        self.threads = []
        self.symlinks = []

    def _cleanup(self, *args):
        """
        优雅地关闭所有资源。
        """
        if self.shutdown_event.is_set():
            return
            
        logging.info("捕获到关闭信号，正在清理资源...")
        self.shutdown_event.set()

        # 等待所有线程结束
        for t in self.threads:
            # 增加超时以防线程卡死
            t.join(timeout=2)

        # 删除创建的符号链接
        for link_path in self.symlinks:
            try:
                if os.path.islink(link_path):
                    os.remove(link_path)
                    logging.info(f"已删除符号链接: {link_path}")
            except OSError as e:
                logging.error(f"删除符号链接 {link_path} 失败: {e}")
        
        logging.info("代理已成功关闭。")

    def start(self):
        """
        启动代理服务。
        """
        # 注册信号处理器以实现优雅关闭
        signal.signal(signal.SIGINT, self._cleanup)
        signal.signal(signal.SIGTERM, self._cleanup)

        # --- 1. 创建虚拟串口 ---
        logging.info(f"正在创建 {self.num_virtual_ports} 个虚拟串口...")
        for i in range(self.num_virtual_ports):
            try:
                master_fd, slave_fd = pty.openpty()
                slave_name = os.ttyname(slave_fd)
                
                # 为虚拟串口设置一个易于识别的名称（通过符号链接）
                link_path = f"/dev/{self.virtual_port_prefix}{i}"
                if os.path.exists(link_path):
                    os.remove(link_path)
                os.symlink(slave_name, link_path)
                self.symlinks.append(link_path)
                
                # 赋予所有用户读写权限
                os.chmod(slave_name, 0o666)
                
                logging.info(f"已创建虚拟串口: {slave_name} -> {link_path}")
                
                # 为每个虚拟串口创建一个处理器
                client = VirtualPortHandler(master_fd, slave_name, self.send_queue, self.shutdown_event)
                self.virtual_clients.append(client)
                
                # 启动对应的线程
                thread = threading.Thread(target=client.run, daemon=True)
                thread.start()
                self.threads.append(thread)

            except Exception as e:
                logging.error(f"创建虚拟串口 {i} 失败: {e}")
                self._cleanup()
                return

        # --- 2. 启动物理串口处理器 ---
        try:
            physical_handler = PhysicalPortHandler(
                self.physical_device,
                self.baudrate,
                self.send_queue,
                self.virtual_clients,
                self.shutdown_event
            )
            physical_thread = threading.Thread(target=physical_handler.run, daemon=True)
            physical_thread.start()
            self.threads.append(physical_thread)
            
            logging.info(f"代理已启动。物理端口: {self.physical_device}@{self.baudrate}. 虚拟端口: /dev/{self.virtual_port_prefix}[0-{self.num_virtual_ports-1}]")
            logging.info("按 Ctrl+C 停止代理。")
            
            # 主线程保持运行，等待关闭信号
            physical_thread.join()

        except serial.SerialException as e:
            logging.error(f"无法打开物理串口 {self.physical_device}: {e}")
            self._cleanup()
        except Exception as e:
            logging.error(f"启动物理端口处理器时发生未知错误: {e}")
            self._cleanup()
        finally:
            # 确保即使物理端口线程异常退出，也能执行清理
            self._cleanup()

    def toggle_forwarding(self, virtual_port_index, enable):
        """
        切换虚拟端口是否转发到物理端口。
        """
        if 0 <= virtual_port_index < len(self.virtual_clients):
            self.virtual_clients[virtual_port_index].set_forwarding(enable)
            logging.info(f"虚拟端口 {virtual_port_index} 转发 {'启用' if enable else '禁用'}")
        else:
            logging.warning(f"无效的虚拟端口索引: {virtual_port_index}")

class PhysicalPortHandler:
    """
    处理物理串口的读写。
    """
    def __init__(self, device, baudrate, send_queue, clients, shutdown_event):
        self.device = device
        self.baudrate = baudrate
        self.send_queue = send_queue
        self.clients = clients
        self.shutdown_event = shutdown_event
        self.ser = None

    def run(self):
        logging.info(f"物理端口线程启动，正在连接 {self.device}...")
        try:
            self.ser = serial.Serial(self.device, self.baudrate, timeout=0.1)
        except serial.SerialException as e:
            logging.error(f"物理端口打开失败: {e}")
            self.shutdown_event.set() # 通知其他线程退出
            return

        while not self.shutdown_event.is_set():
            # 1. 从物理串口读取数据并广播给所有虚拟客户端
            try:
                if self.ser.in_waiting > 0:
                    data_from_physical = self.ser.read(self.ser.in_waiting)
                    if data_from_physical:
                        logging.debug(f"从物理端口收到: {data_from_physical!r}")
                        for client in self.clients:
                            client.write_to_virtual(data_from_physical)
            except OSError as e:
                logging.error(f"读取物理串口时出错: {e}")
                break # 出现IO错误，很可能设备已断开，退出线程

            # 2. 从队列获取数据并写入物理串口
            try:
                data_to_physical = self.send_queue.get_nowait()
                self.ser.write(data_to_physical)
                logging.debug(f"向物理端口发送: {data_to_physical!r}")
            except queue.Empty:
                # 队列为空是正常情况，无需处理
                pass
            except Exception as e:
                logging.error(f"写入物理串口时出错: {e}")
                
        if self.ser and self.ser.is_open:
            self.ser.close()
        logging.info("物理端口线程已停止。")


class VirtualPortHandler:
    """
    处理单个虚拟串口的读写。
    """
    def __init__(self, master_fd, slave_name, send_queue, shutdown_event):
        self.master_fd = master_fd
        self.slave_name = slave_name
        self.send_queue = send_queue
        self.shutdown_event = shutdown_event
        self.forwarding_enabled = True

    def set_forwarding(self, enable):
        """设置是否启用转发到物理端口。"""
        self.forwarding_enabled = enable

    def run(self):
        logging.info(f"虚拟端口线程启动: {self.slave_name}")
        while not self.shutdown_event.is_set():
            try:
                # 使用select监听master_fd是否可读，超时0.1秒
                # 这可以防止线程在os.read上永久阻塞
                r, _, _ = select.select([self.master_fd], [], [], 0.1)
                if r:
                    data_from_virtual = os.read(self.master_fd, 1024)
                    if data_from_virtual:
                        logging.debug(f"从 {self.slave_name} 收到: {data_from_virtual!r}")
                        if self.forwarding_enabled:
                            self.send_queue.put(data_from_virtual)
            except OSError as e:
                logging.error(f"虚拟端口 {self.slave_name} 发生错误: {e}")
                break # 出现IO错误，退出线程

        os.close(self.master_fd)
        logging.info(f"虚拟端口线程已停止: {self.slave_name}")

    def write_to_virtual(self, data):
        """由物理端口线程调用，将数据写入此虚拟端口。"""
        try:
            os.write(self.master_fd, data)
        except OSError as e:
            logging.warning(f"向虚拟端口 {self.slave_name} 写入失败 (可能客户端已关闭): {e}")

def main():
    parser = argparse.ArgumentParser(description="Linux UART串口虚拟设备代理")
    parser.add_argument('physical_port', type=str, help="物理串口设备路径, 例如 /dev/ttyUSB0")
    parser.add_argument('--baud', '-b', type=int, default=115200, help="物理串口的波特率 (默认: 115200)")
    parser.add_argument('--num-ports', '-n', type=int, default=2, help="要创建的虚拟串口数量 (默认: 2)")
    parser.add_argument('--prefix', '-p', type=str, default="ttyPX", help="虚拟串口设备名称前缀 (默认: ttyPX)")
    
    args = parser.parse_args()
    
    if os.geteuid() != 0:
        logging.warning("警告: 此脚本最好以root权限运行，以便在/dev下创建符号链接和修改权限。")
        logging.warning("否则，你可能需要手动创建符号链接并设置权限。")

    proxy = UartProxy(
        physical_device=args.physical_port,
        baudrate=args.baud,
        num_virtual_ports=args.num_ports,
        virtual_port_prefix=args.prefix
    )
    proxy.start()

if __name__ == '__main__':
    main()