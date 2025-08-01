#include "../include/uart.hpp"
#include <regex>
#include <string>
#include <sstream>

using namespace LibSerial;
using namespace std;

/**
 * @brief 串口接收字节数据
 *
 * @param charBuffer
 * @param msTimeout
 * @return int
 */
int Uart::receiveBytes(unsigned char &charBuffer, size_t msTimeout)
{
    /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
    如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
    try
    {
        /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
        如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
        serialPort->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
    }
    catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
    {
        // std::cerr << "The ReadByte() call has timed out." << std::endl;
        return -2;
    }
    catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
    {
        std::cerr << "Port Not Open ..." << std::endl;
        return -1;
    }
    return 0;
};

/**
 * @brief
 *
 * @param data
 * @return int
 */
int Uart::transmitByte(unsigned char data)
{
    // try检测语句块有没有异常
    try
    {
        serialPort->WriteByte(data); // 写数据到串口
    }
    catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
    {
        std::cerr << "The Write() runtime_error." << std::endl;
        return -2;
    }
    catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
    {
        std::cerr << "Port Not Open ..." << std::endl;
        return -1;
    }
    serialPort->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
    return 0;
}

int Uart::open(void)
{
    serialPort = std::make_shared<SerialPort>();
    if (serialPort == nullptr)
    {
        std::cerr << "Serial Create Failed ." << std::endl;
        return -1;
    }
    // try检测语句块有没有异常
    try
    {
        serialPort->Open(portName);                                 // 打开串口
        serialPort->SetBaudRate(BaudRate::BAUD_115200);             // 设置波特率
        serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
        serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
        serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
        serialPort->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位
    }
    catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
    {
        std::cerr << "Serial port: " << portName << "open failed ..."
                  << std::endl;
        isOpen = false;
        return -2;
    }
    catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
    {
        std::cerr << "Serial port: " << portName << "open failed ..."
                  << std::endl;
        isOpen = false;
        return -3;
    }
    catch (...) // catch捕获并处理 try 检测到的异常。
    {
        std::cerr << "Serial port: " << portName << " recv exception ..."
                  << std::endl;
        isOpen = false;
        return -4;
    }

    serialStr.start = false;
    serialStr.index = 0;
    isOpen = true;

    // 新增：启动行接收线程
    startLineReceive();

    return 0;
}

/**
 * @brief 启动接收子线程
 *
 */
void Uart::startReceive(void)
{
    if (!isOpen) // 串口是否正常打开
        return;

    // 启动串口接收子线程
    threadRec = std::make_unique<std::thread>([this]()
                                              {
      while (1) {
        receiveCheck(); // 串口接收校验
      } });
}

/**
 * @brief 关闭串口通信
 *
 */
void Uart::close(void)
{
    // printf(" uart thread exit!\n");
    carControl(0, PWMSERVOMID);
    stopLineReceive(); // 新增：关闭行接收线程
    // threadRec->join();
    if (serialPort != nullptr)
    {
        serialPort->Close();
        serialPort = nullptr;
    }
    isOpen = false;
}

/**
 * @brief 串口接收校验
 *
 */
void Uart::receiveCheck(void)
{
    if (!isOpen) // 串口是否正常打开
        return;

    uint8_t resByte = 0;
    int ret = receiveBytes(resByte, 0);
    if (ret == 0)
    {
        if (resByte == USB_FRAME_HEAD && !serialStr.start) // 监听帧头
        {
            serialStr.start = true;                   // 开始接收数据
            serialStr.buffRead[0] = resByte;          // 获取帧头
            serialStr.buffRead[2] = USB_FRAME_LENMIN; // 初始化帧长
            serialStr.index = 1;
        }
        else if (serialStr.index == 2) // 接收帧的长度
        {
            serialStr.buffRead[serialStr.index] = resByte;
            serialStr.index++;
            if (resByte > USB_FRAME_LENMAX ||
                resByte < USB_FRAME_LENMIN) // 帧长错误
            {
                serialStr.buffRead[2] = USB_FRAME_LENMIN; // 重置帧长
                serialStr.index = 0;
                serialStr.start = false; // 重新监听帧长
            }
        }
        else if (serialStr.start &&
                 serialStr.index < USB_FRAME_LENMAX) // 开始接收数据
        {
            serialStr.buffRead[serialStr.index] = resByte; // 读取数据
            serialStr.index++;                             // 索引下移
        }

        // 帧长接收完毕
        if ((serialStr.index >= USB_FRAME_LENMAX ||
             serialStr.index >= serialStr.buffRead[2]) &&
            serialStr.index > USB_FRAME_LENMIN) // 检测是否接收完数据
        {
            uint8_t check = 0; // 初始化校验和
            uint8_t length = USB_FRAME_LENMIN;
            length = serialStr.buffRead[2]; // 读取本次数据的长度
            for (int i = 0; i < length - 1; i++)
                check += serialStr.buffRead[i]; // 累加校验和

            if (check == serialStr.buffRead[length - 1]) // 校验和相等
            {
                memcpy(serialStr.buffFinish, serialStr.buffRead,
                       USB_FRAME_LENMAX); // 储存接收的数据
                dataTransform();
            }

            serialStr.index = 0;     // 重新开始下一轮数据接收
            serialStr.start = false; // 重新监听帧头
        }
    }
}

/**
 * @brief 串口通信协议数据转换
 */
void Uart::dataTransform(void)
{
    switch (serialStr.buffFinish[1])
    {
    case USB_ADDR_KEY: // 接收按键信息
        keypress = true;
        break;

    default:
        break;
    }
}

/**
 * @brief 速度+方向控制
 *
 * @param speed 速度：m/s
 * @param servo 方向：PWM（4000-6000）
 */
void Uart::carControl(float speed, uint16_t servo)
{
    if (!isOpen)
        return;

    uint8_t buff[11];  // 多发送一个字节
    uint8_t check = 0; // 校验位
    Bit32Union bit32U;
    Bit16Union bit16U;

    buff[0] = USB_FRAME_HEAD;   // 通信帧头
    buff[1] = USB_ADDR_CARCTRL; // 地址
    buff[2] = 10;               // 帧长

    // if (speed > 0.0)
    //     speed = 0.7f; // 暂时强制设为0.2，速度反馈暂未实现
    // else if (speed < 0.0)
    //     speed = -0.7f; // 暂时强制设为-0.2，速度反馈暂未实现
    // else
    //     speed = 0.0f; // 暂时强制设为0.0，速度反馈暂未实现

    // cout << "Car Control: Speed = " << speed
    //      << ", Servo = " << servo << endl; // 调试输出

    bit32U.float32 = speed; // X轴线速度
    for (int i = 0; i < 4; i++)
        buff[i + 3] = bit32U.buff[i];

    // 反转（4000-6000）->（6000-4000）
    bit16U.uint16 = 2 * PWMSERVOMID - servo; // Y轴线速度
    buff[7] = bit16U.buff[0];
    buff[8] = bit16U.buff[1];

    for (int i = 0; i < 9; i++)
        check += buff[i];
    buff[9] = check; // 校验位

    // 循环发送数据
    for (size_t i = 0; i < 11; i++)
        transmitByte(buff[i]);
}

/**
 * @brief 蜂鸣器音效控制
 *
 * @param sound
 */
void Uart::buzzerSound(Buzzer sound)
{
    if (!isOpen)
        return;
    uint8_t buff[6];   // 多发送一个字节
    uint8_t check = 0; // 校验位

    buff[0] = USB_FRAME_HEAD;  // 帧头
    buff[1] = USB_ADDR_BUZZER; // 地址
    buff[2] = 5;               // 帧长
    switch (sound)
    {
    case Buzzer::BUZZER_OK: // 确认
        buff[3] = 1;
        break;
    case Buzzer::BUZZER_WARNNING: // 报警
        buff[3] = 2;
        break;
    case Buzzer::BUZZER_FINISH: // 完成
        buff[3] = 3;
        break;
    case Buzzer::BUZZER_DING: // 提示
        buff[3] = 4;
        break;
    case Buzzer::BUZZER_START: // 开机
        buff[3] = 5;
        break;
    }

    for (size_t i = 0; i < 4; i++)
        check += buff[i];
    buff[4] = check;

    // 循环发送数据
    for (size_t i = 0; i < 6; i++)
        transmitByte(buff[i]);
}

void Uart::lineReceiveThread()
{
    std::string buffer;
    std::regex pattern(R"(D:(-?\d+\.\d+)\s*,\s*V:(-?\d+\.\d+)\s*,\s*S:(-?\d+\.\d+))");
    while (!stopLineRecv && isOpen && serialPort != nullptr)
    {
        try
        {
            char c;
            // 逐字节读取，直到遇到换行
            while (serialPort->IsOpen() && !stopLineRecv)
            {
                serialPort->ReadByte(c, 10); // 10ms超时
                if (c == '\n' || c == '\r')
                {
                    if (!buffer.empty())
                    {
                        parseLine(buffer);
                        buffer.clear();
                    }
                }
                else
                {
                    buffer += c;
                    // 防止异常长行
                    if (buffer.size() > 128)
                        buffer.clear();
                }
            }
        }
        catch (...)
        {
            // 读取异常，短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void Uart::parseLine(const std::string &line)
{
    static std::regex pattern(R"(D:(-?\d+\.\d{3})\s*,\s*V:(-?\d+\.\d{2})\s*,\s*S:(-?\d+\.\d{3})\s*)");
    std::smatch m;
    if (std::regex_match(line, m, pattern))
    {
        UartStatus new_status;
        new_status.distance = std::stof(m[1]);
        new_status.voltage = std::stof(m[2]);
        new_status.speed = std::stof(m[3]);
        std::lock_guard<std::mutex> lock(status_mutex);
        status = new_status;
    }
    else
    {
        // 解析失败，可能是格式不正确或数据异常
        std::cerr << "[ERROR] Failed to parse line: " << line << std::endl;
    }
    // 可选：else分支可记录无法解析的数据
}

UartStatus Uart::getStatus()
{
    std::lock_guard<std::mutex> lock(status_mutex);
    return status;
}

void Uart::startLineReceive()
{
    stopLineRecv = false;
    if (!threadLineRecv)
    {
        threadLineRecv = std::make_unique<std::thread>(&Uart::lineReceiveThread, this);
    }
}

void Uart::stopLineReceive()
{
    stopLineRecv = true;
    if (threadLineRecv && threadLineRecv->joinable())
    {
        threadLineRecv->join();
        threadLineRecv.reset();
    }
}
