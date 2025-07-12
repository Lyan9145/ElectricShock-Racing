#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

#include "../include/common.hpp"        // 公共类方法文件
#include "../include/detection.hpp"     // 百度Paddle框架移动端部署
#include "../include/uart.hpp"          // 串口通信驱动
#include "../include/motion.hpp"        // 运动控制类
#include "../include/thread.hpp"        // 线程管理
#include "../include/controlcenter.hpp" // 控制中心
#include "../include/tracking.hpp"      // 跟踪算法
#include "../include/bridge.hpp"
#include "../include/catering.hpp"
#include "../include/crosswalk.hpp"
#include "../include/layby.hpp"
#include "../include/obstacle.hpp"
#include "../include/parking.hpp"
#include "../include/crossroad.hpp"
#include "../include/ring.hpp"

using namespace std;
using namespace cv;

Display display; // 初始化UI显示窗口

int main(int argc, char const *argv[])
{
    // 初始化配置文件
    Motion motion;

    // USB转串口初始化： /dev/ttyUSB0
    // 检测/dev/ttyPX0是否存在
    const char *ttyPX0 = "/dev/ttyPX1";
    const char *ttyUSB0 = "/dev/ttyUSB0";
    shared_ptr<Uart> uart = nullptr;
    if (access(ttyPX0, F_OK) == 0)
    {
        uart = make_shared<Uart>(ttyPX0); // 切换到/dev/ttyPX0
        std::cout << "[INFO] Using " << ttyPX0 << " for UART communication." << std::endl;
    }
    else if (access(ttyUSB0, F_OK) == 0)
    {
        uart = make_shared<Uart>(ttyUSB0); // 切换到/dev/ttyUSB0
        std::cout << "[INFO] Using " << ttyUSB0 << " for UART communication." << std::endl;
    }
    else
    {
        std::cerr << "[Error] Neither " << ttyPX0 << " nor " << ttyUSB0 << " exists." << std::endl;
        return -1;
    }

    // 设置全局uart指针用于信号处理
    g_uart = uart;

    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    int ret = uart->open();
    if (ret != 0)
    {
        printf("[Error] Uart Open failed!\n");
        exit(-1);
    }
    // uart->startReceive();             // 启动数据接收子线程
    uart->carControl(0, PWMSERVOMID); // 停车舵机中位

    // USB摄像头初始化
    // 构造 GStreamer 管道字符串，明确指定 MJPG 格式、分辨率和帧率
    // 使用 std::to_string 将 int 转换为字符串，以确保兼容性
    std::string gstreamer_pipeline =
        "v4l2src device=/dev/video0 ! "
        "image/jpeg,width=" +
        std::to_string(COLSIMAGE_CAM) +
        ",height=" + std::to_string(ROWSIMAGE_CAM) +
        ",framerate=60/1 ! " // 60/1 表示 60fps
        // "queue ! " // 在解码前加队列
        "jpegdec ! "
        // "queue ! " // 在转换前加队列
        "videoconvert ! "    // 转换为 OpenCV 兼容的颜色空间 (如 RGB 或 BGR)
        // "videoscale ! video/x-raw,width=640,height=480 ! "
        "appsink drop=true max-buffers=1";           // OpenCV 通过 appsink 从 GStreamer 管道获取帧

    // 使用 GStreamer 管道和 cv::CAP_GSTREAMER 标志初始化 VideoCapture
    cv::VideoCapture capture;
    capture = cv::VideoCapture(gstreamer_pipeline, cv::CAP_GSTREAMER);
    if (!capture.isOpened())
    {
        std::cerr << "[Error] Could not open video capture with GStreamer pipeline." << std::endl;
        return -1;
    }

    Factory<TaskData> task_factory(3);    // 任务数据工厂
    Factory<AIData> AI_task_factory(3); // AI任务数据工厂
    Factory<DebugData> debug_factory(5);
    std::vector<PredictResult> predict_result;
    std::mutex predict_result_lock; // 保护预测结果的互斥锁

    std::thread task_producer(&producer, std::ref(task_factory), std::ref(AI_task_factory), std::ref(capture));
    std::thread AI_consumer(&AIConsumer, std::ref(AI_task_factory), std::ref(predict_result), std::ref(predict_result_lock), std::ref(motion));
    std::thread task_consumer(&consumer, std::ref(task_factory), std::ref(debug_factory), std::ref(predict_result), std::ref(predict_result_lock), std::ref(motion), std::ref(*uart));
    std::thread debug_data_consumer(&debugDataConsumer, std::ref(debug_factory));
    
    UartStatus status = uart->getStatus();
    std::cout << "[INFO] Battery voltage: " << status.voltage << std::endl;
    sleep(3);
    
    task_producer.join();
    task_consumer.join();
    AI_consumer.join();
    debug_data_consumer.join();
    
    std::cout << "[INFO] All threads have finished." << std::endl;
    status = uart->getStatus();
    std::cout << "[INFO] Battery voltage: " << status.voltage << std::endl;

    // 可在合适位置读取串口状态，例如：
    // UartStatus status = uart->getStatus();
    // printf("距离: %.3f m, 电压: %.2f V, 速度: %.3f m/s\n", status.distance, status.voltage, status.speed);

    uart->close(); // 串口通信关闭 去掉接收线程
    capture.release();
    return 0;
}
