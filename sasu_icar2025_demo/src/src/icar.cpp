/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */

#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/obstacle.cpp"    //AI检测：障碍区
#include "detection/catering.cpp"    //AI检测：餐饮区
#include "detection/layby.cpp"       //AI检测：临时停车区
#include "detection/parking.cpp"     //AI检测：充电停车场
#include "detection/crosswalk.cpp"   //AI检测：停车区
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

void mouseCallback(int event, int x, int y, int flags, void *userdata);
Display display; // 初始化UI显示窗口

// 全局变量用于信号处理
shared_ptr<Uart> g_uart = nullptr;
volatile sig_atomic_t g_exit_flag = 0;
    


// 图像信息显示函数
void displayImageInfo(const Mat& img, long preTime) {
  static int frameCount = 0;
  static auto lastTime = chrono::high_resolution_clock::now();
  frameCount++;
  
  auto currentTime = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::milliseconds>(currentTime - lastTime);
  
  if (duration.count() >= 1000) { // 每秒更新一次
    double fps = frameCount * 1000.0 / duration.count();
    printf("Resolution: %dx%d | FPS: %.2f\n", img.cols, img.rows, fps);
    frameCount = 0;
    lastTime = currentTime;
  }
}


int main(int argc, char const *argv[]) {

  // USB转串口初始化： /dev/ttyUSB0
  shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  
  // 设置全局uart指针用于信号处理
  g_uart = uart;
  
  // 注册信号处理函数
  signal(SIGINT, signalHandler);
  
  int ret = uart->open();
  if (ret != 0) {
    printf("[Error] Uart Open failed!\n");
    exit(-1);
  }
  uart->startReceive(); // 启动数据接收子线程
  uart->carControl(0, PWMSERVOMID); // 停车舵机中位

  // USB摄像头初始化
  if (motion.params.debug) {
      // 如果是调试模式，仍然打开本地视频文件
      capture = cv::VideoCapture(motion.params.video);
  } else {
      // 构造 GStreamer 管道字符串，明确指定 MJPG 格式、分辨率和帧率
      // 使用 std::to_string 将 int 转换为字符串，以确保兼容性
      std::string gstreamer_pipeline =
          "v4l2src device=/dev/video0 ! "
          "image/jpeg,width=" + std::to_string(COLSIMAGE_CAM) +
          ",height=" + std::to_string(ROWSIMAGE_CAM) +
          ",framerate=60/1 ! " // 60/1 表示 60fps
          "jpegdec ! "         // 解码 MJPG 压缩流到原始格式 (x-raw)
          "videoconvert ! "    // 转换为 OpenCV 兼容的颜色空间 (如 RGB 或 BGR)
          "appsink";           // OpenCV 通过 appsink 从 GStreamer 管道获取帧

      // 使用 GStreamer 管道和 cv::CAP_GSTREAMER 标志初始化 VideoCapture
      capture = cv::VideoCapture(gstreamer_pipeline, cv::CAP_GSTREAMER);
  }


  // if (motion.params.debug)
  // {
  //   display.init(4); // 调试UI初始化
  //   display.frameMax = capture.get(CAP_PROP_FRAME_COUNT) - 1;
  //   createTrackbar("Frame", "ICAR", &display.index, display.frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
  //   setMouseCallback("ICAR", mouseCallback);                                               // 创建鼠标键盘快捷键事件
  // }

	Factory<TaskData> task_factory(3);
	Factory<TaskData> AI_task_factory(3);
	Factory<DebugData> debug_factory(5);
	std::vector<PredictResult> predict_result;
	
	std::thread task_producer(&producer, std::ref(task_factory), std::ref(AI_task_factory), std::ref(config));
	std::thread AI_producer(&AIConsumer, std::ref(task_factory), std::ref(predict_result), std::ref(config));
	std::thread task_consumer(&consumer, std::ref(task_factory), std::ref(debug_factory), std::ref(predict_result), std::ref(config), std::ref(uart));
	if (config.debug) {
		std::thread debug_data_consumer(&debugDataConsumer, std::ref(debug_factory));
		debug_data_consumer.join();
	}
	
	task_producer.join();
	task_consumer.join();
	AI_producer.join();



  uart->close(); // 串口通信关闭
  capture.release();
  return 0;
}

/**
 * @brief 鼠标的事件回调函数
 *
 */
void mouseCallback(int event, int x, int y, int flags, void *userdata)
{
  double value;
  switch (event)
  {
  case EVENT_MOUSEWHEEL: // 鼠标滑球
  {
    value = getMouseWheelDelta(flags); // 获取滑球滚动值
    if (value > 0)
      display.index++;
    else if (value < 0)
      display.index--;

    if (display.index < 0)
      display.index = 0;
    if (display.index > display.frameMax)
      if (display.index > display.frameMax)
        display.index = display.frameMax;
    break;
  }
  default:
    break;
  }
}