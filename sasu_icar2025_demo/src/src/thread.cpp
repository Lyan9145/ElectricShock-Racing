#include "../include/common.hpp"		// 公共类方法文件
#include "../include/detection.hpp"		// 百度Paddle框架移动端部署
#include "../include/uart.hpp"			// 串口通信驱动
#include "../include/motion.hpp"		// 运动控制类
#include "../include/thread.hpp"		// 线程管理
#include "../include/preprocess.hpp"	// 图像预处理类
#include "../include/controlcenter.hpp" // 控制中心
#include "../include/tracking.hpp"		// 跟踪算法
#include "../include/bridge.hpp"
#include "../include/catering.hpp"
#include "../include/crosswalk.hpp"
#include "../include/layby.hpp"
#include "../include/obstacle.hpp"
#include "../include/parking.hpp"
#include "../include/crossroad.hpp"
#include "../include/ring.hpp"
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace cv;

shared_ptr<Uart> g_uart = nullptr;
volatile sig_atomic_t g_exit_flag = 0;

void drawUI(Mat &img, std::vector<PredictResult> results)
{
    for (int i = 0; i < results.size(); i++)
    {
        PredictResult result = results[i];

        auto score = std::to_string(result.score);
        int pointY = result.y - 20;
        if (pointY < 0)
            pointY = 0;
        cv::Rect rectText(result.x, pointY, result.width, 20);
        cv::rectangle(img, rectText, getCvcolor(result.type), -1);
        std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 3) + "]";
        cv::Rect rect(result.x, result.y, result.width, result.height);
        cv::rectangle(img, rect, getCvcolor(result.type), 1);
        cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
    }
}


/**
 * @brief 获取Opencv颜色
 *
 * @param index 序号
 * @return cv::Scalar
 */
cv::Scalar getCvcolor(int index)
{
	switch (index)
	{
	case 0:
		return cv::Scalar(0, 255, 0); // 绿
		break;
	case 1:
		return cv::Scalar(255, 255, 0); // 天空蓝
		break;
	case 2:
		return cv::Scalar(0, 0, 255); // 大红
		break;
	case 3:
		return cv::Scalar(0, 250, 250); // 大黄
		break;
	case 4:
		return cv::Scalar(250, 0, 250); // 粉色
		break;
	case 5:
		return cv::Scalar(0, 102, 255); // 橙黄
		break;
	case 6:
		return cv::Scalar(255, 0, 0); // 深蓝
		break;
	case 7:
		return cv::Scalar(255, 255, 255); // 大白
		break;
	case 8:
		return cv::Scalar(247, 43, 113);
		break;
	case 9:
		return cv::Scalar(40, 241, 245);
		break;
	case 10:
		return cv::Scalar(237, 226, 19);
		break;
	case 11:
		return cv::Scalar(245, 117, 233);
		break;
	case 12:
		return cv::Scalar(55, 13, 19);
		break;
	case 13:
		return cv::Scalar(255, 255, 255);
		break;
	case 14:
		return cv::Scalar(237, 226, 19);
		break;
	case 15:
		return cv::Scalar(0, 255, 0);
		break;
	default:
		return cv::Scalar(255, 0, 0);
		break;
	}
}

extern Uart uart;
bool flag = false;

// 图像信息显示函数
void displayImageInfo(const Mat &img, long preTime, string info = "")
{
  static int frameCount = 0;
  static auto lastTime = chrono::high_resolution_clock::now();
  frameCount++;

  auto currentTime = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::milliseconds>(currentTime - lastTime);

  if (duration.count() >= 1000)
  { // 每秒更新一次
    double fps = frameCount * 1000.0 / duration.count();
    printf("[%s] Resolution: %dx%d | FPS: %.2f\n", info.c_str(), img.cols, img.rows, fps);
    frameCount = 0;
    lastTime = currentTime;
  }
}

// 性能监控
void performanceMonitor(
    std::chrono::high_resolution_clock::time_point &lastTime,
    int &frameCounter,
    std::chrono::milliseconds &totalWorkDuration, // 新增：总工作时间
    const std::string& info)
{
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto intervalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);

    if (intervalDuration.count() >= 1000)
    { // 每秒更新一次
        // 使用浮点数进行计算，避免整数除法问题，也更精确
        double intervalSeconds = intervalDuration.count() / 1000.0;
        double fps = frameCounter / intervalSeconds;
        
        // 使用累加的总处理时间来计算负载
        double loadPercent = totalWorkDuration.count() * 100.0 / intervalDuration.count();
        // 确保负载不超过100% (理论上可能因计时误差略微超过)
        if (loadPercent > 100.0) loadPercent = 100.0; 

        printf("[%s] FPS: %.2f, Load: %.1f %%\n", info.c_str(), fps, loadPercent);

        // 重置计数器
        frameCounter = 0;
        lastTime = currentTime;
        totalWorkDuration = std::chrono::milliseconds::zero(); // 重置累加时间
    }
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */

// 信号处理函数
void signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		printf("\n[INFO] Received Ctrl+C signal, stopping car safely...\n");
		g_exit_flag = 1;

		if (g_uart != nullptr)
		{
			g_uart->carControl(0, PWMSERVOMID); // 停车
			sleep(1);							// 等待1s确保停车指令发送完成
			printf("[INFO] Car stopped successfully.\n");
			g_uart->close();
		}
		sleep(1); // 等待串口关闭完成
		printf("[INFO] Exiting program...\n");
		exit(0);
	}
}

bool producer(Factory<TaskData> &task_data, Factory<AIData> &AI_task_data, cv::VideoCapture &capture)
{
	try
	{
		Preprocess preprocess;
		cv::Mat img_buffer;
		auto lastTime = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds totalWorkDuration = std::chrono::milliseconds::zero(); // 用于累加处理时间		
		int frameCounter = 0; // 帧计数器
		long preTime1;
		while (true)
		{
			if (g_exit_flag)
			{
				printf("[INFO] Producer thread exiting...\n");
				break;
			}
			if (!capture.read(img_buffer))
			{
				// std::this_thread::sleep_for(std::chrono::milliseconds(1));
				usleep(500);
				continue;
			}
			auto frameStartTime = std::chrono::high_resolution_clock::now();
			TaskData src;
			AIData ai_src;
			// auto time_now = std::chrono::high_resolution_clock::now();
			src.timestamp = frameStartTime;
			// 图像预处理
			// src.img = img_buffer.clone(); // 克隆图像数据
			resize(img_buffer, src.img, Size(640, 480), 0, 0, INTER_LINEAR);
			displayImageInfo(src.img, preTime1, "capture");
			src.img = preprocess.correction(src.img); // 图像矫正 
			src.img = preprocess.resizeImage(src.img); // 图像尺寸标准化

			ai_src.timestamp = frameStartTime;
			ai_src.img = src.img;

			task_data.produce(src);
			AI_task_data.produce(ai_src);
			// 性能监控
			auto frameEndTime = std::chrono::high_resolution_clock::now();
            // 累加本帧的处理耗时
            totalWorkDuration += std::chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - frameStartTime);
            
            frameCounter++;
            
            // 调用改进后的性能监控函数
            performanceMonitor(lastTime, frameCounter, totalWorkDuration, "Producer");
		}
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << "[Error] Exception in producer thread: " << e.what() << '\n';
		g_exit_flag = 1; // 设置退出标志
	}
	return false;
	
}

bool AIConsumer(Factory<AIData> &task_data, std::vector<PredictResult> &predict_result, std::mutex &predict_result_lock, Motion &motion)
{
	try
	{
		// 目标检测类(AI模型文件)
		shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
		detection->score = motion.params.score; // AI检测置信度
		// long preTime1;
		auto lastTime = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds totalWorkDuration = std::chrono::milliseconds::zero(); // 用于累加处理时间		
		int frameCounter = 0; // 帧计数器
		while (true)
		{
			if (g_exit_flag)
			{
				printf("[INFO] AI Consumer thread exiting...\n");
				break;
			}
			AIData dst;
			task_data.consume(dst);
			auto frameStartTime = std::chrono::high_resolution_clock::now();
			detection->inference(dst.img);
			// displayImageInfo(dst.img, preTime1, "AI inference");
			predict_result_lock.lock();
			predict_result = detection->results;
			predict_result_lock.unlock();
			// 性能监控
			auto frameEndTime = std::chrono::high_resolution_clock::now();
            // 累加本帧的处理耗时
            totalWorkDuration += std::chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - frameStartTime);
            frameCounter++;
			performanceMonitor(lastTime, frameCounter, totalWorkDuration, "AI Consumer");
		}
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << "[Error] Exception in AI Consumer thread: " << e.what() << '\n';
		g_exit_flag = 1; // 设置退出标志
	}
	return false;
}

bool consumer(Factory<TaskData> &task_data, Factory<DebugData> &debug_data, std::vector<PredictResult> &predict_result, std::mutex &predict_result_lock, Motion &motion, Uart &uart)
{
	try
	{
		// Standard standard(config);
		Preprocess preprocess; // 图像预处理类
		// Motion motion;			  // 运动控制类
	
		Tracking tracking;		  // 赛道识别类
		Crossroad crossroad;	  // 十字道路识别类
		Ring ring;				  // 环岛识别类
		Bridge bridge;			  // 坡道区检测类
		Catering catering;		  // 快餐店检测类
		Obstacle obstacle;		  // 障碍区检测类
		Layby layby;			  // 临时停车区检测类
		Parking parking;		  // 充电停车场检测类
		StopArea stopArea;		  // 停车区识别与路径规划类
		ControlCenter ctrlCenter; // 控制中心计算类
		VideoCapture capture;	  // Opencv相机类
	
		int countInit = 0;					  // 初始化计数器
		Scene scene = Scene::NormalScene;	  // 初始化场景：常规道路
		Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
		long preTime1;
		long preTime2;
		long preTime3;
		Mat img;
		std::vector<PredictResult> predict_result_buffer;

		auto lastTime = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds totalWorkDuration = std::chrono::milliseconds::zero(); // 用于累加处理时间		
		int frameCounter = 0; // 帧计数器
		auto lastImageTime = std::chrono::high_resolution_clock::now();

	
		while (true)
		{
			auto start = std::chrono::high_resolution_clock::now();
			if (g_exit_flag)
			{
				printf("[INFO] Consumer thread exiting...\n");
				break;
			}
	
			TaskData src;
			task_data.consume(src);
			if (src.img.empty())
			{
				// printf("[Warning] No image data received in consumer\n");
				continue;
			}

			lastImageTime = src.timestamp; // 更新最后图像时间戳
			auto getimg = std::chrono::high_resolution_clock::now();

			// displayImageInfo(src.img, preTime1, "Control loop");
			
			// 读取模型结果
			try
			{
				predict_result_lock.lock();
				predict_result_buffer = predict_result;
				predict_result_lock.unlock();
				// if (predict_result_buffer.empty())
				// {
				// 	printf("[Warning] No prediction results available.\n");
				// }
			}
			catch (const std::exception &e)
			{
				printf("[Warning] Failed to read predict result: %s\n", e.what());
			}
			auto getpredict = std::chrono::high_resolution_clock::now();
	
			Mat imgBinary = preprocess.binaryzation(src.img);
			auto frameStartTime = std::chrono::high_resolution_clock::now();
			//[04] 赛道识别
			tracking.rowCutUp = motion.params.rowCutUp;			// 图像顶部切行（前瞻距离）
			tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
			// tracking.trackRecognition(imgBinary);
			UartStatus status = uart.getStatus();
			// printf("距离: %.3f m, 电压: %.2f V, 速度: %.3f m/s\n", status.distance, status.voltage, status.speed);

			Mat result_img = src.img.clone(); // 克隆原图像用于绘制结果
			tracking.trackRecognition_new(imgBinary, result_img, src, predict_result_buffer, status);
			drawUI(result_img, predict_result_buffer); // 绘制检测结果
			auto trackEndTime = std::chrono::high_resolution_clock::now();

			DebugData D_data;
			D_data.img = result_img; // 克隆结果图像
			debug_data.produce(D_data);
			// imshow("Tracking", result_img);
			// waitKey(1);
			auto trackShowTime = std::chrono::high_resolution_clock::now();
	
			//[05] 停车区检测
			if (motion.params.stop)
			{
				if (stopArea.process(predict_result_buffer))
				{
					scene = Scene::StopScene;
					if (stopArea.countExit > 20)
					{
						uart.carControl(0, PWMSERVOMID); // 控制车辆停止运动
						sleep(1);
						printf("Car stopping in stop area...\n");
						g_exit_flag = 1;
						// 程序退出
						continue;
					}
				}
			}
	
	
			//[14] 运动控制(速度+方向)
			if (countInit > 30)
			{
				uart.carControl(src.speed, src.steering_pwm); // 串口通信控制车辆
																 // TODO:串口现在为阻塞发送，改为异步
			}
			else
				countInit++;
	
			// 性能监控
			auto frameEndTime = std::chrono::high_resolution_clock::now();
            // 累加本帧的处理耗时
            totalWorkDuration += std::chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - frameStartTime);
            frameCounter++;
			performanceMonitor(lastTime, frameCounter, totalWorkDuration, "Consumer");
			if (chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - start).count() > 50)
			{
				cout << "[Warning] Consumer loop took too long: " 
					<< chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - start).count() << " ms\n";
				cout << "Get image time: " 
					<< chrono::duration_cast<std::chrono::milliseconds>(getimg - start).count() << " ms\n";
				cout << "Get predict time: " 
					<< chrono::duration_cast<std::chrono::milliseconds>(getpredict - getimg).count() << " ms\n";
				cout << "Track time: " 
					<< chrono::duration_cast<std::chrono::milliseconds>(trackEndTime - getpredict).count() << " ms\n";
				cout << "Show time: " 
					<< chrono::duration_cast<std::chrono::milliseconds>(trackShowTime - trackEndTime).count() << " ms\n";
			}
	
		}
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << "[Error] Exception in consumer thread: " << e.what() << '\n';
		g_exit_flag = 1; // 设置退出标志
	}
	return false;
}

bool debugDataConsumer(Factory<DebugData> &debug_data)
{
	const int target_fps = 24;
	const auto frame_duration = std::chrono::milliseconds(1000 / target_fps); // ~41.67ms per frame
	auto last_display_time = std::chrono::high_resolution_clock::now();

	// 视频保存相关
	cv::VideoWriter video_writer;
	bool is_writer_initialized = false;
	auto now = std::chrono::system_clock::now();
	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	std::tm tm_now;
#ifdef _WIN32
	localtime_s(&tm_now, &now_time);
#else
	localtime_r(&now_time, &tm_now);
#endif
	char datetime_buf[32];
	std::strftime(datetime_buf, sizeof(datetime_buf), "%Y%m%d_%H%M%S", &tm_now);
	const std::string video_filename = std::string("recorder/run_") + datetime_buf + ".mp4";
	const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // mp4v编码
	const cv::Size video_size(320, 240);

	while (true)
	{
		if (g_exit_flag)
		{
			printf("[INFO] Debug Consumer thread exiting...\n");
			break;
		}

		DebugData dst;
		debug_data.consume(dst);
		if (dst.img.empty())
			continue;

		auto current_time = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_display_time);

		// 只有当经过足够时间才显示帧
		if (elapsed >= frame_duration)
		{
			cv::Mat display_img;
			cv::resize(dst.img, display_img, video_size);
			cv::imshow("output", display_img);
			cv::waitKey(1);
			last_display_time = current_time;

			// // 初始化VideoWriter
			// if (!is_writer_initialized)
			// {
			// 	video_writer.open(video_filename, fourcc, target_fps, video_size, true);
			// 	if (!video_writer.isOpened())
			// 	{
			// 		std::cerr << "[Error] Failed to open video file for writing: " << video_filename << std::endl;
			// 	}
			// 	else
			// 	{
			// 		is_writer_initialized = true;
			// 		printf("[INFO] Video recording started: %s\n", video_filename.c_str());
			// 	}
			// }
			// // 写入视频帧
			// if (is_writer_initialized)
			// {
			// 	video_writer.write(dst.img);
			// }
		}
	}

	// 释放VideoWriter资源
	if (is_writer_initialized)
	{
		video_writer.release();
		printf("[INFO] Video recording stopped.\n");
	}
	return true;
}
