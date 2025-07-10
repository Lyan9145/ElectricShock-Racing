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

bool producer(Factory<TaskData> &task_data, Factory<TaskData> &AI_task_data, cv::VideoCapture &capture)
{
	try
	{
		Preprocess preprocess;
		cv::Mat img_buffer;
		auto lastTime = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds totalWorkDuration = std::chrono::milliseconds::zero(); // 用于累加处理时间		
		int frameCounter = 0; // 帧计数器
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
			// auto time_now = std::chrono::high_resolution_clock::now();
			src.timestamp = frameStartTime;
			// 图像预处理
			src.img = img_buffer.clone(); // 克隆图像数据
			resize(img_buffer, src.img, Size(640, 480), 0, 0, INTER_LINEAR);
			src.img = preprocess.correction(src.img); // 图像矫正 
			src.img = preprocess.resizeImage(src.img); // 图像尺寸标准化
			// displayImageInfo(src.img, preTime1, "producer capture");

			task_data.produce(src);
			AI_task_data.produce(src);
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

bool AIConsumer(Factory<TaskData> &task_data, std::vector<PredictResult> &predict_result, std::mutex &predict_result_lock, Motion &motion)
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
			TaskData dst;
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
		Mat result_img;
		std::vector<PredictResult> predict_result_buffer;

		auto lastTime = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds totalWorkDuration = std::chrono::milliseconds::zero(); // 用于累加处理时间		
		int frameCounter = 0; // 帧计数器
	
		while (true)
		{
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
	
			Mat imgBinary = preprocess.binaryzation(src.img);
			auto frameStartTime = std::chrono::high_resolution_clock::now();
			//[04] 赛道识别
			// tracking.rowCutUp = motion.params.rowCutUp;			// 图像顶部切行（前瞻距离）
			// tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
			// tracking.trackRecognition(imgBinary);
			UartStatus status = uart.getStatus();
    		// printf("距离: %.3f m, 电压: %.2f V, 速度: %.3f m/s\n", status.distance, status.voltage, status.speed);
			

			result_img = src.img.clone(); // 克隆原图像用于绘制结果
			tracking.trackRecognition_new(imgBinary, result_img, src, predict_result_buffer, status);
			drawUI(result_img, predict_result_buffer); // 绘制检测结果

			imshow("Tracking", result_img);
			waitKey(1);
	
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
	
			// //[06] 快餐店检测
			// if ((scene == Scene::NormalScene || scene == Scene::CateringScene) &&
			// 	motion.params.catering)
			// {
			// 	if (catering.process(tracking, imgBinary, predict_result_buffer)) // 传入二值化图像进行再处理
			// 		scene = Scene::CateringScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[07] 临时停车区检测
			// if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
			// 	motion.params.catering)
			// {
			// 	if (layby.process(tracking, imgBinary, predict_result_buffer)) // 传入二值化图像进行再处理
			// 		scene = Scene::LaybyScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[08] 充电停车场检测
			// if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
			// 	motion.params.parking)
			// {
			// 	if (parking.process(tracking, imgBinary, predict_result_buffer)) // 传入二值化图像进行再处理
			// 		scene = Scene::ParkingScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[09] 坡道区检测
			// if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
			// 	motion.params.bridge)
			// {
			// 	if (bridge.process(tracking, predict_result_buffer))
			// 		scene = Scene::BridgeScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// // [10] 障碍区检测
			// if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) &&
			// 	motion.params.obstacle)
			// {
			// 	if (obstacle.process(tracking, predict_result_buffer))
			// 	{
			// 		scene = Scene::ObstacleScene;
			// 	}
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[11] 十字道路识别与路径规划
			// if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
			// 	motion.params.cross)
			// {
			// 	if (crossroad.crossRecognition(tracking))
			// 		scene = Scene::CrossScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[12] 环岛识别与路径规划
			// if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
			// 	motion.params.ring && catering.noRing)
			// {
			// 	if (ring.process(tracking, imgBinary))
			// 		scene = Scene::RingScene;
			// 	else
			// 		scene = Scene::NormalScene;
			// }
	
			// //[13] 车辆控制中心拟合
			// // ctrlCenter.fitting(tracking);
	
			// if (scene != Scene::ParkingScene)
			// {
			// 	if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
			// 	{
			// 		uart.carControl(0, PWMSERVOMID); // 控制车辆停止运动
			// 		cout << "PANIC: Out of track!" << endl;
			// 		// sleep(2);
			// 		// printf("Car stopping due to derailment...\n");
			// 		// g_exit_flag = 1;; // 程序退出
			// 		// break;
			// 	}
			// }
	
			//[14] 运动控制(速度+方向)
			if (countInit > 30)
			{
				// 触发停车
				// if ((catering.stopEnable && scene == Scene::CateringScene) || (layby.stopEnable && scene == Scene::LaybyScene) || (parking.step == parking.ParkStep::stop))
				// {
				// 	motion.speed = 0;
				// }
				// else if (scene == Scene::CateringScene)
				// 	motion.speed = motion.params.speedCatering;
				// else if (scene == Scene::LaybyScene)
				// 	motion.speed = motion.params.speedLayby;
				// else if (scene == Scene::ParkingScene && parking.step == parking.ParkStep::trackout) // 倒车出库
				// 	motion.speed = -motion.params.speedDown;
				// else if (scene == Scene::ParkingScene) // 减速
				// 	motion.speed = motion.params.speedParking;
				// else if (scene == Scene::BridgeScene) // 坡道速度
				// 	motion.speed = motion.params.speedBridge;
				// else if (scene == Scene::ObstacleScene) // 危险区速度
				// 	motion.speed = motion.params.speedObstacle;
				// else if (scene == Scene::RingScene) // 环岛速度
				// 	motion.speed = motion.params.speedRing;
				// else if (scene == Scene::StopScene)
				// 	motion.speed = motion.params.speedDown;
				// else
				// 	motion.speedCtrl(true, false, ctrlCenter); // 车速控制
	
				// motion.poseCtrl(ctrlCenter.controlCenter);		 // 姿态控制（舵机）
				uart.carControl(src.speed, src.steering_pwm); // 串口通信控制车辆
																 // TODO:串口现在为阻塞发送，改为异步
			}
			else
				countInit++;
	
			// 命令行调试输出
			// printf(">> Speed: %.2fm/s | Servo: %d | Scene: %s\n",
			// 	   motion.speed, motion.servoPwm,
			// 	   sceneToString(scene).c_str());
	
			//[15] 综合显示调试UI窗口
			// if (0)
			// {
			// 	Mat imgWithDetection = src.img.clone();
			// 	drawUI(imgWithDetection, predict_result_buffer); // 绘制检测结果
			// 	ctrlCenter.drawImage(tracking, imgWithDetection); // 图像绘制路径计算结果（控制中心）
			// 	Mat imgRes = imgWithDetection.clone();			  // 复制图像用于后续处理
			// 	switch (scene)
			// 	{
			// 	case Scene::NormalScene:
			// 		break;
			// 	case Scene::CrossScene:					   // [ 十字区 ]
			// 		crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::RingScene:				  // [ 环岛 ]
			// 		ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::CateringScene:				  // [ 餐饮区 ]
			// 		catering.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::LaybyScene:				   // [ 临时停车区 ]
			// 		layby.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::ParkingScene:				 // [ 充电停车场 ]
			// 		parking.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::BridgeScene:				// [ 坡道区 ]
			// 		bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	case Scene::ObstacleScene:		//[ 障碍区 ]
			// 		obstacle.drawImage(imgRes); // 图像绘制特殊赛道识别结果
			// 		break;
			// 	default: // 常规道路场景：无特殊路径规划
			// 		break;
			// 	}
			// 	imshow("AI Detection", imgRes);
			// 	waitKey(1); // 等待1ms，使窗口能够刷新显示
			// }
	
			//[16] 状态复位
			// if (sceneLast != scene)
			// {
			// 	// printf(">> Scene changed from [%s] to [%s]\n", sceneToString(sceneLast).c_str(), sceneToString(scene).c_str());
			// }
			// sceneLast = scene; // 记录当前状态
			// if (scene == Scene::ObstacleScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::CrossScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::RingScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::CateringScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::LaybyScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::ParkingScene)
			// 	scene = Scene::NormalScene;
			// else if (scene == Scene::StopScene)
			// 	scene = Scene::NormalScene;

			// 性能监控
			auto frameEndTime = std::chrono::high_resolution_clock::now();
            // 累加本帧的处理耗时
            totalWorkDuration += std::chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - frameStartTime);
            frameCounter++;
			performanceMonitor(lastTime, frameCounter, totalWorkDuration, "Consumer");
	
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
	// while (true)
	// {
		// DebugData dst;
		// debug_data.consume(dst);
		// if (dst.img.empty())
		// 	continue;
		// drawUI(dst.img, dst.results);
		// cv::resize(dst.img, cv::Size(640, 480));
		// cv::imshow("output", dst.img);
		// cv::waitKey(1);
	// }
	return true;
}
