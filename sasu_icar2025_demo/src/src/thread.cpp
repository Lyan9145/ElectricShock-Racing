#include "../include/thread.hpp"
#include "../include/common.hpp"	 //公共类方法文件
#include "../include/detection.hpp"	 //百度Paddle框架移动端部署
#include "../include/uart.hpp"		 //串口通信驱动
#include "../include/motion.hpp"	 //智能车运动控制类
#include "../include/controlcenter.hpp"
#include "detection/bridge.cpp"		 //AI检测：坡道区
#include "detection/obstacle.cpp"	 //AI检测：障碍区
#include "detection/catering.cpp"	 //AI检测：餐饮区
#include "detection/layby.cpp"		 //AI检测：临时停车区
#include "detection/parking.cpp"	 //AI检测：充电停车场
#include "detection/crosswalk.cpp"	 //AI检测：停车区
#include "preprocess.cpp"			 //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"		 //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"	 //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace cv;

extern Uart uart;
bool flag = false;

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
	Preprocess preprocess;
	cv::Mat img_buffer;
	while (true)
	{
		if (g_exit_flag) {
			break;
		}
		if (!capture.getImage(img_buffer))
		{
			usleep(50);
			continue;
		}
		TaskData src;
		src.img = img_buffer.clone();
		auto time_now = std::chrono::high_resolution_clock::now();
		src.timestamp = time_now;
		// 图像预处理
		src.img = preprocess.resizeImage(src.img); // 图像尺寸标准化
		// src.img = preprocess.correction(src.img); // 图像矫正 TODO: 需要相机标定

		task_data.produce(src);
		AI_task_data.produce(src);
	}
	return true;
}

bool AIConsumer(Factory<TaskData> &task_data, std::vector<PredictResult> &predict_result, Motion &motion)
{
  	// 目标检测类(AI模型文件)
  	shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
	detection->score = motion.params.score; // AI检测置信度
	std::mutex lock;
	while (true)
	{
		if (g_exit_flag) {
			break;
		}
		TaskData dst;
		task_data.consume(dst);
		detection->inference(dst.img);
		lock.lock();
		predict_result = detection->results;
		lock.unlock();
	}
	return true;
}

bool consumer(Factory<TaskData> &task_data, Factory<DebugData> &debug_data, std::vector<PredictResult> &predict_result, Motion &motion, Uart &uart)
{
	// Standard standard(config);
	Preprocess preprocess;	  // 图像预处理类
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

	int countInit = 0;		  // 初始化计数器
	Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  	Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
	long preTime;
  	Mat img;

	if (!uart.isOpen)
	{
		printf("[Error] Uart is not open!\n");
		return false;
	}

	while (true)
	{
		if (g_exit_flag) {
        	break;
		}

		task_data.consume(src);
		if (src.img.empty())
		{
			printf("[Warning] No image data received in consumer\n");
			continue;
		}

		Mat imgBinary = preprocess.binaryzation(src.img);
		
		//[04] 赛道识别
		tracking.rowCutUp = motion.params.rowCutUp;			// 图像顶部切行（前瞻距离）
		tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
		tracking.trackRecognition(imgBinary);


		//[05] 停车区检测
		if (motion.params.stop)
		{
			if (stopArea.process(detection->results))
			{
				scene = Scene::StopScene;
				if (stopArea.countExit > 20)
				{
					uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
					sleep(1);
					printf("Car stopping in stop area...\n");
					g_exit_flag = 1;; // 程序退出
					break;
				}
			}
		}

		//[06] 快餐店检测
		if ((scene == Scene::NormalScene || scene == Scene::CateringScene) &&
			motion.params.catering)
		{
			if (catering.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
				scene = Scene::CateringScene;
			else
				scene = Scene::NormalScene;
		}

		//[07] 临时停车区检测
		if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
			motion.params.catering)
		{
			if (layby.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
				scene = Scene::LaybyScene;
			else
				scene = Scene::NormalScene;
		}

		//[08] 充电停车场检测
		if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
			motion.params.parking)
		{
			if (parking.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
				scene = Scene::ParkingScene;
			else
				scene = Scene::NormalScene;
		}

		//[09] 坡道区检测
		if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
			motion.params.bridge)
		{
			if (bridge.process(tracking, detection->results))
				scene = Scene::BridgeScene;
			else
				scene = Scene::NormalScene;
		}

		// [10] 障碍区检测
		if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) &&
			motion.params.obstacle)
		{
			if (obstacle.process(tracking, detection->results))
			{
				uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
				scene = Scene::ObstacleScene;
			}
			else
				scene = Scene::NormalScene;
		}

		//[11] 十字道路识别与路径规划
		if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
			motion.params.cross)
		{
			if (crossroad.crossRecognition(tracking))
				scene = Scene::CrossScene;
			else
				scene = Scene::NormalScene;
		}

		//[12] 环岛识别与路径规划
		if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
			motion.params.ring && catering.noRing)
		{
			if (ring.process(tracking, imgBinary))
				scene = Scene::RingScene;
			else
				scene = Scene::NormalScene;
		}

		//[13] 车辆控制中心拟合
		ctrlCenter.fitting(tracking);

		if (scene != Scene::ParkingScene)
		{
			if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
			{
				// uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
				cout << "PANIC: Out of track!" << endl;
				// sleep(2);
				// printf("Car stopping due to derailment...\n");
				// g_exit_flag = 1;; // 程序退出
				// break;
			}
		}

		//[14] 运动控制(速度+方向)
		if (!motion.params.debug && countInit > 30) // 非调试模式下
		{
			// 触发停车
			if ((catering.stopEnable && scene == Scene::CateringScene) || (layby.stopEnable && scene == Scene::LaybyScene) || (parking.step == parking.ParkStep::stop))
			{
				motion.speed = 0;
			}
			else if (scene == Scene::CateringScene)
				motion.speed = motion.params.speedCatering;
			else if (scene == Scene::LaybyScene)
				motion.speed = motion.params.speedLayby;
			else if (scene == Scene::ParkingScene && parking.step == parking.ParkStep::trackout) // 倒车出库
				motion.speed = -motion.params.speedDown;
			else if (scene == Scene::ParkingScene) // 减速
				motion.speed = motion.params.speedParking;
			else if (scene == Scene::BridgeScene) // 坡道速度
				motion.speed = motion.params.speedBridge;
			else if (scene == Scene::ObstacleScene) // 危险区速度
				motion.speed = motion.params.speedObstacle;
			else if (scene == Scene::RingScene) // 环岛速度
				motion.speed = motion.params.speedRing;
			else if (scene == Scene::StopScene)
				motion.speed = motion.params.speedDown;
			else
				motion.speedCtrl(true, false, ctrlCenter); // 车速控制

			motion.poseCtrl(ctrlCenter.controlCenter);		 // 姿态控制（舵机）
			uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
															 // TODO:串口现在为阻塞发送，改为异步
		}
		else
			countInit++;

		// 命令行调试输出
		printf(">> Speed: %.2fm/s | Servo: %d | Scene: %s\n",
			   motion.speed, motion.servoPwm,
			   sceneToString(scene).c_str());

		//[15] 综合显示调试UI窗口
		Mat imgWithDetection = imgCorrect.clone();
		detection->drawBox(imgWithDetection);
		ctrlCenter.drawImage(tracking, imgWithDetection); // 图像绘制路径计算结果（控制中心）
		Mat imgRes = imgWithDetection.clone(); // 复制图像用于后续处理
		switch (scene)
		{
		case Scene::NormalScene:
		break;
		case Scene::CrossScene:                  // [ 十字区 ]
		crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "+", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::RingScene:              // [ 环岛 ]
		ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "H", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::CateringScene:              // [ 餐饮区 ]
		catering.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "C", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::LaybyScene:              // [ 临时停车区 ]
		layby.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "T", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::ParkingScene:              // [ 充电停车场 ]
		parking.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::BridgeScene:              // [ 坡道区 ]
		bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		case Scene::ObstacleScene:    //[ 障碍区 ]
		obstacle.drawImage(imgRes); // 图像绘制特殊赛道识别结果
		// circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
		// putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
		break;
		default: // 常规道路场景：无特殊路径规划
		break;
		}
		imshow("AI Detection", imgRes);
		waitKey(1); // 等待1ms，使窗口能够刷新显示



		//[16] 状态复位
		if (sceneLast != scene)
		{
			if (scene == Scene::NormalScene)
				uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
			else
				uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
		}
		sceneLast = scene; // 记录当前状态
		if (scene == Scene::ObstacleScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::CrossScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::RingScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::CateringScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::LaybyScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::ParkingScene)
			scene = Scene::NormalScene;
		else if (scene == Scene::StopScene)
			scene = Scene::NormalScene;

		return true;
	}
}

bool debugDataConsumer(Factory<DebugData> & debug_data)
{
	while (true)
	{
		DebugData dst;
		debug_data.consume(dst);
		if (dst.img.empty())
			continue;
		drawBox(dst.img, dst.results);
		cv::resize(dst.img, cv::Size(640, 480));
		cv::imshow("output", dst.img);
		cv::waitKey(1);
	}
	return true;
}
