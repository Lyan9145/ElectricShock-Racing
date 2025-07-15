#pragma once

class Motion; // 前向声明 Motion 类
class Uart;   // 你的代码中也用到了 Uart&，可能也需要前向声明

#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <chrono>
#include "common.hpp"     //公共类方法文件
#include "detection.hpp"  //百度Paddle框架移动端部署
#include "uart.hpp"       //串口通信驱动
// #include "motion.hpp"      //运动控制类

#include <sys/stat.h>
#define mkdir_if_needed(path) mkdir(path, 0755)

// 全局变量用于信号处理
extern shared_ptr<Uart> g_uart;
extern volatile sig_atomic_t g_exit_flag;


struct DebugData{
	cv::Mat img;
	std::vector<PredictResult> results;
	int steering_pwm = 0;
	double speed = 0;
	Scene scene = Scene::NormalScene; // 场景类型
	std::chrono::high_resolution_clock::time_point timestamp;
};

struct TaskData{
	cv::Mat img;
	std::chrono::high_resolution_clock::time_point timestamp;
	int steering_pwm = 4000; // 4000-5000-6000, 0 means off
	double speed = 0; // m/s
};

struct AIData{
	cv::Mat img;
	std::chrono::high_resolution_clock::time_point timestamp;
	std::vector<PredictResult> results;
};

template<typename T>
class Factory{
private:
	std::deque<T> buffer;
	int buffer_size;
	std::mutex lock;
public:
	Factory(int size) {
		buffer_size = size;	
	}
	bool produce(T &product);
	bool consume(T &product);
};

template<typename T>
bool Factory<T>::produce(T &product) {
	lock.lock();
	if (buffer.size() < buffer_size) {
		buffer.push_back(product);
	} else {
		buffer.pop_front();
		buffer.push_back(product);
	}
	lock.unlock();

	return true;
}

template<typename T>
bool Factory<T>::consume(T &product) {
	unsigned int wait_count = 0;
	while (true) {
		lock.lock();
		if(!buffer.empty()) 
			break;
		lock.unlock();
		usleep(200);
		wait_count++;
		if (wait_count > 5000)
		{ // 1s超时
			printf("[Warning] Consumer thread waiting too long, exiting...\n");
			return false;
		}
	}
	
	product = buffer.front();
	buffer.pop_front();
	lock.unlock();

	return true;
}
// ------------------------------------ //
void signalHandler(int signal);

bool producer(Factory<TaskData> &task_data, Factory<AIData> &AI_task_data, cv::VideoCapture &capture);
bool AIConsumer(Factory<AIData> &task_data, std::vector<PredictResult> &predict_result, std::mutex &predict_result_lock, Motion &motion);
bool consumer(Factory<TaskData> &task_data, Factory<DebugData> &debug_data, std::vector<PredictResult> &predict_result, std::mutex &predict_result_lock, Motion &motion, Uart &uart);
void drawUI(Mat &img, std::vector<PredictResult> results);
cv::Scalar getCvcolor(int index);
bool debugDataConsumer(Factory<DebugData> & debug_data);
void displayImageInfo(const Mat &img, long preTime, string info);
void performanceMonitor(
    std::chrono::high_resolution_clock::time_point &lastTime,
    int &frameCounter,
    std::chrono::milliseconds &totalWorkDuration, // 新增：总工作时间
    const std::string& info);
