#pragma once

#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>
#include <unistd.h>


// #include "../detector/detection.hpp"
// #include "../param/param.hpp"
// #include "../capture/capture.h"
// #include "../track/standard/standard.h"
// // #include "../port/canPort.hpp"
// #include "../port/serialPort.hpp"

#include "common.hpp"     //公共类方法文件
#include "detection.hpp"  //百度Paddle框架移动端部署
#include "uart.hpp"       //串口通信驱动
#include "../src/motion.hpp"

struct DebugData{
	cv::Mat img;
	cv::Mat bin_img;
	std::vector<PredictResult> results;
	int steering_pwm = 0;
	double speed = 0;
	Scene scene = Scene::NormalScene; // 场景类型
	std::chrono::high_resolution_clock::time_point timestamp;
};

struct TaskData{
	cv::Mat img;
	std::chrono::high_resolution_clock::time_point timestamp;
	int steering_pwm = 0; // 4000-5000-6000, 0 means off
	double speed = 0; // m/s
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
	while (true) {
		lock.lock();
		if(!buffer.empty()) 
			break;
		lock.unlock();
		usleep(50);
	}
	product = buffer.front();
	buffer.pop_front();
	lock.unlock();

	return true;
}
// ------------------------------------ //
void signalHandler(int signal);

bool producer(Factory<TaskData> &task_data, Factory<TaskData> &AI_task_data, cv::VideoCapture &capture);
bool AIConsumer(Factory<TaskData> &task_data, std::vector<PredictResult> &predict_result, Motion &motion);
bool consumer(Factory<TaskData> &task_data, Factory<DebugData> &debug_data, std::vector<PredictResult> &predict_result, Motion &motion, Uart &uart);
// void drawBox(Mat &img, std::vector<PredictResult> results);
bool debugDataConsumer(Factory<DebugData> & debug_data);

// cv::Scalar getCvcolor(int index);
