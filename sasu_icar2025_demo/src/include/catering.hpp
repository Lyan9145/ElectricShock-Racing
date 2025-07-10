#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"
// #include "tracking.hpp"
#include "uart.hpp"

using namespace cv;
using namespace std;

class Catering
{
public:
    enum CateringState
    {
        None,       // 无快餐店
        Enter,      // 进入快餐店
        In,         // 在快餐店
        Stopping,   // 减速停车
        Leave       // 离开快餐店
    };
    enum CateringDirection
    {
        Unknown,
        Left,       // 左侧进入快餐店
        Right      // 右侧进入快餐店
    };

    CateringState state = CateringState::None;
    CateringDirection direction = CateringDirection::Unknown;

    bool process(vector<PredictResult> predict);
    int run(vector<PredictResult> predict, UartStatus &status);


private:
    bool detected = false; // 是否检测到快餐店
    int counter = 0;
    float start_odometer = 0.0f; // 起始里程
    const float stop_distance = 0.8f; // 快餐店距离
};