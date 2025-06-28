#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"
#include "tracking.hpp"

using namespace cv;
using namespace std;

class Catering
{
public:

    bool stopEnable = false;        // 停车使能标志
    bool noRing = false;            // 用来区分环岛路段

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict);
    void drawImage(Tracking track, Mat &image);


private:
    uint16_t counterSession = 0;    // 图像场次计数器
    uint16_t counterRec = 0;        // 汉堡标志检测计数器
    bool cateringEnable = false;    // 岔路区域使能标志
    bool burgerLeft = true;         // 汉堡在左侧
    bool turning = true;            // 转向标志
    int burgerY = 0;                // 汉堡高度
    int truningTime = 25;           // 转弯时间 25帧
    int travelTime = 10;            // 行驶时间 10帧 在斜线路段的行驶时间
    int stopTime = 25;              // 停车时间 25帧
};