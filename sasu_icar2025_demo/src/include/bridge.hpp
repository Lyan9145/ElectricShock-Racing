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

class Bridge
{
public:
    bool process(Tracking &track, vector<PredictResult> predict);
    void drawImage(Tracking track, Mat &image);

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    bool bridgeEnable = false;   // 桥区域使能标志
};