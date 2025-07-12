#pragma once
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"
// #include "tracking.hpp"

using namespace cv;
using namespace std;

class Bridge
{
public:
    enum State
    {
        None,
        Enter,
        Up,
        Down
    };
    State state = State::None; // 桥区域状态

    bool process(vector<PredictResult> predict);
    void run(vector<PredictResult> predict);


private:
    int accframes = 30;
    int counter = 0;
    bool bridgeEnable = false;   // 桥区域使能标志
};