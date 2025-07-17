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

class Layby
{
public:
    enum LaybyState
    {
        None,
        Enter,
        In,
        Stopping,
        Leave
    };
    enum LaybyDirection
    {
        Unknown,
        Left,
        Right
    };

    LaybyState state = LaybyState::None;
    LaybyDirection direction = LaybyDirection::Unknown;

    bool process(vector<PredictResult> &predict);
    int run(vector<PredictResult> &predict, UartStatus &status);
    float getTrackOffset() const { return track_offset; }


private:
    PredictResult target;
    bool detected = false;
    int detection_counter = 0; // 检测计数器
    int counter = 0;
    float start_odom = 0.0f;
    
    const float track_offset = 0.10f;
    const float stop_distance = 0.04f;
};