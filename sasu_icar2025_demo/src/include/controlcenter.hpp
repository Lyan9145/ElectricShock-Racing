#pragma once

#include "common.hpp"
#include "tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class ControlCenter
{
public:
    int controlCenter;           // 智能车控制中心（0~320）
    vector<POINT> centerEdge;    // 赛道中心点集
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差

    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void fitting(Tracking &track);
    bool derailmentCheck(Tracking track);
    void drawImage(Tracking track, Mat &centerImage);

private:
    int countOutlineA = 0; // 车辆脱轨检测计数器
    int countOutlineB = 0; // 车辆脱轨检测计数器
    string style = "";     // 赛道类型
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft);
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight);
    vector<POINT> centerCompute(vector<POINT> pointsEdge, int side);
};
