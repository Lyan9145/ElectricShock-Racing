#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file parking.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 充电停车场
 * @version 0.1
 * @date 2025/03/04 20:29:04
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"
#include "tracking.hpp

using namespace cv;
using namespace std;

class Parking
{
public:
    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        enable,   // 停车场使能
        turning,  // 入库转向
        stop,     // 停车
        trackout  // 出库
    };

    ParkStep step = ParkStep::none; // 停车步骤

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict);
    void drawImage(Tracking track, Mat &image);

private:
    uint16_t counterSession = 0;         // 图像场次计数器
    uint16_t counterRec = 0;             // 加油站标志检测计数器
    bool garageFirst = true;             // 进入一号车库
    int lineY = 0;                       // 直线高度
    bool startTurning = false;           // 开始转弯
    vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
    vector<vector<POINT>> pathsEdgeRight;
    Point ptA = Point(0, 0); // 记录线段的两个端点
    Point ptB = Point(0, 0);
    int truningTime = 21;   // 转弯时间 21帧
    int stopTime = 40;      // 停车时间 40帧
    float swerveTime = 0.2; // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
};