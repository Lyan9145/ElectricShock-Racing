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
 * @file layby.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 临时停车区
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

#include "common.hpp
#include "detection.hpp"
#include "tracking.hpp"

using namespace cv;
using namespace std;

class Layby
{
public:
    bool stopEnable = false; // 停车使能标志

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict);
    void drawImage(Tracking track, Mat &image);
    void curtailTracking(Tracking &track, bool left);

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 标识牌检测计数器
    bool laybyEnable = false;    // 临时停车区域使能标志
    bool leftEnable = true;      // 标识牌在左侧
    bool searchingLine = false;  // 搜索直线标志
    vector<Vec4i> mergedLines;   // 合并后的线段用于绘制
    int moment = 110;            // 停车时机，屏幕上方的像素值，值越大越越晚停车
    int stopTime = 40;           // 停车时间 40帧
};