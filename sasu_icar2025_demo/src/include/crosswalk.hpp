#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.HC.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file crosswalk.cpp
 * @author HC
 * @brief 停车区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"

using namespace std;
using namespace cv;

/**
 * @brief 停车区AI识别与路径规划类
 *
 */
class StopArea
{
private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        init = 0, // 初始化屏蔽
        det,      // AI标识检测
        enable,   // 场景使能
        stop      // 准备停车
    };
    Step step = Step::init; // 场景状态
    uint16_t countRec = 0;  // AI场景识别计数器
    uint16_t countSes = 0;  // 场次计数器

public:
    uint16_t countExit = 0; // 程序退出计数器
    bool park = false;      // 停车标志

    bool process(vector<PredictResult> predict);
    void drawImage(Mat &img);
};
