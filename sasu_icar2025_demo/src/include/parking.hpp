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
// #include "tracking.hpp"
#include "uart.hpp"

using namespace cv;
using namespace std;

class Parking
{
public:
    /**
     * @brief 停车步骤
     *
     */
    enum State
    {
        None,
        Enter,
        In
    };
    enum Position
    {
        Left,
        Right
    };

    State state = State::None;
    Position position = Position::Left; // 停车位置
    bool process(vector<PredictResult> &predict); // 处理停车场检测结果
    void run(vector<PredictResult> &predict, UartStatus &status); // 处理停车场检测结果


private:
    bool detected = false; // 是否检测到停车场标志
    int counter = 0; // 计数器
    int counter2 = 0; // 第二个计数器
    int startOdometer = 0; // 起始里程计
    const int stopDistance = 1.5f; // 停车距离

};