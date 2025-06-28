#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */


#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "tracking.hpp"

using namespace cv;
using namespace std;

class Ring
{
public:
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void);

    bool process(Tracking &track, Mat &imagePath);

    void drawImage(Tracking track, Mat &ringImage);

private:
    uint16_t counterSpurroad = 0; // 岔路计数器
    // 临时测试用参数
    int _ringStep;
    int _ringEnable;
    int _tmp_ttttt;
    int _index = 0;
    int countExitRing = 0;
    POINT _ringPoint = POINT(0, 0);

    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束
    };

    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
};
