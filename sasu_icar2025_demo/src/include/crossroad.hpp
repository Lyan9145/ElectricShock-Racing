#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file crossroad.cpp
 * @author Leo
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：tracking.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp
#include "tracking.hpp

using namespace cv;
using namespace std;

class Crossroad
{
public:

    void reset(void);
    bool crossRecognition(Tracking &track);
    void drawImage(Tracking track, Mat &Image);


private:
    int _index = 0; // 测试

    POINT pointBreakLU;
    POINT pointBreakLD;
    POINT pointBreakRU;
    POINT pointBreakRD;
    uint16_t counterFild = 0;
    /**
     * @brief 十字道路类型
     *
     */
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     // 左斜入十字
        CrossroadRight,    // 右斜入十字
        CrossroadStraight, // 直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; // 十字道路类型


    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft);
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft);
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight);
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight);
    bool searchStraightCrossroad(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight);

};